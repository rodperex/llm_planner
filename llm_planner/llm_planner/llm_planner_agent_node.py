# Copyright 2026 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ─────────────────────────────────────────────────────────────────────────────
# llm_planner_agent_node.py
#
# Drop-in replacement for llm_planner_node that adds a deterministic
# validation loop after each plan/replan generation:
#
#   [Generate plan]
#       │
#       ▼
#   Phase 1 — Structural validation (programmatic, free)
#       │  ✗ → inject error feedback, retry
#       ▼
#   Phase 2 — Feasibility validation (LLM judge, 1 extra call)
#       │  ✗ → inject error feedback, retry
#       ▼
#   Phase 3 — Replan consistency (programmatic, free — replan only)
#       │  ✗ → inject error feedback, retry
#       ▼
#   [Accept plan → respond]
#
# Exposes the same services as llm_planner_node:
#   /plan_task   (llm_planner_interfaces/srv/PlanTask)
#   /replan_task (llm_planner_interfaces/srv/ReplanTask)
#
# Launch with: node_type:=agent in llm_planner.launch.py
# ─────────────────────────────────────────────────────────────────────────────

import datetime
import os
import re
import threading
import time

import requests
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from llm_planner_interfaces.srv import PlanTask, ReplanTask


# Maximum generation+validation attempts per request
MAX_RETRIES = 6


class LLMPlannerAgentNode(Node):

    def __init__(self):
        super().__init__('llm_planner_agent_node')

        # ── Parameters (identical to llm_planner_node) ────────────────────────
        self.declare_parameter('llm_provider',       'gemini')
        self.declare_parameter('llm_model_id',       'gemini-2.5-flash')
        self.declare_parameter('llm_api_url',        '')
        self.declare_parameter('llm_api_key',        '')
        self.declare_parameter('plan_prompt_file',   'plan_prompt.txt')
        self.declare_parameter('replan_prompt_file', 'replan_prompt.txt')
        self.declare_parameter('validate_prompt_file', 'validate_plan_prompt.txt')

        self.llm_provider = self.get_parameter('llm_provider').value.lower()
        self.llm_model_id = self.get_parameter('llm_model_id').value
        self._setup_api_key()

        self._plan_prompt     = self._load_prompt(self.get_parameter('plan_prompt_file').value)
        self._replan_prompt   = self._load_prompt(self.get_parameter('replan_prompt_file').value)
        self._validate_prompt = self._load_prompt(self.get_parameter('validate_prompt_file').value)

        self._call_counter = 0
        self._call_lock    = threading.Lock()

        # ── Services ──────────────────────────────────────────────────────────
        self.srv        = self.create_service(PlanTask,   'plan_task',   self.plan_task_callback)
        self.replan_srv = self.create_service(ReplanTask, 'replan_task', self.replan_task_callback)

        api_url = self.get_parameter('llm_api_url').value or '(auto)'
        self.get_logger().info(
            f'llm_planner_agent_node ready.\n'
            f'  provider : {self.llm_provider}\n'
            f'  model    : {self.llm_model_id}\n'
            f'  api_url  : {api_url}\n'
            f'  api_key  : {"(set)" if self.api_key and self.api_key != "sk-no-key-needed" else "(not set)"}'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Service callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def plan_task_callback(self, request, response):
        with self._call_lock:
            self._call_counter += 1
            call_id = self._call_counter

        goal    = request.goal.strip()
        context = request.context.strip()
        skills  = [s for s in request.skills if s.strip()]
        mission_name = request.mission_name.strip()
        self.get_logger().info(f'[plan #{call_id}] goal="{goal}"')

        skills_block = ''
        if skills:
            skills_lines = '\n'.join(f'  - "{s}"' for s in skills)
            skills_block = f'\nskills:\n{skills_lines}'

        base_user_prompt = (
            f'goal: "{goal}"\ncontext: "{context}"{skills_block}\n\n'
            f'Generate the execution plan.'
        )

        plan_yaml = self._generate_with_validation(
            system_prompt=self._plan_prompt,
            base_user_prompt=base_user_prompt,
            goal=goal,
            skills=skills,
            call_id=f'plan #{call_id}',
        )

        if plan_yaml is None:
            response.success = False
            response.message = f'Failed to generate a valid plan after {MAX_RETRIES} attempts.'
            return response

        parsed = yaml.safe_load(plan_yaml)
        steps  = parsed.get('steps', [])
        response.success  = True
        response.plan_yaml = plan_yaml
        response.message  = f'Plan generated with {len(steps)} steps (agent-validated).'
        self.get_logger().info(
            f'[plan #{call_id}] {response.message}\n' +
            ''.join(f'  [{s["step_id"]}] {s.get("description", "?")}\n' for s in steps)
        )
        self._save_plan(plan_yaml, prefix=self._plan_prefix(), goal=goal, skills=skills,
                        mission_name=mission_name)
        return response

    def replan_task_callback(self, request, response):
        with self._call_lock:
            self._call_counter += 1
            call_id = self._call_counter

        goal              = request.goal.strip()
        failed_step       = request.failed_step
        failure_reason    = request.failure_reason.strip()
        previous_failures = list(request.previous_failures)
        skills            = [s for s in request.skills if s.strip()]
        mission_name      = request.mission_name.strip()
        self.get_logger().info(
            f'[replan #{call_id}] goal="{goal}" failed_step={failed_step} '
            f'reason="{failure_reason}" previous_attempts={len(previous_failures)}'
        )

        original_plan    = yaml.safe_load(request.plan_yaml or '{}') or {}
        original_context = original_plan.get('context', '').strip()
        steps            = original_plan.get('steps', [])
        achieved = [s for s in steps if s.get('step_id', -1) < failed_step]
        achieved_text = '\n'.join(
            f"  - step {s['step_id']}: {s.get('description', '')}" for s in achieved
        ) or '  (none)'
        failed_desc = (
            steps[failed_step].get('description', '?') if failed_step < len(steps) else '?'
        )

        previous_text = ''
        blocked_text  = ''
        if previous_failures:
            lines = '\n'.join(
                f'  attempt {i + 1}: {r}' for i, r in enumerate(previous_failures)
            )
            previous_text = (
                f'PREVIOUS FAILED STRATEGIES FOR THIS STEP (treat each as a BLOCKED CAPABILITY — '
                f'do NOT use any of these approaches or any variation of them, '
                f'even across different steps):\n{lines}\n\n'
            )
            blocked_lines = '\n'.join(f'  - {r}' for r in previous_failures)
            blocked_text  = (
                f'ADDITIONAL BLOCKED CAPABILITIES (from previous attempts — '
                f'these must ALSO appear as constraints in the updated context field):\n'
                f'{blocked_lines}\n\n'
            )

        skills_section = ''
        if skills:
            skills_lines   = '\n'.join(f'  - {s}' for s in skills)
            skills_section = (
                f'ROBOT SKILLS (the robot has ONLY these capabilities — '
                f'every step MUST use one of them):\n{skills_lines}\n\n'
            )

        base_user_prompt = (
            f'ORIGINAL GOAL: "{goal}"\n\n'
            f'ORIGINAL CONTEXT (robot role — MUST be preserved in every objective.description):\n'
            f'  {original_context}\n\n'
            f'{skills_section}'
            f'ALREADY COMPLETED STEPS (do not repeat):\n{achieved_text}\n\n'
            f'FAILED STEP {failed_step}: "{failed_desc}"\n'
            f'FAILURE REASON: {failure_reason}\n\n'
            f'{previous_text}'
            f'{blocked_text}'
            f'CONSTRAINT: The failure reason above represents something the robot CANNOT do.\n'
            f'Do NOT plan any step that relies on the same capability.\n\n'
            f'Generate a new plan covering the REMAINING work using only alternative approaches.'
        )

        plan_yaml = self._generate_with_validation(
            system_prompt=self._replan_prompt,
            base_user_prompt=base_user_prompt,
            goal=goal,
            skills=skills,
            call_id=f'replan #{call_id}',
            is_replan=True,
            original_plan_yaml=request.plan_yaml,
            failed_step=failed_step,
            previous_failures=previous_failures,
        )

        if plan_yaml is None:
            response.success = False
            response.message = f'Failed to generate a valid replan after {MAX_RETRIES} attempts.'
            return response

        parsed    = yaml.safe_load(plan_yaml)
        new_steps = parsed.get('steps', [])
        response.success      = True
        response.new_plan_yaml = plan_yaml
        response.message      = f'Replan generated with {len(new_steps)} steps (agent-validated).'
        self.get_logger().info(
            f'[replan #{call_id}] {response.message}\n' +
            ''.join(f'  [{s["step_id"]}] {s.get("description", "?")}\n' for s in new_steps)
        )
        self._save_plan(
            plan_yaml, prefix=self._replan_prefix(), goal=goal, skills=skills,
            failed_step=failed_step, failure_reason=failure_reason,
            previous_failures=previous_failures,
            mission_name=mission_name,
        )
        return response

    # ─────────────────────────────────────────────────────────────────────────
    # Filename prefix hooks (overridable by subclasses)
    # ─────────────────────────────────────────────────────────────────────────

    def _plan_prefix(self):
        return 'agent_plan'

    def _replan_prefix(self):
        return 'agent_replan'

    # ─────────────────────────────────────────────────────────────────────────
    # Core validation loop
    # ─────────────────────────────────────────────────────────────────────────

    def _generate_with_validation(
        self,
        system_prompt,
        base_user_prompt,
        goal,
        skills,
        call_id,
        is_replan=False,
        original_plan_yaml=None,
        failed_step=None,
        previous_failures=None,
    ):
        """Generate a plan and validate it through 3 phases, retrying on failure."""
        errors_so_far = []

        for attempt in range(1, MAX_RETRIES + 1):
            self.get_logger().info(f'[{call_id}] Generation attempt {attempt}/{MAX_RETRIES}')

            # ── Build user prompt (append accumulated errors) ─────────────────
            user_prompt = base_user_prompt
            if errors_so_far:
                error_block = '\n'.join(f'  - {e}' for e in errors_so_far)
                user_prompt += (
                    f'\n\n## ERRORS FROM PREVIOUS ATTEMPT — fix ALL of these:\n'
                    f'{error_block}\n\n'
                    f'Regenerate a corrected plan that resolves every error above.'
                )

            # ── LLM call ─────────────────────────────────────────────────────
            raw = self._call_llm(system_prompt, user_prompt, call_id=call_id)
            if raw is None:
                self.get_logger().warn(f'[{call_id}] LLM call failed, retrying...')
                time.sleep(3.0)
                continue

            plan_yaml = self._extract_yaml(raw)

            # ── Phase 1: Structural validation (programmatic) ─────────────────
            ok, msg = self._validate_structure(plan_yaml, skills)
            if not ok:
                self.get_logger().warn(f'[{call_id}] Phase 1 FAIL: {msg}')
                errors_so_far = [f'STRUCTURE ERROR: {msg}']
                continue

            self.get_logger().info(f'[{call_id}] Phase 1 OK (structure)')

            # ── Phase 2: Feasibility validation (LLM judge) ───────────────────
            ok, msg = self._validate_feasibility(plan_yaml, goal, skills, call_id)
            if not ok:
                self.get_logger().warn(f'[{call_id}] Phase 2 FAIL: {msg}')
                errors_so_far = [f'FEASIBILITY ERROR: {msg}']
                continue

            self.get_logger().info(f'[{call_id}] Phase 2 OK (feasibility)')

            # ── Phase 3: Replan consistency (programmatic, replan only) ───────
            if is_replan:
                ok, msg = self._validate_replan_consistency(
                    plan_yaml, original_plan_yaml, failed_step, previous_failures or []
                )
                if not ok:
                    self.get_logger().warn(f'[{call_id}] Phase 3 FAIL: {msg}')
                    errors_so_far = [f'CONSISTENCY ERROR: {msg}']
                    continue

                self.get_logger().info(f'[{call_id}] Phase 3 OK (replan consistency)')

            # ── All phases passed ─────────────────────────────────────────────
            self.get_logger().info(f'[{call_id}] ✅ Plan accepted after {attempt} attempt(s).')
            return plan_yaml

        self.get_logger().error(f'[{call_id}] ❌ All {MAX_RETRIES} attempts exhausted.')
        return None

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 1 — Structural validation (programmatic)
    # ─────────────────────────────────────────────────────────────────────────

    def _validate_structure(self, plan_yaml, skills):
        """
        Checks:
        - YAML is parseable
        - Has a non-empty 'steps' list
        - Each step has: step_id (int), description (str), objective (dict)
        - objective has: name, description, steps
        - step_id values are 0-based and sequential without gaps
        - Each skills_used entry exists in the robot skills list
        """
        try:
            data = yaml.safe_load(plan_yaml)
        except yaml.YAMLError as e:
            return False, f'YAML parse error: {e}'

        if not isinstance(data, dict):
            return False, 'Plan must be a YAML mapping (dict), not a list or scalar.'

        steps = data.get('steps')
        if not isinstance(steps, list) or len(steps) == 0:
            return False, '"steps" must be a non-empty list.'

        skills_set = set(skills)

        for i, step in enumerate(steps):
            sid = step.get('step_id')
            if sid is None:
                return False, f'Step at index {i} is missing "step_id".'
            if sid != i:
                return False, (
                    f'step_id values must be sequential starting at 0. '
                    f'Expected {i}, got {sid}.'
                )

            if not isinstance(step.get('description', None), str):
                return False, f'Step {sid} is missing a "description" string.'

            obj = step.get('objective')
            if not isinstance(obj, dict):
                return False, f'Step {sid} is missing an "objective" block (must be a dict).'
            for field in ('name', 'description', 'steps'):
                if field not in obj:
                    return False, f'Step {sid} objective is missing required field "{field}".'
            if not isinstance(obj['steps'], list) or len(obj['steps']) == 0:
                return False, f'Step {sid} objective.steps must be a non-empty list.'

            # skills_used check (only when a skills list was provided)
            if skills_set:
                skills_used = step.get('skills_used', []) or []
                for skill in skills_used:
                    if skill not in skills_set:
                        return False, (
                            f'Step {sid} uses skill "{skill}" which is NOT in the '
                            f'robot capabilities list.'
                        )

        # Cross-step I/O consistency: every input key must be produced by a prior step
        available_outputs = set()
        for i, step in enumerate(steps):
            sid = step.get('step_id', i)
            obj = step.get('objective', {}) or {}
            inputs = obj.get('inputs', []) or []
            outputs = obj.get('outputs', []) or []

            if not isinstance(inputs, list):
                return False, f'Step {sid} objective.inputs must be a list of strings.'
            if not isinstance(outputs, list):
                return False, f'Step {sid} objective.outputs must be a list of strings.'

            for key in inputs:
                if not isinstance(key, str) or not key.strip():
                    return False, f'Step {sid} objective.inputs contains a non-string entry.'
                if key not in available_outputs:
                    return False, (
                        f'Step {sid} declares input "{key}" but no previous step '
                        f'declares it as an output. Add an earlier step that writes "{key}".'
                    )

            for key in outputs:
                if not isinstance(key, str) or not key.strip():
                    return False, f'Step {sid} objective.outputs contains a non-string entry.'
                available_outputs.add(key)

        return True, 'OK'

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 2 — Feasibility validation (LLM judge)
    # ─────────────────────────────────────────────────────────────────────────

    def _validate_feasibility(self, plan_yaml, goal, skills, call_id):
        """
        Asks a second LLM call to judge the plan for:
        - Goal coverage (does the plan fully achieve the goal?)
        - Perception-before-action (are entities detected before being acted on?)
        - Dependency order (does each step use only information established so far?)
        - Skill compliance (no implicit use of capabilities not in the list)
        Returns (True, 'OK') or (False, 'reason').
        """
        if not self._validate_prompt:
            self.get_logger().warn(
                f'[{call_id}] validate_plan_prompt.txt not found — skipping Phase 2.'
            )
            return True, 'OK (prompt missing, skipped)'

        skills_block = ''
        if skills:
            skills_block = 'ROBOT SKILLS:\n' + '\n'.join(f'  - {s}' for s in skills) + '\n\n'

        user_content = (
            f'GOAL: "{goal}"\n\n'
            f'{skills_block}'
            f'PLAN TO VALIDATE:\n{plan_yaml}'
        )

        raw = self._call_llm(
            self._validate_prompt, user_content,
            max_retries=2, base_delay=2.0,
            call_id=f'{call_id}/judge',
        )

        if raw is None:
            self.get_logger().warn(
                f'[{call_id}] Feasibility judge call failed — treating as VALID to avoid deadlock.'
            )
            return True, 'OK (judge unavailable, skipped)'

        verdict = raw.strip()
        self.get_logger().info(f'[{call_id}] Judge verdict: {verdict[:120]}')

        if verdict.startswith('VALID'):
            return True, 'OK'
        elif verdict.startswith('ERROR:'):
            return False, verdict[len('ERROR:'):].strip()
        else:
            # Unexpected format — log and continue (avoid blocking on bad judge output)
            self.get_logger().warn(
                f'[{call_id}] Judge returned unexpected format: "{verdict[:80]}". '
                f'Treating as VALID.'
            )
            return True, 'OK (unexpected judge format, treated as valid)'

    # ─────────────────────────────────────────────────────────────────────────
    # Phase 3 — Replan consistency (programmatic)
    # ─────────────────────────────────────────────────────────────────────────

    def _validate_replan_consistency(
        self, new_plan_yaml, original_plan_yaml, failed_step, previous_failures
    ):
        """
        Checks (replan-specific):
        - No new step has step_id < failed_step (would repeat completed work)
        - No step description/objective mentions a strategy from previous_failures
        """
        try:
            new_plan = yaml.safe_load(new_plan_yaml)
        except yaml.YAMLError as e:
            return False, f'YAML parse error in replan: {e}'

        new_steps = new_plan.get('steps', [])

        # Check: no step_id overlaps with already-completed steps
        for step in new_steps:
            sid = step.get('step_id', -1)
            if isinstance(sid, int) and sid < failed_step:
                return False, (
                    f'Replan step_id {sid} overlaps with already-completed steps '
                    f'(steps 0..{failed_step - 1} were done). Do not repeat them.'
                )

        # Check: no step echoes a previously blocked strategy (keyword heuristic)
        if previous_failures:
            for step in new_steps:
                sid  = step.get('step_id', '?')
                text = (
                    str(step.get('description', '')) + ' ' +
                    str(step.get('objective', {}).get('description', '')) + ' ' +
                    ' '.join(str(s) for s in step.get('objective', {}).get('steps', []))
                ).lower()
                for pf in previous_failures:
                    # Extract key words from the failure reason (longer tokens)
                    keywords = [w.lower() for w in re.split(r'\W+', pf) if len(w) > 4]
                    matches  = [kw for kw in keywords if kw in text]
                    if len(matches) >= 3:
                        return False, (
                            f'Step {sid} appears to repeat a previously blocked strategy: '
                            f'"{pf}". Use a genuinely different approach.'
                        )

        return True, 'OK'

    # ─────────────────────────────────────────────────────────────────────────
    # LLM & utility helpers (same as llm_planner_node)
    # ─────────────────────────────────────────────────────────────────────────

    def _setup_api_key(self):
        provider  = self.llm_provider
        param_key = self.get_parameter('llm_api_key').value
        if param_key and param_key not in ('', 'sk-no-key-needed'):
            self.api_key = param_key
            return
        env_map = {
            'gemini':    ['GEMINI_API_KEY', 'GOOGLE_API_KEY'],
            'openai':    ['OPENAI_API_KEY'],
            'anthropic': ['ANTHROPIC_API_KEY'],
            'deepseek':  ['DEEPSEEK_API_KEY'],
        }
        for env in env_map.get(provider, ['LLM_API_KEY']):
            key = os.getenv(env, '')
            if key:
                self.api_key = key
                return
        self.api_key = 'sk-no-key-needed'

    def _build_endpoint(self):
        provider = self.llm_provider
        model_id = self.llm_model_id
        api_url  = self.get_parameter('llm_api_url').value
        defaults = {
            'gemini':    ('https://generativelanguage.googleapis.com'
                         '/v1beta/openai/chat/completions'),
            'anthropic': 'https://api.anthropic.com/v1/messages',
            'deepseek':  'https://api.deepseek.com/v1/chat/completions',
            'ollama':    'http://localhost:11434/v1/chat/completions',
        }
        url = (api_url if api_url
               else defaults.get(provider, 'https://api.openai.com/v1/chat/completions'))
        return provider, model_id, url

    def _call_llm(self, system_prompt, user_prompt, max_retries=5, base_delay=5.0,
                  call_id='?'):
        provider, model_id, url = self._build_endpoint()
        messages = [
            {'role': 'system', 'content': system_prompt},
            {'role': 'user',   'content': user_prompt},
        ]
        for attempt in range(1, max_retries + 1):
            try:
                if provider == 'anthropic':
                    headers = {
                        'Content-Type':      'application/json',
                        'x-api-key':         self.api_key,
                        'anthropic-version': '2023-06-01',
                    }
                    payload = {'model': model_id, 'messages': messages,
                               'temperature': 0.1, 'max_tokens': 2048}
                    resp = requests.post(url, headers=headers, json=payload, timeout=60)
                    resp.raise_for_status()
                    return resp.json()['content'][0]['text']
                else:
                    headers = {
                        'Content-Type':  'application/json',
                        'Authorization': f'Bearer {self.api_key}',
                    }
                    payload = {'model': model_id, 'messages': messages, 'temperature': 0.1}
                    resp    = requests.post(url, headers=headers, json=payload, timeout=60)
                    resp.raise_for_status()
                    return resp.json()['choices'][0]['message']['content']
            except Exception as exc:
                retryable = (
                    isinstance(exc, requests.exceptions.HTTPError) and
                    exc.response is not None and
                    exc.response.status_code in (429, 500, 502, 503, 504)
                )
                if retryable and attempt < max_retries:
                    delay = base_delay * (2 ** (attempt - 1))
                    self.get_logger().warn(
                        f'[{call_id}] LLM error (attempt {attempt}/{max_retries}): {exc} '
                        f'— retrying in {delay:.0f}s...'
                    )
                    deadline = time.monotonic() + delay
                    while time.monotonic() < deadline:
                        if not rclpy.ok():
                            return None
                        time.sleep(0.1)
                else:
                    self.get_logger().error(f'[{call_id}] LLM call failed: {exc}')
                    return None

    @staticmethod
    def _extract_yaml(text):
        text  = text.strip()
        match = re.search(r'```(?:yaml)?\s*(.*?)```', text, re.DOTALL)
        return match.group(1).strip() if match else text

    def _load_prompt(self, filename):
        try:
            pkg_path    = get_package_share_directory('llm_planner')
            prompt_path = os.path.join(pkg_path, 'prompts', filename)
            if not os.path.exists(prompt_path):
                base        = pkg_path.split('/install/')[0]
                prompt_path = os.path.join(
                    base, 'src', 'llm_planner', 'llm_planner', 'prompts', filename
                )
            if os.path.exists(prompt_path):
                with open(prompt_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                self.get_logger().info(f'📄 Loaded prompt: {prompt_path}')
                return content
            self.get_logger().warn(f'⚠️  Prompt not found: {filename}')
            return ''
        except Exception as e:
            self.get_logger().error(f'❌ Could not load prompt "{filename}": {e}')
            return ''

    def _get_src_plans_path(self):
        try:
            share_dir = get_package_share_directory('llm_planner')
            if 'install' in share_dir:
                base_path = share_dir.split('/install/')[0]
                return os.path.join(
                    base_path, 'src', 'llm_planner', 'llm_planner', 'generated_plans'
                )
        except Exception:
            pass
        return os.path.join(os.getcwd(), 'generated_plans')

    def _save_plan(self, plan_yaml, *, prefix, goal, skills=None,
                   failed_step=None, failure_reason=None, previous_failures=None,
                   mission_name=''):
        plans_dir = self._get_src_plans_path()
        os.makedirs(plans_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        prefix_part = f'{mission_name}_{prefix}' if mission_name else prefix
        filename  = f'{prefix_part}_{timestamp}_{self.llm_model_id}.yaml'
        out_path  = os.path.join(plans_dir, filename)
        try:
            with open(out_path, 'w', encoding='utf-8') as f:
                f.write(f'# goal: {goal}\n')
                for s in (skills or []):
                    f.write(f'# skill: {s}\n')
                if failed_step is not None:
                    f.write(f'# failed_step: {failed_step}\n')
                for i, pf in enumerate(previous_failures or []):
                    f.write(f'# previous_failure[{i}]: {pf}\n')
                if failure_reason:
                    f.write(f'# failure_reason: {failure_reason}\n')
                f.write(plan_yaml)
            self.get_logger().info(f'💾 Plan saved at: {out_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Could not save plan: {e}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerAgentNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
