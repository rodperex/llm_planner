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

import datetime
import os
import re
import time

import requests
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from llm_planner_interfaces.srv import PlanTask, ReplanTask


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        self.declare_parameter('llm_provider', 'gemini')   # openai|gemini|anthropic|deepseek|ollama
        self.declare_parameter('llm_model_id', 'gemini-2.5-flash')
        self.declare_parameter('llm_api_url', '')
        self.declare_parameter('llm_api_key', '')
        self.declare_parameter('plan_prompt_file', 'plan_prompt.txt')
        self.declare_parameter('replan_prompt_file', 'replan_prompt.txt')
        self.declare_parameter('save_plan', False)

        self.llm_provider = self.get_parameter('llm_provider').value.lower()
        self.llm_model_id = self.get_parameter('llm_model_id').value
        self.save_plan    = self.get_parameter('save_plan').value
        self._setup_api_key()

        self._call_counter = 0
        self._call_lock = threading.Lock()

        self.srv = self.create_service(PlanTask, 'plan_task', self.plan_task_callback)
        self.replan_srv = self.create_service(ReplanTask, 'replan_task', self.replan_task_callback)

        api_url = self.get_parameter('llm_api_url').value or '(auto)'
        self.get_logger().info(
            f'llm_planner_node ready.\n'
            f'  provider : {self.llm_provider}\n'
            f'  model    : {self.llm_model_id}\n'
            f'  api_url  : {api_url}\n'
            f'  api_key  : {"(set)" if self.api_key and self.api_key != "sk-no-key-needed" else "(not set)"}'
        )

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def plan_task_callback(self, request, response):
        with self._call_lock:
            self._call_counter += 1
            call_id = self._call_counter
        goal = request.goal.strip()
        context = request.context.strip()
        skills = [s for s in request.skills if s.strip()]
        mission_name = request.mission_name.strip()
        self.get_logger().info(f'[plan #{call_id}] goal="{goal}"')

        skills_block = ''
        if skills:
            skills_lines = '\n'.join(f'  - "{s}"' for s in skills)
            skills_block = f'\nskills:\n{skills_lines}'
        user_prompt = (
            f'goal: "{goal}"\ncontext: "{context}"{skills_block}\n\n'
            f'Generate the execution plan.'
        )
        raw = self._call_llm(
            self._load_prompt(self.get_parameter('plan_prompt_file').value),
            user_prompt, call_id=f'plan #{call_id}')

        if raw is None:
            response.success = False
            response.message = 'LLM call failed.'
            return response

        plan_yaml = self._extract_yaml(raw)
        if not self._validate_plan(plan_yaml):
            response.success = False
            response.message = 'LLM returned invalid YAML plan.'
            response.plan_yaml = raw
            return response

        parsed = yaml.safe_load(plan_yaml)
        steps = parsed.get('steps', [])
        response.success = True
        response.plan_yaml = plan_yaml
        response.message = f'Plan generated with {len(steps)} steps.'
        self.get_logger().info(
            f'[plan #{call_id}] {response.message}\n' +
            ''.join(f'  [{s["step_id"]}] {s.get("description", "?")}\n' for s in steps)
        )
        self._save_plan(plan_yaml, prefix='normal_plan', goal=goal, skills=skills,
                        mission_name=mission_name)
        return response

    def replan_task_callback(self, request, response):
        with self._call_lock:
            self._call_counter += 1
            call_id = self._call_counter
        goal = request.goal.strip()
        failed_step = request.failed_step
        failure_reason = request.failure_reason.strip()
        previous_failures = list(request.previous_failures)
        skills = [s for s in request.skills if s.strip()]
        mission_name = request.mission_name.strip()
        self.get_logger().info(
            f'[replan #{call_id}] goal="{goal}" failed_step={failed_step} '
            f'reason="{failure_reason}" previous_attempts={len(previous_failures)}')

        original_plan = yaml.safe_load(request.plan_yaml or '{}') or {}
        original_context = original_plan.get('context', '').strip()
        steps = original_plan.get('steps', [])
        achieved = [s for s in steps if s.get('step_id', -1) < failed_step]
        achieved_text = '\n'.join(
            f"  - step {s['step_id']}: {s.get('description', '')}" for s in achieved
        ) or '  (none)'
        failed_desc = (steps[failed_step].get('description', '?')
                       if failed_step < len(steps) else '?')

        previous_text = ''
        blocked_text = ''
        if previous_failures:
            lines = '\n'.join(
                f'  attempt {i + 1}: {r}' for i, r in enumerate(previous_failures))
            previous_text = (
                f'PREVIOUS FAILED STRATEGIES FOR THIS STEP (treat each as a BLOCKED CAPABILITY — '
                f'do NOT use any of these approaches or any variation of them, '
                f'even across different steps):\n'
                f'{lines}\n\n'
            )
            blocked_lines = '\n'.join(f'  - {r}' for r in previous_failures)
            blocked_text = (
                f'ADDITIONAL BLOCKED CAPABILITIES (from previous attempts — '
                f'these must ALSO appear as constraints in the updated context field):\n'
                f'{blocked_lines}\n\n'
            )

        skills_section = ''
        if skills:
            skills_lines = '\n'.join(f'  - {s}' for s in skills)
            skills_section = (
                f'ROBOT SKILLS (the robot has ONLY these capabilities — '
                f'every step MUST use one of them):\n'
                f'{skills_lines}\n\n'
            )

        user_prompt = (
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
        raw = self._call_llm(
            self._load_prompt(self.get_parameter('replan_prompt_file').value),
            user_prompt, call_id=f'replan #{call_id}')

        if raw is None:
            response.success = False
            response.message = 'LLM call failed.'
            return response

        plan_yaml = self._extract_yaml(raw)
        if not self._validate_plan(plan_yaml):
            response.success = False
            response.message = 'LLM returned invalid YAML plan during replan.'
            response.new_plan_yaml = raw
            return response

        parsed = yaml.safe_load(plan_yaml)
        new_steps = parsed.get('steps', [])
        response.success = True
        response.new_plan_yaml = plan_yaml
        response.message = f'Replan generated with {len(new_steps)} steps.'
        self.get_logger().info(
            f'[replan #{call_id}] {response.message}\n' +
            ''.join(f'  [{s["step_id"]}] {s.get("description", "?")}\n' for s in new_steps)
        )
        self._save_plan(plan_yaml, prefix='normal_replan', goal=goal, skills=skills,
                        failed_step=failed_step, failure_reason=failure_reason,
                        previous_failures=previous_failures,
                        mission_name=mission_name)
        return response

    # ------------------------------------------------------------------
    # Persistence helpers
    # ------------------------------------------------------------------

    def _get_src_plans_path(self):
        """Resolve src/llm_planner/llm_planner/generated_plans/ from the install share path."""
        try:
            share_dir = get_package_share_directory('llm_planner')
            if 'install' in share_dir:
                base_path = share_dir.split('/install/')[0]
                return os.path.join(base_path, 'src', 'llm_planner', 'llm_planner', 'generated_plans')
        except Exception:
            pass
        return os.path.join(os.getcwd(), 'generated_plans')

    def _load_prompt(self, filename: str) -> str:
        """Load a prompt template from the package's prompts/ directory."""
        try:
            pkg_path = get_package_share_directory('llm_planner')
            prompt_path = os.path.join(pkg_path, 'prompts', filename)
            if not os.path.exists(prompt_path):
                base = pkg_path.split('/install/')[0]
                prompt_path = os.path.join(
                    base, 'src', 'llm_planner', 'llm_planner', 'prompts', filename)
            with open(prompt_path, 'r', encoding='utf-8') as f:
                content = f.read()
            self.get_logger().info(f'📄 Loaded prompt: {prompt_path}')
            return content
        except Exception as e:
            self.get_logger().error(f'❌ Could not load prompt "{filename}": {e}')
            return ''

    def _save_plan(self, plan_yaml: str, *, prefix: str, goal: str,
                   skills: list = None,
                   failed_step: int = None, failure_reason: str = None,
                   previous_failures: list = None,
                   mission_name: str = ''):
        if not self.save_plan:
            return
        plans_dir = self._get_src_plans_path()
        os.makedirs(plans_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_goal = re.sub(r'[^\w\-]+', '_', goal.strip())[:40]
        prefix_part = f'{mission_name}_{prefix}' if mission_name else prefix
        filename = f'{prefix_part}_{timestamp}_{self.llm_model_id}.yaml'
        out_path = os.path.join(plans_dir, filename)
        try:
            with open(out_path, 'w', encoding='utf-8') as f:
                f.write(f'# goal: {goal}\n')
                if skills:
                    for s in skills:
                        f.write(f'# skill: {s}\n')
                if failed_step is not None:
                    f.write(f'# failed_step: {failed_step}\n')
                if previous_failures:
                    for i, pf in enumerate(previous_failures):
                        f.write(f'# previous_failure[{i}]: {pf}\n')
                if failure_reason:
                    f.write(f'# failure_reason: {failure_reason}\n')
                f.write(plan_yaml)
            self.get_logger().info(f'💾 Plan saved at: {out_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Could not save plan: {e}')

    # ------------------------------------------------------------------
    # LLM helpers
    # ------------------------------------------------------------------

    def _setup_api_key(self):
        provider = self.llm_provider
        param_key = self.get_parameter('llm_api_key').value
        if param_key and param_key not in ('', 'sk-no-key-needed'):
            self.api_key = param_key
            return
        env_map = {
            'gemini': ['GEMINI_API_KEY', 'GOOGLE_API_KEY'],
            'openai': ['OPENAI_API_KEY'],
            'anthropic': ['ANTHROPIC_API_KEY'],
            'deepseek': ['DEEPSEEK_API_KEY'],
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
        api_url = self.get_parameter('llm_api_url').value
        defaults = {
            'gemini': ('https://generativelanguage.googleapis.com'
                       '/v1beta/openai/chat/completions'),
            'anthropic': 'https://api.anthropic.com/v1/messages',
            'deepseek': 'https://api.deepseek.com/v1/chat/completions',
            'ollama': 'http://localhost:11434/v1/chat/completions',
        }
        url = (api_url if api_url
               else defaults.get(provider, 'https://api.openai.com/v1/chat/completions'))
        return provider, model_id, url

    def _call_llm(self, system_prompt, user_prompt, max_retries=5, base_delay=5.0,
                  call_id='?'):
        provider, model_id, url = self._build_endpoint()
        messages = [
            {'role': 'system', 'content': system_prompt},
            {'role': 'user', 'content': user_prompt},
        ]
        for attempt in range(1, max_retries + 1):
            try:
                if provider == 'anthropic':
                    headers = {
                        'Content-Type': 'application/json',
                        'x-api-key': self.api_key,
                        'anthropic-version': '2023-06-01',
                    }
                    payload = {'model': model_id, 'messages': messages,
                               'temperature': 0.1, 'max_tokens': 2048}
                    resp = requests.post(url, headers=headers, json=payload, timeout=60)
                    resp.raise_for_status()
                    return resp.json()['content'][0]['text']
                else:
                    headers = {'Content-Type': 'application/json',
                               'Authorization': f'Bearer {self.api_key}'}
                    payload = {'model': model_id, 'messages': messages, 'temperature': 0.1}
                    resp = requests.post(url, headers=headers, json=payload, timeout=60)
                    resp.raise_for_status()
                    return resp.json()['choices'][0]['message']['content']
            except Exception as exc:
                retryable = isinstance(exc, requests.exceptions.HTTPError) and \
                    exc.response is not None and exc.response.status_code in (429, 500, 502, 503, 504)
                if retryable and attempt < max_retries:
                    delay = base_delay * (2 ** (attempt - 1))
                    self.get_logger().warn(
                        f'[{call_id}] LLM error (attempt {attempt}/{max_retries}): {exc} '
                        f'— retrying in {delay:.0f}s...')
                    deadline = time.monotonic() + delay
                    while time.monotonic() < deadline:
                        if not rclpy.ok():
                            return None
                        time.sleep(0.1)
                else:
                    self.get_logger().error(f'[{call_id}] LLM call failed: {exc}')
                    return None

    # ------------------------------------------------------------------
    # YAML utilities
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_yaml(text):
        text = text.strip()
        match = re.search(r'```(?:yaml)?\s*(.*?)```', text, re.DOTALL)
        return match.group(1).strip() if match else text

    @staticmethod
    def _validate_plan(plan_yaml):
        try:
            data = yaml.safe_load(plan_yaml)
            if not (isinstance(data, dict)
                    and 'steps' in data
                    and isinstance(data['steps'], list)
                    and len(data['steps']) > 0):
                return False
            # Each step must have either an 'objective' block or a legacy 'bt_prompt' string
            for step in data['steps']:
                if 'objective' not in step and 'bt_prompt' not in step:
                    return False
            return True
        except Exception:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
