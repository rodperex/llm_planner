#!/usr/bin/env python3
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

import copy
import re
import textwrap

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from llm_planner.llm_planner_node import LLMPlannerNode


class LLMMultistepPlannerNode(LLMPlannerNode):
    """Planner variant that builds a plan incrementally with multiple LLM calls."""

    def __init__(self):
        super().__init__()
        self.get_logger().info(
            'llm_multistep_planner_node ready. Using skeleton + per-step expansion.'
        )

    def plan_task_callback(self, request, response):
        return self._multistep_callback(request, response, is_replan=False)

    def replan_task_callback(self, request, response):
        return self._multistep_callback(request, response, is_replan=True)

    def _multistep_callback(self, request, response, *, is_replan: bool):
        with self._call_lock:
            self._call_counter += 1
            call_id = self._call_counter

        goal = request.goal.strip()
        context = request.context.strip()
        skills = [s for s in request.skills if s.strip()]
        mission_name = request.mission_name.strip()
        mode_label = 'replan' if is_replan else 'plan'
        self.get_logger().info(f'[{mode_label} #{call_id}] goal="{goal}"')
        self.get_logger().info(f'[{mode_label} #{call_id}] Phase 1: building plan skeleton...')

        prompt_blocks = self._build_prompt_blocks(request, goal, context, skills, is_replan)

        skeleton = self._generate_skeleton(
            request=request,
            call_id=call_id,
            prompt_blocks=prompt_blocks,
            goal=goal,
            context=context,
            skills=skills,
            is_replan=is_replan,
        )
        if skeleton is None:
            response.success = False
            response.message = f'Failed to generate a valid {mode_label} skeleton.'
            return response

        self.get_logger().info(
            f'[{mode_label} #{call_id}] Phase 1 complete: skeleton has {len(skeleton["steps"])} steps.'
        )

        completed_steps = []
        expanded_steps = []
        for step_index, skeleton_step in enumerate(skeleton['steps']):
            self.get_logger().info(
                f'[{mode_label} #{call_id}] Phase 2: expanding step {step_index} '
                f'({skeleton_step.get("description", "no description")})...'
            )
            expanded_step = self._generate_step_object(
                request=request,
                call_id=call_id,
                step_index=step_index,
                prompt_blocks=prompt_blocks,
                skeleton=skeleton,
                completed_steps=completed_steps,
                skeleton_step=skeleton_step,
                is_replan=is_replan,
            )
            if expanded_step is None:
                response.success = False
                response.message = f'Failed to expand step {step_index} during {mode_label} generation.'
                return response

            self.get_logger().info(
                f'[{mode_label} #{call_id}] Step {step_index} raw expansion received; normalizing...'
            )

            merged_step = self._merge_step(skeleton_step, expanded_step)
            self.get_logger().info(
                f'[{mode_label} #{call_id}] Step {step_index} normalized with '
                f'{len(merged_step.get("objective", {}).get("steps", []))} objective substeps.'
            )
            if not self._validate_step(merged_step, expected_step_id=step_index, skills=skills):
                response.success = False
                response.message = f'Invalid expanded step {step_index} during {mode_label} generation.'
                return response

            expanded_steps.append(merged_step)
            completed_steps.append(copy.deepcopy(merged_step))
            self.get_logger().info(
                f'[{mode_label} #{call_id}] Step {step_index} accepted and appended '
                f'({len(expanded_steps)}/{len(skeleton["steps"])}).'
            )

        final_plan = copy.deepcopy(skeleton)
        final_plan['steps'] = expanded_steps

        self.get_logger().info(
            f'[{mode_label} #{call_id}] Phase 3: validating final plan with {len(expanded_steps)} steps...'
        )

        if not self._validate_plan(final_plan, skills=skills):
            response.success = False
            response.message = f'Final {mode_label} plan did not pass validation.'
            return response

        final_yaml = yaml.safe_dump(final_plan, sort_keys=False, allow_unicode=True)
        final_yaml = f'\n\n# Plan generated by {self.llm_model_id}.\n\n' + final_yaml

        response.success = True
        if is_replan:
            response.new_plan_yaml = final_yaml
            response.message = f'Replan generated with {len(expanded_steps)} steps (multistep).'
        else:
            response.plan_yaml = final_yaml
            response.message = f'Plan generated with {len(expanded_steps)} steps (multistep).'

        self.get_logger().info(
            f'[{mode_label} #{call_id}] {response.message}\n' +
            ''.join(f'  [{s["step_id"]}] {s.get("description", "?")}\n' for s in expanded_steps)
        )
        self.get_logger().info(f'[{mode_label} #{call_id}] Plan generation finished successfully.')

        save_kwargs = dict(
            prefix='multistep_replan' if is_replan else 'multistep_plan',
            goal=goal,
            skills=skills,
            mission_name=mission_name,
        )
        if is_replan:
            save_kwargs.update(
                failed_step=request.failed_step,
                failure_reason=request.failure_reason.strip(),
                previous_failures=list(request.previous_failures),
            )
        self._save_plan(final_yaml, **save_kwargs)
        return response

    def _build_prompt_blocks(self, request, goal, context, skills, is_replan):
        skills_block = ''
        if skills:
            skills_lines = '\n'.join(f'  - "{s}"' for s in skills)
            skills_block = f'\nskills:\n{skills_lines}'

        preconditions_block = ''
        if request.preconditions:
            pre_lines = '\n'.join(f'  - "{p}"' for p in request.preconditions)
            preconditions_block = f'\npreconditions:\n{pre_lines}'

        postconditions_block = ''
        if request.postconditions:
            post_lines = '\n'.join(f'  - "{p}"' for p in request.postconditions)
            postconditions_block = f'\npostconditions:\n{post_lines}'

        useful_info = request.useful_info.strip() if hasattr(request, 'useful_info') else ''
        useful_info_block = ''
        if useful_info:
            useful_info_block = f'\nuseful_info: |\n  {useful_info.replace(chr(10), chr(10) + "  ")}\n'

        if not is_replan:
            return (
                f'goal: "{goal}"\ncontext: "{context}"{skills_block}'
                f'{preconditions_block}{postconditions_block}{useful_info_block}\n\n'
                f'Generate the execution plan.'
            )

        original_plan = yaml.safe_load(request.plan_yaml or '{}') or {}
        original_context = original_plan.get('context', '').strip()
        steps = original_plan.get('steps', [])
        failed_step = request.failed_step
        failure_reason = request.failure_reason.strip()
        achieved = [s for s in steps if s.get('step_id', -1) < failed_step]
        achieved_text = '\n'.join(
            f"  - step {s['step_id']}: {s.get('description', '')}" for s in achieved
        ) or '  (none)'
        failed_desc = steps[failed_step].get('description', '?') if failed_step < len(steps) else '?'

        previous_text = ''
        if request.previous_failures:
            lines = '\n'.join(
                f'  attempt {i + 1}: {r}' for i, r in enumerate(request.previous_failures)
            )
            previous_text = (
                'PREVIOUS FAILED STRATEGIES FOR THIS STEP (treat each as a blocked capability):\n'
                f'{lines}\n\n'
            )

        blocked_text = ''
        if request.previous_failures:
            blocked_lines = '\n'.join(f'  - {r}' for r in request.previous_failures)
            blocked_text = (
                'ADDITIONAL BLOCKED CAPABILITIES (from previous attempts):\n'
                f'{blocked_lines}\n\n'
            )

        return (
            f'ORIGINAL GOAL: "{goal}"\n\n'
            f'ORIGINAL CONTEXT (robot role — MUST be preserved in every objective.description):\n'
            f'  {original_context}\n\n'
            f'{preconditions_block}{postconditions_block}'
            f'{skills_block}\n'
            f'{useful_info_block}'
            f'ALREADY COMPLETED STEPS (do not repeat):\n{achieved_text}\n\n'
            f'FAILED STEP {failed_step}: "{failed_desc}"\n'
            f'FAILURE REASON: {failure_reason}\n\n'
            f'{previous_text}'
            f'{blocked_text}'
            'CONSTRAINT: The failure reason above represents something the robot CANNOT do.\n'
            'Do NOT plan any step that relies on the same capability.\n\n'
            'Generate a new plan covering the REMAINING work using only alternative approaches.'
        )

    def _generate_skeleton(self, *, request, call_id, prompt_blocks, goal, context, skills, is_replan):
        system_prompt = self._load_prompt(self.get_parameter('plan_prompt_file').value)
        phase_prompt = (
            f'{prompt_blocks}\n\n'
            'PHASE 1: Generate only a plan skeleton.\n'
            'Return ONLY YAML.\n'
            'The output may be a YAML mapping with goal/context/steps or just a YAML list of steps.\n'
            'Each step must contain step_id, description, objective.name, and objective.description.\n'
            'Do NOT include objective.steps yet.\n'
            'Do NOT include inputs, outputs, recovery_policy, allow_forced_plan_fail, constraints, or style yet.\n'
            'Keep the plan short and sequential.'
        )
        raw = self._call_llm(system_prompt, phase_prompt, call_id=f'plan-skeleton #{call_id}')
        if raw is None:
            return None

        mode_label = 'replan' if is_replan else 'plan'
        self.get_logger().info(
            f'[{mode_label} #{call_id}] === RAW SKELETON ===\n{raw}\n=== END RAW ==='
        )
        extracted = self._extract_yaml(raw)
        self.get_logger().info(
            f'[{mode_label} #{call_id}] === SKELETON YAML ===\n{extracted}\n=== END SKELETON ==='
        )

        data = self._load_yaml_with_repair(extracted, mode_label=mode_label, call_id=call_id, stage='skeleton')
        if data is None:
            return None

        data = self._normalize_skeleton_data(data, goal=goal, context=context)
        self.get_logger().info(
            f'[{mode_label} #{call_id}] Skeleton normalized; validating structure...'
        )
        if not self._validate_skeleton(data, skills=skills):
            self.get_logger().error(f'[{mode_label} #{call_id}] skeleton validation failed.')
            return None
        return data

    def _load_yaml_with_repair(self, text, *, mode_label, call_id, stage):
        """Parse YAML and apply minimal repairs for common LLM formatting drift."""
        candidates = [text]

        dedented = textwrap.dedent(text or '').strip()
        if dedented and dedented != text:
            candidates.append(dedented)

        rebuilt = self._repair_steps_list_yaml(dedented or text)
        if rebuilt and rebuilt not in candidates:
            candidates.append(rebuilt)

        repaired_step = self._repair_step_item_yaml(dedented or text)
        if repaired_step and repaired_step not in candidates:
            candidates.append(repaired_step)

        last_error = None
        for index, candidate in enumerate(candidates):
            try:
                return yaml.safe_load(candidate)
            except Exception as exc:
                last_error = exc
                self.get_logger().warn(
                    f'[{mode_label} #{call_id}] {stage} parse attempt {index + 1} failed: {exc}'
                )

        self.get_logger().error(f'[{mode_label} #{call_id}] {stage} parse failed: {last_error}')
        return None

    def _repair_step_item_yaml(self, text):
        """Repair malformed single-step YAML (list or dict) with inconsistent indentation."""
        source = (text or '').strip()
        if not source:
            return source

        lines = source.splitlines()
        first_non_empty = next((line for line in lines if line.strip()), '')
        list_mode = re.match(r'^\s*-\s*step_id\s*:', first_non_empty) is not None
        dict_mode = re.match(r'^\s*step_id\s*:', first_non_empty) is not None
        step_like_mode = any(re.match(r'^\s*(skills|skills_required|description|objective|steps)\s*:', ln) for ln in lines)
        if not (list_mode or dict_mode or step_like_mode):
            return source

        top_level_keys = {
            'step_id', 'description', 'objective', 'skills_required', 'skills_used',
            'skills', 'preconditions', 'postconditions', 'recovery_actions', 'actions',
            'inputs', 'outputs', 'constraints', 'style', 'recovery_policy',
            'allow_forced_plan_fail',
        }
        objective_child_keys = {
            'name', 'description', 'steps', 'inputs', 'outputs', 'constraints',
            'style', 'recovery_policy', 'allow_forced_plan_fail',
        }
        recovery_child_keys = {'required', 'loop_until_success', 'retry_attempts'}

        rebuilt = []
        in_objective = False
        in_recovery = False

        for line in lines:
            stripped = line.strip()
            if not stripped:
                rebuilt.append('')
                continue

            if re.match(r'^-\s*step_id\s*:', stripped):
                rebuilt.append(f'- {stripped.split("-", 1)[1].strip()}')
                in_objective = False
                in_recovery = False
                list_mode = True
                continue

            if re.match(r'^step_id\s*:', stripped):
                rebuilt.append(f'{stripped}')
                in_objective = False
                in_recovery = False
                continue

            key_match = re.match(r'^([A-Za-z_][A-Za-z0-9_]*)\s*:', stripped)
            if key_match:
                key = key_match.group(1)

                if key in top_level_keys:
                    base = '  ' if list_mode else ''
                    rebuilt.append(f'{base}{stripped}')
                    in_objective = key == 'objective'
                    in_recovery = key == 'recovery_policy'
                    continue

                if in_objective and key in objective_child_keys:
                    base = '    ' if list_mode else '  '
                    rebuilt.append(f'{base}{stripped}')
                    continue

                if in_recovery and key in recovery_child_keys:
                    base = '    ' if list_mode else '  '
                    rebuilt.append(f'{base}{stripped}')
                    continue

            if stripped.startswith('- '):
                base = '    ' if list_mode else '  '
                rebuilt.append(f'{base}{stripped}')
            else:
                base = '  ' if list_mode else ''
                rebuilt.append(f'{base}{stripped}')

        return '\n'.join(rebuilt)

    def _repair_steps_list_yaml(self, text):
        """Recover malformed list-style skeletons by rebuilding list items from step_id anchors."""
        source = (text or '').strip()
        if not source:
            return source

        # If the text already has explicit list markers for steps, no repair needed.
        if re.search(r'(?m)^\s*-\s*step_id\s*:', source):
            return source

        lines = source.splitlines()

        def normalize_key_line(line):
            stripped = line.strip()
            prefix = line[:len(line) - len(line.lstrip(' '))]
            # Common malformed outputs: "step_id 0" or "description \"...\"".
            m = re.match(r'^(step_id)\s+(-?\d+)\s*$', stripped)
            if m:
                return f'{prefix}{m.group(1)}: {m.group(2)}'

            m = re.match(
                r'^(description|name|goal|context|type|target|reason|style|inputs|outputs|constraints|skills_used)\s+(.+)$',
                stripped,
            )
            if m and ':' not in stripped.split(maxsplit=1)[0]:
                return f'{prefix}{m.group(1)}: {m.group(2)}'

            if stripped in ('objective', 'steps', 'recovery_policy'):
                return f'{prefix}{stripped}:'

            return line

        normalized_lines = [normalize_key_line(line) for line in lines]

        if not any(re.search(r'^\s*step_id\s*:', line) for line in normalized_lines):
            return source

        rebuilt_lines = []
        for line in normalized_lines:
            stripped = line.strip()
            if re.match(r'^step_id\s*:', stripped):
                rebuilt_lines.append(f'- {stripped}')
            elif stripped:
                rebuilt_lines.append(f'  {stripped}')
            else:
                rebuilt_lines.append('')

        return '\n'.join(rebuilt_lines)

    def _generate_step_object(self, *, request, call_id, step_index, prompt_blocks, skeleton,
                              completed_steps, skeleton_step, is_replan):
        system_prompt = self._load_prompt(self.get_parameter('plan_prompt_file').value)
        prompt = (
            f'{prompt_blocks}\n\n'
            f'PHASE 2: Expand only step {step_index}.\n'
            'Return ONLY YAML for a single step object.\n'
            'Do not return the full plan.\n'
            'The output must contain step_id, description, objective, and if relevant skills_used.\n'
            'The objective block must include name, description, and steps.\n'
            'Add inputs, outputs, recovery_policy, allow_forced_plan_fail, constraints, and style when relevant.\n\n'
            'SKELETON:\n'
            f'{yaml.safe_dump(skeleton, sort_keys=False, allow_unicode=True)}\n\n'
            'ALREADY EXPANDED STEPS:\n'
            f'{yaml.safe_dump(completed_steps, sort_keys=False, allow_unicode=True)}\n\n'
            'CURRENT STEP SEED:\n'
            f'{yaml.safe_dump(skeleton_step, sort_keys=False, allow_unicode=True)}\n\n'
            'Only expand the current step. Preserve the step_id and description exactly.'
        )
        raw = self._call_llm(system_prompt, prompt, call_id=f'plan-step-{step_index} #{call_id}')
        if raw is None:
            return None

        mode_label = 'replan' if is_replan else 'plan'
        self.get_logger().info(
            f'[{mode_label} #{call_id}] === RAW STEP {step_index} ===\n{raw}\n=== END RAW ==='
        )
        extracted = self._extract_yaml(raw)
        self.get_logger().info(
            f'[{mode_label} #{call_id}] === STEP {step_index} YAML ===\n{extracted}\n=== END STEP ==='
        )

        data = self._load_yaml_with_repair(
            extracted,
            mode_label=mode_label,
            call_id=call_id,
            stage=f'step {step_index}',
        )
        if data is None:
            return None

        self.get_logger().info(
            f'[{mode_label} #{call_id}] Step {step_index}: coercing model output into canonical step format...'
        )
        step = self._coerce_step_object(data, expected_step_id=step_index)
        if step is None:
            self.get_logger().error(
                f'[{mode_label} #{call_id}] Step {step_index}: could not coerce model output.'
            )
            return None
        normalized = self._normalize_expanded_step(step, skeleton_step=skeleton_step, step_index=step_index)
        normalized = self._finalize_canonical_step(normalized, skeleton_step=skeleton_step)
        self.get_logger().info(
            f'[{mode_label} #{call_id}] Step {step_index}: normalization complete.'
        )
        return normalized

    def _extract_yaml(self, text):
        text = (text or '').strip()
        match = re.search(r'```(?:yaml|yml)?\s*(.*?)```', text, re.DOTALL | re.IGNORECASE)
        if match:
            extracted = match.group(1).strip()
            extracted = re.sub(r'^(?:yaml|yml)\s*', '', extracted, flags=re.IGNORECASE).strip()
            return extracted

        # Keep list markers when the first YAML anchor appears as "- step_id: ...".
        line_anchor = re.search(r'(?mi)^\s*(?:-\s*)?(goal:|step_id:)\s*', text)
        if line_anchor:
            return text[line_anchor.start():].strip()

        for anchor in ('goal:', 'step_id:'):
            idx = text.find(anchor)
            if idx >= 0:
                if anchor == 'step_id:' and idx >= 2 and text[idx - 2:idx] == '- ':
                    idx -= 2
                return text[idx:].strip()
        return text

    def _normalize_skeleton_data(self, data, *, goal, context):
        """Accept either a full plan mapping or a raw list of steps and normalize it."""
        if isinstance(data, dict):
            steps = data.get('steps')
            if isinstance(steps, list):
                normalized_steps = []
                for index, step in enumerate(steps):
                    normalized_step = self._coerce_skeleton_step(step, index)
                    if normalized_step is not None:
                        normalized_steps.append(normalized_step)
                normalized = copy.deepcopy(data)
                normalized['goal'] = normalized.get('goal', goal)
                normalized['context'] = normalized.get('context', context)
                normalized['steps'] = normalized_steps
                return normalized

            # Single-step dict fallback (when model omits the surrounding list/steps key).
            if any(key in data for key in ('step_id', 'description', 'objective', 'step')):
                single = self._coerce_skeleton_step(data, 0)
                if single is not None:
                    return {
                        'goal': data.get('goal', goal),
                        'context': data.get('context', context),
                        'steps': [single],
                    }
            return data

        if isinstance(data, list):
            normalized_steps = []
            for index, step in enumerate(data):
                normalized_step = self._coerce_skeleton_step(step, index)
                if normalized_step is not None:
                    normalized_steps.append(normalized_step)
            return {
                'goal': goal,
                'context': context,
                'steps': normalized_steps,
            }

        return data

    def _coerce_skeleton_step(self, step, step_index):
        """Coerce a loose skeleton step into the minimal valid planner shape."""
        if not isinstance(step, dict):
            return None

        description = str(
            step.get('description')
            or step.get('step')
            or step.get('name')
            or f'step {step_index}'
        ).strip()

        objective_raw = step.get('objective')
        if isinstance(objective_raw, dict):
            objective_name = str(
                objective_raw.get('name')
                or step.get('objective_name')
                or description
            ).strip()
            objective_description = str(
                objective_raw.get('description')
                or description
            ).strip()
        elif isinstance(objective_raw, str):
            objective_name = objective_raw.strip() or description
            objective_description = description
        else:
            objective_name = str(step.get('objective_name') or description).strip()
            objective_description = description

        normalized_step = {
            'step_id': step_index,
            'description': description,
            'objective': {
                'name': objective_name,
                'description': objective_description,
            },
        }

        if 'skills_used' in step and isinstance(step['skills_used'], list):
            normalized_step['skills_used'] = copy.deepcopy(step['skills_used'])

        return normalized_step

    def _coerce_step_object(self, data, expected_step_id):
        if isinstance(data, list):
            for item in data:
                if isinstance(item, dict) and item.get('step_id') == expected_step_id:
                    return item
            if len(data) == 1 and isinstance(data[0], dict):
                return data[0]

        if isinstance(data, dict) and 'steps' in data and isinstance(data['steps'], list):
            for item in data['steps']:
                if isinstance(item, dict) and item.get('step_id') == expected_step_id:
                    return item
            if len(data['steps']) == 1 and isinstance(data['steps'][0], dict):
                return data['steps'][0]

        if isinstance(data, dict) and data.get('step_id') == expected_step_id:
            return data

        if isinstance(data, dict) and any(k in data for k in ('description', 'objective', 'actions', 'skills')):
            coerced = copy.deepcopy(data)
            coerced['step_id'] = expected_step_id
            return coerced

        return None

    def _merge_step(self, skeleton_step, expanded_step):
        merged = copy.deepcopy(skeleton_step)
        for key, value in expanded_step.items():
            if key == 'objective' and isinstance(value, dict):
                objective = copy.deepcopy(merged.get('objective', {}))
                objective.update(value)
                merged['objective'] = objective
            else:
                merged[key] = value
        return merged

    def _normalize_expanded_step(self, step, *, skeleton_step, step_index):
        """Translate model-specific step layouts into the planner's canonical layout."""
        if not isinstance(step, dict):
            return None

        normalized = copy.deepcopy(step)
        normalized['step_id'] = step_index
        normalized.setdefault('description', skeleton_step.get('description', ''))

        objective = copy.deepcopy(normalized.get('objective', {}))
        if not isinstance(objective, dict):
            objective = {}

        if 'steps' not in objective or not isinstance(objective.get('steps'), list):
            translated_steps = self._translate_actions_to_objective_steps(normalized)
            if translated_steps:
                objective['steps'] = translated_steps
                normalized['actions'] = normalized.get('actions', [])

        normalized['objective'] = objective
        return normalized

    def _finalize_canonical_step(self, step, *, skeleton_step):
        """Fill missing canonical fields so downstream consumers always see a complete step."""
        if not isinstance(step, dict):
            return step

        finalized = copy.deepcopy(step)
        objective = copy.deepcopy(finalized.get('objective', {}))
        if not isinstance(objective, dict):
            objective = {}

        objective.setdefault('inputs', [])
        objective.setdefault('outputs', [])
        objective.setdefault('constraints', {})
        objective.setdefault('style', [])

        if 'recovery_policy' not in objective or not isinstance(objective.get('recovery_policy'), dict):
            objective['recovery_policy'] = self._infer_recovery_policy(finalized, skeleton_step)

        if 'allow_forced_plan_fail' not in objective:
            objective['allow_forced_plan_fail'] = self._infer_allow_forced_plan_fail(finalized)

        finalized['objective'] = objective
        return finalized

    def _infer_recovery_policy(self, step, skeleton_step):
        text = ' '.join(
            str(part).lower()
            for part in [
                step.get('description', ''),
                (step.get('objective') or {}).get('name', ''),
                (step.get('objective') or {}).get('description', ''),
                skeleton_step.get('description', ''),
                (skeleton_step.get('objective') or {}).get('name', ''),
                (skeleton_step.get('objective') or {}).get('description', ''),
            ]
        )

        if any(token in text for token in ('spin', 'search', 'until', 'keep trying', 'until success')):
            return {
                'required': True,
                'loop_until_success': True,
                'retry_attempts': 'forever',
            }

        if any(token in text for token in ('validate', 'validation', 'check appointment', 'appointment id')):
            return {
                'required': True,
                'loop_until_success': False,
                'retry_attempts': 3,
            }

        if any(token in text for token in ('follow', 'track', 'monitor')):
            return {
                'required': True,
                'loop_until_success': True,
                'retry_attempts': 'forever',
            }

        return {
            'required': False,
            'loop_until_success': False,
            'retry_attempts': 1,
        }

    def _infer_allow_forced_plan_fail(self, step):
        text = ' '.join(
            str(part).lower()
            for part in [
                step.get('description', ''),
                (step.get('objective') or {}).get('name', ''),
                (step.get('objective') or {}).get('description', ''),
            ]
        )
        return 'appointment' in text and 'validate' in text

    def _translate_actions_to_objective_steps(self, step):
        """Convert a loose actions list into objective.steps entries."""
        actions = step.get('actions')
        if not isinstance(actions, list) or not actions:
            return []

        translated = []
        for action in actions:
            if isinstance(action, dict):
                parts = []
                action_type = action.get('type')
                target = action.get('target')
                reason = action.get('reason')
                if action_type:
                    parts.append(str(action_type).replace('_', ' '))
                if target:
                    parts.append(str(target))
                if reason:
                    parts.append(f'because {reason}')
                text = ' '.join(parts).strip()
                if not text:
                    text = yaml.safe_dump(action, sort_keys=False, allow_unicode=True).strip()
                translated.append({'step': text})
            else:
                translated.append({'step': str(action)})

        self.get_logger().info(
            f'[multistep] Translated {len(translated)} action items into objective.steps entries.'
        )
        return translated

    def _validate_skeleton(self, plan, *, skills):
        if not isinstance(plan, dict):
            return False
        if not isinstance(plan.get('goal'), str) or not plan['goal'].strip():
            return False
        if not isinstance(plan.get('context'), str):
            return False
        steps = plan.get('steps')
        if not isinstance(steps, list) or not steps:
            return False
        for i, step in enumerate(steps):
            if not self._validate_step(step, expected_step_id=i, skills=skills, skeleton_mode=True):
                return False
        return True

    def _validate_step(self, step, *, expected_step_id, skills, skeleton_mode=False):
        if not isinstance(step, dict):
            return False
        if step.get('step_id') != expected_step_id:
            return False
        if not isinstance(step.get('description'), str) or not step['description'].strip():
            return False
        objective = step.get('objective')
        if not isinstance(objective, dict):
            return False
        if not isinstance(objective.get('name'), str) or not objective['name'].strip():
            return False
        if not isinstance(objective.get('description'), str) or not objective['description'].strip():
            return False
        if skeleton_mode:
            return True
        obj_steps = objective.get('steps')
        if not isinstance(obj_steps, list) or not obj_steps:
            return False
        for entry in obj_steps:
            if not isinstance(entry, dict):
                return False
            if 'step' not in entry and 'parallel' not in entry:
                return False
        if 'skills_used' in step:
            if not isinstance(step['skills_used'], list):
                return False
            skills_set = set(skills)
            if skills_set:
                for skill in step['skills_used']:
                    if skill not in skills_set:
                        return False
        return True

    def _validate_plan(self, plan, *, skills):
        if not isinstance(plan, dict):
            return False
        if not isinstance(plan.get('goal'), str) or not plan['goal'].strip():
            return False
        if 'steps' not in plan or not isinstance(plan['steps'], list) or not plan['steps']:
            return False
        for i, step in enumerate(plan['steps']):
            if not self._validate_step(step, expected_step_id=i, skills=skills, skeleton_mode=False):
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = LLMMultistepPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()