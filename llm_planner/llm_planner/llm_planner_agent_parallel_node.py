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
# llm_planner_agent_parallel_node.py
#
# Extension of LLMPlannerAgentNode that allows objective.steps to contain
# parallel groups alongside sequential steps:
#
#   objective:
#     steps:
#       - step: "<sequential sub-action>"
#       - parallel:
#           - step: "<concurrent sub-action A>"
#           - step: "<concurrent sub-action B>"
#       - step: "<sequential sub-action after the parallel group>"
#
# The Behavior Tree generator (llm_bt_builder) receives the objective YAML and
# is expected to map parallel groups to a BT <Parallel> control node.
#
# Only differences from LLMPlannerAgentNode:
#   1. Default plan_prompt_file → plan_parallel_prompt.txt
#   2. _validate_structure accepts 'parallel:' entries in objective.steps
#   3. Node name is 'llm_planner_agent_parallel_node'
# ─────────────────────────────────────────────────────────────────────────────

import rclpy
from rclpy.executors import MultiThreadedExecutor

from llm_planner.llm_planner_agent_node import LLMPlannerAgentNode


class LLMPlannerAgentParallelNode(LLMPlannerAgentNode):

    def __init__(self):
        # Override the default prompt file before calling super().__init__()
        # by monkey-patching via a sentinel; instead we call Node.__init__
        # indirectly through super() and then re-declare the parameter.
        # The cleanest way: declare parameters in __init__ before super reads them.
        # We achieve this by calling rclpy Node init directly via the MRO,
        # but LLMPlannerAgentNode reads parameters in its __init__, so we pass
        # the overridden default via a parameter override file instead.
        #
        # Simplest compatible approach: call super().__init__() which declares
        # 'plan_prompt_file' with default 'plan_prompt.txt', then re-load the
        # parallel prompt right after and replace self._plan_prompt.
        super().__init__()

        # Override node name (ROS 2 node name is fixed at construction; we shadow
        # the logger tag by using a subclass-level attribute — the actual ROS name
        # should be set via the launch file or remapping).
        self.get_logger().info(
            'llm_planner_agent_parallel_node ready '
            '(parallel sub-actions enabled in objective.steps).'
        )

        # Re-load the parallel-aware plan prompt, overriding what super() loaded.
        self._plan_prompt = self._load_prompt('plan_parallel_prompt.txt')
        if not self._plan_prompt:
            self.get_logger().warn(
                'plan_parallel_prompt.txt not found — falling back to plan_prompt.txt'
            )
            self._plan_prompt = self._load_prompt('plan_prompt.txt')

    # ─────────────────────────────────────────────────────────────────────────
    # Override: structural validation accepting parallel groups
    # ─────────────────────────────────────────────────────────────────────────

    def _validate_structure(self, plan_yaml, skills):
        """
        Like the parent validator, but objective.steps entries may be either:
          - {step: str}                         — sequential sub-action
          - {parallel: [{step: str}, ...]}      — parallel group (≥2 members)
        """
        import yaml

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

            obj_steps = obj['steps']
            if not isinstance(obj_steps, list) or len(obj_steps) == 0:
                return False, f'Step {sid} objective.steps must be a non-empty list.'

            ok, msg = self._validate_obj_steps(sid, obj_steps)
            if not ok:
                return False, msg

            # skills_used check (only when a skills list was provided)
            if skills_set:
                skills_used = step.get('skills_used', []) or []
                for skill in skills_used:
                    if skill not in skills_set:
                        return False, (
                            f'Step {sid} uses skill "{skill}" which is NOT in the '
                            f'robot capabilities list.'
                        )

        return True, 'OK'

    def _validate_obj_steps(self, sid, obj_steps):
        """Validate each entry in objective.steps: either {step:} or {parallel:[...]}."""
        for j, entry in enumerate(obj_steps):
            if not isinstance(entry, dict):
                return False, (
                    f'Step {sid} objective.steps[{j}] must be a dict '
                    f'(got {type(entry).__name__}).'
                )

            keys = set(entry.keys())

            if 'step' in keys:
                if not isinstance(entry['step'], str) or not entry['step'].strip():
                    return False, (
                        f'Step {sid} objective.steps[{j}].step must be a non-empty string.'
                    )

            elif 'parallel' in keys:
                group = entry['parallel']
                if not isinstance(group, list) or len(group) < 2:
                    return False, (
                        f'Step {sid} objective.steps[{j}].parallel must be a list '
                        f'with at least 2 entries.'
                    )
                for k, sub in enumerate(group):
                    if not isinstance(sub, dict) or 'step' not in sub:
                        return False, (
                            f'Step {sid} objective.steps[{j}].parallel[{k}] '
                            f'must be a dict with a "step" key.'
                        )
                    if not isinstance(sub['step'], str) or not sub['step'].strip():
                        return False, (
                            f'Step {sid} objective.steps[{j}].parallel[{k}].step '
                            f'must be a non-empty string.'
                        )

            else:
                return False, (
                    f'Step {sid} objective.steps[{j}] must have either a "step" key '
                    f'or a "parallel" key (got keys: {sorted(keys)}).'
                )

        return True, 'OK'

    # ─────────────────────────────────────────────────────────────────────────
    # Filename prefix hooks
    # ─────────────────────────────────────────────────────────────────────────

    def _plan_prefix(self):
        return 'agent_parallel_plan'

    def _replan_prefix(self):
        return 'agent_parallel_replan'


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerAgentParallelNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
