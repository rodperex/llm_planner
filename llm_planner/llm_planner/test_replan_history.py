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

"""
Demo client that simulates a step that has already failed twice.

Sends a ReplanTask request for step 0 with:
  - failure_reason  : the CURRENT (3rd) failure
  - previous_failures: the two earlier failure reasons

This exercises the LLM's ability to avoid strategies that were already tried.

Usage:
  ros2 run llm_planner test_replan_history
"""

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from llm_planner_interfaces.srv import ReplanTask

class TestReplanHistoryNode(Node):
    def __init__(self):
        super().__init__('test_replan_history')

        self.declare_parameter('plan_file', '')

        plan_file = self.get_parameter('plan_file').get_parameter_value().string_value
        if not plan_file:
            pkg = get_package_share_directory('llm_planner')
            plan_file = pkg + '/config/test_replan_history.yaml'

        try:
            with open(plan_file, 'r') as f:
                plan_yaml = f.read()
            data = yaml.safe_load(plan_yaml)
            goal = data.get('goal', '')
            failed_step = data.get('failed_step', 0)
            previous_failures = data.get('previous_failures', [])
            current_failure = data.get('current_failure', '')
            skills_raw = data.get('skills', [])
            skills = [str(s) for s in skills_raw if str(s).strip()]
            mission_name = data.get('mission_name', '')
        except Exception as e:
            self.get_logger().error(f"Could not load plan file '{plan_file}': {e}")
            raise SystemExit(1)

        self._client = self.create_client(ReplanTask, 'replan_task')
        self.get_logger().info('Waiting for /replan_task service...')
        self._client.wait_for_service()

        self.get_logger().info(
            f'Service available.\n'
            f'  plan_file        : {plan_file}\n'
            f'  goal             : {goal}\n'
            f'  failed_step      : {failed_step}\n'
            f'  current failure  : {current_failure}\n'
            f'  previous failures: {len(previous_failures)}\n'
            f'  skills           : {len(skills)}'
        )

        req = ReplanTask.Request()
        req.goal = goal
        req.plan_yaml = plan_yaml
        req.failed_step = failed_step
        req.failure_reason = current_failure
        req.previous_failures = previous_failures
        req.skills = skills
        req.mission_name = mission_name

        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info('Replan generated:\n' + response.new_plan_yaml)
        else:
            self.get_logger().error(f'Replan failed: {response.message}')
        exit(0)


def main():
    rclpy.init()
    try:
        node = TestReplanHistoryNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()
