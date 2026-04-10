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
Standalone client to test the replan_task service.

Requires a YAML file with the current plan plus the failed step info.

Usage:
  ros2 run llm_planner test_replan_task \
    --ros-args \
    -p plan_file:=/path/to/plan.yaml \
    -p failed_step:=1 \
    -p failure_reason:="Robot could not detect the customer"

The plan_file must contain a valid plan YAML with a top-level 'goal' key.
"""

import rclpy
from rclpy.node import Node

from llm_planner_interfaces.srv import ReplanTask


class TestReplanTaskNode(Node):
    def __init__(self):
        super().__init__('test_replan_task')

        self.declare_parameter('plan_file', '')
        self.declare_parameter('failed_step', 0)
        self.declare_parameter('failure_reason', '')

        plan_file = self.get_parameter('plan_file').get_parameter_value().string_value
        failed_step = self.get_parameter('failed_step').get_parameter_value().integer_value
        failure_reason = self.get_parameter('failure_reason').get_parameter_value().string_value

        if not plan_file:
            self.get_logger().error("Parameter 'plan_file' is required")
            raise SystemExit(1)

        try:
            with open(plan_file, 'r') as f:
                plan_yaml = f.read()
        except OSError as e:
            self.get_logger().error(f"Could not read plan file '{plan_file}': {e}")
            raise SystemExit(1)

        # Extract goal from the YAML plan
        import yaml
        try:
            doc = yaml.safe_load(plan_yaml)
            goal = doc.get('goal', '')
        except yaml.YAMLError as e:
            self.get_logger().error(f"Invalid YAML in plan file: {e}")
            raise SystemExit(1)

        if not goal:
            self.get_logger().error("Plan YAML must contain a top-level 'goal' key")
            raise SystemExit(1)

        self._client = self.create_client(ReplanTask, 'replan_task')
        self.get_logger().info('Waiting for /replan_task service...')
        self._client.wait_for_service()
        self.get_logger().info(
            f'Service available — requesting replan for goal="{goal}", '
            f'failed_step={failed_step}, reason="{failure_reason}"'
        )

        req = ReplanTask.Request()
        req.goal = goal
        req.plan_yaml = plan_yaml
        req.failed_step = failed_step
        req.failure_reason = failure_reason

        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info('Replan generated successfully:\n' + response.new_plan_yaml)
        else:
            self.get_logger().error(f'Replan failed: {response.message}')
        # raise SystemExit(0)
        exit(0)


def main():
    rclpy.init()
    try:
        node = TestReplanTaskNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()
