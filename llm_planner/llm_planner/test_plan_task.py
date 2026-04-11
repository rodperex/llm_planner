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
Standalone client to test the plan_task service.

Usage:
  ros2 run llm_planner test_plan_task \
    --ros-args -p goal:="Greet the customer" -p context:="NAO robot in a restaurant" \\
    -p skills:="[\"Speak using TTS\", \"Listen using STT\", \"Navigate to a location\"]"

The skills parameter is a string array listing the robot's available capabilities.
If omitted, no skills constraint is sent and the LLM plans freely.
"""

import rclpy
from rclpy.node import Node

from llm_planner_interfaces.srv import PlanTask


class TestPlanTaskNode(Node):
    def __init__(self):
        super().__init__('test_plan_task')

        self.declare_parameter('goal', '')
        self.declare_parameter('context', '')
        self.declare_parameter('skills', [''])
        self.declare_parameter('mission_name', '')

        self._goal = self.get_parameter('goal').get_parameter_value().string_value
        self._task_context = self.get_parameter('context').get_parameter_value().string_value
        skills_raw = self.get_parameter('skills').get_parameter_value().string_array_value
        self._skills = [s for s in skills_raw if s.strip()]
        self._mission_name = self.get_parameter('mission_name').get_parameter_value().string_value

        if not self._goal:
            self.get_logger().error("Parameter 'goal' is required")
            raise SystemExit(1)

        self._client = self.create_client(PlanTask, 'plan_task')
        self.get_logger().info('Waiting for /plan_task service...')
        self._client.wait_for_service()
        self.get_logger().info('Service available — sending request')

        req = PlanTask.Request()
        req.goal = self._goal
        req.context = self._task_context
        req.skills = self._skills
        req.mission_name = self._mission_name

        future = self._client.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info('Plan generated successfully:\n' + response.plan_yaml)
        else:
            self.get_logger().error('Plan generation failed')
        # raise SystemExit(0)
        exit(0)


def main():
    rclpy.init()
    try:
        node = TestPlanTaskNode()
        rclpy.spin(node)
    except SystemExit as e:
        pass
    finally:
        rclpy.shutdown()
