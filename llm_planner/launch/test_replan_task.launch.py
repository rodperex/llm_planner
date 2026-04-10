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
Launch file to test replan generation.

Starts llm_planner_node and calls test_replan_task with the given plan file,
failed step index, and failure reason.

Usage:
  ros2 launch llm_planner test_replan_task.launch.py
  ros2 launch llm_planner test_replan_task.launch.py \
    plan_file:=/path/to/plan.yaml \
    failed_step:=1 \
    failure_reason:="Robot could not detect the customer"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    plan_file_arg = DeclareLaunchArgument(
        'plan_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('llm_planner'), 'config', 'test_replan.yaml'
        ]),
        description='Path to plan YAML file (must have top-level goal + steps keys)',
    )

    failed_step_arg = DeclareLaunchArgument(
        'failed_step',
        default_value='0',
        description='0-based index of the step that failed',
    )

    failure_reason_arg = DeclareLaunchArgument(
        'failure_reason',
        default_value='The robot could not navigate to the target location',
        description='Why the step failed (used by LLM to avoid repeating the same error)',
    )

    test_node = Node(
        package='llm_planner',
        executable='test_replan_task',
        name='test_replan_task',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'plan_file': LaunchConfiguration('plan_file'),
            'failed_step': LaunchConfiguration('failed_step'),
            'failure_reason': LaunchConfiguration('failure_reason'),
        }],
    )

    return LaunchDescription([
        plan_file_arg,
        failed_step_arg,
        failure_reason_arg,
        test_node,
    ])
