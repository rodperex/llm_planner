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
Demo: replan with failure history.

Assumes llm_planner_node is already running (e.g. via llm_planner.launch.py).
Sends a replan request for step 0 that has already failed twice before,
so the LLM must produce a strategy different from both previous attempts.

Usage:
  ros2 launch llm_planner test_replan_history.launch.py
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
            FindPackageShare('llm_planner'), 'config', 'test_replan_history.yaml'
        ]),
        description='Path to plan YAML file to replan from',
    )

    test_node = Node(
        package='llm_planner',
        executable='test_replan_history',
        name='test_replan_history',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'plan_file': LaunchConfiguration('plan_file'),
        }],
    )

    return LaunchDescription([plan_file_arg, test_node])
