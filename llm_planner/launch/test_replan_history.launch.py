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
  ros2 launch llm_planner test_replan_history.launch.py config_file:=test_replan_history.yaml

The config_file argument (or the TEST_REPLAN_CONFIG env var) selects which YAML
file under <package>/config/ is used as the default plan_file.
"""

import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_config_file() -> str:
    """Pick the config filename at parse time.

    Priority (highest first):
      1. config_file:=<name> launch argument (parsed from sys.argv)
      2. TEST_REPLAN_CONFIG environment variable
      3. Default: test_replan_history.yaml
    """
    import os
    for arg in sys.argv:
        if arg.startswith('config_file:='):
            return arg.split(':=', 1)[1]
    return os.environ.get('TEST_REPLAN_CONFIG', 'test_replan_history.yaml')


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=_resolve_config_file(),
        description='Config YAML filename (under <package>/config/) used as the default plan_file',
    )

    plan_file_arg = DeclareLaunchArgument(
        'plan_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('llm_planner'), 'config', _resolve_config_file()
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

    return LaunchDescription([config_file_arg, plan_file_arg, test_node])
