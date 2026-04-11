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
Launch file to test plan generation.

Starts llm_planner_node and calls test_plan_task with the given goal and context.

Usage:
  ros2 launch llm_planner test_plan_task.launch.py
  ros2 launch llm_planner test_plan_task.launch.py \
    goal:="Deliver food to table 2" \
    context:="NAO robot in a restaurant"
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_default(key: str, fallback=''):
    """Read a default value from config/test_plan.yaml at parse time."""
    import os
    import ament_index_python.packages as ament_idx
    try:
        share = ament_idx.get_package_share_directory('llm_planner')
        with open(os.path.join(share, 'config', 'test_plan.yaml')) as f:
            return yaml.safe_load(f).get(key, fallback)
    except Exception:
        return fallback


def generate_launch_description():

    goal_arg = DeclareLaunchArgument(
        'goal',
        default_value=_load_default('goal'),
        description='High-level goal for the planner',
    )

    context_arg = DeclareLaunchArgument(
        'context',
        default_value=_load_default('context'),
        description='Context describing robot role and environment',
    )

    test_node = Node(
        package='llm_planner',
        executable='test_plan_task',
        name='test_plan_task',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'goal': LaunchConfiguration('goal'),
            'context': LaunchConfiguration('context'),
            'skills': _load_default('skills', []),
        }],
    )

    return LaunchDescription([
        goal_arg,
        context_arg,
        test_node,
    ])
