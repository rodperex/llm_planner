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
  ros2 launch llm_planner test_plan_task.launch.py config_file:=test_plan_2.yaml
  ros2 launch llm_planner test_plan_task.launch.py \
    goal:="Deliver food to table 2" \
    context:="Service robot in a restaurant"

The config_file argument (or the TEST_PLAN_CONFIG env var) selects which YAML
file under <package>/config/ is used to populate the default values for goal,
context, mission_name and skills.
"""

import sys
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_config_file() -> str:
    """Pick the config filename at parse time.

    Priority (highest first):
      1. config_file:=<name> launch argument (parsed from sys.argv)
      2. TEST_PLAN_CONFIG environment variable
      3. Default: test_plan.yaml
    """
    import os
    for arg in sys.argv:
        if arg.startswith('config_file:='):
            return arg.split(':=', 1)[1]
    return os.environ.get('TEST_PLAN_CONFIG', 'test_plan.yaml')


def _load_default(key: str, fallback=''):
    """Read a default value from the resolved config YAML at parse time."""
    import os
    import ament_index_python.packages as ament_idx
    try:
        share = ament_idx.get_package_share_directory('llm_planner')
        cfg = _resolve_config_file()
        with open(os.path.join(share, 'config', cfg)) as f:
            return yaml.safe_load(f).get(key, fallback)
    except Exception:
        return fallback


def generate_launch_description():

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=_resolve_config_file(),
        description='Config YAML filename (under <package>/config/) with default goal/context/skills',
    )

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

    mission_name_arg = DeclareLaunchArgument(
        'mission_name',
        default_value=_load_default('mission_name', ''),
        description='Short identifier used to prefix saved plan filenames',
    )

    test_node = Node(
        package='llm_planner',
        executable='test_plan_task',
        name='test_plan_task',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'goal': ParameterValue(LaunchConfiguration('goal'), value_type=str),
            'context': ParameterValue(LaunchConfiguration('context'), value_type=str),
            'mission_name': ParameterValue(LaunchConfiguration('mission_name'), value_type=str),
            'skills': _load_default('skills', []),
        }],
    )

    return LaunchDescription([
        config_file_arg,
        goal_arg,
        context_arg,
        mission_name_arg,
        test_node,
    ])
