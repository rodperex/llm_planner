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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the LLM planner node standalone.

    node_type:=normal  →  llm_planner_node        (generate only)
    node_type:=agent   →  llm_planner_agent_node   (generate + validate loop)
    """

    provider_arg = DeclareLaunchArgument(
        'provider',
        default_value='openai',
        description='LLM provider: gemini | openai | anthropic | deepseek | ollama',
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='gpt-4o',
        description='Model ID (e.g. gemini-2.5-flash, gpt-4o, llama3.1)',
    )

    key_arg = DeclareLaunchArgument(
        'key',
        default_value='',
        description='API key (optional — auto-detected from env vars if empty)',
    )

    node_type_arg = DeclareLaunchArgument(
        'node_type',
        default_value='normal',
        description=(
            'Planner variant: "normal" (generate only), "agent" (generate + validate), '
            '"parallel" (agent + parallel sub-actions in objective.steps)'
        ),
    )

    # ── Normal node ───────────────────────────────────────────────────────────
    normal_node = Node(
        package='llm_planner',
        executable='llm_planner_node',
        name='llm_planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'llm_provider': LaunchConfiguration('provider'),
            'llm_model_id': LaunchConfiguration('model'),
            'llm_api_key':  LaunchConfiguration('key'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('node_type'), "' == 'normal'"])
        ),
    )

    # ── Agent node (generate + 3-phase validation loop) ───────────────────────
    agent_node = Node(
        package='llm_planner',
        executable='llm_planner_agent_node',
        name='llm_planner_agent_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'llm_provider': LaunchConfiguration('provider'),
            'llm_model_id': LaunchConfiguration('model'),
            'llm_api_key':  LaunchConfiguration('key'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('node_type'), "' == 'agent'"])
        ),
    )

    # ── Parallel-aware agent node ─────────────────────────────────────────────
    parallel_node = Node(
        package='llm_planner',
        executable='llm_planner_agent_parallel_node',
        name='llm_planner_agent_parallel_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'llm_provider': LaunchConfiguration('provider'),
            'llm_model_id': LaunchConfiguration('model'),
            'llm_api_key':  LaunchConfiguration('key'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('node_type'), "' == 'parallel'"])
        ),
    )

    return LaunchDescription([
        provider_arg,
        model_arg,
        key_arg,
        node_type_arg,
        normal_node,
        agent_node,
        parallel_node,
    ])
