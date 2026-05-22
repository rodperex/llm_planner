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

    node_type:=normal     →  llm_planner_node               (generate only)
    node_type:=agent      →  llm_planner_agent_node          (generate + validate loop)
    node_type:=multistep  →  llm_multistep_planner_node      (skeleton + step expansion)
    """

    provider_arg = DeclareLaunchArgument(
        'provider',
        default_value='openai',
        description='LLM provider: gemini | openai | anthropic | deepseek | ollama | groq | sambanova | cerebras',
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='gpt-4o',
        description='Model ID (e.g. gemini: gemini-2.5-flash, openai: gpt-4o, ollama: llama3.1, groq: llama-3.3-70b-versatile, sambanova: Meta-Llama-3.3-70B-Instruct, cerebras: llama3.1-8b).',
    )

    key_arg = DeclareLaunchArgument(
        'key',
        default_value='',
        description='API key (optional — auto-detected from env vars if empty)',
    )

    node_type_arg = DeclareLaunchArgument(
        'node_type',
        default_value='mcp',
        description=(
            'Planner variant: "normal" (generate only), "mcp" (normal + MCP context), "agent" (generate + validate), '
            '"parallel" (agent + parallel sub-actions in objective.steps), '
            '"multistep" (incremental skeleton + per-step expansion)'
        ),
    )

    mcp_enabled_arg = DeclareLaunchArgument(
        'mcp_enabled',
        default_value='true',
        description='Enable MCP context enrichment (used by mcp node).',
    )

    mcp_cmd_arg = DeclareLaunchArgument(
        'mcp_cmd',
        default_value='ros2 run mcp_context_server mcp_context_server',
        description='Command to start the MCP server process.',
    )

    mcp_timeout_arg = DeclareLaunchArgument(
        'mcp_timeout_sec',
        default_value='2.0',
        description='Timeout for MCP calls in seconds.',
    )

    mcp_fail_open_arg = DeclareLaunchArgument(
        'mcp_fail_open',
        default_value='true',
        description='If true, planner continues when MCP is unavailable.',
    )

    save_plan_arg = DeclareLaunchArgument(
        'save_plan',
        default_value='false',
        description='Save generated plans to disk (true/false).',
    )

    plan_prompt_file_arg = DeclareLaunchArgument(
        'plan_prompt_file',
        default_value='mcp_plan_prompt.txt',
        description='Plan prompt file for any planner node.',
    )

    replan_prompt_file_arg = DeclareLaunchArgument(
        'replan_prompt_file',
        default_value='mcp_replan_prompt.txt',
        description='Replan prompt file for any planner node.',
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
            'save_plan':    LaunchConfiguration('save_plan'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('node_type'), "' == 'normal'"])
        ),
    )

    # ── MCP normal node (generate only + MCP context enrichment) ───────────
    mcp_node = Node(
        package='llm_planner',
        executable='mcp_llm_planner_node',
        name='mcp_llm_planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'llm_provider': LaunchConfiguration('provider'),
            'llm_model_id': LaunchConfiguration('model'),
            'llm_api_key':  LaunchConfiguration('key'),
            'save_plan':    LaunchConfiguration('save_plan'),
            'plan_prompt_file': LaunchConfiguration('plan_prompt_file'),
            'replan_prompt_file': LaunchConfiguration('replan_prompt_file'),
            'mcp_enabled': LaunchConfiguration('mcp_enabled'),
            'mcp_cmd': LaunchConfiguration('mcp_cmd'),
            'mcp_timeout_sec': LaunchConfiguration('mcp_timeout_sec'),
            'mcp_fail_open': LaunchConfiguration('mcp_fail_open'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('node_type'), "' == 'mcp'"])
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
            'save_plan':    LaunchConfiguration('save_plan'),
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
            'save_plan':    LaunchConfiguration('save_plan'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('node_type'), "' == 'parallel'"])
        ),
    )

    # ── Multistep node (incremental plan generation) ─────────────────────────
    multistep_node = Node(
        package='llm_planner',
        executable='llm_multistep_planner_node',
        name='llm_multistep_planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'llm_provider': LaunchConfiguration('provider'),
            'llm_model_id': LaunchConfiguration('model'),
            'llm_api_key':  LaunchConfiguration('key'),
            'save_plan':    LaunchConfiguration('save_plan'),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('node_type'), "' == 'multistep'"])
        ),
    )

    return LaunchDescription([
        provider_arg,
        save_plan_arg,
        model_arg,
        key_arg,
        node_type_arg,
        plan_prompt_file_arg,
        replan_prompt_file_arg,
        mcp_enabled_arg,
        mcp_cmd_arg,
        mcp_timeout_arg,
        mcp_fail_open_arg,
        normal_node,
        mcp_node,
        agent_node,
        parallel_node,
        multistep_node,
    ])
