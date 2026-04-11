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

from setuptools import find_packages, setup

package_name = 'llm_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/llm_planner.launch.py',
            'launch/test_plan_task.launch.py',
            'launch/test_replan_task.launch.py',
            'launch/test_replan_history.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/test_plan.yaml',
            'config/test_plan_2.yaml',
            'config/test_parallel.yaml',
            'config/test_replan.yaml',
            'config/test_replan_history.yaml',
        ]),
        ('share/' + package_name + '/prompts', [
            'prompts/plan_prompt.txt',
            'prompts/plan_parallel_prompt.txt',
            'prompts/replan_prompt.txt',
            'prompts/validate_plan_prompt.txt',
        ]),
    ],
    install_requires=['setuptools', 'requests', 'pyyaml'],
    zip_safe=True,
    maintainer='Rodrigo Pérez-Rodríguez',
    maintainer_email='rodrigo.perez@urjc.es',
    description='ROS 2 node that generates and revises step-by-step YAML execution plans for service robots using an LLM.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'llm_planner_node = llm_planner.llm_planner_node:main',
            'llm_planner_agent_node = llm_planner.llm_planner_agent_node:main',
            'llm_planner_agent_parallel_node = llm_planner.llm_planner_agent_parallel_node:main',
            'test_plan_task = llm_planner.test_plan_task:main',
            'test_replan_task = llm_planner.test_replan_task:main',
            'test_replan_history = llm_planner.test_replan_history:main',
        ],
    },
)
