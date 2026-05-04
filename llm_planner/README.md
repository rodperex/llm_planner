# llm_planner

Python package of the planner stack. It contains the ROS 2 nodes that turn a high-level goal into a YAML execution plan and, in agent modes, validate that plan before accepting it.

For the full repository overview, see the package-level README at `../README.md`. This file focuses on the Python package itself.

## Node variants

| Node | Purpose |
| --- | --- |
| `llm_planner_node.py` | Single-pass planning and replanning |
| `llm_planner_agent_node.py` | Planning with structural + feasibility + replan-consistency validation |
| `llm_planner_agent_parallel_node.py` | Same as agent mode, but accepts parallel groups in `objective.steps` |

## Services

| Service | Interface | Purpose |
| --- | --- | --- |
| `/plan_task` | `llm_planner_interfaces/PlanTask` | Generate a new plan from goal and context |
| `/replan_task` | `llm_planner_interfaces/ReplanTask` | Generate a revised plan after execution failure |

## What This Package Produces

Each plan step is designed to be passed downstream to `llm_bt_builder`. The most important block is `objective`, which must be self-contained.

Minimal step shape:

```yaml
steps:
  - step_id: 0
    description: "Detect waiting customer"
    objective:
      name: "Detect customer"
      description: "<role and environment>. Overall goal: <goal>. In this step: <task>."
      inputs:
        - some_input
      outputs:
        - some_output
      recovery_policy:
        required: true
        loop_until_success: true
        retry_attempts: forever
      steps:
        - step: "Set the perception system to search for a person"
        - step: "Spin while searching"
        - step: "Detect a person and store the result"
      style:
        - attentive
```

## Recovery Policy

`recovery_policy` is part of the planner-to-BT-builder contract.

```yaml
recovery_policy:
  required: true|false
  loop_until_success: true|false
  retry_attempts: int|forever
```

Meaning:

- `required`: the step needs an explicit success branch and a separate recovery/reacquisition branch.
- `loop_until_success`: the recovery branch must be designed to repeat until success.
- `retry_attempts`: retry bound expected later in the generated BT.

Guideline:

- If the task semantics mean "continue until success" or "hasta que", the policy should normally be `required: true`, `loop_until_success: true`, `retry_attempts: forever`.

This is important because `llm_bt_builder` validates BT XML against this contract. If the policy says recovery is required, the generated BT must contain explicit branching and matching retry structure.

## Agent Validation Loop

`llm_planner_agent_node.py` uses a 3-phase loop:

1. Programmatic structural validation
2. LLM-based feasibility validation
3. Programmatic replan consistency validation for replans

Failures are injected back into the next prompt under an error section so the LLM can self-correct.

## Prompts Used By This Package

Prompt files live in `prompts/`:

- `plan_prompt.txt`
- `plan_parallel_prompt.txt`
- `mcp_plan_prompt.txt`
- `replan_prompt.txt`
- `mcp_replan_prompt.txt`
- `validate_plan_prompt.txt`

The plan prompts define the output schema and the semantics of `recovery_policy`.

## Build And Run

```bash
cd <workspace_root>
colcon build --packages-select llm_planner_interfaces llm_planner
source install/setup.bash
```

Examples:

```bash
ros2 launch llm_planner llm_planner.launch.py node_type:=normal provider:=openai model:=gpt-4o
ros2 launch llm_planner llm_planner.launch.py node_type:=agent provider:=gemini model:=gemini-2.5-flash
ros2 launch llm_planner llm_planner.launch.py node_type:=parallel provider:=openai model:=gpt-4o
```

## Package Layout

```text
llm_planner/
├── llm_planner_node.py
├── llm_planner_agent_node.py
├── llm_planner_agent_parallel_node.py
├── test_plan_task.py
├── test_replan_task.py
└── test_replan_history.py
```

## Integration Note

When debugging a bad BT, inspect this package's generated `objective` first. Many downstream BT problems originate from a plan whose task text and `recovery_policy` disagree.

## License

Apache License 2.0
