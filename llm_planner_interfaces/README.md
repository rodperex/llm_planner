# llm_planner_interfaces

ROS 2 CMake package that provides the custom service definitions used by [`llm_planner`](../llm_planner/). These interfaces decouple the service contracts from the implementation, making them reusable by any other package in the workspace (e.g. an orchestrator or a BT node).

---

## Services

### `PlanTask`

Generates a structured YAML execution plan from a high-level goal using an LLM.

**Request**

| Field | Type | Description |
|---|---|---|
| `goal` | `string` | High-level natural-language goal (e.g. `"Deliver food to table 2"`). |
| `context` | `string` | Optional context: robot role, environment layout, constraints, or current state. Helps the LLM break the goal into relevant sequential steps. |

**Response**

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | `true` if a valid plan was generated. |
| `plan_yaml` | `string` | YAML string of the generated plan (see plan format below). |
| `message` | `string` | Human-readable status or error message. |

---

### `ReplanTask`

Generates a revised YAML plan when a step has failed during execution. The new plan covers only the remaining work, avoids the confirmed broken capability, and respects the full history of previously blocked strategies.

**Request**

| Field | Type | Description |
|---|---|---|
| `goal` | `string` | Original high-level goal. |
| `plan_yaml` | `string` | The current plan YAML (original or a previous replan). |
| `failed_step` | `int32` | Zero-based index of the step that failed. |
| `failure_reason` | `string` | Why the step failed (world error, missing BT node, timeout, etc.). Used by the LLM to avoid the same failure. |
| `previous_failures` | `string[]` | Failure reasons from earlier replans of this same step. Empty on the first replan; grows with each retry. |

**Response**

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | `true` if a valid new plan was generated. |
| `new_plan_yaml` | `string` | YAML string of the revised plan. |
| `message` | `string` | Human-readable status or error message. |

---

### `StartMission`

Sent to an orchestrator node to start executing a high-level goal end-to-end. The orchestrator is responsible for planning, BT generation, step execution, and replanning on failure.

**Request**

| Field | Type | Description |
|---|---|---|
| `goal` | `string` | High-level natural-language goal. |
| `context` | `string` | Optional context: robot role, environment, constraints. BT node capabilities are provided separately via an orchestrator parameter. |

**Response**

| Field | Type | Description |
|---|---|---|
| `accepted` | `bool` | `false` if the orchestrator is already busy with another goal. |
| `message` | `string` | Human-readable status message. |

---

## Plan YAML format reference

Every `plan_yaml` / `new_plan_yaml` string follows this structure:

```yaml
goal: "<original goal>"
context: "<context used for planning>"
steps:
  - step_id: 0
    description: "<short human-readable label, 5-10 words>"
    objective:
      name: "<concise step name>"
      description: "<robot role from context>. Overall goal: <goal>. In this step: <specific task>"
      steps:
        - step: "<atomic sub-action>"
        - step: "<next sub-action>"
      constraints:
        <key>: <value>
      style:
        - <behavioural trait>
  - step_id: 1
    ...
```

---

## Build

```bash
cd <workspace_root>
colcon build --packages-select llm_planner_interfaces
source install/setup.bash
```

Verify the services are available after sourcing:

```bash
ros2 interface list | grep llm_planner_interfaces
ros2 interface show llm_planner_interfaces/srv/PlanTask
```

---

## Dependencies

| Dependency | Role |
|---|---|
| `ament_cmake` | Build system |
| `rosidl_default_generators` | Interface code generation (build time) |
| `rosidl_default_runtime` | Interface runtime support |

---

## License

Apache License 2.0
