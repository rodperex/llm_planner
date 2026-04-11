# llm_planner

ROS 2 metapackage that turns a high-level natural-language goal into a structured, step-by-step YAML execution plan for a service robot, using a large language model (LLM) as the planning engine.

The repository contains two packages:

| Package | Type | Purpose |
|---|---|---|
| [`llm_planner`](llm_planner/) | `ament_python` | ROS 2 nodes that query an LLM to produce and revise plans |
| [`llm_planner_interfaces`](llm_planner_interfaces/) | `ament_cmake` | Custom service definitions used by the planner |

---

## Planner modes

Two node variants are available, selectable at launch with `node_type:=normal|agent`.

| Mode | Node | `node_type` | Plan files |
|---|---|---|---|
| **Normal** | `llm_planner_node` | `normal` | `plan_*.yaml` / `replan_*.yaml` |
| **Agent** | `llm_planner_agent_node` | `agent` | `agent_plan_*.yaml` / `agent_replan_*.yaml` |

---

### Normal mode (`node_type:=normal`)

**Node:** `llm_planner_node`

The simplest variant. Receives a goal, calls the LLM once, and returns the plan.

```
caller ──► /plan_task ──► LLMPlannerNode ──► LLM API ──► YAML plan
caller ──► /replan_task ─┘
```

**Behaviour:**
1. Builds a prompt from `plan_prompt.txt` / `replan_prompt.txt` and the request fields.
2. Calls the configured LLM provider (single attempt).
3. Returns the raw YAML response without any structural or feasibility check.
4. Saves the plan to `generated_plans/plan_TIMESTAMP_MODEL.yaml`.

**Best for:** fast prototyping, interactive demos, providers whose output is already very reliable.

---

### Agent mode (`node_type:=agent`)

**Node:** `llm_planner_agent_node`

A drop-in replacement that wraps every plan/replan generation in a deterministic **3-phase validation loop** (up to `MAX_RETRIES = 6` attempts). Each failed phase injects the error back into the next prompt so the LLM can self-correct.

```
[Generate plan]
    │
    ▼
Phase 1 — Structural validation  (programmatic, free)
    │  ✗ → inject error, retry
    ▼
Phase 2 — Feasibility validation  (LLM judge, 1 extra call)
    │  ✗ → inject error, retry
    ▼
Phase 3 — Replan consistency  (programmatic, free — replan only)
    │  ✗ → inject error, retry
    ▼
[Accept plan → respond]
```

**Phase 1 — Structural validation (programmatic)**

Checks the YAML output without any LLM call:

- YAML is parseable and is a dict.
- `steps` is a non-empty list.
- Each step has `step_id` (int), `description` (str), and `objective` (dict with `name`, `description`, `steps`).
- `step_id` values are sequential starting at 0 (no gaps, no reordering).
- Every entry in `skills_used` exists in the skills list provided by the caller.

**Phase 2 — Feasibility validation (LLM judge)**

A second, independent LLM call evaluates the plan against four semantic rules:

| Rule | Description |
|---|---|
| **SKILL COMPLIANCE** | Every step — including `objective.steps` text — uses only skills from the provided list. |
| **GOAL COVERAGE** | The full plan, when executed, achieves the entire goal with no sub-goal left unaddressed. |
| **PERCEPTION BEFORE ACTION** | Any step that navigates to or interacts with an ambiguous entity must be preceded by a detection step. |
| **DEPENDENCY ORDER** | Each step may only rely on information available from the context or from prior completed steps. |

The judge responds with exactly `VALID` or `ERROR: <step_id and rule violated>`. If the judge is unavailable or returns an unexpected format, Phase 2 is skipped (treated as valid) to avoid blocking the pipeline.

**Phase 3 — Replan consistency (programmatic, replan only)**

Checks that the new plan:

- Contains no step with `step_id < failed_step` (would repeat already-completed work).
- Does not re-propose any strategy that appears in `previous_failures` (keyword heuristic).

**Error feedback loop**

When a phase fails, the exact error message is appended to the next LLM prompt under a `## ERRORS FROM PREVIOUS ATTEMPT — fix ALL of these` section, guiding targeted correction instead of a blind retry.

**Plan files** are saved to  
`generated_plans/agent_plan_TIMESTAMP_MODEL.yaml` (plan) and  
`generated_plans/agent_replan_TIMESTAMP_MODEL.yaml` (replan).

**Best for:** production deployments, unattended missions, situations where downstream BT generation depends on a structurally and semantically correct plan.

---

## Supported LLM providers

| Provider | `llm_provider` value | Default endpoint |
|---|---|---|
| OpenAI | `openai` | `https://api.openai.com/v1/chat/completions` |
| Google Gemini | `gemini` | `https://generativelanguage.googleapis.com/v1beta/openai/chat/completions` |
| Anthropic Claude | `anthropic` | `https://api.anthropic.com/v1/messages` |
| DeepSeek | `deepseek` | `https://api.deepseek.com/v1/chat/completions` |
| Ollama (local) | `ollama` | `http://localhost:11434/v1/chat/completions` |

API keys are read from the `llm_api_key` parameter or from the corresponding environment variables (`OPENAI_API_KEY`, `GEMINI_API_KEY` / `GOOGLE_API_KEY`, `ANTHROPIC_API_KEY`, `DEEPSEEK_API_KEY`). For Ollama no key is needed.

---

## Prerequisites

- ROS 2 (Humble or later)
- Python dependencies: `requests`, `pyyaml`, `rclpy`
- A valid API key for the chosen provider (or a running Ollama instance)

---

## Build

```bash
cd <workspace_root>
colcon build --packages-select llm_planner_interfaces llm_planner
source install/setup.bash
```

---

## Quick start

### Start the planner node

```bash
# Normal mode — OpenAI GPT-4o
ros2 launch llm_planner llm_planner.launch.py node_type:=normal provider:=openai model:=gpt-4o

# Normal mode — Google Gemini
ros2 launch llm_planner llm_planner.launch.py node_type:=normal provider:=gemini model:=gemini-2.5-flash

# Agent mode — OpenAI GPT-4o (generate + 3-phase validation)
ros2 launch llm_planner llm_planner.launch.py node_type:=agent provider:=openai model:=gpt-4o

# Agent mode — local Ollama
ros2 launch llm_planner llm_planner.launch.py node_type:=agent provider:=ollama model:=llama3.1
```

### Test plan generation

```bash
# Uses goal/context from config/test_plan.yaml by default
ros2 launch llm_planner test_plan_task.launch.py

# Override inline
ros2 launch llm_planner test_plan_task.launch.py \
  goal:="Deliver food to table 2" \
  context:="NAO robot acting as a waiter in a restaurant with 5 tables"
```

### Test replanning after a step failure

```bash
# Uses a pre-built plan from config/test_replan.yaml with step 0 marked as failed
ros2 launch llm_planner test_replan_task.launch.py
```

### Test replanning with previous-failure history

```bash
# Passes two earlier blocked strategies so the LLM must propose a third approach
ros2 launch llm_planner test_replan_history.launch.py
```

---

## Repository structure

```
llm_planner/               ← repo root
├── llm_planner/           ← Python ROS 2 package
│   ├── llm_planner/
│   │   ├── llm_planner_node.py         Normal mode node
│   │   ├── llm_planner_agent_node.py   Agent mode node (3-phase validation)
│   │   ├── test_plan_task.py           CLI client: PlanTask
│   │   ├── test_replan_task.py         CLI client: ReplanTask
│   │   └── test_replan_history.py      CLI client: ReplanTask with history
│   ├── launch/
│   │   ├── llm_planner.launch.py       Main launch file (node_type:=normal|agent)
│   │   ├── test_plan_task.launch.py
│   │   ├── test_replan_task.launch.py
│   │   └── test_replan_history.launch.py
│   ├── config/
│   │   ├── test_plan.yaml              default goal/context for plan test
│   │   ├── test_replan.yaml            pre-built plan for replan test
│   │   └── test_replan_history.yaml    plan + history for history test
│   ├── prompts/
│   │   ├── plan_prompt.txt             system prompt for PlanTask
│   │   ├── replan_prompt.txt           system prompt for ReplanTask
│   │   └── validate_plan_prompt.txt    system prompt for the feasibility judge (agent mode)
│   └── generated_plans/               auto-saved plans (gitignored except .gitkeep)
│       ├── plan_*.yaml                 saved by normal mode
│       ├── replan_*.yaml               saved by normal mode
│       ├── agent_plan_*.yaml           saved by agent mode
│       └── agent_replan_*.yaml         saved by agent mode
└── llm_planner_interfaces/  ← CMake interfaces package
    └── srv/
        ├── PlanTask.srv
        ├── ReplanTask.srv
        └── StartMission.srv
```

---

## ROS 2 services

Both modes expose the same service interface.

| Service | Type | Description |
|---|---|---|
| `/plan_task` | `llm_planner_interfaces/PlanTask` | Generate a YAML plan from a goal |
| `/replan_task` | `llm_planner_interfaces/ReplanTask` | Generate a new plan after a step failure |

---

## Node parameters

| Parameter | Default | Description |
|---|---|---|
| `llm_provider` | `gemini` | LLM provider: `gemini` \| `openai` \| `anthropic` \| `deepseek` \| `ollama` |
| `llm_model_id` | `gemini-2.5-flash` | Model identifier (provider-specific) |
| `llm_api_url` | `''` | Override the default endpoint URL |
| `llm_api_key` | `''` | API key (auto-detected from env vars if empty) |
| `plan_prompt_file` | `plan_prompt.txt` | System prompt file for plan generation |
| `replan_prompt_file` | `replan_prompt.txt` | System prompt file for replanning |
| `validate_prompt_file` | `validate_plan_prompt.txt` | System prompt for the feasibility judge *(agent mode only)* |

---

## License

Apache License 2.0 — see individual package files for details.
