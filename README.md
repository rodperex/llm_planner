# llm_planner

ROS 2 metapackage that turns a high-level natural-language goal into a structured, step-by-step YAML execution plan for a service robot, using a large language model (LLM) as the planning engine.

The repository contains two packages:

| Package | Type | Purpose |
|---|---|---|
| [`llm_planner`](llm_planner/) | `ament_python` | ROS 2 node that queries an LLM to produce and revise plans |
| [`llm_planner_interfaces`](llm_planner_interfaces/) | `ament_cmake` | Custom service definitions used by the planner |

---

## How it works

```
caller ──► /plan_task ──► LLMPlannerNode ──► LLM API ──► YAML plan
caller ──► /replan_task ─┘   (retry on failure + history of blocked strategies)
```

1. A client sends a `PlanTask` request with a high-level **goal** and optional **context** (robot role, environment, constraints).
2. The node builds a prompt from a configurable template and calls the configured LLM provider.
3. The LLM returns a YAML plan with 3–8 sequential steps. Each step contains a `description` and a self-contained `objective` block ready for a downstream Behavior Tree generator (e.g. `llm_bt_builder`).
4. If a step fails at execution time, the caller sends a `ReplanTask` request. The node passes the failure reason and the history of previous blocked strategies to the LLM, which replans avoiding all known-broken capabilities.
5. Every generated plan is persisted to `llm_planner/generated_plans/` with a timestamped filename.

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
# OpenAI GPT-4o (API key from OPENAI_API_KEY env var)
ros2 launch llm_planner llm_planner.launch.py provider:=openai model:=gpt-4o

# Google Gemini
ros2 launch llm_planner llm_planner.launch.py provider:=gemini model:=gemini-2.5-flash

# Local Ollama model
ros2 launch llm_planner llm_planner.launch.py provider:=ollama model:=llama3.1
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
│   │   ├── llm_planner_node.py      main ROS 2 node
│   │   ├── test_plan_task.py        CLI client: PlanTask
│   │   ├── test_replan_task.py      CLI client: ReplanTask
│   │   └── test_replan_history.py   CLI client: ReplanTask with history
│   ├── launch/
│   │   ├── llm_planner.launch.py
│   │   ├── test_plan_task.launch.py
│   │   ├── test_replan_task.launch.py
│   │   └── test_replan_history.launch.py
│   ├── config/
│   │   ├── test_plan.yaml           default goal/context for plan test
│   │   ├── test_replan.yaml         pre-built plan for replan test
│   │   └── test_replan_history.yaml plan + history for history test
│   ├── prompts/
│   │   ├── plan_prompt.txt          system prompt for PlanTask
│   │   └── replan_prompt.txt        system prompt for ReplanTask
│   ├── generated_plans/             auto-saved generated plans (gitignored except .gitkeep)
└── llm_planner_interfaces/  ← CMake interfaces package
    └── srv/
        ├── PlanTask.srv
        ├── ReplanTask.srv
        └── StartMission.srv
```

---

## ROS 2 services

| Service | Type | Description |
|---|---|---|
| `/plan_task` | `llm_planner_interfaces/PlanTask` | Generate a YAML plan from a goal |
| `/replan_task` | `llm_planner_interfaces/ReplanTask` | Generate a new plan after a step failure |

---

## License

Apache License 2.0 — see individual package files for details.
