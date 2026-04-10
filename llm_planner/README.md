# llm_planner

ROS 2 Python package that exposes a planning node capable of converting a high-level natural-language robot goal into a structured, step-by-step YAML execution plan using a large language model (LLM). Supports initial planning and iterative replanning when a step fails.

---

## Overview

`LLMPlannerNode` provides two ROS 2 services:

| Service | Interface | Description |
|---|---|---|
| `/plan_task` | `llm_planner_interfaces/PlanTask` | Generate a full execution plan from a goal and context |
| `/replan_task` | `llm_planner_interfaces/ReplanTask` | Generate a revised plan when a step has failed, avoiding all previously blocked strategies |

Each plan is a YAML document made of sequential **steps**. Every step contains:
- `step_id` — zero-based index.
- `description` — short human-readable label (5–10 words).
- `objective` — a self-contained block (`name`, `description`, `steps`, `constraints`, `style`) intended to be passed directly to a downstream Behavior Tree generator such as `llm_bt_builder`.

Generated plans are automatically persisted to `generated_plans/` with a timestamped, model-tagged filename.

---

## Supported LLM providers

| Provider | `llm_provider` | Default API URL | Key env var |
|---|---|---|---|
| OpenAI | `openai` | `https://api.openai.com/v1/chat/completions` | `OPENAI_API_KEY` |
| Google Gemini | `gemini` | `https://generativelanguage.googleapis.com/v1beta/openai/chat/completions` | `GEMINI_API_KEY` or `GOOGLE_API_KEY` |
| Anthropic Claude | `anthropic` | `https://api.anthropic.com/v1/messages` | `ANTHROPIC_API_KEY` |
| DeepSeek | `deepseek` | `https://api.deepseek.com/v1/chat/completions` | `DEEPSEEK_API_KEY` |
| Ollama (local) | `ollama` | `http://localhost:11434/v1/chat/completions` | *(not needed)* |

The API key is resolved in order: `llm_api_key` ROS parameter → environment variable → `sk-no-key-needed` (for Ollama).

---

## Node parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `llm_provider` | `string` | `gemini` | LLM provider identifier (see table above) |
| `llm_model_id` | `string` | `gemini-2.5-flash` | Model ID string passed to the API |
| `llm_api_url` | `string` | `''` | Override the default API endpoint (leave empty to use the default) |
| `llm_api_key` | `string` | `''` | API key (optional — auto-detected from env vars if empty) |
| `plan_prompt_file` | `string` | `plan_prompt.txt` | Filename of the system prompt used for `PlanTask`, loaded from `prompts/` |
| `replan_prompt_file` | `string` | `replan_prompt.txt` | Filename of the system prompt used for `ReplanTask`, loaded from `prompts/` |

---

## Package structure

```
llm_planner/
├── llm_planner/
│   ├── llm_planner_node.py        ROS 2 node (LLMPlannerNode)
│   ├── test_plan_task.py          CLI helper: calls /plan_task once and prints the result
│   ├── test_replan_task.py        CLI helper: calls /replan_task with a pre-built plan
│   └── test_replan_history.py     CLI helper: calls /replan_task with failure history
├── launch/
│   ├── llm_planner.launch.py              Start the planner node
│   ├── test_plan_task.launch.py           Start node + single plan request
│   ├── test_replan_task.launch.py         Start node + replan request (failed step)
│   └── test_replan_history.launch.py      Start node + replan request (with history)
├── config/
│   ├── test_plan.yaml             Default goal/context loaded by test_plan_task.launch.py
│   ├── test_replan.yaml           Pre-built plan (step 0 marked failed) for replan test
│   └── test_replan_history.yaml   Plan + two earlier blocked strategies for history test
├── prompts/
│   ├── plan_prompt.txt            System prompt for plan generation
│   └── replan_prompt.txt          System prompt for replanning
└── generated_plans/               Auto-saved generated plans (gitignored, .gitkeep only)
```

---

## Build

```bash
cd <workspace_root>
colcon build --packages-select llm_planner_interfaces llm_planner
source install/setup.bash
```

---

## Usage

### Run the planner node

```bash
# Default: OpenAI GPT-4o
ros2 launch llm_planner llm_planner.launch.py provider:=openai model:=gpt-4o

# Google Gemini 2.5 Flash (key from GEMINI_API_KEY)
ros2 launch llm_planner llm_planner.launch.py provider:=gemini model:=gemini-2.5-flash

# Local Ollama
ros2 launch llm_planner llm_planner.launch.py provider:=ollama model:=llama3.1
```

### Call the service manually

```bash
ros2 service call /plan_task llm_planner_interfaces/srv/PlanTask \
  "{goal: 'Deliver food to table 2', context: 'NAO robot waiter in a restaurant'}"
```

### Integrated test launches

```bash
# Plan from default config/test_plan.yaml
ros2 launch llm_planner test_plan_task.launch.py

# Override goal/context inline
ros2 launch llm_planner test_plan_task.launch.py \
  goal:="Guide the customer to their table" \
  context:="NAO robot host in a restaurant with 5 tables"

# Replan after step 0 failure (uses config/test_replan.yaml)
ros2 launch llm_planner test_replan_task.launch.py

# Replan with previous-failure history (uses config/test_replan_history.yaml)
ros2 launch llm_planner test_replan_history.launch.py
```

---

## Plan YAML format

```yaml
goal: "Deliver food to table 2"
context: "NAO robot acting as a waiter in a restaurant with 5 tables."
steps:
  - step_id: 0
    description: "Locate the customer at table 2"
    objective:
      name: "Detect customer at table 2"
      description: >
        NAO robot acting as a waiter in a restaurant with 5 tables.
        Overall goal: Deliver food to table 2.
        In this step: Use the camera to identify the customer seated at table 2.
      steps:
        - step: "Activate camera and scan the restaurant"
        - step: "Detect person at table 2 using object detection"
      constraints:
        max_distance_m: 5
      style:
        - safe
        - polite
  - step_id: 1
    ...
```


---

## Prompt customization

The system prompts that guide the LLM are plain text files in `prompts/`:

- `plan_prompt.txt` — instructs the LLM to break a goal into 3–8 sequential steps, require perception before navigation, and output only valid YAML with no markdown fences.
- `replan_prompt.txt` — instructs the LLM to avoid only the confirmed broken capability, preserve all completed steps, and propose a genuinely different approach for each blocked strategy.

You can swap or extend these files without rebuilding — just update the `plan_prompt_file` / `replan_prompt_file` parameters to point to your custom files.

---

## License

Apache License 2.0
