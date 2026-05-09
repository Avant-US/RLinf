# CLAUDE.md

This file is the Claude entrypoint for repository guidance.

For the canonical agent instructions, repository orientation, and workflow expectations, see [AGENTS.md](AGENTS.md).

For full contribution flow, code style, and PR process, see [CONTRIBUTING.md](CONTRIBUTING.md).

Keeping the detailed guidance in `AGENTS.md` avoids duplication and prevents the two files from drifting out of sync.
**Single machine:** Install via Docker or `bash requirements/install.sh embodied --model <model> --env <env>` (set `REPO_PATH` and any asset paths). Ray may auto-start; or run `ray start --head`. Use a config with `cluster.num_nodes: 1` (e.g. from `examples/embodiment/config/`). Launch with `bash examples/embodiment/run_embodiment.sh <config_name>` or `python examples/embodiment/train_embodied_agent.py --config-name <config_name>`, and set env vars the example needs (e.g. `MUJOCO_GL=egl`, `ROBOT_PLATFORM`).

**Multiple machines:** On each node, *before* `ray start`: set `export RLINF_NODE_RANK=<0..N-1>` (unique) and optionally `RLINF_COMM_NET_DEVICES`. Head: `ray start --head --port=6379 --node-ip-address=<head_ip>`. Workers: `ray start --address=<head_ip>:6379`. You can use `ray_utils/start_ray.sh`. Set `cluster.num_nodes` to the total; optionally use `node_groups` and `component_placement` (see `rlinf/scheduler/cluster/config.py` and the [heterogeneous cluster tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/advance/hetero.html)). Run the entry script *only on the head*; it attaches to the existing Ray cluster and schedules workers by placement.

---

## Configuration guides

- **Placement and throughput:** Configure `cluster.component_placement` (collocated vs disaggregated vs hybrid, node groups, hardware ranks). See [placement tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/user/placement.html) and [execution modes](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/mode/index.html).
- **OOM:** Tune env (`total_num_envs`, `group_size`), rollout (batch/seq, `gpu_memory_utilization`, `enable_offload`), actor (`micro_batch_size`, `global_batch_size`, `gradient_checkpointing`, `enable_offload`). Example configs in `examples/embodiment/config/`. See [FAQ](https://rlinf.readthedocs.io/en/latest/rst_source/faq.html) for SGLang/memory issues.
- **Multi-node and hetero:** Set `cluster.num_nodes`; set `RLINF_NODE_RANK` (and optionally `RLINF_COMM_NET_DEVICES`) **before** `ray start` on each node—Ray captures env at start time. Optional `node_groups` and `component_placement` in YAML; `env_configs` (e.g. `env_vars`, `python_interpreter_path`) are applied at worker allocation. See [heterogeneous cluster](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/advance/hetero.html) and `rlinf/scheduler/cluster/config.py`.

---

## Metrics, checkpoints, and evaluation

- **Metrics:** Runners use `MetricLogger`; set `runner.logger.logger_backends` (e.g. tensorboard, wandb, swanlab). Namespaces include `train/`, `eval/`, `env/`, `rollout/`, `time/`. See [logger tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/advance/logger.html).
- **Checkpoints:** Saved every `runner.save_interval` under `.../checkpoints/global_step_<N>/`. To resume, set `runner.resume_dir` to that path and relaunch; some runners support `resume_dir: auto`. See [checkpoint resume tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/advance/resume.html).
- **Evaluation:** During training, `runner.val_check_interval` triggers validation. Standalone embodied: `bash examples/embodiment/eval_embodiment.sh <config_name>` with an eval config; see [VLA evaluation](https://rlinf.readthedocs.io/en/latest/rst_source/start/vla-eval.html). Reasoning/LLM: see [LLM evaluation](https://rlinf.readthedocs.io/en/latest/rst_source/start/llm-eval.html).

---

## When things go wrong

For debugging (breakpoints, rendering/EGL, network, NCCL/CUDA, timeouts), see the [FAQ](https://rlinf.readthedocs.io/en/latest/rst_source/faq.html) in Further reading.

---

## Key ideas and plugging in

**Config** (`rlinf/config.py`): `build_config` / `validate_cfg` produce the full DictConfig. New model or env types go into `SupportedModel` / `SupportedEnvType` and validation.

**Cluster and placement:** `ClusterConfig` and strategies in `rlinf/scheduler/placement/`, `rlinf/utils/placement.py`. Placement controls where actor/rollout/env run (one node vs many, GPU vs CPU, heterogeneous).

**Algorithms:** Advantage and loss functions are registered in `rlinf/algorithms/` (registry + decorators); rewards are registered in `rlinf/algorithms/rewards/`. Config keys `algorithm.adv_type` and `algorithm.loss_type` select them. See [Extending RLinf: algorithms, models, envs](#extending-rlinf-algorithms-models-envs) for step-by-step instructions.

**Models (embodied):** Register in `SupportedModel` in `config.py`, implement under `rlinf/models/embodiment/<name>/` (e.g. `BasePolicy`), wire in config and workers. Use add-install-docker-ci-e2e for install/Docker/CI. Details in the extension section below.

**Environments:** Register in `SupportedEnvType` and `get_env_cls()` in `rlinf/envs/__init__.py`, implement under `rlinf/envs/<name>/`. Use add-install-docker-ci-e2e and add-example-doc-model-env for install and docs. Details below.

**Workers:** Subclass `Worker`, implement `initialize` and your API, launch with `create_group(...).launch(...)`. Use `self.log_info` / `log_warning` / `log_error`; no print.

**Runners:** They own the training loop. New task type = new runner + entry script that builds Cluster, placement, worker groups, and calls the runner.

---

## Extending RLinf: algorithms, models, envs

### New algorithms (advantage, loss, reward)

**Advantage function**

- Implement a function that takes the same keyword args as existing ones (e.g. `rewards`, `values`, `dones`, `gamma`, `loss_mask`, …) and returns `(advantages, returns)`. See `rlinf/algorithms/advantages.py` (e.g. `compute_gae_advantages_and_returns`) for signatures.
- Register it: `from rlinf.algorithms.registry import register_advantage` then `@register_advantage("my_adv")` on your function. The name is case-normalized to lowercase.
- In config YAML set `algorithm.adv_type: my_adv`. Actor workers call `calculate_adv_and_returns(adv_type=...)` which dispatches via `get_adv_and_returns(name)`.
- For non-GAE styles (e.g. GRPO, Reinforce++), `rlinf/algorithms/utils.py` may need to compute scores first; check how `adv_type` is used in `calculate_adv_and_returns` and in the actor worker.

**Policy loss**

- Implement a function that accepts the kwargs passed by the actor (e.g. `logprobs`, `old_logprobs`, `advantages`, `clip_ratio_low`, `clip_ratio_high`, `loss_mask`, …) and returns `(loss_tensor, metrics_dict)`. See `rlinf/algorithms/losses.py` (e.g. `compute_ppo_actor_loss`, `compute_ppo_actor_critic_loss`, `compute_grpo_actor_loss_fn`).
- Register: `from rlinf.algorithms.registry import register_policy_loss` then `@register_policy_loss("my_loss")`.
- In config set `algorithm.loss_type: my_loss`. For PPO-style actor+critic you need a critic and value loss; the unified entry is `policy_loss(loss_type=..., **kwargs)` in `registry.py`. Add validation in `rlinf/config.py` if your loss has special requirements (e.g. `validate_cfg` already checks `loss_type == "actor_critic"` for value head).

**Reward**

- Add a reward class (e.g. under `rlinf/algorithms/rewards/<domain>/`) that matches the interface expected by the reward worker (e.g. callable or class with a clear contract for prompt/completions/ids).
- In `rlinf/algorithms/rewards/__init__.py`: import the class, then `register_reward("my_reward", MyRewardClass)`. The registry is `reward_registry`; lookup via `get_reward_class(name)`.
- Wire the reward name in config and in the runner/reward worker so the correct class is instantiated and used. For reasoning/agent tasks the config path may be under `reward.path` or similar.

### New embodied model

- **Registration:** In `rlinf/config.py`, add a new value to the `SupportedModel` enum: `MY_MODEL = ("my_model", "embodied")`. Use `get_supported_model(model_type)` in validation so `model.model_type: my_model` is accepted.
- **Implementation:** Create a package under `rlinf/models/embodiment/my_model/`. For policies that fit the embodied actor interface, inherit from `rlinf.models.embodiment.base_policy.BasePolicy` and implement `default_forward` and `predict_action_batch`; add other forward types (e.g. `sac_forward`, `crossq_forward`) if the algorithm needs them. For HuggingFace-based VLAs, follow the pattern in the docs: register config and processor in `rlinf/models/__init__.py` (`get_model_config_and_processor`), then implement an action model that wraps generation and optional value head.
- **Config and workers:** Ensure `build_config` / default configs provide the right `model.model_type`, checkpoint paths, and any model-specific options. Actor and rollout workers already branch on `cfg.actor.model.model_type` / `cfg.rollout.model.model_type`; add branches or a factory so your model is instantiated and used. For FSDP+HuggingFace, see the [new model (FSDP) tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/extend/new_model_fsdp.html); for Megatron there is a separate [new model (Megatron) tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/extend/new_model_megatron.html).
- **Install and CI:** If the model needs extra deps or a dedicated venv, add it to `requirements/install.sh` (e.g. `SUPPORTED_MODELS`, and an `install_my_model()` or branch in the model switch). For Docker and e2e: use the skill `.cursor/skills/add-install-docker-ci-e2e` (install script, Dockerfile stage, CI job, e2e config under `tests/e2e_tests/embodied/`).

### New environment

- **Registration:** In `rlinf/envs/__init__.py`, add a member to `SupportedEnvType`: e.g. `MY_ENV = "my_env"`. In `get_env_cls(env_type, env_cfg=None, ...)` add an `elif env_type == SupportedEnvType.MY_ENV:` branch that imports your env class and returns it (lazy import to avoid loading heavy deps at import time). If the env needs a task id (like IsaacLab), use `env_cfg` and document the expected shape.
- **Implementation:** Create `rlinf/envs/my_env/` with at least one module defining a gym-style env (e.g. `gymnasium.Env`): `reset`, `step`, and the usual attributes (`observation_space`, `action_space`). Follow the [new environment tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/extend/new_env.html) for the expected structure (e.g. vectorized `num_envs`, `group_size`, `ret_device`). If your env uses custom action formatting, add a branch in `rlinf/envs/action_utils.py` in `prepare_actions(env_type, ...)` so rollout/workers pass correctly shaped actions.
- **Config:** Set `env.train.env_type` and `env.eval.env_type` to the string value of your enum (e.g. `my_env`). Add any env-specific defaults or validation in `rlinf/config.py` (e.g. `validate_cfg` already has env-specific checks for ManiSkill, Behavior, etc.; add similar ones if needed).
- **Install and docs:** For install/Docker/CI, use `.cursor/skills/add-install-docker-ci-e2e` (add env to `SUPPORTED_ENVS`, install logic, e2e config). For example docs and RST, use `.cursor/skills/add-example-doc-model-env`.

---

## Style and contributing

Google Python style; Ruff for lint/format; docstrings and type hints on public APIs. Logging: `rlinf.utils.logging.get_logger()` or Workers’ `self.log_*`. Config YAML: static values only; no computed fields; don’t overwrite user-facing fields in code. Commits: [Conventional Commits](https://www.conventionalcommits.org/), ~72-char subject, imperative; every commit `Signed-off-by:` (e.g. `git commit -s`). PRs: same title format, fill template, link issues; for perf-sensitive changes include test results. New behavior needs tests (unit or e2e); if e2e needs GPUs/hardware, document and skip appropriately in CI. Full details: [CONTRIBUTING.md](CONTRIBUTING.md).

---

## Further reading

- [Docs (EN)](https://rlinf.readthedocs.io/en/latest/) · [中文](https://rlinf.readthedocs.io/zh-cn/latest/)
- [Installation](https://rlinf.readthedocs.io/en/latest/rst_source/start/installation.html) · [VLA quickstart](https://rlinf.readthedocs.io/en/latest/rst_source/start/vla.html)
- [Example gallery](https://rlinf.readthedocs.io/en/latest/rst_source/examples/index.html) · configs in `examples/embodiment/config/`, `examples/reasoning/`, etc.
- Tutorials: [placement / cluster / YAML](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/user/index.html), [hybrid / disaggregated](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/mode/index.html), [heterogeneous cluster](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/advance/hetero.html), [extend (new env/model)](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/extend/index.html), [RL algorithms](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/rlalg/index.html), [logger (metrics)](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/advance/logger.html), [checkpoint resume](https://rlinf.readthedocs.io/en/latest/rst_source/tutorials/advance/resume.html)
- Evaluation: [VLA evaluation](https://rlinf.readthedocs.io/en/latest/rst_source/start/vla-eval.html) · [LLM evaluation](https://rlinf.readthedocs.io/en/latest/rst_source/start/llm-eval.html)
- [APIs](https://rlinf.readthedocs.io/en/latest/rst_source/apis/index.html) (actor, channel, cluster, placement, worker, env, data, …) · [FAQ](https://rlinf.readthedocs.io/en/latest/rst_source/faq.html)


# Old_CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RLinf is a distributed RL training infrastructure for Embodied and Agentic AI. It uses **Ray** for distributed process management and **Hydra** for configuration. Python >=3.10, <=3.11.14.

## Common Commands

### Installation
```bash
# Embodied RL with a specific model and environment
bash requirements/install.sh embodied --model pi0 --env maniskill

# Reasoning/agentic tasks with SGLang
bash requirements/install.sh reason && pip install -e ".[agentic-sglang]"

# Install as library
pip install rlinf
```

### Development & Linting
```bash
# Setup pre-commit (required before making commits)
pip install pre-commit && pre-commit install --hook-type commit-msg

# Run linter/formatter on all files
pre-commit run --all-files

# Format and lint a specific file
ruff check --fix <file.py> && ruff format <file.py>
```

### Testing
```bash
# All unit tests (fast, no GPU)
pytest tests/unit_tests/ -v

# Single test file
pytest tests/unit_tests/test_placement.py -v

# Single test
pytest tests/unit_tests/test_placement.py::test_node_placement_strategy -v

# E2E tests (require GPU)
pytest tests/e2e_tests/embodied/ -v
```

### Training
```bash
# Single-node embodied RL
python examples/embodiment/train_embodied_agent.py --config-name maniskill_ppo_openvlaoft

# Multi-node: set RLINF_NODE_RANK on each node BEFORE ray start
export RLINF_NODE_RANK=0  # unique per node
ray start --head --port=6379 --node-ip-address=<head_ip>
# Then run training on head node only
```

### Docs
```bash
cd docs && bash autobuild.sh        # English at localhost:8000
cd docs && bash autobuild.sh zh     # Chinese
```

## Architecture

Three-layer architecture: **Scheduler → Workers → Runner**.

**Scheduler** (`rlinf/scheduler/`): Sets up Ray cluster, places components on nodes/GPUs. Key abstractions: `Cluster`, `PlacementStrategy`, `Worker`/`WorkerGroup`, `Channel` (async message passing between groups).

**Workers** (`rlinf/workers/`): Each implements an RL role:
- `actor/` — Policy/critic updates (FSDP or Megatron backends)
- `rollout/` — Trajectory generation via HuggingFace, SGLang, or vLLM
- `env/` — Simulator or real-world environment wrappers
- `reward/` — Scalar reward computation
- `critic/` — Critic network workers
- `agent/`, `inference/` — LLM agent workers (SearchR1, WideSeek-R1)
- `sft/` — Supervised fine-tuning workers

**Runners** (`rlinf/runners/`): Orchestrate the training loop:
`collect_rollouts → compute_rewards → compute_advantages → training_step → checkpoint/eval`

**Algorithms** (`rlinf/algorithms/`): Registry-based advantages, losses, and rewards selected via YAML config keys `algorithm.adv_type` and `algorithm.loss_type`.

## Configuration Flow

1. **YAML config** (e.g. `examples/embodiment/config/*.yaml`) specifies model, env, algorithm, cluster layout, hyperparams
2. **`validate_cfg()`** in `rlinf/config.py` parses and validates config
3. **Entry script** (e.g. `train_embodied_agent.py`) uses `@hydra.main()` to load config, creates Cluster → PlacementStrategy → WorkerGroups → Runner → `runner.train()`

Key config/registry files:
- `rlinf/config.py` — `SupportedModel` enum, `validate_cfg()`, Hydra dataclasses
- `rlinf/envs/__init__.py` — `SupportedEnvType` enum, `get_env_cls()`
- `rlinf/algorithms/registry.py` — `@register_advantage`, `@register_policy_loss`, `@register_reward`

## Code Style & Conventions

- **Style**: Google Python Style Guide; Ruff for lint + format
- **Logging**: Use `self.log_info/log_warning/log_error` in Workers; use `rlinf.utils.logging.get_logger()` elsewhere — never `print`
- **Config YAML**: Static values only; no computed fields; never overwrite user-facing fields in code
- **Commits**: Must be Signed-off-by (`git commit -s`), follow Conventional Commits (`feat(scope): description`)
- **Pre-commit hooks**: Ruff lint/format + commit-check (message format, branch name, signoff)

## Extension Guide

See [AGENTS.md](AGENTS.md) for detailed step-by-step instructions on adding new algorithms, models, and environments. Key patterns:
- **New algorithm**: Register function with decorator in `rlinf/algorithms/`, select via YAML
- **New model**: Add to `SupportedModel` enum, implement under `rlinf/models/embodiment/<name>/`
- **New environment**: Add to `SupportedEnvType`, implement under `rlinf/envs/<name>/`, register in `get_env_cls()`

## CI & Testing

- **Unit tests** (`tests/unit_tests/`): Scheduler, placement, config — fast, no GPU
- **E2E tests** (`tests/e2e_tests/`): Full training runs — require GPU
- CI triggers on PR to `main` with `run-ci` label; draft PRs won't run
- All user-facing changes require tests and documentation

## References

- [AGENTS.md](AGENTS.md) — Detailed codebase layout and extension guide
- [CONTRIBUTING.md](CONTRIBUTING.md) — Full code style, commit/PR flow, review process
- [Official docs](https://rlinf.readthedocs.io/)

## Design Overview
请参考 RLinf 的论文 `@bt/docs/ov/RLinf Flexible and Efficient Large-scale Reinforcement Learning via Macro-to-Micro Flow Transformation.pdf`, 参考 RLinf 的官网 https://rlinf.readthedocs.io/en/latest/index.html , 参考 @docs/ 中的各个rst与md文档, 参考 @bt/docs/ 中的各个md文档, 也可参考网上与 RLinf 相关的文章与讨论. 参考官方github(https://github.com/RLinf/RLinf)中的Issues, Commits, Pull requests 和 Disscussions 等等. 重要的是要以深入分析本地的 RLinf 代码库为基础, 以本地代码为准, 对 RLinf 的设计架构与实现进行分析, 分析要紧密结合机器人行业和强化学习, 要考虑到软件工程的方方面面, 要写得比它的论文和官方文档都要好(比如考虑更周到, 细致, 方案更好更有可行性等等), 要图文并茂(比如要有架构图,各种UML图,mermaid等等). 关于RLinf的设计分析写在 @bt/docs/ov/RLinf_Architecture_and_Design2.md 中.

## R1 Pro integration
你是机器人专家. 请参考galaxea (星海图) R1 Pro 机器人及其SDK的官方文档 @https://docs.galaxea-dynamics.com/Guide/R1Pro/, @https://docs.galaxea-dynamics.com/Guide/R1Pro/quick_start/R1Pro_Getting_Started/ , @https://docs.galaxea-dynamics.com/Guide/R1Pro/software_introduction/R1Pro_Software_Guide_ROS2/ , @https://docs.galaxea-dynamics.com/zh/Guide/sdk_change_log/galaxea_changelog/#ros-2-humble , https://docs.galaxea-dynamics.com/Guide/R1Pro/hardware_introduction/R1Pro_Hardware_Introduction/ 和 ROS2 humble 的官方文档 @https://docs.ros.org/en/humble/ 等等, 也可参考搜索到的其它相关的网络文章. 参考后, 请深入分析 @install/ 中 galaxea (星海图) R1 Pro 机器人的 SDK, 分析它的架构是怎么设计的, 模块是怎么划分和分层的, 请用类图表达它的核心概念, 关键模块和核心类; 深入分析它的模块除了在这个文件夹内还分布在该Ubuntu的那些地方, 它还依赖该Ubuntu里面的那些软件, 它是如何与各种硬件交互的, 它是如何调用ROS2的; 深入分析在机器人各个关键场景中它的工作流程,模块或代码间的调用关系,数据流等是怎样的; 深入分析这个 SDK 为什么要这样设计, 有什么开源的软件能够提供与它相似的功能(不一定所有功能,部分也可以), 它与这些开源软件相比的优缺点是什么. 可使用工作流图,序列图等UML图(用 mermaid 画)来分析与说明, 要有对核心代码的解释, 说明要深入浅出, 图文并茂, 有理有据. 这些分析和说明写在 RLinf 的 @bt/docs/rwRL/glx/R1ProSDKAnalysis.md 中.