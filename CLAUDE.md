# CLAUDE.md

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
请参考 RLinf 的论文 `@bt/docs/ov/RLinf Flexible and Efficient Large-scale Reinforcement Learning via Macro-to-Micro Flow Transformation.pdf`, 参考 RLinf 的官网 https://rlinf.readthedocs.io/en/latest/index.html , 参考 @docs/ 中的各个rst与md文档, 参考 @bt/docs/ 中的各个md文档, 也可参考网上与 RLinf 相关的文章与讨论. 参考官方github(https://github.com/RLinf/RLinf)中的Issues, Commits, Pull requests 和 Disscussions 等等. 重要的是要以深入分析本地的 RLinf 代码库(/home/Luogang/SRC/RL/RLinf)为基础, 以本地代码为准, 对 RLinf 的设计架构与实现进行分析, 分析要紧密结合机器人行业和强化学习, 要考虑到软件工程的方方面面, 要写得比它的论文和官方文档都要好(比如考虑更周到, 细致, 方案更好更有可行性等等), 要图文并茂(比如要有架构图,各种UML图,mermaid等等). 关于RLinf的设计分析写在 @bt/docs/ov/RLinf_Architecture_and_Design2.md 中.

## R1 Pro integration
你是机器人专家. 请参考galaxea (星海图) R1 Pro 机器人及其SDK的官方文档 @https://docs.galaxea-dynamics.com/Guide/R1Pro/, @https://docs.galaxea-dynamics.com/Guide/R1Pro/quick_start/R1Pro_Getting_Started/ , @https://docs.galaxea-dynamics.com/Guide/R1Pro/software_introduction/R1Pro_Software_Guide_ROS2/ , @https://docs.galaxea-dynamics.com/zh/Guide/sdk_change_log/galaxea_changelog/#ros-2-humble , https://docs.galaxea-dynamics.com/Guide/R1Pro/hardware_introduction/R1Pro_Hardware_Introduction/ 和 ROS2 humble 的官方文档 @https://docs.ros.org/en/humble/ 等等, 也可参考搜索到的其它相关的网络文章. 参考后, 请深入分析 @install/ 中 galaxea (星海图) R1 Pro 机器人的 SDK, 分析它的架构是怎么设计的, 模块是怎么划分和分层的, 请用类图表达它的核心概念, 关键模块和核心类; 深入分析它的模块除了在这个文件夹内还分布在该Ubuntu的那些地方, 它还依赖该Ubuntu里面的那些软件, 它是如何与各种硬件交互的, 它是如何调用ROS2的; 深入分析在机器人各个关键场景中它的工作流程,模块或代码间的调用关系,数据流等是怎样的; 深入分析这个 SDK 为什么要这样设计, 有什么开源的软件能够提供与它相似的功能(不一定所有功能,部分也可以), 它与这些开源软件相比的优缺点是什么. 可使用工作流图,序列图等UML图(用 mermaid 画)来分析与说明, 要有对核心代码的解释, 说明要深入浅出, 图文并茂, 有理有据. 这些分析和说明写在 RLinf 的 @bt/docs/rwRL/glx/R1ProSDKAnalysis.md 中.