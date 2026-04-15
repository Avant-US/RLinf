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
