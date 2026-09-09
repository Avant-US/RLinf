# CLAUDE.md

This file is the Claude entrypoint for repository guidance.

For the canonical agent instructions, repository orientation, and workflow expectations, see [AGENTS.md](AGENTS.md).

For full contribution flow, code style, and PR process, see [CONTRIBUTING.md](CONTRIBUTING.md).

Keeping the detailed guidance in `AGENTS.md` avoids duplication and prevents the two files from drifting out of sync.
**Single machine:** Install via Docker or `bash requirements/install.sh embodied --model <model> --env <env>` (set `REPO_PATH` and any asset paths). Ray may auto-start; or run `ray start --head`. Use a config with `cluster.num_nodes: 1` (e.g. from `examples/embodiment/config/`). Launch with `bash examples/embodiment/run_embodiment.sh <config_name>` or `python examples/embodiment/train_embodied_agent.py --config-name <config_name>`, and set env vars the example needs (e.g. `MUJOCO_GL=egl`, `ROBOT_PLATFORM`).

**Multiple machines:** On each node, *before* `ray start`: set `export RLINF_NODE_RANK=<0..N-1>` (unique) and optionally `RLINF_COMM_NET_DEVICES`. Head: `ray start --head --port=6379 --node-ip-address=<head_ip>`. Workers: `ray start --address=<head_ip>:6379`. You can use `ray_utils/start_ray.sh`. Set `cluster.num_nodes` to the total; optionally use `node_groups` and `component_placement` (see `rlinf/scheduler/cluster/config.py` and the [heterogeneous cluster tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/guides/hetero.html)). Run the entry script *only on the head*; it attaches to the existing Ray cluster and schedules workers by placement.

---

## Configuration guides

- **Placement and throughput:** Configure `cluster.component_placement` (collocated vs disaggregated vs hybrid, node groups, hardware ranks). See [placement tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/concepts/placement.html) and [execution modes](https://rlinf.readthedocs.io/en/latest/rst_source/concepts/execution_modes.html).
- **OOM:** Tune env (`total_num_envs`, `group_size`), rollout (batch/seq, `gpu_memory_utilization`, `enable_offload`), actor (`micro_batch_size`, `global_batch_size`, `gradient_checkpointing`, `enable_offload`). Example configs in `examples/embodiment/config/`. See [FAQ](https://rlinf.readthedocs.io/en/latest/rst_source/resources/faq.html) for SGLang/memory issues.
- **Multi-node and hetero:** Set `cluster.num_nodes`; set `RLINF_NODE_RANK` (and optionally `RLINF_COMM_NET_DEVICES`) **before** `ray start` on each node—Ray captures env at start time. Optional `node_groups` and `component_placement` in YAML; `env_configs` (e.g. `env_vars`, `python_interpreter_path`) are applied at worker allocation. See [heterogeneous cluster](https://rlinf.readthedocs.io/en/latest/rst_source/guides/hetero.html) and `rlinf/scheduler/cluster/config.py`.

---

## Metrics, checkpoints, and evaluation

- **Metrics:** Runners use `MetricLogger`; set `runner.logger.logger_backends` (e.g. tensorboard, wandb, swanlab). Namespaces include `train/`, `eval/`, `env/`, `rollout/`, `time/`. See [logger tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/guides/logger.html).
- **Checkpoints:** Saved every `runner.save_interval` under `.../checkpoints/global_step_<N>/`. To resume, set `runner.resume_dir` to that path and relaunch; some runners support `resume_dir: auto`. See [checkpoint resume tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/guides/resume.html).
- **Evaluation:** During training, `runner.val_check_interval` triggers validation. Standalone embodied: `bash evaluations/run_eval.sh <benchmark> <config_name>` (configs under `evaluations/<benchmark>/`); see [Evaluation](https://rlinf.readthedocs.io/en/latest/rst_source/evaluations/index.html). Reasoning/LLM: see [LLMEvalKit](https://github.com/RLinf/LLMEvalKit).

---

## When things go wrong

For debugging (breakpoints, rendering/EGL, network, NCCL/CUDA, timeouts), see the [FAQ](https://rlinf.readthedocs.io/en/latest/rst_source/resources/faq.html) in Further reading.

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
- **Config and workers:** Ensure `build_config` / default configs provide the right `model.model_type`, checkpoint paths, and any model-specific options. Actor and rollout workers already branch on `cfg.actor.model.model_type` / `cfg.rollout.model.model_type`; add branches or a factory so your model is instantiated and used. For FSDP+HuggingFace, see the [new model (FSDP) tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/extending/new_model_fsdp.html); for Megatron there is a separate [new model (Megatron) tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/extending/new_model_megatron.html).
- **Install and CI:** If the model needs extra deps or a dedicated venv, add it to `requirements/install.sh` (e.g. `SUPPORTED_MODELS`, and an `install_my_model()` or branch in the model switch). For Docker and e2e: use the skill `.cursor/skills/add-install-docker-ci-e2e` (install script, Dockerfile stage, CI job, e2e config under `tests/e2e_tests/embodied/`).

### New environment

- **Registration:** In `rlinf/envs/__init__.py`, add a member to `SupportedEnvType`: e.g. `MY_ENV = "my_env"`. In `get_env_cls(env_type, env_cfg=None, ...)` add an `elif env_type == SupportedEnvType.MY_ENV:` branch that imports your env class and returns it (lazy import to avoid loading heavy deps at import time). If the env needs a task id (like IsaacLab), use `env_cfg` and document the expected shape.
- **Implementation:** Create `rlinf/envs/my_env/` with at least one module defining a gym-style env (e.g. `gymnasium.Env`): `reset`, `step`, and the usual attributes (`observation_space`, `action_space`). Follow the [new environment tutorial](https://rlinf.readthedocs.io/en/latest/rst_source/extending/new_env.html) for the expected structure (e.g. vectorized `num_envs`, `group_size`, `ret_device`). If your env uses custom action formatting, add a branch in `rlinf/envs/action_utils.py` in `prepare_actions(env_type, ...)` so rollout/workers pass correctly shaped actions.
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
- Tutorials: [placement / cluster / YAML](https://rlinf.readthedocs.io/en/latest/rst_source/concepts/index.html), [hybrid / disaggregated](https://rlinf.readthedocs.io/en/latest/rst_source/concepts/execution_modes.html), [heterogeneous cluster](https://rlinf.readthedocs.io/en/latest/rst_source/guides/hetero.html), [extend (new env/model)](https://rlinf.readthedocs.io/en/latest/rst_source/extending/overview.html), [RL algorithms](https://rlinf.readthedocs.io/en/latest/rst_source/reference/index.html), [logger (metrics)](https://rlinf.readthedocs.io/en/latest/rst_source/guides/logger.html), [checkpoint resume](https://rlinf.readthedocs.io/en/latest/rst_source/guides/resume.html)
- Evaluation: [Evaluation](https://rlinf.readthedocs.io/en/latest/rst_source/evaluations/index.html) · [LLMEvalKit](https://github.com/RLinf/LLMEvalKit)
- [APIs](https://rlinf.readthedocs.io/en/latest/rst_source/reference/api/index.html) (actor, channel, cluster, placement, worker, env, data, …) · [FAQ](https://rlinf.readthedocs.io/en/latest/rst_source/resources/faq.html)


## 一些官方参考资料
参考 RLinf 的官网 https://rlinf.readthedocs.io/en/latest/index.html , 参考 @docs/ 中的各个rst与md文档, 参考 `b/d/` 中的各个pdf和md文档, 也可参考网上与 RLinf 相关的文章与讨论. 参考官方github(https://github.com/RLinf/RLinf)中的Issues, Commits, Pull requests 和 Disscussions 等等. 重要的是要以深入分析该本地的 RLinf 代码库为基础, 以本地代码为准, 对 RLinf 的设计架构与实现进行分析, 分析要紧密结合机器人行业和强化学习, 要考虑到软件工程的方方面面, 要写得比它的论文和官方文档都要好(比如考虑更周到, 细致, 方案更好更有可行性等等), 要图文并茂(比如要有架构图,序列图,数据流图,各种UML图等等). 图表我们用mermaid写, 数学相关的我们用LaTex写. 

## Who are you
你是机器人专家. VLA/VAM/LLM/VLM等领域的AI专家, 你也是强化学习专家. 你曾经在DeepMind, Nvidia, Physical Intelligence, Boston Dynamics, Figure AI 等著名公司任职资深科学家或技术专家岗位. 


# 做分析,解析与写文档的规范
* 图表用mermaid, 数学相关的用LaTex, 必要时可以用py脚本画一些更能帮助读者理解的图片(图片中的文字用英文). 这些脚本和图一般放在与生成的文档同目录的`asset`子文件夹中.
* 如果在公式和内容中用到了数学符号或代号, 请在该公式或内容的附近对该符号给予解释.
* 分析,解析和撰写文档时, 可以参考论文或代码库的官网, 官方文档, GItHUb, 参考github中的issues, 代码和pull requests, 也可参考网上其它可信来源的相关文章, 但参考内容要列出, 所生产的文档中若有与被参考对象相关的内容也要指出内容的出处. 
* 分析要深入仔细, 既要包括纵向分析(算法或方法的由来与演进历史, 以及在该算法或方法的基础上又演进和优化出了些什么解决类似问题的方法, 新老方法各有什么优缺点, 各适合应用到什么场景), 纵向分析(同时期同类算法的对比分析, 不同算法或方法各有什么优缺点, 各适合应用到什么场景), 和 消融分析(算法或方法中哪些点是在benchmark实验或实践中被证明有效的, 哪些点相对来说更有效, 哪些没那么有效).
* 记得深入分析模型或方法的输入,输出,在输入输出间做了些什么处理. 当然, 各组成模块的输入输出以及中间的处理也要分析. 为了训这个模型用了什么数据集和任务, 训出来后能做什么任务, 训练和推理时的输入输出数据格式大概长什么样.
* 系统或程序的设计要包括静态架构(组件图,类图,组件和类的职责与关系等等)和动态架构(数据流图,序列图,工作流图,不同场景下的各组件或类的调用与协调图.如果是算法还会涉及forward阶段的数据流,模型组件间的调用,以及backwawrd阶段的数据流,gradient流,哪些权重冻结哪些会被更新,和模型组件间的调用等等).
* 如果是设计与实施落地相关的文档, 要遵守这些设计原则: 
    - 扩展由于修改, 尽量通过各种设计模式来扩展新模块新功能, 而不是通过修改原来的代码得到新特性; 
    - 尽量复用原有代码, 若不能复用要给出理由; 会随着软硬件环境, 机器人, 底层框架或者云上环境变化而变化的点, 要抽象出来, 作为关键配置点, 最好不同的软硬件环境可对应一个配置文件, 并对配置项和配置文件做详细说明; 
    - 会随着实验的不同, 数据准备, 训练, 评估的不同而变化的点, 也要抽象出来, 作为配置关键点, 最后不同的实验可对应一个配置文件, 并对配置项和配置文件做详细说明.
* 如果是设计与实施落地相关的文档, 要给出测试方案, 验收方案, 以及相关的代码和脚本, 并对方案和代码/脚本的输入输出, 测试前提, 验收条件, 覆盖和没覆盖的分支等相关细节进行详细解释.按设计与实施落地文档执行时,除了要把所有工作完成外, 还要把所有测试和评估都通过了才算成功.
* 代码还是以该代码库的本地代码为准, 但可用参考网上GitHub的issues, commits, pull requests等.
* 解释要深入浅出, 图文并茂, 可以举一些易于理解的例子帮助说明, 对关键的逻辑也要进行深入的代码解读, 要用严谨的科普论文的风格.
<!--
# 编码规范

* 尽量在`.vscode/settings.json`中指定的虚拟环境或指定的python所属的虚拟环境中开发或测试,实在不行才用系统级的python环境.
* 尽量利用`rlinf`已有的功能与模块.
* 要遵守"扩展大于修改的原则".
* 新加的功能和代码要放在 `rlinf/_au/` 里, 该文件夹中的目录要参考 `rlinf/` 的目录设定, 所有定制化的扩展代码都写在 `rlinf/_au/` 中的相应目录里.
* 测试用例和验收脚本要放在 `tests_au/` 里, 该文件夹中的目录要参考  `tests/` 的目录设定, 后续所有定制化的扩展代码的测试代码都写在 `tests_au/` 中的相应目录里.
* 验收脚本也写在 `tests_au/` 中的相应目录里, 验收脚本必须以`accept_`开头, 并且要有调用的sh脚本. 
* 测试和验收数据集用更真实的如下数据集:
   + `/mnt/r/DATA/tst/Galaxea-Open-World-Dataset/Connect_Router_Cables_20250625_002/`
* 配置文件yaml中引用定制化的扩展功能的方法不能用简单的单文件方式, 而是更企业化的模块引入或包引入方式. -->