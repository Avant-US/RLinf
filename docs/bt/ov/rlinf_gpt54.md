# RLinf reinforcement learning framework overview
_Exported on 3/12/2026 at 16:03:35 GMT+8 from Cursor (2.4.32)_

---

**User**

这个网址 @Browser https://rlinf.readthedocs.io/en/latest/下有很多篇RLinf官方的关于RLinf reinforcement learning framework的介绍文章, 结合RLinf相关代码和其他与RLinf有关的文章,论文等,详细解释一下RLinf有什么功能,可以用在什么场景,它的设计思想,它的代码架构,它的这种实现有什么优点和缺点.解释的方式最好能形成知识体系.

---

**Cursor**

我会先交叉核对 RLinf 官方文档、仓库代码结构和公开资料，再把它整理成一套从“定位-能力-架构-设计取舍”展开的知识体系。

我先按文档校验流程把“官网介绍”和“代码事实”对齐，避免后面解释里把功能、配置名或架构说错。

我已经确认了 RLinf 的主线不是“单一算法库”，而是一个把多种 RL 工作流抽象成统一分布式运行时的基础设施。接下来我补充它的设计论文和关键实现细节，这样能把“为什么这么设计”和“这样做的代价”讲清楚。

现在已经能把 RLinf 分成两条主线来看了：一条是 embodied/VLA/机器人，一条是 reasoning/agentic/LLM。为了把“适用场景”和“边界”讲得更完整，我再补一下 examples 和测试覆盖面，确认哪些能力是宣传页写了、哪些是真有工程落地。

## 一句话定位
`RLinf` 不是那种“只实现几个 RL 算法”的轻量库，而更像一个面向 **基础模型后训练** 的 **分布式 RL 基础设施层**。它试图把 `LLM reasoning RL`、`agentic RL`、`VLA/机器人 RL`、`真实世界在线学习` 这些看起来差异很大的工作流，统一到一套 `Cluster + Placement + Worker + Channel + Runner + YAML` 的系统抽象里。

我交叉看了官网文档、仓库代码和公开论文后，结论是：**RLinf 的核心价值不只是“支持哪些算法”，而是把复杂 RL 工作流变成可编排、可扩展、可跨节点调度的系统工程。**

## 1. RLinf 的功能体系
可以把它理解成 6 层能力。

1. **任务层**
- `embodied`：机器人/VLA 强化学习，含仿真与真实机器人。
- `reasoning`：数学推理、VQA、LLM 后训练。
- `agentic`：Search-R1、rStar2、多轮工具调用、coding online RL。
- `sft`：VLA/VLM 的监督微调也纳入同一体系。

代码里直接把任务类型分成了 `embodied`、`reasoning`、`reasoning_eval`、`coding_online_rl`、`sft`，不是文档层面的口号，而是配置和运行时里的一级概念，见 `rlinf/config.py`。

2. **算法层**
- 官网教程列出 `PPO`、`GRPO`、`DAPO`、`REINFORCE++`、`SAC`、`CrossQ`、`RLPD`、`Async PPO`。
- 代码里算法通过 registry 分发，优势函数、loss、reward 都是可插拔的，适合做算法扩展而不是把逻辑写死在 trainer 里。

```1:18:rlinf/algorithms/registry.py
ADV_REGISTRY: dict[str, Callable] = {}

def register_advantage(name: str):
    """Decorator to register advantage & returns function."""
    def decorator(fn):
        @wraps(fn)
        def wrapper(*args, **kwargs):
            return fn(*args, **kwargs)
        ADV_REGISTRY[name.lower()] = wrapper
        return wrapper
    return decorator
```

3. **模型层**
- 推理/agent 方向支持 `Qwen2.5`、`Qwen2.5-VL`、`Qwen3`、`Qwen3-MoE`。
- Embodied 方向支持 `OpenVLA`、`OpenVLA-OFT`、`OpenPI/pi0`、`GR00T`、`Dexbotic`、以及 `MLP/CNN/Flow/CMA` 等策略。
- SFT 也有独立模型类型。

4. **环境层**
- 机器人/仿真环境支持非常广，代码里有 `maniskill`、`libero`、`robotwin`、`isaaclab`、`metaworld`、`behavior`、`calvin`、`robocasa`、`realworld`、`frankasim`、`habitat`，还包括 `opensora_wm`、`wan_wm` 这类 world model 作为“环境”。

5. **系统层**
- `Ray` 管集群和远程 actor。
- `Hydra/OmegaConf` 管配置。
- `FSDP` 和 `Megatron` 两条训练后端。
- `SGLang` 和 `vLLM` 两条 rollout/inference 后端。
- 支持 `collocated`、`disaggregated`、`hybrid`、`AUTO` 这几种执行模式。

6. **工程层**
- 有 examples、e2e tests、docs、Docker、install script、PyPI 包。
- 从工程成熟度看，它是“能跑论文 recipe 的系统”，不只是研究代码仓。

## 2. 它适合什么场景
最典型的是下面几类。

- **VLA/机器人后训练**  
  这是 RLinf 最强的一条线。适合把 `OpenVLA/OpenVLA-OFT/pi0/GR00T` 这类模型放到 `ManiSkill/LIBERO/RoboTwin/IsaacLab/MetaWorld/CALVIN` 里做 PPO/GRPO/SAC 等训练。`RLinf-VLA` 论文就是这条主线。

- **LLM reasoning RL**  
  比如数学推理、VQA、长上下文 GRPO/PPO。官网和代码都把这作为第一类场景，而不是附带功能。

- **Agentic RL / Tool-use / Search / Coding**  
  Search-R1、rStar2、工具调用、多轮 agent、在线 coding RL。也就是说 RLinf 不只针对“生成一个答案”的 rollout，还能针对“调用工具、检索、交互”的轨迹做训练。

- **真实机器人在线学习**  
  `RLinf-USER` 这条线强调物理世界训练，把机器人当成一等资源来调度，不只是把真实环境当成一个慢版 simulator。

- **World model + RL**  
  RLinf 已经把 `OpenSora/Wan` 这类视频世界模型纳入环境接口，`WoVR` 则是更进一步的 world-model-as-simulator 思路。

- **多机多卡、异构集群、需要系统调度的实验**  
  如果你要跨节点训练，或者 actor / rollout / inference / reward / env 的资源需求不一样，RLinf 会比“单 trainer + 几个 callback”类框架更合适。

## 3. 它的设计思想
### 3.1 核心思想：M2Flow
RLinf 论文把自己的核心概念叫 **Macro-to-Micro Flow Transformation**。

我的理解可以概括成一句话：

- **Macro flow**：研究者看到的是高层 RL 逻辑，比如“采样 -> 算奖励 -> 算优势 -> 更新策略”。
- **Micro flow**：系统真正执行的是更细的 GPU/节点/队列/通信/流水线计划。

也就是说，RLinf 试图把“算法逻辑”和“物理执行方式”解耦。用户写的是逻辑工作流，系统再决定这些 component 怎样放到 GPU 上、是否流水化、是否异步、是否切成多组 worker 去跑。

### 3.2 设计不是围绕算法，而是围绕“组件”
它不是以单个 trainer 为中心，而是以组件为中心：

- `actor`
- `rollout`
- `inference`
- `reward`
- `env`
- `scheduler`

入口脚本几乎都遵循同一个模式：先验配置，建 `Cluster`，算 `Placement`，再 launch 各类 `WorkerGroup`，最后交给 `Runner` 驱动。

```38:75:examples/embodiment/train_embodied_agent.py
cluster = Cluster(cluster_cfg=cfg.cluster)
component_placement = HybridComponentPlacement(cfg, cluster)

# Create actor worker group
actor_placement = component_placement.get_strategy("actor")
# ...

# Create rollout worker group
rollout_placement = component_placement.get_strategy("rollout")
# ...

# Create env worker group
env_placement = component_placement.get_strategy("env")
# ...

runner = EmbodiedRunner(
    cfg=cfg,
    actor=actor_group,
    rollout=rollout_group,
    env=env_group,
)

runner.init_workers()
runner.run()
```

### 3.3 “灵活性优先”是第一原则
主论文的一个很明确观点是：**RL 系统的瓶颈很多时候不是单点算力，而是缺少足够灵活的执行组织方式。**

所以 RLinf 很强调：
- placement 可配置
- worker 组合可替换
- backend 可替换
- reward/adv/loss 可注册
- 通信可以同步也可以异步
- 同一套系统既能做 LLM RL，也能做 embodied RL

### 3.4 把“异构性”当常态而不是异常
这点很关键。很多框架默认“所有 GPU 都一样、所有 worker 结构相同”。RLinf 不是，它把下面这些都当成正常需求：
- actor 和 rollout 在不同 GPU
- inference 可能独立出来
- env 既可能是 CPU simulator，也可能是 GPU simulator，也可能是真实机器人
- 不同 node group 可以有不同 env config / software / hardware

这也是它比常规 trainer 更像“系统软件”的原因。

## 4. 它的代码架构
可以把仓库理解成下面这个知识图谱。

- `rlinf/config.py`
  统一配置校验中心。任务类型、模型类型、后端、batch 约束、placement 约束很多都在这里收口。

- `rlinf/scheduler/`
  系统核心。包括 `cluster`、`worker`、`worker_group`、`channel`、`placement`、`manager`、`dynamic_scheduler`。这是 RLinf 的“内核”。

- `rlinf/runners/`
  顶层控制循环。不同任务有不同 runner，比如 `EmbodiedRunner`、`ReasoningRunner`、`AgentRunner`、`coding_online_rl_runner`、`sft_runner`。

- `rlinf/workers/`
  真正执行工作的远程 actor。按职责分成 actor、rollout、env、reward、inference、tool 等。

- `rlinf/models/`
  模型接入层，负责把具体模型接到统一 actor/rollout 接口上。

- `rlinf/envs/`
  环境抽象层，负责把不同 simulator/real-world/world model 包装成统一接口。

- `rlinf/algorithms/`
  advantage、loss、reward registry 与实现。

- `examples/`
  各类 recipe。这个目录几乎就是“官方支持面”的最好索引。

- `tests/e2e_tests/`
  很重要。这里有 embodied、reasoning、agent、coding_online_rl、dynamic_scheduler、auto_placement、sft 等配置，说明这些不是只在 PPT 里存在。

### 4.1 运行时主链路
Embodied 任务主链非常清晰：`env -> rollout -> actor -> train`。

```165:193:rlinf/runners/embodied_runner.py
with self.timer("step"):
    with self.timer("sync_weights"):
        if _step % self.weight_sync_interval == 0:
            self.update_rollout_weights()
    with self.timer("generate_rollouts"):
        env_handle: Handle = self.env.interact(
            input_channel=self.rollout_channel,
            output_channel=self.env_channel,
        )
        rollout_handle: Handle = self.rollout.generate(
            input_channel=self.env_channel,
            output_channel=self.rollout_channel,
            actor_channel=self.actor_channel,
        )
        self.actor.recv_rollout_trajectories(
            input_channel=self.actor_channel
        ).wait()
        rollout_handle.wait()

    with self.timer("cal_adv_and_returns"):
        actor_rollout_metrics = (
            self.actor.compute_advantages_and_returns().wait()
        )
```

Reasoning 任务则更像 LLM RL 生产线：`data -> rollout -> reward -> inference(optional) -> actor training`。

```381:418:rlinf/runners/reasoning_runner.py
with self.timer("step"):
    with self.timer("prepare_data"):
        self._put_batch(batch)

    with self.timer("sync_weights"):
        self._sync_weights()

    if self.scheduler is not None:
        scheduler_handle = self.scheduler.schedule()

    rollout_handle: Handle = self.rollout.rollout(
        input_channel=self.dataloader_channel,
        output_channel=self.rollout_channel,
    )

    reward_handle: Handle = self.reward.compute_rewards(
        input_channel=self.rollout_channel,
        output_channel=self.reward_channel,
    )

    if self.recompute_logprobs:
        infer_handle: Handle = self.inference.run_inference(
            input_channel=self.reward_channel,
            output_channel=self.inference_channel,
            compute_ref_logprobs=self.compute_ref_logprobs,
        )
        inference_channel = self.inference_channel
    else:
        infer_handle = None
        inference_channel = self.reward_channel

    actor_handle: Handle = self.actor.run_training(
        input_channel=inference_channel,
    )
```

### 4.2 它为什么能统一这么多任务
关键是三件事：

- `Worker`
  把远程执行、分布式环境变量、通信能力统一抽象掉。

- `Channel`
  把组件间交互做成队列/路由/加权批处理接口，适合流水线和异步执行。

- `Placement`
  决定这些组件怎么映射到 GPU/节点上。

从系统设计角度看，这比“一个 trainer 管所有事情”更适合复杂 RL。

## 5. 这种实现的优点
- **适用面很广**：一个框架覆盖 reasoning、agentic、embodied、real-world、world-model。
- **扩展性强**：模型、环境、loss、advantage、reward 都是 registry/插件式。
- **资源利用率高**：文档和论文都强调通过 hybrid/disaggregated/dynamic scheduling 提升吞吐，尤其 embodied RL。
- **适合大规模**：Ray + Megatron/FSDP + SGLang/vLLM 的组合，明显面向多机多卡。
- **工程可复现性较好**：examples、docs、e2e tests、install/docker 都比较全。
- **对异构系统友好**：不仅考虑 GPU，还考虑 simulator、real robot、node group 差异。
- **有“系统研究”味道**：不是只拼模型效果，还把执行计划、流水线、调度、通信当成优化对象。

## 6. 它的缺点和代价
- **学习曲线高**：要理解 `Ray + Hydra + Placement + WorkerGroup + Channel + Runner`，门槛明显高于 `trl`、`verl recipe`、`cleanrl` 这类框架。
- **配置复杂**：灵活性的代价就是 YAML 选项很多，batch/parallelism/placement 约束也多。
- **运行时复杂度高**：异步、分布式、动态调度带来更强能力，也带来更难 debug 的问题。
- **更依赖基础设施**：这不是“单机一张卡就能舒服开发”的典型框架，小实验会显得过重。
- **后端差异会泄漏出来**：虽然抽象统一，但 FSDP/Megatron、SGLang/vLLM 的能力和调参方式并不完全等价。
- **部分错误更晚暴露**：大量 registry + lazy import + config-driven 设计，让一些问题变成运行时错误，而不是静态结构错误。
- **依赖矩阵重**：机器人环境、world model、Megatron、仿真器，各自依赖都不轻。

## 7. 适不适合你
适合用 RLinf 的情况：
- 你做的是 **基础模型后训练**，不是经典 tabular/small-scale RL。
- 你需要 **多组件流水线**，比如 actor/rollout/reward/inference/env 分开。
- 你要做 **VLA/机器人 RL** 或 **agentic RL**。
- 你关心 **系统吞吐、资源调度、跨节点扩展**。
- 你希望在一个统一框架里复现和扩展多种算法/模型/环境。

不太适合的情况：
- 你只想快速验证一个小算法想法。
- 你主要是单机小规模实验。
- 你不想处理分布式系统复杂性。
- 你更需要的是“最小 trainer”，而不是“可编排基础设施”。

## 结论
如果把 RL 领域的框架做一个粗分类：

- `Stable-Baselines/CleanRL` 更像算法教学和轻量实验框架。
- `TRL/VeRL/AReaL` 更偏 LLM 后训练框架。
- **RLinf** 则更像一个面向 **Embodied + Agentic + Reasoning** 的 **统一 RL 系统平台**。

它最有价值的地方，在我看来不是“又支持了一个算法”，而是它试图回答一个更大的问题：**当 RL 工作流越来越像一个多组件、异构、动态的系统工程时，应该怎么把逻辑抽象和物理执行分开，仍然保持高吞吐和可扩展性。**

我核对的官网主线能力与代码实现基本一致，尤其是这几个判断比较稳：
- RLinf 的核心不是单算法，而是分布式基础设施。
- 它的两条主干是 `reasoning/agentic` 和 `embodied/real-world`。
- `hybrid placement + channel + worker-based orchestration` 是它区别于常见 RL 框架的关键。

如果你愿意，我下一步可以继续给你做两种延伸之一：
1. 按“新手到专家”的顺序，整理一版 **RLinf 学习路线图**。
2. 按仓库目录，详细讲一遍 **`scheduler / runners / workers / envs / models` 的源码阅读顺序**。

---

**User**

详细讲一遍 scheduler / runners / workers / envs / models 的源码阅读顺序

---

**Cursor**

我按“先建立运行时心智模型，再进入各层实现细节”的顺序补齐关键源码入口，这样你后面自己读代码时不容易迷路。

如果你的目标是“真正读懂 RLinf 是怎么跑起来的”，我建议不要按目录平铺去读，而要按 **调用链** 去读。最有效的顺序是：

**先看入口脚本怎么拼系统，再看 scheduler 提供了哪些运行时抽象，再看 runner 如何驱动一轮训练，最后才钻进 workers / envs / models。**

下面我给你一套我认为最稳的源码阅读顺序。

## 先建立总图
先把 RLinf 想成一句话：

- `examples/*.py` 负责“搭系统”
- `scheduler/` 负责“提供分布式运行时”
- `runners/` 负责“定义训练循环”
- `workers/` 负责“干具体活”
- `envs/` 和 `models/` 负责“领域接入”

最典型的入口脚本已经把这件事写得很清楚了：

```38:75:examples/embodiment/train_embodied_agent.py
cluster = Cluster(cluster_cfg=cfg.cluster)
component_placement = HybridComponentPlacement(cfg, cluster)

# Create actor worker group
actor_placement = component_placement.get_strategy("actor")
# ...

# Create rollout worker group
rollout_placement = component_placement.get_strategy("rollout")
# ...

# Create env worker group
env_placement = component_placement.get_strategy("env")
# ...

runner = EmbodiedRunner(
    cfg=cfg,
    actor=actor_group,
    rollout=rollout_group,
    env=env_group,
)

runner.init_workers()
runner.run()
```

所以你读代码时，脑子里一直盯着这条主线就行：

1. 配置怎么校验  
2. 组件怎么放到集群上  
3. worker group 怎么创建  
4. runner 怎样调这些 group  
5. group 背后各 worker 真正做什么

---

## 第一阶段：先读入口，不要急着进底层
### 1. 先读两个入口脚本
先读这两个文件：

- `examples/embodiment/train_embodied_agent.py`
- `examples/reasoning/main_grpo.py`

这两个入口基本代表了 RLinf 的两条主干：`embodied` 和 `reasoning/agentic`。

你重点看三件事：
- `validate_cfg(cfg)` 做了什么
- `Cluster + Placement` 怎么创建
- 哪些 worker group 被 launch 了，最后交给哪个 runner

Reasoning 入口会比 embodied 多出 `reward`、`inference`、`scheduler` 这些组件：

```44:115:examples/reasoning/main_grpo.py
cluster = Cluster(cluster_cfg=cfg.cluster)
component_placement = ModelParallelComponentPlacement(cfg, cluster)

rollout_worker_cls = get_rollout_backend_worker(cfg)
rollout_group = rollout_worker_cls.create_group(cfg, component_placement).launch(...)

inference_worker_cls = get_inference_backend_worker(cfg)
# ...

reward_group = RewardWorker.create_group(cfg, component_placement).launch(...)
actor_group = actor_worker_cls.create_group(cfg, component_placement).launch(...)

runner = ReasoningRunner(
    cfg=cfg,
    placement=component_placement,
    train_dataset=train_ds,
    val_dataset=val_ds,
    rollout=rollout_group,
    inference=inference_group,
    actor=actor_group,
    reward=reward_group,
    scheduler=scheduler,
)
```

这一步的目标不是看实现细节，而是建立“**RLinf 是怎么装配系统的**”这个印象。

---

## 第二阶段：读 `config + placement`，理解“为什么这么装配”
很多人一上来就钻 `worker.py`，会很痛苦。更好的顺序是先读：

- `rlinf/config.py`
- `rlinf/utils/placement.py`
- 再补 `rlinf/scheduler/placement/` 里的基础 placement 抽象

### 2. 先读 `rlinf/config.py`
这个文件是整个框架的“约束中心”。你要重点看：

- `SupportedModel`
- `SUPPORTED_TASK_TYPE`
- `SUPPORTED_TRAINING_BACKENDS`
- `validate_embodied_cfg`
- `validate_reasoning_cfg`
- `validate_coding_online_rl_cfg`
- `validate_cfg`

它回答的是：
- 任务类型有哪些
- 哪些模型属于 embodied，哪些属于 reasoning
- 什么时候必须有 inference
- FSDP 和 Megatron 分别要求什么
- env 数量、batch size、pipeline stage 等约束如何检查

你读完 `config.py` 后，会对 RLinf 的产品边界有非常清楚的认识。

### 3. 再读 `rlinf/utils/placement.py`
这个文件是 RLinf 的系统味最浓的地方之一。

核心要看：
- `PlacementMode`
- `HybridComponentPlacement`
- `ModelParallelComponentPlacement`

你要弄懂的不是每个细节，而是这三个概念：

- `COLLOCATED`：多个组件共用 GPU
- `DISAGGREGATED`：组件分离到不同 GPU / 节点
- `HYBRID`：针对 embodied 场景做细粒度组合
- `AUTO`：在 disaggregated 基础上再加自动调度/自动规划

这一步的目标是回答：

- 为什么 embodied 用 `HybridComponentPlacement`
- 为什么 reasoning 常用 `ModelParallelComponentPlacement`
- 为什么 RLinf 一直强调 placement，而不是一个固定 trainer

---

## 第三阶段：系统核心，按 `Cluster -> Worker -> WorkerGroup -> Channel`
这一层才是 `scheduler/` 的阅读重点。

### 4. 先看 `rlinf/scheduler/__init__.py`
这个文件本身很薄，但它能帮你快速看清 scheduler 这一层公开了哪些核心抽象：

```15:27:rlinf/scheduler/__init__.py
from .channel import Channel, ChannelWorker, WeightedItem
from .cluster import Cluster, ClusterConfig
from .collective import CollectiveGroupOptions
from .hardware import AcceleratorType, AcceleratorUtil, FrankaHWInfo, HardwareInfo
from .manager import WorkerInfo
from .placement import (
    ComponentPlacement,
    FlexiblePlacementStrategy,
    NodePlacementStrategy,
    PackedPlacementStrategy,
    PlacementStrategy,
)
from .worker import Worker, WorkerAddress, WorkerGroupFuncResult
```

你可以把这里当成 RLinf 运行时的“公开 API 索引”。

### 5. 读 `rlinf/scheduler/cluster/cluster.py`
这是集群和 Ray 的入口。重点看：

- `Cluster.__init__`
- `_init_and_launch_managers`
- `ray.init(...)`
- 环境变量注入
- 多节点发现与 namespace 管理

它回答的问题是：
- RLinf 怎么接管 Ray
- 为什么要求某些环境变量在 `ray start` 之前设置
- 多节点信息怎么进入 RLinf 运行时
- 扩展模块 `RLINF_EXT_MODULE` 怎么挂进去

这一层先建立“**Cluster 是全局资源视图**”这个概念。

### 6. 读 `rlinf/scheduler/worker/worker.py`
这是最重要的基类之一。不要试图一次读完全部，第一遍只抓住：

- `Worker` 是一个 Ray actor 封装
- 自动设置了分布式训练相关环境变量
- 提供 `send / recv / broadcast` 等通信原语
- 提供 timer / logging / failure capture 等公共能力

你要把 `Worker` 理解成：

- “远程进程抽象”
- “分布式通信抽象”
- “训练 worker 的公共运行时”

### 7. 再读 `rlinf/scheduler/worker/worker_group.py`
这是从“单 worker”升级到“一个组件的一组 worker”。

重点看：
- `create_group`
- `launch`
- `_create_workers`
- `execute_on`
- `from_group_name`

最关键的是理解：**RLinf 里你平时操作的不是单个 worker，而是一个 `WorkerGroup`。**

```144:194:rlinf/scheduler/worker/worker_group.py
def launch(
    self,
    cluster: Cluster,
    placement_strategy: Optional[PlacementStrategy] = None,
    name: Optional[str] = None,
    max_concurrency: Optional[int] = None,
    isolate_gpu: bool = True,
    catch_system_failure: Optional[bool] = None,
):
    self._cluster = cluster
    self._placement_strategy = placement_strategy
    # ...
    self._create_workers()
    self._attach_cls_func()
    self._is_ready()
    return self
```

读这层时你脑子里应该形成一句话：

- `Worker` 是执行单元
- `WorkerGroup` 是组件实例

### 8. 然后读 `rlinf/scheduler/channel/channel.py`
`Channel` 是 RLinf 跟很多 trainer 框架很不一样的地方。

重点看：
- `Channel.create`
- `Channel.connect`
- `put/get`
- `distributed=True` 的含义
- key-based routing
- weighted batch

它不是简单 queue，而是组件之间的数据平面。  
你读 runners 和 workers 时，如果看到很多 `input_channel` / `output_channel`，就回想这里。

一句话理解：

- `Worker/WorkerGroup` 负责“谁在算”
- `Channel` 负责“数据怎么流”

---

## 第四阶段：读 `runners/`，理解“训练循环怎么编排”
这一层是源码阅读的分水岭。读完 runner，你就不再把 RLinf 看成一堆分散的 worker 了。

### 9. 第一优先：`rlinf/runners/embodied_runner.py`
先读 embodied runner，因为链路更直观。

重点看：
- `init_workers`
- `update_rollout_weights`
- `evaluate`
- `run`

最重要的是 `run()`。你要读出它的 4 步节奏：

1. sync 权重  
2. env 和 rollout 联合生成轨迹  
3. actor 算 advantages/returns  
4. actor 更新参数  

### 10. 第二优先：`rlinf/runners/reasoning_runner.py`
再读 reasoning runner，对比 embodied runner。你会马上发现：

- embodied 是 `env -> rollout -> actor`
- reasoning 是 `data -> rollout -> reward -> inference(optional) -> actor`

这一步能让你看懂 RLinf 其实不是“一个训练循环适配所有任务”，而是 **每类任务有自己的 orchestration 模板**。

### 11. 第三优先：`rlinf/runners/agent_runner.py`
这个文件很值得读，因为它揭示了 RLinf 对 agentic RL 的建模方式：

- agent 不是另起炉灶
- 而是在 `ReasoningRunner` 之上继续叠加 `tool workers + agent loop`

```42:55:rlinf/runners/agent_runner.py
class AgentRunner(ReasoningRunner):
    """Runner for agent task RL training."""
```

这句继承关系本身就非常重要。它说明 RLinf 认为：

- `agent RL = reasoning RL + 工具/会话/agent loop 编排`

然后你再看它创建的 channel：
- `GenerateInput`
- `GenerateOutput`
- `Tool-*`
- `ToolOutput`

这会帮助你理解 agent 场景为什么需要更多 channel 和 serverless rollout。

### 12. 最后补 `coding_online_rl_runner.py` 和 `sft_runner.py`
这两个不要最开始读。

建议顺序：
- 先读完 embodied / reasoning / agent
- 再回头看 `coding_online_rl_runner.py`
- 最后看 `sft_runner.py`

因为它们更多是在已有模式上的变体，不适合拿来建立第一印象。

---

## 第五阶段：读 `workers/`，但要“按一条场景链路”读
这里最容易迷失。我的建议是：**不要横向扫整个 `workers/` 目录。**

### 13. 先读 selector / factory 文件
先看这几个很短的入口文件：

- `rlinf/workers/actor/__init__.py`
- `rlinf/workers/rollout/utils.py`
- `rlinf/workers/inference/utils.py`

它们告诉你“根据配置，最终选中了哪个具体 worker 类”。

比如 actor：

```20:30:rlinf/workers/actor/__init__.py
def get_actor_worker(cfg: DictConfig) -> Worker:
    if cfg.actor.training_backend == "fsdp":
        from .fsdp_actor_worker import FSDPActor
        return FSDPActor
    elif cfg.actor.training_backend == "megatron":
        from .megatron_actor_worker import MegatronActor
        return MegatronActor
```

这个顺序很重要，因为你必须先知道“当前配置走哪条实现线”，再去读具体 worker。

### 14. Embodied 路线这样读
如果你先学 embodied，建议顺序：

- `rlinf/workers/env/env_worker.py`
- `rlinf/workers/rollout/hf/huggingface_worker.py`
- `rlinf/workers/actor/fsdp_actor_worker.py`
- 如果是 SAC/async，再补对应专门 worker

先看 `EnvWorker` 很有帮助，因为 embodied 的数据从 env 出来。

```31:49:rlinf/workers/env/env_worker.py
class EnvWorker(Worker):
    def __init__(self, cfg: DictConfig):
        Worker.__init__(self)
        self.cfg = cfg
        # ...
        self._component_placement = HybridComponentPlacement(cfg, Cluster())
        self.stage_num = self.cfg.rollout.pipeline_stage_num
```

继续看它的 `init_worker`，你会明白：
- env 类怎么从 `get_env_cls` 选出来
- stage/pipeline 怎么切
- env 和 rollout 的 rank 映射怎么建
- auto reset / eval env / video wrapper / data collection 怎么挂

这时再去读 rollout worker 和 actor worker，就会自然很多，因为你已经知道它们在接谁、发谁。

### 15. Reasoning 路线这样读
如果你想主攻 LLM/agent 方向，建议顺序：

- `rollout/utils.py` 先确认是 `SGLang` 还是 `vLLM`
- 读对应 rollout worker
- `reward/reward_worker.py`
- `actor/fsdp_actor_worker.py` 或 `actor/megatron_actor_worker.py`
- `inference/*_inference_worker.py`

这条线里，一个非常关键的问题是：

- rollout 负责生成什么
- reward 读到什么结构
- inference 为什么有时单独存在
- actor 最终吃到的 batch 是什么

不要急着看训练细节，先看这些 worker 的 `init_worker`、`run_training`、`rollout`、`compute_rewards`、`sync_model_*` 方法。

### 16. Agent 路线最后读
建议顺序：

- `rlinf/workers/agent/agent_loop.py`
- `rlinf/workers/agent/tool_worker.py`
- 再回头配合 `AgentRunner` 一起看

agent 这一层本质上是“在 rollout 外面再套一层多轮交互控制器”，如果你没读过 reasoning runner，会很难理解它。

---

## 第六阶段：读 `envs/`，从注册表到一个具体环境
### 17. 先读 `rlinf/envs/__init__.py`
这里是所有环境的注册入口。一定先读它，因为它回答两个问题：

- RLinf 支持哪些 env type
- 一个 env type 最终会返回哪个类

### 18. 再读 `envs/action_utils.py`
这个文件经常被忽略，但很重要。

因为 embodied 任务里，模型输出的 action 表示和环境真正要的 action 格式未必一致。  
如果你不读这里，很容易以为 `model -> env` 是直接对接的。

### 19. 最后挑一个具体环境精读
推荐你只选一个先深入，比如：
- `rlinf/envs/maniskill/maniskill_env.py`
- 或 `rlinf/envs/libero/libero_env.py`

读的时候重点看：
- `reset`
- `step`
- observation/action space
- batch / vectorized env 逻辑
- 与 `EnvWorker` 的接口契约

这样你会明白：`EnvWorker` 是运行时壳子，具体环境逻辑还是在 `envs/<name>/` 里。

---

## 第七阶段：读 `models/`，从统一入口到一个具体模型
### 20. 先读 `rlinf/models/__init__.py`
这个文件是模型选择入口。它告诉你：

- 配置里的 `model_type` 最终映射到哪个 `get_model`
- 什么时候加 LoRA
- 模型最后如何搬到设备上

### 21. 先读 `rlinf/models/embodiment/base_policy.py`
这个文件很重要，因为它定义了 embodied policy 的最小接口：

```19:27:rlinf/models/embodiment/base_policy.py
class ForwardType(Enum):
    DEFAULT = "default"
    SFT = "sft"
    SAC = "sac"
    SAC_Q = "sac_q"
    CROSSQ = "crossq"
    CROSSQ_Q = "crossq_q"
```

```28:67:rlinf/models/embodiment/base_policy.py
class BasePolicy(ABC):
    def forward(self, forward_type=ForwardType.DEFAULT, **kwargs):
        if forward_type == ForwardType.DEFAULT:
            return self.default_forward(**kwargs)
        else:
            raise NotImplementedError

    @abstractmethod
    def default_forward(self, **kwargs): ...

    @abstractmethod
    def predict_action_batch(self, **kwargs): ...
```

这能帮你理解：
- 一个 embodied 模型在 RLinf 里必须提供什么能力
- 为什么同一个模型能支持 PPO / SAC / CrossQ / SFT 等不同前向模式

### 22. 最后只挑一个具体模型精读
推荐只选你最关心的那一个：
- `openvla`
- `openvla_oft`
- `openpi`
- `gr00t`
- 或者更简单的 `mlp_policy`

读法是：
1. 先看 `get_model`
2. 再看核心 policy class
3. 再回头看 actor worker 是怎样调用它的

不要一上来同时看 5 个模型，不然很容易把“框架接口”和“模型实现细节”混在一起。

---

## 第八阶段：最后再回头读算法层
严格说你这次问的是 `scheduler / runners / workers / envs / models`，但我建议最后补一遍：

- `rlinf/algorithms/registry.py`
- `rlinf/algorithms/advantages.py`
- `rlinf/algorithms/losses.py`
- `rlinf/algorithms/rewards/`

原因很简单：

**只有当你已经知道 batch 是怎么流动的、worker 在什么时候调用算法，算法层代码才真正有上下文。**

---

## 一条最推荐的“源码阅读路线”
如果你想要一条可直接照着走的顺序，我会推荐这条：

1. `examples/embodiment/train_embodied_agent.py`
2. `examples/reasoning/main_grpo.py`
3. `rlinf/config.py`
4. `rlinf/utils/placement.py`
5. `rlinf/scheduler/__init__.py`
6. `rlinf/scheduler/cluster/cluster.py`
7. `rlinf/scheduler/worker/worker.py`
8. `rlinf/scheduler/worker/worker_group.py`
9. `rlinf/scheduler/channel/channel.py`
10. `rlinf/runners/embodied_runner.py`
11. `rlinf/runners/reasoning_runner.py`
12. `rlinf/runners/agent_runner.py`
13. `rlinf/workers/actor/__init__.py`
14. `rlinf/workers/rollout/utils.py`
15. `rlinf/workers/inference/utils.py`
16. `rlinf/workers/env/env_worker.py`
17. 你当前任务对应的具体 actor / rollout / inference worker
18. `rlinf/envs/__init__.py`
19. `rlinf/envs/action_utils.py`
20. 一个具体 env
21. `rlinf/models/__init__.py`
22. `rlinf/models/embodiment/base_policy.py`
23. 一个具体 model
24. 最后再补 `rlinf/algorithms/*`

---

## 每一层你要回答的核心问题
为了避免“读完了但没形成知识”，建议每读一层都强迫自己回答下面的问题。

### `scheduler/`
- 资源是谁管理的？
- worker 是怎么被启动到 Ray 上的？
- worker group 和单 worker 的关系是什么？
- 组件间靠什么传数据？
- placement 决定了哪些运行时行为？

### `runners/`
- 一轮 step 的顺序是什么？
- 哪些组件同步，哪些异步？
- 哪些 channel 在 step 中传递数据？
- 这个任务为什么要有这些组件？

### `workers/`
- 这个 worker 的输入输出是什么？
- 它依赖哪几个 channel / peer group？
- 它最关键的方法是哪几个？
- 它到底是“计算型 worker”还是“编排型 worker”？

### `envs/`
- 具体 env 类和 `EnvWorker` 的职责边界是什么？
- action 适配发生在哪里？
- vectorization / reset / rollout chunking 怎么做？

### `models/`
- RLinf 对模型定义了什么最小接口？
- actor worker 实际上调用模型的哪个 forward？
- rollout 和 actor 是否用的是同一套模型权重/表示？

---

## 两个常见误区
- **误区 1：从具体模型开始读。**  
  这样容易把注意力陷进 `HF module`、`vision encoder`、`action head`，但你还不知道整个系统怎么调度它。

- **误区 2：从 `worker.py` 一口气读到底。**  
  `worker.py` 很长，而且很多细节只有放回 `runner + placement + concrete worker` 的上下文里才有意义。

---

## 你可以这样分 3 天读
### 第一天
- 入口脚本
- `config.py`
- `utils/placement.py`
- `cluster.py`
- `worker.py`
- `worker_group.py`

目标：建立运行时总图。

### 第二天
- `channel.py`
- `embodied_runner.py`
- `reasoning_runner.py`
- `agent_runner.py`

目标：搞懂一轮训练怎么编排。

### 第三天
- `workers/*`
- `envs/*`
- `models/*`
- `algorithms/*`

目标：把具体组件实现填进总图里。

---

如果你愿意，我下一步可以直接继续帮你做一版更细的东西：

- 按这条顺序，给你列一份 **“每个文件该重点看哪些类、哪些函数、为什么”** 的精读清单。
- 或者我直接带你从 `examples/reasoning/main_grpo.py` 开始，做一次 **逐文件源码导读**。

