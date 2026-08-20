
# 1. Catkin 是什么？（`franka.rst` 第 207 行）

### 文档原文语境

`docs/source-zh/rst_source/examples/embodied/franka.rst` 在说明 `SKIP_ROS=1` 跳过安装脚本时，要求用户自行 source：

- ROS 系统层：`/opt/ros/noetic/setup.bash`
- 用户编译层：`<your_catkin_ws>/devel/setup.bash`（franka_ros、serl_franka_controllers）

### 定义

**Catkin** 是 **ROS 1**（Franka 真机流程使用 **ROS Noetic**）的构建与工作空间管理系统。`<your_catkin_ws>` 即 **Catkin 工作空间**，用于编译、管理 `franka_ros`、`serl_franka_controllers` 等 ROS 包。

### RLinf 自动安装时的路径

`requirements/install.sh` 中 `install_franka_env()` 会创建：

```
$VENV_DIR/franka_catkin_ws/
├── src/
│   ├── franka_ros/
│   └── serl_franka_controllers/
├── build/
└── devel/
    └── setup.bash    # 文档中的 <your_catkin_ws>/devel/setup.bash
```

并在 `$VENV_DIR/bin/activate` 末尾追加：

```bash
source /opt/ros/noetic/setup.bash
source $ROS_CATKIN_PATH/devel/setup.bash
```

因此使用安装脚本或 Docker 时，`source .venv/bin/activate` 已间接完成 catkin workspace 的加载；手动安装（`SKIP_ROS=1`）则需自行指定 `<your_catkin_ws>`。

相关脚本注释见 `ray_utils/realworld/setup_before_ray.sh`：

```bash
# source <your_catkin_ws>/devel/setup.bash
```

### 为什么必须 source `devel/setup.bash`？

`FrankaController` 运行时 lazy-import ROS 包（`rlinf/envs/realworld/franka/franka_controller.py`）：

- `geometry_msgs`、`rospy` 等 → 来自 `/opt/ros/noetic/setup.bash`
- `franka_msgs` → 来自 `franka_ros`
- `serl_franka_controllers.msg` → 来自 `serl_franka_controllers`

后两者需在工作空间内 `catkin_make` 编译，并通过 `devel/setup.bash` 写入 `ROS_PACKAGE_PATH`、`PYTHONPATH`、`LD_LIBRARY_PATH` 等环境变量。

Ray 在 `ray start` 时冻结当前 shell 的环境变量；控制节点启动 Ray 前未 source 上述脚本，会导致 `FrankaController` import 失败。

### 两层 ROS 环境对照

| 层级 | setup 脚本 | 提供内容 |
|------|-----------|---------|
| ROS 系统层 | `/opt/ros/noetic/setup.bash` | ROS 核心、`rospy`、`geometry_msgs` 等 |
| 用户编译层 | `<your_catkin_ws>/devel/setup.bash` | `franka_ros`、`serl_franka_controllers` |

---

# 2. Peg-insertion：为什么要获取目标末端位姿？怎么获取？

### 文档位置

`docs/source-zh/rst_source/examples/embodied/franka.rst` 第 304–322 行（运行 → 前置准备 → 获取任务的目标位姿）。

### 为什么要获取？

真机 Peg-insertion 的孔/槽位置因实验台布局而异，无法写死统一坐标。配置项 `target_ee_pose` 表示 **插销成功插入时末端执行器（TCP）应在的位姿**，是任务的几何锚点。

代码中的用途（`rlinf/envs/realworld/franka/`）：

| 用途 | 代码位置 / 逻辑 |
|------|----------------|
| 成功判定 / 奖励 | `FrankaEnv._calc_step_reward`：TCP 与 `target_ee_pose` 的 xyz 偏差 ≤ `reward_threshold` 则 reward=1 |
| Episode 重置起点 | `PegInsertionConfig.__post_init__`：`reset_ee_pose = target_ee_pose + [0, 0, clip_z_range_high, 0, 0, 0]` |
| 安全工作空间 | `ee_pose_limit_min/max` 以 `target_ee_pose` 为中心按 `clip_x/y/z_range` 裁剪 |
| 随机 reset | 在 `target_ee_pose` 附近做 xy、rz 小范围扰动 |

Peg-insertion 默认 `reward_threshold` 为 xyz 各 **0.01 m（1 cm）**（`peg_insertion_env.py`）。

成功判定核心逻辑（`franka_env.py`）：

```python
position = np.hstack([self._franka_state.tcp_pose[:3], euler_angles])
target_delta = np.abs(position - self.config.target_ee_pose)
is_in_target_zone = np.all(target_delta[:3] <= self.config.reward_threshold[:3])
```

### 怎么获取？

1. Franka 切换到可编程模式（FCI）。
2. **手动**将机械臂移到目标位姿（通常为插销完全插入后的 TCP 位置）。
3. 设置环境变量并运行测试脚本：

```bash
export FRANKA_ROBOT_IP=<robot_ip>
python -m toolkits.realworld_check.test_franka_controller
```

4. 在交互提示中输入 `getpos_euler`，得到 `[x, y, z, roll, pitch, yaw]`（米 + 弧度，欧拉角 **xyz** 顺序）。

脚本实现（`toolkits/realworld_check/test_franka_controller.py`）：

```python
tcp_pose = controller.get_state().wait()[0].tcp_pose
r = R.from_quat(tcp_pose[3:].copy())
euler = r.as_euler("xyz")
print(np.concatenate([tcp_pose[:3], euler]))
```

5. 写入 YAML，例如 `realworld_collect_data.yaml` 或 `realworld_peginsertion_rlpd_cnn_async.yaml`：

```yaml
env:
  eval:
    override_cfg:
      target_ee_pose: [0.5, 0.0, 0.1, -3.14, 0.0, 0.0]  # 替换为实测值
```

注意：`getpos` 输出四元数格式，配置需用 `getpos_euler` 的欧拉角格式。

---

# 3. HG-DAgger 采集：`demos/` 与 `collected_data/` 的区别

### 文档位置

`docs/source-zh/rst_source/examples/embodied/hg-dagger.rst` 第 226–229 行；详细格式见 `docs/source-zh/rst_source/guides/data_collection.rst`。

同一次 `collect_data.sh` 启动的采集运行会 **并行** 写出两份数据，来源相同（SpaceMouse 干预动作 `info["intervene_action"]`），格式与下游用途不同。

### `collect_data.sh` 起什么作用？必须启动它才能遥操作吗？

**作用：** `examples/embodiment/collect_data.sh` 是一个薄封装启动脚本，本身不包含采集逻辑。它做三件事（见脚本源码）：

1. 设置 `EMBODIED_PATH`、`REPO_PATH`、`PYTHONPATH`
2. 创建带时间戳的日志目录 `logs/{timestamp}/`
3. 启动 `python examples/embodiment/collect_real_data.py`，并传入 Hydra 配置名（默认 `realworld_collect_data`，也可传自定义名如 `my_realworld_pnp_collect`）

真正干活的是 `collect_real_data.py` 里的 `DataCollector` Worker：

- 初始化 `RealWorldEnv`（含 Franka 控制 + 相机）
- 若配置 `use_spacemouse: True`，通过 wrapper 栈挂载 `SpacemouseIntervention`（`rlinf/envs/realworld/common/wrappers/apply.py`）
- 循环 `env.step()`，从 `info["intervene_action"]` 读取 SpaceMouse 动作并驱动机器人
- 成功 episode 写入 `demos/`（replay buffer）和/或 `collected_data/`（LeRobot）
- 达到 `runner.num_data_episodes`（默认 20）后自动退出

**是否必须启动它才能遥操作？—— 不是。**

遥操作能力来自环境 wrapper（`SpacemouseIntervention` / `GelloIntervention` / `PicoIntervention` 等），只要某个 RLinf 进程以 `use_spacemouse: True`（或对应设备 flag）运行 **真机 env**，SpaceMouse 就能接管控制。`collect_data.sh` 只是 **离线示教数据采集** 的入口之一。

| 场景 | 入口脚本 | 是否遥操作 | 是否落盘示教数据 |
|------|---------|-----------|----------------|
| 离线采集（HG-DAgger Step 1 / RLPD 先验） | `collect_data.sh` | 是 | 是（`demos/` + `collected_data/`） |
| 在线 HG-DAgger 训练 | `run_realworld_async.sh realworld_pnp_dagger_openpi` | 是（`use_spacemouse: True`） | 干预步进内存 replay buffer，非初始 `demos/` |
| 仅读 TCP 位姿标定 | `python -m toolkits.realworld_check.test_franka_controller` | 否（只读状态） | 否 |

前提条件（与是否用 `collect_data.sh` 无关）：控制节点需已 source ROS/catkin/venv，Ray 集群（若多节点）已就绪，Franka 处于可编程模式，`FrankaController` 能正常连接。

### 如何获得？

入口：`examples/embodiment/collect_real_data.py` 的 `DataCollector`。

- **replay-buffer 轨迹** → `logs/{timestamp}/demos/`：由 `TrajectoryReplayBuffer(auto_save=True, auto_save_path=.../demos)` 写入。
- **LeRobot 数据** → `logs/{timestamp}/collected_data/`：由 `CollectEpisode` wrapper（需 `env.eval.data_collection.enabled=True`）写入。

HG-DAgger 文档推荐配置（`export_format: "lerobot"`）：

```yaml
env:
  eval:
    data_collection:
      enabled: True
      save_dir: ${runner.logger.log_path}/collected_data
      export_format: "lerobot"
      only_success: True
      robot_type: "panda"
      fps: 10
```

启动：

```bash
bash examples/embodiment/collect_data.sh my_realworld_pnp_collect
```

### 各自用途（HG-DAgger 流程）

| | `demos/`（replay-buffer 轨迹） | `collected_data/`（LeRobot） |
|---|---|---|
| 格式 | RLinf `Trajectory`，`.pt` 文件 | LeRobot 标准（Parquet + JSON meta） |
| HG-DAgger 主路径 | 初始采集写入磁盘；**SFT 不直接使用** | **SFT 与 norm stats 的输入** |
| 典型下游 | RLPD 离线先验、`toolkits/replay_buffer/visualize.py` | `calculate_norm_stats.py`、OpenPI SFT |
| 在线 HG-DAgger | 训练时在 **内存 replay buffer** 累积新干预步（`only_save_expert: True`），非读取初始 `demos/` | 在线阶段一般不再追加 |

HG-DAgger 三阶段数据流：

```
采集 (collect_data.sh)
  collected_data/  →  calculate_norm_stats  →  OpenPI SFT (realworld_sft_openpi.yaml)
  demos/           →  备用（RLPD / 可视化等）

在线 HG-DAgger (realworld_pnp_dagger_openpi.yaml)
  内存 replay buffer ← 运行时专家干预步 → DAgger actor loss
```

SFT 配置引用 LeRobot 路径（`examples/sft/config/realworld_sft_openpi.yaml`）：

```yaml
data:
  train_data_paths: "/path/to/realworld-franka-bin-relocation-dataset"
```

在线 DAgger 配置（`examples/embodiment/config/realworld_pnp_dagger_openpi.yaml`）：

```yaml
algorithm:
  dagger:
    only_save_expert: True
  replay_buffer:
    auto_save: False   # 在线 buffer 在内存中，非写入初始 demos/
```

### 数据格式

#### A. replay-buffer 轨迹 — `logs/{timestamp}/demos/`

目录结构（`rlinf/data/storage/replay/buffer.py`）：

```
demos/
├── metadata.json
├── trajectory_index.json
├── trajectory_0_.pt
├── trajectory_1_.pt
└── ...
```

单条 `.pt` 为 `Trajectory` dataclass 序列化字典，主要字段（`rlinf/data/schema/embodied_types.py`、`collect_real_data.py`）：

```python
{
    "actions":          Tensor[T, action_dim],
    "rewards":          Tensor[T, 1],
    "dones":            Tensor[T, 1],
    "terminations":     Tensor[T, 1],
    "truncations":      Tensor[T, 1],
    "intervene_flags":  Tensor[T, ...],   # 采集时全设为 1（全程专家）
    "curr_obs": {
        "states":       Tensor[T, ...],
        "main_images":  Tensor[T, H, W, 3],
    },
    "next_obs": { ... },
}
```

文档说明（`data_collection.rst`）：含 `curr_obs` / `next_obs` 成对 transition，适合 replay buffer 采样；采集时 `intervene_flags` 全为 1，标记专家演示。

#### B. LeRobot 数据 — `logs/{timestamp}/collected_data/`

`export_format: "lerobot"` 时目录（`CollectEpisode` + `data_collection.rst`）：

```
collected_data/
└── rank_0/
    └── id_0/
        ├── meta/
        │   ├── info.json
        │   ├── episodes.jsonl
        │   ├── tasks.jsonl
        │   └── stats.json
        └── data/chunk-000/
            ├── episode_000000.parquet
            └── ...
```

Parquet 每行一帧，典型列：`image`、`state`、`actions`、`task`、`intervene_flag`、`done`、`is_success`、`timestamp`、`frame_index`、`episode_index` 等。

`CollectEpisode._buffer_to_lerobot_ep`（`rlinf/envs/wrappers/collect_episode.py`）逐步字段：

```python
frame = {
    "state": ...,
    "actions": ...,          # 有干预时用 intervene_action
    "task": task_desc,
    "is_success": ...,
    "done": ...,
    "intervene_flag": ...,
    "image": ...,            # 可选
    "wrist_image": ...,      # 可选
}
```

### 为何一次采集写两份？

HG-DAgger 混合两阶段：

1. **离线 SFT**：OpenPI 需要 LeRobot 帧格式 `(image, state, action, task)`。
2. **在线 DAgger**：RLinf actor 从 `Trajectory` replay buffer 采样干预 transition 做监督更新。

`collect_real_data.py` 在同一次遥操作中同时导出，避免重复采集。

---

## 4. 在线 HG-DAgger：人如何介入示教？

### 4.1 算法含义（与经典 DAgger 的区别）

**经典 DAgger**（Ross et al., 2010；RLinf 仿真配置如 `libero_spatial_dagger_openpi.yaml`）：

- Rollout 以概率 `beta` 自动切换 **专家模型** / **学生模型** 出动作。
- 学生执行的 step 还会被专家模型 **重标注**（re-label）后写入 buffer。
- 配置：`rollout.expert_model` + `only_save_expert: False`。

**HG-DAgger（Human-Gated DAgger，RLinf 真机路径）**：

- **人**（SpaceMouse / PICO / GELLO）是专家，不是第二个神经网络。
- 真机配置 **不设** `rollout.expert_model`（见 `realworld_dual_franka_dagger_openpi.yaml` 注释：此时 `beta` 不用于模型混合）。
- 学生策略（SFT 后的 OpenPI π₀）**始终**在 rollout GPU 上推理出动作；人只在认为必要时 **手动接管**。
- `only_save_expert: True`：**只把人工接管 step 写入 replay buffer**，学生独自执行的 step 不用于训练（不做 re-label）。

### 4.2 整体流程（离线 + 在线）

```
[已完成] 离线 collect_data.sh → LeRobot 数据 → norm stats → OpenPI SFT → ckpt
                                    ↓
[在线] run_realworld_async.sh realworld_pnp_dagger_openpi
  ├─ GPU 节点：Rollout(OpenPI 推理) + Actor(梯度更新)
  └─ Franka 控制节点：Env + SpaceMouse + 真机执行
         ↓ 循环
  学生自动跑 → 人 SpaceMouse 接管 → 接管步进 buffer → Actor SFT loss 更新 → 权重同步回 Rollout
```

### 4.3 硬件与人员站位

| 组件 | 所在节点 | 说明 |
|------|---------|------|
| SpaceMouse | **Franka 控制节点**（`RLINF_NODE_RANK=1`） | USB 接在控制节点，与 `pyspacemouse` 通信 |
| 操作员 | 控制节点旁 | 一手 SpaceMouse，目视机械臂与相机画面 |
| GPU 训练 | Head 节点（`RLINF_NODE_RANK=0`） | 无需接 SpaceMouse |
| 机械臂 / 相机 | 控制节点 | 经 ROS + `FrankaController` 驱动 |

配置要求（`realworld_pnp_dagger_openpi.yaml`）：`env.train/eval.use_spacemouse: True`，`no_gripper: False`（7 维动作含夹爪）。

### 4.4 启动在线 HG-DAgger 的步骤

**Step 0 — 前置：** 已完成 SFT，`runner.ckpt_path` 指向 SFT checkpoint；Ray 双节点集群已就绪（见 `hg-dagger.rst` 集群设置）。

**Step 1 — 控制节点（rank 1）：** source ROS / venv，设置 `RLINF_NODE_RANK=1`，`ray start --address=<head_ip>:6379`（若尚未加入集群）。

**Step 2 — GPU head（rank 0）：** source OpenPI 环境，`RLINF_NODE_RANK=0`，确认 `realworld_pnp_dagger_openpi.yaml` 中 `robot_ip`、`target_ee_pose`、`camera_serials`、模型路径、ckpt 路径正确。

**Step 3 — 在 head 节点启动训练：**

```bash
bash examples/embodiment/run_realworld_async.sh realworld_pnp_dagger_openpi
```

脚本启动 `train_async.py` → `AsyncEmbodiedRunner`，**三个长期并行协程**（`async_embodied_runner.py`）：

1. `env.interact()` — 控制节点真机 env 循环
2. `rollout.generate()` — GPU 上 OpenPI 推理
3. `actor.recv_rollout_trajectories()` + 周期性 `actor.run_training()` — 收轨迹、更新权重

**Step 4 — 操作员就位：** SpaceMouse 放在控制节点旁，训练开始后 **无需额外命令**；机械臂会按学生策略自动运动。

### 4.5 运行时：一步数据流（代码路径）

每个 control step 大致顺序（`env_worker.py` + `huggingface_worker.py` + `spacemouse_intervention.py`）：

1. **Env** 把当前观测（相机 + 状态）发给 **Rollout**。
2. **Rollout** 用 **学生 OpenPI** 推理，得到 `num_action_chunks=10` 的动作 chunk（无 `expert_model` 时不走 `beta` 混合）。
3. **Env** 收到学生动作，调用 `env_interact_step` → `chunk_step` → 底层 `SpacemouseIntervention.step()`：
   - 读 SpaceMouse 6 轴 + 左右键；
   - 若最近 **0.5 秒内** 有移动或按键 → **用 SpaceMouse 动作替换学生动作**，写 `info["intervene_action"]`；
   - 否则执行学生动作。
4. `RealWorldEnv` 根据是否有 `intervene_action` 设 `info["intervene_flag"]=True`（`realworld_env.py`）。
5. **轨迹构建：** 下一步开始时 `update_last_actions()` 把该 step 记录的动作 **改写成人工动作**，并打上 `intervene_flags`（`embodied_trajectory_builder.py`）。
6. Rollout epoch 结束 → 轨迹送 **Actor** → `extract_intervene_traj(mode="all")` **只保留** `intervene_flags` 全为 True 的 step → 写入内存 **replay buffer**（`async_fsdp_dagger_policy_worker.py`）。
7. **Actor** 从 buffer 采样，算 `embodied_dagger` 监督损失（OpenPI `prepare_dagger_sft_batch`），更新权重；定期 sync 回 Rollout。

### 4.6 操作员具体怎么介入（SpaceMouse）

代码逻辑（`SpacemouseIntervention`）：

| 操作 | 效果 |
|------|------|
| **推动/旋转 SpaceMouse 帽** | 6 DoF 末端增量控制；norm > 0.001 即视为干预，**接下来约 0.5 s 内** 持续用 SpaceMouse 动作 |
| **左键** | 夹爪闭合命令 |
| **右键** | 夹爪张开命令 |
| **松开 SpaceMouse、停止按键** | 约 0.5 s 后恢复 **学生策略** 控制 |

**推荐操作习惯：**

1. **默认放手**：让学生策略执行，观察 TensorBoard / 现场表现。
2. **即将失败或偏离时**：立即推动 SpaceMouse 接管，把臂引回正确轨迹（如 PnP 中对准、下降、释放）。
3. **修正完成后**：松开 SpaceMouse，让学生继续；不必停训练脚本。
4. **每个 episode** 可多次「学生跑 → 人接 → 人放 → 学生跑」；只有 **实际接管的 step** 会进 buffer。
5. Episode 结束（成功/超时 `max_episode_steps: 180`）后 env **auto_reset**，下一轮继续。

**与离线采集相同：** 介入方式一致，区别是在线阶段数据进 **内存 replay buffer** 并 **实时更新模型**，而非写 `demos/` / `collected_data/`。

### 4.7 训练何时真正开始更新？

Actor 等待 replay buffer 达到 `min_buffer_size`（默认 **4** 条干预样本，`realworld_pnp_dagger_openpi.yaml`）才开始 `run_training()`。初期需 **至少几次 SpaceMouse 接管**，TensorBoard 才会出现 `train/dagger/actor_loss`。

关注指标（`hg-dagger.rst`）：

- `train/dagger/actor_loss` — 干预样本上的监督损失
- `train/replay_buffer/num_trajectories` / `total_samples` — buffer 中干预轨迹量
- `env/success_once` 等 — 任务成功率是否随训练上升

### 4.8 配置项与常见误解

| 配置 | 真机 HG-DAgger 实际作用 |
|------|------------------------|
| `algorithm.dagger.only_save_expert: True` | 只训练人工接管 step |
| `algorithm.dagger.init_beta` / `beta_decay` | **无 expert_model 时不控制 rollout**；可保留默认值 |
| `rollout.expert_model` | 真机 HG-DAgger **不配置** |
| `env.*.use_spacemouse: True` | **必须**；否则无法人工接管 |
| `collect_data.sh` | **仅离线阶段**；在线 HG-DAgger **不需要**再跑它 |

### 4.9 相关源码索引（在线介入）

| 环节 | 路径 |
|------|------|
| SpaceMouse 接管逻辑 | `rlinf/envs/realworld/common/wrappers/spacemouse_intervention.py` |
| intervene_flag 上报 | `rlinf/envs/realworld/realworld_env.py` |
| 轨迹改写 + 发送 Actor | `rlinf/workers/env/env_worker.py` |
| 只存专家步 | `rlinf/workers/actor/async_fsdp_dagger_policy_worker.py` |
| DAgger 损失 | `rlinf/workers/actor/fsdp_dagger_policy_worker.py` |
| 异步训练主循环 | `rlinf/runners/async_embodied_runner.py` |
| 真机 HG-DAgger 配置 | `examples/embodiment/config/realworld_pnp_dagger_openpi.yaml` |

### 4.10 核心结论

在线 HG-DAgger 时，**不需要单独启动 `collect_data.sh`**。你启动的是 `run_realworld_async.sh`，机械臂会 **一直由 SFT 后的 OpenPI 学生策略驱动**；操作员在控制节点用 **SpaceMouse 随时接管**，接管的那些 step 才会进入 replay buffer 并更新模型。

这就是 **Human-Gated（人门控）**：何时示教由人决定，而不是仿真 DAgger 里用 `beta` 随机切专家模型。

---

### 4.11、和经典 DAgger 差在哪？

| | 仿真经典 DAgger | 真机 HG-DAgger |
|---|---|---|
| 专家是谁 | 配置里的 `rollout.expert_model` | **人**（SpaceMouse） |
| 何时用专家 | 概率 `beta` 自动混合 | **人推动 SpaceMouse 时** |
| 学生 step 怎么办 | 常被专家 re-label 后入库 | **`only_save_expert: True` 时直接丢弃** |
| 真机配置 | 有 `expert_model` | **不设** `expert_model` |

`realworld_dual_franka_dagger_openpi.yaml` 里写得很清楚：没有 `rollout.expert_model` 时，`init_beta` **不会**用来做模型混合；人工介入走 `info["intervene_action"]`。

---

### 4.12、完整流程（离线 + 在线）

```
阶段 A（离线，已完成）
  collect_data.sh → LeRobot → norm stats → OpenPI SFT → checkpoint

阶段 B（在线，你要操作的）
  run_realworld_async.sh realworld_pnp_dagger_openpi
    → 学生自动跑臂
    → 你 SpaceMouse 接管
    → 接管步 → replay buffer → Actor 更新 OpenPI
    → 权重同步回 Rollout → 下一轮学生更聪明
```

---

### 4.13、在线阶段：一步步怎么操作

#### 准备（训练启动前）

1. **双节点 Ray 集群**（GPU head rank 0 + Franka 控制 rank 1），见 `hg-dagger.rst`。
2. **控制节点**：source ROS / venv；SpaceMouse **USB 接在控制节点**（不是 GPU 节点）。
3. **改好** `realworld_pnp_dagger_openpi.yaml`：
   - `runner.ckpt_path` → SFT checkpoint
   - `robot_ip`、`target_ee_pose`、`camera_serials`
   - `env.train/eval.use_spacemouse: True`（必须）
   - `algorithm.dagger.only_save_expert: True`

#### 启动训练（GPU head 节点）

```bash
bash examples/embodiment/run_realworld_async.sh realworld_pnp_dagger_openpi
```

这会拉起三个 **并行、长期运行** 的组件（`AsyncEmbodiedRunner`）：

- **Env**（控制节点）：读相机、执行动作、读 SpaceMouse
- **Rollout**（GPU）：OpenPI 学生推理
- **Actor**（GPU）：收干预轨迹、算 `embodied_dagger` 损失、更新权重

#### 训练跑起来后，人做什么？

**你不需要再敲任何命令。** 站在控制节点旁，一手 SpaceMouse，眼看机械臂：

| 情况 | 你要做的 |
|------|---------|
| 学生做得对 | **不要碰** SpaceMouse，让它自动跑 |
| 要撞、抓偏、轨迹不对 | **立即推/转 SpaceMouse 帽** 接管 |
| 接管中 | 像离线采集一样遥操作，修正到正确轨迹 |
| 修正完成 | **松开 SpaceMouse**，约 0.5 s 后自动交回学生 |
| 一个 episode 结束 | env 自动 reset，继续下一轮 |

#### SpaceMouse 具体操作（代码：`SpacemouseIntervention`）

- **帽体平移/旋转** → 6 DoF 末端控制；有输入则视为干预
- **左键** → 夹爪关
- **右键** → 夹爪开
- **干预后 0.5 s 内** 即使暂时不动，仍算人工控制；超过约 0.5 s 无输入才回到学生策略

轴映射在 `SpaceMouseExpert` 里与机器人 base 坐标系对齐。

---

### 4.14、背后发生了什么？（一次 step）

理解这个有助于你知道「什么时候算示教数据」：

```
1. 相机观测 → Rollout(GPU) → 学生 OpenPI 出 10 步 action chunk
2. 动作送到控制节点 env
3. SpacemouseIntervention 每子步检查：
     - SpaceMouse 有输入 → 执行人工动作，写 info["intervene_action"]
     - 无输入 → 执行学生动作
4. RealWorldEnv 设 info["intervene_flag"]=True（若本步被接管）
5. EnvWorker 把该步记录的动作改写成人工动作，打 intervene_flags
6. 本轮 rollout 结束 → 轨迹送 Actor
7. extract_intervene_traj(mode="all") → 只保留 intervene 步 → replay buffer
8. buffer ≥ min_buffer_size(默认4) → Actor SFT 更新 → 权重 sync 回 Rollout
```

所以：**只有你真的推动了 SpaceMouse 的那些 step 才会训练**；学生独自跑的 step 在 `only_save_expert: True` 下不会进 buffer。

---

### 4.15、什么时候能看到模型在学习？

- 至少要 **几次 SpaceMouse 接管**，buffer 攒够 `min_buffer_size: 4` 条干预样本，Actor 才开始更新。
- TensorBoard 看：
  - `train/dagger/actor_loss` — 是否在降
  - `train/replay_buffer/total_samples` — 干预样本是否在涨
  - `env/success_once` — 成功率是否提升

---

### 4.16、常见误解

1. **「要跑 collect_data.sh 才能在线示教？」** — 否。`collect_data.sh` 仅 **离线** 用；在线示教在 `run_realworld_async.sh` 运行中 **直接推 SpaceMouse**。
2. **「beta=1.0 表示一直用专家？」** — 真机没配 `expert_model` 时，**beta 不影响 rollout**；专家就是人。
3. **「要暂停训练才能接管？」** — 否。接管是 **运行时热切换**，无需停脚本。
4. **「GPU 节点也要接 SpaceMouse？」** — 否。SpaceMouse 必须在 **Franka 控制节点**。

---

更完整的逐步说明（含源码索引）已写入 [`b/d/knwldge.md`](/home/luogang/SRC/RL/RLinf/b/d/knwldge.md) 第 4 节。若你用的是 PICO VR 双臂流程，介入设备换成 PICO，逻辑相同，可参考 `dual_franka_pico_dagger.rst`。

---

# 5. 笛卡尔阻抗控制模式（相对关节阻抗 / 关节运动）

### 文档与代码语境

`b/d/frk1/franka_3.md` Step 3 验收写「臂（关节 home、**单关节 nudge**、笛卡尔阻抗）」；底层实现在 `rlinf/envs/realworld/franka/franky_controller.py` 与扩展包 `b/x/franky_ext/controller_extended.py`。

franky 栈里**不是**「阻抗 vs 完全不阻抗」，而是至少三条控制路径：

| 模式 | 底层 | 入口 API | 控制量 |
|------|------|----------|--------|
| **关节阻抗** | `JointImpedanceTracker` | `move_joints(q)` | 7 个关节角目标 |
| **笛卡尔阻抗** | `CartesianImpedanceTracker` | `move_tcp_pose` / `move_arm` | TCP 位姿 `[x, y, z, quat]` |
| **关节位置运动**（一次性） | `JointMotion` | `reset_joint(home)` | 绝对关节角，走完后结束 |

切换逻辑：

- 调 `move_joints` → 先停掉笛卡尔 tracker，启动 **关节阻抗** tracker
- 调 `move_tcp_pose` / `move_arm` → 先停掉关节 tracker，启动 **笛卡尔阻抗** tracker
- 调 `reset_joint` → **两种 tracker 都停掉**，用 `JointMotion` 做一次到位运动

同一时刻 franky 栈里**只跑一种 tracker**，避免两套控制打架。

文档里的「笛卡尔阻抗」主要指 `CartesianImpedanceTracker` 这一路；Step 3 的「关节 nudge」走的是 `JointImpedanceTracker`（或 home 时的 `JointMotion`），**不是**笛卡尔阻抗。

---

### 5.1 Step 3 里两个术语分别指什么

#### 关节 nudge（单关节微动）

在**关节空间**里，给某一个关节加一个很小的角度增量，验证 Controller 能否正确下发 `move_joints`。

`b/x/scripts/step3_test_controller.py`：读当前 7 个关节角，只改 **J1**：`target[0] += 0.05`（约 **+0.05 rad ≈ 2.9°**），再 `ctrl.move_joints(target)`。

交互版 `toolkits/realworld_check/test_franky_controller.py` 对应命令：`nudge <关节编号 1..7> <增量 rad>`，例如 `nudge 2 0.05`。

| | Step 3 关节 nudge | Step 5 micro-nudge |
|--|-------------------|---------------------|
| 控制空间 | **关节角**（可指定单个关节） | **TCP 笛卡尔**（末端位姿增量） |
| 典型幅度 | J1 +0.05 rad | Δx ≈ 5 mm |
| 关节行为 | 主要动一个关节 | 7 关节**协同**实现末端微动 |

Step 5 **不能替代** Step 3 的关节 nudge：Step 5 测的是 env 的 TCP action，测不到「单关节 `move_joints` 是否正常」。

#### 笛卡尔阻抗（Cartesian impedance / compliance）

把机械臂切到**笛卡尔阻抗控制模式**——末端（TCP）在 x/y/z 和姿态方向上有「弹簧-阻尼」般的柔顺性，而不是硬位置跟踪。RL 里 `env.step` 发的是 TCP 位姿/增量，`FrankaEnv` 的 `move_arm` 依赖的就是这种模式。

Step 3 调用 `reconfigure_compliance_params`，设置平移/旋转刚度，底层启动 franky 的 `CartesianImpedanceTracker`：

```python
ctrl.reconfigure_compliance_params(
    {"translational_stiffness": 2000.0, "rotational_stiffness": 150.0}
)
```

对应文档里的 `impedance 2000 150`。`FrankaEnv.reset()` 会调 `reconfigure_compliance_params`；之后 `move_arm` → `move_tcp_pose` 要在笛卡尔阻抗模式下工作。Step 3 验证的是：扩展 Controller 能把 ROS 侧的 compliance 参数接到 franky，且不会 `AttributeError`。

**和关节 nudge 的关系：**

- **关节 nudge** → 关节空间跟踪（`move_joints` / `JointImpedanceTracker`）
- **笛卡尔阻抗** → 末端空间柔顺控制（`move_arm` / `CartesianImpedanceTracker`）

Step 3 要两种都过，因为 env 训练链里两种 API 都会用到。

| 术语 | 测什么 | Step 3 里对应动作 |
|------|--------|-------------------|
| **关节 home** | 能否回到标准 home 姿态 | `reset_joint(HOME_JOINTS)` |
| **单关节 nudge** | 关节空间小步运动是否正常 | J1 +0.05 rad，`move_joints` |
| **笛卡尔阻抗** | compliance / 阻抗模式能否配置 | `reconfigure_compliance_params(2000, 150)` |

三者都在 **Ray Worker 上的 `FrankyControllerExtended`** 里测，**不经过 Gym env**；Step 5 只测 env 全链路，且 5c 只是 TCP 毫米级微动，覆盖不了上面这三项。

---

### 5.2 原理：共同点与空间差异

#### 共同点：都是「阻抗 / 柔顺」思想

阻抗控制的直觉是：**末端（或关节）像连在一根弹簧 + 阻尼器上**。

- 离目标越远 → 控制器产生越大的力/力矩去拉回
- 外力推过来 → 允许一定位移，力与位移大致成正比（刚度 K）
- K 越大 → 越「硬」，越像 rigid position control
- K 越小 → 越「软」，越容易被人/环境推开（适合接触、插孔）

这和纯位置控制（无限大刚度、误差必须为 0）不同；Franka 的 impedance 是**有限刚度**，兼顾跟踪和接触安全。

#### 关节空间 vs 笛卡尔空间

```
关节阻抗 (move_joints)
  你指定：q1…q7 各多少 rad
  控制器：在关节空间里用 7 组刚度/阻尼跟踪
  末端轨迹：由正运动学决定，一般不是直线

笛卡尔阻抗 (move_arm / move_tcp_pose)
  你指定：TCP 在基座坐标系下 x,y,z + 姿态
  控制器：在笛卡尔空间里用平移/旋转刚度跟踪
  关节怎么动：由逆解 + 冗余度（nullspace）算出来
  末端轨迹：更符合「手在空间某方向移动/保持姿态」的直觉
```

**RL 真机 env 为什么用笛卡尔阻抗：**  
`FrankaEnv.step` 的 action 是 **TCP 增量**（Δx, Δy, Δz, Δr, Δp, Δy, gripper），每次调用的是 `move_arm` → 必须在 **笛卡尔空间** 跟踪目标，而不是直接给 7 个关节角。

```python
# rlinf/envs/realworld/franka/franka_env.py
def _move_action(self, position: np.ndarray):
    self._controller.move_arm(position.astype(np.float32)).wait()
```

`reset()` 时还会先配 compliance，再 `go_to_rest`：

```python
self._controller.reconfigure_compliance_params(
    self.config.compliance_param
).wait()
```

---

### 5.3 使用场景

| 场景 | 更合适的方式 | 原因 |
|------|--------------|------|
| 回 home、大范围关节复位 | `reset_joint`（`JointMotion`） | 一次性轨迹，不依赖持续 tracker |
| Step 3 测单关节小动 | `move_joints`（关节阻抗） | 直接验「关节指令链」 |
| 日常 RL / 遥操作 step | `move_arm`（笛卡尔阻抗） | action 本来就是 TCP 空间 |
| 插孔、贴面、轻触 | 笛卡尔阻抗 + **较低**平移刚度 | 允许沿接触方向柔顺，减少顶死、过力 |
| 需要精确走直线/固定姿态 | 笛卡尔阻抗 + **较高**刚度 | 跟踪紧，但接触时力也大 |

Step 3 里 `reconfigure_compliance_params(2000, 150)` 是在验：**env reset 时要用的那套笛卡尔刚度参数，扩展 Controller 能正确设进去**，而不是为了配合后面的关节 nudge（关节 nudge 走 `move_joints`，会切到关节 tracker）。

---

### 5.4 程序操控

#### 你发什么指令

```python
# 关节空间 — Step 3 nudge
ctrl.move_joints(target_q)          # shape (7,)

# 笛卡尔空间 — env / RL
ctrl.move_arm(tcp_pose)             # shape (7,) [x,y,z, quat_xyzw]
# FrankyControllerExtended 内部 → move_tcp_pose

# 配刚度 — env reset 前
ctrl.reconfigure_compliance_params({
    "translational_stiffness": 2000.0,
    "rotational_stiffness": 150.0,
})
```

#### 内部会发生什么

1. **`move_joints`**
   - `_stop_cart_tracking_motion()`
   - 创建 `JointImpedanceTracker`（每关节一组 `stiffness` / `damping`）
   - `set_target(q, dq=...)`

2. **`move_arm` / `move_tcp_pose`**
   - `_stop_tracking_motion()`（停关节 tracker）
   - 创建 `CartesianImpedanceTracker`（用 `translational_stiffness` / `rotational_stiffness`）
   - `set_target(Affine(T))`

3. **`reconfigure_compliance_params`**（扩展类）
   - 更新 `_compliance_trans_k` / `_compliance_rot_k`
   - `_stop_cart_tracking_motion()` — 下次 `move_arm` 会用新刚度重建笛卡尔 tracker

4. **`reset_joint`**
   - 两个 tracker 都停
   - 单次 `JointMotion`，不属于持续阻抗跟踪

---

### 5.5 「平移刚度」和「旋转刚度」

这两个参数只作用于 **笛卡尔阻抗模式**（`CartesianImpedanceTracker`）。默认值在 `franky_controller.py`：

```python
_CART_TRANS_STIFFNESS = float(os.environ.get("RLINF_CART_K_T", 500.0))  # N/m
_CART_ROT_STIFFNESS = float(os.environ.get("RLINF_CART_K_R", 40.0))      # Nm/rad
```

#### 平移刚度 `translational_stiffness`（N/m）

TCP 在 x / y / z 方向偏离目标位置时，控制器沿该方向产生的恢复力，大致 **F ≈ K_t × 位置误差(m)**。

- K_t = 500 N/m：偏 1 cm → 约 5 N 恢复力
- K_t = 2000 N/m：偏 1 cm → 约 20 N，**更硬**，更贴目标，但顶到障碍物时力更大

管的是末端**在空间平移**上有多「硬」，不管关节角单独怎么变。

#### 旋转刚度 `rotational_stiffness`（N·m/rad）

TCP 姿态（roll/pitch/yaw）偏离目标时，产生的恢复力矩，大致 **τ ≈ K_r × 角度误差(rad)**。

- K_r 大 → 姿态锁得紧，不容易被拧歪
- K_r 小 → 手腕方向更「软」，适合需要顺应接触姿态的任务

管的是末端**朝向**有多硬，和平移刚度是分开的两组参数。

#### 和「关节刚度」的对比

关节阻抗用的是 **7 个关节各自的刚度**（`_JOINT_STIFFNESS`，单位 N·m/rad，按关节）：

```python
_JOINT_STIFFNESS = [103.75, 265.734, 227.273, 221.445, 13.5, 12.818, 5.134]
```

| | 关节阻抗 | 笛卡尔阻抗 |
|--|----------|------------|
| 刚度定义在 | 每个关节 q1…q7 | TCP 的 x,y,z + 姿态 |
| 参数个数 | 7 个关节 K（+ 阻尼） | 平移 1 个 K_t + 旋转 1 个 K_r（+ nullspace 等） |
| 你关心的量 | 「第 3 关节多转 0.05 rad」 | 「手尖往前 5 mm、姿态不变」 |

Step 3 文档里的 `impedance 2000 150` 指的是 **笛卡尔** 的平移/旋转刚度。交互脚本里 ROS 风格的 `impedance <k1…k7>` 是另一套（7 关节）；本方案 Step 3 自动化脚本用的是 **translational / rotational** 这一对。

---

### 5.6 Step 3 串起来看在测什么

```
reset_joint(HOME)     → JointMotion，大复位，不是阻抗 tracker
open/close/grip       → 夹爪，与臂控制模式无关
reconfigure(2000,150) → 为后续 env 准备笛卡尔刚度（停掉旧 cart tracker）
move_joints(J1+0.05)  → 切到关节阻抗，验关节链
```

- **关节 nudge**：验 **关节空间 + JointImpedanceTracker**
- **笛卡尔阻抗配置**：验 **TCP 空间 + CartesianImpedanceTracker 的参数能设、能用于 env 的 `move_arm`**

Step 5 的 `micro-nudge` 只在 **env 已处于笛卡尔控制链** 上发 TCP 增量，不会替 Step 3 测关节 `move_joints`，也不会单独验 `reconfigure_compliance_params`。

#### 调参时怎么记

- **任务要稳、跟踪紧、不太接触** → 提高平移/旋转刚度（如 2000 / 150）
- **任务要插孔、贴面、怕顶死** → 降低平移刚度（尤其接触方向），旋转刚度视是否要保持工具朝向而定
- **只动一个关节做诊断** → 用 `move_joints`，看关节链
- **和 RL action 一致** → 必须用 `move_arm` + 合适的笛卡尔刚度

---

# 五知识点:路点(waypoint), ramp, "旧脚本硬编码 K_t=2000、clip=0.05——也就是 2000 N/m × 0.05 m = 100 N 的力上限" , "刚度翻倍时 clip 自动减半，力上限不变", "ramp 结束后的 settle 窗口"

这五个点其实都围绕同一个物理对象：**笛卡尔阻抗控制 = 一根弹簧**。先建立这个图景，五个点就都顺了。

## 基础图景：阻抗控制是一根弹簧

控制器每一时刻都在算：

```
力 F = K_t × (目标位置 − 实际位置) = K_t × e
```

`K_t` 是弹簧刚度（N/m），`e` 是位置误差。目标在哪，弹簧就把臂往哪拉；误差越大，拉力越大。臂不是"被搬到目标"，而是"被弹簧追着目标拖过去"——有惯性、会滞后、会冲过头。

## 1. 路点（waypoint）

`_interpolate_move(目标)` 不是把终点直接丢给控制器，而是把路径切成一串**中间目标**，按约 10 Hz 逐个下发：

```
当前 ──► 路点1 ──► 路点2 ──► ... ──► 终点
        (t=0.1s)   (t=0.2s)
```

每个中间目标就是一个路点。相当于挂在驴前面的胡萝卜，每次只往前挪一小步，弹簧永远只追一个"很近的目标"，误差 `e` 始终很小，力也就很小、很平滑。`--test-waypoints` 复现的就是这套节奏——它是 `reset` 实际下发的命令序列。

## 2. ramp

诊断脚本里 `--test-impedance` 要抬 3 cm，但它**不让目标从 0 直接跳到 3 cm**，而是 3 秒内分 30 步、每步 +1 mm：

```
目标z
  │      ／￣￣ ← 到顶后保持
  │    ／
  │  ／        ← 这一段斜线就是 ramp（斜坡）
  │／
  └────────────► 时间
```

目标位置随时间的曲线是个斜坡，所以叫 ramp。作用：限制**指令速度**（这里是 1 cm/s），避免目标瞬移造成误差 `e` 瞬间拉满、弹簧全力猛拽。

## 3. "2000 N/m × 0.05 m = 100 N 力上限"

弹簧公式 `F = K_t × e` 里，`e` 理论上可以无限大——目标在 1 m 外就是 2000 N，臂会飞出去。所以 franky 有个保险：`translational_error_clip = 0.05 m`，**误差超过 5 cm 就按 5 cm 算**。

于是这个控制器能输出的最大力被钉死在：

```
F_max = K_t × clip = 2000 N/m × 0.05 m = 100 N
```

100 N 是什么概念：大约等于用 10 kg 的哑铃片压在臂上持续推。Panda 臂本身约 18 kg，100 N 足以让它在零点几秒内加速到几十 cm/s——这正是 LOG-019 里臂冲过目标 26 cm 的能量来源。旧脚本把这两个数硬编码进去，等于把事故当天的力上限原样搬到了"诊断"里。

## 4. "刚度翻倍时 clip 自动减半，力上限不变"

这是新旧两种参数思路的差别：

| | 旧思路 | 新思路（`motion_limits`） |
|---|---|---|
| 固定什么 | `clip = 0.05` 固定 | **`F_max` 固定**（比如 30 N） |
| clip 怎么来 | 拍脑袋 | `clip = F_max / K_t` 推出来 |
| K_t 2000→4000 时 | clip 还是 0.05，**力上限悄悄变成 200 N** | clip 自动变 0.0075，**力上限仍是 30 N** |

关键在"谁是不变量"。安全上真正该守恒的是**力**（力决定加速度、决定能撞多重），而不是误差截断。旧思路里调刚度是个隐蔽动作——你只想让臂"跟得紧一点"，实际却把力上限也放大了。新思路下刚度随便调，最坏情况的力永远不变。

## 5. ramp 结束后的 settle 窗口

ramp 到顶、目标停住的那一刻，**臂通常还没停**——它一直在追一个移动的目标，身上带着速度。目标停了，惯性带着臂继续往前冲，越过目标，然后弹簧反向（误差变号）把它拉回来，可能来回荡几下才稳。这段"目标不动、臂自己收敛"的时间就叫 settle（稳定）窗口。

**超调只发生在这个窗口里。** 旧脚本在 ramp 最后一步立刻 `tracker.stop()`——等于在球刚抛到最高点时就把眼睛闭上了，冲出去多远完全没记录。所以 LOG-019 的 26 cm 超调在旧脚本里是不可见的；现在的脚本在 ramp 结束后继续保持目标、持续采样整个 settle 过程，超调才第一次被量出来。

---

串起来看 LOG-019 的教训就是：**弹簧权限给到 100 N（点 3、4）+ 目标停后没人看（点 5）= 臂冲出去 26 cm 而脚本报告"一切正常"**。现在的脚本把力上限锁成不变量、把 settle 窗口纳入观测、再加护栏在超调发生时主动刹车。


# 解析 motion_limits.py.describe_authority()

这是一个**日志摘要函数**：给定一组阻抗刚度 `(K_t, K_r)`，算出控制器此刻实际拥有的"权限"——最大能输出多少力、多少力矩、运动限速多少、护栏余量多少——拼成一行给人看。它在五个地方被调用：烟测脚本、诊断脚本、控制器（每次 `reconfigure_compliance_params` 和 tracker 启动时）、标定 REPL（改完刚度后立刻显示后果）。

## 逐行拆解

```637:659:b/x/franky_ext/motion_limits.py
def describe_authority(k_trans: float, k_rot: float) -> str:
    ...
    root3 = float(np.sqrt(3.0))
    trans_clip, rot_clip = error_clips_for_stiffness(k_trans, k_rot)
    kt, kr = clamp_stiffness(k_trans, k_rot)
    f_axis = kt * trans_clip
    t_axis = kr * rot_clip
```

- **645 行**：clip 不是独立参数，而是由刚度反推（`clip = 力上限 / K`），即上次说的"力上限是不变量"。
- **646 行**：刚度先被钳进硬范围（平移 50–3000 N/m）。所以打印的是**实际生效**的 K，不是你请求的 K。
- **647–648 行**：核心乘积。`f_axis = K_t × clip` 就是**单轴弹簧力上限**（上次讲的 100 N 那个量）；`t_axis` 是单轴力矩上限。

## 为什么要打两个数（per-axis 和 worst-case）

这是这个函数存在的核心理由，docstring 里写得很直白：franky 的 clip 是**逐轴**截断的。如果 x、y、z 三轴**同时**吃满，合力是单轴的 √3 倍：

```
单轴 20 N  →  三轴同时饱和最坏情况 20 × √3 ≈ 34.6 N
```

第一版这个函数只打印了一个 `= 25.0N`，把**单轴**上限当成了**总力**上限展示——看日志的人会以为最坏情况就是 25 N，实际是 43 N。所以现在两个数都报，并且 `error_clips_for_stiffness` 在推导 clip 时也是按"哪个上限先碰到听哪个"来取 min 的（402–404 行：单轴上限和"三轴合力上限 ÷ √3"取小）。

## "damping term NOT clipped"

弹簧力 `K×e` 有 clip 封顶，但阻尼力 `−c×v` **不在**这个封顶之内。这行注释是防止误读：打印出来的数不含阻尼项，但这不是漏洞——阻尼永远**逆着**运动方向，只会消耗能量、不会推着臂加速，所以它对"最坏情况能撞多重"没有贡献。

## 后半行

```
interp<=2.0cm/s, <=0.35m, <=0.60rad; guard margin=0.050m floor=0.010m lag=0.050m
```

- `interp`：插值限速/限距/限角——单次插值最多 2 cm/s、总行程不超过 35 cm、转角不超过 0.6 rad。
- `guard`：三道护栏的余量——z 上限提前 5 cm 刹车、z 地板提前 1 cm 刹车、跟踪滞后超过 5 cm 刹车。

## 一句话总结

它是把"这组刚度到底给了机械臂多大权限"**翻译成一行人类可读的数字**，并且吸取了两个教训：乘积（K×clip）才是权限而不是单个参数；单轴和 √3 最坏情况都要报，因为逐轴截断意味着三轴可以同时饱和。每次改刚度它都会被打印，让人在臂动起来**之前**就看到力上限是多少。


# 解析 step_cube_place_robot.py._hover_and_box()

这个函数是**烟测脚本的"预飞几何计算器"**：给定 H1 标定点 `target`，算出两样东西——悬停位姿和安全盒——供 `connect` 打印出来给人核对。它的重点不在算什么，而在**按谁的方式算**。

## 算什么

```142:153:b/x/scripts/step_cube_place_robot.py
    reset = list(target)
    reset[2] = float(target[2]) + float(z_high)
    lim_min, lim_max = effective_ee_pose_limits(...)
    lim_min[2] = float(target[2]) - float(z_low)
    lim_max[2] = float(target[2]) + float(z_high)
    return reset, lim_min, lim_max
```

- `reset`：悬停位姿 = H1 接触点沿 z 抬 `z_high`（0.08 m）。镜像 `PegInsertionConfig.__post_init__` 里 `reset_ee_pose = target + [0,0,clip_z_range_high,...]` 的算法。
- `lim_min` / `lim_max`：安全盒 `ee_pose_limit_{min,max}`，即 env 在 `step()` 里真正裁剪动作用的边界。
- 151–152 行：把 z 边界显式重写一遍（数值上和 `effective_ee_pose_limits` 算出来的相同，属于防御性重申，保证"盒底 = 接触点 − 5 mm、盒顶 = 接触点 + 8 cm"这个语义一眼可见）。

## 为什么必须用 `effective_ee_pose_limits` 而不是通用盒子

docstring 记录的是一个"看起来在验证、实际在误导"的坑。旧版用的是通用辅助函数 `ee_pose_limits_from_probe`：以位姿为中心、六个自由度统一用一个 margin 撑一个盒。但 env 真实的行为是 `PegInsertionConfig.__post_init__`：

```87:105:rlinf/envs/realworld/franka/tasks/peg_insertion_env.py
        self.ee_pose_limit_min = np.array(
            [
                self.target_ee_pose[0] - self.clip_x_range,
                ...
                self.target_ee_pose[3] - 0.01,   # roll: 硬编码 ±0.01 rad
                self.target_ee_pose[4] - 0.01,   # pitch: 硬编码 ±0.01 rad
                self.target_ee_pose[5] - self.clip_rz_range,  # 只有 yaw 用 clip_rz
            ]
        )
```

两个关键事实：

1. **`__post_init__` 会用 `target_ee_pose` + `clip_*` 重新计算并覆盖 `ee_pose_limit_*`**——你在 `override_cfg` 里传什么盒子都会被丢弃。所以脚本打印的盒子如果不是按这套算法算的，打印的就是一个不会被遵守的盒子。
2. **roll / pitch 的半宽是硬编码的 ±0.01 rad（约 0.57°），只有 yaw 用 `clip_rz_range`（0.35 rad）**。旧版通用盒子给三个旋转轴都套 0.35，于是预飞打印告诉操作员"姿态窗口 ±0.35 rad"，而实际执行的是 ±0.01 rad——**宽了 35 倍**。

docstring 里那句话是这轮审计的总结：*打印一个比实际宽 35 倍的姿态窗口，比什么都不打印更糟，因为它读起来像已经验证过了*（"a reassuring number presented as pre-flight verification"）。

## 在这套流程里的位置

`connect` 子命令会打印 `reset_ee_pose hover` 和 `ee_pose_limit_min/max` 供人核对；`reset` / `box` 也用这组值做事后门闩（悬停高度误差、盒底下限）。这个函数保证**打印的、门闩检查的、env 实际执行的三者是同一个盒子**——这正是它被重写的原因。


# 用 Franky 的 b/x/ 版插充电器例子与RLinf原版的区别

问得很好，这个问题值得掰开揉碎。先给结论：**`b/x/` 不是「重新搞一套」，它恰恰就是「拿原版代码小改」——只是用继承而不是复制粘贴来改的。真正迫使它存在的只有一个硬约束：这台机器人的固件是 5.10.0，而原版 charger 例子的单臂控制链路只支持固件 < 5.9 的 ROS 栈。** 下面逐层给依据。

## 一、b/x 对原版的复用程度，比看上去高得多

看继承链，b/x 里真正「新写」的只是一层薄薄的适配：

```141:141:b/x/franky_ext/tasks/cube_place.py
class FrankyCubePlaceEnv(FrankySingleFrankaEnvMixin, PegInsertionEnv):
```

```96:96:b/x/franky_ext/controller_extended.py
class FrankyControllerExtended(FrankyController):
```

- 任务类 `FrankyCubePlaceEnv` 继承的就是**原版 charger 的任务类** `PegInsertionEnv`（观测空间、动作空间、reward、`go_to_rest` 两段式 reset 全是原版的）；
- 控制器 `FrankyControllerExtended` 继承的是**原版** `rlinf/envs/realworld/franka/franky_controller.py` 里的 `FrankyController`；
- env 本体是原版 `FrankaEnv`，mixin 只换掉其中一个方法（控制器启动）；
- 训练 YAML（`realworld_cube_place_sac.yaml`）就是照 `realworld_charger_sac_cnn_async_standalone_reward.yaml` 改的——`algorithm`（embodied_sac、gamma、tau、entropy tuning）、`actor`/`critic`/`replay_buffer` 结构原样照搬。

所以「为什么不直接参考原版」这个前提本身不完全成立：b/x 从头到尾就是在原版骨架上长出来的。用继承而不用复制，是有意的工程选择——复制会 fork 代码、丢失上游修复；继承让差异点显式可见、上游更新可跟进。

## 二、真正的分叉点：ROS vs franky，而这是固件决定的

原版 charger 例子的单臂链路是 **ROS-only** 的：

```95:116:rlinf/envs/realworld/franka/franka_controller.py
        # Lazy-import ROS packages so the module can be imported on non-ROS nodes.
        import rospy
        ...
        self._ros = ROSController()
        self._init_ros_channels()
```

它要求：ROS noetic + catkin 工作区 + 从源码构建的 `serl_franka_controllers` + `franka_ros` + `roslaunch` 起一个**常驻**的阻抗控制器节点。而这套东西对固件版本有硬限制——`charger_sac_async.md` 里记录得很明确：

- 官方 ROS 路径要求固件 **< 5.9.0**（推荐 5.7.2）；
- 本机 Desk 固件是 **5.10.0** →「官方 YAML **FAIL**，适配路径继续」「本 5090 不要用 ROS 脚本（固件 5.10.0、无 serl）」。

serl_franka_controllers 配套的 franka_ros 0.10.0 / 旧 libfranka 跟 5.10 固件说不上话；能让 5.10.0 工作的只有 libfranka 0.19 + franky 这条直连 FCI 的路。

那「上游不是有 `FrankyController` 吗」——有，但上游只把它接在了**双臂** env 上：

```15:15:rlinf/envs/realworld/franka/dual_franka_env.py
"""Dual-arm Franka env driven through ``FrankyController`` (libfranka)."""
```

单臂 `FrankaEnv`（charger 的基类）在上游**没有** franky 路径。「单臂 + franky + 原装 Franka Hand」这个组合上游根本不存在——上游 `FrankyController` 连原装夹爪都不支持（`gripper_type=franka` 会 `NotImplementedError`，所以 b/x 补了 `franka_libfranka_gripper.py`）。`FrankySingleFrankaEnvMixin` 填的就是这个上游缺口。

## 三、b/x 与原版的实质差异清单

除了控制栈替换，差异全是两次真机事故（LOG-017 静默不动、LOG-019 过冲 26cm）逼出来的安全层：

| 维度 | 原版（ROS charger） | b/x（franky cube-place） |
|---|---|---|
| 阻抗控制器 | `roslaunch` 起的**常驻** ROS 节点，Python 侧发 topic | 进程内异步线程 `CartesianImpedanceTracker`，`set_target` 发目标 |
| 控制器死了 | ROS 侧有节点状态可查 | `set_target` **静默无效** → b/x 加响亮失败（tracker 死亡检测 + 真因重抛） |
| 机器人模式 | 无预检 | `tcp_probe` 卡 `robot_mode`（LOG-017 的 `UserStopped` 静默 `dz=0` 就是这么抓到的） |
| 运动权限 | 无显式上限 | `motion_limits.py`：力 ≤20N/轴、速度钳制、clip 推导 |
| 运动护栏 | 无 | motion guard：几何围栏 + 过冲/滞后主动刹车（LOG-019 产物） |
| 插值速度 | `timeout` 硬编码 1.0/1.5s，10cm 抬升=10cm/s 指令（LOG-019 R1 诱因） | 按位移推导时长，**速度**为常数 |
| 夹爪 | 不支持原装 Hand | 补了 libfranka 直连夹爪 |

这些不是「重写」，是原版代码在这台机器、这只手臂、这个固件上**缺失**的东西。

## 四、「又要重新标定」与代码无关

标定的是 `target_ee_pose`——物理世界里「方块要放到的那个点」。charger 的目标是插座孔，cube-place 的目标是桌上标记，换任务换桌面就必须重标。而且原版 charger YAML 里这一项同样是占位符：

```108:108:examples/embodiment/config/realworld_charger_sac_cnn_async_standalone_reward.yaml
      target_ee_pose: TARGET_EE_POSE
```

用原版代码跑，这一步也省不掉。

## 五、我的观点

1. **如果固件 < 5.9，你的直觉完全正确**：复制 charger YAML、改 `target_ee_pose` 和 clip 范围、标定、开跑，就是最短路径，b/x 这套不必存在。b/x 的存在理由是固件把 ROS 路堵死了，不是「不想参考原版」。
2. **b/x 里有两类东西应该分开评价**：适配层（mixin、夹爪、gym 注册）是「上游缺口的最小补齐」，可以也应该上游化；安全层（mode gate、motion guard、力上限）是真机事故的直接产物，对上游也是通用改进——上游 charger 代码假设的是「实验室里有人全程盯着」的环境。
3. **代价要承认**：b/x 链路上游 CI 不覆盖，每次上游更新都要重新验证；安全层是自研的，正确性靠本机的分阶段验收（step0–9、diag、camera accept）背书。这是「在上游空白区走路」的固有成本，不是设计失误。

一句话：b/x = 原版 env/任务/算法骨架 + 上游缺的「单臂 franky 适配层」+ 两次事故换来的「安全层」。它不是绕开原版，而是在原版走不通的地方（固件 5.10 + 单臂 franky）把路接上。


# 为什么 Franky 版插充电器比 RLinf的ROS版更容易接近奇异点

证据齐了。可以给出一个有依据的回答了。

## 一句话结论

这不是「`b/x/` 的代码写出了上游没有的 bug」，而是三件事叠加：**cube-place 的任务几何把臂带到了 charger 任务从来没去过的地方（腕部奇异面附近）；两栈对「目标」的处理有一个真实的架构差异（ROS 有 1kHz 目标低通，franky 是零阶保持）；而 b/x 有一道上游根本没有的看门狗，把这件事第一次「看见」了。** 逐层给证据。

## 1. 主因是任务几何，不是控制栈——b/x 自己的对照实验已经证明了这一点

LOG-036 里有一组关键对照：**同一套 b/x franky 代码、同一台臂**，在悬停位形（`q5=0.18`，远离 0）跑旋转阶梯四档全过，`peak|dq|` 最差 0.379 rad/s；而两次跳闸都发生在策略把臂带到 `x≈0.72–0.78`（接近伸展极限）、`q5` 被压到 **0.085 rad** 的路上。也就是说，代码相同，位形不同，结果完全不同——变量是**几何**。

而 cube-place 与 charger 的几何差异是设计出来的：

- cube-place 的 H1 目标姿态 `roll≈−3.137`（≈−π），T19 怀疑这个选择让任务天然住在 `q5≈0` 的奇异面附近；
- 工作空间更大：cube-place `clip_x/y=0.05`、`clip_z_high=0.08`，charger 是 `clip_x/y=0.02`、`clip_z_high=0.05`（charger YAML 109-115 行）——策略探索到近奇异位形的概率本来就低得多；
- charger 的 `target_ee_pose` 是「插头插进插座时」的姿态（`charger_sac_async.md` §6.2.1），那个姿态邻域里 `q5` 不在 0 附近——纯粹是他们运气好/任务如此。

**关节速度放大是手臂 + 轨迹的物理属性**（雅可比条件数），它不认控制栈：把 cube-place 的几何给 ROS 栈跑，同样的轨迹一样会放大关节速度。

## 2. 「上游没发生」很大程度上等于「上游没有仪器能看见」

LOG-034/036 的「跳闸」动作本身就不是固件 reflex，而是 **b/x 自加的 motion guard 看门狗**（`|measured−commanded| > 5cm` 刹车）——上游 charger 代码里没有这个东西。上游若遭遇同样的近奇异发散，表现会是：跟踪误差悄悄拉大、可能某次固件 joint velocity reflex、或者干脆「学不动」收场，**没有任何日志会把它归因到 q5**。没有发生过 ≠ 不会发生，只是没人测过。

## 3. 两栈确实有一个真实的架构差异：目标滤波

我进容器翻了 serl 控制器源码（`serl_franka_controllers/src/cartesian_impedance_controller.cpp`），这是 ROS 栈真正「更温柔」的地方：

```cpp
// 1kHz 控制循环里，目标姿态先做一阶低通再进阻抗律：
position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
// filter_params_ = 0.005（头文件默认值）
```

10Hz 的路点阶跃被抹成 1kHz 的指数平滑轨迹，目标速度天然有界（≈ `0.005×1000×步长`）。而 franky 的 `set_target` 是**零阶保持**——目标瞬时跳变（LOG-019 分析里「10 Hz 零阶保持目标」说的就是它）。在近奇异位形附近，阶跃目标要求瞬时大关节速度，滤波目标把这份需求摊到时间轴上。这是 ROS 栈「更不容易」踩雷的真实缓解因素——但注意是缓解不是免疫：任务几何若持续要求 `q5→0`，滤波也救不了。

## 4. 顺便排除两个「看似差异其实不是」的点

- **零空间锚定语义两栈相同**：serl 是 `q_d_nullspace_ = q_initial`（启动时刻的 q），franky 是 `nullspace_target = 当前 q`——都不是「固定的安全 ready 位形」，谁也不比谁优越。（刚度配置不同：serl 全局 0.2 但 joint1 单独 100；franky `K_ns=5` 全关节——影响的是肘部自由度，不是 q5 的主因。）
- **reset 路径是 b/x 从上游原样继承的**：`PegInsertionEnv` 的两段式笛卡尔插值 reset，charger 也是这条路径。风险不是 b/x 引入的。

## 我的观点

T19 的表述方向是对的，但可以补一句更准确的归因：**这不是「b/x 代码 vs 上游代码」的差异，而是「b/x 的任务 + 上游从没被仪器化过的物理」**。上游 charger 在它自己的几何里从来没靠近过奇异面，所以「末端限幅就够」这个隐含假设从来没被检验过；b/x 的 cube-place 几何把它带到了那里，而 b/x 的看门狗第一次让这件事可见。也因此，T19 列的两个候选修法（关节空间的软件限速、或换一个远离 `q5≈0` 的等价目标姿态）才是对的方向——它们针对的是物理，而不是某个栈的实现细节；继续在笛卡尔指令侧加钳（T16 的思路）注定只能延后，LOG-036 已经用「两个分量都钳在预算内、`|dq|` 仍冲到 2.03」把这一点证死了。

## 为什么 RLinf 原代码可以做到"上游 charger 在它自己的几何里从来没靠近过奇异面"

好问题——但严格地说，这个问句的前提需要修正：**RLinf 原代码并没有「做到」任何事来避开奇异面，它里面没有任何一个机制是为此设计的。** 「charger 从来没靠近过奇异面」是三个代码之外的因素叠加的结果，外加一个证据强度问题。逐层说。

## 1. 标定 protocol 本身就是一个「非奇异选择器」

`target_ee_pose` 的标定方式（`charger_sac_async.md` §6.2.1）：操作员**手扶引导键**，把夹着插头的臂推到「插到位」的姿态，读 `getpos_euler`。这个过程有个隐含性质——**人扶臂时 physically 走不到奇异位形附近**：`q5≈0` 时雅可比病态，臂在那个位形下某些方向会变得很难推（这是病态的物理手感），人会本能地绕开；而且人扶臂走的是关节空间路径，不会把臂拧到极限伸展。所以凡是用这个 protocol 标定的任务，目标位形天然落在条件数良好的区域。

这不是 RLinf 代码的功劳，是「人手引导」这个动作的性质。cube-place 的 H1 也是同样标定的——所以差异不在标定过程，在下面两条。

## 2. 任务摆位：charger 在近场，cube-place 在远场

这是真正的分叉点，而它是**物理摆位**，不是代码：

- **cube-place 的 H1 在 `x≈0.727m`**——Panda 最大伸展约 0.855m，这是 85% 伸展；盒角到 0.777（91%），LOG-036 的跳闸点在 `x=0.7788`。远伸展 + 法兰朝下（`roll≈−π`）这个组合，在运动学上**结构性要求腕关节伸直**——`q5` 小不是偶然，是这个位形的必然。悬停位形 `q5=0.18` 就已经偏小，策略把臂带向箱角时压到 0.085。
- **charger 的插座在演示里就摆在臂的正前方近处**。中等伸展 + 折叠腕（`q5≈π/2` 附近的舒适区）就能完成插入。他们的 `target_ee_pose` 也是占位符（YAML 里就是 `TARGET_EE_POSE`），具体值随他们的桌面摆位——但「插座摆在臂边」这个选择本身就保证了整个任务区域 `q5` 远离 0。

换句话说：不是 RLinf 代码选了好几何，是**他们把插座摆在了臂的舒适区里，而我们把标记摆在了 0.73m 外的伸展边缘**。

## 3. 工作空间大小决定「箱角会不会伸进近奇异区」

就算目标位形离奇异面有一定距离，策略的活动范围由 clip 盒子决定：

| | charger | cube-place |
|---|---|---|
| `clip_x/y` | ±**0.02** | ±0.05 |
| `clip_z_high` | +0.05 | +0.08 |
| 活动范围 | 目标周围 4×4×5cm 的小盒 | 10×10×8.5cm，盒角到 91% 伸展 |

charger 的盒子小到就算目标附近有条件数稍差的方向，臂也走不上去；cube-place 的盒角直接伸进了 92–98% 伸展区（LOG-034 的记录）。盒子是硬围栏，策略再探索也出不去——所以盒子放在哪、多大，就是任务会不会碰奇异面的决定变量。

## 4. 证据强度问题：「没靠近过」其实是「没测过」

上游没有 motion guard、没有 `q5` 记录、没有 `|dq|` 监控。如果 charger 训练里偶尔擦边近奇异位形，表现只会是跟踪误差拉大、或一次固件 reflex、或「学不动」——**没有任何日志会把它归因到奇异点**。所以严谨的表述是：charger 在它的几何里「没有可观测地」遭遇过这个问题。这和「从未发生」是两回事。

## 5. 还有一个领域惯例因素

charger 的任务谱系来自 SERL（rail-berkeley）的真机 RL 基准——PCB 插入、布线、插充电器。这类基准的设计目标是**对比学习算法**，不是挑战运动学极限，所以任务清一色近场、小盒子、舒适工作区。这个传统间接保护了所有复现者：照着做的人不会碰到奇异面，也就没人需要写防奇异的代码——**上游代码里没有这个机制，恰恰是因为它的任务从来不需要**。

## 对 T19 的含义

这个分析其实加强了 T19 候选修法②的理由：既然「远离奇异面」在上游是靠**任务摆位**实现的，那 cube-place 最治本的修法也是摆位/姿态层面的——把标记移近一点（减小伸展比）、或标定时刻意选一个肘部构型让 `q5` 余量更大（同一末端位姿下 elbow swivel 可以改变 `q5`，虽然 85%+ 伸展时 swivel 自由度会缩水）。而且有一个零成本的验证手段：引导模式下把臂推到盒子四个角各读一次 `q5`（`diag --probe` 会打印 `q`），就能画出任务区域的 `q5` 地图，直接量化「箱角离奇异面有多近」——不用改任何代码。

# 应该如何摆位去避免接近奇异点(基于上两章节)

好——这是把上一轮的归因变成可操作的摆位指南。核心思想一句话：**奇异面避不避得开，是由「标记摆在哪 + 标定时肘部怎么收 + 盒子多大」决定的，而这三个都可以在不动代码的情况下测量和选择。** 下面是完整流程。

## 判据先定下来：盒内任意点 `q5 ≥ 0.3 rad`（建议 ≥ 0.5）

参考标尺（都是本机实测值）：

| 位形 | `q5`（`probe q` 的第 5 个数） | 结局 |
|---|---|---|
| 舒适 ready 位形 | ≈1.57（π/2） | 最安全 |
| 现在 H1 的悬停位形 | 0.18 | 阶梯测试干净，但训练仍会跳闸 |
| LOG-036 跳闸点（x=0.7788） | **0.085** | lag 看门狗跳闸 |

所以目标：**任务盒内（含 8 个盒角 + 悬停层）能到达的每一点，`q5` 都不低于 0.3 rad**。注意不是只测目标点——两次跳闸都发生在「去盒角的路上」，盒角才是重点。

## 测量工具：引导模式 + `diag-probe`

- Desk 解锁、**Activate FCI**、方块夹紧（与训练同构型同载荷）、急停可及。
- 按住臂上引导键把臂推到待测位形，**保持不动**，另一个终端跑：

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
bash b/x/scripts/run_cube_place_phase2.sh diag-probe
# 读输出里的: probe q=[q1 q2 q3 q4 q5 q6 q7]
```

引导模式下 `robot_mode` 会显示 `Guiding`，这不影响读数（状态读取在任何模式都行）。probe 用子进程连接、读完即释放 FCI，可以反复跑。

## 一步一步摆位

**第 1 步：候选点初筛（只测目标高度）**
在台面上选 2–3 个候选标记位，从 **x≈0.45–0.55m**（径向伸展 ~55–65%）开始试，y 可以正前（0）或侧放（±0.15m）。对每个候选：引导臂到「方块放到标记上」的姿态（法兰朝下，`roll≈−π` 不变——这是任务要求），读 `q5`。
**目标点本身 `q5 < 0.3` 就直接淘汰**——盒角只会更差，不用往下测。

**第 2 步：画盒角 q5 地图**
对通过初筛的候选，引导臂依次到 10 个点各读一次 `q5`：

- 目标高度（z = 标记面）：中心 + xy 四角（±5cm）——5 点；
- 悬停高度（z = 标记 + 8cm）：中心 + 四角——5 点。

全程取 **min(q5)**。≥0.3 通过；否则把标记挪近 10cm 重测。推臂时留意手感：**哪个方向「发涩」，哪个方向就是条件数变差的方向**——这就是奇异面的方向，免费的定性反馈。

**第 3 步：选肘部构型（同一位置还能再榨出余量）**
同一个末端位姿，不同的肘部摆法（elbow swivel）会给不同的 `q5`。在最终选定的标记位，同一 TCP 位姿下试两种构型：肘明显弯曲收起（参考：舒适位形 `q4≈−2.36`；跳闸时 `q4=−0.42`，已经快伸直了）vs 肘外展，各读 `q5`，**取大者**。定性规则一句话：**肘部保持明显弯曲，不要让臂「伸直了够」**。
这个构型就是正式标定要用的构型——franky 的零空间弹簧锚定在 tracker 创建时的 `q`，你标定时的构型会被控制器记住并持续回拉。

**第 4 步：正式标定 H1 并同步三处**
用选定构型引导到「方块放到标记上」，用 REPL（`python b/x/scripts/test_franky_controller_ext.py`）读 `getpos_euler`，然后：

```bash
# franky 容器内
bash b/x/scripts/run_cube_place_phase2.sh write-pose x y z roll pitch yaw
```

再把新六元组同步到三处（`ee_pose_limits` 会由 config 自动推导，不用手算）：`cube_place_target_ee_pose.yaml`、`realworld_cube_place_camera.yaml`（train+eval）、`realworld_cube_place_sac.yaml`（train+eval）。**标定后台面与标记不能再挪动**——一动就必须重标（charger 文档的规矩同样适用）。

**第 5 步：验证链（顺序不要反）**

1. `connect`——几何打印 + 三道硬门，不动臂；
2. 相机验收脚本（H1 变了但相机 serial 没变，确认链路无恙）；
3. 2.4b 旋转阶梯在**新悬停位形**重跑——这是新基线，旧的四档全过是在旧位形上做的，不能外推；
4. `reset` / `box` 烟测；
5. 短训试跑，盯 watchdog 的 lag 跳闸与 `|dq|` 峰值。

## 旋钮与权衡（按有效性排序）

| 旋钮 | 效果 | 代价/约束 |
|---|---|---|
| **x 距离**（主旋钮） | 每近 10cm，`q5` 余量显著改善 | 必须实测，不要假设；相机视野要复核 |
| **肘部构型** | 同位置可多榨出 0.1–0.3 rad | 标定一次定型，之后靠零空间弹簧维持 |
| **yaw / y 侧放** | 立方体近似对称，绕 z 转 90°/180° 是自由变量；侧放有时比正前方构型好 | 要重新确认相机取景与操作员动线 |
| **缩盒子**（`clip_x/y` 0.05→0.03，`clip_z_high` 0.08→0.06） | 直接压缩风险区 | 探索空间变小，任务变难一点点 |

## 回退方案

如果台面物理上不允许挪标记（固定装置），那就回到 T19 候选①——在控制器或 `_clamp_step_slew` 里加**关节空间**的软件限速（预测/回读式 `|dq|` 钳制），那是代码方案，与摆位正交，也可以两者都做（防御纵深）。

需要的话，我可以把这套摆位流程固化成 `dmo_place_2.md` 的一个新小节（比如 §S2.x「任务摆位与 q5 地图」），并在 T19 条目里补上「摆位法」作为首选修法。


# 关键源码与文档索引

| 主题 | 路径 |
|------|------|
| Franka 真机文档（中文） | `docs/source-zh/rst_source/examples/embodied/franka.rst` |
| HG-DAgger 文档（中文） | `docs/source-zh/rst_source/examples/embodied/hg-dagger.rst` |
| 数据采集格式说明 | `docs/source-zh/rst_source/guides/data_collection.rst` |
| Catkin 工作空间安装 | `requirements/install.sh` → `install_franka_env()` |
| Ray 启动前环境 | `ray_utils/realworld/setup_before_ray.sh` |
| 真机采集启动脚本 | `examples/embodiment/collect_data.sh` |
| 真机采集逻辑 | `examples/embodiment/collect_real_data.py` |
| LeRobot 导出 wrapper | `rlinf/envs/wrappers/collect_episode.py` |
| Replay buffer 存储 | `rlinf/data/storage/replay/buffer.py` |
| 目标位姿测试脚本 | `toolkits/realworld_check/test_franka_controller.py` |
| Peg-insertion 环境 | `rlinf/envs/realworld/franka/tasks/peg_insertion_env.py` |
| Franka 环境 / 奖励 | `rlinf/envs/realworld/franka/franka_env.py` |
| Franky 关节/笛卡尔阻抗 tracker | `rlinf/envs/realworld/franka/franky_controller.py` |
| 扩展 Controller（compliance / move_arm） | `b/x/franky_ext/controller_extended.py` |
| Step 3 Controller smoke | `b/x/scripts/step3_test_controller.py` |
| 交互 Controller smoke | `toolkits/realworld_check/test_franky_controller.py` |
| OpenPI SFT 配置 | `examples/sft/config/realworld_sft_openpi.yaml` |
| HG-DAgger 训练配置 | `examples/embodiment/config/realworld_pnp_dagger_openpi.yaml` |

