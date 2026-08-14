
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
| OpenPI SFT 配置 | `examples/sft/config/realworld_sft_openpi.yaml` |
| HG-DAgger 训练配置 | `examples/embodiment/config/realworld_pnp_dagger_openpi.yaml` |

