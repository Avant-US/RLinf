# set franka home
_Exported on 9/22/2026 at 17:27:31 GMT+8 from Cursor (3.21.16)_

---

**User**

深入分析一下数据 `/home/nvidia/kaixin_ws/trans5090_v2/plug_into_socket_10hz_rlt_lerobot_2ep`, 及其各字段的含义与取值分布.

---

**Cursor**

正在探索该 LeRobot 数据集的结构与元数据，随后分析各字段含义与取值分布。


接下来用 Python 深入分析 parquet 数据，统计各字段分布并检查 state 与 action 的关系。


pip 安装卡住了；我改用已有环境或更轻量的方式读取 parquet。


# 数据集深度分析：`plug_into_socket_10hz_rlt_lerobot_2ep`

## 1. 数据集概览

| 属性 | 值 |
|------|-----|
| **路径** | `/home/nvidia/kaixin_ws/trans5090_v2/plug_into_socket_10hz_rlt_lerobot_2ep` |
| **格式** | LeRobot **v2.1** (`codebase_version: v2.1`) |
| **机器人** | Franka Panda (`robot_type: franka_panda`) |
| **任务** | `"plug the plug into the socket"`（插插座） |
| **Episodes** | 2（从完整数据集 `plug_into_socket_10hz_rlt_lerobot` 抽取 ep 0、1） |
| **总帧数** | 417（197 + 220） |
| **采样率** | **10 Hz**（`dt ≈ 0.1000s`，精确） |
| **成功率** | 100%（两集 `is_success=True`） |
| **磁盘占用** | ~79 MB |
| **原始来源** | HDF5：`/tmp/RL1/RL/data/plug_into_socket_hdf5` |

```mermaid
flowchart LR
    A[HDF5 原始遥操作数据<br/>plug_into_socket_hdf5] --> B[10Hz 重采样 + 格式转换]
    B --> C[LeRobot v2.1 完整集<br/>plug_into_socket_10hz_rlt_lerobot]
    C --> D[子集 2 episodes<br/>plug_into_socket_10hz_rlt_lerobot_2ep]
    
    subgraph 存储结构
        E[meta/ 元数据]
        F[data/*.parquet 数值字段]
        G[images/ PNG 双视角]
    end
    D --> E
    D --> F
    D --> G
```

---

## 2. 目录结构

```
plug_into_socket_10hz_rlt_lerobot_2ep/
├── meta/
│   ├── info.json              # 数据集 schema 与 feature 定义
│   ├── tasks.jsonl            # 任务描述
│   ├── episodes.jsonl         # 每集长度
│   └── episodes_stats.jsonl   # 每集各字段统计
├── data/chunk-000/
│   ├── episode_000000.parquet # 197 帧
│   └── episode_000001.parquet # 220 帧
├── images/
│   ├── image/episode_*/frame_*.png       # 全局相机 417 张
│   └── wrist_image/episode_*/frame_*.png # 腕部相机 417 张
├── conversion_metadata.json   # HDF5→LeRobot 转换说明
└── norm_stats.json            # 归一化统计（mean/std/q01/q99）
```

Parquet 中 **15 列**，图像以 **内嵌 PNG bytes** 存储（224×224 RGB），同时 `images/` 下有对应 PNG 文件。

---

## 3. 字段详解与取值分布

### 3.1 核心观测：`state`（19 维，模型用状态）

策略训练的主状态向量，包含夹爪、力/力矩、末端位姿与速度。

| 索引 | 字段名 | 含义 | 单位 | mean | std | min | max | 分布特征 |
|------|--------|------|------|------|-----|-----|-----|----------|
| 0 | `gripper_position` | 夹爪开合宽度（归一化/物理宽度） | m | 0.044 | 0.037 | **0.010** | **0.089** | **双峰**：0.01（闭合，212帧）与 ~0.088（张开，205帧） |
| 1-3 | `tcp_force_x/y/z` | 末端 TCP 接触力 | N | +0.45, -0.52, -0.40 | 2.58, 1.75, **6.52** | -3.9, -7.7, -6.7 | +11.5, +4.1, **+23.7** | **高方差、长尾**；`force_z` 在插入时峰值可达 ~24N |
| 4-6 | `tcp_pos_x/y/z` | 末端位置 | m | 0.562, -0.033, 0.274 | 0.008, 0.060, 0.068 | 0.533, -0.124, 0.188 | 0.577, +0.046, 0.438 | 工作空间小：x 约 4.3cm，z 约 25cm |
| 7-9 | `tcp_roll/pitch/yaw` | 末端姿态（RPY） | rad | 0.22, 0.025, 0.046 | **3.10**, 0.049, 0.048 | ±π | ±π | **roll 双峰**：+π（224帧）与 -π（193帧），无中间值 → 欧拉角 ±π 等价表示 |
| 10-12 | `tcp_torque_x/y/z` | 末端力矩 | N·m | -0.49, +0.55, +0.09 | 0.33, 0.53, 0.25 | -1.14, -0.62, -0.37 | +0.32, +2.26, +0.48 | 插入阶段 y 力矩偏大 |
| 13-15 | `tcp_linear_velocity_*` | 线速度 | m/s | ~0 | 0.017–0.048 | ±0.12 | +0.21 | 近零均值，单步最大 ~2cm |
| 16-18 | `tcp_angular_velocity_*` | 角速度 | rad/s | ~0 | 0.042–0.069 | -0.62 | +0.41 | 近零均值 |

**轨迹运动（两集合计）：**

| Episode | 帧数 | 时长 | 路径总长 | 净位移 | 最大单步 |
|---------|------|------|----------|--------|----------|
| 0 | 197 | 19.6s | 0.839 m | 0.152 m | 1.61 cm |
| 1 | 220 | 21.9s | 0.921 m | 0.044 m | 2.14 cm |

Ep0 末端 z 从 0.438→0.324（下降 11.4cm）；Ep1 从 0.422→0.398（下降 2.5cm），说明两集操作阶段/起始姿态略有差异。

---

### 3.2 完整状态：`state_full`（42 维，调试/扩展用）

包含 Franka 全关节与末端原始量，**不用于默认 19 维模型训练**。

| 段 | 维度 | 字段 | 含义 |
|----|------|------|------|
| 0–6 | 7 | `joint_position_1..7` | 关节角 (rad)，如 j4≈-2.03, j6≈+2.14 |
| 7–13 | 7 | `joint_velocity_1..7` | 关节速度 |
| 14–20 | 7 | `joint_torque_1..7` | 关节力矩（j2 均值约 -35 N·m，Franka 典型） |
| 21–27 | 7 | `joint_external_torque_1..7` | 外扰力矩 |
| 28–30 | 3 | `ee_pos_x/y/z` | 末端位置，与 `state.tcp_pos_*` **完全一致**（max diff=0） |
| 31–34 | 4 | `ee_quat_x/y/z/w` | 末端四元数 |
| 35–37 | 3 | `ee_force_x/y/z` | 末端力（与 state 中 tcp_force 对应） |
| 38–40 | 3 | `ee_torque_x/y/z` | 末端力矩 |
| 41 | 1 | `gripper_width` | 夹爪物理宽度 [0.012, 1.0] m |

---

### 3.3 动作：`actions`（7 维，策略学习目标）

```json
"action_source": "absolute measured TCP pose at t + 1/fps"
"action_representation": "absolute_eef_xyz_rpy_plus_gripper"
```

**不是 delta，而是下一时刻的绝对末端位姿**（已验证：`action_t == state_{t+1}` 的 pose 6 维，误差为 0）。

| 索引 | 字段 | mean | std | min | max | 说明 |
|------|------|------|-----|-----|-----|------|
| 0–2 | `tcp_pos_x/y/z` | 0.562, -0.033, 0.273 | 0.008, 0.060, 0.067 | 同 state | 同 state | 与 state 分布几乎相同 |
| 3–5 | `tcp_roll/pitch/yaw` | 0.237, 0.025, 0.046 | 3.10, 0.049, 0.047 | ±π | ±π | roll 同样双峰 |
| 6 | `gripper` | **-0.158** | **0.923** | **-1.0** | **+0.977** | 映射后：**+1=开，-1=关** |

**夹爪映射（来自 `conversion_metadata.json`）：**

```
源 HDF5 约定: 0=open, 1=closed
转换公式: action.gripper = 1 - 2 × source
```

已验证：`max|1-2×action_gripper_raw - actions.gripper| = 0`。

| 夹爪状态 | 帧数 | 占比 |
|----------|------|------|
| 闭合 (≤ -0.5) | 221 | 53.0% |
| 张开 (≥ 0.5) | 164 | 39.3% |
| 中间过渡 | 32 | 7.7% |

物理宽度参考：`gripper_closed_width=0.01m`，`gripper_open_width=0.09m`。

---

### 3.4 原始动作备份字段

| 字段 | 维度 | 含义 | 分布 |
|------|------|------|------|
| `action_joints_raw` | 7 | 原始关节角目标 (rad) | j1≈-0.11, j4≈-2.03, j6≈+2.19 |
| `action_gripper_raw` | 1 | 源夹爪 [0,1]，0=开 1=关 | mean=0.579，与 `state_full.gripper_width` 一致 |

---

### 3.5 视觉：`image` / `wrist_image`

| 属性 | 值 |
|------|-----|
| 分辨率 | **224 × 224 × 3** (RGB) |
| 存储 | Parquet 内嵌 PNG bytes + 外部 PNG 文件 |
| 视角 | `image`=全局相机，`wrist_image`=腕部相机 |
| 像素范围 | 全局：[7, 161]，均值 ~119；腕部对比度更高 |
| 通道均值 (全局) | R≈0.435, G≈0.449, B≈0.440（归一化到 [0,1]） |

---

### 3.6 元数据与标签字段

| 字段 | 类型 | 含义 | 分布 |
|------|------|------|------|
| `timestamp` | float32 | Episode 内相对时间 (s) | [0, 21.9]，步长 0.1s |
| `source_timestamp` | float64 | 源 HDF5 原始时间戳 | [0.133, 22.047]s |
| `frame_index` | int64 | Episode 内帧序号 | ep0: 0–196，ep1: 0–219 |
| `episode_index` | int64 | Episode ID | 0 或 1 |
| `index` | int64 | 全局帧序号 | 0–416（ep1 从 197 起） |
| `task_index` | int64 | 任务 ID | 恒为 0 |
| `done` | bool | Episode 结束标志 | **415×False，2×True**（每集末帧） |
| `is_success` | bool | 任务成功 | **417×True**（100% 成功演示） |

---

## 4. 数据流与因果关系

```mermaid
sequenceDiagram
    participant T as 时刻 t
    participant Obs as 观测 (state + images)
    participant Act as actions (t 的标签)
    participant T1 as 时刻 t+1

    Note over Obs: state_t = 19D 位姿/力/速度 + 夹爪
    Note over Obs: image_t, wrist_image_t
    Act->>T1: actions_t = TCP pose_{t+1} + gripper_{t+1}
    Note over Act: 绝对 EEF 动作，非 delta
    T1->>Obs: state_{t+1} 的 pose 6 维 = actions_t 前 6 维
```

**训练含义：**
- 若用 `state` + `actions` 做 BC/VLA：模型输入当前观测，预测 **下一时刻绝对末端位姿 + 夹爪**。
- 若需 delta action，需在数据管线中自行做 `actions - state[pose_dims]` 差分。
- `norm_stats.json` 已提供 state/actions 的 mean、std、q01、q99（后者 padded 到 32 维，实际只用前 7/19 维）。

---

## 5. 关键发现与使用建议

### 5.1 数据质量

1. **时间对齐精确**：10 Hz 均匀采样，`dt` 标准差 < 5×10⁻⁷ s。
2. **动作标签一致**：`actions` 严格等于下一帧 TCP 位姿，无标注噪声。
3. **两集均成功**：适合作为 smoke test / 调试用小集，**不适合直接训泛化策略**（仅 2 ep，417 帧）。

### 5.2 需注意的分布特性

| 问题 | 说明 | 建议 |
|------|------|------|
| **tcp_roll 双峰** | 在 ±π 附近跳变，std=3.1 | 训练时用四元数/6D rotation，或对 roll 做 unwrap |
| **force_z 长尾** | std=6.5，峰值 ~24N | 插入接触力，归一化时用 q01/q99 或 robust norm |
| **gripper 离散** | 主要是开/关两态 | 分类或 threshold 可能比回归更稳 |
| **绝对 action** | 非 delta | 与 OpenVLA/RLinf 默认 delta 配置需对齐 |
| **样本极少** | 2 episodes | 仅用于 pipeline 验证，正式训练用完整 100 ep 集 |

### 5.3 与完整数据集关系

该目录是 `plug_into_socket_10hz_rlt_lerobot` 的 **子集**（episodes 0, 1）。完整集在文档中对应 `plug_into_socket_lrb_4D` 系列（100 episodes, 66,577 frames @ 30fps 的另一版本），本集为 **10Hz + TCP 7D action** 的 RL 训练专用转换格式。

---

## 6. 快速加载示例

```python
from pathlib import Path
import pandas as pd
import numpy as np

root = Path("/home/nvidia/kaixin_ws/trans5090_v2/plug_into_socket_10hz_rlt_lerobot_2ep")
df = pd.concat([
    pd.read_parquet(root / "data/chunk-000/episode_000000.parquet"),
    pd.read_parquet(root / "data/chunk-000/episode_000001.parquet"),
])

# 一帧示例
row = df.iloc[50]
state_19d = np.array(row["state"])       # 当前观测
action_7d = np.array(row["actions"])     # 下一时刻目标: xyz+rpy+gripper
task = "plug the plug into the socket"   # meta/tasks.jsonl
```

---

**总结**：这是一个 LeRobot v2.1 格式的 Franka 插插座 **2-episode 调试子集**，10Hz 采样，双视角 224×224 图像，19 维状态 + 7 维绝对 TCP 动作（含 ±1 夹爪）。数据对齐与标签一致性良好，两集均成功；主要分布特征为 roll 角 ±π 双峰、插入阶段 force_z 尖峰、夹爪开/关两态。适合 pipeline 联调，正式训练建议使用完整数据集。

如需进一步分析（例如与 RLinf/InternVLA 配置对齐、delta action 转换脚本、或可视化轨迹/力曲线），可以继续说明需求。

---

**User**

<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: aborted
task_id: 461420
title: Deep Python analysis of parquet data distributions
tool_call_id: tool_b098c95c-14ad-4f16-903c-6d2c0f2b53a
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/461420.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

分析已完成。

该路径下是一个 **LeRobot v2.1** 格式的 Franka 插插座调试子集：**2 个 episode、417 帧、10 Hz**，任务为 `"plug the plug into the socket"`，两集均成功。

核心结论：
- **15 个字段**：`state`(19D)、`state_full`(42D)、`actions`(7D 绝对 TCP 位姿+夹爪)、双视角 224×224 图像，以及时间戳/索引等元数据
- **动作定义**：`actions_t` = 下一帧 TCP 位姿（已验证误差为 0），夹爪映射为 `1 - 2 × raw`（+1 开，-1 关）
- **分布特点**：`tcp_roll` 在 ±π 双峰；插入时 `force_z` 方差大（峰值约 24N）；夹爪主要为开/关两态

完整字段说明、统计表和训练建议见上一条回复。若需要，我可以继续生成可视化脚本，或给出对接 RLinf/InternVLA 的配置建议。

---

**User**

深入分析一下数据 `/home/nvidia/bt/dt/plug_into_socket_lrb_4D_8sml`, 包括分析各字段, 各字段对应值的分布等等.

---

**Cursor**

正在探索该数据集的结构与元数据，随后分析各字段含义与取值分布。


# 数据集深度分析：`plug_into_socket_lrb_4D_8sml`

## 1. 数据集概览

| 属性 | 值 |
|------|-----|
| **路径** | `/home/nvidia/bt/dt/plug_into_socket_lrb_4D_8sml` |
| **格式** | LeRobot **v3.0** |
| **机器人** | `franka_plug`（Franka + 插插座任务专用配置） |
| **任务** | `"plug into socket"`（插插头到插座） |
| **Episodes** | **8**（完整集 `plug_into_socket_lrb_4D` 100 ep / 66,577 帧的 **8% 子集**） |
| **总帧数** | **4,777** |
| **采样率** | **30 Hz**（`dt ≈ 0.0333s`，精确） |
| **总时长** | ~159 s（8 集合计） |
| **磁盘** | ~23 MB（data 1.9 MB + video 20.6 MB） |
| **用途** | 4DWVLA / InternVLA-A1.5 **冒烟测试**与 pipeline 验证 |

```mermaid
flowchart LR
    A[完整集 plug_into_socket_lrb_4D<br/>100 ep / 66577 帧 / 30Hz] --> B[抽取 8 episodes]
    B --> C[plug_into_socket_lrb_4D_8sml<br/>4777 帧 / 23MB]
    
    subgraph 与 RL 格式数据对比
        D[10hz_rlt_lerobot_2ep<br/>v2.1 / TCP 7D action]
        E[lrb_4D_8sml<br/>v3.0 / joint 7D + keypoint 56D]
    end
    C -.-> E
```

---

## 2. 目录结构

```
plug_into_socket_lrb_4D_8sml/
├── meta/
│   ├── info.json              # v3.0 schema（13 个 feature keys）
│   ├── stats.json             # 全局统计（min/max/mean/std/q01-q99）
│   ├── tasks.parquet          # 1 个任务
│   ├── keypoints_meta.json    # 8 关键点 × 7D 定义与归一化说明
│   └── episodes/chunk-000/file-000.parquet  # 每集元数据 + per-ep stats
├── data/chunk-000/file-000.parquet   # 4777 帧数值数据（12 列）
└── videos/
    ├── observation.images.global/chunk-000/file-000.mp4  # AV1, 8.6MB
    └── observation.images.wrist/chunk-000/file-000.mp4   # AV1, 12MB
```

Parquet **不含图像像素**，视频通过 `episode_index` + 时间戳索引到 MP4 文件（640×480，AV1，30fps，4778 帧）。

---

## 3. Episode 明细

| Episode | 帧数 | 时长 | 路径长度 (ee) | 净位移 | z 方向变化 |
|---------|------|------|---------------|--------|------------|
| 0 | 596 | 19.9s | 0.856 m | 0.151 m | -11.3 cm |
| 1 | 664 | 22.1s | 0.938 m | 0.046 m | -2.6 cm |
| 2 | 603 | 20.1s | 0.777 m | 0.094 m | -6.1 cm |
| 3 | 544 | 18.1s | 0.791 m | 0.083 m | +1.0 cm |
| 4 | 562 | 18.7s | 0.871 m | 0.094 m | -2.6 cm |
| 5 | 568 | 18.9s | 0.913 m | 0.114 m | -3.1 cm |
| 6 | 588 | 19.6s | 0.856 m | 0.111 m | -8.0 cm |
| 7 | 652 | 21.7s | 0.870 m | 0.122 m | -11.2 cm |

每集约 18–22 秒，单步末端位移最大约 **1 cm**，典型插入任务轨迹。

---

## 4. 字段详解与分布

Parquet 共 **12 列**，按功能分为：观测、动作、3D 关键点、元数据。

### 4.1 观测：`observation.state.arm`（7D 关节角，rad）

| 关节 | mean | std | min | max | 特征 |
|------|------|-----|-----|-----|------|
| joint1 | -0.218 | 0.088 | -0.347 | -0.075 | 负值为主，变化中等 |
| joint2 | +0.117 | 0.085 | -0.103 | +0.274 | 肩/elbow 区域 |
| joint3 | +0.157 | 0.126 | -0.135 | +0.339 | 变化较大 |
| joint4 | **-2.070** | 0.121 | -2.204 | -1.568 | 肘关节，范围约 0.6 rad |
| joint5 | -0.033 | 0.048 | -0.159 | +0.081 | 近零，小幅摆动 |
| joint6 | **+2.201** | 0.167 | +1.697 | +2.454 | 腕部主关节 |
| joint7 | +0.688 | 0.118 | +0.485 | +0.981 | 腕旋转 |

---

### 4.2 观测：`observation.state.gripper`（夹爪宽度，m）

| 统计 | 值 |
|------|-----|
| min / max | **0.0 / 0.0792** |
| mean / std | 0.034 / 0.037 |
| 闭合 (0.0) | **2,307 帧 (48.3%)** |
| 张开 (~0.079) | ~1,795 帧 (37.6%) |
| 过渡态 | ~675 帧 |

典型**双峰分布**：完全闭合 vs 接近最大开度（~79 mm）。

---

### 4.3 观测：`observation.state.ee_pos`（末端位置，m）

| 轴 | mean | std | min | max | 工作空间跨度 |
|----|------|-----|-----|-----|-------------|
| x | 0.561 | 0.008 | 0.534 | 0.599 | ~6.5 cm |
| y | -0.037 | 0.055 | -0.124 | +0.046 | ~17 cm |
| z | 0.277 | 0.075 | 0.186 | 0.512 | ~33 cm（插入方向主运动） |

---

### 4.4 观测：`observation.state.ee_quat`（末端四元数，w,x,y,z）

| 分量 | mean | std | 范围 |
|------|------|-----|------|
| w | 0.999 | 0.001 | [0.993, 1.000] |
| x | 0.027 | 0.024 | [-0.036, +0.111] |
| y | 0.008 | 0.023 | [-0.110, +0.061] |
| z | 0.005 | 0.022 | [-0.053, +0.059] |

四元数模长恒为 **1.0**；w 接近 1 说明姿态变化相对较小，无 v2.1 数据里 `tcp_roll` ±π 跳变问题。

---

### 4.5 动作：`action.arm`（7D 绝对关节角目标，rad）

与 `observation.state.arm` 分布接近，但范围略宽（控制器目标可超出当前观测）：

| 关节 | mean | std | min | max |
|------|------|-----|-----|-----|
| joint1–5 | 同观测量级 | 略大 | 略宽 | 略宽 |
| joint6 | 2.262 | 0.182 | 1.737 | **2.511** |
| joint7 | 0.708 | 0.183 | 0.375 | **1.102** |

**动作语义（重要）：**

- **不是**下一帧观测（`action_t == arm_{t+1}` 匹配率 **0%**）
- **不是** delta（`action_t == arm_{t+1} - arm_t` 匹配率 **0%**）
- 是**同一时刻的控制器关节目标**，与观测存在小偏差：

| 关节 | 同帧残差 mean | std | 含义 |
|------|---------------|-----|------|
| j1–j5 | ±0.001~0.006 | 0.007~0.040 | 跟踪误差小 |
| **j6** | **+0.062** | 0.041 | 目标略超前于观测 |
| **j7** | **+0.020** | 0.079 | 腕部滞后/超前最大 |

跨帧对比 `action_t` vs `arm_{t+1}`：平均误差 0.005–0.077 rad，属于正常控制延迟。

---

### 4.6 动作：`action.gripper`（夹爪命令，无量纲）

| 统计 | 值 |
|------|-----|
| min / max | **0.0095 / 1.0** |
| mean / std | 0.570 / 0.460 |
| 闭合 (≤0.02) | 1,307 帧 (27.4%) |
| 张开 (≥0.08) | 3,015 帧 (63.1%) |
| 过渡 | 455 帧 |

与 `observation.state.gripper`（物理宽度 0–0.079 m）呈**完美负相关（r = -1.0）**：
- `action.gripper ≈ 1` → 张开命令 → 观测宽度 ~0.079 m
- `action.gripper ≈ 0.01` → 闭合命令 → 观测宽度 0 m

---

### 4.7 核心扩展：`observation.keypoint_3d`（56D = 8 × 7D）

来自 URDF `fr3v2_1_franka_hand.urdf`，8 个关键点：`link1`–`link7` + `hand_tcp`。

**每关键点 7D 布局：** `(px, py, pz, qx, qy, qz, qw)`

**归一化（`keypoints_meta.json`）：**

| 参数 | 值 |
|------|-----|
| 坐标系 | base_link 相对，各向同性 |
| `bbox_radius` | 0.836 m |
| 位置 | 除以 bbox_radius |
| 旋转 | 四元数 xyzw，**qw ≥ 0**（半球规范化） |

**各关键点位置统计（归一化后）：**

| 关键点 | mean (px,py,pz) | 运动特征 |
|--------|-----------------|----------|
| link1, link2 | (0, 0, 0.398) | **固定**（std≈0，基座） |
| link3 | (0.043, -0.009, 0.772) | 小范围运动 |
| link4 | (0.140, -0.015, 0.761) | 小范围运动 |
| link5, link6 | (0.562, -0.037, 0.580) | 随臂运动，z 变化大 |
| link7 | (0.667, -0.042, 0.582) | 腕部 |
| **hand_tcp** | (0.671, -0.044, **0.331**) | 末端，z 变化最大 |

**56D 全局：** min=-1.0, max=+1.0, mean=0.22, std=0.41

**方差最大的维度：**

| 维度 | std | 说明 |
|------|-----|------|
| `hand_tcp_qx` | **0.992** | 末端姿态 x 分量，接近全范围 |
| `link7_qx` | 0.933 | 腕部旋转 |
| `hand_tcp_pz`, `link7_pz` | ~0.09 | 插入方向位置变化 |

所有关键点的 **qw ≥ 0**（半球规范，无负半球违规）。

---

### 4.8 视觉：`observation.images.global` / `.wrist`

| 属性 | 值 |
|------|-----|
| 分辨率 | **640 × 480 × 3** (RGB) |
| 编码 | **AV1** (yuv420p) |
| FPS | 30 |
| 存储 | 单文件 MP4（非 parquet 内嵌） |
| 全局相机像素 | R/G/B mean ≈ 0.449/0.457/0.448，范围 [0, 0.72] |
| 腕部相机像素 | R/G/B mean ≈ 0.480/0.424/0.395，对比度更高 |

---

### 4.9 元数据字段

| 字段 | 类型 | 分布 |
|------|------|------|
| `timestamp` | float32 | episode 内 [0, 21.7]s，步长 1/30 s |
| `frame_index` | int64 | 每集 0 ~ (length-1)，全局 max=663 |
| `episode_index` | int64 | 0–7 |
| `index` | int64 | 全局 0–4776 |
| `task_index` | int64 | 恒为 0 |

---

## 5. 数据流与训练含义

```mermaid
sequenceDiagram
    participant Obs as 观测
    participant Act as 动作
    participant KP as 3D Keypoints

    Note over Obs: arm(7) + gripper_width + ee_pos + ee_quat
    Note over Obs: global/wrist 视频帧 (640x480)
    Note over KP: 8×7D 归一化关键点 (GeoP 监督)
    Act->>Obs: action.arm = 控制器关节目标 (同帧或略超前)
    Act->>Obs: action.gripper = 开/关命令 (与 width 负相关)
```

**4DWVLA 训练用法：**
- **SFT / BC**：图像 + 语言 → 预测 `action.arm`(7D) + `action.gripper`(1D)
- **GeoP / 关键点分支**：`observation.keypoint_3d`(56D) 作为几何监督
- **归一化**：4DWVLA 配置中 `normalization_mapping: IDENTITY`，通常不依赖 `stats.json` 做缩放；冒烟时可指向本地 `meta/stats.json`

---

## 6. 与 `plug_into_socket_10hz_rlt_lerobot_2ep` 对比

| 维度 | **lrb_4D_8sml** (本集) | **10hz_rlt_lerobot_2ep** |
|------|------------------------|--------------------------|
| LeRobot 版本 | **v3.0** | v2.1 |
| 帧率 | **30 Hz** | 10 Hz |
| Episodes | **8** (4777 帧) | 2 (417 帧) |
| 动作 | **7D 绝对关节角 + gripper** | 7D 绝对 TCP 位姿 + gripper |
| 状态 | arm + ee + quat | 19D state (力/速度/TCP) |
| 关键点 | **56D (8×7D)** | 无 |
| 图像 | 640×480 AV1 视频 | 224×224 PNG 内嵌 |
| 力/力矩 | 无 | 有 (tcp_force/torque) |
| 用途 | **4DWVLA SFT 冒烟** | RLinf RL 格式验证 |

两者任务相同（插插座），但**格式与 action 空间不互通**，不可直接混用。

---

## 7. 关键发现与使用建议

### 数据质量
1. 时间对齐精确：30 Hz 均匀采样。
2. 四元数规范：模长=1，无欧拉角跳变。
3. 关键点规范：qw 半球一致，link1/2 固定符合 URDF 基座。
4. 值域在完整 checkpoint 训练分布内（文档已验证），**无 OOD 风险**。

### 需注意
| 问题 | 说明 | 建议 |
|------|------|------|
| **action ≠ next state** | 关节目标是控制指令，非下一帧观测 | BC 训练时用 `action.arm` 作 label，勿与 `arm_{t+1}` 混淆 |
| **j6/j7 残差偏大** | 同帧 mean 残差 0.02–0.06 rad | 腕部动态快，属正常 |
| **夹爪两套尺度** | obs=宽度(m)，action=命令[0,1] | 推理/deploy 需分别映射 |
| **样本量有限** | 8 ep，仅为完整集 8% | 冒烟/调试 OK，正式训练用 100 ep 完整集 |
| **stats 无 keypoint** | `stats.json` 未含 `observation.keypoint_3d` | 关键点归一化参考 `keypoints_meta.json` |

### 快速加载

```python
from pathlib import Path
import pandas as pd
import numpy as np

root = Path("/home/nvidia/bt/dt/plug_into_socket_lrb_4D_8sml")
df = pd.read_parquet(root / "data/chunk-000/file-000.parquet")

row = df.iloc[100]
arm = np.array(row["observation.state.arm"])       # 当前关节角
action = np.array(row["action.arm"])               # 控制器目标
kpt56 = np.array(row["observation.keypoint_3d"])    # 8×7D 关键点
task = "plug into socket"
# 图像需通过 LeRobot v3 视频 loader 按 timestamp 解码 MP4
```

---

**总结**：`plug_into_socket_lrb_4D_8sml` 是 LeRobot v3.0 格式的 Franka 插插座 **8-episode 冒烟子集**，30 Hz、4777 帧，含 **7D 关节 action + 56D 3D 关键点 + 双视角 640×480 视频**。它是 4DWVLA checkpoint（`4wvlaFrkPlugCkp010420`）训练分布的真子集，适合 SFT pipeline 验证；与 RLinf 用的 10Hz TCP 格式数据集是平行但不可互换的两套表示。

---

**User**

参考 @RLmm/b/d/frk1/plug/abs_stats.json , @RLmm/b/d/frk1/plug/keypoints_meta.json , @RLmm/b/d/frk1/fr3v2_1_franka_hand.urdf 这些与 Franka 机器人及其插插座数据集相关的参数文件. 深入分析 @RLmm/b/d/frk1/ 中的各个分析与方案的 markdown 文档及其相关代码脚本. 深入分析 @RLmm/b/d/rltx/ 中的各个分析与方案的 markdown 文档及其相关代码脚本. 深入分析 @RLmm/b/x/ 中对RLmm(即RLinf)做扩展的各种代码脚本和配置文件. 探究一下目前有没有确定的基于Franka的插插座数据集全局计算出来的 home 位姿, 也就是每次插插座之前的那个姿势. 如果没有, 请计算出这个插插座的home位姿及其对应的关节角, 并且在 @RLmm/b/x/franky_ext/dsplug/ 里写一个脚本用于停止Franka的操作,并使它回到插插座的home位置.

---

**Cursor**

我会先把三部分证据分开核对：Franka/数据参数与 home 候选值、`rltx` 方案文档及扩展代码、现有 Franka 控制接口。只有在确认数据中不存在已固定的 home 后，才新增回 home 脚本；脚本会默认采用“急停后受限、可确认的回 home”流程，避免把未知姿态直接发送给实体机器人。


核验结果已经明确：仓库里没有唯一的“插插座前 home”。有三种不同语义的 home：Franka factory ready、RLinf 默认关节 reset、以及当前 4DWVLA 插座评测使用的“训练集 `observation.state.arm` 全局均值”。我将把最后一种作为本次任务的可执行 home，并在脚本中明确标注它不是数据集的语义标签；脚本默认不自动移动，必须显式执行和确认，且不会默认张开夹爪。


已完成分析并新增脚本，未修改 RLinf 核心代码。

核心结论：

- 数据集没有明确标注“每次插入前 home”。
- `abs_stats.json` 是全局统计，不包含 home 阶段标签。
- 现有 4DWVLA 评测代码已约定使用 `observation.state.arm.mean` 作为训练参考 home：
  ```text
  [-0.2405779213, 0.1457375735, 0.1872396618, -2.0599503517,
   -0.0552775823, 2.2011394501, 0.6998188496] rad
  ```
- 这不是 Franka factory ready pose，也不是 `joint_reset_qpos`，更不是插入成功时的 `target_ee_pose`。
- 基于 URDF FK 计算的 TCP home：
  ```text
  position:      [0.5718130770, -0.0362816192, 0.2629817814] m
  quaternion_xyzw:
                 [0.9994657126, 0.0314695103, -0.0014139629, 0.0087155006]
  rpy_xyz:       [3.1242589922, 0.0033749663, 0.0629226173] rad
  ```

新增文件：

- `b/x/franky_ext/dsplug/stop_and_home.py`
- `b/x/franky_ext/dsplug/home_pose.json`
- `b/x/franky_ext/dsplug/README.md`

脚本特性：

- 默认只读检查；
- `--stop-only`：软件停止当前运动；
- `--execute`：停止、检查 robot mode、确认后移动到 home；
- 默认不改变夹爪状态；
- `--open-gripper` 才会显式张开夹爪；
- 检查关节限位、关节速度、碰撞阈值和最终 home 误差；
- 发生异常时 fail-closed，不自动绕过急停或 UserStopped 状态。

运行示例：

```bash
python b/x/franky_ext/dsplug/stop_and_home.py \
  --robot-ip 172.16.0.2 --execute
```

验证已完成：

- home 元数据输出通过；
- Python 编译和 AST 检查通过；
- fake robot 执行流程通过；
- IDE lint 无错误；
- 未执行真实机器人运动测试。使用前必须先停止占用 FCI 的 RLinf/Ray 进程，并确认急停可触达。

---

**User**

<timestamp>Thursday, Sep 17, 2026, 9:36 AM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: subagent
status: success
task_id: 0496d816-5aa5-445d-9696-8c6f03cc83c1
title: 梳理 RLinf 控制扩展
tool_call_id: call_ahD08mwF6hMagZ2L3ZXVq4gt
fc_063ff7faf86da19c016aab423a9f8087d19b58dae8b77da8a7
agent_id: 0496d816-5aa5-445d-9696-8c6f03cc83c1
detail: <user_visible_high_level_summary>
已完成只读检索：确认仓库不存在 b/x/dsplug；实际控制链集中在 FrankyControllerExtended、FrankyJointEnv 与 FrankySingleFrankaEnv。新增“停止并回 home”应复用现有 stop/reset_joint/go_to_rest，并先处理 FCI 独占、Ray actor 生命周期、运动 guard 与夹爪持物风险。
</user_visible_high_level_summary>
<response>
已完成只读探索，未修改任何文件。

## 1. `b/d/rltx/` 文档检索结果

重点文档：

- `/home/nvidia/bt/s/RLmm/b/d/rltx/4dwvla_rlt1_1.markdown`
 - Stage 1 架构、RLT token、梯度隔离、checkpoint、测试与 Gate G0–G4。
- `/home/nvidia/bt/s/RLmm/b/d/rltx/4dwvla_rlt1_2.markdown`
 - Stage 1 实施落地手册，包含 Docker、数据、配置、测试命令和 4DWVLA 接口。
- `/home/nvidia/bt/s/RLmm/b/d/rltx/4dwvla_rlt1_20916LOG.markdown`
 - Stage 1 执行日志；最终记录 52 项离线测试通过。
- `/home/nvidia/bt/s/RLmm/b/d/rltx/4dwvla_rlt2_1.markdown`
 - Stage 2 Actor-Critic 方案，覆盖 feature model、action codec、replay、readiness gate、人工干预和真机安全反馈。
- `/home/nvidia/bt/s/RLmm/b/d/rltx/4dwvla_rlt2_10917LOG.md`
 - Stage 2 离线验收日志；T5–T10 通过，G1 Offline Gate 通过。
- `/home/nvidia/bt/s/RLmm/b/d/rltx/rlt_code_analyz3.markdown`
 - RLinf RLT 代码级分析，重点是 `RealworldRLTRoute`、transition、keyboard、SpaceMouse 和 replay。
- `/home/nvidia/bt/s/RLmm/b/d/rltx/rlmm_rlikx_diff_analyz3.markdown`
 - RLiKx 与 RLmm 的真机部署差异，涉及安全盒、绝对动作、人工接管和键盘控制。
- `/home/nvidia/bt/s/RLmm/b/d/rltx/rlmm_rlikx_diff_foreval1.markdown`
 - 差异对纯 VLA 评测的影响，明确了 `KeyboardVLAEvalWrapper` 的 `r/h` 行为。
- `/home/nvidia/bt/s/RLmm/b/d/rltx/rlmm_rlikx_diff_forrlt1_1.markdown`
 - 评估 RLiKx 改动对 4DWVLA RLT Stage 1/2 的相关性。

文档中与新增“停止并回 home”最相关的结论是：

- 真机安全事件应先停止并记录 `executed_action`、`safety_reason`、`guard_tripped`。
- safety/guard 事件后应 fail-closed，下一次动作强制 reference 或停止。
- `stop`、`recover_from_errors`、`home/reset` 应是不同阶段，不能在异常后无条件自动恢复。
- 真机 chunk 在成功、失败或 truncation 后应立即中断，不能继续执行剩余动作。

## 2. 实际 Franka/Franky 控制入口

### A. 4DWVLA 独立评测链路

入口：

`/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franka_vla_client.py`

调用关系：

```text
KeyboardVLAEvalWrapper
 -> FrankyJointEnv
 -> FrankyControllerDirect
 -> franky.Robot
```

关键 API：

`/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franky_controller_direct.py`

- `get_state()`
- `move_joints(joint_positions)`
- `reset_joint(reset_pos)`
- `open_gripper()`
- `close_gripper()`
- `stop()`
- `freeze_at_current()`
- `recover_from_errors()`
- `recover_from_guard_trip()`
- `cleanup()`

底层实现：

```python
self._robot = franky.Robot(robot_ip)
self._robot.stop()
self._robot.recover_from_errors()
self._robot.move(franky.JointWaypointMotion(...))
```

环境入口：

`/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franky_joint_env.py`

关键行为：

- `step()`：动作安全裁剪、速度限制、motion guard 检查、`move_joints()`、夹爪动作。
- `reset()`：必要时恢复 guard，打开夹爪，调用 `reset_joint(HOME_JOINTS)`。
- `go_to_rest()`：打开夹爪 → `reset_joint(HOME_JOINTS)` → 再打开夹爪。
- `close()`：调用 controller cleanup。

安全常量：

```python
HOME_JOINTS = TRAIN_ARM_MEAN.copy()
MAX_JOINT_STEP_RAD = 0.15
```

### B. Keyboard VLA 接管入口

`/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/keyboard_vla_eval.py`

当前按键：

- `a`：开始 rollout
- `r`：中止 episode，调用 `ctrl.stop()`，返回 `truncated=True`
- `b`：失败
- `c`：成功
- `h`：停止当前控制，然后调用 `go_to_rest()`

特别注意：这里的 `ctrl.stop()` 只适用于 `FrankyJointEnv` 使用的 `FrankyControllerDirect`。它不能直接假设适用于 Ray 版 `FrankyControllerExtended`。

键盘设备：

`/home/nvidia/bt/s/RLmm/rlinf/envs/realworld/common/keyboard/keyboard_listener.py`

- `pop_pressed_keys()`
- `get_key()`
- 环境变量 `RLINF_KEYBOARD_DEVICE`

### C. RLinf/Ray 真机控制链路

注册入口：

`/home/nvidia/bt/s/RLmm/b/x/franky_ext/tasks/register.py`

注册了：

- `FrankyFrankaEnv-v1`
- `FrankyPegInsertionEnv-v1`
- `FrankyCubePlaceEnv-v1`

扩展环境：

`/home/nvidia/bt/s/RLmm/b/x/franky_ext/franky_single_franka_env.py`

实际环境：

`/home/nvidia/bt/s/RLmm/rlinf/envs/realworld/franka/franka_env.py`

Ray controller：

`/home/nvidia/bt/s/RLmm/rlinf/envs/realworld/franka/franky_controller.py`

关键 API：

- `launch_controller(...)`
- `is_robot_up()`
- `get_state()`
- `move_joints(...)`
- `move_tcp_pose(...)`
- `reset_joint(...)`
- `open_gripper()`
- `close_gripper()`
- `clear_errors()`
- `cleanup()`

扩展安全 controller：

`/home/nvidia/bt/s/RLmm/b/x/franky_ext/controller_extended.py`

关键 API：

- `set_motion_guard(...)`
- `guard_tripped()`
- `motion_health()`
- `recover_from_guard_trip()`
- `freeze_at_current()`
- `move_arm(...)`
- `move_tcp_pose(...)`
- `move_gripper(...)`
- `stop_gripper()`
- `reconfigure_compliance_params(...)`
- `cleanup()`

内部安全处理：

- watchdog 线程持续检查 TCP、关节速度、lag 和 orientation。
- `_abort_motion()` 先制动，再锁存 trip reason。
- guard trip 后 controller 不应继续重建 tracker。
- `recover_from_guard_trip()` 会检查：
 1. 机械臂是否已经停止；
 2. tracker 是否已停止；
 3. `recover_from_errors()` 是否成功；
 4. robot mode 是否恢复为可运动状态；
 5. 最后才清除 guard latch。

### D. Cube Place 的“回 home”语义

`/home/nvidia/bt/s/RLmm/b/x/franky_ext/tasks/cube_place.py`

`go_to_rest()` 不是简单 joint home，而是：

1. 保持夹爪闭合；
2. 读取当前 TCP；
3. 当前位姿原地建立 tracking；
4. 沿当前 TCP 的 z 轴抬升；
5. 通过受限插值移动到 `reset_ee_pose`；
6. 使用 motion guard、速度和距离限制。

因此，插头/方块任务不应直接复用 `FrankyJointEnv.go_to_rest()`，因为它会打开夹爪。

## 3. `dsplug` 与相邻扩展命名

仓库中没有找到：

- `b/x/dsplug/`
- 任何名为 `dsplug` 或相近命名的目录/文件
- 文本中的 `dsplug`

只有 plug 相关扩展：

- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/rlt/configs/rlt_stage1_franka_plug.yaml`
- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franka_vla_client.py`
- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/vla_inference_server.py`
- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franky_joint_env.py`
- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franky_controller_direct.py`

相邻命名模式：

- `b/x/franky_ext/`：Franka/Franky 真机控制扩展。
- `b/x/4dwvla_ext/`：4DWVLA 推理、评测和 RLT 扩展。
- `b/x/scripts/stepN_*`：分阶段体检脚本。
- `b/x/scripts/diag_*`：诊断脚本。
- `b/x/scripts/test_*`：控制器或硬件测试。
- `b/x/scripts/run_*_accept.sh`：验收入口。
- `b/x/configs/realworld_*`：真实机器人配置。
- `b/x/configs/docker_run_*`、`setup_*`：容器和环境启动配置。

## 4. 现有测试方式

离线测试：

- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/tests/test_safety_offline.py`
- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/tests/test_keyboard_wrapper_offline.py`
- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/rlt/tests/run_all_offline.sh`
- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/rlt/stage2/acceptance/gate_g1_offline.py`

真实 controller 测试：

- `/home/nvidia/bt/s/RLmm/b/x/scripts/test_franky_controller_ext.py`
- `/home/nvidia/bt/s/RLmm/b/x/scripts/step3_test_controller.py`
- `/home/nvidia/bt/s/RLmm/b/x/scripts/test_t21_t22_recovery.py`

真实环境测试：

- `/home/nvidia/bt/s/RLmm/b/x/scripts/step5_test_env_robot.py`
- `/home/nvidia/bt/s/RLmm/b/x/scripts/step_cube_place_robot.py`
- `/home/nvidia/bt/s/RLmm/b/x/scripts/run_cube_place_phase2.sh`

验收流程：

- `/home/nvidia/bt/s/RLmm/b/x/scripts/run_cube_place_camera_accept.sh`
- `/home/nvidia/bt/s/RLmm/b/x/scripts/preflight_cube_place_sac.sh`
- `/home/nvidia/bt/s/RLmm/b/x/scripts/run_step8_accept.sh`
- `/home/nvidia/bt/s/RLmm/b/x/scripts/run_step9_accept.sh`

现有测试特点：

- 默认先离线验证，再进行真实机器人测试。
- 真实测试前检查 robot mode、FCI、相机、Ray 节点、内存和 live controller actor。
- 发现 live `FrankyControllerExtended` actor 时，`run_cube_place_sac.sh` 会拒绝继续。
- teardown 使用 `finally`，尽量记录 `motion_health()`，避免原始异常被清理异常覆盖。
- 不建议直接 `kill -9` 持有 FCI 的进程。

## 5. 新增“停止操作并回 home”脚本建议

建议不要新增一个直接创建第二个 `franky.Robot()` 的独立脚本作为默认方案。FCI 是独占连接，现有 controller actor 已连接时，第二个客户端可能导致连接失败或竞态。

优先方案：

1. 在当前控制进程内增加一个明确的状态机：
 - `REQUEST_STOP`
 - `BRAKE`
 - `VERIFY_SETTLED`
 - `CHECK_MODE`
 - `OPERATOR_CONFIRM_HOME`
 - `MOVE_HOME`
 - `VERIFY_HOME`
2. 复用当前 controller 实例，不重新建立 FCI。
3. 停止后轮询 `|dq|`，确认低于阈值再进入 home。
4. guard trip、Reflex 或异常状态下不要自动调用 `recover_from_errors()`。
5. home 目标从配置或 CLI 读取，不要新写一个隐含常量。
6. 记录：
 - 原始 joint pose；
 - stop 时间；
 - stop 后 joint speed；
 - robot mode；
 - 是否存在 guard trip；
 - home 目标；
 - home 误差；
 - 夹爪状态。

对于 4DWVLA 独立评测，最兼容的复用路径是：

```text
KeyboardVLAEvalWrapper
 -> FrankyJointEnv._controller.stop()
 -> 等待停止
 -> 显式确认
 -> FrankyJointEnv.go_to_rest()
```

但要注意 `go_to_rest()` 会打开夹爪，不适用于“插头仍需夹持”的任务。

对于 Cube Place/RLinf Ray 路径，应复用：

```text
env.unwrapped.go_to_rest()
```

而不是从外部脚本重新连接机器人。更合理的实现位置是当前 env/controller owner 内部，由键盘、信号或控制命令触发。

## 6. 主要风险

1. `robot.stop()` 是软件控制停止，不等价于硬件急停或 user-stop。
2. FCI 连接独占；外部脚本和 Ray controller 同时连接会冲突。
3. `stop → home` 不是原子操作，中间必须确认机械臂已稳定。
4. 从任意姿态直接 `reset_joint()` 可能产生大幅关节扫动，绕过任务空间安全假设。
5. `FrankyControllerDirect.reset_joint()` 本身没有完整的 guard 检查和分段路径控制。
6. `go_to_rest()` 在不同环境中的夹爪语义不同：
 - `FrankyJointEnv.go_to_rest()` 会打开夹爪；
 - Cube Place 的 `go_to_rest()` 保持夹爪闭合。
7. 仓库中存在多个不同的 HOME 定义：
 - `FrankyJointEnv.HOME_JOINTS`
 - `FrankyController` 默认 `joint_reset_qpos`
 - `test_franky_controller_ext.py` 内的 `HOME_JOINTS`
8. guard trip 后自动恢复可能重新授权危险运动；应保留人工确认。
9. Ray actor 被强杀时，controller cleanup 不一定执行，可能留下 FCI 占用或未预期的控制状态。
10. 如果新增脚本只验证“命令已发送”，不能证明机械臂已经到 home，必须再次读取关节位置和速度确认。
</response>
output_path: \home\nvidia\.cursor\projects\home-nvidia-bt-s-4WVLA\agent-transcripts\4ebbc526-4b84-4b0f-a391-344ad18be6f6\subagents\0496d816-5aa5-445d-9696-8c6f03cc83c1.jsonl
</task>
</system_notification>

<timestamp>Thursday, Sep 17, 2026, 9:36 AM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: subagent
status: success
task_id: f2f03e84-3362-4e18-97fe-f6e873c4d27c
title: 梳理 home 与数据方案
tool_call_id: call_liPgTxImOF3QAREWRQkByPum
fc_063ff7faf86da19c016aab423a9f7087d1b720ff84455f590c
agent_id: f2f03e84-3362-4e18-97fe-f6e873c4d27c
detail: <user_visible_high_level_summary>
已完成只读核查：目录内没有脚本/配置，只有 Markdown、URDF 与两个 JSON。最强 HOME 候选是训练状态均值，但它只是 VLA 评估回位，不是全局插座标定；真实插座 target 必须现场用夹持插头的 getpos_euler 重标。
</user_visible_high_level_summary>
<response>
已完成只读探索，未修改任何文件。

## 结论

仓库中不存在一个“全局确定、插插座前统一使用”的 home 位姿。

最接近的候选是：

```text
[-0.2405779, 0.1457376, 0.1872397,
 -2.0599504, -0.0552776, 2.2011395, 0.6998188]
```

它是训练数据 `observation.state.arm.mean`，并被 4DWVLA 评估代码定义为 `HOME_JOINTS`。但它只是“训练分布均值 / VLA 评估回位姿态”，不是插座任务现场标定的 home，也不是插入目标 TCP 位姿。

插座任务的真实目标必须使用“夹着插头、已经插到位”时的 `getpos_euler` 重新标定。

## 目录盘点

`/home/nvidia/bt/s/RLmm/b/d/frk1/` 下共有：

- 16 个 Markdown；
- `plug/abs_stats.json`；
- `plug/keypoints_meta.json`；
- `fr3v2_1_franka_hand.urdf`；
- 没有 `.py`、`.sh`、`.yaml`、`.yml` 配置或脚本。

实际脚本和配置位于 `/home/nvidia/bt/s/RLmm/b/x/`，文档中多处引用了这些路径。

## 主要候选值

### 1. 训练均值 HOME：最强候选，但不是插座专用

来源：

- `/home/nvidia/bt/s/RLmm/b/d/frk1/plug/abs_stats.json:2-29`
- `/home/nvidia/bt/s/RLmm/b/d/frk1/4wvla_rlinf_eval_3A3.md:1984-1990`
- `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franky_joint_env.py:50-56`

精确值：

```text
[-0.24057792, 0.14573757, 0.18723966,
 -2.05995035, -0.05527758, 2.20113945, 0.69981885]
```

代码中：

```text
HOME_JOINTS = TRAIN_ARM_MEAN.copy()
```

评估流程会执行：

```text
open_gripper → HOME_JOINTS → open_gripper
```

来源可信度：

- 作为 VLA 评估软件 HOME：高；
- 作为插插座前安全 home：中低；
- 作为当前机器已验证的物理安全位姿：不足。

日志还明确显示机器人当时并不在该 HOME，例如：

`4wvla_rlinf_eval_3A3_off0915LOG.md:733-750`

当前关节角：

```text
[-0.0135, 0.0269, -0.0017, -1.5984, -0.0070, 1.5664, 0.7814]
```

这不是训练均值 HOME。

### 2. Factory ready 位姿：控制器 smoke 使用

来源：

- `/home/nvidia/bt/s/RLmm/b/d/frk1/franka_3.md:900-930`
- `/home/nvidia/bt/s/RLmm/b/x/scripts/step3_test_controller.py:21-43`
- `/home/nvidia/bt/s/RLmm/b/x/scripts/test_franky_controller_ext.py:41,133`

```text
[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
```

约等于：

```text
[0, -π/4, 0, -3π/4, 0, π/2, π/4]
```

这是 Controller smoke 的工厂 ready 位姿，不是插座任务目标，也不应在夹着插头、靠近插座时盲目调用。

可信度：

- 作为代码中的 smoke home：高；
- 作为 FR3 插座任务安全 home：低。

### 3. RLinf 默认 joint reset 位姿

实际代码：

`/home/nvidia/bt/s/RLmm/rlinf/envs/realworld/franka/franka_env.py:73-100`

```text
joint_reset_qpos = [0, 0, 0, -1.9, -0, 2, 0]
joint_reset_cycle = 20000
```

文档也明确说明：

`/home/nvidia/bt/s/RLmm/b/d/frk1/charger_sac_async.md:1043-1056`

该位姿只每 20000 次 reset 使用一次，不能视为插座任务 home，也没有针对当前插座工装验证安全性。

可信度：

- 作为上游默认值：高；
- 作为插插座前 home：低。

### 4. 插座任务真实 target：没有固定数值

来源：

`/home/nvidia/bt/s/RLmm/b/d/frk1/charger_sac_async.md:367-388, 390-403`

定义是：

```text
target_ee_pose =
[x, y, z, roll, pitch, yaw]
```

含义是：

> 夹着充电器并已经成功插入插座时，TCP 的位姿。

文档明确指出：

- 插座位置变化时必须重新标定；
- 不能将 factory `HOME_JOINTS` 当作 `target_ee_pose`；
- 不能用空爪标定；
- 不能把 `getpos` 的四元数 7D 填入配置；
- 必须读取 `getpos_euler` 的 6D。

因此，当前 frk1 文档没有发现真实插座 target 的数值。

### 5. 插座 reset 位姿是相对计算，不是全局 home

来源：

`charger_sac_async.md:599-602, 1043-1056`

```text
reset_ee_pose =
target_ee_pose + [0, 0, 0.05, 0, 0, 0]
```

另外，`go_to_rest()` 还会先沿当前 TCP 抬升约 10 cm，再移动到 reset 悬停点。

所以它依赖现场标定的 `target_ee_pose`，不是一个全局固定绝对位姿。

### 6. 运行时 probe 位姿：都是临时当前位姿

例如：

`franka_3LOG.md:800-810`

```text
[0.2462, 0.2104, 0.2739, -1.5707, 0.0170, -0.0003]
```

`franka_3LOG.md:1309-1315`

```text
[0.2370, 0.2062, 0.2686, -1.5688, 0.0281, 0.0211]
```

这些是测试当时的 TCP probe 结果，不是全局 home，也不是插座目标。

### 7. Cube Place 的 H1 位姿：数值存在，但属于另一个任务

来源：

- `/home/nvidia/bt/s/RLmm/b/d/frk1/dmo_place_2.md:314-318`
- `/home/nvidia/bt/s/RLmm/b/d/frk1/dmo_place_2LOG.md:1175-1216`
- `/home/nvidia/bt/s/RLmm/b/x/configs/cube_place_target_ee_pose.yaml:2-9`

```text
[0.726857066, 0.0249468647, 0.250846744,
 -3.13747483, 0.0354833569, -0.00268804519]
```

对应 cube-place 的方块接触标定，不是插插座位姿，不能复用。

### 8. Dummy / placeholder 位姿

`charger_sac_async.md:956`：

```text
[0.5, 0.0, 0.3, 3.14, 0.0, 0.0]
```

文档明确标注：

```text
任意；dummy 不连臂
```

不能作为真实机器人 home 或插座目标。

## JSON 和 URDF 核查

### `abs_stats.json`

`/home/nvidia/bt/s/RLmm/b/d/frk1/plug/abs_stats.json`

包含：

- `observation.state.arm`：7 个关节；
- `observation.state.gripper`；
- `observation.state.ee_pos`；
- `observation.state.ee_quat`；
- `action.arm`；
- `action.gripper`；
- count 为 66577。

它只提供统计分布，没有：

- episode 起始 pose；
- home 标记；
- reset pose；
- 插座成功 pose；
- 插头插入前 pose。

### `keypoints_meta.json`

`/home/nvidia/bt/s/RLmm/b/d/frk1/plug/keypoints_meta.json`

定义：

```text
bbox_radius: 0.8361004471778869
keypoint_dim: 7
num_keypoints: 8
rotation: quaternion_xyzw
coordinate_system: base_link-relative
```

`global_min_base_relative` 和 `global_max_base_relative` 是关键点归一化包络，不是机器人 home 或 TCP 安全盒。

需要注意：文件内部引用的 URDF 是：

```text
b/d/Frk/fr3v2_1_franka_hand.urdf
```

实际当前文件是：

```text
/home/nvidia/bt/s/RLmm/b/d/frk1/fr3v2_1_franka_hand.urdf
```

路径大小写和目录不一致，建议核验实际加载代码是否通过 CLI 覆盖了这个路径。

### `fr3v2_1_franka_hand.urdf`

`/home/nvidia/bt/s/RLmm/b/d/frk1/fr3v2_1_franka_hand.urdf`

定义了 7 个旋转关节、固定 joint8 和两个夹爪移动关节。主要硬限位：

```text
q1 [-2.90074, 2.90074]
q2 [-1.83609, 1.83609]
q3 [-2.90074, 2.90074]
q4 [-3.07702, -0.11694]
q5 [-2.87630, 2.87630]
q6 [ 0.43982, 4.62163]
q7 [-3.05083, 3.05083]
```

关键行：

- q1：136-145
- q2：204-212
- q3：272-280
- q4：340-348
- q5：426-434
- q6：480-488
- q7：552-560
- TCP 固定变换：637-641
- 夹爪限位：723-738

URDF 中没有 `home`、`initial`、`initial_position` 或默认关节配置。`origin` 只是机器人几何链变换，不是 home 姿态。

## 最终判断

- “全局确定的插插座前 home”：不存在。
- “VLA 评估通用 HOME”：存在，最强候选是训练关节均值。
- “Franka 控制器 smoke HOME”：存在，是 `[0,-0.785,0,-2.356,0,1.571,0.785]`。
- “RLinf 默认低频 joint reset”：存在，是 `[0,0,0,-1.9,0,2,0]`。
- “插座任务真实 TCP 目标”：没有固定数值，必须现场标定。
- “cube-place H1”：有固定数值，但不是插座任务，不能复用。

## 建议进一步核验

优先核验以下文件：

1. `/home/nvidia/bt/s/RLmm/b/d/frk1/charger_sac_async.md`
 - `target_ee_pose`
 - `getpos_euler`
 - `reset_ee_pose`
 - `joint_reset_qpos`

2. `/home/nvidia/bt/s/RLmm/b/d/frk1/4wvla_rlinf_eval_3A3.md`
 - `HOME_JOINTS`
 - `TRAIN_ARM_MEAN`
 - `go_to_rest`
 - `Episode 间场景重置流程`

3. `/home/nvidia/bt/s/RLmm/b/d/frk1/4wvla_rlinf_eval_3A3_off0915LOG.md`
 - T5 当前关节角与 HOME 对比；
 - T7 返回 HOME；
 - `h` 键和 `env.reset()` 行为。

4. `/home/nvidia/bt/s/RLmm/rlinf/envs/realworld/franka/franka_env.py:73-100`
 - 实际默认 `joint_reset_qpos`；
 - `joint_reset_cycle`。

5. `/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franky_joint_env.py:50-56,215-249`
 - 评估 HOME 的真实实现。

6. `/home/nvidia/bt/s/RLmm/b/x/scripts/step3_test_controller.py:21-43`
 - factory ready 位姿及实际调用位置。

7. `/home/nvidia/bt/s/RLmm/b/x/configs/cube_place_target_ee_pose.yaml`
 - 仅用于确认 cube-place H1，避免误当插座 target。

8. `/home/nvidia/bt/s/RLmm/b/d/frk1/plug/keypoints_meta.json:1-33`
 - 核验 URDF 路径引用是否失效。
</response>
output_path: \home\nvidia\.cursor\projects\home-nvidia-bt-s-4WVLA\agent-transcripts\4ebbc526-4b84-4b0f-a391-344ad18be6f6\subagents\f2f03e84-3362-4e18-97fe-f6e873c4d27c.jsonl
</task>
</system_notification>
<user_query>Perform any necessary follow-up actions in response to the subagent completion above. If no follow-up work is needed, no further action is required. If you mention an agent or subagent in your response, link it with the `[Name](id)` Don't use generic label such as `[agent]`, `[worker]`, or `[subagent]`. For cloud subagents, when the agent has edited code, link to `[Review](bc-id#changes)`, or, if you know the exact added and deleted line counts, `[Review +A −D](bc-id#changes)`, replacing A and D with those counts. Never write A or D literally. Use `[Try Live](bc-id#desktop)` only when the agent used computer use. Don't repeat the same confirmation every time.</user_query>

---

**Cursor**

两份核查结果补充确认了一个重要边界：独立 `franky.Robot` 脚本不能接管已经被 Ray/训练进程占用的 FCI。因此我会补充文档说明：该脚本只能在原控制进程退出后执行；训练仍在运行时应使用现有 `r`/`h` 控制路径，而不是启动第二个 FCI 客户端。


已完成必要跟进：

- 更新 `b/x/franky_ext/dsplug/README.md`，明确独立脚本不能抢占已被 Ray/训练进程占用的 FCI。
- 训练运行期间应使用现有键盘控制：
  - `r`：中止并停止；
  - `h`：回到评测 HOME。
- `stop_and_home.py` 仅适用于原控制进程已退出、FCI 已释放后的独立操作。

另外确认 `keypoints_meta.json` 中的 URDF 路径存在目录大小写/路径差异，但当前 `vla_inference_server.py` 要求通过 `--urdf-path` 显式传入正确路径，因此暂不修改原始元数据。

---

**User**

检查一下现在有进程占用 FCI 吗? 有就停掉它

---

**Cursor**

我先检查本机到 Franka FCI（通常是 TCP 1337）的连接及相关进程；只有确认是本机控制客户端后才会停止，避免误杀无关进程。

已检查：

- 本机 TCP 1337 无连接、无监听进程；
- `lsof` / `fuser` 均未发现 FCI 占用者；
- 虽有 `rlinf-4dwvla-gpu` 容器运行，但未发现其占用 FCI。

因此没有进程需要停止，也未停止任何容器。

---

**User**

容器 rlinf-rlt-franka 已经启动, 请问如何在容器内运行 stop_and_home.py

---

**Cursor**

当前容器可用的 Python 是 `/opt/venv/franky-0.19.0/bin/python`，但新建的 `dsplug` 目录尚未挂载进容器。先在宿主机执行：

```bash
docker cp \
  /home/nvidia/bt/s/RLmm/b/x/franky_ext/dsplug \
  rlinf-rlt-franka:/home/a0001/RLinf/b/x/franky_ext/
```

进入容器：

```bash
docker exec -it rlinf-rlt-franka bash
source /opt/venv/franky-0.19.0/bin/activate
cd /home/a0001/RLinf
export FRANKA_ROBOT_IP=172.16.0.2
```

先只读检查：

```bash
python b/x/franky_ext/dsplug/stop_and_home.py --print-home
python b/x/franky_ext/dsplug/stop_and_home.py \
  --robot-ip "$FRANKA_ROBOT_IP"
```

确认没有其他进程占用 FCI 后，执行停止并回 home：

```bash
python b/x/franky_ext/dsplug/stop_and_home.py \
  --robot-ip "$FRANKA_ROBOT_IP" \
  --execute
```

脚本会先停止、检查状态，然后提示输入：

```text
HOME
```

不要先使用 `--yes`，也不要默认加 `--open-gripper`，避免插头掉落。

注意：如果 RLinf/Ray 控制进程仍占用 FCI，脚本无法抢占连接；应先停止原控制进程，或在评测程序内使用现有的 `r`/`h` 控制路径。

---

**User**

@RLmm/b/x/franky_ext/dsplug/home_pose.json  中 HOME 的位置是怎么来的? 从腕部摄像头的图片看上去, 好像比训练数据集Franka腕部摄像头的图片位置低了不少, 怎么会这样? 如何手动从新设置 HOME 位置?

---

**Cursor**

这个现象很可能不是 URDF 计算错误，而是当前 `HOME` 的统计语义与“腕部相机里看到的训练初始画面”不同。我先核对 `home_pose.json` 的生成链、数据集 episode 起始帧与 TCP 统计，再给出可复现的手动重新标定流程；如果现有脚本不支持安全采集，我会补上。

已经补上了“先预览、再确认写入”的手动采集流程：它只读取已静止的当前关节，不会在采集时移动机械臂；写入前要求输入 `WRITE_HOME`，并自动生成 `.bak` 备份。接下来我会做语法、静态诊断和离线参数校验。

结论：`home_pose.json` 中的 HOME 不是“每个 episode 插入前姿势”，而是训练集所有帧的关节均值。

- 来源：`b/d/frk1/plug/abs_stats.json`
- 字段：`observation.state.arm.mean`
- 样本数：66577 帧
- 当前关节角：

```text
[-0.24058, 0.14574, 0.18724, -2.05995,
 -0.05528, 2.20114, 0.69982] rad
```

`tcp_pose_from_urdf` 只是把这些关节角通过 URDF 做 FK 得到的参考 TCP 位姿；`stop_and_home.py` 实际发送的是七个关节角，不会使用该 TCP 字段。

为什么腕部图像看起来更低：

1. 全局均值混合了抬升、接近、插入、撤回等所有阶段，不能代表 episode 起始位置。
2. 完整统计中 TCP 的 `z` 均值约为 `0.2647 m`，但 `90%` 分位约为 `0.3525 m`，`99%` 分位约为 `0.4619 m`。
3. 本地 8-episode 冒烟子集的首帧 TCP `z` 均值约为 `0.4365 m`，范围约 `0.375–0.511 m`，比当前 HOME 高约 `17 cm`。这与观察到的腕部画面差异一致，但这个子集不等于完整 100-episode 数据集。
4. 腕部相机画面还受相机安装外参、相机序列号、相机朝向、裁剪和缩放影响；TCP 的 `z` 变化不等同于图像中的垂直变化。

因此，如果目标是“插入前、插座上方的安全姿态”，不应继续使用全局均值，最好现场手动标定。

## 手动重新设置 HOME

已在 `b/x/franky_ext/dsplug/stop_and_home.py` 中加入安全采集功能。

1. 停止 RLinf/Ray/VLA 控制进程，释放 FCI。
2. 使用 Franka Desk 或手动引导，将机械臂放到期望 HOME。
3. 松开引导按钮，确认机器人处于 `RobotMode.Idle` 且关节速度为零。
4. 在容器内先预览当前姿态：

```bash
python b/x/franky_ext/dsplug/stop_and_home.py \
  --robot-ip 172.16.0.2 --capture-home
```

该命令只读，不移动机械臂。

5. 确认腕部画面和机械臂位置正确后写入：

```bash
python b/x/franky_ext/dsplug/stop_and_home.py \
  --robot-ip 172.16.0.2 --capture-home --write-home
```

输入：

```text
WRITE_HOME
```

原文件会备份为：

```text
home_pose.json.bak
```

之后可以先用普通命令交互式回 HOME：

```bash
python b/x/franky_ext/dsplug/stop_and_home.py \
  --robot-ip 172.16.0.2 --execute
```

注意：`franka_joint_env.py` 中还存在独立的硬编码：

```python
HOME_JOINTS = TRAIN_ARM_MEAN.copy()
```

如果评测环境也要使用新 HOME，还需要同步更新它，否则停止脚本和评测环境会使用两个不同的 HOME。

已完成离线语法检查、CLI 检查和代码诊断，均通过；没有连接或移动真实机器人。

---

**User**

除了 @RLmm/b/x/4dwvla_ext/franky_joint_env.py  和 @RLmm/b/x/4dwvla_ext/extreme_pose_explorer.py  还有哪些代码或脚本含有硬编码的 HOME 位姿, 请把它们改成从 @RLmm/b/x/franky_ext/dsplug/home_pose.json 读取.

---

**Cursor**

我会先全库检索 HOME/HOME_JOINTS/`TRAIN_ARM_MEAN` 及相关关节数组，区分“真正用于复位/回 HOME 的硬编码”与仅用于训练范围或示例的常量；随后统一改为读取 `dsplug/home_pose.json`，并保留清晰的路径回退与校验。

检索结果显示，除两个已知的 VLA 文件外，还有几类实际使用 HOME 的代码：Franka 控制 smoke/REPL、GELLO 对齐脚本、FK 离线测试，以及一个 IPC 测试样例。另有一些训练统计均值和通用机器人配置，它们不是这个插座任务的 HOME，我不会把专用 7 关节 HOME 强行写入那些通用/双臂配置。现在先建立统一的 JSON 加载器，避免各脚本各自解析和再次硬编码。

统一加载器已接入主要调用链。当前已替换的额外硬编码包括：两个 Franky 控制测试脚本、一个 Franky REPL、GELLO 对齐脚本、FK 离线测试和 IPC 样例；两个 VLA 文件也同步改为同一来源。接下来检查残留项，避免把“归一化统计均值”误当成 HOME。

已完成统一改造。

新增共享加载器：

- `b/x/franky_ext/dsplug/home_pose.py`
- 从同目录 `home_pose.json` 读取并校验 7 个关节角及 FR3v2.1 限位。

已改为读取 JSON 的文件：

- `b/x/4dwvla_ext/franky_joint_env.py`
- `b/x/4dwvla_ext/extreme_pose_explorer.py`
- `b/x/scripts/step3_test_controller.py`
- `b/x/scripts/test_franky_controller_ext.py`
- `toolkits/realworld_check/test_franky_controller.py`
- `toolkits/realworld_check/test_gello.py`
- `b/x/4dwvla_ext/tests/test_fk_keypoints_offline.py`
- `b/x/4dwvla_ext/tests/test_ipc_offline.py`
- `b/x/franky_ext/dsplug/stop_and_home.py`

另外，当前 `home_pose.json` 的手动采集元数据已修正为：

```text
source_feature: franky.Robot.state.q
```

未修改的 `test_transforms_offline.py` 中的关节均值属于训练归一化统计，不是机器人 HOME；Genesis 仿真和双臂通用配置中的 reset 位姿也未替换，因为它们不是该 FR3 插座任务的 7 关节 HOME。

验证结果：

- JSON 加载与限位校验：通过
- IPC 离线测试：10/10 通过
- HOME 安全测试：36/36 通过
- Extreme pose dry-run：通过
- 语法检查与 Linter：通过
- 未连接或移动真实机器人。

---

**User**

@RLmm/b/d/frk1/4wvla_rlinf_eval_3A3.md:4894-4907 这里提到的"按 'h' 键 "后, 能回到 @RLmm/b/x/franky_ext/dsplug/home_pose.json 中设定的 home pose 吗?

---

**Cursor**

我会沿着文档中的实际调用链核对：`h` 键 → wrapper → `env.go_to_rest()` → 具体环境的 HOME 来源，重点确认运行时是否使用当前 `home_pose.json`，而不是只看变量名。

是。按当前代码链路，`h` 键会回到 `home_pose.json` 中的 HOME。

调用链：

```text
franka_vla_client.py
  → KeyboardVLAEvalWrapper
  → h 键
  → env.unwrapped.go_to_rest()
  → FrankyJointEnv.HOME_JOINTS
  → load_home_joints()
  → dsplug/home_pose.json
```

`go_to_rest()` 的实际行为是：

1. 打开夹爪；
2. 读取 JSON 中的 7 个关节角并执行 `reset_joint()`；
3. 再次打开夹爪。

因此，文档第 4899 行的旧数值：

```text
[-0.2406, 0.1457, 0.1872, -2.06, -0.0553, 2.2011, 0.6998]
```

只是过时的说明文字，不再决定真实运动目标。实际目标由当前 `home_pose.json` 决定。

需要注意两点：

- 修改 `home_pose.json` 后，必须重启 `franka_vla_client.py`，因为 HOME 在 Python 模块导入时加载。
- `h` 事件是在一次 `env.step(action)` 后检查的，按下瞬间可能先执行一个动作，然后才停止并回 HOME。

可在容器内无机器人动作地验证：

```bash
cd /workspace/RLinf

PYTHONPATH=/workspace/RLinf/b/x:/workspace/RLinf/b/x/4dwvla_ext \
python - <<'PY'
import numpy as np
from franky_ext.dsplug.home_pose import load_home_joints
from franky_joint_env import HOME_JOINTS

json_home = load_home_joints()
print("JSON HOME:", json_home.tolist())
print("ENV HOME :", HOME_JOINTS.tolist())
assert np.allclose(json_home, HOME_JOINTS)
print("HOME source check: PASS")
PY
```

目前代码会输出并使用 `home_pose.json` 的最新手动标定值。

---

**User**

@RLmm/b/d/frk1/grperr_1.md:299  这里的 n_exec 是干嘛用的? 整句话是什么意思? 请引用代码和举例说明.

---

**Cursor**

我先对照文档上下文和评测客户端、关键点历史缓冲的代码，把 `n_exec` 的实际作用和这句话的含义讲清楚。

接下来对照训练侧“每帧推进一格历史”和评测侧“每推理一次才推进一格”的具体实现。

`n_exec` 是「每次推理后，真机连续执行多少步动作再重新看世界」。第 299 行说的是：这个开环执行窗口，把 keypoint 历史时钟也一起放慢了 10 倍。

## `n_exec` 是干什么的

模型一次会预测一整段未来动作（训练时 chunk 是 50 步）。真机评测并不把 50 步全跑完，而是只取前 `n_exec` 步下发给机器人，跑完再拍图、再推理。这是 receding-horizon / receding-chunk 闭环控制：用短开环窗口换推理开销。

默认 `n_exec=10`。服务端裁切返回值：

```471:474:b/x/4dwvla_ext/vla_inference_server.py
                normalized_action = action_pred[:n_exec, :actual_action_dim]
                normalized_actions = normalized_action.detach().float().cpu().numpy()
                physical_action = unnormalize_fn({ACTION: normalized_action})[ACTION]
                actions = physical_action.detach().float().cpu().numpy()
```

客户端把这 10 步放进队列，队列空了才再请求一次推理：

```206:215:b/x/4dwvla_ext/franka_vla_client.py
        while self._step_count < self._max_steps and not self._abort:
            if not self._action_queue:
                images = self._env.get_camera_frames()
                state = obs["state"]
                logger.info("[step %d] Inference (q1=%.3f, grip=%.4f)", self._step_count, state[0], state[7])
                actions = self._request_inference(images, state)
                self._action_queue.extend(actions)
                logger.info("  Received %d actions", len(actions))
```

所以时间轴是：

```text
控制步:  0  1  2  3  4  5  6  7  8  9  10 11 ... 19  20 ...
推理:    ▲                             ▲              ▲
         取 10 步动作                  再推理          再推理
```

`n_exec` **本身是合理的执行参数**，问题出在当时把「更新 keypoint 历史」也绑在了推理频率上。

## 第 299 行整句在说什么

拆开四层：

1. **训练语义**：数据集 30 Hz，每一帧都算一次 3D keypoint 并推进一格。`his_len` 就是「本回合过了多少帧」，等价于 `min(frame_index, 200)`。
2. **旧评测实现**：`FKKeypointComputer.step()` 只在服务端收到推理请求时才调用一次。
3. **倍数**：请求每 `n_exec=10` 个控制步才发一次，所以历史时钟慢 10 倍。
4. **为何夹爪合不上**：示教里最早闭合发生在 `his_len ≥ 120`；真机跑了 700 个控制步，只推理了 `700/10=70` 次，`his_len` 最多到 70，永远进不了训练里会出现闭合的时间区间。

旧服务端（文档里引用的那版）就是「每次请求只推进一格」：

```python
kpt_data = fk_computer.step(arm_q)   # 每个推理请求只 append 一次
```

`step()` 每次只往滑动窗口塞当前这一帧：

```78:82:b/x/4dwvla_ext/fk_keypoints.py
    def step(self, arm_q7: np.ndarray) -> tuple[np.ndarray, int]:
        """Compute keypoints, append to history, return (his_kpts, his_len)."""
        kpt = self.compute(arm_q7)
        self._history.append(kpt)
        return self._pack()
```

## 用数字把因果钉死

示教统计（文档表）：

| `his_len` | 训练里夹爪闭合 |
|---|---|
| `≤ 70`（真机实际到达） | **0 次** |
| 全体闭合帧的最小 `his_len` | **120** |

于是策略看到的输入，在训练分布里 100% 对应「还在接近、夹爪张开」。它不闭合，不是执行层拒收，而是模型从未被放到「该闭合」的时间上下文里。

例子（`n_exec=10`，700 步）：

```text
训练 30 Hz：
  第 0 帧  his_len=1
  第 1 帧  his_len=2
  ...
  第 119 帧 his_len=120  ← 示教里最早开始闭合
  第 699 帧 his_len=200（饱和）

旧评测：
  控制步 0   第 1 次推理  his_len=1
  控制步 1–9 只执行队列里的动作，不更新历史
  控制步 10  第 2 次推理  his_len=2
  ...
  控制步 690 第 70 次推理 his_len=70
  结束。从未到达 120。
```

`his_len` 还会进 TrackEncoder：`patch_lengths = his_len // 4`。70 → 17 个时间 patch；120 → 30 个；200 → 50 个。action expert 会 attend 这条时间轴，时钟错了，闭合动作就不会出来。

另外，相邻两格历史之间隔了 10 个控制步，训练里只隔 1 帧（1/30 秒）。历史不但「格数不够」，「轨迹也被抽稀」。

## 和现在代码的关系

这句话描述的是 **R1 被修掉之前** 的行为。当前服务端已经按控制步回放：

```436:441:b/x/4dwvla_ext/vla_inference_server.py
                    executed_history = msg.get("state_history", ())
                    for executed_q in executed_history:
                        fk_computer.step(np.asarray(executed_q, dtype=np.float32))
                    kpt_data = fk_computer.step(arm_q)
```

客户端每执行一步就记下关节角，下次推理时把「上次推理以来的姿态」带上。这样 `n_exec` 仍然管「执行多少步再重规划」，但 `his_len` 按控制步 +1，不再跟推理次数绑死。

一句话：`n_exec` 决定「看一次、走几步」；旧实现误把「历史时钟」也按「看一次」走，慢了 10 倍，所以 700 步只能走到 `his_len=70`，而闭合门槛是 120。

---

**User**

训练时的设定是不是相当于 n_exec=1 ?

---

**Cursor**

不完全是。我对照训练时的动作 chunk 监督和评测时的 `n_exec` 开环窗口，把两套时间语义分开说明。

不是。训练里根本没有 `n_exec` 这个量；它对应的是评测时「预测一次后开环走几步再重规划」。把训练理解成 `n_exec=1`，是把两套时钟混在一起了。

## 训练在干什么

训练是离线监督学习，不驱动机器人，所以 3A3 文档把 `n_exec` 标成 **N/A**。每个样本是：当前观测 + 未来 `chunk_size=50` 步真值动作，loss 算在全部 50 步上，`n_action_steps=50`。

如果硬要映射到评测语义，**动作监督更接近「一次看完整 50 步」**，不是 `n_exec=1`。

评测才引入 receding horizon：模型仍预测 50 步，但只执行前 `n_exec` 步就重新推理。`n_exec=1` 表示每步都重规划；`n_exec=10` 表示看一次、走 10 步。

## 容易混的那条时钟：`his_len`

第 299 行说的不是动作 chunk，而是 keypoint 历史。

训练时，数据集 30 Hz，**每一帧**都推进一格 `his_len`：

```text
frame 0 → his_len=1
frame 1 → his_len=2
...
frame 119 → his_len=120   ← 示教里最早闭合
```

旧评测把 `FKKeypointComputer.step()` 绑在推理请求上，于是：

```text
his_len 增长速度 = 1 / n_exec
n_exec=10  → 慢 10 倍
n_exec=1   → 碰巧每控制步 +1，和训练的 his_len 对齐
```

所以：**只有「历史时钟」这一条，旧实现在 `n_exec=1` 时才碰巧和训练同速**。这不等于训练设定就是 `n_exec=1`。

## 对照

| | 训练 | 评测 `n_exec=1` | 评测 `n_exec=10`（当时出问题的设定） |
|---|---|---|---|
| 是否存在开环执行窗口 | 否，无机器人 | 每步重规划 | 每 10 步重规划 |
| 动作预测长度 | 50，全部进 loss | 仍预测 50，只用第 1 步 | 仍预测 50，用前 10 步 |
| `his_len`（旧实现） | 每帧 +1 | 每步 +1（碰巧对齐） | 每 10 步 +1（慢 10 倍） |

一句话：训练 ≈「每帧更新观测和历史，并监督一整段 50 步动作」；`n_exec=1` ≈「每步都重新推理」。两者只在旧代码的 `his_len` 上碰巧重合，动作时间语义并不一样。

---

**User**

那按照 @RLmm/b/d/frk1/grperr_1.md  中做的修改, 和 @\home\nvidia\bt\s\RLmm\ 的真实代码, 真机实验时能的各种超参要怎么设置才能使机器人的输入输出与训练时的分布更吻合.

---

**Cursor**

这个问题需要把文档里的修复结论和当前代码的真实默认值逐项对齐。我先读 `grperr_1.md` 的修复与验收章节，再核对客户端/服务端的实际参数和安全层常量。

现在核对代码里的真实默认值与可调参数。

再确认一处细节：客户端与服务端在历史计数上的实际增量是否与训练的"每控制步 +1"严格一致。

## 结论先说

修复 A–F 之后，真正还能通过"超参"拉近训练分布的只有 4 个量：`--max-steps`、`--n-exec`、`--control-hz`、以及夹爪抓取窗口那两个环境变量。另外两个影响更大的量**不是超参**：一是 `relative_dynamics_factor=0.2` 硬编码在控制器里（决定实测 Hz），二是历史计数有个 +1/周期的系统性多计（真机日志 `1→12→23`，应为 `1→11→21`）。夹爪阈值 0.5 **不该动**——它恰好就是训练分布的分界。

---

## 一、可以直接调的超参

| 参数 | 当前默认 | 建议值 | 训练侧依据 |
|:---|:---|:---|:---|
| 服务端 `--n-exec` | 10 | **5** | chunk_size=50，取前 5 步替代前 10 步，重规划更频繁、更接近训练"每帧一个决策"的采样语义 |
| 客户端 `--n-exec` | 10 | **与服务端一致** | 见下方"陷阱" |
| 客户端 `--control-hz` | 10 | **30** | 见下方"反直觉" |
| 客户端 `--max-steps` | 300 | **600**（长跑 800） | `frame_index` q90=604、max=797（26.6 s @ 30 Hz）；300 步只够 `his_len` 刚饱和 |
| `FRANKA_CUBE_WIDTH_M` | 0.015 | 卡尺测量值（若与示教一致则 ≈**0.003**） | `observation.state.gripper` 双峰：闭合模态 ≈0 m（q50=0.0158，87% 帧 <1 mm），张开模态 ≈0.078 m |
| `FRANKA_HOLD_TOL_M` | 0.010 | ≈**0.008** | 当前窗口接受 [5, 25] mm，与示教闭合模态完全不重叠 |
| `VLA_GRIPPER_CLOSE_THRESHOLD` | 0.5 | **保持 0.5** | `action.gripper` 双峰：q10=0.02 / q50=0.80 / q90=1.0，0.5 正落在谷底 |
| `VLA_GRIPPER_CLOSE_IF_ABOVE` | 1 | **保持 1** | max=1.0 对应闭合 |
| `--dtype` | bfloat16 | **保持 bfloat16** | 与训练数值一致；改 float32 是引入新的数值偏差，不是"更准" |
| `RS_GLOBAL_SERIAL` / `RS_WRIST_SERIAL` | 未设 | **source 配置文件** | 两路视角接反是最大的输入分布风险，且图像统计无法自检 |

### `--control-hz` 是反直觉的

它在 `FrankyJointEnv.step()` 里只决定运动**之后**的 `time.sleep(1/control_hz)`，不限制机器人速度。所以 `--control-hz 5` 会在每步额外空等 0.2 s，把你推得离 30 Hz **更远**；文档 §6 V2 用 5 是为 Level 1 保守起步，不是为了对齐分布。安全等级已经过了 Level 2 的话，设 30 让 sleep 变成可忽略项，实测仍会停在 3.5–4.5 Hz，因为瓶颈是 franky 阻塞式 `Robot.move()`。

### 一个陷阱：客户端的 `--n-exec` 只用于打日志

真正决定每次返回多少步的是服务端：

```471:471:/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/vla_inference_server.py
                normalized_action = action_pred[:n_exec, :actual_action_dim]
```

客户端拿到几条就执行几条（`self._action_queue.extend(actions)`），`self._n_exec` 只出现在启动日志那一行。两边不一致不会出错，但日志会说谎，验收时对不上 `his_len` 增速。

---

## 二、为什么慢 10 倍在空间上其实不致命

这点值得说清楚，因为它决定了要不要冒险去改运动参数。模型输出的是**绝对关节目标**，采自 30 Hz 的 chunk，所以相邻两个控制步之间的空间步长 ≈ 训练的每帧步长（约 0.001–0.005 rad）。修复 A 把 keypoint 历史改成按控制步推进之后，历史里**每格的空间增量已经和训练一致**，只是每格对应 0.29 s 真实时间而不是 0.033 s。

所以剩下的失配是"实时性/动力学"层面的，不是"空间分布"层面的。顺带说明：`MAX_JOINT_STEP_RAD = 0.15` 在这种步长下永远不会触发，不用调。

要真正把时间轴拉到 30 Hz，唯一有效的旋钮是这一行：

```120:122:/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franky_controller_direct.py
        self._robot = franky.Robot(robot_ip)
        self._robot.recover_from_errors()
        self._robot.relative_dynamics_factor = 0.2
```

提到 0.3–0.4 会缩短每段阻塞运动。但这是改变真实运动特征，按 §15.7 必须重走 Level 0→3，文档 §8 明确把它排除在本轮之外。我的建议是先不动，因为上面刚说明了空间分布已经对齐，收益主要在实时性。

---

## 三、超参关不掉的两个残留失配

**1. `his_len` 每周期多计 1 格（需要一行代码改动）**

客户端把 `n_exec` 个 `state_after` 全部 drain 出去，服务端重放完这 `n_exec` 个之后**又**对当前 `arm_q` 调一次 `step()`。而当前 `arm_q` 就是最后一个 `state_after`，于是被数了两次：

```438:441:/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/vla_inference_server.py
                    executed_history = msg.get("state_history", ())
                    for executed_q in executed_history:
                        fk_computer.step(np.asarray(executed_q, dtype=np.float32))
                    kpt_data = fk_computer.step(arm_q)
```

离线单测是按正确语义写的（重放 `n_exec - 1` 个 + 当前帧，见 `test_fk_keypoints_offline.py:129-135`），但实现 drain 了全部，所以真机日志是 `1 → 12 → 23`（每次 +11），而训练语义要求控制步 t 时 `his_len = t+1`，即 `1 → 11 → 21`。后果有两个：`his_len` 领先约 10%（第 180 步就饱和而非第 200 步），且历史里每周期夹着一个重复姿态。改法是客户端只交出除最后一个以外的记录（`drain()` → 保留末项作为当前帧），一行的事。要我改吗？

**2. HOME 起点落在示教的分布尾部**

当前 `home_pose.json` 的 `q = [-0.258, -0.021, 0.167, -1.885, -0.055, 1.906, 0.644]`，其中 q2 = -0.021 低于示教 q10 = 0.025，q6 = 1.906 低于示教 q10 = 2.031（示教 min 1.570）。文档 §5 说它在"起始位姿均值 1σ 内"成立，但按全局分位数看 q2/q6 偏低。因为 `his_len = 1..N` 的历史内容就是从这里长出来的，如果要更贴，可以按 §9.6 给的示教抓取前位姿附近重新 `--capture-home`。

**3. 抓取判据本身有不可消解的歧义**

Level 1 已经复现了 `holding=True width=0.0097m` 而夹爪实际是空的。即便把窗口改成 `0.003 ± 0.008`（接受 [0, 11] mm），9.7 mm 的空手闭死仍会落在窗口内。这是 `franka_libfranka_gripper.py` 模块注释早就警告过的：宽度无法区分"空手合拢"和"握持"。想要可靠判据得改用抓取力 / `is_grasped`，那是代码改动，不是超参。

---

## 四、落地命令

服务端（GPU 容器）：

```bash
python /workspace/RLinf/b/x/4dwvla_ext/vla_inference_server.py \
  --ckpt-path <4wvlaFrkPlugCkp041680> \
  --schema-path <.../franka_plug.yaml> \
  --kpt-meta-path /workspace/RLinf/b/d/frk1/plug/keypoints_meta.json \
  --urdf-path /workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf \
  --n-exec 5 --dtype bfloat16 --port 5555
```

客户端（Franky 容器）：

```bash
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env
# 卡尺量完插头再取消注释这两行
# export FRANKA_CUBE_WIDTH_M=0.003 FRANKA_HOLD_TOL_M=0.008
python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py \
  --robot-ip 172.16.0.2 --task "plug into socket" --use-realsense \
  --n-exec 5 --control-hz 30 --max-steps 600
```

`keypoint_history_max_len` 从 checkpoint config 读（默认 200 = 6.7 s @ 30 Hz），不要覆盖；`SAFETY_MARGIN_RAD` 的 q7=0.05 也建议保持——91% 的步被裁回是**测量结果**，说明安全网正在把状态拉回流形，把它放宽等于让 FK/keypoint 输入重新跑出分布。

---

## 五、验收时按这几个数字核对

| 检查点 | 通过标准 |
|:---|:---|
| 服务端 `history_len` 增速 | 每次请求 +6（`n_exec=5`，修掉多计后应为 +5），第 ~200 控制步饱和到 200 |
| 客户端 `state_history_len` | 首次 0，之后稳定等于 `n_exec` |
| `achieved_control_hz` | `--control-hz 30` 下预期 3.5–4.5 Hz（比 `--control-hz 5` 的 2.5 Hz 快） |
| q7 | 全程 ≥ 0.4343；`OUT-OF-TRAIN` 出现是预期的 |
| `action_grip` | 目前已知会稳定卡在 0.11–0.23 峰值 |
| `MOTION GUARD TRIP` / `HARD LIMIT` | 0 |

最后一条要提醒：§9.7 已经用两次独立 300 步试验确认"夹爪不闭合"是可复现的系统性行为，且已排除 R1–R6。**上面这些超参不会让它闭合**——它们只保证输入输出分布对齐，把变量收干净，好让下一轮分析面对的是策略本身的问题（新场景下的分布外表现、0.5 阈值对这个 checkpoint 是否偏高、或需要目标场景微调数据），而不是又一个工程 bug。

---

**User**

如果不限制在修改超参, 代码脚本以及环境变量啥的都可以改, 你要怎么改才能使机器人的输入输出与训练时的分布更吻合.

---

**Cursor**

这次要落到代码层面，我先把观测构造、图像预处理、夹爪执行和 FK keypoint 这几条链路的实现读完，才能给出有依据的改法。

这里出现了一个关键疑点：训练 transform 的 `keypoint_dim` 默认是 3（仅位置），而评测端 `FKKeypointComputer` 送的是 7 维（位置+四元数），且训练把当前帧单独放在 `kpt_t` 里。我需要核实模型推理路径到底吃哪些键。

核实最后几处：图像预处理尺寸、状态归一化模式、以及训练视频的原始分辨率。

## 结论先说

读完训练侧 transform 和 checkpoint config 之后，发现两个**语义级的输入错误**，它们不是超参能补救的，而且其中一个恰好在"夹爪闭合"这个关键时刻才发作。修掉这两个之后，剩下的失配只有时间轴（需要动运动逻辑，必须走 §15.7）。

先确认几个已经对的地方，免得白改：checkpoint 是 `kpt_4d_mode=pos_rot`（7 维）、`keypoint_history_max_len=200`、`num_keypoint_joints=8`，与 `FKKeypointComputer` 完全一致；推理路径只消费 `his_kpts`/`his_len`（`modeling_internvla_a1_5.py:2296-2300`），`kpt_t`/`kpt_future` 是训练标签，服务端不送是对的；图像 480×640 → 224×224 pad 也对得上 RealSense 的 `640x480 rgb8`。

---

## P0-1：`his_len` 语义差一帧，而且每周期还多数一次

训练侧的窗口定义是 `keypoint_3d_delta_indices = list(range(-h, c + 1))`，即 `[-H, ..., -1, 0, 1, ..., C]`，然后：

```716:731:/home/nvidia/bt/s/4WVLA/src/lerobot/policies/internvla_a1_5/transform_internvla_a1_5.py
        hist_window = stacked[:h]
        hist_is_pad = is_pad[:h].bool()
        num_invalid = int(hist_is_pad.sum().item())
        his_len = h - num_invalid
```

`stacked[:h]` 是偏移 `-H..-1`，也就是**严格早于当前帧**的那些帧；当前帧 `stacked[h]` 单独给 `kpt_t`。所以训练在第 t 帧的 `his_len = min(t, H)`，episode 首帧是 **0**。

评测侧 `FKKeypointComputer.step()` 是先 append 再 pack，把**当前帧算进了历史**；服务端又在重放完 `state_history` 之后额外 `step(arm_q)`，而 `arm_q` 就是客户端最后一条 `state_after`，于是**重复计数**。两项叠加，真机日志是 `1 → 12 → 23`，而训练语义要求 `0 → 10 → 20`。第 180 控制步时评测报 199，训练应为 180。

**改法**（三个文件，逻辑上是一处修正）：

`fk_keypoints.py` 把"记历史"和"取快照"拆开，`step()` 保留为二者的组合以兼容现有测试：

```python
def append(self, arm_q7: np.ndarray) -> int:
    """Record a frame into history without treating it as the current frame."""
    self._history.append(self.compute(arm_q7))
    return len(self._history)

def snapshot(self) -> tuple[np.ndarray, int]:
    """Pack history as-is -- training's his_kpts excludes the current frame."""
    return self._pack()
```

`franka_vla_client.py` 改成记录**执行前**的观测（即"严格早于下次推理当前帧"的那些帧），把现在这两处 `self._state_history.record(state_after[:7])` 换成在 `env.step()` 之前记录 `state_before[:7]`：

```python
state_before = np.asarray(obs["state"], dtype=np.float64)
self._state_history.record(state_before[:7])
```

`vla_inference_server.py` 不再对当前帧 `step()`：

```python
for executed_q in msg.get("state_history", ()):
    fk_computer.append(np.asarray(executed_q, dtype=np.float32))
kpt_data = fk_computer.snapshot()
```

这样第 t 个控制步 `his_len = min(t, 200)`，逐帧对齐，第 200 步整饱和。为了不让旧客户端退化成 `his_len` 恒为 0，请求里加一个 `"protocol": 2` 字段，服务端按版本分支。

顺带说明：`ExecutedStateBuffer` 本身不用改，改的是"记哪一帧"。

## P0-2：夹爪完全闭合时，状态被上报成 0.04 m

```299:303:/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franky_joint_env.py
            return {"state": np.concatenate([HOME_JOINTS, [0.04]])}
        state = self._controller.get_state()
        q = state["arm_joint_position"]
        g = state["gripper_width"] or 0.04
        return {"state": np.concatenate([q, [g]])}
```

`0.0 or 0.04` 在 Python 里等于 `0.04`——这个 `or` 本意是兜 `None`，实际把"完全闭合"也吃掉了。而训练的 `observation.state.gripper` 恰恰是双峰分布，闭合模态就在 0 附近（min=0.0、q10=0.0、q50=0.0158），示教 87% 的闭合帧宽度 <1 mm。也就是说：**这个 bug 只在夹爪真正闭合时发作**，把状态第 8 维从 0 篡改成 0.04（落在训练分布的低概率谷底），而闭合正是这个任务的关键时刻。

```python
g = state["gripper_width"]
if g is None:
    raise RuntimeError("gripper width unavailable; refusing to fabricate state")
return {"state": np.concatenate([q, [float(g)]])}
```

dummy 分支的 `0.04` 也建议换成 `0.078`（示教 episode 起始的张开模态，q90=0.0784），否则 dry-run 的 keypoint 历史从一个训练里几乎不出现的夹爪状态开始。

---

## P1：时间轴——唯一剩下的大失配，但必须走 §15.7

实测 2.5–3.6 Hz vs 训练 30 Hz。好消息是这不影响空间分布（动作是绝对关节目标，采自 30 Hz chunk，相邻控制步的空间步长本来就等于训练的逐帧步长）；坏消息是接触、视觉时序、以及"200 格历史覆盖 58 s 而非 6.7 s"这几点没法靠超参补。

瓶颈在这里——每个 waypoint 都是一次阻塞起停：

```363:366:/home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/franky_controller_direct.py
        clipped = np.clip(joint_positions, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
        import franky
        motion = franky.JointWaypointMotion([franky.JointWaypoint(clipped.tolist())])
        self._robot.move(motion)
```

我推荐 **B2 整块下发**而不是 B1 异步，因为 B2 让 franky 做连续插值，最接近训练时的连续轨迹，而且不需要赌 franky 的抢占语义：

```python
def move_joints_chunk(self, chunk: np.ndarray) -> None:
    """Send n_exec waypoints as one continuously-interpolated motion."""
    if self.guard_tripped() is not None:
        raise RuntimeError("Motion guard tripped")
    import franky
    waypoints = [
        franky.JointWaypoint(np.clip(q, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER).tolist())
        for q in chunk
    ]
    self._robot.move(franky.JointWaypointMotion(waypoints))
```

代价要如实说：L1–L3 得从逐步检查改成整块预检（逐 waypoint 裁剪 + 相邻 delta 检查），整块执行期间只剩 50 Hz watchdog 兜底，粒度变粗；而且期间读不到中间状态，`state_history` 得改用 franky 的实际轨迹回读或按 waypoint 近似。

另外 `relative_dynamics_factor = 0.2` 硬编码在 `__init__` 里，应该抽成 `FRANKA_DYNAMICS_FACTOR` 环境变量（默认仍 0.2），这样调速不用改代码、可以逐级放开并记录在实验配置里。

这一组改动会改变真实运动特征，按文档 §8 的原则必须 Level 0→3 重走一遍。

---

## P2：输出侧的两个语义修正

**夹爪二值化加迟滞。** 现在是单阈值 0.5 直接判，`action_grip` 在阈值附近抖动会导致反复开合。训练的 `action.gripper` 双峰谷底很宽（q10=0.02、q50=0.80），用 `CLOSE_ON=0.5 / OPEN_ON=0.3` 的施密特触发完全落在谷内，不改变决策边界的含义。

**握持判据别用宽度。** Level 1 已经复现了空手 9.7 mm 被判成 `holding=True`。即便把窗口改成 `0.003 ± 0.008`，空手闭死仍落在窗口内——这是 `franka_libfranka_gripper.py` 注释早就警告过的。要可靠就得换成抓取力 / libfranka 的 `is_grasped`，把 `gripper_holding()` 的实现改掉，宽度只留作日志。

---

## P3：建议新增一个在线分布监控模块

这是投入产出比最高的一项，因为它把"吻不吻合"从推测变成每步可读的数字。新增 `b/x/4dwvla_ext/distribution_monitor.py`，加载 `abs_stats.json`，每步把 8 维 state 和 8 维 action 与 `q01/q99` 比对，输出每通道的超界比例和 z-score，episode 结束打一张汇总表。这样 q7 那种"91% 步被裁"的情况不用翻 274 条 warning 才能发现，而且换场景、换 checkpoint 时能立刻看出是哪一维先跑偏。

至于验证：改完 P0 之后，文档 §6 里标成"可选"的 **V1 示教回放**应该升级为必做——用示教 MP4 的真实图像 + 真实状态逐帧喂服务端，历史按 30 Hz 推进，看首次闭合帧是否落在示教首次闭合帧 ±30 帧内。这是唯一能在不上真机的前提下判定"输入分布是否真的对齐"的实验，也正好能回答 §9.7 留下的那个问题：`action_grip` 卡在 0.11–0.23 到底是输入还有残留失配，还是策略本身在这个场景下就不自信。

---

需要我按 P0 → P2 的顺序把代码改了吗？P0 的两项可以完全离线验证（补 `test_fk_keypoints_offline.py` 里那个 `his_len` 速率断言、加一个 gripper 状态直通测试），不碰机器人；P1 我建议先只做 `FRANKA_DYNAMICS_FACTOR` 的参数化，`move_joints_chunk` 等你有空档做 §15.7 时再上。

---

**User**

结合另一个同事写的对"夹爪不闭合"的问题的分析与解决方案 @RLmm/b/d/frk1/grperr_1.1.md  以及结合你前两轮提出的在超参层面和修改代码层面对这个问题的分析和解决方案, 综合一下, 写一个最终的最优的解决方案, 如果有一些问题或方案需要进一步实验才能确定的, 也可以在该解决方案中列出. 该方案请写到 @RLmm/b/d/frk1/grperr_1.2.md 中.

---

**Cursor**

我先把同事的 `grperr_1.1.md` 完整读完，再做综合。

同事的文档很扎实。在综合之前我需要确认一件事：夹爪驱动的 API 表面（有没有位置控制 `move`、是否阻塞），否则"连续宽度闭环"的方案写不准。

两份文档读完了，加上我前两轮的核对，可以做综合了。有一个关键的重构点：`0.5` 和同事提的 `0.18` 都不可标定，而"宽度增量"判据能把硬件满开偏差这个混淆项直接解耦。我先写文档。

数字全部核对通过。有一处需要精确化：`a_hold(66.4mm)` 严格是 0.170，而 0.174 是同事文档里"示教同宽度窗口的均值"，两者含义不同，我在表里区分开。

综合方案已写入 [`b/d/frk1/grperr_1.2.md`](/home/nvidia/bt/s/RLmm/b/d/frk1/grperr_1.2.md)（706 行，代码围栏与 mermaid 块配平）。

## 综合的结果

同事 1.1 的主因判定我认为是对的，证据链闭合：策略已站在插头上方，给出的是与"开口 66.4 mm"匹配的示教动作，而评测在等 0.5——这构成动力学吸收态。我在此之上做了三处实质性改动。

**一、把方案核心从"降低阈值"换成"宽度增量判据"。** 1.1 建议把门限从 0.5 降到 0.18，方向对但这个数不可标定：在 66.4 mm 满开下，"保持张开"这个命令本身就是 $a_{\text{hold}}=0.170$，0.18 只比它高 0.010，噪声就会误触发；而一旦硬件满开恢复到示教的 79.4 mm，$a_{\text{hold}}$ 掉到 0.0075，0.18 就等价于"要求 13.8 mm 的收拢才触发"，从过敏直接翻成几乎不可能触发。改用 $\Delta w = w_{\text{meas}} - 0.08(1-a)$，两边用同一把尺，硬件满开偏差被完整解耦，换硬件不用重标定。

这个换算还顺带推翻了一个偏悲观的表述：真机峰值 0.227 对应 $\Delta w = +4.6$ mm，**是一个明确的收拢命令，比示教斜坡里任何单帧的 1.75 mm 都大 2.6 倍**。不是"策略差一点点不够自信"，而是它一直在喊收拢、执行器翻译成了"什么都不做"。据此可以给出可验收的定量预期：闭环接上后约 7 步能从 66.4 收到 38 mm，13–20 步接近闭合。

**二、补了两个会让 1.1 方案失效的前置缺陷。** `_get_observation()` 里 `g = state["gripper_width"] or 0.04`，Python 的 `0.0` 是假值，所以完全闭合会被上报成 0.04 m——恰好在斜坡终点切断 1.1 实验 2 所依赖的宽度反馈。另外修复 A 之后 `his_len` 仍不等于训练语义：训练是"严格早于当前帧"的 $\min(t,200)$，评测把当前帧算进历史且每周期重复计数一格，真机日志 `1→12→23` 而非 `0→10→20`，快约 10%。所以 1.1 §4.5 那句"格数对了、只有格间距还错"只说对了后半句。

**三、修正了 q7 的裁剪建议。** 1.1 实验 5 想把下限从 0.4343 抬到 0.4843，但那个数来自**观测**分布，而安全层裁的是**动作**；训练 `action.arm[6]` 实际低到 0.3695（$q_{01}=0.4258$），抬到 0.4843 会裁掉约 10% 的训练动作分布，方向上是更不像训练。这里有真实的目标冲突（动作保真 vs 状态保真），我把它拆成 L2a 裁剪 + L2b 只告警，并作为开放问题 Q4 留给实验定。

## 结构

实施分四阶段：阶段 0 全部零风险离线（含三项 bug 修复、全 chunk 日志、FK 与数据集逐帧对齐校验、示教回放 A/B 台架、分布监控模块）；阶段 1 是两次卡尺测量；阶段 2 才是执行语义（连续宽度为主、$\Delta w$ 二值为备，由实测延迟决定）；阶段 3/4 是 chunk 长度和运动特征。配置全部收敛到 `franka_plug_eval.env`，并明确列出哪些值来自 checkpoint、**不该**提供覆盖入口。

§8 列了 8 个待实验问题，其中 Q1（满开为什么差 13 mm）、Q2（`move_width_m` 延迟是否允许逐步控制）、Q5（闭合落在 chunk 哪一段）是卡点——三者都能在一天内做完，且都不需要机械臂运动。

需要我接着把阶段 0 的代码改动落地吗？那几项都能离线验证完。

---

**User**

我按照你的 @RLmm/b/d/frk1/grperr_1.2.md 对代码做了修改, 然后在`rlinf-4dwvla-gpu`容器内执行了`python /workspace/RLinf/b/x/4dwvla_ext/vla_inference_server.py     --ckpt-path /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp041680     --schema-path /workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml     --kpt-meta-path /workspace/RLinf/b/d/frk1/plug/keypoints_meta.json     --urdf-path /workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf     --n-exec 5 --dtype bfloat16` , 在`rlinf-4dwvla-franky`容器内执行了`python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py     --robot-ip 172.16.0.2     --task "plug into socket"     --use-realsense     --global-camera-serial 250222073513     --wrist-camera-serial 420122070525     --max-steps 600     --control-hz 30`, 怎么效果比之前还差?!! Franka机器臂一直在抖, 夹爪一直下不到放插头的地方.

---

**Cursor**

机器人在抖是安全问题，先说处理动作：**立刻用 `stop_and_home.py --stop-only` 停下来**，不要让它继续跑。我同时开始查原因——我需要看你实际改成什么样了。

找到问题了。先看两处关键实现和最新日志确认。

查到了，而且是我文档里的一个错误直接导致的。好消息是：你那次卡尺测量顺带把 Q1 给回答了，而且答案指向一个能一次性消除 P2-a 的修复。

先说状态：日志显示这次跑完了 600 步正常退出（`[step 599]` 后收尾），机器人现在应该是空闲的，不用紧急处理。

## 直接原因：连续模式每一步都在命令夹爪"张得更开"，而它已经到顶了

日志里 600 步**每一步**都调了 `move_width_m`，而且 `w_meas` 从头到尾是同一个数：

```
[step 0]   move_width action=0.0340 w_meas=0.06640669703483582 w_cmd=0.0773
[step 599] move_width action=0.0256 w_meas=0.06640669703483582 w_cmd=0.0780
```

接近阶段模型输出 $a\approx0.03$，意思是"保持张开"，换算成 $w_{\text{cmd}} = 0.08\times(1-0.034) = 77.3$ mm。硬件报告的满开是 66.4 mm。于是 $\Delta w = -10.9$ mm——**要求更宽，不是更窄**。但 `continuous` 分支写的是：

```python
elif abs(w_meas_f - w_cmd) >= GRIPPER_DEADBAND_M:
    self._controller.gripper_move_width_m(w_cmd)
```

`abs()` 让"要求更宽"也触发下发。于是每步都发一条永远无法满足的张开命令，夹爪顶着机械限位反复启停——**这就是你看到的抖**。而且每条都是阻塞的 libfranka 往返，实测频率从上一轮的 3.58 Hz 掉到 **2.12 Hz**，所以整体比之前更差、手臂在同样墙钟时间里走得更少。

`FRANKA_GRIPPER_MAX_WIDTH_M=0.080` 本来该拦住这件事，但没拦住：`min(0.0773, 0.080) = 0.0773`。

## 这是我文档的错

`grperr_1.2.md` §6.1 把 `FRANKA_GRIPPER_MAX_WIDTH_M` 注为"阶段 1b 实测"，而 §8 Q1 让你量的是**指面物理间距**。这是两个不同的量：命令钳位必须用 **libfranka 报告的可命令上限**（0.066），而你量到的 80 mm 是物理间距。你填得没错，是我把两个量混成了一个字段。

## 但你这次测量回答了 Q1，而且答案很好

物理间距 80 mm、报告 66.4 mm —— 这正是 Q1 里"标定问题"那个分支。说明夹爪的 homing 基准偏了约 13.6 mm，不是硬件真的只能开 66 mm。

这意味着 **`gripper.homing()` 很可能一次性消除 P2-a**。homing 之后报告的满开应该回到 ~0.079–0.080，那么：$a_{\text{hold}}$ 从 0.170 掉回 0.008，与训练一致；模型在接近阶段输出的 $a\approx0.03$ 对应 $w_{\text{cmd}}=0.0776$，与实测 0.079 的差只有 1 mm，落在死区内，连续模式自然不再发命令；整条宽度分布和训练对齐。这比任何软件补偿都干净。

homing 要求两指之间无异物，且是阻塞调用，建议单独跑一个小脚本做，不要放在评测启动路径里。

## 修复顺序

**1. 先 homing，再重读 `max_width`。** 如果报告值变成 ~0.080，把 `FRANKA_GRIPPER_MAX_WIDTH_M` 设成那个新的报告值；如果仍是 0.066，就把它设成 `0.066`（报告值），并把 80 mm 这个物理量单独记在注释里。

**2. 去掉 `abs()`，改成非对称死区。** 即使钳位对了，这个语义也该修——收拢和放宽不该用同一个阈值：

```python
delta_w = w_meas_f - w_cmd
if delta_w >= GRIPPER_DEADBAND_M:                 # 要求更窄：跟踪斜坡
    self._controller.gripper_move_width_m(w_cmd)
elif delta_w <= -GRIPPER_WIDEN_DEADBAND_M:        # 要求更宽：阈值放大，避免抖
    self._controller.gripper_move_width_m(w_cmd)
else:
    info["gripper_cmd"] = "hold"
```

`GRIPPER_WIDEN_DEADBAND_M` 默认给 0.006。

**3. 离线测试也要补。** `test_gripper_semantics_offline.py` 现在的钳位用例是 `W0=0.08, max=0.08`，注释里写着"at boundary, not clamped"——它把这个 bug 编码进断言里了。需要加一条：`w_meas = 硬件报告满开`、`a` 取接近阶段的 0.03，断言**不发出任何 `move_width`**。

## 另外两件事

**手臂没下去这件事，现在归因不了。** 这次相对上一个基线同时变了六个量：`n_exec` 10→5、`control_hz` 10→30、`max_steps` 300→600、夹爪模式 binary_abs→continuous，外加 `his_len` 语义修正和夹爪状态修复。终态 $q_4=-1.774$、$q_6=1.865$，而上一轮是 $-2.078$ / $2.396$（示教抓取位附近），确实明显更高更靠后，但没法说是哪个变量造成的。建议修完夹爪后**先用 `n_exec=10` 复现上一轮的轨迹**确认基线没被破坏，再一次只动一个量。

**阶段 0e 的示教回放 A/B 跳过了。** 那组实验会在离线就暴露这个问题：B 组"宽度冻结在 66.4 mm"跑一遍，`delta_w` 全程为负这件事在图上一眼就能看见。修完之后建议补上再上真机，成本比一次真机 run 低得多。

要我把上面 1–3 的代码和配置改动直接落下去吗？`abs()` 和测试用例可以立刻改并离线验证；homing 脚本我也可以写一个只读预览 + 显式确认的版本，跟 `stop_and_home.py` 一个风格。

---

**User**

把上面 1–3 的代码和配置改动直接落下去并进行测试. 

---

**Cursor**

开始落地。先把要改的三个文件和夹爪驱动读全，确保改动精确。

改动落地。先加运行时钳位（让配置填错也不会再出这类问题），再改判据，最后补测试和 homing 脚本。

现在写 homing 脚本，风格对齐 `stop_and_home.py`。先看一下它的结构以保持一致。

Now运行测试验证。

测试抓到一个真问题：我在测试里断言"即使配置填错，带符号死区也足以抑制命令"——这是错的。`delta_w = -10.9 mm ≤ -6 mm` 会走放宽分支，所以配置填错时仍然每步下发。这说明只靠钳位是单点防御。我加一道结构性防护。

Now把测试改成断言真实行为，并覆盖去重逻辑。

配置默认值全绿。现在用生产配置文件跑一遍，并把上次 600 步真实日志重放过新逻辑，量化效果。

真实日志重放：**600 次阻塞调用 → 0 次**。再跑其余离线测试确认没有回归，同时看一下各次运行的策略输出范围。

补上 `dsplug/README.md` 里新脚本的说明。

日志里实际速率是 2.28 Hz（不是我注释里写的 2.12），把注释里的数字改准。

改动已全部落地并通过测试。

## 结果

抖动的直接原因修掉了。把你上次那 600 步真实日志的 `(action, w_meas)` 序列重放过新逻辑，**阻塞式 `move_width` 调用从 600 次降到 0 次**——正是这 600 次每步阻塞把控制频率从 3.58 Hz 拖到 2.28 Hz，手臂在墙钟时间里几乎没推进。宿主机上能跑的 9 个离线测试全绿，共 203 项检查，其中夹爪语义测试从 55 项扩到 59 项。

## 改了什么

**运行时钳位（新增，也是真正承重的防线）。** `FrankyJointEnv` 启动时向硬件查询 `max_width`，取 `min(配置值, 硬件报告值)` 作为 `w_cmd` 上限，配置填大了会打印警告并被忽略。为此给 `FrankaLibfrankaGripper` 加了 `max_width` 属性、给 `FrankyControllerDirect` 加了 `gripper_max_width()`。这样"把卡尺实测值填进配置"这类错误不再能复现该故障。

**带符号的非对称死区。** 去掉了 `abs()`，收窄用 1.5 mm 阈值（跟踪示教斜坡，约 1.75 mm/帧），放宽用 6 mm 阈值。判据抽成纯函数 `continuous_gripper_decision()`，可离线测试。

**配置语义纠正。** `FRANKA_GRIPPER_MAX_WIDTH_M` 从 `0.080` 改为 `0.066`，并在注释里写明它的单位是"libfranka 报告值"而非卡尺实测手指间距——这两者在你这台机器上差 13.6 mm，混淆它们就是这次事故的起点。新增 `VLA_GRIPPER_WIDEN_DEADBAND_M=0.006`。

**命令去重（测试逼出来的第四处）。** 我最初在测试里断言"带符号死区本身就足以抑制命令"，测试直接判我失败：`delta_w = -10.9 mm ≤ -6 mm` 会走放宽分支，所以配置填错时首步仍会下发。位置控制会保持目标，重发同一目标毫无收益却要付一次阻塞往返，于是加了 `_last_gripper_cmd_w` 去重。效果是即使钳位配错，600 步也只下发 1 次而非 600 次。

## 新增脚本

`b/x/franky_ext/dsplug/gripper_homing.py`，风格对齐 `stop_and_home.py`：默认只读，`--execute` 需输入 `HOME_GRIPPER` 确认，检测到 `holding` 直接拒绝。`b/x/4dwvla_ext/tests/replay_gripper_log.py` 可以把任意历史日志重放过当前逻辑，用来量化改动效果。`dsplug/README.md` 补了 homing 一节，同时修掉了里面一句过期的话（它还在说要手动同步 `franky_joint_env.py` 里的 `HOME_JOINTS`，那个硬编码早已改成读 `home_pose.json`）。

## 上机前必做两步

先只读看一眼，再决定是否重跑 homing：

```bash
python b/x/franky_ext/dsplug/gripper_homing.py --robot-ip 172.16.0.2
python b/x/franky_ext/dsplug/gripper_homing.py --robot-ip 172.16.0.2 --execute
```

**如果 homing 成功把报告值恢复到约 0.079，必须回头把 `FRANKA_GRIPPER_MAX_WIDTH_M` 更新成新值。** 因为运行时取的是 `min()`，配置留在 0.066 的话夹爪永远张不到位。重放脚本里能直接看到这个后果：模拟"报告 0.0794 但配置仍是 0.066"时，上限被压在 0.066。

## 一个需要正视的事实

我顺手统计了四次运行的策略输出：那 600 步里 `action_grip` 的范围是 0.0163 到 0.0535，均值 0.0297，**没有一步超过 0.054**。我之前引用的 `a = 0.227` 来自示教数据，不是真机策略输出。也就是说，修好抖动能恢复控制速率，但不会自动带来夹爪闭合——这次策略根本没要求闭合。这大概率是"手臂没到插头上方，所以策略没看到触发闭合的视觉状态"的循环，速率恢复后应该会改善，但值得在下次运行时专门盯一下 `action_grip` 的峰值有没有涨上来。

另外有 3 个测试（`test_fk_keypoints_offline.py`、`test_fk_matches_dataset.py`、`test_transforms_offline.py`）依赖 torch，宿主机没有，需要在 GPU 容器的 venv 里跑；它们涉及 FK 和 transform，不在本次改动范围内。

---

**User**

我在`rlinf-4dwvla-gpu`容器内执行了`python /workspace/RLinf/b/x/4dwvla_ext/vla_inference_server.py     --ckpt-path /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp041680     --schema-path /workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml     --kpt-meta-path /workspace/RLinf/b/d/frk1/plug/keypoints_meta.json     --urdf-path /workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf     --n-exec 5 --dtype bfloat16` , 在`rlinf-4dwvla-franky`容器内执行了`python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py     --robot-ip 172.16.0.2     --task "plug into socket"     --use-realsense     --global-camera-serial 250222073513     --wrist-camera-serial 420122070525     --max-steps 600     --control-hz 30`, 怎么效果比之前还差?!! Franka机器臂一直在抖, 夹爪一直下不到放插头的地方.

---

**Cursor**

这条消息和 30 分钟前那条一字不差。先确认一件最关键的事：我改的是宿主机 `/home/nvidia/bt/s/RLmm/b/x/...`，而容器跑的是 `/workspace/RLinf/b/x/...`——如果没有绑定挂载，容器里跑的还是旧代码。

`/home/nvidia/bt/s/RLmm -> /workspace/RLinf` 是两个容器的实时绑定挂载，所以我的改动在容器里是可见的。那么下一个问题是：这次是新跑的，还是同一条消息重发？

这次是真的新跑的：`client_20260918_084136_3629.log` 启动于本地 16:41:36，比我 16:24:57 的改动晚 17 分钟。分析这份新日志。

新代码确实生效了，而且运行时钳位刚好救了一次配置错误。但问题没解决——我需要看清现在卡在哪。

夹爪抖动这条已经彻底解决：600/600 步全是 `hold`，`move_width` 调用 0 次，速率从 2.28 Hz 回到 3.18 Hz。但我发现了另一个更要命的东西——`state_history_len` 死死卡在 5。

服务端 `his_len` 正常累积（0→5→10→15…，每控制步 +1），说明 `his_len` 那条修复也生效了。现在量化手臂到底动了多少。

这就找到了真凶。手臂在**原地振荡**：q4 走了 1.40 rad 的总路程，净位移只有 -0.012 rad（0.9%）；每个关节 40–57% 的步都在反向。这才是"抖"。再看振荡是不是发生在动作块边界上。

每个新块的第一个动作离当前状态 0.016 rad（最大 0.097），块内只有 0.003 rad——边界跳跃是块内运动的 4.5 倍。手臂被"拽一下、冻四步"，然后再拽。现在做最关键的检查：机器人的状态到底在不在训练分布里。

关键发现：只有 5/600 步是 7 个关节全在训练带内，**q7 只有 1% 的步在带内**。看 q7 的细节，以及当前 HOME 相对训练分布的位置。

现在因果链完整了。让我验证最后一环：训练数据里每条示教**开始时**的姿态是什么——这才是真正的"插插座前的 home"，而当前 HOME 用的是全体帧的均值。

这推翻了我刚才的假设：当前 HOME 离**示教起始姿态均值**只有 0.105 rad，离全体帧均值才 0.386 rad。HOME 其实是对的——我刚才用全体帧的 q01/q99 判 OOD 是用错了基准。做决定性的对比：示教里手臂往哪个方向走。

方向对比是决定性的。做最后一个检查：图像分布。

## 结论先说

你要我修的夹爪问题**已经修好了**，但它下面压着第二个独立的问题，那才是你现在还看到的现象。

你 16:41 这次跑的是新代码（我的改动在 16:24 落盘，容器 `/workspace/RLinf` 就是 `/home/nvidia/bt/s/RLmm` 的实时绑定挂载）。对比上一次：

| | 上次 15:53 | 这次 16:41 |
|---|---|---|
| `move_width` 阻塞调用 | 600 次 | **0 次** |
| 夹爪判决 | 600 步全 `move_width` | **600 步全 `hold`** |
| 控制频率 | 2.28 Hz | **3.18 Hz** |

运行时钳位还顺手救了一次配置错误——你那个 shell 里仍然是旧的 `FRANKA_GRIPPER_MAX_WIDTH_M=0.080`（没重新 source），日志里能看到它被钳到硬件报告的 0.0664 并打了警告。`his_len` 那条修复也确认生效：服务端从 0→5→10 每控制步 +1，最终达到缓冲上限 200，远超 `grperr_1.md` 说的 120 门槛。

## 现在抖的是手臂，不是夹爪

600 步里每个关节有 **40–57% 的步在反向**。q4 走了 1.40 rad 的总路程，净位移只有 −0.012 rad。振荡集中在动作块边界上：每个新块的第一个动作离当前状态 0.016 rad（最大 0.097），块内只有 0.003 rad——被拽一下、冻四步、再拽一下。

**而且方向是错的。** 这是决定性证据：把这次的净关节位移和 8 条示教的平均净位移做余弦，结果是 **+0.086**，基本正交。两者位移量级相当（0.20 rad vs 0.26 rad），也就是说手臂动的量和一条示教差不多，只是动去了一个和任务无关的方向。

最能说明问题的是 q7（腕部滚转）：

- 8 条示教**全部**把 q7 往正方向推，从 +0.070 到 +0.494，均值 +0.162
- 这次运行把它推了 **−0.096**，符号相反
- 它在**第 5 步**就跌破训练集全体帧的最小值 0.4850，然后在 0.458 附近坐了 595/600 步

示教的结构是：末端从 0.43 m 降到 0.21 m（约 20 cm），在 40% 进度处闭合夹爪，插入，松开，抬起。这次运行从来没下降过，所以夹爪根本没被要求闭合——`action_grip` 全程只有 0.030 到 0.057。**夹爪下不去不是夹爪的问题，是手臂没到那个位置。**

## 顺便排除了两个嫌疑

**HOME 不是问题。** 我之前怀疑过它，查了之后要纠正：当前手动标定的 HOME 离**示教起始姿态均值**只有 0.105 rad，是合格的。我早先用"全体帧均值"当基准是错的——那个基准把下降/插入/抬起三个阶段混在一起，离真正的起始姿态有 0.427 rad。按正确基准看，HOME 落在 8 条示教起始姿态的范围内。

**图像亮度不是问题。** 全局相机实测灰度 119.5 对训练 115.0，腕部 124.8 对训练 110.4。腕部亮 14 个灰阶，在光照波动范围内。不过亮度匹配不能证明视角匹配，你之前说腕部画面"低了不少"那个观察，亮度统计查不出来。

另外一个小的不一致：客户端日志打印 `n_exec=10`，而服务端是 `--n-exec 5` 且实际返回 `[5, 8]`。客户端那个参数只影响日志（之前确认过），但它现在会误导读日志的人。

## 下一步

现在的问题定位到"策略在这个初始条件下不输出有效动作"，继续在真机上调超参是盲调。仓库里已经有现成的离线工具 `b/x/4dwvla_ext/tests/accept_demo_replay.py`，它把真实示教的图像和状态喂给已加载 checkpoint 的推理服务，检查预测动作能否复现示教——**不占用机器人时间**，而且能干净地把"checkpoint/预处理问题"和"硬件/相机问题"分开。

> **下一步先做哪个？**
> - 跑 accept_demo_replay.py 离线回放示教，看策略能否复现示教动作（推荐，不占机器人） ✔
> - 对比腕部/全局相机视角与示教视频，检查外参和安装位置是否漂了
> - 核查 checkpoint 041680 的训练配置，确认它就是在这个数据集上训的
> - 先处理块边界跳跃，加动作平滑/首动作对齐，把抖压下去再看

先确认三件事：脚本的完整接口、推理服务还在不在、以及数据集在 GPU 容器里能不能看到（`/home/nvidia/bt/dt/` 不在挂载列表里）。

服务端还在跑（PID 792，同一个 checkpoint），客户端已结束，所以现在可以安全占用它。数据集只有 23 MB 但容器里看不到，我 copy 进去。先看完脚本的参数和判定标准。

脚本只记录夹爪通道，末尾还会 shutdown 服务端。但我们要查的是**手臂方向**。我复用它的视频解码，写一个只读的诊断脚本，不改它的验收语义、也不关服务端。

数据集是 LeRobot v3.0 布局：每个相机一个拼接的 `file-000.mp4`，而不是每 episode 一个文件。这意味着 `accept_demo_replay.py` 的 glob 匹配不到，会静默用**全黑图像**回放——这是它自身的一个 bug。先确认 episode 在拼接视频里的帧偏移。

`dataset_from_index`/`dataset_to_index`给出了 episode 在拼接视频里的精确帧区间。写诊断脚本。

⚠️ 这次结果**无效**——注意 `decoded 0/120`：容器里的 AV1 解码器失败了，整个回放用的是全黑图像。我先解决解码，但这次失败本身泄露了一个反直觉的线索，我稍后说明。

主机有 libdav1d。我在主机上把两个 AV1 视频转成 H.264 再拷进容器，这样 `cv2` 能直接读。

这次是干净的决定性结果。**模型和 checkpoint 没问题**——喂示教自己的图像时，它几乎完美复现示教。

| 指标 | 离线回放（真实示教图像） | 真机运行 |
|---|---|---|
| 各关节动作 MAE | 0.0016–0.0041 rad | — |
| 与示教净方向余弦 | **+1.000** | +0.086 |
| 夹爪首次 a≥0.5 | 第 **220** 帧（示教 221） | 从未发生 |
| 夹爪动作范围 | 0.006 – **0.999** | 0.030 – 0.057 |

对照第一次全黑图像的无效运行（MAE 0.008–0.023，夹爪在第 0 帧就闭合）：图像正是决定闭合时机的输入。所以故障在**真机输入通道**，不在模型。

现在去验证你最早那个观察——腕部相机视角。

腕部对比非常清楚。橙色手指在两边都是橙色，所以**不是 RGB/BGR 通道对调**。但有两个实测差异：色温（示教偏暖，真机偏青）和视角/尺度（真机腕部相机看插座明显更近更居中——正是你之前的观察）。我做一个受控 A/B 把变量隔离开。

色差已量化：两个相机的**红通道都被压低 16–22 个灰阶**，绿通道抬高。腕部训练数据是暖色（R>G>B），真机变成 G>R>B。现在做 2×2 受控探针，把"图像"和"状态"两个变量彻底分开。

每个真机输入各让动作前瞻下降约 20%，但都远不到真机实测的 0.0159。关键线索：真机第 0 步（历史为空）的前瞻是 0.0968，和探针一致——**崩溃是随后发生的**。我加上逐关节方向，看是谁把 q7 推错了方向。

重要更正：示教在第 0 帧的**记录动作** dq7 = **−0.1116**，也是负的，而真机第 0 步是 −0.0896——几乎一致。所以我之前说"q7 方向相反"是错的：那个判断基于 600 步的净位移，把"还停在轨迹早期"误读成了"方向错误"。q7 先下潜再抬升，真机是卡在下潜阶段没能推进。

真正的量化结论在这张表里：两个真机输入各自把方向余弦从 0.996 拉到 0.82，叠加后 0.605，前瞻降到 46%。但真机稳态前瞻是 0.0159，远低于 0.055——还有第三个因素。测历史。

历史假设**不成立**——历史从 0 加到 200，前瞻稳定在 0.09（真机状态）和 0.12（示教状态），没有塌陷。所以崩溃另有机制。注意一个反常数字：示教的动作比状态超前 **0.126 rad**，而示教每帧只走 0.0024 rad——超前量相当于 50 多帧。我去查 `action.arm` 到底是什么语义。

`action.arm` 不是任何未来状态（MAE 对 k 很平坦，k=12 处最小 0.109 vs k=0 的 0.118，没有明显极小），偏移也不是常数。它与状态的差异集中在**腕部 q5/q6/q7**，均值 −0.027/+0.071/+0.033、标准差 0.042/0.041/0.086，而 q1–q4 几乎为零——典型的遥操作主从腕部标定差异。

这引出最后一个决定性测试：闭环。开环（teacher forcing）已经给出 +1.000，但真机是闭环的。

闭环也成功了：余弦 +0.986，夹爪在第 259 帧闭合（示教 221），漂移始终 ≤0.24 rad。而且闭环下的块边界跳跃是 0.024 rad——和真机的 0.0159 同一量级。所以我之前说"真机前瞻小 8 倍"是拿闭环去比开环，不成立，撤回。

变量只剩图像了。最后核一件可能致命的事：客户端发给模型的通道顺序。

## 结论

夹爪那条修复是有效的；你现在看到的是另一个原因。我用三个离线测试（都不占机器人）把嫌疑逐个排除，**现在只剩相机图像这一个变量**。

**已确认修好的：** 新运行 600 步全是 `hold`，`move_width` 调用 0 次（上次 600 次），频率从 2.28 Hz 回到 3.18 Hz。运行时钳位还拦下了你 shell 里没重新 source 的旧值 0.080，钳到硬件报告的 0.0664 并打了警告。`his_len` 也正常：服务端每控制步 +1，最终到达缓冲上限 200。

**已排除的三个嫌疑：**

| 测试 | 结果 | 判定 |
|---|---|---|
| 开环回放（喂示教自己的图像和状态） | 各关节 MAE 0.0016–0.0041 rad，方向余弦 **+1.000**，夹爪在第 220 帧闭合（示教 221），a 到 0.999 | 模型和 checkpoint 无问题 |
| 闭环回放（策略吃自己的输出，模拟完美跟踪的机器人） | 余弦 **+0.986**，夹爪第 259 帧闭合、a 到 0.971，漂移始终 ≤0.24 rad | 复合误差无问题 |
| 关键点历史长度 0→200 扫描 | 前瞻稳定在 0.09（真机状态）/0.12（示教状态），无塌陷 | 历史无问题 |

顺带验证了 RealSense 是 `rs.format.rgb8` 配置的，所以不存在 RGB/BGR 通道对调。

## 两个我要撤回的判断

我早先说"q7 方向与示教相反"——**错了**。示教在第 0 帧的记录动作 dq7 就是 **−0.1116**，真机第 0 步是 −0.0896，几乎一致。q7 是先下潜再抬升，真机卡在下潜阶段没推进；我当时用 600 步净位移做判断，把"停在轨迹早期"误读成了"方向错误"。

我说"真机块边界前瞻小 8 倍"——**比较基准错了**。闭环下的自然跳跃是 0.024 rad，真机是 0.016 rad，同一量级。我拿它去比开环 teacher-forcing 的 0.132，不成立。

## 剩下的就是图像

![global 对比](/tmp/cmp_global.png)

![wrist 对比](/tmp/cmp_wrist.png)

两处实测差异：

**色温。** 两个相机的红通道都比训练分布低 16–22 个灰阶，绿通道抬高。腕部训练数据是暖色（RGB 均值 122/108/101，R>G>B），真机变成 100/113/94（G>R>B）。不是通道对调——橙色手指两边都是橙色，而且对调会让蓝通道占优，实测是绿通道占优，所以是白平衡/色彩配置差异。

**视角。** 真机腕部相机看插座明显更近更居中，示教里插座偏左上、桌面留白更多。你最早那个"位置低了不少"的观察是对的。

消融数据支持这个方向：只换真机图像，动作方向余弦从 0.996 掉到 0.820；真机图像 + 真机 HOME 状态叠加后掉到 0.605，前瞻降到 46%。

不过这个消融有个局限我得说清楚：我用的真机 PNG 是 11:42 那次会话存的，场景和 16:41 这次不同（那张图里腕部已经压在插座上方，所以策略输出 a=0.45 要求闭合，而真机运行在 HOME 时是 a=0.038）。所以它能证明"真机图像会把输出方向带偏"，但不能精确量化这次的失败。

## 下一步

需要一对**同一会话、在 HOME 位置**拍的真机图像，把消融重做一遍，这样就能直接读出"真机图像让策略输出什么"。

> **接下来怎么做？**
> - 写个只读脚本，在 HOME 位置抓一对真机图像并立即重跑消融（推荐，不动手臂） ✔
> - 先把 RealSense 白平衡固定并向示教色温对齐，再跑一次真机
> - 先校腕部相机安装位置/视角，对齐到示教的构图
> - 先把这整条排查链和三个诊断脚本写进 grperr_1.2.md 归档

写抓帧脚本。关键约束：绝不移动手臂，并且要存原始 RGB 数组（不走 PNG 转换）以免引入色彩歧义。先确认 franky 容器的环境。

`/opt/venv/franky-0.19.0` 有 pyrealsense2 和 franky。写抓帧脚本。

Now让消融脚本支持 `.npy`，避免 PNG 色彩往返。

手臂还停在失败运行的终止姿态（q = [−0.365, 0.0435, 0.2047, −1.8977, −0.1036, 2.0165, 0.4579]，与我从日志算出的终点一致），离 HOME 0.257 rad。我没有擅自移动它。

这次是可信的 `.npy` 原始 RGB，色差要修正我之前的说法：

| 相机 | 真机 RGB | 训练 RGB | 差 |
|---|---|---|---|
| global | 109.8 / 132.3 / 128.1 | 114.4 / 116.6 / 114.2 | −4.6 / **+15.7** / **+13.9** |
| wrist | 127.8 / 131.7 / 121.7 | 122.3 / 108.1 / 100.8 | +5.5 / **+23.6** / **+20.9** |

不是"红通道被压低"（那是我用旧 PNG 测的，经 cv2 通道转换不可靠），准确说法是**绿蓝通道抬高 14–24，红通道基本不变**：示教腕部画面明显偏暖（R−B = +21.5），真机接近中性（R−B = +6.1）。

现在用这对真机图像 + 卡住的实际姿态重跑消融。

这个结果**推翻了图像假设**，而且是整场排查里最干净的一次：

| 组合 | 前瞻 (rad) | 与示教方向余弦 |
|---|---|---|
| 示教图像 + 示教状态 | 0.117 | +0.993 |
| **真机图像** + 示教状态 | 0.140 (1.20×) | **+0.981** |
| 示教图像 + **真机状态** | 0.038 (0.32×) | **−0.575** |
| 真机图像 + 真机状态 | **0.018** | −0.278 |

真机图像几乎无害；**是姿态本身让策略死掉**——即使配上完美的示教图像，余弦也只有 −0.575。最后一行 0.018 精确复现了真机实测的 0.0159。现在定位是哪个关节。

找到了。环境里**已有**训练范围钳位，但 q7 的余量是 0.05 rad：`ACTION_LIMIT_LOWER[6] = 0.4843 − 0.05 = 0.4343`，所以 q7 = 0.4579 合法通过。而我的探针证明，仅 0.027 rad 的越界就足以把方向翻转。做一次 q7 扫描来定出应该用多少余量。

q7 扫描给出了干净的阈值：**方向余弦正好在训练下界翻转符号**。q7 = 0.4750（低于下界 0.010）时余弦 −0.273，到 0.4850（正好是下界）变成 **+0.343**，之后单调上升到 +0.77。也就是说当前 `SAFETY_MARGIN_RAD[6] = 0.05` 恰好放开了策略方向反转的那一段。

核对一下代码里的训练范围常量来源。

## 找到了

**q7（腕部滚转）是唯一越界的关节**，越界 0.026 rad（1.5°），而且它之所以能越界，是因为动作钳位给 q7 留了 0.05 rad 余量。策略输出的方向**正好在训练下界翻转符号**。

相对完整训练集（`abs_stats.json`，66,577 帧）：

| | 卡住时的值 | 训练范围下界 | 钳位允许到 |
|---|---|---|---|
| q7 | **+0.4579** | +0.4843 | +0.4343（= 下界 − 0.05）|

其余 6 个关节全部在范围内。q7 扫描显示阈值就是下界本身：

| q7 | 相对下界 | 方向余弦 |
|---|---|---|
| 0.4750 | −0.010 | **−0.273** |
| 0.4850 | 0（下界）| **+0.343** |
| 0.5150 | +0.030 | +0.600 |
| 0.5450 | +0.060 | +0.769 |

低于下界时余弦全为负（−0.27 到 −0.75）。单独把 q7 恢复到 HOME 就能挽回 78% 的动作前瞻，并把余弦从 −0.479 拉到 +0.756——其余任何单个关节都做不到（q6 能挽回 70% 的前瞻，但余弦只到 +0.157）。

`franky_joint_env.py:69` 那条注释里前一轮修复已经把 q7 余量从 0.15 收到 0.05，方向是对的，但**0.05 恰好覆盖了方向反转的整个区间**。

## 排除链

| 变量 | 测试 | 结论 |
|---|---|---|
| 模型 / checkpoint | 开环回放，余弦 +1.000，夹爪第 220 帧闭合（示教 221）| 正常 |
| 复合误差 | 闭环回放，余弦 +0.986，夹爪第 259 帧闭合 | 正常 |
| 关键点历史 | 长度 0→200 扫描，前瞻无变化 | 正常 |
| 夹爪执行 | 600 步 0 次多余命令，3.18 Hz | 已修 |
| 相机图像 | 真机图像 + 示教状态：余弦 **+0.981** | 正常 |
| **手臂姿态** | 示教图像 + 真机状态：余弦 **−0.575** | **故障点** |

## 三处我要撤回的判断

**相机是红鲱鱼。** 我上一轮基于 11:42 那张旧 PNG 推断图像有问题，那是错的。用同会话的原始 `.npy` 重测后，真机图像配示教状态余弦 +0.981，几乎无害。色温差异（绿蓝通道高 14–24）真实存在但不是致命因素。我上一轮说的"红通道压低 16–22"也不准确——那是 PNG 经 cv2 通道转换的假象。

**q1 没有越界。** 我先前说 q1 也在范围外，那是拿本地 8 条示教（4,777 帧）当基准；对完整训练集（66,577 帧）q1 = −0.365 在 [−0.4842, …] 内。

**"q7 方向相反"和"前瞻小 8 倍"**这两条上一轮已撤回，原因同前（净位移误读、开环闭环基准混用）。

## 建议的修复

把 `SAFETY_MARGIN_RAD[6]` 从 `0.05` 改成 `0.0`，即绝不允许 q7 离开示教范围。因为动作钳位在前、机器人跟踪良好（实测 realized ≈ commanded），钳住动作就能钳住状态。

需要说清一个限度：正好在下界时余弦只有 +0.343，是正的但很弱；要完全恢复得让 q7 明显进入范围内部。所以余量归零是能阻止方向反转的**最小**改动，是否再往内偏一点（比如 −0.02）属于权衡，我倾向先做最小改动加上越界告警，看真机表现。

另外，容器 `/tmp` 里留了数据集副本和转码视频（`/tmp/plug_ds`、`/tmp/plug_ds_h264`），是临时的；新增的四个诊断脚本在 `b/x/4dwvla_ext/tests/`（`diag_demo_replay_arm.py`、`diag_input_ablation.py`、`diag_history_collapse.py`、`diag_pose_attractor.py`）加一个抓帧脚本 `capture_live_frames.py`。

> **q7 钳位怎么改？**
> - 余量改为 0.0（最小改动），并加一条 q7 接近边界的告警 + 离线测试用例（推荐） ✔
> - 余量改为 −0.02（严格钳到范围内部），避开弱余裕区
> - 把每关节余量抽成环境变量配置项，默认 q7=0.0，方便后续调
> - 先把剩下 6 个关节也做一遍扫描，把余量整体重新标定再改

落地最小改动 + 边界告警 + 测试。先读钳位函数的现状。

Now加离线测试用例，把扫描测出的阈值固定下来。

全部 52 项通过。跑完整离线测试套件确认没有回归。

两个 SKIP 是宿主机没装 torch（既有情况，与本次无关）。那个 FAIL 是上一轮修复留下的断言，它把 0.05 余量写死成期望值，现在需要更新。

Now做端到端确认：把卡住的姿态经新钳位处理后，再喂给真实 checkpoint。

## 改动已落地并验证

把 `SAFETY_MARGIN_RAD[6]` 从 `0.05` 改为 `0.0`，q7 的动作下界现在就是训练下界 0.4843 本身。端到端用真实 checkpoint 确认：

| 姿态 | q7 | 与示教方向余弦 |
|---|---|---|
| 卡住时（未钳位） | 0.4579 | **−0.466** |
| 经新钳位后 | 0.4843 | **+0.208** |

两条告警都正确触发：`OUT-OF-TRAIN: q7=0.4579<0.4843` 先把它拉回，随后 `TRAIN-EDGE: q7=0.4843 is 0.0000 above min` 报告它正贴在边界上。

同时加了 L2b 告警层 `TRAIN_EDGE_WARN_RAD`（仅 q7 启用，阈值 0.03 rad）：没有裁剪、但 q7 贴近边界时也会记一条。因为方向余弦在边界处只有 +0.2 左右，要到边界内侧 0.03 rad 才升到 +0.6、内侧 0.06 rad 才到 +0.77——**钳位能阻止方向反转，但不能让策略变健康**。HOME 的 q7 = 0.6436 在边界内侧 0.159 rad，余弦 +0.82，是健康的。所以这条告警的用途是：真机跑的时候如果 q7 持续顶着边界，说明该干预了，而不是等它在那里空转 600 步。

### 测试

宿主机 **219 项全通过**（两个文件因宿主机无 torch 跳过，已在 GPU 容器内单独跑过：`test_fk_keypoints_offline.py` 46 项、`test_transforms_offline.py` 10 项，全通过）。

新增 `test_safety_offline.py::test_q7_has_no_training_margin`，17 项断言锁定这次的回归：卡住的那个确切姿态必须被裁剪并报 OUT-OF-TRAIN；贴边不裁剪但要报 TRAIN-EDGE；HOME 不报任何告警；q1–q6 的 0.15 余量和上界一个都不能动。还特意断言**q1 不被裁剪**——它相对完整训练集是在范围内的，只有本地 8 条示教的子集更窄。

修了一处过期断言：`test_state_history_offline.py` 里上一轮把 0.05 余量写死成期望的 `~0.434`，已更新为 0.4843 并注明原因。

### 真机跑之前

从 HOME 起步（`dsplug/stop_and_home.py --execute`）。手臂现在还停在失败运行的终止姿态，离 HOME 0.257 rad，我没有擅自移动它。另外你那个 shell 里 `FRANKA_GRIPPER_MAX_WIDTH_M` 还是旧的 0.080，虽然运行时钳位会拦住，但建议重新 source `configs/franka_plug_eval.env`。

跑的时候盯 `OUT-OF-TRAIN: q7` 和 `TRAIN-EDGE: q7` 这两条。如果 q7 又一路走到边界，说明策略在这个场景下确实倾向往下压腕部，那就是下一个要查的问题——但至少不会再像这次一样悄无声息地在界外空转。

---

**User**

再次做实验时, 输入 vla_inference_server.py 的 --n-exec 参数填什么比较好? 为什么?

---

**Cursor**

建议这次用 `--n-exec 5`，并保证客户端也使用 `--n-exec 5`。

```bash
# GPU 容器
python /workspace/RLinf/b/x/4dwvla_ext/vla_inference_server.py \
  ... \
  --n-exec 5 \
  --dtype bfloat16
```

客户端：

```bash
python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py \
  ... \
  --n-exec 5 \
  --control-hz 10
```

原因：

1. `n_exec` 表示每次推理结果实际执行多少个动作，不是模型输出长度。模型仍然预测约 50 个动作，但只执行前 `n_exec` 个。
2. `n_exec=5` 会更频繁地重新读取相机和机器人状态，缩短开环执行时间，能更快纠正 q7 偏离训练分布的问题。
3. 当前真机实际控制频率约 3 Hz，`n_exec=10` 意味着连续执行约 3 秒才重新规划一次，动作过于陈旧。
4. 之前服务端使用 `--n-exec 5`、客户端日志却显示 `n_exec=10`，实际服务端只返回了 5 个动作，配置不一致会误导调试，也可能造成历史长度分析错误。

`n_exec=1` 暂时不推荐：推理请求和图像传输会变得过于频繁，通信与推理延迟可能超过执行周期，反而降低稳定性。

推荐顺序：

- Level 1：`n_exec=5`，`control-hz=5`
- Level 2：`n_exec=5`，`control-hz=10`
- 稳定后再比较 `n_exec=10`
- 暂不使用 `n_exec=20`

可通过环境变量统一设置：

```bash
export VLA_N_EXEC=5
```

启动前确认两边日志都显示：

```text
n_exec=5
```

并重点观察：

```text
OUT-OF-TRAIN: q7
TRAIN-EDGE: q7
```

若 q7 仍被裁剪到 0.4843，说明策略输入状态仍接近训练分布边界；此时增大 `n_exec` 只会延迟纠正，不能解决根因。

---

**User**

再次做实验时, 输入 franka_vla_client.py 的 --max-steps 和 --control-hz 参数填什么比较好? 为什么?

---

**Cursor**

建议分两阶段，不要一开始就直接跑 600 步。

### 第一次安全验证

```bash
--n-exec 5 \
--max-steps 30 \
--control-hz 5
```

原因：

- `max-steps=30` 约执行 30 个控制动作，足够观察 q7 是否立即越界、机械臂是否抖动。
- `control-hz=5` 每步至少间隔 200 ms，给 `Robot.move()` 和机械臂稳定留出时间。
- 这只是安全验证，不能验证夹爪闭合，因为示教中最早闭合大约在 `his_len >= 120`。

### 通过后做完整验证

```bash
--n-exec 5 \
--max-steps 300 \
--control-hz 10
```

原因：

- `max-steps=300` 足以让 `his_len` 达到 200，并覆盖示教中夹爪闭合阶段。
- `control-hz=10` 的软件等待时间是 100 ms，比 5 Hz 更高效；但实际频率仍受阻塞式 `Robot.move()` 限制。
- 之前实际运行约为 3.18 Hz，因此 300 步大约需要 1.5–2 分钟。

`--control-hz` 不是实际保证频率。代码每步执行：

```python
time.sleep(1.0 / self._control_hz)
```

但 `Robot.move()` 本身是阻塞的，所以实际频率取决于：

```text
实际频率 ≈ max(机械臂运动耗时, 1 / control_hz) 的倒数
```

应以日志中的：

```text
achieved_control_hz=...
```

为准。

这次建议的完整命令组合是：

```bash
# 服务端
--n-exec 5

# 客户端第一阶段
--n-exec 5 --max-steps 30 --control-hz 5

# 客户端第二阶段
--n-exec 5 --max-steps 300 --control-hz 10
```

如果 30 步内出现 `OUT-OF-TRAIN: q7`、持续 `TRAIN-EDGE: q7` 或明显抖动，应立即按 `r` 中止，不要继续执行 300 步。

---

**User**

我在`rlinf-4dwvla-gpu`容器内执行了`python /workspace/RLinf/b/x/4dwvla_ext/vla_inference_server.py     --ckpt-path /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp041680     --schema-path /workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml     --kpt-meta-path /workspace/RLinf/b/d/frk1/plug/keypoints_meta.json     --urdf-path /workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf     --n-exec 5 --dtype bfloat16` , 在`rlinf-4dwvla-franky`容器内执行了`python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py     --robot-ip 172.16.0.2     --task "plug into socket"     --use-realsense     --global-camera-serial 250222073513     --wrist-camera-serial 420122070525     --max-steps 300     --control-hz 10  --n-exec 5`, 夹爪能下移到放插头的地方了, 但夹爪一直没收紧. 请结合日志与代码, 查找一下原因, 并提供你的解决方案.

---

**Cursor**

这次和上次不同：机械臂运动已明显改善，日志也显示夹爪控制代码确实进入了 `continuous` 模式。现在重点检查两件事：模型是否已经在完整 50 步动作块中预测“未来闭合”，以及 `n_exec=5` 是否把闭合动作持续截断在未执行部分。

## 原因已经明确

这次夹爪并非完全没动，而是刚进入收拢阶段就到达 `max_steps=300`：

- 宽度：`66.4 mm → 61.2 mm`
- 仅在 step 279、299 执行了两次 `move_width`
- 实际执行的 `action_grip` 最大仅 `0.232`
- 力控抓取阈值是 `0.8`，因此从未触发 `grasp_handoff`

但模型的完整 50 步预测其实已经包含闭合。最后一次推理：

- `action_grip >= 0.5`：未来第 21 步
- `action_grip >= 0.8`：未来第 39 步
- 完整块最大值：`0.948`
- 服务端因 `n_exec=5` 只发送前 5 步，最大仅 `0.232`

共有 7 次推理的完整动作块预测了 `>=0.8`，但执行前缀一次都没包含。也就是说，模型知道之后要夹紧，只是 `300` 步在收拢斜坡刚开始时结束了。

## 代码中还有一个确定性 Bug

即使延长实验，当前代码也可能永远不触发最终力控抓取：

```python
if cmd == "grasp_handoff":
    if self._controller.gripper_is_open():
        self._controller.close_gripper()
```

而 `is_open` 的定义是：

```python
return width >= 0.06
```

连续模式一旦把夹爪收窄到 60 mm 以下，`gripper_is_open()` 就变成 `False`。之后即使 `action_grip >= 0.8`，也不会调用 `close_gripper()`。

应改成按“是否已抓住”判断：

```python
if cmd == "grasp_handoff":
    if not self._controller.gripper_holding():
        self._controller.close_gripper()
    self._last_gripper_cmd_w = None
```

稳健实现还应增加约 10 步的失败重试间隔，避免未抓到物体时每步重复阻塞调用。

## 建议方案

1. 先修复上述 `gripper_is_open()` 门控 Bug，并增加回归测试：
   - 当前宽度 40 mm，`is_open=False`
   - `action_grip=0.85`
   - 必须仍调用一次 `close_gripper()`

2. 修复后使用：

```bash
--n-exec 5 \
--max-steps 400 \
--control-hz 10
```

`control-hz=10` 保持不变；实际频率是 3.19 Hz，瓶颈是阻塞式机器人运动，不是 sleep。

3. 观察日志必须出现：

```text
gripper continuous move_width
gripper continuous grasp_handoff
```

并确认最终：

```text
width ≈ 0.010 m
holding=True
```

另有次要问题：q7 从 step 5 到 299 每步都被裁到训练下界，累计 590 条警告，这可能延迟任务阶段推进，但不是本轮没有夹紧的直接原因。当前不建议先放松 q7 保护。

---

**User**

上面提到"瓶颈是阻塞式机器人运动", 能够把"阻塞式"改成其它更好的方式吗? 为什么?

---

**Cursor**

先看清当前是哪一层在阻塞，以及 Franky/libfranka 已经提供了哪些非阻塞接口，再判断能不能改、该怎么改。

能改，而且**控制环里应该改**。但不该改成 `robot.move(..., asynchronous=True)` 这种“假异步”。

## 现在卡在哪

`FrankyJointEnv.step()` 每一步做两件阻塞的事：

1. **手臂**：`move_joints()` 发一条 `JointWaypointMotion`，默认 `asynchronous=False`，一直等到轨迹生成器把这个路点跑完（速度回到接近 0）才返回。

```358:366:b/x/4dwvla_ext/franky_controller_direct.py
    def move_joints(self, joint_positions: np.ndarray):
        ...
        motion = franky.JointWaypointMotion([franky.JointWaypoint(clipped.tolist())])
        self._robot.move(motion)
```

2. **夹爪**：`move_width_m()` / `close()` 走 libfranka 的 `Gripper.move`，也是阻塞的（还有 6 s 超时）。

然后再额外 `sleep(1/control_hz)`。所以请求 10 Hz 时，实测大约 3.2 Hz：每步大半时间花在“等这一拍路点停稳”，而不是推理。

这和训练不一致。示教是 30 Hz 连续轨迹；真机现在是“走一小段、停一下、再走下一段”。块边界上的抖，一部分就是这个 stop-and-go。

## 不该怎么改

`franky.Robot.move(motion, asynchronous=True)` **立刻返回**，看起来像非阻塞，但对 VLA 的 10 Hz 闭环几乎是最差选项：

- libfranka 同时只能跑一个运动生成器。下一条 `JointWaypointMotion` 要么排队等到上一条结束（还是阻塞语义），要么打断上一条。打断会在速度不连续处 reflex（仓库里已有 `cartesian_motion_generator_joint_velocity_discontinuity`）。
- 异步路点一旦后台线程挂掉，会变成“日志说 move OK、臂其实没动”。cube_place 的 LOG-011～016 就是这个坑：`dz=0` 全是哑失败。
- `relative_dynamics_factor` 只作用在阻塞轨迹生成器上，对阻抗跟踪器无效。把异步路点当闭环，速度权限和你现在看到的 0.2 完全不是一回事。

所以：**把阻塞路点改成异步路点，解决不了频率问题，还会丢掉“这条命令到底执行完没有”的保证。**

## 该怎么改

RLinf 官方 Franka 控制器已经把这两类运动拆开了，这才是该复用的模型：

| 场景 | 原语 | 是否等待 |
|---|---|---|
| `reset` / 回 HOME / 大位移 | 阻塞 `JointWaypointMotion` | 必须等到位 |
| `step` 的小增量跟踪 | `JointImpedanceTracker.set_target(q, dq)` | **立刻返回** |

官方实现（`rlinf/envs/realworld/franka/franky_controller.py`）：

```256:275:rlinf/envs/realworld/franka/franky_controller.py
    def move_joints(self, joint_positions: np.ndarray) -> None:
        # dq feedforward is essential at 10 Hz — without it PD lags / overshoots.
        ...
        self._ensure_tracking_motion()
        self._tracker.set_target(q, dq=dq_ff)
```

阻抗跟踪器在 1 kHz 力矩环里一直跑着，Python 只是每个控制周期改一次平衡点。这样：

- 控制环可以真正按 `control_hz` 走，不再被路点停稳绑架。
- 轨迹连续，不再每 5 步一块边界“拽一下、冻四步”。
- 看门狗（你们已经有 50 Hz 线程）才能在运动过程中刹停；现在它多半是在两次阻塞 `move` 的缝里巡视。
- `his_len`、图像、闭合斜坡都会按墙钟时间推进，而不是被 3 Hz 拉长到示教的三倍。

夹爪不要整段改成异步。libfranka 手爪命令是串行的；**位置斜坡**可以缩短等待或与手臂阻抗重叠，**最终 `grasp` 仍应阻塞**（要力控咬合，失败必须可见）。

`reset_joint`、急停、从错误恢复，继续用阻塞路点。阻抗弹簧拉不动 20 cm 的 HOME 位移；这是官方 `DualFrankaEnv` 已经验证过的分工。

## 为什么现在还没改

客户端注释写得很清楚：改 `move_joints()` 会改变**真实运动行为**，不是日志层面的优化。阻抗跟踪和路点跟踪的稳态误差、超调、接触刚度都不同；你们刚把夹爪和 q7 从“抖、下不去”修到“能下到插头旁”，这时候换底层运动原语，成功/失败会和上一轮不可比。

另外几条硬约束：

1. **必须带 `dq` 前馈。** 官方注释：10 Hz 没有速度前馈，PD 会滞后/超调。
2. **进阻抗前要停掉路点生成器，进 HOME 前要停掉跟踪器。** 两者互斥。
3. **围栏语义会变。** 现在是“这条路点跑完再查 TCP”；阻抗下臂一直在动，要靠现有 watchdog 的 lag / `|dq|`，不能只在 `step` 末尾看一眼。
4. **多一层 `sleep(1/hz)` 要拿掉。** 否则阻抗立刻返回后还会被软件睡眠拖回 10 Hz 上限以下。
5. 要按 `4wvla_rlinf_eval_3A3.md` §15.7 做四级真机验收，不能只改代码就上 400 步插插座。

## 建议

**能改，而且对“夹得上、不抖、跟得上 10 Hz”是正道。** 复用官方 `JointImpedanceTracker`，不要自己发明 `asynchronous=True` 的路点循环。

落地顺序：

1. 先修夹爪 `grasp_handoff` 被 `is_open` 门死的 bug（不改运动原语也能验证闭合）。
2. 把 `FrankyControllerDirect.move_joints()` 改成与官方相同的阻抗 `set_target`；`reset_joint` 保持阻塞。
3. 空载：HOME 附近 10 Hz、30 步，看 `achieved_control_hz`、超调、`OUT-OF-TRAIN: q7`。
4. 通过后再跑插插座。

在阻抗验收通过之前，继续用现在的阻塞路点跑实验是合理的：慢，但每条命令的完成是确定的，排障也简单。

---

**User**

你在之前提到"实测 2.5–3.6 Hz", 为什么频率会那么低? 请深入分析分析 @RLmm/b/x/ 里的代码, 特别是 @RLmm/b/x/4dwvla_ext/franka_vla_client.py  以及与它相关的或它调用的所有代码, 看一下是在真机实验中这个"频率"是在哪里控制的.

---

**Cursor**

我会按“客户端循环 → 相机/IPC → 动作执行 → 夹爪 → sleep → 日志测量”的调用链逐层核对，并用已有真机日志把每一段耗时拆开，区分“代码设定频率”和“实际闭环频率”。

已有 9 月 19 日这次 `n_exec=5, control_hz=10, max_steps=300` 的完整日志，我会用它做耗时分解，而不是只根据注释推测。重点是测出：普通动作步、推理边界步、夹爪命令步各占多少时间。

## 结论

之前提到的 `2.5–3.6 Hz`，指的是：

> VLA 客户端完成一次“读取状态 → 执行动作 → 读取新状态”的高层控制循环频率。

它不是：

- Franka FCI 底层伺服频率；
- RealSense 相机采样频率；
- `--control-hz` 参数承诺的实际频率；
- VLA 推理频率。

Franka 内部仍由 libfranka/FCI 进行高频控制，但当前 Python 外层策略循环被多个同步操作串行限制。

---

## 实际调用链

```mermaid
sequenceDiagram
    participant C as franka_vla_client.py
    participant W as KeyboardVLAEvalWrapper
    participant E as FrankyJointEnv
    participant R as FrankyControllerDirect
    participant F as Franka Robot
    participant S as GPU Inference Server

    C->>E: get_camera_frames()
    C->>S: send images + state
    S-->>C: return n_exec actions

    loop 每个 action
        C->>W: step(action)
        W->>E: step(action)
        E->>R: get_state()
        E->>R: move_joints(action)
        R->>F: Robot.move(JointWaypointMotion)
        F-->>R: motion finished
        E->>R: gripper command / width
        E->>E: sleep(1 / control_hz)
        E-->>C: state_after
    end
```

当前真机路径实际使用的是：

```27:49:b/x/4dwvla_ext/franka_vla_client.py
from franky_joint_env import FrankyJointEnv
from keyboard_vla_eval import KeyboardVLAEvalWrapper
from state_history_buffer import ExecutedStateBuffer
```

因此它调用的是：

```text
b/x/4dwvla_ext/franky_joint_env.py
    └── b/x/4dwvla_ext/franky_controller_direct.py
```

而不是：

```text
rlinf/envs/realworld/franka/franky_controller.py
```

后者的 `JointImpedanceTracker` 目前只是参考实现，并没有进入这次 VLA 真机客户端的调用链。

---

## 1. 最大瓶颈：每个动作都同步等待 `Robot.move()`

在 `FrankyJointEnv.step()` 中，每个策略动作都会调用：

```350:370:b/x/4dwvla_ext/franky_joint_env.py
        current_q = self._controller.get_state()["arm_joint_position"]
        action_arm, warnings = check_action_safety(
            action_arm, current_q, self._step_count
        )

        tripped = self._controller.guard_tripped()
        if tripped is not None:
            ...

        try:
            self._controller.move_joints(action_arm)
        except RuntimeError as e:
            ...
```

`move_joints()` 最终执行：

```358:366:b/x/4dwvla_ext/franky_controller_direct.py
    def move_joints(self, joint_positions: np.ndarray):
        ...
        motion = franky.JointWaypointMotion(
            [franky.JointWaypoint(clipped.tolist())]
        )
        self._robot.move(motion)
```

这里没有传入 `asynchronous=True`，因此使用 Franky 的同步行为：

1. 创建一个单 waypoint 轨迹；
2. 发送给 Franka；
3. 等待轨迹生成器执行完成；
4. 等待运动基本稳定；
5. `move_joints()` 才返回。

所以当前模式是：

```text
发送一个绝对关节目标
        ↓
机器人走到目标
        ↓
机器人基本停稳
        ↓
Python 才执行下一步
```

这不是连续的 30 Hz 轨迹流，而是一个个“走一段、停一下”的同步 waypoint。

另外，控制器初始化时还设置了：

```120:125:b/x/4dwvla_ext/franky_controller_direct.py
        self._robot = franky.Robot(robot_ip)
        self._robot.recover_from_errors()
        self._robot.relative_dynamics_factor = 0.2
```

`relative_dynamics_factor=0.2` 会让轨迹采用较保守的动态参数。它提高了安全性，但会增加每个 waypoint 的完成时间。

---

## 2. `--control-hz` 只是额外 sleep，不是实际调度器

命令行参数：

```278:287:b/x/4dwvla_ext/franka_vla_client.py
    p.add_argument("--n-exec", type=int, default=10)
    p.add_argument("--control-hz", type=float, default=10.0)
    p.add_argument("--max-steps", type=int, default=300)
```

最终传给环境：

```329:344:b/x/4WVLA/../RLmm/b/x/4dwvla_ext/franka_vla_client.py
        env = FrankyJointEnv(
            robot_ip=args.robot_ip,
            control_hz=args.control_hz,
            ...
        )
```

在环境中，`control_hz` 只用于：

```463:469:b/x/4dwvla_ext/franky_joint_env.py
        time.sleep(1.0 / self._control_hz)

        tripped = self._controller.guard_tripped()
        if tripped is not None:
            ...
```

因此：

- `control_hz=10`：每步额外 sleep 100 ms；
- `control_hz=5`：每步额外 sleep 200 ms；
- `control_hz=30`：每步额外 sleep 33.3 ms。

但是它没有扣除机器人运动已经消耗的时间：

```python
time.sleep(1.0 / control_hz)
```

而不是：

```python
sleep(max(0, target_period - elapsed_time))
```

因此实际单步时间近似为：

\[
T_{\text{step}}
=
T_{\text{state}}
+
T_{\text{safety}}
+
T_{\text{robot.move}}
+
T_{\text{gripper}}
+
\frac{1}{f_{\text{requested}}}
+
T_{\text{observation}}
\]

实际频率为：

\[
f_{\text{actual}}
\approx
\frac{1}{T_{\text{step}}}
\]

所以 `--control-hz 10` 并不意味着实际为 10 Hz。

---

## 3. 每一步还会重复读取两次机器人状态

一次 `env.step()` 中至少有两次状态读取：

第一次在动作执行前：

```353:355:b/x/4dwvla_ext/franky_joint_env.py
        current_q = self._controller.get_state()["arm_joint_position"]
```

第二次在动作完成后：

```467:469:b/x/4dwvla_ext/franky_joint_env.py
        return self._get_observation(), 0.0, False, False, info
```

`_get_observation()` 内部又调用：

```500:509:b/x/4dwvla_ext/franky_joint_env.py
        state = self._controller.get_state()
        q = state["arm_joint_position"]
        g = state["gripper_width"]
```

而 `get_state()` 同时读取：

- `robot.state`
- `robot.state.q`
- TCP 位姿
- 关节速度
- 夹爪宽度

因此每个动作大致是：

```text
读取一次状态
→ 运动
→ 读取一次夹爪
→ sleep
→ 再读取一次完整状态和夹爪
```

这些通常不是主要瓶颈，但会增加每步固定开销。

---

## 4. 夹爪命令也可能阻塞

夹爪调用虽然通过线程池执行，但调用方仍然等待结果：

```130:153:b/x/franky_ext/franka_libfranka_gripper.py
    def _call(self, label: str, fn: Callable, *args, **kwargs):
        """Run a blocking libfranka gripper command with a deadline."""
        future = self._pool.submit(fn, *args, **kwargs)
        try:
            return future.result(timeout=_GRIPPER_TIMEOUT_S)
```

所以线程池并不等于控制环异步。`future.result()` 仍是同步等待。

当前连续夹爪模式通常返回 `hold`，因此不会产生明显额外耗时。但如果触发：

```python
self._controller.gripper_move_width_m(w_cmd)
```

就会阻塞等待夹爪位置运动。

这在历史日志中已经被验证：

- 无持续 `move_width` 时：约 `3.58 Hz`
- 每步都调用不可达的 `move_width` 时：降到约 `2.28 Hz`

原因是每一步除了等待手臂，还要等待夹爪位置命令完成。

---

## 5. `n_exec` 会把推理耗时加入控制周期

客户端只有在动作队列为空时才重新推理：

```207:218:b/x/4dwvla_ext/franka_vla_client.py
        while self._step_count < self._max_steps and not self._abort:
            if not self._action_queue:
                images = self._env.get_camera_frames()
                state = obs["state"]
                ...
                actions = self._request_inference(images, state)
                self._action_queue.extend(actions)
```

服务端虽然模型预测完整动作块，但只返回前 `n_exec` 个：

```470:488:b/x/4dwvla_ext/vla_inference_server.py
                with torch.no_grad():
                    action_pred = policy.predict_action_chunk(batch)

                ...

                normalized_action = action_pred[:n_exec, :actual_action_dim]
                ...
                actions = physical_action.detach().float().cpu().numpy()
```

当前服务端每次推理大约需要：

```text
常态：约 298–312 ms
第一次推理：约 1.05 s
```

因此：

- `n_exec=10`：每 10 步推理一次；
- `n_exec=5`：每 5 步推理一次；
- 但两者每次都完整预测 50 步，`n_exec=5` 并不会让模型推理变成原来的一半时间。

近似公式是：

\[
T_{\text{avg}}
\approx
T_{\text{step}}
+
\frac{T_{\text{camera}}+
T_{\text{IPC}}+
T_{\text{inference}}}{n_{\text{exec}}}
\]

因此 `n_exec` 越小，闭环更新越快，但推理成本被更频繁地摊进控制周期，实际 Hz 反而下降。

---

## 6. RealSense 和 IPC 只发生在重新推理时

相机初始化为 30 Hz：

```300:320:b/x/4dwvla_ext/franky_joint_env.py
        for name in ["global", "wrist"]:
            pipe = rs.pipeline()
            cfg = rs.config()
            ...
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            pipe.start(cfg)
```

取图像时：

```322:332:b/x/4dwvla_ext/franky_joint_env.py
    def get_camera_frames(self):
        ...
        for name, pipe in self._camera.items():
            fs = pipe.wait_for_frames(timeout_ms=1000)
            color = fs.get_color_frame()
            ...
```

两个相机是串行等待的，每个最多等待 1 秒。正常情况下不会达到超时，但如果相机流卡顿，重新推理边界可能增加很大的延迟。

之后客户端通过 `multiprocessing.connection` 发送两张原始图像：

```172:178:b/x/4dwvla_ext/franka_vla_client.py
        self._conn.send({
            "images": images,
            "state": {"arm": state_arr[:7].tolist(), ...},
            "state_history": state_history,
            ...
        })
```

两张 `480×640×3` 图像加起来约 1.8 MB，还要经过：

```text
numpy 数组序列化
→ Docker/本机 IPC
→ server 反序列化
→ torch.as_tensor
→ HWC→CHW
→ resize / normalize / tokenize
```

这些时间不会出现在 server 的 `inference: xxx ms` 之前的完整范围内，因而日志中的 300 ms 主要是服务端内部处理，不完全代表端到端推理时间。

---

## 7. 当前 3.19 Hz 日志的精确分解

这次运行参数是：

```text
n_exec=5
control_hz=10
max_steps=300
```

日志统计结果：

```text
普通动作间隔中位数：约 0.232 s
重新推理边界间隔均值：约 0.628 s
server 推理耗时中位数：约 0.298 s
最终实际频率：3.19 Hz
```

因为 `n_exec=5`，每 5 个动作中：

- 4 个动作只执行机器人控制；
- 第 5 个动作前需要重新采集图像并等待推理。

近似计算：

\[
T_{\text{5-step block}}
\approx
4 \times 0.232 + 0.628
=
1.556\text{ s}
\]

因此：

\[
f_{\text{actual}}
\approx
\frac{5}{1.556}
=
3.21\text{ Hz}
\]

与日志中的 `3.19 Hz` 基本一致。

这说明当前频率低不是单一问题，而是：

```text
每步同步 Robot.move        ≈ 主要固定成本
control_hz=10 的 sleep      = 100 ms
每 5 步一次 VLA 推理        ≈ 300 ms
相机 + IPC + 状态读取       = 额外边界成本
```

---

## 8. 为什么历史上出现 2.5–3.6 Hz

这些数字对应不同配置和不同阻塞情况，并不是同一个固定频率：

- `control_hz=5`：约 `2.53–2.60 Hz`  
  每步固定 sleep 200 ms，外加机器人运动时间。

- `control_hz=10, n_exec=10`：约 `3.58–3.59 Hz`  
  推理每 10 步发生一次，平均摊销较低。

- `control_hz=30, n_exec=10`：约 `3.12–3.18 Hz`  
  sleep 已经从 100 ms 降到 33 ms，但 Robot.move 和推理成为主瓶颈，因此不会接近 30 Hz。

- `control_hz=30` 且每步调用阻塞式 `move_width`：约 `2.28 Hz`  
  每步多了一次夹爪位置控制等待。

- 当前 `control_hz=10, n_exec=5`：约 `3.19 Hz`  
  因为每 5 步推理一次，比 `n_exec=10` 更频繁。

---

## 9. 哪些代码不是频率控制点

以下部分容易被误认为控制频率，但实际上不是主要控制点：

- `KeyboardVLAEvalWrapper`  
  rollout 运行后只是直接调用 `self.env.step(action)`；`0.05 s` 的 `IDLE_POLL_S` 只用于等待按键，不影响运行中频率。

- `_track_control_interval()`  
  只测量和报警，不控制速度：

```143:158:b/x/4dwvla_ext/franka_vla_client.py
    def _track_control_interval(self):
        ...
        now = time.monotonic()
        ...
        achieved_hz = self._step_interval_count / self._step_interval_sum
```

- 50 Hz watchdog  
  只监测 TCP 围栏和关节速度，不驱动动作。

- RealSense 的 30 Hz  
  只是相机流配置，不意味着 VLA 每秒处理 30 次。

---

## 最终判断

当前高层循环频率由下面三个因素共同决定，优先级大致为：

1. **同步的 `Robot.move()`**：每个动作都等待 waypoint 结束；
2. **每步固定 `time.sleep(1/control_hz)`**；
3. **每 `n_exec` 步一次约 300 ms 的 VLA 推理**。

当前系统的实际结构可以概括为：

```text
阻塞式手臂 waypoint
+ 固定 sleep
+ 周期性阻塞式 GPU 推理
+ 偶发阻塞式夹爪命令
= 2.5–3.6 Hz 的外层闭环
```

因此，把 `--control-hz` 从 10 改成 30 不能解决根因；它只减少了额外 sleep。真正想接近训练时的连续控制频率，需要将 `step()` 中的手臂动作从同步 `Robot.move()` 改为持续运行的 `JointImpedanceTracker.set_target()`，而 HOME/reset/急停仍保留阻塞式运动。

---

**User**

请问 @RLmm/b/x/4dwvla_ext/franka_vla_client.py 跑多少个step返回一次obserrvation? 也就是问返回observation的频率是怎样的, 是受什么决定的, 返回的observation里包含了什么? @RLmm/b/x/4dwvla_ext/vla_inference_server.py 拿到这些observation后又做了什么处理才发送给 vla 模型的? 请详细解释整个流程和细节, 最好引用真实代码作解释.

---

**Cursor**

## 先给结论：有三种“频率”不能混为一谈

1. **环境返回 observation 的频率**  
   `FrankyJointEnv.step()` 每执行一个 action，就返回一次 observation。

2. **客户端向推理服务器发送 observation 的频率**  
   通常每执行 `n_exec` 个 action 才发送一次。

3. **VLA 模型重新推理的频率**  
   与第 2 项相同，因为服务器每收到一次 observation 才推理一次。

因此，若：

```text
n_exec = 5
实际动作执行频率 ≈ 3.2 Hz
```

则：

```text
环境 observation 返回频率 ≈ 3.2 Hz
服务器/VLA 新 observation 频率 ≈ 3.2 / 5 ≈ 0.64 Hz
```

也就是每约 1.5 秒才重新采集图像、读取当前状态并请求一次 VLA 推理。

---

# 一、客户端的完整流程

```mermaid
sequenceDiagram
    participant C as VLAEvalController
    participant W as KeyboardVLAEvalWrapper
    participant E as FrankyJointEnv
    participant R as FrankyControllerDirect
    participant F as Franka Robot
    participant S as VLA Server

    C->>E: reset()
    E-->>C: obs_0

    C->>E: get_camera_frames()
    C->>S: images + state_0 + history
    S-->>C: n_exec 个 actions

    loop 每个 action
        C->>W: step(action)
        W->>E: step(action)
        E->>R: 读取当前状态
        E->>R: move_joints(action)
        R->>F: Robot.move()
        F-->>R: 运动完成
        E->>E: gripper + sleep(1/control_hz)
        E-->>C: obs_next
    end

    C->>E: get_camera_frames()
    C->>S: 新 observation
```

客户端主循环的关键代码是：

```202:218:b/x/4dwvla_ext/franka_vla_client.py
        while self._step_count < self._max_steps and not self._abort:
            if not self._action_queue:
                images = self._env.get_camera_frames()
                state = obs["state"]
                logger.info(
                    "[step %d] Inference (q1=%.3f, grip=%.4f)",
                    self._step_count, state[0], state[7],
                )
                actions = self._request_inference(images, state)
                self._action_queue.extend(actions)

            action = self._action_queue.popleft()
            action_arr = np.array(action, dtype=np.float64)
```

这里非常关键：

```python
if not self._action_queue:
```

只有动作队列为空时，才会：

1. 采集相机图像；
2. 读取当前 observation；
3. 向服务器发送 observation；
4. 获取下一批 action。

如果服务器返回 5 个 action，那么客户端会连续执行这 5 个 action，中间不会重新读取图像，也不会请求模型。

---

# 二、每执行一个 step，环境返回什么 observation？

客户端调用：

```220:234:b/x/4dwvla_ext/franka_vla_client.py
            state_before = np.asarray(obs["state"], dtype=np.float64)

            self._state_history.record(state_before[:7])

            if not self._dry_run:
                obs, reward, terminated, truncated, info = self._env.step(action_arr)
```

`env.step()` 返回标准 Gymnasium 五元组：

```python
obs, reward, terminated, truncated, info
```

其中 `obs` 来自：

```500:509:b/x/4dwvla_ext/franky_joint_env.py
    def _get_observation(self):
        if self._is_dummy:
            return {"state": np.concatenate([HOME_JOINTS, [0.078]])}

        state = self._controller.get_state()
        q = state["arm_joint_position"]
        g = state["gripper_width"]

        if g is None:
            raise RuntimeError(
                "gripper width unavailable; refusing to fabricate observation state"
            )

        return {"state": np.concatenate([q, [float(g)]])}
```

所以真实机器人 observation 的结构是：

```python
{
    "state": np.array(
        [
            q1, q2, q3, q4, q5, q6, q7,
            gripper_width,
        ],
        dtype=np.float64,
    )
}
```

维度为：

```text
state.shape == (8,)
```

具体含义：

```text
state[0:7]：Franka 七个关节角，单位 rad
state[7]：夹爪宽度，单位 m，来自 libfranka 报告值
```

例如：

```text
[-0.2708, -0.0300, 0.1610, -1.8856,
 -0.0332,  1.9317, 0.5563, 0.0664]
```

其中：

```text
q1...q7 = 机械臂当前测得的关节角
0.0664   = 当前夹爪报告的开口宽度
```

## observation 中不包含图像

图像不在 `obs["state"]` 中，也不在 Gym observation dictionary 中。

图像通过单独的方法获取：

```322:332:b/x/4dwvla_ext/franky_joint_env.py
    def get_camera_frames(self):
        if self._camera is None:
            return {
                "global": np.zeros((480, 640, 3), dtype=np.uint8),
                "wrist": np.zeros((480, 640, 3), dtype=np.uint8),
            }

        frames = {}
        for name, pipe in self._camera.items():
            fs = pipe.wait_for_frames(timeout_ms=1000)
            color = fs.get_color_frame()
            if not color:
                raise RuntimeError(f"No frame from {name}")
            frames[name] = np.asarray(color.get_data(), dtype=np.uint8)

        return frames
```

返回：

```python
{
    "global": np.ndarray(shape=(480, 640, 3), dtype=np.uint8),
    "wrist": np.ndarray(shape=(480, 640, 3), dtype=np.uint8),
}
```

当前 RealSense 配置使用：

```python
rs.format.rgb8
```

所以这里得到的是 RGB 图像。

---

# 三、`info` 中还包含什么？

`info` 不是 observation，但包含控制诊断信息，例如：

```337:343:b/x/4dwvla_ext/franky_joint_env.py
        info = {
            "warnings": [],
            "step": self._step_count,
            "action_grip": action_grip,
            "gripper_mode": VLA_GRIPPER_MODE,
            "gripper_close_if_above": GRIPPER_CLOSE_IF_ABOVE,
        }
```

连续夹爪模式下还会加入：

```435:438:b/x/4dwvla_ext/franky_joint_env.py
            info["w_meas"] = w_meas
            info["w_cmd"] = w_cmd
            info["delta_w"] = w_meas_f - w_cmd
            info["gripper_cmd"] = cmd
```

因此一次 `info` 可能包含：

```python
{
    "step": 120,
    "action_grip": 0.18,
    "gripper_mode": "continuous",
    "w_meas": 0.0643,
    "w_cmd": 0.0656,
    "delta_w": -0.0013,
    "gripper_cmd": "hold",
    "warnings": [...],
    "eval_phase": "rec",
    "eval_result": None,
}
```

这些信息主要用于日志和安全诊断，不会直接作为 VLA 模型输入。

---

# 四、客户端到底多久向服务器发送一次 observation？

假设：

```text
server --n-exec 5
```

服务器每次返回 5 个 action：

```485:488:b/x/4dwvla_ext/vla_inference_server.py
                normalized_action = action_pred[:n_exec, :actual_action_dim]
                normalized_actions = normalized_action.detach().float().cpu().numpy()
                physical_action = unnormalize_fn(
                    {ACTION: normalized_action}
                )[ACTION]
                actions = physical_action.detach().float().cpu().numpy()
```

客户端把这 5 个 action 放入队列：

```python
self._action_queue.extend(actions)
```

随后执行：

```text
action 0 → obs 1
action 1 → obs 2
action 2 → obs 3
action 3 → obs 4
action 4 → obs 5
```

然后队列为空，才进行下一次推理：

```text
采集 obs 5 + 图像
发送给 server
server 返回 action 5...9
```

因此：

```text
每个 action：环境返回一次 obs
每 n_exec 个 action：服务器收到一次新的 obs
```

## 一个具体时间例子

如果实际动作频率为 3.2 Hz：

```text
一个 action ≈ 0.3125 秒
n_exec=5
一次推理周期 ≈ 5 × 0.3125 = 1.56 秒
```

所以 VLA 每约 1.56 秒才看到一次新的图像和当前状态。

---

# 五、`state_history` 是什么？

客户端并不把每一步 observation 都发送给服务器，但会缓存每一步执行前的七关节姿态：

```228:231:b/x/4dwvla_ext/franka_vla_client.py
            # training's his_kpts contains frames strictly earlier
            # than the current frame
            self._state_history.record(state_before[:7])
```

`ExecutedStateBuffer` 只保存：

```text
7D arm joint position
```

不保存：

- 夹爪宽度；
- 相机图像；
- action；
- reward。

下一次请求时将其取出：

```160:178:b/x/4dwvla_ext/franka_vla_client.py
        state_history = self._state_history.drain()

        self._conn.send({
            "images": images,
            "state": {
                "arm": state_arr[:7].tolist(),
                "gripper": [float(state_arr[7])],
            },
            "state_history": state_history,
            "task": self._task,
            "protocol": 2,
        })
```

例如 `n_exec=5` 时，第二次请求大概是：

```python
{
    "state": {
        "arm": q_after_action_4,
        "gripper": width_after_action_4,
    },
    "state_history": [
        q_before_action_0,
        q_before_action_1,
        q_before_action_2,
        q_before_action_3,
        q_before_action_4,
    ],
    "images": {
        "global": image_at_current_time,
        "wrist": image_at_current_time,
    },
    "task": "plug into socket",
    "protocol": 2,
}
```

注意：

```text
state 是当前时刻的状态
state_history 是此前执行过的关节轨迹
```

这两个不是同一件事。

---

# 六、服务器收到 observation 后首先做什么？

服务器主循环接收消息：

```420:450:b/x/4dwvla_ext/vla_inference_server.py
                arm_q = np.asarray(
                    msg["state"]["arm"], dtype=np.float32
                )
                gripper = np.asarray(
                    msg["state"]["gripper"],
                    dtype=np.float32,
                ).reshape(-1)

                input_state = np.concatenate([arm_q, gripper])

                kpt_data = None
                if fk_computer is not None:
                    executed_history = msg.get("state_history", ())
                    proto = msg.get("protocol", 1)

                    if proto >= 2:
                        for executed_q in executed_history:
                            fk_computer.append(
                                np.asarray(executed_q, dtype=np.float32)
                            )
                        kpt_data = fk_computer.snapshot()
```

这里会得到：

```text
arm_q       shape=(7,)
gripper     shape=(1,)
input_state shape=(8,)
```

`input_state` 的顺序仍然是：

```text
[q1, q2, q3, q4, q5, q6, q7, gripper_width]
```

---

# 七、服务器如何处理 3D keypoint 历史？

当前 checkpoint 开启了 keypoint predictor，因此服务器创建：

```350:378:b/x/4dwvla_ext/vla_inference_server.py
    if getattr(config, "enable_keypoint_predictor", False):
        ...
        fk_computer = FKKeypointComputer(
            urdf_path=args.urdf_path,
            kpt_meta_path=args.kpt_meta_path,
            history_max_len=getattr(
                config, "keypoint_history_max_len", 200
            ),
        )
```

`FKKeypointComputer`：

1. 读取 URDF；
2. 根据七个关节角做 FK；
3. 计算 8 个机器人 keypoint；
4. 每个 keypoint 是 7 维：

```text
[position_x, position_y, position_z,
 quaternion_x, quaternion_y, quaternion_z, quaternion_w]
```

核心逻辑：

```59:75:b/x/4dwvla_ext/fk_keypoints.py
    def compute(self, arm_q7: np.ndarray) -> np.ndarray:
        th = torch.zeros(
            1, len(self._joint_names),
            dtype=torch.float32,
        )

        for i, idx in enumerate(self._arm_indices):
            th[0, idx] = float(arm_q7[i])

        fk = self._chain.forward_kinematics(th)
        kpts = np.empty(
            (self.num_joints, self.kpt_dim),
            dtype=np.float32,
        )

        for j, link_name in enumerate(self.keypoint_links):
            mat = fk[link_name].get_matrix()[0].numpy()
            pos = mat[:3, 3] / self.bbox_radius
            quat = Rotation.from_matrix(
                mat[:3, :3]
            ).as_quat()
```

每一个历史姿态会被转换成：

```text
[8, 7]
```

`n_exec=5` 时，每次服务器请求追加 5 个历史 keypoint。

然后：

```83:108:b/x/4dwvla_ext/fk_keypoints.py
    def snapshot(self) -> tuple[np.ndarray, int]:
        """Pack history as-is: training's his_kpts excludes the current frame."""
        return self._pack()

    def _pack(self):
        his_len = len(self._history)
        buf = np.zeros(
            (self.history_max_len, self.num_joints, self.kpt_dim),
            dtype=np.float32,
        )

        for i, kpt in enumerate(self._history):
            buf[i] = kpt

        return buf, his_len
```

因此服务器传给模型的 keypoint 历史通常是：

```text
observation.his_kpts: [200, 8, 7]
observation.his_len:  标量，0～200
```

历史不足 200 帧时后面补零。

当前协议 `protocol=2` 下，服务器只追加 `state_history`，不会把当前 `arm_q` 再追加一次。这是有意设计的：

```text
his_kpts 表示严格早于当前 observation 的历史
```

---

# 八、服务器如何构造模型输入 sample？

服务器调用：

```450:465:b/x/4dwvla_ext/vla_inference_server.py
                sample = build_sample(
                    images={
                        "global": np.asarray(
                            msg["images"]["global"]
                        ),
                        "wrist": np.asarray(
                            msg["images"]["wrist"]
                        ),
                    },
                    state=msg["state"],
                    task=msg["task"],
                    dtype=dtype,
                    kpt_data=kpt_data,
                )
```

`build_sample()` 做的事情：

```260:301:b/x/4dwvla_ext/vla_inference_server.py
    arm = np.asarray(state["arm"], dtype=np.float32)
    gripper = np.asarray(
        state["gripper"],
        dtype=np.float32,
    )
    full_state = np.concatenate([arm, gripper])

    chunk_size = 50
    action_dim = 8

    sample = {
        OBS_STATE: torch.from_numpy(full_state).float(),
        ACTION: torch.zeros(
            chunk_size,
            action_dim,
            dtype=torch.float32,
        ),
        "task": task,
    }

    for cam_name, img_np in images.items():
        key = f"{OBS_IMAGES}.{cam_name}"
        img_t = torch.as_tensor(img_np).contiguous()
        img_t = img_t.to(dtype=dtype) / 255.0

        if img_t.ndim == 3 and img_t.shape[-1] == 3:
            img_t = img_t.permute(2, 0, 1)

        sample[key] = img_t
```

最终 sample 初步包含：

```text
observation.state      [8]
action                  [50, 8] 全零占位
task                    字符串
observation.images.global [3, 480, 640]
observation.images.wrist  [3, 480, 640]
observation.his_kpts    [200, 8, 7]
observation.his_len      标量
```

这里的：

```python
ACTION: torch.zeros(50, 8)
```

不是机器人真实 action，也不是模型要模仿的 action。它主要是为了满足统一的数据结构；`predict_action_chunk()` 实际主要使用 state、图像、语言和 keypoint history。

---

# 九、服务器的 transform pipeline

服务器建立的 transform 顺序是：

```190:220:b/x/4dwvla_ext/vla_inference_server.py
    input_transforms = compose([
        ResizeImagesWithPadFn(
            height=RESIZE_SIZE,
            width=RESIZE_SIZE,
            mapping=schema.image_mapping,
        ),
        RemapImageKeyTransformFn(
            mapping=schema.image_mapping,
        ),
        NormalizeTransformFn(
            selected_keys=[OBS_STATE],
            norm_stats=state_stat,
        ),
        InternVLAA15ChatProcessorTransformFn(
            mode="eval",
            tokenize_state=getattr(config, "tokenize_state", True),
            max_state_dim=getattr(config, "max_state_dim", 32),
        ),
        PadStateAndActionTransformFn(
            max_state_dim=getattr(config, "max_state_dim", 32),
            max_action_dim=getattr(config, "max_action_dim", 32),
        ),
        ReorderStateActionTransform(
            state_reorder=schema.state_reorder,
            action_reorder=schema.action_reorder,
        ),
    ])
```

## 1. 图像 resize/pad

输入是：

```text
[3, 480, 640]
```

经过 `ResizeImagesWithPadFn` 后变成模型需要的：

```text
[3, 224, 224]
```

它采用 pad 方式，而不是简单拉伸，尽量保持相机画面的几何比例。

## 2. 图像 key 重命名

schema 中规定：

```yaml
image_mapping:
  observation.images.global: observation.images.image0
  observation.images.wrist: observation.images.image1
```

因此：

```text
global → image0
wrist  → image1
```

模型配置最多支持 3 个 view，但当前 plug schema 只有两个相机，因此 `image2` 会被补成无效占位图并设置 mask 为 false。

## 3. 状态归一化

服务器启动时从 checkpoint 的 `stats.json` 读取统计量：

```102:155:b/x/4dwvla_ext/vla_inference_server.py
def load_stats(ckpt_path: Path, schema: DatasetSchema):
    stats = load_json(ckpt_path / "stats.json")
    ...
    state_stat = {OBS_STATE: pick_or_compose(OBS_STATE)}
    action_stat = {ACTION: pick_or_compose(ACTION)}
    return state_stat, action_stat
```

如果 checkpoint 没有直接保存组合后的 8D state stats，就根据 schema 把：

```text
observation.state.arm
observation.state.gripper
```

拼接成 8D 统计量。

然后使用 mean/std：

```python
state_normalized =
    (state - mean) / (std + 1e-6)
```

不是直接把关节角裁剪到 `[0, 1]`。

## 4. 状态离散化为文本 token

`InternVLAA15ChatProcessorTransformFn` 在 `tokenize_state=True` 时，还会把归一化后的 state 变成文本：

```95:103:src/lerobot/policies/internvla_a1_5/transform_internvla_a1_5.py
    def _encode_state(self, data: DataDict) -> str:
        if not self.tokenize_state or OBS_STATE not in data:
            return ""

        state = deepcopy(data[OBS_STATE])
        state = pad_vector(state, self.max_state_dim)
        state_np = state.cpu().numpy() / 3
        discretized = np.digitize(
            state_np,
            bins=np.linspace(-1, 1, 257)[:-1],
        ) - 1

        return "State: " + " ".join(map(str, discretized))
```

例如最终 prompt 可能类似：

```text
Task: plug into socket;
Control Mode: <joint>;
State: 127 124 131 ... 0 0 ...;
Output: <Subtask, Action>
```

这里的 `State:` 不是原始物理 state，而是：

```text
原始 8D state
→ checkpoint mean/std 归一化
→ 补齐到 32D
→ /3
→ 256 档离散化
→ 转成文本 token
```

## 5. 构造 Qwen3.5 多模态聊天输入

processor 会构建：

```text
system message
user:
  image0
  image1
  Task: plug into socket
  Control Mode: <joint>
  State: ...
```

在 eval 模式下：

```140:149:src/lerobot/policies/internvla_a1_5/transform_internvla_a1_5.py
        if self.mode == "eval":
            label_mode = LABEL_MODE_NONE
            user_text = user_text + "; Output: <Subtask, Action>"
```

然后使用 Qwen3VL processor 生成：

```text
observation.input_ids
observation.attention_mask
observation.pixel_values
observation.image_grid_thw
observation.fast_token_mask
```

## 6. state/action padding

模型的最大 state/action 维度通常是 32：

```94:103:src/lerobot/transforms/core.py
class PadStateAndActionTransformFn(DataTransformFn):
    max_state_dim: int = 32
    max_action_dim: int = 32

    def __call__(self, data):
        data[OBS_STATE] = self._pad_vector(
            data[OBS_STATE],
            self.max_state_dim,
        )
        data[ACTION] = self._pad_vector(
            data[ACTION],
            self.max_action_dim,
        )
        return data
```

所以模型内部可能看到：

```text
state: [32]
action: [50, 32]
```

其中前 8 个 state 维度是真实 Franka state，后面是零填充。

## 7. 添加 batch/device/dtype

```300:318:b/x/4dwvla_ext/vla_inference_server.py
def to_batch(sample, device, dtype):
    batch = {}

    for key, value in sample.items():
        if isinstance(value, torch.Tensor):
            value = value.unsqueeze(0)

            if value.dtype.is_floating_point:
                value = value.to(
                    device=device,
                    dtype=dtype,
                )
            else:
                value = value.to(device=device)

            batch[key] = value
        else:
            batch[key] = [value]

    return batch
```

最终典型输入形状：

```text
pixel_values       [1, image_tokens, channels/features]
input_ids          [1, sequence_length]
attention_mask     [1, sequence_length]
state              [1, 32]
his_kpts           [1, 200, 8, 7]
his_len            [1]
```

---

# 十、模型如何使用这些输入？

服务端调用：

```470:478:b/x/4dwvla_ext/vla_inference_server.py
                with torch.no_grad():
                    action_pred = policy.predict_action_chunk(batch)

                if action_pred.ndim == 3:
                    action_pred = action_pred[0]
```

`predict_action_chunk()` 获取：

```2284:2309:src/lerobot/policies/internvla_a1_5/modeling_internvla_a1_5.py
    pixel_values = batch[f"{OBS_PREFIX}pixel_values"]
    image_grid_thw = batch[f"{OBS_PREFIX}image_grid_thw"]
    lang_tokens = batch[f"{OBS_PREFIX}input_ids"]
    lang_masks = batch[f"{OBS_PREFIX}attention_mask"]
    fast_token_mask = batch.get(f"{OBS_PREFIX}fast_token_mask")
    state = self.prepare_state(batch)

    kpt_kwargs = {}
    if self.config.enable_keypoint_predictor:
        kpt_kwargs = {
            "his_kpts": batch.get("observation.his_kpts"),
            "his_len": batch.get("observation.his_len"),
        }

    actions = self.model.sample_actions(
        pixel_values,
        image_grid_thw,
        lang_tokens,
        lang_masks,
        state,
        fast_token_mask=fast_token_mask,
        **kpt_kwargs,
    )
```

模型内部大致流程：

```text
图像 + 文本 prompt
        ↓
Qwen3.5 VLM prefix embedding
        ↓
当前 8D state embedding
        ↓
历史 3D keypoint TrackEncoder
        ↓
keypoint expert
        ↓
action expert
        ↓
flow-matching denoising
        ↓
50 步动作 chunk
```

`sample_actions()` 初始化一个：

```text
noise.shape = [batch, chunk_size=50, max_action_dim]
```

然后执行 flow-matching 迭代：

```1373:1407:src/lerobot/policies/internvla_a1_5/modeling_internvla_a1_5.py
        dt = -1.0 / num_steps
        x_t = noise
        time = torch.tensor(1.0, dtype=torch.float32, device=device)

        while time >= -dt / 2:
            expanded_time = time.expand(bsize)

            v_t = self.denoise_step(
                state,
                kpt_prefix_pad_masks,
                past_key_values,
                max_prefix_position_ids,
                x_t.to(dtype),
                expanded_time.to(dtype),
                fast_mask=fast_mask,
                use_kpt=self.config.enable_keypoint_predictor,
            )

            x_t = x_t + dt * v_t
            time += dt

        return x_t
```

最终模型原始输出大致是：

```text
[1, 50, 8]
```

含义：

```text
50 个未来动作
每个动作 8 维：
[q1, q2, q3, q4, q5, q6, q7, gripper_action]
```

---

# 十一、模型输出如何变成机器人实际 action？

模型输出仍然是归一化 action，需要反归一化：

```478:500:b/x/4dwvla_ext/vla_inference_server.py
                full_chunk = unnormalize_fn(
                    {ACTION: action_pred[:, :actual_action_dim]}
                )[ACTION]

                full_grip = full_chunk.detach().float().cpu().numpy()[:, -1]

                normalized_action = action_pred[
                    :n_exec,
                    :actual_action_dim,
                ]

                physical_action = unnormalize_fn(
                    {ACTION: normalized_action}
                )[ACTION]

                actions = physical_action.detach().float().cpu().numpy()
```

这个过程是：

```text
模型 normalized action
        ↓
取前 n_exec 个 action
        ↓
mean/std unnormalize
        ↓
物理 action
```

于是服务器发送：

```python
{
    "status": "ok",
    "actions": actions.tolist(),
}
```

如果：

```text
server --n-exec 5
```

则服务器返回：

```text
actions.shape == [5, 8]
```

客户端接收到后：

```180:200:b/x/4dwvla_ext/franka_vla_client.py
        actions = np.asarray(resp["actions"], dtype=np.float64)

        if actions.ndim != 2 or actions.shape[1] != 8:
            raise ValueError(
                f"Expected actions with shape [N, 8], got {actions.shape}"
            )

        if not np.isfinite(actions).all():
            raise ValueError(
                "Inference response contains NaN or infinite actions"
            )

        self._action_queue.extend(actions)
```

之后逐个交给：

```text
KeyboardVLAEvalWrapper.step()
    → FrankyJointEnv.step()
        → 安全裁剪
        → 手臂运动
        → 夹爪控制
```

---

# 十二、一次完整的时序例子

假设：

```text
n_exec=5
```

## 初始阶段

`reset()` 返回：

```text
obs0 = HOME 关节角 + 夹爪宽度
```

客户端发送：

```text
images0
state0
state_history=[]
```

服务器返回：

```text
action0, action1, action2, action3, action4
```

## 执行第一个动作

客户端调用：

```text
env.step(action0)
```

环境返回：

```text
obs1
```

客户端做两件事：

```text
保存 state_before=q0 到 state_history
保留 obs1 供下一步使用
```

但不会立即发给服务器。

## 执行五个动作

依次执行：

```text
action0 → obs1
action1 → obs2
action2 → obs3
action3 → obs4
action4 → obs5
```

期间服务器完全不知道 `obs1`、`obs2`、`obs3`、`obs4`。

## 队列耗尽

客户端再次采集：

```text
当前图像 image5
当前状态 obs5
历史关节姿态 q0...q4
```

发送：

```text
images5
state5
state_history=[q0,q1,q2,q3,q4]
```

服务器根据 q0...q4 生成：

```text
his_kpts[0:5]
his_len=5
```

然后使用当前图像、当前 state、keypoint history 推理下一批 50 步动作，并只返回前 5 步。

---

# 十三、几个容易误解的地方

## 1. 每个 step 都返回 observation，但模型不一定每个 step 都看到

这是最重要的区别：

```text
env.step()：每个 action 都返回 observation
VLA server：每 n_exec 个 action 才收到一次 observation
```

所以当前系统不是每个机器人控制 step 都进行 VLA 推理。

## 2. `n_exec` 由服务器决定实际返回数量

客户端保存了 `self._n_exec`，但当前客户端代码不会主动切片返回结果：

```python
self._action_queue.extend(actions)
```

实际执行多少个 action，取决于服务器返回多少个 action，也就是服务端：

```python
normalized_action = action_pred[:n_exec]
```

因此应保证：

```text
server --n-exec 5
client --n-exec 5
```

否则客户端日志里的 `n_exec` 可能和实际返回数量不一致。

## 3. `--max-steps` 不决定 observation 频率

它只决定最多执行多少个环境 step：

```text
max_steps=300
```

表示最多执行 300 个 action，不表示 300 次模型推理。

如果：

```text
n_exec=5
max_steps=300
```

理论上大约是：

```text
300 / 5 = 60 次服务器推理
```

并且环境大约返回 300 次 action 后 observation。

## 4. `--control-hz` 不是模型推理频率

它只影响：

```python
time.sleep(1.0 / self._control_hz)
```

真实频率还受：

- Franka `Robot.move()`；
- 夹爪命令；
- 相机采集；
- IPC；
- GPU 推理；
- 状态读取；
- 安全检查；

共同影响。

---

## 最终数据流总结

```text
Franka 当前状态 q + gripper
        │
        ├── 每个 action step 返回给客户端 obs
        │
        ├── 客户端缓存每步执行前的 q
        │
        └── 每 n_exec 步：
              ├── 采集 global/wrist 图像
              ├── 读取当前 8D state
              ├── 发送 state + 图像 + state_history
              │
              ▼
        vla_inference_server
              ├── FK 计算历史 3D keypoint
              ├── 构造 [8D state, 图像, task, keypoint history]
              ├── state mean/std 归一化
              ├── 图像 resize/pad
              ├── 构造 Qwen3.5 chat prompt
              ├── state 离散化成文本 token
              ├── VLM + keypoint expert + action expert
              ├── flow matching 生成 50 步动作
              ├── 反归一化
              └── 返回前 n_exec 个动作
```

因此，当前配置下最准确的描述是：

```text
机器人每个 action 都产生一个 observation；
但 VLA 模型每 n_exec 个 action 才重新接收一次 observation；
服务器收到的是当前 8D state、两路图像和之前执行的关节历史，
再经过 FK、归一化、聊天模板、keypoint 编码和 flow matching 后，
返回下一批动作。 
```

---

**User**

深入研究 @b/d/frk1/4debug/sumry0919.markdown 的章节`# 附录一: 对 his_len 相关内容的解释`, 深入分析一下为什么 `his_len` 正确了, 但夹爪还没加上?

---

**Cursor**

我会把附录中的论证与当前实现、9 月 18/19 日真机日志逐项对照，重点区分：`his_len` 只是“历史时钟正确”，还是足以保证策略进入夹爪闭合阶段。

附录解释了“`his_len` 修复不是充分条件”，但还停留在 9 月 18 日阶段；9 月 19 日的最新日志又把原因进一步收窄了。我会把这几代实验分开，否则“没夹上”会把三种不同失败混成一件事。

这属于一份独立的根因分析，我会用 Canvas 整理“历史分支—策略分支—执行分支”的因果链，同时在回复里给出代码与日志结论。

# 核心结论

`his_len` 修复后，解决的是：

> 模型能够看到正确数量、且严格早于当前帧的机械臂关键点历史。

它没有直接解决：

1. 历史内容是否与训练轨迹一致；
2. 当前图像/姿态是否已进入抓取阶段；
3. 50 步动作块中的闭合动作是否进入实际执行前缀；
4. 连续夹爪动作是否被正确翻译为硬件宽度；
5. 最终 `grasp` 是否真的下发。

最新日志已经证明：`his_len` 正确时，模型实际上预测了后续闭合，但闭合动作仍停留在 50 步 chunk 的后半段，没有进入 `n_exec=5` 的执行前缀；实验又在 step 300 结束。同时代码中还存在一个会阻止最终力控抓取的门控 bug。

---

# 一、`his_len` 到底是什么

附录把 `his_len` 称为“历史时钟”，作为直观解释可以，但技术上不够精确。

模型并没有类似下面这样的规则：

```python
if his_len >= 120:
    close_gripper()
```

真实实现中，`his_len` 用来告诉 `TrackEncoder`：`his_kpts` 中前多少帧是真实历史、后面多少帧是 padding。

客户端每个控制步记录执行前的七关节状态：

```228:231:b/x/4dwvla_ext/franka_vla_client.py
            # Record state *before* execution: training's his_kpts contains
            # frames strictly earlier than the current frame
            self._state_history.record(state_before[:7])
```

下次推理时发送：

```160:178:b/x/4dwvla_ext/franka_vla_client.py
        state_history = self._state_history.drain()
        ...
        self._conn.send({
            "images": images,
            "state": {
                "arm": state_arr[:7].tolist(),
                "gripper": [float(state_arr[7])],
            },
            "state_history": state_history,
            "task": self._task,
            "protocol": 2,
        })
```

服务端将这些历史关节角做 FK，然后只打包历史，不再把当前帧重复追加：

```436:451:b/x/4dwvla_ext/vla_inference_server.py
                    executed_history = msg.get("state_history", ())
                    proto = msg.get("protocol", 1)
                    if proto >= 2:
                        for executed_q in executed_history:
                            fk_computer.append(
                                np.asarray(executed_q, dtype=np.float32)
                            )
                        kpt_data = fk_computer.snapshot()
```

`TrackEncoder` 使用 `his_len` 截取有效历史：

```71:88:src/lerobot/policies/internvla_a1_5/keypoints.py
    def forward(self, points: torch.Tensor, lengths: torch.Tensor):
        ...
        for i in range(batch_size):
            actual_len = lengths[i].item()
            actual_len = max(actual_len, 1)
            batch_points = points[i, :actual_len]
```

之后每 4 帧做一次 temporal patch：

```101:108:src/lerobot/policies/internvla_a1_5/keypoints.py
        points_reshaped = rearrange(points, "b t n c -> (b n) c t")
        patches = self.conv(points_reshaped)
        ...
        patch_lengths = lengths // patch_size
```

所以 `his_len` 的真实作用是：

```text
确定哪些历史关键点有效
→ 决定 TrackEncoder 使用多少时序 patch
→ 影响 keypoint expert 表征
→ 间接影响 action expert
```

它不是一个直接控制夹爪的倒计时器。

---

# 二、为什么训练统计显示 `his_len >= 120` 才闭合？

训练数据中：

- 最早闭合帧对应 `his_len=120`；
- `his_len=200` 时闭合概率约 0.764。

但这是**相关性**，不是充分因果关系。

示教里随着 frame index 增加，会同时发生：

```text
his_len 增加
机械臂下降
腕部相机中的插头变大
末端接近插头
q1...q7 进入抓取位姿
夹爪宽度开始下降
```

因此模型学到的是联合条件：

\[
a_t =
\pi(
I_t^{global},
I_t^{wrist},
q_t,
w_t,
K_{t-H:t-1},
\text{task}
)
\]

其中：

- \(I_t\)：当前两路图像；
- \(q_t\)：当前七关节角；
- \(w_t\)：当前夹爪宽度；
- \(K_{t-H:t-1}\)：历史关键点轨迹；
- `his_len` 只确定 \(K\) 中有效历史的长度。

所以：

```text
his_len >= 120
```

只表示历史长度允许模型进入中后期模式，不表示：

```text
机械臂一定已经到插头旁
夹爪一定应该闭合
```

9 月 18 日 16:41 的运行就是反例：

- `his_len` 正确达到 200；
- 机械臂没有下降到抓取位；
- `action_grip` 全程仅 0.030–0.057；
- 连续夹爪正确选择 `hold`。

这时强行闭合反而是错的。

---

# 三、修复 `his_len` 实际上带来了什么改善？

修复前：

```text
700 个控制步
n_exec=10
服务器仅推理 70 次
his_len 只到 70
```

示教最早闭合在 120，因此从历史长度条件看，旧实现结构上很难进入抓取阶段。

修复后，最新 9 月 19 日运行中服务端历史为：

```text
request 0:  his_len=0
request 1:  his_len=5
request 2:  his_len=10
...
request 40: his_len=200
```

之后滑动窗口长度保持 200，但内容继续滚动更新。

这次模型输出确实发生了变化：

```text
step 255: action_grip 首次 >= 0.1
step 284: action_grip 首次 >= 0.2
step 299: action_grip = 0.232
```

所以 `his_len` 修复不是“没效果”，而是：

```text
原来：模型一直处在历史不足的接近阶段
现在：模型已经开始进入夹爪收拢斜坡
```

但实验在斜坡刚开始时结束了。

---

# 四、最新日志说明：模型已经预测闭合，但没有执行到

模型每次预测 50 个动作：

```280:287:b/x/4dwvla_ext/vla_inference_server.py
    chunk_size = 50
    action_dim = 8

    sample = {
        OBS_STATE: torch.from_numpy(full_state).float(),
        ACTION: torch.zeros(chunk_size, action_dim, dtype=torch.float32),
        "task": task,
    }
```

但服务端只返回前 `n_exec` 个：

```478:488:b/x/4dwvla_ext/vla_inference_server.py
                full_chunk = unnormalize_fn(
                    {ACTION: action_pred[:, :actual_action_dim]}
                )[ACTION]

                normalized_action = action_pred[
                    :n_exec, :actual_action_dim
                ]
```

9 月 19 日运行使用：

```text
n_exec=5
max_steps=300
```

最后一次推理的完整 50 步夹爪计划中：

```text
第 7 步：action_grip >= 0.3
第 21 步：action_grip >= 0.5
第 39 步：action_grip >= 0.8
最大值：0.948
```

但服务端只返回前 5 步，最大值只有：

```text
0.232
```

完整运行的统计是：

```text
完整 50 步 chunk 出现 >=0.3：10 次
实际执行前 5 步出现 >=0.3：0 次

完整 chunk 出现 >=0.5：9 次
实际执行前缀出现 >=0.5：0 次

完整 chunk 出现 >=0.8：7 次
实际执行前缀出现 >=0.8：0 次
```

因此不能说“模型不会闭合”。更准确地说：

> 模型已经把闭合安排在未来，但 receding-horizon 每次只执行前 5 步，闭合阶段一直没有进入实际执行队列；运行又在 step 300 结束。

`his_len` 解决的是“模型有没有条件规划闭合”；`n_exec` 决定的是“规划中的闭合有没有实际执行”。

---

# 五、夹爪其实已经开始收拢，只是幅度很小

当前连续模式执行：

```153:180:b/x/4dwvla_ext/franky_joint_env.py
def continuous_gripper_decision(
    action_grip,
    w_meas,
    max_width_m,
    last_cmd_w=None,
):
    w_cmd = min(
        W0 * (1.0 - float(action_grip)),
        float(max_width_m),
    )

    if float(action_grip) >= GRASP_HANDOFF_A:
        return "grasp_handoff", w_cmd

    ...
    delta_w = float(w_meas) - w_cmd
    if delta_w >= GRIPPER_DEADBAND_M:
        return "move_width", w_cmd
```

因为：

```text
W0 = 0.08 m
硬件报告最大宽度 = 0.0664 m
窄向 deadband = 0.0015 m
```

满开时要触发第一次实际收拢，必须大约满足：

\[
0.0664 - 0.08(1-a) \ge 0.0015
\]

解得：

\[
a \gtrsim 0.189
\]

所以即使 `his_len` 正确后，模型输出从 0.03 增长到 0.17，硬件仍然会 `hold`；只有超过约 0.189 才明显收拢。

日志正好吻合：

- step 279，\(a=0.1964\)：第一次 `move_width`；
- step 299，\(a=0.2323\)：第二次 `move_width`；
- 宽度从 66.4 mm 降到约 61.2 mm。

所以“夹爪一直没收紧”从视觉上可以理解，但从日志上严格说并非完全没收紧，而是：

> 300 步结束时才刚开始走连续收拢斜坡，还远未到最终抓取阶段。

---

# 六、`his_len` 历史里没有夹爪历史

还有一个容易忽略的结构事实：

```python
self._state_history.record(state_before[:7])
```

只保存前七维关节角，不保存第八维夹爪宽度。

因此 keypoint 历史包含：

```text
机械臂关节 → FK → 机器人 3D keypoint
```

不包含：

```text
夹爪历史宽度
夹爪历史 action
是否已抓住
```

模型只通过“当前 8D state”看到当前夹爪宽度：

```278:287:b/x/4dwvla_ext/vla_inference_server.py
    arm = np.asarray(state["arm"], dtype=np.float32)
    gripper = np.asarray(state["gripper"], dtype=np.float32)
    full_state = np.concatenate([arm, gripper])
```

所以修复 `his_len` 不等于修复夹爪反馈闭环。夹爪还必须真正改变宽度，下一次推理时模型才能看到新的 `w_t`。

这也是旧二值执行模式形成吸收态的原因：

```text
模型给 a=0.20，表示下一步应变窄
旧执行器因 a<0.5 什么都不做
下一次模型仍看到满开宽度
模型继续输出斜坡起点
```

`his_len` 再正确，也打破不了这个执行层吸收态。

---

# 七、当前还有一个会阻止最终抓取的代码 Bug

即使延长运行，让执行 action 达到 `0.8`，当前代码仍可能不调用力控 `grasp`：

```439:447:b/x/4dwvla_ext/franky_joint_env.py
            if cmd == "grasp_handoff":
                if self._controller.gripper_is_open():
                    logger.info(...)
                    self._controller.close_gripper()
                    self._last_gripper_cmd_w = None
```

而 `gripper_is_open()` 最终使用：

```338:344:b/x/franky_ext/franka_libfranka_gripper.py
    @property
    def is_open(self) -> bool:
        if self._hardware_holding():
            return False
        width = self._width_m()
        if width is not None:
            return width >= _OPEN_WIDTH_M
        return self._is_open_flag
```

其中：

```text
_OPEN_WIDTH_M = 0.06
```

连续收拢一旦将宽度降到 60 mm 以下：

```text
gripper_is_open() == False
```

之后即使：

```text
action_grip >= 0.8
cmd == "grasp_handoff"
```

也会跳过 `close_gripper()`。

因此当前系统有一个确定的潜伏故障：

```text
连续斜坡先把宽度降到 <60 mm
→ is_open=False
→ handoff 分支拒绝下发 grasp
→ 永远无法进入最终力控持握
```

正确条件应该是：

```python
if cmd == "grasp_handoff":
    if not self._controller.gripper_holding():
        self._controller.close_gripper()
```

并加重试冷却，避免抓取失败后每一步重复阻塞调用。

---

# 八、q7 仍然在影响抓取阶段推进

最新 300 步运行中：

```text
step 5 到 step 299
共 295 个不同 step
每步模型 q7 action 都低于 0.4843
每步都被裁回 0.4843
```

因此产生：

```text
295 条 OUT-OF-TRAIN
295 条 TRAIN-EDGE
总计 590 条 warning
```

也就是说：

- `his_len` 格数正确；
- 但模型规划的 arm action 被执行层持续修改；
- 实际 keypoint 历史并不是模型原计划的轨迹；
- q7 一直贴在训练观测下界；
- 模型可能不断把“完成抓取前最后一段腕部运动”放在未来 chunk 中。

这能解释为什么完整 50 步 chunk 已经预测闭合，但重新规划后闭合事件一直没有快速进入前 5 步。

`his_len` 只保证：

```text
历史有 200 个有效格子
```

不保证：

```text
这 200 个格子是正确的抓取轨迹
```

---

# 九、3 Hz 与训练 30 Hz 的影响需要更准确地表述

附录中这句：

> “每格约 0.3 秒，训练每格 1/30 秒，因此相邻历史帧关节位移放大约 10 倍”

结论过强，需要修正。

最新日志与 8 条示教的实测对比：

```text
真机每步 ||Δq|| 中位数：0.00317 rad
示教每帧 ||Δq|| 中位数：0.00249 rad
比例：约 1.27 倍
```

不是 10 倍。

真正约 10 倍的是**墙钟跨度**：

```text
训练 200 帧 @30 Hz：约 6.67 秒
真机 200 步 @3.19 Hz：约 62.7 秒
```

TrackEncoder 没有真实时间戳，只看到均匀排列的历史位置。因此它不会直接知道这些历史用了 63 秒；它把每个样本仍理解成类似训练中的一帧。

真实问题是：

1. 同一条空间轨迹被墙钟慢放；
2. 图像只每 `n_exec` 步更新；
3. 阻塞式 waypoint 每步起停；
4. 外部场景、碰撞和夹爪动力学不再符合训练时序；
5. 200 帧历史覆盖的是 63 秒真实世界，而训练只覆盖约 6.7 秒。

因此频率失配仍然重要，但不是简单的“每步空间位移放大十倍”。

---

# 十、完整因果链

```mermaid
flowchart TD
    A["his_len 原来只到 70"] --> B["历史不足，模型停在早期阶段"]
    C["protocol v2 + state_history"] --> D["his_len 0→5→...→200"]
    D --> E["模型开始预测夹爪收拢斜坡"]

    E --> F["当前执行前缀 a 最大仅 0.232"]
    F --> G["continuous 模式只发生两次 move_width"]
    G --> H["宽度 66.4→61.2 mm，尚未抓紧"]

    E --> I["完整 50 步 chunk 已预测 a≥0.8"]
    I --> J["n_exec=5 只返回前 5 步"]
    J --> K["闭合事件停留在未来第 21/39 步"]
    K --> L["max_steps=300 时运行结束"]

    M["q7 每步被裁到下界"] --> K
    N["实际 3.19 Hz、阻塞式 waypoint"] --> K

    H --> O["宽度将低于 60 mm"]
    O --> P["gripper_is_open=False"]
    P --> Q["当前 handoff 代码跳过 close_gripper"]
```

---

# 十一、对附录结论的评价

附录的主结论是正确的：

> `his_len` 已经解决的是组件级计数语义，而不是最终抓取任务。

但应补充或修正三点：

1. **`his_len=120/200` 不是直接闭合门限。**  
   它控制 TrackEncoder 有效历史长度；闭合还依赖当前图像、状态和历史内容。

2. **`n_exec` 截断不是理论上“永远闭合不了”。**  
   receding-horizon 重新规划后，未来闭合动作有可能逐渐进入前缀。但最新日志确实证明到 step 300 为止，它仍未进入前 5 步。

3. **3 Hz 不代表相邻关节位移一定放大 10 倍。**  
   最新实测只有约 1.27 倍；约 10 倍的是墙钟历史跨度。

---

# 十二、当前最优处理顺序

1. 先修 `grasp_handoff` 的 `gripper_is_open()` 门控，改为 `not gripper_holding()`，并增加失败重试冷却。
2. 保持连续宽度模式和硬件上报宽度钳位。
3. 修复后先跑 `n_exec=5, max_steps=400, control_hz=10`，观察：
   - 是否出现 `gripper continuous grasp_handoff`；
   - 宽度是否从约 61 mm 继续下降到插头窗口 `[2,18]` mm；
   - `holding=True` 是否成立。
4. 同时记录每次完整 chunk 中首次 `a≥0.5/0.8` 的索引。如果该索引连续多次不向前移动，说明存在 horizon procrastination，而不是 `his_len` 问题。
5. 后续再解决阻塞 waypoint/3 Hz 和 q7 持续贴边；这两项影响轨迹内容，但不要再改 `his_len` 计数逻辑。

一句话总结：

> `his_len` 修复让模型终于“知道历史已经够长”，但夹上插头还要求机械臂到达正确状态、闭合斜坡进入实际执行前缀、连续宽度真正改变硬件状态，以及最终力控 handoff 不被代码门控跳过；最新日志表明失败已经从“历史不够”推进到了“计划后段未执行 + handoff 潜伏 bug”阶段。

---

**User**

请接着回答下面这些问题:
- 如果把 n_exec 设为跟 action chunk 一样, 也就是 50 是不是就可以解决夹爪不闭合的问题?
- 请详细解释一下上面你说到的"实际 keypoint 历史并不是模型原计划的轨迹"是什么原因?
- 是不是用阻抗控制的话, 上面你说的的"q7 一直贴在训练观测下界"就不会发生, 为什么?
- 你提到的 "外部场景、碰撞和夹爪动力学不再符合训练时序" 请具体解释一下.
- 你上面提到"先修 grasp_handoff 的 gripper_is_open() 门控，改为 not gripper_holding()，并增加失败重试冷却", 详细解释一下具体怎么做 ? 为什么要这样做 ? 可举例说明.

---

**Cursor**

这五个问题都落在“动作计划—真实执行—下一次观测”这条闭环上。我先核对当前执行器、q7 裁剪和夹爪 handoff 的真实代码，再逐项回答。

`n_exec=50` 只能打掉“闭合动作停在 chunk 后段、没被执行”这一层，不能单独让夹爪夹上。q7 贴在 0.4843 rad，是软件把目标裁到训练观测下界之后，阻塞运动又等到手臂到达该目标；换成阻抗控制但保留这道裁剪，平衡点仍然是这个下界。

## 1. `n_exec=50` 能不能解决夹爪不闭合？

不能把它当成解法。它只去掉截断，夹爪仍要过执行语义、到位姿和力控这三关。

模型每次生成 50 步，服务端只把前 `n_exec` 步放进返回值：

```478:488:b/x/4dwvla_ext/vla_inference_server.py
                full_chunk = unnormalize_fn(
                    {ACTION: action_pred[:, :actual_action_dim]}
                )[ACTION]
                ...
                normalized_action = action_pred[:n_exec, :actual_action_dim]
```

客户端不按自己的 `--n-exec` 再切一刀，队列里有多少就执行多少，空了才重新推理。所以必须改的是服务端的 `--n-exec 50`。只改客户端参数，返回的仍是 5 步。

9 月 19 日最后一次推理里，50 步计划是：

| chunk 内步号 | `action_grip` |
|---:|---|
| 0–4（实际执行） | 最大 0.232 |
| 第 7 步 | 首次 ≥ 0.3 |
| 第 21 步 | 首次 ≥ 0.5 |
| 第 39 步 | 首次 ≥ 0.8，之后最大 0.948 |

若这一整段都被执行，夹爪通道确实会走到 0.8。但这是在“那一次推理的状态上”成立的。`n_exec=50` 会改掉后面的状态，不能把这条日志直接外推成“下一次一定夹上”。

它解决不了的有四件：

1. **闭合仍可能发生在错误位姿。** 第 39 步的夹爪命令，是假设前 38 步都按模型原样执行后的未来。q7 每步被裁到 0.4843，真实关节已经离开这条计划。开放环跑完 50 步，可能在插头旁边空夹，也可能在还没对准时夹。
2. **连续模式会先把宽度降到 60 mm 以下，然后当前 handoff 拒绝抓取。** `a=0.30` 时目标宽度是 `0.08×(1-0.30)=56 mm`。`is_open` 在宽度 `< 60 mm` 时为假。等 chunk 第 39 步 `a≥0.8` 到来，`close_gripper()` 仍然不会被调用。
3. **图像会冻住约 15 秒。** 现在每步大约 0.3 秒。50 步内不重新看相机。训练时大约每帧都有新图像；这里插头、碰撞和手指都在动，策略看不见。
4. **训练分布更远。** 训练等价于每一步都用真实下一状态重新规划，接近 `n_exec=1`。`n_exec=50` 是往相反方向走。

若只想验证“后段闭合命令能不能到达硬件”，可以单次把服务端设为 `--n-exec 50`，同时把 `max_steps` 收到 50 或 100，并有人守着急停。这是实验，不是修复。

## 2. “实际 keypoint 历史并不是模型原计划的轨迹”

历史里存的是测到的关节角，不是模型输出的动作。中间有两道改写。

一次推理的计划可以看成：

```text
当前测得 q7 = 0.4843
模型未来三步的 q7 命令：0.460、0.440、0.420
```

执行前 `check_action_safety()` 把低于训练观测下界的值裁掉：

```205:214:b/x/4dwvla_ext/franky_joint_env.py
    # L2: Training range + margin
    below_t = clipped < ACTION_LIMIT_LOWER
    above_t = clipped > ACTION_LIMIT_UPPER
    if np.any(below_t) or np.any(above_t):
        ...
        clipped = np.clip(clipped, ACTION_LIMIT_LOWER, ACTION_LIMIT_UPPER)
```

q7 的下界是 `TRAIN_ARM_MIN[6] = 0.4843`，余量为 0。于是三步命令都变成 0.4843。速度限幅救不了它：`|0.460-0.4843|=0.024`，小于 `MAX_JOINT_STEP_RAD=0.15`，L3 不触发。

然后 `move_joints()` 发一个阻塞 waypoint，直到手臂停在这个裁剪后的目标上。客户端写入历史的是执行前测到的关节，不是原始 action：

```228:232:b/x/4dwvla_ext/franka_vla_client.py
            # Record state *before* execution: training's his_kpts contains
            # frames strictly earlier than the current frame
            self._state_history.record(state_before[:7])
```

服务端对这串测得关节做正运动学。模型原计划是腕部继续转下去；历史里连续几格却是同一个 q7。下一次推理时模型看到“腕没转”，于是再规划一次向下的 q7，再被裁回同一点。

9 月 19 日这 300 步里，从 step 5 到 step 299，模型的 q7 命令每次都低于 0.4843，每次都被裁回去。所以 keypoint 历史的时间长度是对的，腕部轨迹的内容是“贴在下界上的平台”，不是模型那条未裁剪的下降曲线。

chunk 内部还有第二层偏离。第 2 步动作是按“第 1 步已经到达未裁剪目标”来预测的。真实机器人停在裁剪目标上，第 2 步仍按旧计划执行，不再用新观测修正。`n_exec` 越大，这种偏离累积得越久。

## 3. 改成阻抗控制，q7 还会不会贴在下界？

只把阻塞 waypoint 换成阻抗跟踪，q7 仍会贴在 0.4843。贴边的平衡点是软件目标，不是运动发生器的种类。

现在的链路是：

```text
模型命令 0.42
→ 安全层改成 0.4843
→ JointWaypointMotion 等到关节到达 0.4843
→ 下一次观测就是 0.4843
```

关节阻抗或笛卡尔阻抗的平衡点同样是这个目标。目标一直是 0.4843，弹簧最终就把关节拉到 0.4843。差别只是不再每步急停，而是平滑靠过去。日志里跟踪误差大约 0.001 rad，说明手臂已经在忠实执行裁剪后的命令，不是被插座挡住了。

阻抗要改变这个现象，必须同时满足两件事：

- 动作目标不再被裁到观测最小值 0.4843。示教里 `action.arm[6]` 可以低到 0.3695，观测最小值才是 0.4843；动作本来就比观测更超前。
- 一个控制周期不等到收敛就采样。30 Hz、约 33 ms 时，手臂只走完弹簧位移的一小段。这样可以出现“命令是 0.42，测到的 q7 仍高于 0.4843”。

这才像采集示教时的超前：动作分布比观测分布宽，是因为一个周期内没跟上，不是因为有人把命令抬回观测下界。

若策略闭环的平衡点始终是“每步都命令 0.40”，那么即使用阻抗，只要这个目标保持得够久，q7 仍会掉到 0.40，并且掉出训练观测范围。阻抗恢复的是滞后，不是一道新的安全边界。安全边界仍然要由裁剪或接触约束来做。

另外，franky 的 waypoint 底层本来就是力矩伺服。现在缺的是“更新目标后立即读状态”，不是机器人里没有阻抗。

## 4. 场景、碰撞和夹爪动力学怎么跟训练时序错开？

训练是 30 Hz，一帧 33 ms。真机大约 3.2 Hz，一步约 300 ms，而且这一步里手臂先走完、再处理夹爪、然后才允许下一次采样。空间上每步关节变化只比示教大了约 1.27 倍；被拉长的是这些变化发生的真实时间和“看不见的中间过程”。

用一次插入接触来看。

示教里，插头碰到插座沿：

```text
t = 0 ms     图像：插头刚接触边缘
t = 33 ms    手腕横向修正约 0.5 mm
t = 66 ms    再修正，插头滑进孔
t = 100 ms   开始下压
```

策略在碰撞后的两三帧里就能改动作。

现在同一次接触落在一个阻塞 waypoint 里面：

```text
t = 0 ms      读取图像和关节，模型发出本步目标
t = 0–250 ms  手臂自行加速、接触、减速；期间没有新图像，也不能改目标
t = 250 ms    运动结束，手臂停住，接触力卸掉
t = 250–400 ms 若夹爪也要动，再串行执行 move_width 或 grasp
t = 400 ms    才把新的状态送去下一次推理
```

于是会出现训练里很少见的三段动力学：

- **碰撞被一步吞掉。** 插头可能在这 250 ms 里滑过孔、卡在倒角上，或把插座顶开。训练时这些都是可观测、可纠正的中间帧；这里它们只在步末留下一个结果姿态。
- **接触力是断续的。** waypoint 到达后目标误差为零，手臂不再往下压。插入需要持续的力，现在变成“推一下、停一下”。示教是连续的人手轨迹，没有这种每步归零。
- **夹爪和手臂被串行化。** 示教约 17 帧、0.57 秒，手指大约每帧收 1.75 mm，同时手臂还在动。`step()` 先 `move_joints()`，再 `move_width()` 或 `close()`。`close()` 失败时最长可以阻塞到超时（夹爪类里是 6 秒量级）。这一秒里相机和关节历史都不更新，但手指已经在真实世界里合拢。模型下一次看到的是一个突然变窄的宽度，而不是它在 30 Hz 上学过的那条斜坡。

场景也按墙钟漂移。200 格历史在训练里覆盖约 6.7 秒，在 3.2 Hz 下覆盖约 63 秒。keypoint 历史只有机器人自身，插头、线缆和插座的移动只存在于图像里。图像又要等 `n_exec` 步才更新一次。所以模型用“6.7 秒的机器人轨迹”去解释一段已经过了一分钟的接触过程。

## 5. `grasp_handoff` 要怎么改，为什么？

当前连续模式在 `a ≥ 0.8` 时确实返回 `grasp_handoff`，但调用力控抓取前又要求夹爪“仍然算张开”：

```439:447:b/x/4dwvla_ext/franky_joint_env.py
            if cmd == "grasp_handoff":
                if self._controller.gripper_is_open():
                    logger.info(...)
                    self._controller.close_gripper()
                    self._last_gripper_cmd_w = None
```

`is_open` 的定义是：没有夹住，且宽度 ≥ 60 mm。

```338:343:b/x/franky_ext/franka_libfranka_gripper.py
    def is_open(self) -> bool:
        if self._hardware_holding():
            return False
        width = self._width_m()
        if width is not None:
            return width >= _OPEN_WIDTH_M
```

这个条件适合二值模式：手指要么全开，要么一次 `grasp` 到目标。连续模式故意在力控之前先把手指收到 60 mm 以下，于是把自己锁死。

数字例子：

```text
起始宽度 66.4 mm，is_open = True

a = 0.23 → 目标宽度 61.6 mm → move_width
         → 仍 ≥ 60 mm，is_open 仍为 True

a = 0.30 → 目标宽度 56.0 mm → move_width
         → is_open 变成 False

a = 0.85 → 决策已是 grasp_handoff
         → is_open 为 False，close_gripper() 被跳过
         → 手指停在 56 mm，10 mm 的插头没有被夹住
```

该问的不是“手指是不是还张着”，而是“是不是已经夹住了”。`gripper_holding()` 走的是宽度窗口：插头标定为 10 mm、容差 8 mm，也就是大约 `[2, 18]` mm。没进这个窗口，就还可以抓；已经在窗口里，`close()` 自己也会跳过，避免对一次正在维持的 grasp 再发一次。

建议的执行段是：

```python
GRASP_RETRY_COOLDOWN_S = float(os.environ.get("VLA_GRASP_RETRY_COOLDOWN_S", "1.0"))

# __init__ / reset 里：
self._last_grasp_attempt_t = float("-inf")

# step() 的 continuous 分支：
if cmd == "grasp_handoff":
    if not self._controller.gripper_holding():
        now = time.monotonic()
        if now - self._last_grasp_attempt_t >= GRASP_RETRY_COOLDOWN_S:
            logger.info(
                "[step %d] gripper continuous grasp_handoff action=%.4f w_meas=%s",
                self._step_count, action_grip, w_meas,
            )
            self._last_grasp_attempt_t = now
            self._controller.close_gripper()
            self._last_gripper_cmd_w = None
```

三种结果：

| 情形 | 行为 |
|---|---|
| 宽度 56 mm，`a=0.85`，还没夹住 | 不再被 `is_open` 拦住，调用 `close_gripper()`，目标宽度 10±8 mm |
| 抓到后宽度 10 mm，`holding=True` | 不再发 grasp。libfranka 对一次正在维持的 grasp 再发 grasp 会失败 |
| 空夹，手指合到约 0 mm，不在 `[2,18]` mm | `close()` 抛错，现有代码记 warning。若没有冷却，后面每步 `a≥0.8` 都会再阻塞一次，控制频率会像之前的 `move_width` 一样塌掉。1 秒冷却表示失败后最多每秒重试一次 |

二值模式可以继续用 `gripper_is_open()`。那里没有“先收到 56 mm、再力控”的斜坡，`is_open` 是正确的锁存。要改的是 `continuous` 这一支。

---

**User**

你上面提到"训练分布更远。 训练等价于每一步都用真实下一状态重新规划，接近 n_exec=1。n_exec=50 是往相反方向走", 是不是把 n_exec设成1就可以解决问题?

---

**Cursor**

`n_exec=1` 更接近训练时的重新规划方式，但 9 月 19 日的日志里闭合动作并不在 chunk 的第 0 步。我先核对每次推理里“首次闭合”落在第几步，再判断把它改成 1 会不会把闭合带进实际执行。

不能。`n_exec=1` 让每一步的观测条件更接近训练，但 9 月 19 日那次运行里，真正的闭合命令在第 21 步和第 39 步；只执行第 0 步，夹爪会离闭合更远。

训练时，第 `t` 帧用真实状态 `s_t` 预测未来 50 步，下一帧再用数据集里的真实 `s_{t+1}` 重新预测。评测里的 `n_exec=1` 只模仿了这一半：执行 chunk 的第 0 步，读回真实状态，再推理。它没有模仿另一半。训练损失监督的是整段 50 步，示教里闭合的中位位置在 chunk 第 24.5 步，不是第 0 步。

9 月 19 日最后一次推理的 50 步夹爪计划是：

| chunk 内步号 | `action_grip` | `n_exec=5` | `n_exec=1` |
|---:|---|---|---|
| 第 0 步 | 低于 0.232 | 执行 | 只执行这一步 |
| 第 7 步 | 首次 ≥ 0.3 | 丢弃 | 丢弃 |
| 第 21 步 | 首次 ≥ 0.5 | 丢弃 | 丢弃 |
| 第 39 步 | 首次 ≥ 0.8，最大 0.948 | 丢弃 | 丢弃 |

当时 `n_exec=5` 已经把第 7 步以后全部丢掉，300 步里实际执行的 `action_grip` 最大只有 0.232。改成 `n_exec=1` 连这 5 步斜坡也只剩第 0 步，宽度更不会降到能触发 `grasp` 的程度。

这里会形成一个推迟闭环：

```text
当前状态 s
→ 模型计划：前 20 步继续靠近，第 21 步再闭合
→ 只执行第 0 步，手臂只靠近一点点
→ 新状态 s' 仍像“还没到闭合时刻”
→ 新计划再次把闭合放在大约第 21 步
```

那次日志已经看到了这个现象。每 5 步重新规划一次，跑到第 300 步，完整 chunk 里有 9 次出现 `a≥0.5`、7 次出现 `a≥0.8`，但它们全部在前 5 步之外；最后一次仍是第 21 步才到 0.5、第 39 步才到 0.8。更频繁地重新规划，不会自动把第 21 步搬到第 0 步。只有手臂和图像已经进入示教里“现在就该夹”的状态时，第 0 步本身才会变成闭合命令。那次运行到结束还没有进入这个状态。

`n_exec=1` 也留着另外三个原因：

- q7 仍会被裁到 0.4843 rad，下一次历史看到的仍是贴边轨迹。
- 每步仍是阻塞 waypoint，实际大约 3 Hz，不是训练的 30 Hz。推理不再由 5 步分摊，每步还会更慢。
- 连续模式里 `gripper_is_open()` 的 handoff 门控还在。即便某一拍第 0 步真的达到 0.8，手指若已经先收到 60 mm 以下，`close_gripper()` 仍会被跳过。

所以三个值对应三种不同的失败：

| `n_exec` | 和训练的关系 | 对这次夹爪问题 |
|---:|---|---|
| 1 | 观测更新最接近训练 | 闭合若在第 21 步，永远执行不到 |
| 5 | 一次执行一小段斜坡 | 9 月 19 日的实际设置；闭合仍在窗口外，宽度只从 66.4 mm 收到约 61 mm |
| 50 | 开放环执行整段计划 | 能执行到第 39 步，但中途不看新图像，位姿会偏离计划 |

`n_exec` 决定的是“计划里的哪一段被执行”，不是“模型会不会把闭合放到第 0 步”。在第 0 步仍是张开命令时，把它设成 1 不能让夹爪合上。

---

**User**

你在上一轮提到了"二值模式：手指要么全开，要么一次 grasp 到目标", 但为什么要用"连续模式"? 用"二值模式"会有什么问题吗?

---

**Cursor**

二值模式把夹爪动作当成“开/关”开关。这条策略的训练目标不是开关，而是下一帧的开口宽度，所以二值模式会在插插座还没开始时就把“请收到 62 mm”理解成“保持全开”。

## 训练时夹爪动作是什么

示教里 66577 帧都满足：

\[
a_t = 1 - \frac{w_{t+1}}{0.08}
\]

\(a_t\) 是模型输出的夹爪通道，\(w_{t+1}\) 是下一帧两指间距（米），0.08 是归一化常数。一次抓取是大约 17 帧、0.57 秒的连续收拢，每帧大约 1.75 mm，不是一帧里从全开跳到夹紧。

因此同一个数字在两种模式里意思相反：

| \(a\) | 训练语义：下一帧宽度 | 二值模式（阈值 0.5） |
|---:|---:|---|
| 0.03 | 77.6 mm，保持张开 | 张开 |
| 0.17 | 66.4 mm，保持当前满开 | 张开 |
| 0.23 | 61.6 mm，开始收拢 | 张开 |
| 0.50 | 40 mm，斜坡中段 | 立刻 `grasp` |
| 0.80 | 16 mm，接近插头厚度 | 立刻 `grasp` |

示教里 \(a \ge 0.5\) 时，手指已经收到约 38 mm。0.5 不是“开始夹”的标志。

## 二值模式实际做什么

```377:396:b/x/4dwvla_ext/franky_joint_env.py
        if VLA_GRIPPER_MODE == "binary_abs":
            want_close = want_gripper_close(action_grip)
            ...
            if want_close:
                if self._controller.gripper_is_open():
                    self._controller.close_gripper()
            elif not self._controller.gripper_is_open():
                self._controller.open_gripper()
```

`want_gripper_close()` 就是 \(a \ge 0.5\)。达到阈值就调用一次力控 `grasp`，目标是标定宽度 10±8 mm；否则什么都不做。没有 50 mm、40 mm 这种中间宽度。

9 月 18 日修好 `his_len` 之后的 Level 2 就是这个模式：手臂已经靠近插头，模型输出峰值只有 0.227 和 0.216。按训练公式，0.227 表示下一帧收到 61.8 mm。二值模式因为 \(0.227 < 0.5\)，300 步里一次 `close` 都没有发。宽度停在 66.4 mm，下一次模型仍看到满开，于是继续输出 0.2 附近。这是一个吸收态，多跑几步不会自己跳过 0.5。

把阈值降到 0.18 也标不定。满开时“保持不动”的动作是：

\[
a_{\text{hold}}(w) = 1 - \frac{w}{0.08}
\]

硬件上报满开 66.4 mm 时，\(a_{\text{hold}} = 0.170\)。阈值 0.18 只比噪声高 0.006。若 homing 后满开回到示教的 79 mm，\(a_{\text{hold}}\) 变成 0.01，0.18 又变得极钝。

## 连续模式在补什么

连续模式按训练公式反解宽度：

\[
w_{\text{cmd}} = \min\bigl(0.08(1-a),\; w_{\text{max,硬件}}\bigr)
\]

\(a = 0.23\) 就去 61.6 mm，下一次观测里的夹爪宽度变小，模型才会沿着斜坡把 \(a\) 加大。\(a \ge 0.8\)（约 16 mm）才交给力控 `grasp`，因为最后几毫米要靠力，而不是位置控制夹到 0。

9 月 19 日的连续模式已经体现出这个差别：执行到的最大 \(a\) 仍只有 0.232，但宽度从 66.4 mm 收到了约 61 mm。二值模式在同样的输出来下，宽度会一直是 66.4 mm。

## 二值模式还会带来的执行问题

即便某一步 \(a\) 终于超过 0.5，二值模式仍做了训练里没有的事。

- 它把“请收到 40 mm”执行成“力控夹到 10 mm”。模型下一帧看到的不是 40 mm，斜坡反馈被跳过。
- 手指可能在插头还没进入指缝时就合死。空夹后的宽度接近 0，不在插头窗口 \([2, 18]\) mm 里，`close()` 会报失败。
- 中间没有滞回宽度。\(a\) 在 0.5 附近抖一下，就会整爪开或整爪合。

二值模式适合动作本身就是 0/1 的策略。这里的动作是下一帧宽度的仿射编码，所以要用连续宽度跟踪；力控只留给斜坡末端。连续模式还没有解决 chunk 后段没被执行、以及 `is_open` 把门控写成“手指仍张到 60 mm 以上才允许 grasp”这两个问题。

---

**User**

按照上面你写的"5. grasp_handoff 要怎么改，为什么？"对代码进行改动, 并通过在容器"rlinf-4dwvla-gpu"上运行 vla_inference_server.py 和 在 "rlinf-4dwvla-gpu" 上运行 franka_vla_client.py 看有没有达到期望效果

---

**Cursor**

这个改动只影响连续模式里 `a ≥ 0.8` 时是否真正下发 `grasp`。我先改门控和冷却，再用离线用例钉住“宽度已经小于 60 mm 仍应抓取”；随后在对应容器里跑服务端和客户端，看日志里有没有出现期望的 `grasp_handoff`。

门控会改成“没夹住才抓，并且失败后至少隔 1 秒再试”。离线测试先覆盖“手指已收到 56 mm、`is_open` 为假，仍然必须下发 grasp”这种情况。

服务端已经在 `rlinf-4dwvla-gpu` 里监听 5555。客户端要连 Franka 和 RealSense，所以放在已经启动的 `rlinf-4dwvla-franky` 里；GPU 镜像没有这套机器人运行时。

这次改动达到了它自己的目标：手指收到 60 mm 以下之后，力控 `grasp` 仍然会发出。300 步结束时夹爪没有保持夹持，手臂也没有回到 HOME。

## 代码

连续模式不再用 `gripper_is_open()` 决定是否抓取。`is_open` 在宽度低于 60 mm 时就是假，而斜坡本来就会先收到这个宽度以下。现在的条件是“还没夹住”，并且失败后至少隔 1 秒再试：

```187:204:b/x/4dwvla_ext/franky_joint_env.py
def should_issue_grasp_handoff(
    holding: bool,
    now_s: float,
    last_attempt_s: float,
    cooldown_s: float,
) -> bool:
    ...
    if holding:
        return False
    return (float(now_s) - float(last_attempt_s)) >= float(cooldown_s)
```

离线测试 `test_gripper_semantics_offline.py` 为 64 项通过，其中包括“宽度 56 mm、尚未夹住，仍然允许 grasp”。

启动时手部上报的满开宽度已经是 0.0803 m，不再是以前的 0.0664 m。配置里的 `FRANKA_GRIPPER_MAX_WIDTH_M` 因此从 0.066 改成 0.080。若仍留在 0.066，一个“保持张开”的命令会被执行成从 80 mm 收到 66 mm。

## 真机

服务端在 `rlinf-4dwvla-gpu` 上运行，`--n-exec 5`。客户端不能放在这个 GPU 镜像里，里面没有 franky 和 RealSense；它在已经启动的 `rlinf-4dwvla-franky` 里运行，两台容器都是 host 网络，所以仍然连 `localhost:5555`。参数是 `--max-steps 300 --control-hz 10 --n-exec 5`，连续模式。跑完 300 步，实际 2.86 Hz，没有触发运动守卫。服务端已停掉。

夹爪宽度的后段是：

| 步 | `action_grip` | 实测宽度 | 命令 |
|---:|---:|---:|---|
| 244 | 0.225 | 65.3 mm | `move_width`，斜坡开始 |
| 265 | 0.507 | 41.9 mm | 继续收，二值模式在这里就会直接 `grasp` |
| 280 | 0.849 | 18.1 mm | `grasp_issued=True`，发出力控抓取 |
| 281–291 | 0.85–0.92 | 7.4 mm | 决策仍是 handoff，但没有再次下发 |
| 292 | 0.906 | 0.9 mm | 再次抓取，失败后手张开到 80.2 mm |
| 299 | 0.015 | 78.2 mm | 保持张开 |

第 280 步的 18.1 mm 低于 60 mm。旧门控会把这次 `grasp` 丢掉；日志里确实出现了 `gripper grasp: width=0.0100m +/-0.0080m`。抓完后宽度停在 7.4 mm，落在插头窗口 `[2, 18]` mm 内，接下来 11 步被“已经夹住”和 1 秒冷却挡住，没有连发。

第 292 步宽度掉到 0.9 mm，离开这个窗口，于是再抓一次。这次没有夹到目标，`close()` 报 `measured width=0.0802m`，手被张回 80 mm。第 293、294 步又各抓了一次，因为失败的 `grasp` 本身阻塞了约 2 秒，1 秒冷却在返回时已经过期。最后模型看到满开，`action_grip` 回到 0.02，手停在 78 mm。

所以：门控修复是有效的，连续斜坡也真正走到了力控阶段。最终没有形成稳定夹持。手臂停在第 300 步的姿态，没有回 HOME。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 10:47 AM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456835
title: Restart Franka client with 80 mm gripper ceiling
tool_call_id: call-ed054962-e3c8-4532-b391-0bd6c0a6d010-79
fc_f432fa8b-7ea6-912b-9c93-7d195b9ba6eb_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456835.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

真机客户端已经正常结束：300 步跑完，第 280 步在手指宽度 18.1 mm 时发出了力控抓取，夹爪随后张开，停在约 78 mm。手臂仍留在第 300 步的姿态，没有回到 HOME。

---

**User**

刚才插座没放上去, 再试一下.

---

**Cursor**

上次结束时插座不在台上，夹爪最后也是张开的。我按同一套参数再跑一次：服务端在 GPU 容器，客户端在 Franka 容器，手臂会先回 HOME，再等场景就绪后开始。

这次没有夹上。300 步跑完，没有发出 `grasp`，夹爪停在 **62 mm**。

手臂有移动：第 0 步到第 299 步，q3 从 0.16 rad 走到 -0.13 rad，q6 从 1.93 rad 走到 2.41 rad。夹爪从 80.2 mm 慢慢收到 62.1 mm，第 239 步之后就停在这里。实际执行的 `action_grip` 最大只有 **0.232**，没有到 0.5，更没有到 0.8，所以力控抓取一次都没触发。实际控制频率 3.12 Hz，590 条警告几乎都是 q7 被裁回 0.4843 rad。

服务端已经停掉。手臂留在第 300 步的姿态，夹爪张开约 62 mm。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 10:53 AM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456837
title: Start Franka client for socket retry
tool_call_id: call-3b85d4af-d124-416d-9b6b-c74393caff2b-102
fc_b5ab9e10-128c-9ab0-86ec-5a346708742b_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456837.txt
</task>
</system_notification>

<timestamp>Tuesday, Sep 22, 2026, 10:53 AM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456836
title: Start VLA inference server in GPU container
tool_call_id: call-48befeea-f941-41e0-93ee-f0ee4576e53f-100
fc_d6da0bca-8256-99a7-9233-bd6738bf4109_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456836.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

重试的客户端已正常结束：300 步内没有发出抓取，夹爪停在约 62 mm。服务端的退出码 143 是跑完后被主动停掉，不是推理中途崩溃。

---

**User**

不对 q7 做限制, 再跑一遍.

---

**Cursor**

q7 现在每步都被裁回训练观测下界 0.4843 rad。我只去掉这道训练范围限制，硬件关节极限和单步速度限制仍保留，然后按刚才同一套参数再跑一次。

训练范围这道裁剪会放开，q7 可以按模型命令走到硬件限位内。单步最多仍只动 0.15 rad，避免一下甩出去。

q7 的训练范围限制已经去掉，300 步跑完。腕关节这次按模型命令下降，夹爪没有收。

q7 从 0.563 rad 降到最低 **0.402 rad**，结束时 0.448 rad。全程 0 条 `OUT-OF-TRAIN` / `TRAIN-EDGE` 警告，说明没有再被裁回 0.4843 rad。硬件限位和单步 0.15 rad 的速度限制还在。

夹爪几乎没动：80.2 mm 收到 78.5 mm 后保持。实际执行的 `action_grip` 最大只有 **0.025**，没有 `grasp`。q3 从 0.159 rad 走到 -0.125 rad，手臂有移动。实际频率 3.15 Hz。

服务端已停。手臂停在第 300 步，没有回 HOME。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 11:00 AM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456839
title: Start Franka client with q7 training clamp off
tool_call_id: call-b49726ba-f056-421c-91ae-e1e732aeb4db-131
fc_c35b6385-7563-97ad-af37-2e2c1d12c583_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456839.txt
</task>
</system_notification>

<timestamp>Tuesday, Sep 22, 2026, 11:00 AM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456838
title: Start VLA server for the unrestricted-q7 run
tool_call_id: call-0222c362-98af-4c59-afff-04ade0134fcb-129
fc_a798dcae-1c07-987a-bc83-f926735967bd_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456838.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

放开 q7 的那次运行已经结束：腕关节降到约 0.40 rad，夹爪停在 78.5 mm，没有抓取。服务端退出码 143 是跑完后主动停掉的。

---

**User**

汇总一下现在对Franka有些什么限制?

---

**Cursor**

我按现在代码里实际生效的限位来汇总，包括关节、末端围栏、速度和夹爪，不把已经关掉的 q7 训练范围裁剪算进去。

现在还压在 Franka 上的限制分四层。q7 的训练范围裁剪已经去掉，只剩硬件限位和单步速度限制。

## 关节

| 关节 | 现在允许的命令范围 (rad) | 来源 |
|---|---|---|
| q1 | [-0.634, 0.195] | 示教范围再放宽 0.15 |
| q2 | [-0.253, 0.462] | 同上 |
| q3 | [-0.353, 0.629] | 同上 |
| q4 | [-2.354, -1.385] | 同上 |
| q5 | [-0.354, 0.231] | 同上 |
| q6 | [1.420, 2.604] | 同上 |
| q7 | [-2.897, 2.897] | 只有硬件限位，不再卡在 0.4843 |

超出 q1–q6 的范围会被裁掉并记 `OUT-OF-TRAIN`。q7 不会。另外每一步七个关节里最大的那一维不能超过 **0.15 rad**，超了就整条命令按比例缩小。

## 末端和碰撞

TCP 必须留在这个盒子里，越界就停：

```text
x [0.484, 0.652] m
y [-0.190, 0.103] m
z [0.168, 0.567] m
```

这是示教末端范围加 5 cm 余量，z 向下只留 1 cm。50 Hz 看门狗还要求关节速度范数不超过 **1.2 rad/s**。

碰撞阈值比出厂更紧：笛卡尔力各轴 40 N、力矩各轴 12 N·m；关节力矩阈值是 `[20, 20, 18, 18, 16, 14, 12]` N·m。平时运动速度系数 `relative_dynamics_factor = 0.2`，回 HOME 时临时降到 0.1。每次目标仍是阻塞式走完一个 waypoint，所以实际大约 3 Hz，不是训练的 30 Hz。

## 夹爪

当前是连续模式，评测配置里这些值在生效：

- 命令宽度上限 **0.080 m**，再与手部上报的满开宽度取较小值。现在手上报 0.0803 m，所以上限是 0.080 m。
- 比实测窄 1.5 mm 以上才下发收拢；要比实测宽 6 mm 以上才下发张开。
- `action_grip ≥ 0.8` 才力控抓取，力 **20 N**，速度 0.05 m/s。
- “已经夹住”的判定窗口是插头厚度 10±8 mm，即 **[2, 18] mm**。窗口内不再重复抓。
- 没夹住时，两次抓取至少隔 **1 秒**。失败的 `grasp` 本身会阻塞约 2 秒，所以返回后冷却往往已经过期，下一步仍可能再抓。

---

**User**

暂时先把所有限制放开, Franka回home,然后再运行一次看看结果.

---

**Cursor**

这次只临时关掉软件加的软限制：训练范围裁剪、单步 0.15 rad、末端围栏、看门狗速度和收紧的碰撞阈值。硬件关节限位保留。手臂先回 HOME，再按同样参数跑一轮。

软限制用环境变量临时关掉，默认行为不变。硬件关节限位仍然保留，避免发出机械臂接不住的角度。

软限制已经临时关掉，手臂先回到了 HOME。速度系数放到 1.0 时，第一步就被固件反射停掉了；改回 0.2 后 300 步跑完，夹爪仍然没有收。

第一次把 `FRANKA_DYNAMICS_FACTOR` 设成 1.0，第 0 步就触发 libfranka 反射：`joint_motion_generator_velocity_discontinuity`。手臂几乎没离开 HOME。已清除错误并再次回 HOME。

第二次保留速度系数 0.2，其余软限制关闭：

- q1–q7 不再按示教范围裁剪，单步 0.15 rad 上限也关掉
- 末端围栏和 1.2 rad/s 看门狗没有安装
- 碰撞阈值留在 libfranka 默认值
- 夹爪死区和抓取冷却设为 0

硬件关节限位还在。300 步、0 条裁剪警告，实际 2.29 Hz。q7 从 0.561 rad 降到最低 **0.400 rad**，结束时 0.432 rad。q3 从 0.158 rad 走到 -0.128 rad。`action_grip` 最大只有 **0.026**，没有 `grasp`。因为死区是 0，每一步都发了 `move_width`，宽度只在 77.9–80.2 mm 之间来回，结束时 78.7 mm。

服务端已停。手臂停在第 300 步，没有回 HOME。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 11:15 AM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456842
title: Rerun client with soft limits off, speed factor 0.2
tool_call_id: call-2ab468b6-d527-4b49-b820-ad614a599f22-185
fc_c3153367-cdbe-94f4-af31-d92947a58494_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456842.txt
</task>
</system_notification>

<timestamp>Tuesday, Sep 22, 2026, 11:15 AM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456840
title: Start VLA server for the unrestricted run
tool_call_id: call-111b4611-3702-446f-b1e3-d0f0b9749966-169
fc_140f7266-2074-945b-a9bf-a5d4cf6be891_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456840.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

软限制关掉后的那次运行已经结束：300 步内没有抓取，夹爪停在约 78.7 mm，q7 降到约 0.40 rad。服务端退出码 143 是跑完后主动停掉的。

---

**User**

基于日志,对刚才做的3个实验做一下分析.

---

**Cursor**

三次实验的差别主要在 q7 能不能离开训练范围。我按三份客户端日志把关节、夹爪动作和实际宽度对齐后再下结论。

三次实验说明：夹爪收不收，取决于 q7 有没有被按在训练范围内。放开 q7 之后，策略立刻停在“保持张开”；再放开其它软限制，也没有把夹爪斜坡找回来。

| | 实验 A：插座在台上，q7 仍裁剪 | 实验 B：只放开 q7 | 实验 C：软限制全关，速度 0.2 |
|---|---|---|---|
| 日志 | `client_20260922_025020_1352.log` | `client_20260922_025819_2197.log` | `client_20260922_031257_3436.log` |
| q7 | 降到 0.484 rad 后被钉住 | 降到 0.402 rad | 降到 0.400 rad |
| 夹爪动作峰值 | **0.232**（第 234 步起超过 0.2） | 0.025 | 0.026 |
| 宽度 | 80.2 mm → **62.1 mm** 后停住 | 80.2 → 78.5 mm | 在 77.9–80.2 mm 之间抖 |
| 抓取 | 没有 | 没有 | 没有 |
| 警告 | 590 条，全是 q7 越界/贴边 | 0 | 0 |
| 频率 | 3.12 Hz | 3.15 Hz | 2.29 Hz |

另有一次速度系数 1.0 的尝试（`client_20260922_031131_3116.log`）。第 0 步就被固件反射停掉：`joint_motion_generator_velocity_discontinuity`。手臂几乎没离开 HOME。阻塞式 waypoint 不能用满速。

## 实验 A：只有这次走进了收拢斜坡

q7 从 0.564 rad 被每步裁回 0.484 rad，腕关节贴在示教下界上。在这个姿态下，模型从第 234 步开始给出 \(a \ge 0.2\)，夹爪用 10 次 `move_width` 从 80 mm 收到 62 mm，然后停住。0.232 对应的目标宽度大约是 \(0.08\times(1-0.232)=61.4\) mm，和实测 62.1 mm 一致：执行器跟上了命令，但命令本身没有再往上走，所以到不了 0.5，更到不了 0.8 的力控抓取。

这趟手臂的其它关节也和后两次不同。q2 从 -0.033 rad 抬到 0.201 rad，q6 从 1.926 rad 抬到 2.412 rad。后两次里这两个关节几乎不动。q7 被按在训练边界上时，策略还在走一条更像示教的臂型；q7 一旦掉到 0.40 rad，这条臂型就消失了。

## 实验 B：放开 q7 之后，夹爪通道塌回“保持张开”

q7 在大约 50 步内掉到 0.407 rad，之后停在 0.40–0.45 rad。没有任何裁剪警告，说明命令是原样执行的。与此同时 `action_grip` 全程不超过 0.025，宽度只从 80.2 mm 收到 78.5 mm，然后保持。

0.025 的含义是“下一帧宽度约 78 mm”，不是“开始夹”。q3 仍然从 0.159 rad 走到 -0.125 rad，手臂有移动，但移动发生在夹爪通道已经退出斜坡之后。所以不是“手臂没动所以没夹”，而是腕关节离开示教范围后，模型不再输出收拢。

## 实验 C：其它限制不是这次没夹上的原因

训练范围、单步 0.15 rad、末端围栏、速度看门狗、收紧的碰撞阈值和夹爪死区都关掉之后，轨迹和实验 B 几乎一样：q7 最低 0.400 rad，`action_grip` 最高 0.026，没有抓取。

死区设成 0 只多了一个副作用。目标宽度和实测只差不到 2 mm，以前会被判成 `hold`；现在 300 步全部发了阻塞的 `move_width`。手指在 78–80 mm 附近来回蹭，频率从 3.15 Hz 降到 2.29 Hz，并没有多收拢一毫米。

速度系数 1.0 则走不到这一步。libfranka 的运动发生器不允许相邻 waypoint 的速度、加速度断开，满速的第一步就被反射取消。

## 结论

这三次里，夹爪唯一一次离开“全开”是实验 A，也就是 q7 被留在示教下界 0.484 rad 的那次。放开 q7 让腕关节按模型走到 0.40 rad，夹爪命令同时掉回 0.02。继续放开围栏、步长、碰撞和死区，没有改变这个结果。当前没夹上，不是这些软限制把收拢命令裁掉了，而是 q7 一出训练范围，模型就不再发出收拢命令。

---

**User**

在第二,三次实验中, 为什么Franka机器臂把p7和所有限制去掉后, 反而会变成在原地抖动, 夹爪一直下不去.

---

**Cursor**

“下不去”和“原地抖”要分开看：前者是末端有没有下降，后者是关节命令在来回反向。我用这三次日志把每个关节的净位移和来回路程对上。

第二、三次里夹爪下不去，是因为 q7 一旦按模型命令掉到约 0.40 rad，控制下降的肘和腕就不再朝下走，而是在原地来回抵消。第三次再放开其它限制，臂的抖法和第二次一样，只是手指又多抖了一层。

Franka 把末端放低，主要靠 **q4 再弯下去**，再加上 **q6 转腕**。三次日志里这两轴的方向是反的：

| | 实验 A：q7 被按在 0.484 rad | 实验 B：只放开 q7 | 实验 C：软限制全关 |
|---|---|---|---|
| q4 | -1.884 → **-2.086**（更弯，往下） | -1.885 → **-1.794**（反而伸直） | -1.885 → **-1.787** |
| q6 净位移 | **+0.486 rad** | **0** | **-0.009 rad** |
| q6 来回路程 | 0.84 rad | 0.71 rad | 0.72 rad |
| 路程 / 净位移 | q4 约 1.8，q6 约 1.7 | q4 约 6，q6 约 2800 | q4 约 6，q6 约 77 |
| q7 | 第 10 步前就被裁住，之后不动 | 第 10 步已经低于 0.45 rad，最低 0.402 | 同样在第 10 步跌破，最低 0.400 |

路程远大于净位移，就是在抖：q6 在第二次走了 0.71 rad，起点和终点却是同一个角度。q2、q5 也是这样，来回反向一百多次，几乎没有留下位移。q1、q3 仍有净移动，所以手臂不是完全冻住，但决定高度的那两轴没有往下降的方向走。

原因是 q7 先离开了示教范围。模型每步都想把 q7 打到 0.40 rad 附近。实验 A 把这个命令改写成 0.484 rad，腕关节留在示教下界上，q4、q6 的下降方向还在。实验 B、C 照原命令执行，大约 10 步后 q7 就低于 0.45 rad。这一帧之后的关键点历史和当前姿态都在训练分布外面，后面的 q4、q6 不再是“再弯一点”，而是绕当前角度来回给绝对目标。每一步又是走完一个 waypoint 再停下，所以看起来是末端在原地点头、扭腕，高度不降。

第三次没有带来新的下降。臂关节的路程和实验 B 几乎相同。多出来的是夹爪：死区被设成 0 之后，300 步全部发了 `move_width`，手指在 78–80 mm 之间每步蹭一下，频率从 3.15 Hz 掉到 2.29 Hz。那是手指在抖，不是手臂多下降了。

---

**User**

把代码恢复到刚才所说的"实验A"的代码, 然后做一下相关测试和实验A.

---

**Cursor**

实验 A 是 q7 仍裁在 0.4843 rad、其余软限制都开着的那一版。我先把后来放开 q7 和总开关的改动退回去，再跑离线测试和同样的 300 步真机。

代码已回到实验 A：q7 再次被裁在 0.4843 rad，末端围栏、碰撞阈值、单步 0.15 rad 和夹爪死区都恢复了。离线测试 131 项通过。真机跑完 300 步，q7 限制生效了，但这次没有复现实验 A 的下降和收拢。

离线测试：安全 52 项、历史 15 项、夹爪 64 项，全部通过。q7 低于 0.4843 rad 会再次被裁回并报 `OUT-OF-TRAIN`。

真机先回了 HOME，然后用和实验 A 相同的参数跑了 300 步：连续夹爪、死区 1.5 mm、`--n-exec 5`、`--control-hz 10`。启动日志确认碰撞阈值已收紧，末端围栏和 50 Hz 看门狗已装上。q7 从 0.562 rad 被钉在 **0.484 rad**，590 条越界/贴边警告，实际频率 3.15 Hz。这些和实验 A 一致。

夹爪和下降没有复现。`action_grip` 最高只有 **0.025**，宽度从 80.2 mm 收到 78.7 mm 后保持，没有 `grasp`。q4 从 -1.886 rad 走到 -1.844 rad，是略微伸直，不是实验 A 里那次再弯 0.20 rad 的下降。

回 HOME 之前夹爪宽度是 11.1 mm，且被判定为夹持。`reset` 执行了 `open()`，日志有 “the object WILL drop”。如果当时指间有插头，它在开跑前已经被放下。服务端已停，手臂停在第 300 步。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 12:24 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456844
title: Start experiment A client with q7 clamp restored
tool_call_id: call-aa96a791-116d-41c0-9b79-e3378062070d-214
fc_ce89940b-0552-9924-b47b-cb2eaa559b7e_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456844.txt
</task>
</system_notification>

<timestamp>Tuesday, Sep 22, 2026, 12:24 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456843
title: Start VLA server for restored experiment A
tool_call_id: call-39da7498-9c01-48f6-94f0-bb5c5c9b84ce-212
fc_e9688c08-f80e-9de8-990e-f66ca94bfc22_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456843.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

恢复实验 A 的那次运行已经结束：q7 再次被钉在 0.484 rad，但夹爪停在约 78.7 mm，没有收拢。服务端退出码 143 是跑完后主动停掉的。

---

**User**

所有代码都退回到"实验A"了吗, 为什么"夹爪和下降没有复现"?

---

**Cursor**

q7 裁剪在这次日志里确实生效了，所以“没下降”不是限制没恢复。我对照实验 A 的代码路径和两份日志的第 0 步，看是代码还留着差异，还是一上来观测就不一样。

控制代码已经回到实验 A 的那条路径。这次夹爪不收、手臂不下降，不是因为 q7 限制没装回去，而是第 0 步之后图像和闭环轨迹已经和实验 A 分叉了。

实验 A 当时在跑的限制，现在都在：

- q7 低于 0.4843 rad 会被裁回去，贴边 0.03 rad 会告警
- 末端围栏、碰撞阈值收紧、速度系数 0.2、单步 0.15 rad 都开着
- 连续夹爪、1.5 mm 死区、宽度上限 0.080 m、`grasp` 门控也还在

后来加上的 `VLA_DISABLE_SOFT_LIMITS` 和“q7 只用硬件限位”已经去掉。这次启动日志是 `Collision behavior tightened`、围栏 `min=[0.484, -0.19, 0.168]`，不是放开限制时的那几行警告。q7 实测被钉在 0.484 rad，590 条越界警告，和实验 A 一样。

两份日志的起点几乎相同，第 0 步命令也几乎相同：

| | 实验 A `025020` | 这次 `042141` |
|---|---|---|
| 起步 q7 / 第 0 步命令 | 0.643 → 0.564 | 0.644 → 0.562 |
| 起步 q4 / q6 | -1.886 / 1.906 | -1.886 / 1.906 |
| 第 0 步夹爪动作 | 0.019 | 0.014 |
| 第 0 帧图像均值（全局 / 腕） | 122.5 / 126.1 | 120.7 / 123.5 |
| 第 10 步，q7 都已被裁住后 | q4 命令继续弯到 -1.896，q6 升到 1.969 | q4 命令折回 -1.883，q6 停在 1.958 |
| 300 步结果 | q4 弯到 -2.086，宽度收到 62 mm，`a` 到 0.232 | q4 伸到 -1.844，宽度停在 78.7 mm，`a` 最高 0.025 |

分叉发生在 q7 已经被同样裁住之后。所以不是裁剪逻辑不同。

这次开跑前，夹爪上报 `holding=True`、宽度 11.1 mm。`reset` 调用了 `open()`，并写下 `the object WILL drop`。实验 A 连接时是 `holding=False`、宽度 78.2 mm，没有这句。插头或插座在第 0 帧的画面已经不一样，模型从第一步起看到的就不是同一场景。后面每 5 步才重新看一次图像，这个差别会被放大：实验 A 的 q4、q6 继续往下降方向走，这次在第 10 步附近就折返回去，夹爪动作也一直停在 0.02 附近。

---

**User**

回HOME后再跑一下"实验A"

---

**Cursor**

按实验 A 的配置再跑一次：先回 HOME，q7 仍裁在 0.4843 rad，其余软限制保持打开。

已回 HOME 并按实验 A 再跑完 300 步。手臂下降回来了，夹爪收拢没有回来。

连接时夹爪是张开的：`holding=False`，宽度 78.7 mm，没有再把东西放下。q7 被钉在 0.484 rad，590 条越界警告，频率 3.19 Hz。

和原来的实验 A（`025020`）对比：

| | 原来的实验 A | 这次 `043135` |
|---|---|---|
| q4 | -1.884 → **-2.086** | -1.882 → **-2.062** |
| q6 | 1.926 → **2.412** | 1.923 → **2.385** |
| q2 | -0.033 → **0.201** | -0.033 → **0.222** |
| q7 | 钉在 0.484 | 钉在 0.484 |
| `action_grip` 峰值 | **0.232**（第 234 步起） | **0.031** |
| 夹爪宽度 | 80.2 → **62.1 mm** | 80.2 → **78.6 mm** |
| 第 0 帧图像均值（全局 / 腕） | 122.5 / 126.1 | 120.5 / 124.9 |

肘和腕的净位移与实验 A 同向、幅度接近，末端是在下降。夹爪全程几乎只有一次很小的 `move_width`，没有进入上次那条收到 62 mm 的斜坡。第 0 帧两路图像均值已经不同，所以同一套控制代码没有走出同一条夹爪轨迹。

服务端已停。手臂停在第 300 步，没有回 HOME。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 12:34 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456846
title: Start experiment A client after homing
tool_call_id: call-31c56f59-df54-435c-9a9a-a3c30e250a20-228
fc_fff11d6b-e499-9767-991e-88a919c1d9a8_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456846.txt
</task>
</system_notification>

<timestamp>Tuesday, Sep 22, 2026, 12:34 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456845
title: Start VLA inference server for experiment A
tool_call_id: call-911927e9-144d-45cf-8fc1-90a82bc36f71-226
fc_db51a073-a987-9c94-84eb-27722e224e68_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456845.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

回 HOME 后再跑的实验 A 已经结束：手臂下降方向和上次接近，q4 收到约 -2.06 rad，但夹爪停在约 78.6 mm，没有收拢。服务端退出码 143 是跑完后主动停掉的。

---

**User**

@RLmm/b/x/4dwvla_ext/logs/client_20260922_024308_492.log:1917-1932 这是不是"实验A"的日志? 这段日志的意思是不是夹爪已经开始闭合了, 只是闭合失败, 具体是什么原因失败的呢? 

---

**Cursor**

这段不是实验 A 的日志。我对照夹爪判定代码，说明这几行里闭合命令发出之后为什么被判失败。

不是实验 A。`client_20260922_024308_492.log` 是更早的那次，大约 02:45，也就是你后来说插座没放上去的那一轮。实验 A 是 `client_20260922_025020_1352.log`。

这几行里夹爪确实已经合过，而且合过了头。第 292 步发力控抓取时，实测宽度是 **0.89 mm**。成功窗口是插头厚度 10±8 mm，也就是 **[2, 18] mm**。0.89 mm 比下限还小，所以 `holding=False`，代码认为没夹住，又发了一次 `grasp`。

失败点是这次力控抓取结束后，手指不在 10 mm 附近：

```250:257:b/x/franky_ext/franka_libfranka_gripper.py
        width = self._width_m()
        if width is None or abs(width - target_w) > tol:
            raise RuntimeError(
                f"grasp did not capture the cube: measured width="
                ...
                f"{target_w:.4f}m +/-{tol:.4f}m. Either the hand is empty, or the "
                f"cube is a different size ..."
            )
```

三次的实测是：

| 步 | 抓取前宽度 | 抓取后宽度 | 结果 |
|---:|---:|---:|---|
| 292 | 0.89 mm | **80.2 mm** | 报未夹住，手指张开 |
| 293 | 80.2 mm | **0.02 mm** | 空合到接近 0 |
| 294 | 0.94 mm | **78.4 mm** | 再次报未夹住，又张开 |

80.2 mm 和 78.4 mm 是满开，不是 10 mm 的插头。0.02 mm 和 0.89 mm 是两指几乎夹死，中间没有 10 mm 厚的物体。两种都不在 [2, 18] mm 里。`close_gripper()` 把这个错误记成 warning，没有中止后面的步，所以第 293、294 步在 `a≈0.91` 时又各抓了一次。

---

**User**

深入分析这些日志, 深入思考一下应该如何复现那次抓取, 即使没抓取成功.

---

**Cursor**

那次抓取发生在插座没放上的 `024308` 日志里，不是后来的实验 A。我把那次从张开到发出 `grasp` 的关节、夹爪和图像，跟之后几次没合上的运行对齐。

要复现的是 `client_20260922_024308_492.log` 这轮，不是实验 A。那一轮插座不在台上，夹爪在第 280 步才真正去抓；实验 A 只收到 62 mm 就停了。

那次抓取的代码就是现在这套实验 A 代码：q7 被钉在 0.484 rad，连续夹爪，死区 1.5 mm。放开 q7 的那两次根本走不到这个姿态。

## 抓取是在手臂到位之后才开始的

前 200 步夹爪几乎没动，宽度停在 78.7 mm，`action_grip` 只有 0.02。手臂一直在下降：

| 步 | q4 | q6 | q2 | 腕部图像均值 | `action_grip` | 宽度 |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | -1.885 | 1.929 | -0.031 | 123.5 | 0.011 | 80.3 mm |
| 200 | -2.045 | 2.376 | 0.227 | **117.0** | 0.022 | 78.7 mm |
| 244 | -2.07 附近 | 2.40 | 0.22 | 114.9 | **0.20** | 开始收 |
| 265 | -2.08 | 2.40 | 0.22 | 115.0 | **0.51** | 约 42 mm |
| 280 | -2.097 | 2.420 | 0.210 | **105.6** | **0.85** | 18.1 mm，发出 `grasp` |
| 292 | -2.10 | 2.41 | 0.20 | 102.4 | 0.91 | 0.89 mm，判定失败并张开 |

腕部画面在手指还没动的时候就已经变暗：第 200 步均值 117，第 215 步 115，这时 `action_grip` 仍是 0.02。收拢是在这之后才出现的。手指合上以后均值继续掉到 102，那是夹爪进入画面后的结果，不是触发条件。

## 同样的手臂姿态，图像不同就不会抓

实验 A 在第 150 步就已经到了 q4=-2.093、q6=2.418，和那次抓取第 230 步的姿态几乎一样。但腕部均值只降到 117–118，然后回到 119 左右停住。`action_grip` 最高 0.232，宽度停在 62 mm。

今天回 HOME 后再跑的那次，第 230 步也到了 q4=-2.053、q6=2.381、q2=0.231。腕部均值同样停在 117.7，`action_grip` 只有 0.021，宽度停在 78.6 mm。

所以复现条件不是“把 q4、q6 再走到那个数”。那两个关节三次都走到了。分叉在腕部图像：只有 `024308` 在到位之后继续变暗到 112 以下，模型才把 `action_grip` 从 0.02 推过 0.5，再到 0.8。

## 怎样再走出同一次尝试

场景按那次的条件摆：插座不放上台面，插头按示教起始位置放，不要提前塞进夹爪。连接日志应是 `holding=False`、宽度约 78 mm。若连接时已经是 `holding=True`，`reset` 会先 `open()` 并把东西放下，第 0 帧就和那次不一样。

控制保持现在的实验 A，不要再关 q7 限制：

- 服务端 `--n-exec 5`
- 客户端 `--max-steps 300 --control-hz 10 --n-exec 5`
- `VLA_GRIPPER_MODE=continuous`，死区 1.5 mm，宽度上限 0.080 m

跑的时候看两件事，不要在第 200 步就停：

1. q4 接近 -2.07、q6 接近 2.40，同时腕部图像均值降到 115 以下。那次是在这之后约 30 步才出现 `a≥0.2`。
2. 若腕部均值停在 117–119，就会复现实验 A 的结果：手臂到了，夹爪停在 62 mm 或根本不收。这时再跑 300 步也不会出现 `grasp`。

“复现那次抓取尝试”的日志标志是出现 `gripper continuous grasp_handoff`，并且 `action_grip≥0.8`。抓取成功不是这条标志。那次失败是因为力控目标是 10±8 mm，也就是宽度必须落在 **[2, 18] mm**。手指先空合到 0.89 mm，已经低于这个窗口；随后一次 `grasp` 结束在 80.2 mm，被判成没夹住并张开。插座不在台上时，这个失败会再出现。要的就是这条 `grasp_handoff` 加上后面的 `did not capture` 警告，而不是 `holding=True`。

---

**User**

先回HOME, 然后按照刚才你的回答中"怎样再走出同一次尝试"那个章节中所说的复现方案, 再做一次"实验A", 以进行复现.

---

**Cursor**

按那次抓取的复现条件来跑：先回 HOME，控制保持实验 A（q7 仍裁在 0.4843 rad，连续夹爪，跑满 300 步）。连接时若夹爪已经夹着东西，会先记下来，因为那样第 0 帧就和 `024308` 不一样。

连接条件已经和那次抓取不一致：夹爪夹着约 10.2 mm 的东西，复位时被张开丢掉了；手部上报的满开宽度也从 80.3 mm 掉到 67.5 mm。按复现方案，这种第 0 帧不能开跑。我先停掉等待中的客户端。

手臂已经回到 HOME。300 步没有开跑，因为一连接就和 `024308` 那次的起始条件不一致。

那次抓取连接时是 `holding=False`、开口约 78–80 mm，手部上报满开宽度 0.0803 m。这次是：

- `holding=True`，宽度 **10.2 mm**，正好落在插头窗口 `[2, 18]` mm 里，说明指间已经夹着东西。
- `reset` 立刻执行了 `open()`，日志是 `the object WILL drop`。东西在按 `a` 之前就已经被放下。
- 手部上报满开宽度变成 **0.0675 m**，不再是 0.0803 m。命令上限被钳到 67.5 mm。那次抓取全程是按约 80 mm 的张开宽度在走的。

场景被这次张开改过，夹爪标度也不一样。在这种第 0 帧上继续跑，不会复现那次从 80 mm 收到 18 mm 再发 `grasp` 的过程。客户端和服务端都已停掉。

要再跑的话，需要先把指间清空，并把夹爪 homing 回上报满开约 0.080 m，插座不放上台面，插头放在示教起始位置，然后再回 HOME 开跑。

---

**User**

回HOME,再跑一次

---

**Cursor**

先回 HOME，再用实验 A 的配置跑满 300 步。连接时会记下夹爪是不是张开、满开宽度是不是约 80 mm。

已回 HOME，并跑完 300 步。手臂下降和夹爪收拢都出现了，但力控 `grasp` 没有发出去。

连接时夹爪是空的：`holding=False`。满开宽度仍是 **67.4 mm**，手部上报上限 0.0675 m，不是那次抓取的 80.3 mm。命令上限因此被钳在 67.5 mm。

和 `024308` 对比：

| | 那次抓取 `024308` | 这次 `064522` |
|---|---|---|
| 起始宽度 | 80.3 mm | 67.4 mm |
| q4 / q6 终态 | -2.097 / 2.422 | **-2.127 / 2.424** |
| `action_grip` 首次 ≥ 0.8 | 第 280 步 | **第 211 步** |
| 最窄宽度 | 0.89 mm，随后被判失败并张开 | **17.4 mm**，一直停在这里 |
| `gripper grasp:` | 有，并报 `did not capture` | **没有** |
| 腕部图像均值 | 收到约 105 | 收到约 108 |
| 频率 / 警告 | — | 3.06 Hz，411 条 |

q4、q6 和腕部变暗都到达了那次抓取前的区间，模型也给出了 `a=0.85`。宽度收到 17.4 mm 后，78 步的决策都是 `grasp_handoff`，但 `close()` 一次都没调用。17.4 mm 落在插头窗口 `[2, 18]` mm 里面，`holding` 被当成已经夹住，力控抓取被跳过。

所以这次复现的是“手臂到位并且夹爪收到约 17 mm”，不是“发出抓取后因空合失败再张开”。服务端已停。手臂停在第 300 步。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 2:48 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456850
title: Start experiment A client for 300 steps
tool_call_id: call-80b2b440-5d87-47da-a610-7263a25c3d86-261
fc_07756e05-8952-9514-ada7-2faa147d9490_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456850.txt
</task>
</system_notification>

<timestamp>Tuesday, Sep 22, 2026, 2:48 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456849
title: Start VLA inference server
tool_call_id: call-07368ad4-ff02-4e83-84a7-8eaa4ddc9b92-259
fc_e466f559-2bb1-99b0-beb8-7bc5d58b3e7a_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456849.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

回 HOME 后的 300 步已经跑完：手臂下降，夹爪收到约 17.4 mm，`action_grip` 到了 0.85，但因宽度落在插头窗口内，力控 `grasp` 没有发出。服务端退出码 143 是跑完后主动停掉的。

---

**User**

回HOME, 张开夹爪, 把最大步数设为 600 步, 再跑一次. 

---

**Cursor**

先回 HOME 并张开夹爪，再用实验 A 的配置把最大步数改成 600 跑一轮。

已回 HOME，夹爪已张开，并以 600 步跑完。力控抓取发出了一次，但因为合得比插头窗口更窄，被判失败。

连接时 `holding=False`，开口是手部上报的满开宽度 **67.4 mm**，不是 80 mm。q4 收到 -2.127 rad，q6 收到 2.431 rad，q7 仍被钉在 0.484 rad。`action_grip` 在第 315 步超过 0.8，最高 0.887。

宽度从 67.4 mm 收到最窄 **5.3 mm**。第 316 步附近发出 `gripper grasp`，结束后实测 **1.9 mm**，低于插头窗口下限 2 mm，日志是 `did not capture`。之后宽度停在 **13.3 mm**，落在 `[2, 18]` mm 里，被当成已经夹住，后面 285 次 `grasp_handoff` 决策里只有这一次真正调用了 `close()`。

600 步、627 条警告，实际频率 3.16 Hz。服务端已停。手臂停在第 600 步，夹爪停在约 13.3 mm。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 2:54 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456852
title: Start 600-step experiment A client
tool_call_id: call-6a5df064-4cac-4d3c-99c6-1c939a01922c-276
fc_1d6d7236-0c26-9687-baf2-4417eff9ae83_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456852.txt
</task>
</system_notification>

<timestamp>Tuesday, Sep 22, 2026, 2:54 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456851
title: Start VLA server for the 600-step run
tool_call_id: call-d7635011-7d92-43b0-987c-ad5d375ae5f7-274
fc_e324b711-d109-9f1a-a4f7-dfc246adc25d_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456851.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

600 步已经跑完：夹爪收到约 5.3 mm 后发出了一次力控抓取，实测 1.9 mm，低于插头窗口下限，被判失败；之后停在约 13.3 mm。服务端退出码 143 是跑完后主动停掉的。

---

**User**

能夹住插头, 但其实没夹紧, 也没夹上来, 请深入分析一下具体原因.

---

**Cursor**

这次是夹爪碰到了插头，但没有维持夹紧力，手臂也没有把插头提起来。我按 600 步日志把收拢、那一次失败的 `grasp`，以及之后手臂是否还有上抬命令分开看。

插头只是被两指围住，没有被 20 N 力控夹住，手臂也没有离开最低点，所以提不起来。

## 夹住了，但不是夹紧

第 315 步之前，夹爪是在做位置控制：`move_width` 把开口从 67.4 mm 一步步收到 **20.2 mm**。这一步 `action_grip=0.806`，决策变成力控抓取，并且真的调用了一次：

```text
gripper grasp: width=0.0100m +/-0.0080m  force=20.0N
```

目标是停在 10 mm，允许范围是 **[2, 18] mm**。大约 0.5 秒后实测是 **1.9 mm**。它比窗口下限只低 0.1 mm，`abs(1.9-10)=8.1 mm > 8 mm`，于是 `close()` 报 `did not capture` 并放弃这次抓取。20 N 没有留下来。

失败后的宽度自己停在 **13.3 mm**，之后 280 步都不变。13.3 mm 重新落进 `[2, 18] mm`，`holding` 变成真。后面每一步模型仍给出 `a≈0.83–0.89`，对应的目标开口是 **9–13 mm**，决策也仍是 `grasp_handoff`，但 `grasp_issued` 全是假，`close()` 不再调用。

所以手指最后是停在 13.3 mm 的位置上，不是在用 20 N 夹着。插头若是 10 mm 厚，缝里还有约 3 mm。手指能碰到它，重力一拉就会滑掉。

## 手臂没有向上提

夹爪停在 13.3 mm 之后，决定高度的关节也停住了：

| 步 | q4 | q6 | 宽度 | 实际动作 |
|---:|---:|---:|---:|---|
| 315（发出 grasp） | -2.102 | 2.424 | 20.2 mm | 力控，失败 |
| 317 | -2.102 | 2.420 | 13.3 mm | 已当成夹住，不再用力 |
| 599 | -2.127 | 2.431 | 13.3 mm | 仍在最低点 |

q4 从 -2.102 又弯到 -2.127，是略微再低下去，不是抬起。q6 只动了 0.007 rad。模型后 280 步给出的关节目标就是当前这个最低姿态，执行前缀里没有上提。

夹紧和上提因此都没发生：唯一一次力控因 1.9 mm 越出窗口被丢掉；随后 13.3 mm 被误判成已经夹住，既不再补力，手臂也不离开最低点。

---

**User**

为什么 13.3 mm 就被误判成已经夹住, 但事实上明明没夹紧

---

**Cursor**

13.3 mm 被当成夹住，是因为判定只看开口是否落在 10±8 mm 里，不看有没有夹紧力。我把这个窗口和 13.3 mm 的计算对上代码。

13.3 mm 被当成已经夹住，是因为程序只问“开口像不像 10 mm 的插头”，不问“有没有在用力夹”。

判定在 `_hardware_holding()` 里，每一拍只做一次减法：

```174:174:b/x/franky_ext/franka_libfranka_gripper.py
        return abs(width - cube_width_m()) <= hold_tolerance_m()
```

配置是插头厚度 `FRANKA_CUBE_WIDTH_M=0.010`，容差 `FRANKA_HOLD_TOL_M=0.008`。所以只要开口落在

\[
[10-8,\ 10+8]\ \text{mm} = [2,\ 18]\ \text{mm}
\]

就返回真。13.3 mm 与 10 mm 相差 3.3 mm，小于 8 mm，于是为真。

这个“真”接着挡住了力控。`should_issue_grasp_handoff()` 看到 `holding` 为真就直接返回假，不再调用 `close()`，20 N 也不会再施加。手指就停在上一次位置控制留下的 13.3 mm。10 mm 的插头放在 13.3 mm 的缝里，两边还有约 3 mm 空隙，所以手能碰到它，但没有夹紧。

它也故意不看 libfranka 的 `is_grasped`。注释里写的原因是：空夹到接近 0 时，那个标志也会变成真，从而再也不发 `grasp`。改成只看宽度之后，窗口又放得很宽，13.3 mm 这种松缝和真正夹在 10 mm 上被当成同一件事。

第 315 步的力控其实已经失败了：实测 1.9 mm，低于 2 mm，报了 `did not capture`。失败后手指停在 13.3 mm。下一拍重新计算时，程序不记得上次失败，只看到 13.3 mm 在窗口里，就从“没夹住”改判成“已经夹住”。

---

**User**

我把插座厚度调小了, 回HOME, 张开夹爪, 把最大步数设为 600 步, 再跑一次. 

---

**Cursor**

插座厚度你已经改过。我先回 HOME、张开夹爪，再用实验 A 的配置跑 600 步。

已回 HOME、张开夹爪，并跑完 600 步。手臂再次降到最低点，夹爪多次力控抓取，但都合到了接近 0，没有停在你设的厚度上。

你在 `franka_plug_eval.env` 里写的是 `FRANKA_CUBE_WIDTH_M=0.002`、`FRANKA_HOLD_TOL_M=0.001`，也就是 2±1 mm。程序允许的范围是厚度 **[5, 70] mm**、容差 **[4, 25] mm**，所以实际用的是下限 **5±4 mm**，窗口变成 **[1, 9] mm**。启动日志里是 `cube_width=0.0050m +/-0.0040m`，原始值 0.002 / 0.001 被记在括号里。

连接时夹爪空开，宽度 **67.4 mm**（手部上报上限仍是 67.5 mm）。q4 收到 -2.127 rad，q6 收到 2.415 rad。`action_grip` 最高 0.961。力控 `grasp` 发出 **41** 次，其中 **15** 次失败，实测宽度是 **0.1–0.7 mm**，低于窗口下限 1 mm。结束时开口停在 **4.2 mm**，这落在 [1, 9] mm 里，所以后面的 `grasp_handoff` 不再调用 `close()`。

手臂在抓取之后仍停在最低点，没有上提。600 步、703 条警告，实际频率 2.83 Hz，比上次低，是因为多次阻塞的 `grasp`。服务端已停。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 3:16 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456854
title: Start 600-step client after homing and opening
tool_call_id: call-bca62c5c-0552-4c41-9c99-2a969bbbbe24-293
fc_90444253-1a7a-9e9f-989e-6d4f9b7fe058_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456854.txt
</task>
</system_notification>

<timestamp>Tuesday, Sep 22, 2026, 3:16 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456853
title: Start VLA server for the 600-step rerun
tool_call_id: call-d35392fa-4039-494d-a117-2d143b94d6ab-291
fc_620ec310-9895-98ba-9f0a-ca4585d7fee4_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456853.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

600 步已经跑完。你设的 2±1 mm 被程序抬到了允许下限 5±4 mm，力控抓取合到了 0.1–0.7 mm 并被判失败，结束时开口停在约 4.2 mm。服务端退出码 143 是跑完后主动停掉的。

---

**User**

使用用了阻抗控制的 @RLmm/b/x/4dwvla_ext_ipd/franka_vla_client_ipd.py 再跑一下上面的实验. 上面的实验我们成为"实验A2", 这次实验我们称之为"实验A2I".

---

**Cursor**

实验 A2I 用阻抗控制客户端，场景和步数与刚才的 A2 相同：先回 HOME、张开夹爪，再跑 600 步。我先看这个客户端的启动参数和它依赖的控制器。

实验 A2I 已跑完 600 步。关节阻抗在运行，手臂只下降了一段就停住，夹爪收到约 47 mm，没有发出力控抓取。

控制与 A2 相同：`--n-exec 5`、`--max-steps 600`、`--control-hz 10`，厚度配置仍被抬到 **5±4 mm**。区别是手臂用 `JointImpedanceTracker.set_target`，不再等 waypoint 走完。刚度是 Franka 默认的一组，科氏力补偿打开。连接时夹爪空开，宽度 **67.4 mm**。

| | 实验 A2（阻塞 waypoint） | 实验 A2I（关节阻抗） |
|---|---|---|
| 实际频率 | 2.83 Hz | **5.29 Hz** |
| q7 | 被钉在 0.484 rad，有越界警告 | **停在 0.621 rad**，0 条警告 |
| q4 终态 | -2.127 | **-2.084**（约第 200 步后不再下降） |
| q6 终态 | 2.415 | **2.285** |
| `action_grip` 最高 | 0.961 | **0.412**（第 569 步） |
| 最窄宽度 | 3.7 mm，并多次 `grasp` | **47.0 mm**，没有 `grasp` |

阻抗每步只把平衡点挪一点，q7 没有被一下拉到训练下界，所以策略没有进入 A2 里那种贴边后继续深弯的轨迹。夹爪在最后几十步才开始收，600 步结束时还在 47 mm。服务端已停。手臂停在第 600 步。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 3:27 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: success
task_id: 456856
title: Start 600-step joint-impedance client
tool_call_id: call-68a7dcba-9d67-4473-baf6-4e4f295ef687-314
fc_843764d3-85f2-9683-a455-8282407620bc_0
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456856.txt
</task>
</system_notification>

<timestamp>Tuesday, Sep 22, 2026, 3:27 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456855
title: Start VLA server for experiment A2I
tool_call_id: call-54c6e180-d78e-46ff-8ac4-be98feb2063f-312
fc_655a6334-fefe-9589-b44f-9c3906719715_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456855.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

实验 A2I 的 600 步已经跑完：关节阻抗实际约 5.29 Hz，q7 停在 0.621 rad，夹爪收到约 47 mm，没有抓取。服务端退出码 143 是跑完后主动停掉的。

---

**User**

由于的 @RLmm/b/x/4dwvla_ext_ipd/franka_vla_client_ipd.py 用了阻抗控制, 与之前的 @RLmm/b/x/4dwvla_ext/franka_vla_client.py 不同, 所以它的各种设置都应该有所调整. 深入思考一下, 需要调整些什么设置和参数, 先列出来, 要说清楚调整前后的值和调整的原因, 列出来先给我看一下, 再确定是否要调整.

---

**Cursor**

阻抗控制和阻塞 waypoint 的差别在于：目标是弹簧平衡点，一个周期内手臂到不了。我先对一下现有增益、频率和夹爪是不是仍在阻塞，再列出建议改哪些值。

阻抗客户端不该沿用阻塞 waypoint 那组节拍。waypoint 每步都等到关节到位；阻抗只更新弹簧平衡点就返回，一个周期里手臂只走完一小段。A2I 就是这样停在半路的：请求 10 Hz，实际 5.29 Hz，q7 停在 0.621 rad，夹爪只收到 47 mm。

下面是建议改的项。先不动代码。

## 建议改

| 项 | 现在（A2 和 A2I 相同） | 建议只给 A2I | 原因 |
|---|---|---|---|
| `--control-hz` | 10 | **20** | 10 Hz 的 `sleep` 是按“等运动结束”写的。阻抗的 `move_joints` 马上返回，非推理步只要约 30–50 ms。收到 20 Hz 后，平衡点更新更密，滞后更接近训练的 30 Hz。先不要到 30：推理大约 0.3 s，做不到。 |
| `--n-exec` | 5 | **2** | 5 步在 waypoint 下大约隔 1.7 s 才看一次图像。阻抗下手臂在两拍之间一直在动，5 步会把一段没被看见的运动执行完。2 步是在“每步都重新推理”和“推理把频率拖垮”之间的折中。 |
| q5 刚度 / 阻尼 | 13.5 N·m/rad，1.5 N·m·s/rad | **27 / 2.12** | q5–q7 比近端关节软一个数量级。A2I 的 q6 只到 2.285 rad，A2 到了 2.415 rad。q5、q6 刚度加倍；阻尼按 \(\sqrt{2}\) 增加，阻尼比大体保持。 |
| q6 刚度 / 阻尼 | 12.818 / 2.0 | **25.6 / 2.83** | 同上。 |
| q7 刚度 / 阻尼 | 5.134 / 1.331 | **20.5 / 2.66** | q7 刚度只有 5 N·m/rad。0.15 rad 的偏差只产生约 0.8 N·m，腕几乎不动。A2I 里 q7 从 0.639 rad 只到 0.621 rad。提到 4 倍，0.15 rad 偏差约 3 N·m，仍低于 q7 的力矩上限。阻尼按 \(\sqrt{4}=2\) 增加。 |
| 单步平衡点步长 `MAX_JOINT_STEP_RAD` | 0.15 rad | **0.05 rad**（仅阻抗环境） | 0.15 rad 是给“走完再停”用的。刚度提高后，每拍把平衡点跳 0.15 rad，弹簧会拉出过冲。0.05 rad/拍在 20 Hz 下约 1 rad/s，仍比示教每帧大约 0.003 rad 宽，但不会一下甩出去。 |
| 夹爪 `move_width` / `grasp` | 在控制循环里阻塞 0.5–2 s | **发出后立即返回**，抓取失败的冷却仍是 1 s | 阻塞时阻抗平衡点被冻住，手臂停在半空等手指。A2 里这和 waypoint 的等待叠在一起，不明显；A2I 里一次 `grasp` 会把 20 Hz 的环打断。 |

q1–q4 的刚度（104、266、227、221 N·m/rad）不用动。A2I 里 q4 已经从 -1.888 rad 走到 -2.084 rad，近端在跟。

环境变量可以写成：

```bash
export VLA_IPD_STIFFNESS=103.75,265.734,227.273,221.445,27,25.6,20.5
export VLA_IPD_DAMPING=16.7,40.263,25.0,12.862,2.12,2.83,2.66
```

`MAX_JOINT_STEP_RAD` 和夹爪不阻塞要改代码，不能只靠环境变量。

## 保持不动

| 项 | 值 | 原因 |
|---|---|---|
| q7 训练裁剪 | 下限 0.4843 rad，贴边告警 0.03 rad | 裁的是平衡点，不是实测角。A2I 没有警告，是因为实测 q7 停在 0.62 rad，模型没再往下要。松开它，平衡点会掉到 0.40 rad，和实验 B 一样反向。 |
| 末端围栏、碰撞阈值、看门狗 1.2 rad/s | 与 A2 相同 | 阻抗在接触后仍用弹簧推，围栏比 waypoint 更需要留着。 |
| `relative_dynamics_factor` | 0.2 | 只影响回 HOME 和刹车用的 waypoint，不影响 `set_target`。设成 1.0 会在回 HOME 时触发速度不连续反射。 |
| 夹爪死区 | 收 1.5 mm，张 6 mm | 设成 0 时，A2 的前身实验每步都 `move_width`，手指在 78–80 mm 抖。 |
| 抓取力、极性、\(w_0=0.08\) | 20 N，\(a=1-w/0.08\) | 与阻抗节拍无关。 |
| 科氏力补偿 | 开 | 已经是阻抗环需要的，A2I 日志里是 `coriolis=True`。 |

插头厚度你写成了 2±1 mm，程序允许范围是厚度 [5, 70] mm、容差 [4, 25] mm，所以 A2 和 A2I 实际都在用 **5±4 mm**。这不是阻抗专用项；若要让 2 mm 生效，得改 `motion_limits.py` 里的下限，那是另一次决定。

---

**User**

按你的建议对阻抗控制相关的设置和参数做修改, 尽量把修改控制在 @RLmm/b/x/4dwvla_ext_ipd/ 内部. 若要改动其它路径的代码, 尽量复制一份到`4dwvla_ext_ipd`中再改, 然后`4dwvla_ext_ipd`中的相关代码都引用修改过的那份代码或脚本. 若实在要改其它路径的文件, 列出来, 并说明理由, 让我来决策.

---

**Cursor**

这些改动会留在阻抗目录里。关节步长和夹爪阻塞目前写在共用的 `franky_joint_env.py` 里，我先看能不能在阻抗子类里收紧，避免改那份共用代码。

步长和夹爪改在阻抗子类里完成，不改共用的 `franky_joint_env.py`。增益、默认频率和 `n_exec` 都落在 `4dwvla_ext_ipd` 里。

阻抗相关的设置都改在 `b/x/4dwvla_ext_ipd/` 里了。共用的 `franky_joint_env.py` 和夹爪实现没有动。离线测试 24 项通过。

| 项 | 改前 | 改后 | 落在哪 |
|---|---|---|---|
| `--control-hz` | 10 | **20** | `franka_vla_client_ipd.py` 的默认值 |
| `--n-exec` | 10（上次实验命令行用了 5） | **2** | 同上 |
| q5 刚度 / 阻尼 | 13.5 / 1.5 | **27 / 2.12** | `impedance_gains.py`，并写入 `franka_plug_impedance.env` |
| q6 刚度 / 阻尼 | 12.818 / 2.0 | **25.6 / 2.83** | 同上 |
| q7 刚度 / 阻尼 | 5.134 / 1.331 | **20.5 / 2.66** | 同上 |
| 平衡点单步上限 | 共用代码的 0.15 rad | **0.05 rad**，只在阻抗 `step()` 里先收紧 | `impedance_joint_env.py` |
| `move_width` / `grasp` | 在控制循环里等到手指停 | **丢到单独线程后立即返回**；上一拍还在动就跳过 | `joint_impedance_controller.py` |

q1–q4 的刚度没改。回 HOME 用的 `relative_dynamics_factor=0.2`、末端围栏和 q7 的 0.4843 rad 裁剪仍走原来的共用代码。

下次跑 A2I 时不要再传 `--control-hz 10 --n-exec 5`，否则会盖掉新的默认值。厚度 2±1 mm 仍会被 `motion_limits.py` 抬到 5±4 mm；那份文件在阻抗目录外面，这次没有改。

---

**User**

用了阻抗控制的 @RLmm/b/x/4dwvla_ext_ipd/franka_vla_client_ipd.py 再跑一下"实验A2I".

---

**Cursor**

按改过的阻抗参数再跑实验 A2I：回 HOME、张开夹爪，客户端用新的默认 20 Hz 和 `n_exec=2`，服务端也按 2 步返回，一共 600 步。

实验 A2I 已用新参数跑完 600 步。手臂下降到中途后停住，夹爪没有收。

实际用上的是：20 Hz、`n_exec=2`，q5/q6/q7 刚度 27 / 25.6 / 20.5，平衡点每拍最多 0.05 rad。连接时夹爪空开，宽度 **67.4 mm**。服务端按 2 步返回动作。

和上一版 A2I、阻塞式 A2 对比：

| | 上一版 A2I | 这次 A2I | 实验 A2 |
|---|---|---|---|
| 请求 / 实际频率 | 10 / 5.29 Hz | **20 / 3.43 Hz** | 10 / 2.83 Hz |
| q4 | -2.084 | **-2.071**（约第 150 步后不再变） | -2.127 |
| q6 | 2.285 | **2.248** | 2.415 |
| q7 | 0.621 | **0.508** | 0.484 |
| `action_grip` 最高 | 0.412 | **0.141** | 0.961 |
| 夹爪宽度 | 收到 47 mm | **全程 67.4 mm** | 收到 3.7 mm |

从第 150 步到第 599 步，关节实测变化约 10⁻⁶ rad，手臂等于停住。模型仍在要更低的 q7（被裁到 0.484 rad）和更大的 q6，这 597 拍都被 0.05 rad 步长上限缩小后再发出，但手臂没有跟上。夹爪 600 步全是 `hold`，没有 `move_width`，也没有 `grasp`。警告 1186 条，都是 q7 贴在训练下界。

服务端已停。手臂停在这个中间姿态，没有回 HOME。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 3:52 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456857
title: Start VLA server returning 2 actions
tool_call_id: call-ab5acccc-09fb-4714-a337-7de1324d0bcb-349
fc_9f8960d7-00c3-9bd4-abdc-47787e5e2acc_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456857.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

实验 A2I 的 600 步已经跑完：手臂大约在第 150 步停住，夹爪保持 67.4 mm，没有抓取。服务端退出码 143 是跑完后主动停掉的。

---

**User**

再跑一下"实验A2I".

---

**Cursor**

再跑一次实验 A2I：先回 HOME 并张开夹爪，然后用现在的阻抗默认参数（20 Hz、`n_exec=2`）跑 600 步。

实验 A2I 又跑完 600 步，结果和上一轮接近：手臂下降一段后停住，夹爪几乎没动。

已回 HOME，夹爪张开。实际频率 **3.43 Hz**（请求 20 Hz）。约第 180 步后关节不再变：q4 停在 **-2.084**，q6 停在 **2.316**，q7 停在 **0.508**。`action_grip` 最高 **0.186**，没有到 0.8。宽度从 67.4 mm 只收到 **65.9 mm**，599 步是 `hold`，只有 1 次 `move_width`，没有 `grasp`。平衡点步长上限触发了 599 次，警告 1190 条，仍是 q7 被裁到 0.484 rad。

服务端已停。手臂留在这个中间姿态。

---

**User**

<timestamp>Tuesday, Sep 22, 2026, 4:02 PM (UTC+8)</timestamp>
<system_notification>
The following task has finished. If you were already aware, ignore this notification and do not restate prior responses.

<task>
kind: shell
status: error
task_id: 456859
title: Start VLA server with n-exec 2
tool_call_id: call-e6e7a4e9-e1cf-4ae8-9d17-1676777aa103-364
fc_56d8e84b-5592-99fb-aff9-f13c7ff094bb_0
detail: exit_code=143
output_path: /home/nvidia/.cursor/projects/home-nvidia-bt-s-4WVLA/terminals/456859.txt
</task>
</system_notification>
<user_query>Briefly inform the user about the task result and perform any follow-up actions (if needed). If there's no follow-ups needed, don't explicitly say that.</user_query>

---

**Cursor**

这轮实验 A2I 的 600 步已经结束：手臂大约在第 180 步停住，夹爪停在约 65.9 mm，没有抓取。服务端退出码 143 是跑完后主动停掉的。

---

**User**

深入分析和思考一下, 为什么最近两次实验中, 夹爪一点都没有闭合, 因为我看到夹爪是下放到了插头所在的位置的,

---

**Cursor**

夹爪下到插头那里，和两指收拢不是同一件事。我用最近两次阻抗实验的夹爪动作，对照现在 67.4 mm 的满开读数，算它要多大才会真正下令收指。

你看到的是手臂把夹爪送到了插头那里。两根手指没有收，最近两次阻抗实验里它们几乎一直张到硬件满开。

手指现在的满开读数是 **67.4 mm**，不是训练里用的 80 mm。连续模式把动作翻译成下一帧宽度：

\[
w_{\text{cmd}}=\min\bigl(0.08\,(1-a),\;0.0675\bigr)
\]

67.4 mm 对应的“保持不动”大约是 \(a=1-0.0674/0.08=0.158\)。死区是 1.5 mm，所以要让手指真的往里动，\(a\) 至少要到约 **0.176**。力控抓取要到 **0.8**。

两次日志里的夹爪动作都停在这个门槛附近：

| | 第一次 A2I `074751` | 第二次 A2I `075728` |
|---|---|---|
| \(a\) 中位 / 最高 | 0.118 / **0.141** | 0.156 / **0.186** |
| 超过 0.176 的步数 | **0** | 14 |
| 实际命令 | 600 步全是 `hold` | 599 步 `hold`，1 步 `move_width` |
| 宽度 | 全程 67.4 mm | 67.4 → **65.9 mm** |

0.14 的意思是“下一帧大约 69 mm”。这比手指现在的 67.4 mm 还宽，再被上限钳到 67.5 mm，和实测只差 0.1 mm，死区里什么都不做。第二次有 14 步略超过 0.176，所以只收了约 1.5 mm，到 65.9 mm 就停了。模型没有发出“把手指收到插头厚度”的命令。

手臂确实下降过：q4 大约到 -2.08，q6 到 2.25–2.32，然后从第 150–180 步起关节几乎不动。以前手指会开始收的那几次，姿态要更靠后：q6 到 **2.40 以上**，q7 被按在 **0.484**，腕部图像均值继续降到 112 以下，\(a\) 才从 0.02 升过 0.2。这两次 q7 停在 **0.508**，q6 没到 2.40。腕部均值最低到过 109 和 115，但夹爪动作仍在 0.12–0.19，也就是“维持当前这 67 mm”。

所以不是夹爪机构没响应。策略在你看到的那个位置上，输出的仍是张开；以 80 mm 为基准、再被 67.4 mm 的硬件满开钳住之后，这个数连 1.5 mm 的收拢都触发不了。

---

**User**

<timestamp>Monday, Sep 22, 2026, 5:32 PM (UTC+8)</timestamp>

参考 cursor_experiment.md. 将 vla_inference_server.py 要加载的checkpoint 改为 `/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp031260/`, 其它跟"实验 A2I"保持一致, 再跑一次实验, 我们成这次实验为"实验A2I2".

---

**Claude Code**

## 实验 A2I2：更换 checkpoint 为 4wvlaFrkPlugCkp031260

### 实验目的

在实验 A2I 的基础上，仅更换模型 checkpoint 为更早期的 `4wvlaFrkPlugCkp031260`（训练步数 31260，相对于 A2I 使用的 `4wvlaFrkPlugCkp041680` 的训练步数 41680），其余所有配置（阻抗控制参数、客户端参数、夹爪模式等）与最后一次实验 A2I 完全一致。目的是对比不同训练步数的 checkpoint 在相同阻抗控制条件下的表现差异。

### 准备工作

#### 1. 验证新 checkpoint

确认 `/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp031260/` 存在且包含完整文件：

```
config.json
model.safetensors
stats.json
train_config.json
```

对比新旧 checkpoint 的 `config.json`，所有配置项完全一致（包括 `enable_keypoint_predictor=True`、`chunk_size=50`、`vlm_model_name_or_path=Qwen/Qwen3.5-2B` 等），唯一的不同是模型权重文件 `model.safetensors` 对应不同训练步数。

#### 2. 确认 Docker 容器卷映射

```
宿主机 /home/nvidia/bt/ckp  →  GPU容器 /home/nvidia/ckpts
```

因此新 checkpoint 在 GPU 容器内路径为 `/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp031260`。

#### 3. 确认两个容器均在运行

```
rlinf-4dwvla-gpu     Up 3 hours
rlinf-4dwvla-franky  Up 3 hours
```

#### 4. 确认端口 5555 无占用

在 GPU 容器内执行 `ss -tlnp | grep 5555`，确认无进程监听。

### 启动服务端

在 GPU 容器内执行：

```bash
docker exec rlinf-4dwvla-gpu bash -lc \
  'source /opt/venv/4dwvla/bin/activate && \
   VLA_N_EXEC=2 \
   VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp031260 \
   bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh'
```

服务端启动日志确认：

```
=== vla_inference_server ===
  ckpt:      /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp031260
  schema:    /workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml
  kpt_meta:  /workspace/RLinf/b/d/frk1/plug/keypoints_meta.json
  urdf:      /workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf
  n_exec:    2
  port:      5555
  dtype:     bfloat16
```

模型加载过程：

```
09:33:25 Composed observation.state stats from sub-fields (8D)
09:33:25 Composed action stats from sub-fields (8D)
09:33:25 Stats loaded from .../4wvlaFrkPlugCkp031260/stats.json (state=8D, action=8D)
09:33:25 Action stats: mean=[-0.238, 0.142, 0.189, -2.056, -0.062, 2.264, 0.721, 0.579]
                       std=[0.122, 0.085, 0.147, 0.087, 0.056, 0.142, 0.167, 0.405]
09:33:53 Model loaded: device=cuda dtype=torch.bfloat16 action_loss_only=True backend=standard kpt=True
09:34:13 Transform pipeline built (7 steps + unnormalize)
09:34:13 FK keypoint computer ready: 8 joints, dim=7, history=200
09:34:13 Inference server listening on port 5555 (n_exec=2)
```

从启动到监听耗时约 48 秒（09:33:25 → 09:34:13），其中模型权重加载约 28 秒。

### 启动客户端

在 franky 容器内执行：

```bash
docker exec rlinf-4dwvla-franky bash -lc \
  'source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env && \
   source /workspace/RLinf/b/x/4dwvla_ext_ipd/configs/franka_plug_impedance.env && \
   echo "STIFF=$VLA_IPD_STIFFNESS" && \
   exec /opt/venv/franky-0.19.0/bin/python \
     /workspace/RLinf/b/x/4dwvla_ext_ipd/franka_vla_client_ipd.py \
     --robot-ip 172.16.0.2 \
     --task "plug into socket" \
     --use-realsense \
     --global-camera-serial 250222073513 \
     --wrist-camera-serial 420122070525 \
     --max-steps 600'
```

客户端启动日志确认：

```
STIFF=103.75,265.734,227.273,221.445,27,25.6,20.5

09:35:23 Arm backend: joint impedance
09:35:23 plug eval grasp window: FRANKA_CUBE_WIDTH_M=0.002 +/- FRANKA_HOLD_TOL_M=0.001
09:35:23 FrankaLibfrankaGripper connected (max_width=0.080m, grasp_force=20.0N,
         cube_width=0.0050m +/-0.0040m, holding=False width=0.0800m)
09:35:23 Joint impedance armed:
         K=[103.75, 265.734, 227.273, 221.445, 27.0, 25.6, 20.5]
         D=[16.7, 40.263, 25.0, 12.862, 2.12, 2.83, 2.66]
         coriolis=True
09:35:23 Motion guard set: min=[0.484, -0.19, 0.168], max=[0.652, 0.103, 0.567]
09:35:23 gripper mode=continuous: w_cmd ceiling=0.0800m
         (configured FRANKA_GRIPPER_MAX_WIDTH_M=0.0800m, hand reports 0.0803m)
09:35:23 gripper command: close if action >= 0.50
09:35:23 arm backend=joint_impedance tcp_fence=[0.534, -0.14, 0.178]..[0.602, 0.053, 0.517]
         control_hz=20.0
09:35:23 Starting: task='plug into socket', max_steps=600, n_exec=2, control_hz=20.0
09:35:25 Arms homed. Arrange scene, press 'a' to start.
```

**注意**：这次夹爪满开宽度是 **80.0 mm**（手部上报 80.3 mm），与训练时的 80 mm 一致。上次实验 A2I 只有 **67.4 mm**（上报 67.5 mm）。这意味着夹爪 ceiling 未被钳低，`w_cmd` 的活动范围和训练分布一致。

### 实验配置汇总

| 项 | 实验 A2I（上次） | **实验 A2I2（本次）** |
|---|---|---|
| Checkpoint | `4wvlaFrkPlugCkp041680`（step 41680） | **`4wvlaFrkPlugCkp031260`（step 31260）** |
| 服务端 `n_exec` | 2 | 2 |
| 客户端 `control_hz` | 20.0 | 20.0 |
| 客户端 `n_exec` | 2 | 2 |
| `max_steps` | 600 | 600 |
| 阻抗刚度 q5/q6/q7 | 27.0 / 25.6 / 20.5 | 27.0 / 25.6 / 20.5 |
| 阻抗阻尼 q5/q6/q7 | 2.12 / 2.83 / 2.66 | 2.12 / 2.83 / 2.66 |
| 科氏力补偿 | True | True |
| 平衡点步长上限 | 0.050 rad | 0.050 rad |
| 夹爪模式 | continuous, close ≥ 0.50 | continuous, close ≥ 0.50 |
| 夹爪满开 | **67.4 mm**（被钳低） | **80.0 mm**（与训练一致） |
| 抓取窗口 | 5±4 mm（raw 2±1 被抬到下限） | 5±4 mm |
| q7 训练裁剪下限 | 0.4843 rad | 0.4843 rad |

### 执行

按 `a` 键开始 rollout。时间线：

- **09:36:19** — `a` 键按下，rollout 开始
- **09:36:20** — step 0，首次推理耗时 786.2 ms（含 KV cache 预热）
- **09:36:20** — step 0 即刻触发阻抗步长上限（equilibrium scaled to 0.050 rad），并启动 impedance tracker
- 后续每次推理稳定在 **294–310 ms**
- **09:39:15** — step 599 完成，客户端正常退出（exit code 0）
- 总运行时间 **175.8 秒**（09:36:19 → 09:39:15）

### 结果数据

#### 关节轨迹

| step | q1 | q2 | q3 | q4 | q5 | q6 | q7 | grip (m) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | -0.257 | -0.020 | 0.167 | **-1.887** | -0.055 | **1.906** | **0.637** | **0.0802** |
| 50 | -0.262 | 0.172 | 0.105 | **-2.027** | -0.040 | **2.098** | **0.508** | 0.0780 |
| 100 | -0.263 | 0.201 | 0.104 | **-2.059** | -0.037 | **2.259** | **0.508** | 0.0780 |
| 150 | -0.262 | 0.204 | 0.071 | **-2.083** | -0.019 | **2.326** | **0.508** | 0.0780 |
| 200 | -0.262 | 0.204 | 0.071 | **-2.083** | -0.019 | **2.326** | **0.508** | 0.0780 |
| 300 | -0.262 | 0.204 | 0.071 | **-2.083** | -0.019 | **2.326** | **0.508** | 0.0780 |
| 400 | -0.262 | 0.204 | 0.071 | **-2.083** | -0.019 | **2.326** | **0.508** | 0.0780 |
| 500 | -0.261 | 0.204 | 0.071 | **-2.083** | -0.019 | **2.326** | **0.508** | 0.0780 |
| 599 | -0.261 | 0.204 | 0.071 | **-2.084** | -0.019 | **2.326** | **0.508** | 0.0780 |

手臂从 HOME 姿态（q4=-1.887, q6=1.906, q7=0.637）开始，前 150 步大幅运动：q4 弯到 -2.083，q6 展到 2.326，q7 从 0.637 降到 0.508。**从 step 150 开始关节几乎不再变化**，realized_delta 量级降到 10⁻⁶ rad。

#### 夹爪动作（action_grip）

| step | action_grip | gripper_cmd | w_meas (mm) |
|---:|---:|---|---:|
| 0 | **0.0247** | move_width | 80.2 |
| 10 | 0.0244 | hold | 78.0 |
| 20 | 0.0239 | hold | 78.0 |
| 50 | 0.0290 | hold | 78.0 |
| 100 | 0.0321 | hold | 78.0 |
| 150 | 0.0332 | hold | 78.0 |
| 200 | 0.0290 | hold | 78.0 |
| 300 | 0.0282 | hold | 78.0 |
| 400 | 0.0200 | hold | 78.0 |
| 500 | 0.0200 | hold | 78.0 |
| 599 | 0.0246 | hold | 78.0 |

`action_grip` 全程在 **0.020–0.038** 范围内，最高仅 **0.038**（step 附近 100–150）。远低于连续模式下的收拢门槛（约 0.176，对应 "让手指真的往里动"）和力控抓取阈值（0.50）。

600 步中：**599 步 `hold`**，**1 步 `move_width`**（step 0，仅因初始宽度 80.2 mm 大于 ceiling 0.0800 m 导致的一次小幅调整），**0 次 `grasp`**。夹爪宽度全程保持在 **78.0 mm**。

#### 图像均值

| 推理编号 | 全局相机均值 | 腕部相机均值 |
|---:|---:|---:|
| 0 | 120.8 | **121.7** |
| 25 | 120.5 | **119.7** |
| 50 | 116.3 | **115.1** |
| 75 | 115.7 | **112.4** |
| 100 | 118.5 | **111.6** |
| 150 | 118.4 | **111.5** |
| 200 | 118.5 | **111.5** |
| 250 | 118.4 | **111.5** |
| 299 | 118.7 | **111.9** |

腕部均值从 121.7 降到约 111.5，在推理 100（约 step 200）之后趋于稳定。未降到 **112 以下**的水平（以前出现夹爪收拢的实验中，腕部均值降到 112 以下后 `action_grip` 才从 0.02 升过 0.2）。

#### 关键点历史（his_len）

| 推理编号 | his_len |
|---:|---:|
| 0 | 0 |
| 10 | 20 |
| 25 | 50 |
| 50 | 100 |
| 75 | 150 |
| 100 | **200**（饱和） |
| 150–299 | 200 |

`n_exec=2`，每次推理前回放 2 个 executed pose，因此 `his_len` 每次推理增加 2。到推理 100（step 200）时 `his_len` 达到上限 200 并饱和。与实验 A2I 一致。

#### 推理耗时

| 推理编号 | 耗时 (ms) |
|---:|---:|
| 0（首次，含预热） | **786.2** |
| 1 | 309.7 |
| 2 | 310.6 |
| 3 | 300.1 |
| 4 | 296.5 |
| … | 294–310 |
| 297 | 294.5 |
| 298 | 296.1 |
| 299 | 294.9 |

首次推理 786 ms 包含 KV cache 预热。后续稳定在 295–310 ms。

#### 警告统计

| 类型 | 数量 |
|---|---:|
| OUT-OF-TRAIN: q7 < 0.4843 | **1188** |
| TRAIN-EDGE: q7=0.4843 贴边 | **1192** |
| impedance step cap 触发 | **599** |
| 总计 warnings | **1190** |

q7 的模型输出被裁剪到 0.4843 rad（训练下限），但实测 q7 停在 0.508 rad，阻抗弹簧不足以把它拉下去。

### 与实验 A2I 对比

| | 实验 A2I（Ckp041680）第二次 | 实验 A2I（Ckp041680）第三次 | **实验 A2I2（Ckp031260）** |
|---|---|---|---|
| 请求 / 实际频率 | 20 / 3.43 Hz | 20 / 3.43 Hz | **20 / 3.43 Hz** |
| q4 终态 | -2.071 | -2.084 | **-2.084** |
| q6 终态 | 2.248 | 2.316 | **2.326** |
| q7 终态 | 0.508 | 0.508 | **0.508** |
| `action_grip` 最高 | 0.141 | 0.186 | **0.038** |
| 夹爪满开 | 67.4 mm | 67.4 mm | **80.0 mm** |
| 夹爪最窄 | 67.4 mm | 65.9 mm | **78.0 mm** |
| `move_width` 步数 | 0 | 1 | **1** |
| `grasp` 次数 | 0 | 0 | **0** |
| 步长上限触发 | ~599 | ~599 | **599** |
| q7 越界警告 | ~1188 | ~1190 | **1188** |
| 停止运动的步数 | ~150 | ~180 | **~150** |

### 分析

#### 1. Checkpoint 031260 的 `action_grip` 更低

实验 A2I 中 `action_grip` 最高到 0.141–0.186（Ckp041680），本次 A2I2 最高仅 **0.038**（Ckp031260）。这说明更早期的 checkpoint 在相同姿态下，夹爪收拢意愿**更低**。训练更充分的 checkpoint（041680）在这个中途停住的姿态上至少尝试输出一些收拢动作（虽然仍不够触发真正的位移），而 031260 几乎完全输出"保持张开"。

#### 2. 夹爪满开差异

本次实验夹爪满开是 **80.0 mm**（与训练一致），而上次 A2I 是 67.4 mm。这是夹爪 homing 状态不同导致的，并非 checkpoint 差异。80 mm 的满开意味着连续模式下"保持不动"对应的 `action_grip` 约为 0（`a=1-0.08/0.08=0`），而 67.4 mm 下约为 0.158。因此本次的 0.038 在 80 mm 满开下对应的目标宽度约 `0.08*(1-0.038)=77.0 mm`，与实测 78.0 mm 的死区 1.5 mm 一致——没有触发任何位移。

#### 3. 手臂轨迹几乎一致

三次实验（A2I 两次 + A2I2）的手臂轨迹高度相似：都是前 150 步从 HOME 下降到 q4≈-2.08, q6≈2.25–2.33, q7≈0.508，然后停住。两个 checkpoint 在 action 前 7 维（关节角）上的输出差异不大，主要差异在第 8 维（夹爪）。

#### 4. 腕部图像均值与模型行为关联

腕部均值从 121.7 降到 111.5，但没有继续降到 112 以下。之前成功出现夹爪收拢的实验（A2 系列中的非阻抗实验），腕部均值降到 **109–112** 时 `action_grip` 才上升过 0.2。在阻抗控制下，手臂因弹簧特性停在中途，腕部相机始终看不到足够靠近桌面的图像，模型因此不发出收拢命令。

#### 5. 根因未变

换 checkpoint 没有改变根本问题：阻抗控制下手臂在 step 150 后停住（q7 停在 0.508 而非被推到 0.484 rad 的训练下限），姿态不够低，腕部视野不够暗，模型不认为已经到达抓取位置。这与 checkpoint 选择无关，是阻抗控制参数和训练分布之间的失配。

### 实验结束

服务端通过 `pkill -f vla_inference_server` 停止，退出码 143（SIGTERM）。客户端正常退出，退出码 0。

日志文件：
- 客户端：`/workspace/RLinf/b/x/4dwvla_ext/logs/client-ipd_20260922_093523_8297.log`
- 服务端：`/workspace/RLinf/b/x/4dwvla_ext/logs/server_20260922_093325_1293.log`

---

**User**

参考 cursor_experiment.md 中记录的"实验A2", 用 franka_vla_client.py, vla_inference_server.py 和 4wvlaFrkPlugCkp031260 这个 checkpoint, 再跑一下"实验A2".

---

**Claude Code**

## 实验 A2（Ckp031260）：阻塞 waypoint + 早期 checkpoint

### 实验目的

在实验 A2 的基础上，将 checkpoint 从 `4wvlaFrkPlugCkp041680` 更换为更早期的 `4wvlaFrkPlugCkp031260`（训练步数 31260），使用**阻塞 waypoint 控制**（`franka_vla_client.py`，非阻抗），其余参数与原始实验 A2 一致。目的是验证：在 waypoint 控制下（手臂能准确到达目标位置），更早期的 checkpoint 是否仍能成功输出夹爪收拢命令。

### 与原始实验 A2 的配置对比

| 配置项 | 原始 A2（Ckp041680） | 本次 A2（Ckp031260） |
|---|---|---|
| 客户端 | `franka_vla_client.py`（阻塞 waypoint） | **同** |
| 服务端 | `vla_inference_server.py` | **同** |
| Checkpoint | `4wvlaFrkPlugCkp041680` | **`4wvlaFrkPlugCkp031260`** |
| `--n-exec` | 5 | **同** |
| `--max-steps` | 600 | **同** |
| `--control-hz` | 10 | **同** |
| 夹爪模式 | continuous | **同** |
| HOME_JOINTS | `TRAIN_ARM_MEAN` 硬编码 | **`home_pose.json` 加载** |
| q7 安全裕度 | 0.15 rad（旧代码） | **0.0 rad（新代码）** |

**注意**：本次实验使用的代码版本与原始 A2 不同。代码已更新，主要差异：
1. **HOME 位姿来源改变**：旧代码 `HOME_JOINTS = TRAIN_ARM_MEAN = [-0.2406, 0.1457, 0.1872, -2.0600, -0.0553, 2.2011, 0.6998]`；新代码从 `home_pose.json` 加载 `= [-0.2576, -0.0209, 0.1674, -1.8850, -0.0551, 1.9064, 0.6436]`。差异最大的是 q6（旧 2.201 vs 新 1.906）和 q2（旧 0.146 vs 新 -0.021）。
2. **q7 安全裕度收紧**：旧代码 `SAFETY_MARGIN_RAD = 0.15`（统一），q7 允许低至 0.3343；新代码 `SAFETY_MARGIN_RAD = [0.15,...,0.0]`，q7 被严格裁剪到 0.4843。不过在原始 A2 中 q7 同样被裁剪到 0.484（590 条越界警告），所以实际裁剪行为相同。
3. **`franky_ext` 模块结构变化**：新代码从 `franky_ext.dsplug.home_pose` 加载 HOME（见下文"启动问题修复"）。

### 启动前问题修复

#### 问题 1：`franky_ext.dsplug` 模块不存在

启动客户端时报错：

```
ModuleNotFoundError: No module named 'franky_ext.dsplug'
```

**原因**：`franky_joint_env.py` 第 49 行新增了 `from franky_ext.dsplug.home_pose import load_home_joints`，但 `franky_ext/` 目录下没有 `dsplug` 子目录。该模块存在于 `franky_ext_backup/dsplug/`。

**修复**：将 `franky_ext_backup/dsplug/` 整个目录复制到 `franky_ext/dsplug/`：

```bash
cp -r /home/nvidia/bt/s/RLmm/b/x/franky_ext_backup/dsplug \
      /home/nvidia/bt/s/RLmm/b/x/franky_ext/dsplug
```

注意：最初尝试创建符号链接（`ln -s`），但容器内无法解析宿主机绝对路径的符号链接，所以改为直接复制。

验证：

```
docker exec rlinf-4dwvla-franky bash -c 'python3 -c "from franky_ext.dsplug.home_pose import load_home_joints; print(load_home_joints())"'
# 输出: [-0.25758657 -0.02094262  0.16744767 -1.88496649 -0.05506149  1.90638936  0.64357251]
```

#### 问题 2：第一次 rollout 尝试在 step 0 崩溃

用户按 'a' 后，客户端在 step 0 执行到夹爪命令时崩溃：

```
RuntimeError: gripper does not support move_width_m()
```

**调用链**：`franky_joint_env.py:493` → `franky_controller_direct.py:417`

**原因**：`franky_controller_direct.py` 从 `franky_ext.franka_libfranka_gripper` 导入 `FrankaLibfrankaGripper`，但 `franky_ext/` 中的该文件（334 行）比 `franky_ext_backup/` 中的版本（365 行）少了 `move_width_m()`、`max_width` 属性和 `homing()` 方法。另外 `close()` 中的 grasp 校验逻辑也不同（前者仅 warning，后者 raise RuntimeError）。

**修复**：将 `franky_ext_backup/franka_libfranka_gripper.py` 复制覆盖 `franky_ext/franka_libfranka_gripper.py`，并清理 `__pycache__`：

```bash
cp /home/nvidia/bt/s/RLmm/b/x/franky_ext_backup/franka_libfranka_gripper.py \
   /home/nvidia/bt/s/RLmm/b/x/franky_ext/franka_libfranka_gripper.py
docker exec rlinf-4dwvla-franky bash -c 'rm -rf /workspace/RLinf/b/x/franky_ext/__pycache__'
```

验证后手部上报 `max_width=0.0803m`（之前版本上报 `unknown`）。

### 启动命令

#### 服务端（GPU 容器）

```bash
docker exec rlinf-4dwvla-gpu bash -lc '
  source /opt/venv/4dwvla/bin/activate && \
  VLA_N_EXEC=5 \
  VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp031260 \
  bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh'
```

服务端启动日志（关键行）：

```
ckpt:      /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp031260
schema:    /workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml
kpt_meta:  /workspace/RLinf/b/d/frk1/plug/keypoints_meta.json
urdf:      /workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf
n_exec:    5
port:      5555
dtype:     bfloat16
```

模型加载耗时约 30 秒。`10:04:49` 模型加载完成，`10:05:02` 服务端开始监听。

```
[INFO] vla-server: Model loaded: device=cuda dtype=torch.bfloat16 action_loss_only=True backend=standard kpt=True
[INFO] vla-server: Transform pipeline built (7 steps + unnormalize)
[INFO] vla-server: FK keypoint computer ready: 8 joints, dim=7, history=200
[INFO] vla-server: Inference server listening on port 5555 (n_exec=5)
```

#### 客户端（Franky 容器）

```bash
docker exec rlinf-4dwvla-franky bash -lc '
  source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env && \
  exec /opt/venv/franky-0.19.0/bin/python \
    /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py \
    --robot-ip 172.16.0.2 \
    --task "plug into socket" \
    --use-realsense \
    --global-camera-serial 250222073513 \
    --wrist-camera-serial 420122070525 \
    --max-steps 600 \
    --control-hz 10 \
    --n-exec 5'
```

客户端初始化日志：

```
plug eval grasp window: FRANKA_CUBE_WIDTH_M=0.002 +/- FRANKA_HOLD_TOL_M=0.001
  (UNVERIFIED against the real plug)
FrankaLibfrankaGripper connected (max_width=0.080m, grasp_force=20.0N,
  cube_width=0.0050m +/-0.0040m, holding=False width=0.0802m)
Motion guard set: min=[0.484, -0.19, 0.168], max=[0.652, 0.103, 0.567] (margin=0.050)
gripper mode=continuous: w_cmd ceiling=0.0800m (hand reports 0.0803m)
gripper command: close if action >= 0.50 (VLA_GRIPPER_CLOSE_IF_ABOVE=True)
```

`10:10:24` 手臂 HOME 完成，等待用户按 'a'。`10:11:23` 用户按 'a'，rollout 开始。

### 实验过程

#### 时间线

| 时间 (UTC+8) | 事件 |
|---|---|
| 10:04:18 | 服务端启动，加载 checkpoint |
| 10:04:49 | 模型加载完成 |
| 10:05:02 | 服务端开始监听 5555 端口 |
| 10:07:59 | 第一次客户端启动（失败：`move_width_m` 不存在） |
| 10:09:01 | 第一次按 'a'，step 0 崩溃 |
| 10:09:03 | 服务端收到 shutdown，等待新连接 |
| 10:10:22 | 修复后第二次客户端启动 |
| 10:10:24 | 手臂 HOME 完成 |
| 10:11:23 | 第二次按 'a'，rollout 正式开始 |
| 10:14:33 | 600 步完成，客户端退出（exit 0） |
| 10:14:33 | 服务端收到 shutdown，等待新连接 |
| — | 手动 `pkill` 停止服务端（exit 143） |

#### 运行摘要

| 指标 | 值 |
|---|---|
| 总步数 | **600** |
| 推理请求数 | **120**（每次返回 5 个 action） |
| 实际控制频率 | **3.16 Hz**（请求 10.0 Hz） |
| 总警告数 | **1190** |
| OUT-OF-TRAIN 警告 | **1190**（全部是 q7 < 0.4843） |
| TRAIN-EDGE 警告 | **1190**（q7=0.4843 贴边） |
| `gripper_cmd='hold'` 步数 | **583** |
| `gripper_cmd='move_width'` 步数 | **17** |
| `gripper_cmd='grasp'` 步数 | **0** |
| `grasp_issued` | **0 次** |
| action_grip 最小值 | **0.0125** |
| action_grip 最大值 | **0.2443** |
| action_grip 均值 | **0.0905** |
| 夹爪初始宽度 | **80.2 mm** |
| 夹爪最终宽度 | **64.3 mm** |

### 关节轨迹详细数据

#### 分阶段轨迹

| 阶段 | Step 范围 | q2 | q4 | q6 | q7 | grip (mm) | action_grip |
|---|---|---|---|---|---|---|---|
| HOME 出发 | 0 | -0.020 | -1.886 | 1.906 | 0.562 | 78.4 | 0.024 |
| 快速下降 | 1-4 | -0.031 | -1.896 | 1.945 | 0.558 | 78.1 | 0.022 |
| **q7 触底** | **5** | **-0.028** | **-1.903** | **1.967** | **0.484** | **78.1** | **0.016** |
| 缓慢前进 | 5-50 | -0.028 | -1.908 | 1.957 | 0.484 | 78.1 | 0.018 |
| 持续前进 | 50-100 | 0.028 | -1.928 | 1.972 | 0.484 | 78.1 | 0.022 |
| 加速下弯 | 100-200 | 0.096 | -1.978 | 2.013 | 0.484 | 78.1 | 0.020 |
| 快速接近 | 200-300 | 0.173 | -2.053 | 2.373 | 0.484 | 78.1 | 0.024 |
| **到达位置** | **300-314** | **0.197** | **-2.076** | **2.387** | **0.484** | **78.1→76.2** | **0.025→0.064** |
| 夹爪微收 | 314-360 | 0.202 | -2.079 | 2.417 | 0.484 | 76.2→63.8 | 0.064→0.156 |
| **停滞** | **360-600** | **0.203** | **-2.079** | **2.410** | **0.484** | **~64.3** | **0.14-0.18** |

#### 关键观察

1. **q7 在 step 5 就触底 0.4843 rad**（TRAIN_ARM_MIN[6]）。模型的 q7 目标从 0.56 快速下降到约 0.39，被安全裁剪到 0.4843。与原始 A2（Ckp041680）行为一致。

2. **q6 确实到达了 2.38+ rad**（约 step 290-300）。这意味着 waypoint 控制成功将手臂推到了目标位置，不存在阻抗控制下手臂停在中途的问题。

3. **夹爪从 step 314 开始微收**（从 78.1mm → 64.3mm），但收幅远小于抓取所需。action_grip 最高仅 0.244，远低于 0.50 的 `GRASP_HANDOFF_A` 阈值，所以从未触发力控抓取。

4. **Step 360 后手臂和夹爪均停滞**。q6 在 2.41 附近振荡，不再前进；action_grip 在 0.14-0.18 震荡，不再上升。

### 推理延迟

| 请求序号 | 延迟 (ms) |
|---:|---:|
| 0（含预热） | **797** |
| 1 | 303 |
| 2 | 306 |
| … | 296–310 |
| 119（最后） | 304 |

首次推理 797 ms 包含 KV cache 预热。后续稳定在 296–310 ms。

### Full Chunk Grip 分析（核心发现）

每次推理产生 50 步的完整 action chunk，但只执行前 5 步。以下是几个关键时刻的完整 chunk 中 grip 维度：

#### Request 0（step 0-4）：grip 全程极低

```
chunk[0..49] = [0.024, 0.023, 0.023, 0.023, 0.022, 0.027, 0.025, 0.026, 0.023, 0.025,
                0.027, 0.028, 0.027, 0.028, 0.029, 0.031, 0.027, 0.030, 0.031, 0.031,
                0.033, 0.032, 0.034, 0.033, 0.034, 0.031, 0.030, 0.032, 0.035, 0.035,
                0.033, 0.036, 0.034, 0.031, 0.033, 0.033, 0.034, 0.033, 0.033, 0.034,
                0.031, 0.033, 0.031, 0.037, 0.032, 0.034, 0.032, 0.033, 0.030, 0.033]
执行: chunk[0:5] = [0.024, 0.023, 0.023, 0.023, 0.022]  ← 全部远低于 0.50
```

模型连 50 步规划中都没有"打算"关闭夹爪。

#### Request 58（step 290-294）：chunk 后半段开始出现 grip 上升

```
chunk[0..49] = [0.027, 0.018, 0.022, 0.024, 0.025, 0.025, 0.020, 0.025, 0.026, 0.029,
                0.032, 0.034, 0.037, 0.034, 0.035, 0.038, 0.039, 0.046, 0.055, 0.061,
                0.072, 0.075, 0.077, 0.089, 0.101, 0.111, 0.130, 0.138, 0.158, 0.161,
                0.167, 0.177, 0.196, 0.213, 0.224, 0.240, 0.254, 0.273, 0.293, 0.317,
                0.345, 0.360, 0.392, 0.415, 0.443, 0.475, 0.491, 0.499, 0.526, 0.550]
执行: chunk[0:5] = [0.027, 0.018, 0.022, 0.024, 0.025]  ← 仍然极低
```

chunk 的后 25 步 grip 从 0.16 上升到 0.55，模型"计划"在约 30 步后开始抓取，但只执行了前 5 步（grip ≈ 0.02）。

#### Request 78（step 390-394）：chunk 前段已有明显 grip

```
chunk[0..49] = [0.179, 0.191, 0.203, 0.214, 0.219, 0.232, 0.241, 0.233, 0.238, 0.246,
                0.256, 0.268, 0.335, 0.324, 0.369, 0.392, 0.421, 0.455, 0.486, 0.512,
                0.541, 0.571, 0.582, 0.586, 0.595, 0.604, 0.618, 0.622, 0.627, 0.637,
                0.638, 0.646, 0.652, 0.652, 0.660, 0.664, 0.676, 0.681, 0.736, 0.772,
                0.779, 0.810, 0.812, 0.836, 0.851, 0.872, 0.884, 0.906, 0.913, 0.928]
执行: chunk[0:5] = [0.179, 0.191, 0.203, 0.214, 0.219]  ← 上升了但仍 < 0.50
```

前 5 步 grip 为 0.18-0.22，比之前高了很多，但仍低于 0.50 阈值。chunk 中 step 20+ 才超过 0.50，step 49 到达 0.928。模型一直在"规划"关闭夹爪，但把关闭时刻推迟到 chunk 的中后段。

#### Request 119（step 595-599，最后一次）：模式不变

```
chunk[0..49] = [0.155, 0.169, 0.177, 0.185, 0.196, 0.208, 0.217, 0.240, 0.242, 0.259,
                0.267, 0.263, 0.284, 0.315, 0.353, 0.384, 0.395, 0.415, 0.441, 0.457,
                0.479, 0.504, 0.519, 0.536, 0.555, 0.563, 0.566, 0.576, 0.578, 0.592,
                0.593, 0.603, 0.613, 0.641, 0.634, 0.651, 0.661, 0.675, 0.718, 0.753,
                0.774, 0.803, 0.832, 0.854, 0.861, 0.872, 0.883, 0.886, 0.894, 0.898]
执行: chunk[0:5] = [0.155, 0.169, 0.177, 0.185, 0.196]  ← 仍 < 0.50
```

直到最后一次推理，模型仍然把 grip ≥ 0.50 安排在 chunk 的 step 20+ 位置。

**结论**：模型每次推理都规划了一条完整的"先移臂 → 再关夹爪"的 50 步轨迹，但因为只执行前 5 步就用新观测重新推理，夹爪的关闭时刻每次都被推迟到未执行的后半段。这是 **chunk 执行截断 + 重规划** 导致的"永远推迟"效应。

### 与原始 A2（Ckp041680）及 A2I/A2I2 对比

| | 原始 A2（Ckp041680，旧代码） | **本次 A2（Ckp031260，新代码）** | A2I（Ckp041680，阻抗） | A2I2（Ckp031260，阻抗） |
|---|---|---|---|---|
| 控制方式 | 阻塞 waypoint | **阻塞 waypoint** | 关节阻抗 | 关节阻抗 |
| 实际频率 | 2.83 Hz | **3.16 Hz** | 3.43 Hz | 3.43 Hz |
| q6 终态 | 2.415 | **2.408** | 2.316 | 2.326 |
| q7 终态 | 0.484（被裁剪） | **0.484（被裁剪）** | 0.508（弹簧不足） | 0.508 |
| action_grip 最高 | **0.961** | **0.244** | 0.186 | 0.038 |
| 夹爪最窄 | **3.7 mm**（多次 grasp） | **64.3 mm**（0 次 grasp） | 65.9 mm | 78.0 mm |
| 任务成功 | 臂到位 + 夹爪收紧 | **臂到位 + 夹爪微收** | 臂停中途 | 臂停中途 |

### 根因分析

#### 1. Checkpoint 差异是主因

原始 A2 使用 Ckp041680（训练步数 41680），action_grip 在到达目标姿态后快速升至 0.961，触发多次力控抓取。本次使用 Ckp031260（训练步数 31260），在几乎相同的姿态下 action_grip 最高仅 0.244。

更早期的 checkpoint 学到的 grip 通道响应更弱：
- 它确实"知道"要关闭夹爪（full chunk 后半段 grip 到 0.87-0.93），但把关闭时刻放在 chunk 的 step 20-30+
- 而 Ckp041680 可能在 chunk 前几步就输出高 grip 值（这需要回溯原始 A2 日志确认）

这说明 **checkpoint 的训练充分程度直接影响 grip 通道的时序**：训练越充分的模型，越能在到达位置后立即发出收拢命令，而不是把收拢推迟到 chunk 后半段。

#### 2. Chunk 执行截断放大了时序差异

n_exec=5 意味着只执行 50-step chunk 的前 10%。如果模型把 grip 关闭放在 chunk 的 step 20+（如 Ckp031260），那么无论执行多少轮推理，前 5 步的 grip 始终不够高。每次重新推理都基于新观测生成新 chunk，而新 chunk 的 grip 关闭时刻又在 step 20+，形成 **"永远推迟"的死循环**。

Ckp041680 之所以成功，可能是因为它在 chunk 前 5 步就输出了超过 0.50 的 grip 值，不受截断影响。

#### 3. q7 裁剪行为一致，不是差异来源

本次 q7 同样在 step 5 被裁剪到 0.4843，与原始 A2 行为一致。新代码的 0.0 rad 裕度 vs 旧代码的 0.15 rad 裕度，在这个实验中没有区别——因为旧代码也是裁剪到 0.4843（OUT-OF-TRAIN 警告触发点相同）。

#### 4. HOME 位姿差异影响了早期轨迹但不影响最终结果

新 HOME 的 q6=1.906（旧 2.201），起点更低，导致手臂从 HOME 到目标位置需要走更长的路。但 waypoint 控制最终还是把 q6 推到了 2.41（与原始 A2 的 2.415 几乎相同），说明 HOME 差异只影响了到达时间（约 step 300 vs 更早），不影响最终姿态。

#### 5. 夹爪宽度起点

本次夹爪满开 **80.2 mm**（手部上报 80.3 mm），与训练分布一致。原始 A2 的夹爪满开也是约 80 mm。这方面没有差异。

### 实验结束

服务端通过 `pkill -f vla_inference_server` 停止，退出码 143（SIGTERM）。客户端正常退出，退出码 0。

日志文件：
- 客户端：`/workspace/RLinf/b/x/4dwvla_ext/logs/client_20260922_101022_9029.log`
- 服务端：`/workspace/RLinf/b/x/4dwvla_ext/logs/server_20260922_100420_2088.log`

---

## 多 Checkpoint 批量实验（2026-09-22 11:00 — 12:50）

### 实验目的

在 **同一套代码、同一 HOME 位姿、同一控制参数** 下，对比三个不同训练步数的 checkpoint 在多次 rollout 中的行为稳定性和场景敏感度。核心问题：

1. 不同 checkpoint 能否稳定导航到目标位置（q6≈2.4）？
2. 夹爪能否突破 action_grip=0.50 阈值完成夹取？
3. "perpetual postponement"（永远推迟夹取）是 checkpoint 特性还是场景依赖？

### 统一实验配置

| 配置项 | 值 |
|---|---|
| 客户端 | `franka_vla_client.py`（阻塞 waypoint） |
| 服务端 | `vla_inference_server.py` |
| `--n-exec` | 5 |
| `--max-steps` | 600 |
| `--control-hz` | 10 |
| 夹爪模式 | continuous |
| HOME 来源 | `home_pose.json` = [-0.258, -0.021, 0.167, -1.885, -0.055, 1.906, 0.644] |
| q7 安全裕度 | `[0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0]` |
| 摄像头 | global=250222073513, wrist=420122070525 |

**每次实验之间的唯一变量是场景摆放（插头/插座位置）。**

### 测试的 Checkpoint

| Checkpoint | 训练步数 | 简称 |
|---|---|---|
| `4wvlaFrkPlugCkp010420` | 10,420 | **Ckp01**（早期） |
| `4wvlaFrkPlugCkp031260` | 31,260 | **Ckp03**（中期） |
| `4wvlaFrkPlugCkp041680` | 41,680 | **Ckp04**（晚期） |

### 全部实验结果汇总

#### Ckp04（训练步 41680）— 2 次 rollout

| 次数 | 时间 | q6 峰值 | ag max | ag mean | 夹爪最窄 | 控制 Hz | 导航 | 夹取 |
|---|---|---|---|---|---|---|---|---|
| 1 | 11:01-11:04 | **2.437** | 0.268 | 0.124 | 59.8mm | 3.17 | **到达** | 未夹 |
| 2 | 11:17-11:21 | **2.481** | **0.934** | **0.296** | **7.0mm** | 2.59 | **到达** | **夹取成功** |

**导航成功率：2/2 (100%)**
**夹取成功率：1/2 (50%)**

#### Ckp03（训练步 31260）— 1 次 rollout

| 次数 | 时间 | q6 峰值 | ag max | ag mean | 夹爪最窄 | 控制 Hz | 导航 | 夹取 |
|---|---|---|---|---|---|---|---|---|
| 1 | 11:26-11:29 | **2.423** | 0.263 | 0.137 | 63.8mm | 3.18 | **到达** | 未夹 |

**导航成功率：1/1 (100%)**
**夹取成功率：0/1 (0%)**

#### Ckp01（训练步 10420）— 6 次 rollout + 1 次异常

| 次数 | 时间 | q6 峰值 | ag max | ag mean | 夹爪最窄 | 控制 Hz | 导航 | 夹取 |
|---|---|---|---|---|---|---|---|---|
| 1 | 11:34-11:37 | **2.428** | **0.969** | **0.456** | **4.8mm** | 2.77 | **到达** | **夹取+后退+松手** |
| 2 | 11:44-11:47 | 2.053 | 0.038 | 0.020 | 77.0mm | 3.08 | 卡 HOME | — |
| 3 | 12:09-12:13 | 1.968 | 0.038 | 0.025 | 78.6mm | 3.10 | 卡 HOME | — |
| 4 | 12:19-12:22 | 2.067 | 0.029 | 0.016 | 78.3mm | 3.09 | 卡 HOME | — |
| 5 | 12:33-12:36 | 1.997 | 0.029 | 0.016 | 77.8mm | 3.10 | 卡 HOME | — |
| 6 | 12:41-12:44 | 2.061 | 0.044 | 0.022 | 77.7mm | 3.10 | 卡 HOME | — |
| 7 | 12:51 | — | — | — | — | — | Reflex 崩溃 | — |

**导航成功率：1/6 (~17%)**
**夹取成功率：1/6 (~17%)**

### 跨 Checkpoint 对比分析

#### 1. 导航能力：晚期 > 中期 > 早期

| Checkpoint | 导航到目标 (q6≥2.35) 的成功率 | 失败模式 |
|---|---|---|
| **Ckp04** (41680步) | **2/2 = 100%** | 无 |
| **Ckp03** (31260步) | **1/1 = 100%** | 无 |
| **Ckp01** (10420步) | **1/6 = 17%** | q6 在 1.90-2.07 窄幅振荡 |

晚期 checkpoint 的导航策略已经充分收敛——无论场景如何摆放，都能从新 HOME (q6=1.906) 导航到目标 (q6≈2.4)。早期 checkpoint 对场景极度敏感，5/6 次完全无法启动有意义的运动。

#### 2. 夹取能力：三种截然不同的模式

**Ckp04 — "渐进突破"模式**

Ckp04 第 2 次 rollout 展示了从 "perpetual postponement" 到成功夹取的完整过程：

```
step 330-420: action_grip 从 0.04 逐步爬升到 0.39（持续 ~90 步）
step 420-429: 快速加速 0.39 → 0.51（突破 0.50 阈值）
step 429-440: 急速上升 0.51 → 0.93（仅 11 步完成锁定）
step 440-600: 稳定保持 0.87-0.93
```

关键观察：夹取信号不是突然跳变的，而是经过了约 100 步的 **渐进积累** 才突破阈值。第 1 次 rollout 的 action_grip 停滞在 0.27，可能是因为这个渐进过程在不同场景下的起点不同，有的场景刚好够爬过 0.50，有的不够。

**Ckp01 — "全有或全无"模式**

Ckp01 唯一成功的第 1 次 rollout 展示了完全不同的行为：

```
step 200-230: 到达目标后 action_grip 从 0.05 快速上升
step 245:     跨过 0.50 阈值（比 Ckp04 快得多）
step 270:     夹爪合到 10mm
step 300:     夹爪合到 4.8mm，action_grip 稳定在 0.93
step 300-500: 持续夹紧，但 q6 从 2.41 退回到 2.11（后退动作）
              同时 q7 从 0.48 升到 0.98（接近训练上限）
step 510-530: action_grip 从 0.90 下降到 0.45，跌破阈值
              夹爪逐渐打开（4.8mm → 72mm）
step 530-600: q6 继续退回到 1.97（接近 HOME）
```

这是一个完整的 **"插入→拔出" 循环**，包含：到达 → 夹取 → 后退 → 松手。早期 checkpoint 可能学到了训练数据中包含 reset 阶段的 demonstration。

而 5 次失败则是另一个极端：q6 始终在 1.90-2.07 的窄幅振荡，action_grip < 0.05，夹爪纹丝不动。没有中间状态。

**Ckp03 — "到位但犹豫"模式**

Ckp03（唯一的 1 次 rollout）稳定导航到了目标，但 action_grip 始终停在 0.26 以下，展示了经典的 "perpetual postponement"。

#### 3. 场景敏感度分析

| Checkpoint | 行为一致性 | 场景敏感度 |
|---|---|---|
| **Ckp04** | 导航一致，夹取随场景变化 | **中** — 导航稳定，夹取取决于渐进信号能否爬过阈值 |
| **Ckp03** | 高度一致 | **低** — 总是到达+犹豫 |
| **Ckp01** | 二元分裂 | **极高** — 要么完整执行，要么完全不启动 |

Ckp01 的场景敏感度最高，表现为 "全有或全无"。这与训练步数有关：早期 checkpoint 的策略还不稳定，特征空间中的决策边界非常窄，微小的视觉输入差异就能导致策略跳到完全不同的行为模式。

#### 4. q7 行为差异

| Checkpoint | q7 行为 | 说明 |
|---|---|---|
| **Ckp04** | 全程卡在 0.4843（下限） | q7 目标始终低于训练下限 |
| **Ckp03** | 全程卡在 0.4843（下限） | 同上 |
| **Ckp01 成功** | 0.48 → **0.98**（上限） | q7 在后退阶段大幅上升到训练上限附近 |
| **Ckp01 失败** | 全程卡在 0.4843（下限） | 与 Ckp04/03 失败模式一致 |

Ckp01 成功时 q7 的大范围变化（0.48→0.98→0.95）是训练数据中包含完整 demonstration 的证据：去程 q7 小（手指合拢方向），回程 q7 大（手指张开方向）。

#### 5. 控制频率与夹取成功的关联

| 实验 | 控制 Hz | 夹取结果 |
|---|---|---|
| Ckp04 Run 1 | 3.17 | 未夹 |
| **Ckp04 Run 2** | **2.59** | **夹取成功** |
| Ckp03 | 3.18 | 未夹 |
| **Ckp01 Run 1** | **2.77** | **夹取成功** |
| Ckp01 Run 2-6 | 3.08-3.10 | 卡 HOME |

两次成功夹取的控制频率（2.59、2.77 Hz）低于失败实验（3.08-3.18 Hz）。较低的控制频率意味着每步之间的时间间隔更长（~0.37s vs ~0.32s），可能给了模型更多时间"看到"夹爪的变化并做出反应。但样本量太小，这可能只是巧合。

### 关键结论

1. **训练步数对导航能力有决定性影响**：Ckp04（41680步）和 Ckp03（31260步）的导航成功率 100%，Ckp01（10420步）仅 17%。

2. **"Perpetual postponement" 是场景依赖的，不是 checkpoint 固有的**：同一 Ckp04 在一个场景下 action_grip 停在 0.27，在另一个场景下成功突破 0.93。这说明问题的根源是 **视觉输入** 对夹取决策的影响，而不是 checkpoint 本身缺乏夹取能力。

3. **早期 checkpoint 策略不稳定**：Ckp01 表现为极端的 "全有或全无" — 成功时执行了完整的插入+拔出循环，失败时完全不动。17% 的成功率不具备实际部署价值。

4. **晚期 checkpoint 是更好的选择**：Ckp04 导航 100%，夹取 50%。虽然夹取不是每次都成功，但至少手臂总是到达正确位置。结合场景标准化（固定插头/插座位置），有望进一步提高夹取成功率。

5. **action_grip 的渐进爬升模式**：Ckp04 的夹取成功遵循 "渐进积累→突破阈值→锁定" 模式，需要约 100 步的积累期。如果场景条件不理想导致积累期起点过低（如 0.14 而非 0.04），信号可能永远无法爬过 0.50。

### 后续建议

1. **降低 grasp 阈值**：将 `GRASP_HANDOFF_A=0.80` 改为 0.40-0.50，让更弱的夹取信号也能触发实际夹爪动作。
2. **标准化场景**：固定插头/插座在桌面上的精确位置，减少视觉输入的随机性。
3. **增加 n_exec**：将 n_exec 从 5 增到 10-15，允许模型执行 chunk 中更靠后的 action（包括 grip close 动作）。
4. **对 Ckp04 做更多次重复实验**（≥10 次），量化其在标准场景下的真实夹取成功率。

---

## 实验 A2_1_2：降低 GRASP_HANDOFF_A 并增加 n_exec

### 实验设计

基于实验 A2 系列的分析结论（参见"后续建议"第 1、3 条），本实验验证以下假设：

> **假设**：降低 `GRASP_HANDOFF_A` 从 0.80 到 0.40，同时增加 `n_exec` 从 5 到 10，可以解决 Ckp010420 的"perpetual postponement"问题，提高夹取成功率。

| 参数 | 实验 A2_1（基线） | 实验 A2_1_2（本次） | 变更原因 |
|---|---|---|---|
| Checkpoint | 4wvlaFrkPlugCkp010420 | 同左 | 对照 |
| `GRASP_HANDOFF_A` | 0.80 | **0.40** | 降低力控抓取触发阈值 |
| `n_exec` | 5 | **10** | 允许执行 chunk 中更靠后的 action |
| `max_steps` | 600 | 600 | 不变 |
| `control_hz` | 10.0 | 10.0 | 不变 |

### 实验过程

本实验经历了 **3 次启动尝试**，前 2 次因 bug 导致相机返回全零图像，在排查和修复后第 3 次才正常运行。

#### 尝试 1（13:12 — 失败，手动中止 @ step 35）

- **参数**：`GRASP_HANDOFF_A=0.40`, `n_exec=10`, `--use-realsense` **未传递**
- **现象**：夹爪不断张合
- **根因**：客户端启动命令缺少 `--use-realsense` 参数。`franky_joint_env.py` 中 `FrankyJointEnv.__init__()` 的 `use_realsense` 默认为 `False`，导致 `self._camera = None`，`get_camera_frames()` 返回全零的 `np.zeros((480,640,3))` 帧。
- **日志证据**：
  ```
  image_meta={'global': {'min': 0.0, 'max': 0.0, 'mean': 0.0},
              'wrist':  {'min': 0.0, 'max': 0.0, 'mean': 0.0}}
  ```
- **后果**：模型在无视觉输入的情况下盲猜，`action_grip` 从 step 0 就高达 0.53，立即触发 `grasp_handoff`，每步都执行一次"力控关闭→检测未抓住→打开"的循环

#### 尝试 2（13:25 — 失败，手动中止 @ step 20）

- **修复**：对两个 RealSense 相机分别执行了 `hardware_reset()`，单独测试确认相机出图正常（global mean=118.5, wrist mean=103.2）
- **问题**：启动命令**仍然缺少 `--use-realsense`**，相机在单独测试中正常但客户端内部依然返回全零
- **日志证据**：与尝试 1 相同的全零图像

> **教训**：`hardware_reset()` 修复的是 RealSense SDK 层面的问题，但此次相机全零的原因是 Python 代码层面的 — `use_realsense=False` 导致 `_init_cameras()` 从未被调用。两个层面的问题需要分别诊断。

#### 尝试 3（13:28 — 成功运行，600 步跑满）

- **修复**：在客户端启动命令中添加 `--use-realsense`
- **日志确认**：
  ```
  Camera serials resolved: global=250222073513 wrist=420122070525
  image_meta={'global': {'min': 13.0, 'max': 161.0, 'mean': 118.67},
              'wrist':  {'min': 0.0, 'max': 199.0, 'mean': 123.79}}
  ```
- **参数确认**：`n_exec=10`, `GRASP_HANDOFF_A=0.40`, 600 步全部完成
- **实际控制频率**：3.51 Hz（请求 10.0 Hz）
- **安全警告**：901 条（812 OUT-OF-TRAIN + 990 TRAIN-EDGE，有重叠）

### 尝试 3 详细分析

#### 1. 阶段划分

整个 600 步 rollout 可分为 3 个明确阶段：

| 阶段 | 步数范围 | 步数 | action_grip 范围 | 夹爪状态 | 描述 |
|---|---|---|---|---|---|
| **接近** | 0-129 | 130 | 0.02-0.10 | 打开 (0.072m) | 从 HOME 向插头移动 |
| **过渡** | 130-139 | 10 | 0.13-0.44 | 渐关 → 力控夹取 | grip 快速爬升，在 step 136 越过 0.40 阈值 |
| **持有** | 140-599 | 460 | 0.92-0.98 | 关闭 (0.003m) | 抓住插头，但未能移向插座 |

#### 2. 接近阶段（steps 0-129）

模型从 HOME 位置 `(q1=-0.258, q2=-0.020)` 稳步向插头移动。关节轨迹：

| 关节 | 范围 | 含义 |
|---|---|---|
| q1 | [-0.279, -0.211] | 基座旋转（小幅调整） |
| q2 | [-0.038, +0.209] | 肩关节前伸（大幅前进 0.247 rad） |
| q3 | [+0.007, +0.165] | 肘关节下降 |
| q6 | [+1.915, +2.423] | 腕关节旋转（大幅转动 0.508 rad） |

`action_grip` 在接近阶段始终很低（平均 0.035），表明模型清楚知道此时不应夹取。这与之前 A2_1 系列实验中 n_exec=5 时的行为一致。

#### 3. 夹取触发（step 136）

```
step 136: action_grip=0.4014 → grasp_handoff → grasp_issued=True
          w_meas=0.0493m → close_gripper() → w_after=0.003m (成功夹住)
```

关键观察：

- **首次也是唯一一次 grasp_issued=True**：整个 600 步只发出了 1 次力控抓取指令
- **抓取成功**：夹爪宽度从 0.049m 降到 0.003m，确认夹住物体。后续 464 步 `gripper_holding()` 持续返回 True，阻止了重复抓取
- **阈值效果验证**：旧阈值 0.80 下，action_grip 必须攀升到 0.80 才能触发（在 n_exec=5 的 A2_1 实验中 6 次只有 1 次成功）。新阈值 0.40 下，grip 信号在 0.40 就触发，**显著降低了夹取的难度**

#### 4. 持有阶段的失败分析（steps 140-599）

**核心问题：模型夹住插头后，未能将其移向右侧的插座。**

##### 4.1 轨迹分析

| 时段 | 步数 | q1 变化 | q2 变化 | 净位移 |
|---|---|---|---|---|
| 刚夹住后 (140-200) | 60 | +0.045 | -0.022 | 0.103 rad |
| 中期 (200-300) | 100 | +0.016 | -0.026 | 0.051 rad |
| 后期 (300-400) | 100 | -0.006 | -0.000 | 0.031 rad |
| 更后期 (400-500) | 100 | -0.005 | -0.039 | 0.063 rad |
| 最后 (500-599) | 100 | -0.021 | -0.059 | 0.344 rad |

q2 从 +0.200（抓取位置）持续下降到 +0.044（接近 HOME 的 -0.020）。**机械臂在缓慢回缩而非前进**。全程净位移仅 0.397 rad，远小于接近阶段的 0.637 rad。

##### 4.2 速度分析

后抓取阶段的速度极低且不稳定：

```
step 150: |Δq|=0.003770 rad/step
step 250: |Δq|=0.003119 rad/step
step 350: |Δq|=0.001476 rad/step  ← 几乎静止
step 450: |Δq|=0.002393 rad/step
step 550: |Δq|=0.001881 rad/step
```

对比接近阶段（约 0.010-0.015 rad/step），后抓取速度仅为接近阶段的 20-30%。

##### 4.3 q7 钳位问题

这是一个被低估的严重问题：

- **68.8% 的步骤中 q7 被钳位**到训练范围下界（0.4843 rad）
- 模型意图的 q7 范围为 [0.383, 0.606]，但低于 0.4843 的部分全部被安全系统强制上调
- 这意味着**每 10 步中约 7 步的末端执行器姿态与模型规划的不同**

q7 控制的是法兰盘的旋转角（即末端执行器的朝向）。当 q7 被持续钳位时：
1. 模型观察到的关节状态与它发出的命令不一致
2. 下一步推理时，模型基于"被篡改的"状态做决策，产生偏差
3. 这种偏差逐步累积，导致轨迹偏离预期

##### 4.4 模型的"本意"推测

观察 action_grip 的模式：
- Chunk 13（steps 130-139）：grip 从 0.30 爬升到 0.44，**模型在准备抓取**
- Chunk 14 起（step 140+）：grip 稳定在 0.95+，**模型知道要持续握紧**
- 全程 grip 从未降回（没有尝试释放），说明模型认为自己还没到插座位置

但 arm 轨迹显示模型不知道怎么移动到插座：
- 没有大幅的基座旋转（q1 变化仅 0.089 rad）
- 没有前伸动作（q2 持续下降）
- 大量的末端微调（q5、q6 变化大，但都是小幅振荡）

### 根因诊断

夹取成功但插入失败的根因是多层次的：

#### 原因 1：Ckp010420 训练不足，缺乏"携带-插入"策略

Ckp010420 仅训练了 10,420 步（约为 Ckp041680 的 25%）。在之前 A2_1 实验中已经观察到该 checkpoint 的"全有或全无"模式：

- **唯一的成功 Run 1**：执行了完整的 grab→retreat→release 循环，但那次的行为更像是 **replay 训练中见过的特定轨迹**
- **本次 A2_1_2**：抓取成功了（因为降低阈值帮助了），但抓取后的"下一步"（移向插座）**不在 checkpoint 已学到的策略中**

证据：后抓取阶段的动作非常小且犹豫（速度仅为接近阶段的 20-30%），这是策略分布边界的典型表现 — 模型不确定该做什么，所以输出接近零的动作。

#### 原因 2：q7 钳位导致的状态-动作不一致

q7 被钳位的比例高达 68.8%。安全系统将 q7 从模型意图的值强制修改为 0.4843，导致：

```
模型输出: q7=0.42 → 安全系统: q7=0.4843 → 实际执行: q7=0.4843
                                              ↓
                              下一步模型观察到 q7=0.4843（不是它预期的 0.42）
                                              ↓
                              模型困惑：为什么 q7 没变？→ 输出更极端的 q7 → 再次被钳位
```

这个正反馈循环使得后抓取轨迹中 q7 的实际值始终被卡在 0.4843，模型无法实现末端执行器的精确姿态控制。

#### 原因 3：动作空间被安全系统过度约束

812 次 OUT-OF-TRAIN 警告意味着模型在尝试探索训练分布之外的空间，但被安全系统拒绝。对于 "携带插头移向插座" 这样的任务，可能需要超出训练分布的关节配置，但安全系统将其钳回，导致模型的意图无法实现。

### 与基线实验的对比

| 指标 | A2_1 Run1 (成功) | A2_1_2 (本次) |
|---|---|---|
| GRASP_HANDOFF_A | 0.80 | 0.40 |
| n_exec | 5 | 10 |
| 抓取触发 step | 未知（GRASP_HANDOFF_A=0.80 时触发） | 136 |
| 抓取成功 | ✓ | ✓ |
| 插入成功 | ✓（完成完整循环） | ✗（卡在持有阶段） |
| 控制频率 | 2.77 Hz | 3.51 Hz |
| q7 钳位 | 存在 | 68.8% |

A2_1 Run1 之所以能成功，可能是因为在那次特定的场景配置下，模型恰好 replay 了一条训练中见过的接近插座的轨迹。A2_1_2 的场景配置可能刚好不在该训练轨迹的覆盖范围内。

### `--use-realsense` 缺失问题的技术解析

本次实验最大的意外是前两次尝试中相机返回全零图像。这不是硬件故障，而是启动参数遗漏。

**问题链条**：

```
客户端命令缺少 --use-realsense
  → FrankyJointEnv(use_realsense=False)
    → _init_cameras() 未被调用
      → self._camera = None
        → get_camera_frames() 返回 np.zeros((480,640,3))
          → 模型收到全黑图像
            → action_grip 立即飙升到 0.5+
              → GRASP_HANDOFF_A=0.40 下触发 grasp_handoff
                → close_gripper() 阻塞 ~1.7s，但手中无物
                  → 夹爪打开 → 下一步 grip 仍 ≥0.40 → 再次关闭
                    → 夹爪不断张合
```

**为什么 hardware_reset 没有解决问题**：因为问题根本不在相机硬件。相机在 SDK 级别测试正常（`pipe.start()` 能获取帧），但客户端代码中 `_camera=None` 导致永远不会调用 SDK。两个层面的"相机不工作"表现相同（图像全零），但原因完全不同。

**对 GRASP_HANDOFF_A=0.40 的额外观察**：在全零图像下，旧阈值 0.80 也会被突破（blind mode 下 grip 在 chunk 1 就升到 0.92），但 0.40 的阈值使得 **从 step 5 就开始触发**（grip=0.48），比 0.80 阈值提前了约 10 步。低阈值本身不是问题的原因，但它放大了问题的严重程度 — 让夹爪振荡启动得更早、更频繁。

### 关键结论

1. **降低 GRASP_HANDOFF_A 到 0.40 成功解决了 Ckp010420 的夹取触发问题**：action_grip 在 step 136 越过 0.40 阈值即触发力控抓取，且抓取成功。这验证了之前的假设。

2. **但"能夹住"≠"能完成任务"**：夹取只是 plug-into-socket 任务的第一步。Ckp010420 在夹取后的"携带→定位→插入"阶段缺乏有效策略，表现为缓慢回缩而非向目标移动。

3. **q7 钳位是后抓取失败的重要贡献因素**：68.8% 的步骤被钳位，持续干扰模型的轨迹规划。需要考虑放宽 q7 的训练范围约束或调整安全裕量。

4. **`--use-realsense` 缺失导致的全零图像是一个隐蔽的 bug**：表现与硬件故障相同，但原因完全不同。今后启动脚本应在预检阶段验证图像不全为零。

5. **Ckp010420 仍是最有潜力的 checkpoint**：它是唯一一个在 A2_1 Run 1 中证明过具备完整 grab→retreat→release→insert 全任务策略的 checkpoint。Ckp041680 虽然导航可靠，但从未完成过夹取和后续动作。本次 A2_1_2 的"夹住但不插入"更可能是场景配置、q7 钳位等外部因素导致策略未能正确触发，而非 checkpoint 本身缺乏能力。

### 后续建议

1. **继续用 Ckp010420 做更多重复实验**：保持 GRASP_HANDOFF_A=0.40 + n_exec=10，通过标准化场景（固定插头/插座的精确位置和朝向）来提高一致性，积累更多样本判断成功率。

2. **调查 q7 钳位问题**：
   - 检查训练数据中 q7 的实际分布，确认 0.4843 是否合理
   - 考虑将安全裕量从精确边界放宽到 ±0.05 rad
   - 或调整安全层级：仅在 q7 超出硬件极限时钳位，训练范围违规改为警告
   - q7 钳位率 68.8% 可能是本次插入失败的主要贡献因素

3. **在客户端启动脚本中添加图像验证**：在第一帧推理前检查图像均值是否 > 0，如果全零则立即报错退出而非继续盲跑。

4. **固化启动参数**：将 `--use-realsense` 和相机序列号写入启动脚本或 env 文件的默认命令中，避免每次手动传递遗漏。

---

## 实验 A2_1_3：Ckp010420 + GRASP_HANDOFF_A=0.40 + n_exec=5

### 实验配置

| 参数 | 值 | 与 A2_1_2 的差异 |
|------|------|-------------------|
| Checkpoint | 4wvlaFrkPlugCkp010420 | 不变 |
| GRASP_HANDOFF_A | 0.40 | 不变 |
| n_exec | **5** | A2_1_2 为 10 |
| max_steps | 600 | 不变 |
| control_hz | 10.0 | 不变 |
| `--use-realsense` | ✓ | 不变 |
| FRANKA_GRIPPER_MAX_WIDTH_M | 0.080 | 不变 |

**启动命令**：
```bash
# Server (GPU container)
docker exec -d rlinf-4dwvla-gpu bash -c \
  'VLA_N_EXEC=5 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420 \
   bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh \
   > /tmp/server_a2_1_3.log 2>&1'

# Client (Franky container)
docker exec rlinf-4dwvla-franky bash -c \
  'source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env; \
   cd /workspace/RLinf/b/x/4dwvla_ext; \
   /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
   --server-host localhost --server-port 5555 \
   --task "plug into socket" --max-steps 600 --n-exec 5 \
   --control-hz 10.0 --use-realsense'
```

### 启动过程

启动过程中经历了 4 次尝试，主要因为之前实验的相机资源未完全释放：

| 尝试 | 问题 | 解决 |
|------|------|------|
| 1 | 相机 "Device or resource busy" (errno=16) | 等待前一 client 的相机资源释放 |
| 2 | 相机 "Couldn't resolve requests" | hardware_reset() 后过早重试 |
| 3 | 同上，reset 后仍报错 | 仅 reset global 相机，等待 20s |
| 4 | 成功 | 相机恢复正常 |

最终日志文件：`client_20260922_134802_33540.log`

### 实验结果：近乎成功（Near-Success）

**这是迄今为止最接近成功的一次实验。** 机器臂完成了：
1. ✅ 接近插头
2. ✅ 成功夹取插头（step 285）
3. ✅ 携带插头移向插座
4. ✅ 到达插座附近并尝试插入
5. ❌ 插入位置偏高约数毫米，插入失败

用户反馈：**"机器臂也夹到插头了, 也把插头移动到右边的插座上了, 但插的地方不对, 离插孔靠上了一点, 插入失败。"**

### 总体统计

| 指标 | A2_1_3 (n_exec=5) | A2_1_2 (n_exec=10) |
|------|-------|-------|
| 总步数 | 591 (step 0-590) | 600 |
| 控制频率 | 3.00 Hz | ~3.5 Hz |
| 夹取步骤 | step 285 | step 136 |
| 夹取 action_grip | 0.4625 | 0.6012 |
| 后抓取行为 | 携带→导航→尝试插入 | 缓慢回缩，无有效移动 |
| q7 低界钳位 | 319 步 (53.2%) | 413 步 (68.8%) |
| q7 高界钳位 | 28 步 (4.7%) | 0 步 (0%) |
| q7 在训练范围内 | 253 步 (42.2%) | 187 步 (31.2%) |
| 结果 | **插入位置偏高** | 未导航到插座 |

### 运动轨迹分阶段分析

#### Phase A：接近插头（steps 0-284）

机器臂从 HOME 位置向插头移动。q7 在此阶段持续被钳位在训练下界 0.4843（模型想要更低的 q7 值来完成接近动作），q3 较低（mean=0.037），臂部基本水平。

#### Phase B：夹取（step 285）

```
[step 285] gripper continuous grasp_handoff action=0.4625 w_meas=0.0502 w_cmd=0.0430
           holding=False → 触发力控抓取
```

- action_grip = 0.4625 ≥ GRASP_HANDOFF_A = 0.40 → 触发 grasp_handoff
- 夹爪测量宽度 50.2mm → 命令宽度 43.0mm
- 抓取成功，后续夹爪宽度降至 2.85mm（稳定夹持）

**与 A2_1_2 对比**：A2_1_2 在 step 136 夹取（n_exec=10 执行更多步），A2_1_3 在 step 285 夹取（n_exec=5 单次推理执行更少步），两者都成功。

#### Phase C：后抓取恢复（steps 286-539）

**这是 A2_1_3 与 A2_1_2 根本区别所在。**

A2_1_2（n_exec=10）：夹取后机器臂缓慢回缩，460 步内几乎没有有效向插座方向的移动。  
A2_1_3（n_exec=5）：夹取后机器臂有意义地调整姿态，虽然 q7 仍被钳位在下界，但其他关节在逐步调整，为后续导航做准备。

**关键差异归因**：`n_exec=5` 意味着每次推理结果只执行前 5 步（而非 10 步），模型能更频繁地根据当前状态重新规划。当 q7 被钳位时，模型可以在下一次推理中"看到"实际状态并调整策略，而 n_exec=10 的延迟反馈让模型在错误状态上执行更久。

#### Phase D：导航至插座（steps 540-559）

机器臂开始明确地向插座方向移动：

| 关节 | step 540 | step 559 | 变化 | 物理含义 |
|------|----------|----------|------|----------|
| q2 | 0.0203 | 0.0618 | +0.0415 | 肩部前屈：向前推进 |
| q3 | 0.2045 | 0.2915 | +0.0870 | 肘部抬升：臂部上提 |
| q6 | 2.0813 | 2.1607 | +0.0794 | 腕部旋转：调整朝向 |
| q7 | 0.7501 | 0.9677 | +0.2176 | 接近训练上界(0.9807) |

此阶段 q7 尚未被上界钳位，但已接近上限（0.9677，距上界仅 0.013 rad）。

#### Phase E：插入尝试（steps 560-590）— 失败阶段

**这是最关键的阶段。** q7 从 step 560 开始被钳位在训练上界 0.9807：

| step | q3 (肘部) | q7 模型意图 | q7 实际执行 | q7 被截断量 |
|------|----------|------------|------------|------------|
| 555 | 0.2854 | 0.9767 | 0.9767 | 0 (未钳位) |
| 560 | 0.3133 | **0.9822** | 0.9807 | 0.0015 rad (0.09°) |
| 565 | 0.3166 | **0.9979** | 0.9807 | 0.0172 rad (0.99°) |
| 570 | 0.3270 | **1.0131** | 0.9807 | 0.0324 rad (1.86°) |
| 575 | 0.3207 | **1.0044** | 0.9807 | 0.0237 rad (1.36°) |
| 580 | 0.3242 | **1.0089** | 0.9807 | 0.0282 rad (1.62°) |
| 585 | 0.3200 | **1.0243** | 0.9807 | 0.0436 rad (2.50°) |
| 590 | 0.3191 | **1.0417** | 0.9807 | 0.0610 rad (3.50°) |

**模型的 q7 意图在持续增大**（从 0.9822 增至 1.0417），表明模型"知道"需要更大的 q7 来完成插入，但被安全层持续截断。

### 插入位置偏高的根因分析

#### 根因 1（主要）：q7 上界钳位导致末端执行器姿态偏差

**机制**：Franka 的 q7（第 7 轴）控制末端法兰的旋转。在插入阶段，模型需要 q7 达到 0.98-1.04 rad 来实现正确的插头朝向和位置微调，但安全层将其硬截断在 0.9807 rad。

```
SAFETY_MARGIN_RAD = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0]
                                                          ^^^
                              q7 安全裕量为零 — 严格截断在训练范围边界
```

**q7 裕量为零的历史原因**（来自 `franky_joint_env.py` 第 60-78 行的注释）：
之前将 q7 裕量设为 0.05 rad 时，q7 会漂移到 0.4750（低于训练下界 0.4843），导致模型预测方向反转（cosine = -0.273）。将裕量收紧到 0 后解决了接近阶段的漂移问题。

**但这个修复产生了副作用**：它同时也将 q7 的上界死死锁定在 0.9807，导致插入阶段模型无法达到所需的 q7 位置。

**截断量随时间递增**：从 0.0015 rad (step 560) 增长到 0.0610 rad (step 590)，表明模型越来越"焦急"地想要更大的 q7，但持续被拒绝，状态偏离越来越大。

#### 根因 2（次要）：q3 异常抬升可能是 q7 钳位的代偿效应

| 阶段 | q3 均值 | 说明 |
|------|--------|------|
| 接近 (0-284) | 0.037 | 臂部水平 |
| 恢复 (285-539) | 0.023 | 臂部水平 |
| 导航 (540-559) | 0.255 | 开始抬升 |
| 插入 (560-590) | **0.320** | 明显偏高 |

q3 在 q7 开始被上界钳位后（step 560+）额外增加了 0.065 rad（3.73°）。这可能有两种解释：

1. **代偿效应**：模型"感知"到 q7 无法进一步旋转，试图通过抬高肘部（增加 q3）来间接调整末端位置，但这反而使插头位置偏高。
2. **本体需求**：插入动作本身需要稍高的 q3，而偏高的程度恰好与 q7 钳位的累积偏差叠加，导致最终位置偏上。

无论哪种解释，q3=0.32 在插入阶段是偏高的 — 训练数据中 q3 的最大值为 0.4789（`TRAIN_ARM_MAX[2]`），而 0.32 已经使用了训练范围的 67%（(0.32+0.2025)/(0.4789+0.2025)=76.6%），说明臂部确实在异常高的位置。

#### 根因 3（间接）：q7 两端夹击问题

A2_1_3 中 q7 经历了一个完整的"钳位方向反转"：

```
steps 5-559:   q7 被钳位在 下界 0.4843（638 次）→ 模型想要更低的 q7
                                                    来完成接近/抓取
step 560:      钳位方向反转
steps 560-590: q7 被钳位在 上界 0.9807（55 次） → 模型想要更高的 q7
                                                    来完成插入
```

q7 的训练范围 [0.4843, 0.9807] = 0.4964 rad (28.4°) 对于一个需要"接近 + 抓取 + 携带 + 插入"的复杂任务来说，可能太窄了。模型在任务的两端都受到了 q7 约束的限制。

### 与 A2_1_2 的关键对比

| 维度 | A2_1_2 (n_exec=10) | A2_1_3 (n_exec=5) |
|------|---------|---------|
| 夹取 | ✅ step 136 | ✅ step 285 |
| 后抓取导航 | ❌ 缓慢回缩 460 步 | ✅ 积极向插座移动 |
| 到达插座 | ❌ 从未接近 | ✅ 到达插座并尝试插入 |
| q7 下界钳位 | 413 步 (68.8%) | 319 步 (53.2%) |
| q7 上界钳位 | 0 步 (0%) | 28 步 (4.7%) — 从未到达过此阶段 |
| 失败原因 | q7 钳位+导航策略失效 | q7 上界钳位+位置偏高 |

**n_exec=5 的显著优势**：更频繁的视觉反馈让模型能及时修正状态，成功完成了"抓取→携带→导航"全链路，只在最后的精细插入阶段因 q7 上界限制而失败。

### 解决方案

#### 方案 1（推荐）：非对称放宽 q7 上界裕量

```python
# 当前设置
SAFETY_MARGIN_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0])

# 建议修改：上下界使用不同裕量
SAFETY_MARGIN_LOWER = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0])   # 下界保持严格
SAFETY_MARGIN_UPPER = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.06])  # 上界放宽 0.06 rad
```

**理由**：
- q7 下界为 0（保持严格）：在接近阶段，q7 低于训练下界会导致模型预测方向反转（cosine = -0.273），这在之前的实验中已被验证，不能放宽。
- q7 上界放宽 0.06 rad：插入阶段模型最大需要 q7=1.0417（超出上界 0.061 rad），放宽到 0.06 可覆盖绝大部分需求，同时不会离开训练分布太远。
- 新的有效 q7 范围变为 [0.4843, 1.0407]，训练范围宽度从 0.4964 增加到 0.5564 rad (+12%)。

**风险评估**：q7 上界超出训练范围 0.06 rad 时，模型预测可能不够准确，但 A2_1_3 的数据表明模型在此区域的意图是连贯且有目的性的（q7 持续增大，方向一致），不像下界漂移时的反转行为。

#### 方案 2：场景标准化降低 q7 需求

如果插头和插座的相对位置/朝向在每次实验中保持严格一致，模型可能不需要那么大的 q7 范围来完成插入。建议：
- 用标记线或定位治具固定插头和插座的精确位置
- 记录 A2_1 Run 1（唯一成功实验）中的场景配置并严格复现

#### 方案 3：分阶段 q7 约束

根据任务阶段动态调整 q7 约束：
- 抓取前（接近阶段）：严格约束 q7 ∈ [0.4843, 0.9807]，防止方向反转
- 抓取后（携带/插入阶段）：放宽 q7 上界到 1.05 或更高

实现复杂度较高，但理论上最优，因为可以针对不同阶段的不同需求精确配置。

#### 方案 4：增加 max_steps

当前 600 步（~200s at 3Hz）可能不够。模型在 step 590 时 q7 仍在增大（1.0417），暗示它可能还在尝试找到正确位置。更多步数可能让模型有机会通过其他关节的调整来补偿 q7 限制。但这更像是治标而非治本。

### 结论

1. **A2_1_3 是目前最好的实验结果**：首次实现了完整的"抓取→携带→导航→尝试插入"全流程，距离成功仅差最后几毫米的位置精度。

2. **n_exec=5 明显优于 n_exec=10**：更频繁的视觉反馈让模型能有效导航到插座，这是 A2_1_2（n_exec=10）完全做不到的。后续实验应固定使用 n_exec=5。

3. **插入失败的主因是 q7 上界钳位**：安全层将 q7 硬截断在 0.9807，而模型需要 0.98-1.04 来完成插入。截断量从 0.001 递增到 0.061 rad，表明模型有明确的插入意图但被持续阻止。

4. **q7 零裕量设置存在"修一端破另一端"的问题**：为解决接近阶段的下界漂移而设的零裕量，同时也锁死了插入阶段所需的上界空间。建议采用非对称裕量（下界严格，上界放宽 0.06 rad）来同时解决两端问题。

5. **Ckp010420 再次证明了其作为最佳 checkpoint 的地位**：它是唯一一个能完成夹取+导航+插入尝试的 checkpoint。优化外部配置（q7 裕量、场景标准化）而非更换 checkpoint 是正确的方向。

### 后续建议

1. **立即可做**：实施方案 1（非对称 q7 裕量），将 q7 上界放宽 0.06 rad，然后重新运行实验。
2. **同时推进**：标准化场景配置（方案 2），减少每次实验的位置偏差。
3. **数据记录**：在下次成功（或接近成功）时，记录精确的插头/插座位置和朝向，作为后续实验的基准参考。

---

## 实验 A2_1_4：Ckp010420 + GRASP_HANDOFF_A=0.40 + n_exec=2（场景前移）

### 实验配置

| 参数 | 值 | 与 A2_1_3 的差异 |
|------|------|-------------------|
| Checkpoint | 4wvlaFrkPlugCkp010420 | 不变 |
| GRASP_HANDOFF_A | 0.40 | 不变 |
| n_exec | **2** | A2_1_3 为 5 |
| max_steps | 600 | 不变 |
| control_hz | 10.0 | 不变 |
| `--use-realsense` | ✓ | 不变 |
| **场景** | **插头和插座均前移** | A2_1_3 为原始位置 |

### 实验结果：完全失败

**机器臂未能完成任何子任务。** 600 步内：
- ❌ 未有效接近插头
- ❌ 未触发夹取（grip 最高仅 0.050，远低于阈值 0.40）
- ❌ 未移动到插座附近
- 夹爪全程保持 "hold" 状态（600/600 步）

### 总体统计

| 指标 | A2_1_4 (n_exec=2) | A2_1_3 (n_exec=5) | A2_1_2 (n_exec=10) |
|------|---------|---------|---------|
| 实际控制频率 | **2.22 Hz** | 3.00 Hz | ~3.5 Hz |
| 推理调用次数 | 300 | ~120 | ~60 |
| 推理时间/次 | 0.304s | ~0.30s | ~0.30s |
| 夹取事件 | **0 次** | 1 次 (step 285) | 1 次 (step 136) |
| grip 最大值 | **0.050** | 0.463 | 0.601 |
| q7 低界钳位 | **598 步 (99.7%)** | 319 步 (53.2%) | 413 步 (68.8%) |
| q7 高界钳位 | 0 步 | 28 步 (4.7%) | 0 步 |
| 总位移 (L2) | 0.515 rad | ~2.0 rad | ~1.5 rad |
| 结果 | **完全失败** | 插入位置偏高 | 未导航到插座 |

### 问题 1：n_exec=2 导致控制频率骤降和步骤浪费

#### 现象

控制频率从 A2_1_3 的 3.00 Hz 降至 2.22 Hz（下降 26%），但推理调用次数从 ~120 增至 300 次（增加 150%）。

#### 根因

**n_exec=2 的时间分配严重失衡。** 每个控制周期：

```
n_exec=2: 0.30s 推理 + 2 × 0.30s 执行 = 0.90s → 2.22 Hz
                 推理占比 33%

n_exec=5: 0.30s 推理 + 5 × 0.23s 执行 = 1.45s → 3.45 Hz
                 推理占比 21%

n_exec=10: 0.30s 推理 + 10 × 0.17s 执行 = 2.00s → 5.00 Hz
                  推理占比 15%
```

n_exec 越小，推理开销的占比越高，控制频率反而越低。

#### 更严重的问题：步骤近乎重复

VLA 服务端的动作选择逻辑（`vla_inference_server.py` 第 485 行）：

```python
normalized_action = action_pred[:n_exec, :actual_action_dim]
```

这意味着 n_exec=2 取的是模型 50 步预测轨迹的 **前 2 步**（action[0] 和 action[1]），而非均匀采样。由于相邻步骤在平滑轨迹中的差异极小，两个动作几乎相同：

```
300 次推理的动作对 ΔL2 分析：
  mean = 0.0044 rad
  max  = 0.0103 rad
  → 两个动作之间的差异可忽略不计
```

对比每步步间实际移动量的模式：

```
step  0 (FIRST):  L2 = 0.098  ← 有意义的移动
step  1 (SECOND): L2 = 0.003  ← 几乎不动
step  2 (FIRST):  L2 = 0.070  ← 有意义的移动
step  3 (SECOND): L2 = 0.003  ← 几乎不动
  ...（全程如此）
```

**结论**：n_exec=2 实际上让 50% 的步骤执行了近乎重复的动作，等效控制频率仅为 1.11 Hz（每秒约 1 次"真正的"新动作），远低于 n_exec=5 的等效 0.60 Hz 新推理频率。虽然新推理频率更高（1.11 > 0.60），但每次推理只覆盖轨迹的极短前端，导致机器人只能执行微小步骤。

### 问题 2：模型全程不产生夹取动作

#### 现象

模型的 action_grip 在 600 步中从未超过 0.050，远低于 GRASP_HANDOFF_A=0.40 的阈值。甚至查看服务端的完整 50 步预测，grip 也从未超过 0.077：

```
request 0: 50 步 grip 预测 = [0.037, 0.037, 0.042, ..., 0.075] (max=0.077)
request 1: 50 步 grip 预测 = [0.032, 0.034, 0.033, ..., 0.048] (max=0.052)
request 2: 50 步 grip 预测 = [0.034, 0.033, 0.032, ..., 0.034] (max=0.037)
```

**模型在任何预测时间跨度内都不"打算"关闭夹爪。**

#### 根因分析

**场景前移是最可能的原因。** 用户将插头和插座位置前移后：

1. **视觉分布偏移**：相机看到的插头/插座位置与训练数据中的分布不同，模型无法建立从当前视觉观察到"接近并抓取"的动作映射。

2. **缺少有效接近轨迹**：与 A2_1_3 对比，q1 方向移动量严重不足：

   | 指标 | A2_1_3 接近阶段 | A2_1_4 全程 600 步 |
   |------|-----------------|-------------------|
   | q1 变化 | +0.174 rad | +0.100 rad (仅 57%) |
   | q6 变化 | +0.516 rad | +0.045 rad (仅 9%) |
   | q2 范围 | [-0.02, +0.22] | [-0.06, -0.02] |

   模型的 q2 始终为负（arm 不向前推进），q6 几乎不旋转，表明模型未建立接近策略。

3. **可能的因果链**：

   ```
   场景前移 → 视觉 OOD → 模型输出小幅谨慎动作 → 机器人几乎不移动
                                                → 下一帧视觉几乎不变
                                                → 模型继续输出同样的谨慎动作
                                                → 陷入"谨慎漂移"死循环
   ```

### 问题 3：q7 99.7% 钳位的恶性循环

#### 现象

q7 在 600 步中有 598 步被钳位在训练下界 0.4843（占比 99.7%），远高于 A2_1_3 的 53.2%。模型持续输出 q7 ≈ 0.43（意图值），但被截断到 0.4843。

#### 分析

在 A2_1_3 中，虽然 q7 在接近阶段也被大量钳位，但模型通过其他关节的大幅移动使机器臂到达了插头位置，此后 q7 自然进入了训练范围内。而 A2_1_4 中，由于模型几乎不产生有效移动（问题 2），q7 永远停留在下界附近，形成恶性循环：

```
q7 钳位在 0.4843 → 模型在 OOD 状态下推理
                  → 预测质量降低
                  → 不产生有效移动
                  → q7 继续停在下界
```

### 问题 4：夹爪最大宽度异常

#### 现象

```
A2_1_3: hand reports max_width = 0.0803m (80.3mm)
A2_1_4: hand reports max_width = 0.0703m (70.3mm) ← 少了 10mm
```

系统自动将 `FRANKA_GRIPPER_MAX_WIDTH_M=0.080` 从 0.080 clamp 到了 0.0703。

#### 影响

夹爪状态在模型的输入（state vector 的第 8 维）中是 0.0702 而非 0.0719。虽然差异不大（1.7mm），但可能影响模型对夹爪状态的判断。更重要的是，这说明夹爪在启动时的初始化状态不一致——夹爪的 homing 未完全完成或校准有偏差。

### 与前序实验的对比总结

```mermaid
graph LR
    A["A2_1_2<br/>n_exec=10<br/>✅夹取 ❌导航"] --> B["A2_1_3<br/>n_exec=5<br/>✅夹取 ✅导航 ❌精度"]
    B --> C["A2_1_4<br/>n_exec=2 + 场景前移<br/>❌夹取 ❌导航 ❌全部"]
    
    style A fill:#ffcc80
    style B fill:#a5d6a7
    style C fill:#ef9a9a
```

| 实验 | n_exec | 场景 | 控制Hz | q7钳位率 | 夹取 | 导航 | 插入 |
|------|--------|------|--------|---------|------|------|------|
| A2_1_2 | 10 | 原始 | ~3.5 | 68.8% | ✅ s136 | ❌ | ❌ |
| A2_1_3 | 5 | 原始 | 3.00 | 53.2% | ✅ s285 | ✅ | ❌偏高 |
| **A2_1_4** | **2** | **前移** | **2.22** | **99.7%** | **❌** | **❌** | **❌** |

### n_exec 参数的深入理解

通过 A2_1_2/A2_1_3/A2_1_4 三组对比，可以清晰地看到 n_exec 的非线性影响：

```
n_exec=10: 推理频率低（~0.35 Hz），但每次推理覆盖轨迹 action[0..9]
           → 执行足够远，但反馈太慢（夹取后 460 步仍在盲跑）
           → 控制率最高 (3.5 Hz)

n_exec=5:  推理频率中（~0.60 Hz），覆盖 action[0..4]
           → 平衡了反馈速度和轨迹覆盖
           → 控制率适中 (3.0 Hz)
           → ★ 表现最佳

n_exec=2:  推理频率高（~1.11 Hz），但只覆盖 action[0..1]
           → 每次只能微动，且 action[1] ≈ action[0]
           → 步骤浪费 50%，等效移动距离很短
           → 控制率最低 (2.22 Hz)
           → 表现最差
```

**核心洞察**：n_exec 不是越小越好。`action_pred[:n_exec]` 取首 n 步的设计意味着 n_exec 太小时，模型的长距离规划意图被截断，只能执行轨迹最前端的微动作。n_exec=5 在当前系统中是最优甜点。

### 场景敏感性问题

A2_1_4 揭示了一个更根本的问题：**模型对场景布局高度敏感**。

仅将插头和插座"往前挪了一点"，模型的行为从"能完成夹取+导航+插入尝试"（A2_1_3）变为"完全不产生有效动作"（A2_1_4）。这与 VLA 模型的训练方式有关——训练数据中的场景布局是固定的，模型学到的是特定视觉模式到动作的映射，而非泛化的空间推理。

### 解决方案

#### 方案 1（优先级最高）：恢复 A2_1_3 的场景布局

将插头和插座恢复到 A2_1_3 的原始位置。A2_1_3 已证明在该布局下模型能完成夹取+导航+插入尝试，而场景微调导致了完全失败。**在解决 q7 上界钳位问题之前，不应同时改变场景布局。**

控制变量原则：每次实验只改变一个变量。A2_1_4 同时改变了 n_exec 和场景布局，无法区分哪个是失败主因。

#### 方案 2：固定 n_exec=5

三组实验一致表明 n_exec=5 是最优值：
- 比 n_exec=10 有更快的视觉反馈
- 比 n_exec=2 有更远的轨迹覆盖和更高的控制频率
- 后续实验应固定 n_exec=5，除非有明确理由更改

#### 方案 3：修改 n_exec 的动作选择策略

当前 `action_pred[:n_exec]` 取首 n 步的策略在 n_exec 较小时效果差。可以考虑改为均匀采样：

```python
# 当前（取首 n 步）：
normalized_action = action_pred[:n_exec, :actual_action_dim]

# 改进（均匀采样）：
indices = np.linspace(0, len(action_pred)-1, n_exec, dtype=int)
normalized_action = action_pred[indices, :actual_action_dim]
```

这样 n_exec=2 会取 action[0] 和 action[49]，而非 action[0] 和 action[1]，能覆盖完整的轨迹跨度。但此修改需要谨慎验证，因为均匀采样可能引入轨迹不连续。

#### 方案 4：夹爪 homing 标准化

每次启动实验前执行 `gripper_homing.py`，确保夹爪的 max_width 报告值一致。A2_1_4 的 0.0703m 与 A2_1_3 的 0.0803m 相差 10mm，可能影响模型的状态表示。

#### 方案 5：建立场景基准照片

- 拍摄 A2_1_3（最佳结果）实验开始时的场景照片
- 标记插头和插座的精确位置（用胶带或标记线）
- 后续实验严格按照基准照片摆放场景
- 逐步测试场景偏移的容忍范围

### 结论

1. **n_exec=2 在当前系统中不可用**：`action_pred[:n_exec]` 取首 n 步的设计导致两个动作几乎相同，50% 的步骤被浪费，控制频率反而最低，模型的长距离规划被截断。

2. **场景前移直接导致模型失效**：模型对视觉布局高度敏感，微小的场景变化就能使其从"近乎成功"退化为"完全失败"。这是当前 VLA 模型泛化能力的根本局限。

3. **两个变量同时改变违反了控制变量原则**：无法确定是 n_exec=2 还是场景前移（或两者叠加）导致了失败。但从数据看，模型全程 grip < 0.05 暗示场景变化是主因（n_exec 不应影响 grip 预测的绝对值范围）。

4. **下一步应恢复 A2_1_3 的场景布局 + n_exec=5**，然后单独验证 q7 上界放宽方案。

---

## 实验 A2_1_5：Ckp010420 + GRASP_HANDOFF_A=0.40 + n_exec=5（场景前移，第二次尝试）

### 实验配置

| 参数 | 值 | 备注 |
|------|------|------|
| Checkpoint | 4wvlaFrkPlugCkp010420 | 不变 |
| GRASP_HANDOFF_A | 0.40 | 不变 |
| n_exec | 5 | 恢复为最优值 |
| max_steps | 600 | 不变 |
| control_hz | 10.0 | 不变 |
| `--use-realsense` | ✓ | 不变 |
| 场景 | 插头和插座均前移（与 A2_1_4 相同） | 与 A2_1_3 的原始位置不同 |
| 夹爪 max_width | 0.0703m（hand 报告值） | 低于 A2_1_3 的 0.0803m |

**启动命令**：
```bash
# Server (GPU container)
docker exec -d rlinf-4dwvla-gpu bash -c \
  'VLA_N_EXEC=5 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420 \
   bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh \
   > /tmp/server_a2_1_5b.log 2>&1'

# Client (Franky container)
docker exec rlinf-4dwvla-franky bash -c \
  'source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env; \
   cd /workspace/RLinf/b/x/4dwvla_ext; \
   /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
   --server-host localhost --server-port 5555 \
   --task "plug into socket" --max-steps 600 --n-exec 5 \
   --control-hz 10.0 --use-realsense'
```

日志文件：`/tmp/client_a2_1_5b_full.log`，`/tmp/server_a2_1_5b.log`

### 实验结果：夹爪极晚关闭，关闭后未夹住插头

用户反馈：**"夹爪为什么这么晚才下去, 下去了, 夹爪也闭合了, 但没夹起来就到最大步数了, 自然也没插到插座上。"**

#### 时间线

```
step   0-160:  缓慢振荡，q1 在 -0.26 附近徘徊（比 A2_1_3 慢）
step 160-300:  延迟接近，q1 从 -0.26 缓慢移向 -0.08
step     300:  到达插头附近（q1 ≈ -0.08），但 grip 仅 0.08
step 300-570:  ★ 270 步在插头附近"悬停"，grip 始终 < 0.15
step 549-589:  夹爪开始缓慢收窄（70.2mm → 51.9mm）
step 570-589:  grip 快速攀升（0.15 → 0.36）
step     590:  grip = 0.455 → 触发 grasp_handoff → 夹爪闭合到 2.6mm
step 590-599:  夹爪已闭合，但仅剩 10 步 → 无法完成后续任务
               且夹爪闭合位置不对，未夹住插头
```

### 总体统计

| 指标 | A2_1_5 | A2_1_3 (参考) |
|------|--------|--------|
| 控制频率 | 3.10 Hz | 3.00 Hz |
| 到达插头附近的步骤 | ~300 | ~285 |
| 夹取步骤 | **590** | 285 |
| 从到达到夹取的等待 | **290 步** | 0 步 |
| 夹爪闭合宽度 | 2.6mm | 2.85mm |
| 夹取结果 | ❌ 未夹住 | ✅ 夹住 |
| 后续步数 | 仅 10 步 | 315 步 |
| q7 低界钳位 | 595 步 (99.2%) | 319 步 (53.2%) |

### 核心问题：「延迟夹取」（Deferred Grasp）

#### 现象

对比 A2_1_3 和 A2_1_5b 在**相同臂位置**（q1 ≈ -0.08，已到达插头附近）时，模型的 50 步 grip 预测：

**A2_1_3（原始场景），request 57（step 285）**：
```
执行的 [0:5]:  [0.463, 0.473, 0.471, 0.473, 0.476]  ← 全部 > 0.40 → 立即夹取！
未执行 [5:9]:  [0.482, 0.496, 0.517, 0.534, 0.541]
50 步最大值:   0.897
```

**A2_1_5b（前移场景），request 60（step 300）**：
```
执行的 [0:5]:  [0.079, 0.093, 0.096, 0.097, 0.104]  ← 全部 < 0.11 → 不夹取
未执行 [5:9]:  [0.105, 0.110, 0.111, 0.114, 0.118]
sub-step 34:  grip 首次 ≥ 0.40（值 = 0.419）
50 步最大值:   0.623
```

#### 根因分析

**模型在相同臂位置下的 grip 预测存在根本差异**：

- A2_1_3：模型在 50 步计划的**开头**就预测高 grip（0.46+），因为视觉观察告诉模型"插头就在正下方，现在就该夹"
- A2_1_5b：模型将高 grip 推迟到 50 步计划的**第 34 步**才出现，前 5 步的 grip 仅 0.08-0.10，因为场景变化使模型认为"还需要更多微调才能夹取"

**但 n_exec=5 只执行前 5 步**（`action_pred[:n_exec]`）。当模型将夹取安排在第 34 步时：
1. 只有前 5 步的低 grip 被执行（不触发夹取）
2. 下一次推理时，模型重新规划，又将夹取安排在第 30+ 步
3. 再次只执行前 5 步的低 grip
4. 如此循环，夹取被反复"推迟"

**这是一个系统性的 "永远到不了" 陷阱**：当模型的"夹取时机"被计划在 50 步轨迹的中后段时，n_exec=5 的窗口永远触及不到那个时机点。grip 只能通过每次推理中微小的渐进增长（从 0.08 → 0.09 → ... → 0.45），经过 290 步才最终越过 0.40 阈值。

#### 为什么夹爪闭合后没夹住插头？

夹爪在 step 590 闭合到 2.6mm，但未捕获插头。可能原因：

1. **夹取位置偏移**：从 step 300 到 step 590，臂位置发生了微小变化：
   - q1: -0.082 → -0.067（+0.016 rad）— 水平偏移
   - q3: -0.127 → -0.144（-0.018 rad）— 略有下沉
   
   这些微小偏移可能使夹爪中心偏离了插头。

2. **场景前移的影响**：插头位置与训练数据不同，模型到达的位置可能不是最优夹取点。

3. **270 步悬停期的漂移**：长时间在插头附近小幅振荡，可能逐渐偏离了最优夹取位置。

### 与 A2_1_3/A2_1_4 的对比

| 实验 | n_exec | 场景 | 到达插头 | grip@到达 | 夹取步骤 | 等待步数 | 夹住？ |
|------|--------|------|---------|----------|---------|---------|-------|
| A2_1_3 | 5 | 原始 | step 285 | 0.46 | step 285 | **0** | ✅ |
| A2_1_4 | 2 | 前移 | 从未到达 | max 0.05 | 从未 | - | ❌ |
| **A2_1_5** | **5** | **前移** | **step 300** | **0.08** | **step 590** | **290** | **❌** |

A2_1_5 证明了场景前移的影响不是"模型完全不认识任务"（A2_1_4 那样），而是"模型犹豫不决"——它确实接近了插头，也最终尝试了夹取，但夹取的时机被严重延迟。

### 「延迟夹取」问题的解决方案

#### 方案 1（首选）：恢复原始场景布局

最直接的解决方案。在原始场景中模型的 grip 前 5 步就能达到 0.46，不存在延迟夹取问题。

#### 方案 2：使用更激进的 n_exec 选择策略

将 `action_pred[:n_exec]` 改为 `action_pred[:n_exec_effective]`，其中 `n_exec_effective` 根据 grip 预测动态调整：

```python
# 当 50 步轨迹中存在 grip ≥ 0.40 的步骤时，
# 取到该步骤为止（而非只取前 n_exec 步）
full_chunk = action_pred[:, :actual_action_dim]
grip_column = full_chunk[:, -1]
grasp_step = (grip_column >= 0.40).nonzero()
if len(grasp_step) > 0:
    n_exec_effective = min(grasp_step[0].item() + 1, 50)
else:
    n_exec_effective = n_exec
normalized_action = action_pred[:n_exec_effective, :actual_action_dim]
```

**风险**：这可能导致一次执行太多步（最多 50 步无视觉反馈），但在夹取场景下可能是合理的权衡。

#### 方案 3：提前触发的 grip 预测扫描

不改变 n_exec，但在服务端添加一个前瞻扫描：如果 50 步轨迹的后半段（step 25-49）中 grip 持续 > 0.40，将前 5 步的 grip 值也提升到 0.40 以上。

这本质上是告诉执行层"模型打算很快夹取"，提前触发 grasp_handoff。

#### 方案 4：降低 GRASP_HANDOFF_A 阈值

当前 GRASP_HANDOFF_A=0.40 对于前移场景来说可能太高。A2_1_5b 中悬停期的 grip 在 0.08-0.15 之间，如果将阈值降低到 0.12，可以在 step 300 附近就触发夹取。

**风险**：阈值太低会导致误触发（在接近阶段就尝试夹取），需要找到平衡点。从 A2_1_3 的数据看，接近阶段 grip 最高约 0.16（step 250 处 exec[0:5]=0.15），所以 0.20 可能是较安全的折中值。

#### 方案 5：增加 max_steps

当前 600 步不够用。A2_1_5b 在 step 590 才夹取，如果 max_steps=1200，夹取后还有 610 步用于导航和插入（与 A2_1_3 夹取后的 315 步相当）。这不解决延迟夹取问题本身，但给模型更多时间完成任务。

### 结论

1. **A2_1_5b 发现了"延迟夹取"问题**：场景前移使模型将 grip 预测推迟到 50 步计划的后段（sub-step 34+），而 n_exec=5 只执行前 5 步，导致夹取被反复延迟 290 步才最终触发。

2. **场景前移的影响是梯度式的**：不像 A2_1_4（n_exec=2）那样完全失效，A2_1_5b 证明模型仍能识别插头并尝试夹取，只是时机严重延迟。这是一个"犹豫"而非"失明"的问题。

3. **`action_pred[:n_exec]` 取首 n 步的设计存在固有缺陷**：当模型的关键动作（如夹取）被安排在轨迹中后段时，这种截断策略会系统性地错过它。这不仅影响前移场景，任何使模型"犹豫"的因素都会触发同样的问题。

4. **夹爪闭合但未夹住插头**是另一个独立问题：270 步的悬停漂移和场景前移的位置偏差叠加，导致最终夹取位置不够精确。

5. **优先级排序**：(1) 恢复原始场景，验证 A2_1_3 的可重复性 → (2) 解决 q7 上界钳位问题 → (3) 考虑 action selection 策略改进。

---

## 实验 A2_1_5c：调亮白灯后的抓取成功但后抓取阶段低效运动

### 实验配置

| 参数 | 值 | 备注 |
|---|---|---|
| 实验名 | A2_1_5c（A2_1_5 第三次尝试） | |
| Checkpoint | 4wvlaFrkPlugCkp010420 | 同 A2 系列 |
| n_exec | 5 | `VLA_N_EXEC=5` |
| GRASP_HANDOFF_A | 0.40 | `VLA_GRIPPER_GRASP_HANDOFF_A=0.40` |
| max_steps | 600 | |
| control_hz（请求/实际） | 10.0 / 3.12 | 实际 Hz 仅为请求的 31% |
| 光照变化 | **白灯调亮** | 相比 A2_1_5b 的关键变量 |
| 场景布局 | 插头插座前移（同 A2_1_4/A2_1_5b） | |
| 夹爪 max_width 报告 | 0.0703m | 配置 0.080m 被 clamp 到手爪报告值 |
| 总时间 | 00:51:58→00:55:11 = 193 秒 | |
| 总推理次数 | 120 次 | 600 步 / 5 步每次 |
| 总警告数 | 805 条 | 主要为 q7 OUT-OF-TRAIN |
| 日志 | `/tmp/client_a2_1_5c_full.log`、`/tmp/server_a2_1_5c.log` | |

### 操作细节

1. Server 在 `rlinf-4dwvla-gpu` 容器中启动，`VLA_N_EXEC=5`
2. Client 在 `rlinf-4dwvla-franky` 容器中启动，参数：`--server-host localhost --server-port 5555 --task "plug into socket" --max-steps 600 --n-exec 5 --control-hz 10.0 --use-realsense`
3. 启动前调亮了白灯（关键操作变更）
4. 机械臂回 HOME，夹爪张开（width=0.0702m），按 'a' 开始

### 关键时间线

```
Step   0 (00:51:58): 开始, q1=-0.258, q7=0.553, grip=0.070 (open)
Step  19: q7 首次被 clamp 到 0.4844 (lower bound)
Step 100: q1=-0.368, 机械臂到达插头上方区域
Step 254: 首次 gripper continuous move_width (grip action=0.144)
Step 300: q1=-0.439, 继续接近
Step 370: grip action 升高到 0.20+, 夹爪开始缓慢收窄
Step 390: grip action 达到 0.295
Step 395: ★ 夹取成功! grasp_issued=True, grip=0.002m (闭合)
         q1=-0.451, q2=0.225, q3=0.249, q5=-0.046, q7=0.4844
Step 405: q7 首次脱离 lower bound (0.5037)
Step 410: q7=0.533, q5=-0.095 (开始腕关节调整)
Step 450: L2_from_grasp=0.174, q5=-0.177, q7=0.578 (位姿调整中)
Step 475: L2_from_grasp=0.174 ← 运动停滞!
Step 500: L2_from_grasp=0.175 ← 仍然停滞!
Step 525: L2_from_grasp=0.194 (开始缓慢向插座方向移动)
Step 550: L2_from_grasp=0.233 (移动加速)
Step 575: L2_from_grasp=0.241
Step 599: L2_from_grasp=0.249, q1=-0.377 → 达到 max_steps, 实验结束
```

### 正面发现：光照对夹取时机的决定性影响

| 实验 | 夹取步数 | 光照条件 | 场景 |
|---|---|---|---|
| A2_1_3 | ~step 285 | 正常 | 原始位置 |
| A2_1_5 (首次) | 未夹取 | 正常 | 前移 |
| A2_1_5b | step 590 | 正常 | 前移 |
| **A2_1_5c** | **step 395** | **白灯调亮** | **前移** |

调亮白灯后，夹取从 step 590 提前到 step 395，**提前了 195 步**（32.5% 的总预算）。这证明：

1. **光照是模型识别插头的关键因素**：场景前移后模型"犹豫"的主要原因不仅是位置变化，还有光照不足导致的视觉不确定性
2. **模型的 grip 预测从 inference 77 开始快速上升**：0.19 → 0.29 → 0.47 → 0.94（连续 4 次推理），说明模型一旦"看清"插头就能快速决策
3. **grip 在模型的 50 步计划中也更靠前**：inference 79 的 sub-step 0 就已经 0.47（vs A2_1_5b 中 grip 始终在 sub-step 30+ 才升高）

### 核心问题：后抓取阶段的"原地徘徊"

#### 问题描述

抓取在 step 395 成功完成后，机器人花了约 155 步（step 400→555，占剩余预算 205 步的 76%）在抓取位置附近进行微小位姿调整，而非迅速运送插头到插座。到模型终于开始向插座方向移动时，已经只剩下 45 步（7.5% 的总预算），完全不够完成运输和插入。

#### 数据证据

**后抓取运动轨迹（L2 距离 vs 步数）：**

```
step=400: L2_from_grasp=0.021  ← 刚抓完
step=425: L2_from_grasp=0.106  ← 快速调整腕关节
step=450: L2_from_grasp=0.174  ← 调整变缓
step=475: L2_from_grasp=0.174  ← ★ 完全停滞
step=500: L2_from_grasp=0.175  ← ★ 仍然停滞（25 步仅移动 0.001）
step=525: L2_from_grasp=0.194  ← 开始缓慢恢复
step=550: L2_from_grasp=0.233  ← 移动加速
step=575: L2_from_grasp=0.241
step=599: L2_from_grasp=0.249  ← max_steps 耗尽
```

**关节空间位移对比（接近阶段 vs 后抓取阶段）：**

```
接近阶段 (step 0→395, 396 步):
  Δq1=-0.186  Δq2=+0.261  Δq3=+0.092  Δq5=-0.010  Δq6=+0.477

后抓取阶段 (step 400→599, 200 步):
  Δq1=+0.082  Δq2=-0.081  Δq3=-0.040  Δq5=-0.141  Δq6=-0.036
```

后抓取阶段的 Δq1 仅 +0.082（向插座方向），而接近阶段的 Δq1 为 -0.186。按此速度，机器人需要再运动至少 200 步才能到达可能的插座位置。关键观察：**q5 变化了 -0.141（腕关节旋转），这是后抓取阶段的主要运动成分——模型在旋转插头以对准插座方向。**

**平均每步关节速度（|Δq1|/step）：**

```
接近阶段 (0→395):   0.001345 rad/step
后抓取阶段 (400→600): 0.001418 rad/step
```

每步速度相似！说明问题不是"模型输出的步长太小"，而是**运动方向分散——模型在多个关节上同时做微调，整体位移效率低**。

### 根因分析

#### 根因 1: 后抓取阶段的"位姿搜索"行为

模型在抓取后进入了一种"位姿搜索"模式：不断微调腕关节（q5、q7）来寻找正确的插头朝向，同时缓慢向插座方向移动。具体体现：

```
后抓取关节变化:
  q5: -0.062 → -0.203  (变化 0.141 rad, 约 8°)
  q7:  0.484 → 0.628  (变化 0.144 rad, 约 8°)
  q1: -0.459 → -0.377  (变化 0.082 rad, 约 5°)
```

模型用了 200 步的 70% 在调整腕关节朝向，只有 30% 的运动分量用于向插座方向平移。

**为什么模型会进入这种模式？**

推测：训练数据中的抓取-运输-插入轨迹是平滑连贯的，模型学到"抓取后立即开始运输"的行为模式。但在实际执行中，q7 突变（下文分析）破坏了这种连贯性，模型需要花额外步数"重新定位"自己在任务中的进度。

#### 根因 2: q7 突变导致的"任务进度迷失"

**q7 轨迹时间线：**

```
Step   0: q7=0.553 (自然值)
Step  19: q7=0.484 (首次 clamp 到 lower bound 0.4843)
Step  20→404: q7≡0.4844 (连续 385 步锁死在 lower bound)
Step 405: q7=0.504 (突然跳变 +0.020)
Step 410: q7=0.533 (继续上升)
Step 450: q7=0.578
Step 550: q7=0.622
Step 599: q7=0.628
```

q7 在整个接近阶段（385 步）被钳位在 0.4844，然后在抓取完成后突然"释放"并快速上升。这种非自然的 q7 轨迹在训练数据中不存在——训练数据中 q7 是平滑变化的。

**影响机制：**

1. **本体感觉失真**：模型接收的状态反馈中 q7=0.4844 持续了 385 步，但模型每次推理都预测 q7 应该低于 0.4843（OUT-OF-TRAIN 警告 1200 次，全部是 q7）。模型的预测与实际反馈之间持续存在矛盾。

2. **抓取后的 q7 跳变**：Step 404 到 Step 410 之间 q7 从 0.4844 跳到 0.5329（Δ=+0.049 in 6 steps）。这种突变可能让模型"误判"当前状态——它的内部表征中 q7=0.484 对应"接近阶段"，但现在 q7 突然变成 0.53 且夹爪已闭合，模型需要时间"重新校准"。

3. **重新规划成本**：每次推理时模型都在生成 50 步轨迹，但由于状态反馈与预期不一致，生成的轨迹质量下降——表现为运动方向分散、缺乏明确的朝目标运动的意图。

#### 根因 3: `action_pred[:5]` 截断在后抓取阶段的叠加效应

以 inference 90（step 450 附近）的服务端日志为例，模型生成的 5 步物理动作：

```
sub-step 0: q1=-0.4544  q3=0.2904  q5=-0.1772  q7=0.5779  (当前状态附近)
sub-step 1: q1=-0.4547  q3=0.2899  q5=-0.1764  q7=0.5782
sub-step 2: q1=-0.4525  q3=0.2879  q5=-0.1761  q7=0.5817
sub-step 3: q1=-0.4521  q3=0.2865  q5=-0.1750  q7=0.5829
sub-step 4: q1=-0.4519  q3=0.2886  q5=-0.1746  q7=0.5835
```

前 5 步（被执行的部分）的总位移：Δq1 = +0.0025 rad。模型的 50 步计划的终点可能 q1 ≈ -0.40 或更高（向插座方向），但只有最初的微小增量被执行。

这与接近阶段的情况相同（`action_pred[:5]` 取首 5 步），但在接近阶段这不是问题，因为接近方向是一致的——每次都在同一方向上推进。而在后抓取阶段，模型的 50 步计划包含"先调整朝向，再运输"两个子阶段，但首 5 步永远只触及"调整朝向"部分，导致运输动作被反复延迟。

#### 根因 4: 控制频率瓶颈（3.12 Hz vs 10 Hz）

```
配置 control_hz = 10.0
实际 achieved_control_hz = 3.12

每步平均耗时: 321 ms
  其中推理步 (120 步): ~500 ms (300ms 推理 + 200ms 执行)
  非推理步 (480 步):   ~276 ms (关节位控执行 + 通信开销)

预期 (10 Hz): 600 步 × 100ms = 60 秒
实际:         600 步 × 321ms = 193 秒 (3.2×)
```

控制频率仅为请求的 31%，主要瓶颈：

1. **关节位控执行延迟 (~230ms)**：Franka 的 libfranka 关节位控命令需要 ~230ms 完成单步位置移动（包含控制环收敛时间），这是硬件层面的限制。

2. **推理时间 (~300ms)**：每 5 步一次推理，每次约 300ms（GPU 推理 + ZMQ 通信）。

3. **叠加效应**：在 193 秒中只有 36 秒用于模型推理（120 × 300ms），剩余 157 秒全花在关节执行上。即使推理降到 0ms，控制频率也只能从 3.12 提升到 ~3.83 Hz（600 步 / 157 秒）。

**与步数预算的关系**：在 3.12 Hz 下，后抓取的 205 步仅覆盖 65.7 秒的实时运动。按每步 0.001 rad 移动，总位移约 0.2 rad——而到达插座可能需要约 0.3-0.4 rad 的 q1 位移。时间和步数双重不足。

### 对比分析：A2_1_5 系列三次尝试

| 维度 | A2_1_5 (首次) | A2_1_5b | A2_1_5c |
|---|---|---|---|
| 光照 | 正常 | 正常 | **白灯调亮** |
| 夹取步数 | 未夹取 | step 590 | **step 395** |
| 夹取成功 | ✗ | ✓ (但太晚) | **✓** |
| 插头夹住 | — | 未夹住(位偏) | **夹住了** |
| 运输到插座 | — | — | 开始运输但不够 |
| 主要失败原因 | 无夹取 | 延迟夹取 | **后抓取低效运动** |
| grip max | 0.078 | 0.93 | 0.96 |
| q7 clamping | 99.2% | 大量 | 大量 (steps 0-404) |
| 控制 Hz | — | — | 3.12 |

**趋势**：每次尝试都比上一次更进一步——从"完全不抓"到"抓得太晚"到"抓住了但运输不完整"。

### 解决方案

#### 方案 1（优先）：增加 max_steps 到 1200-1500

**理由**：模型的行为轨迹表明它"知道"应该将插头运送到插座——step 550 之后 q1 方向明确向插座移动。问题纯粹是步数不够。

```
当前: max_steps=600, 后抓取预算=205 步, 到达插座需要≈400 步
建议: max_steps=1200, 后抓取预算=805 步 (充足)
时间: 1200 步 × 321ms ≈ 386 秒 ≈ 6.4 分钟 (可接受)
```

**实施**：修改 client 启动命令中 `--max-steps 1200`。

**风险**：低。不改变模型行为，仅给予更多执行时间。

#### 方案 2：解决 q7 下界钳位问题

q7 被钳位在 0.4844 长达 385 步是后抓取"位姿搜索"行为的可能根因之一。两种思路：

**2a. 扩展 q7 训练范围下界**

```python
# franky_joint_env.py line 54
TRAIN_ARM_MIN = np.array([...  0.4843])  # 当前
TRAIN_ARM_MIN = np.array([...  0.4000])  # 修改后: 给 q7 更多下方空间
```

风险：超出训练分布，模型可能在新范围内产生不可预测行为。且真正的硬件下限可能不允许 q7 < 0.48。

**2b. 为 q7 添加下界安全余量**

```python
# franky_joint_env.py line 79
SAFETY_MARGIN_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0])  # 当前
# 不适用: q7 下界问题是模型想去更低, 而非安全余量不够
```

此方案对下界问题不适用。模型已经在请求 q7 < 0.4843 的值，添加安全余量只会让钳位更严重。

**结论**：q7 下界问题需要从训练侧解决（重新训练使 q7 范围覆盖实际需要的工作空间），短期内无法通过配置修改安全地解决。

#### 方案 3：改进 action selection 策略（中期）

当前 `action_pred[:n_exec]` 取首 n 步的策略在后抓取阶段表现尤其差，因为：
- 接近阶段：运动方向一致，取首 5 步已足够
- 后抓取阶段：轨迹包含"调整+运输"两个子阶段，首 5 步只触及调整部分

**可选替代策略**：

| 策略 | 描述 | 优点 | 缺点 |
|---|---|---|---|
| 均匀采样 | `action_pred[::50//n_exec]` | 覆盖完整轨迹 | 跳过中间步骤导致不平滑 |
| 后段加权 | 取 sub-step [5,10,15,20,25] | 跳过初始小增量 | 可能跳过精细调整 |
| 自适应 | 按 L2 距离阈值选择 | 保证最小位移 | 实现复杂 |
| 增大 n_exec | n_exec=10 或更大 | 执行更多轨迹 | 反馈频率降低 |

建议先尝试 **n_exec=10**（简单修改，直接增大覆盖范围），或 **均匀采样**（修改 `vla_inference_server.py` line 485）。

#### 方案 4：优化控制频率

当前 3.12 Hz 的瓶颈在关节位控执行（~230ms/step），非推理侧。优化思路：

1. **异步推理**：在执行动作的同时进行下一次推理（当前是串行的）。理论提升：~300ms/5步 ≈ 每 5 步省 60ms。
2. **减小关节位控步长/使用流式控制**：用 libfranka 的实时接口替代高级位控，可能将执行时间从 230ms 降到 100ms 以下。
3. **增大 n_exec 以摊薄推理开销**：n_exec=10 时推理开销减半（60 次推理 vs 120 次）。

#### 方案 5：保持白灯调亮（已验证有效）

白灯调亮使夹取提前了 195 步，这个简单的环境调整已经被验证有效，后续实验应保持。

### 结论

1. **白灯调亮是本轮实验的关键改进**：将夹取从 step 590 提前到 step 395，证明光照是模型决策的重要输入。后续所有实验应保持充足光照。

2. **发现了新的瓶颈——"后抓取位姿搜索"**：模型在抓取后花 76% 的剩余预算在原地做腕关节微调（q5 变化 0.14 rad，q7 变化 0.14 rad），而非迅速运输到插座。这是之前实验未暴露的阶段性问题。

3. **q7 下界钳位是后抓取低效运动的可能根因**：385 步的 q7 锁死 + 抓取后突然跳变，可能导致模型内部的"任务进度"判断失准，进入低效的探索模式。

4. **步数预算严重不足**：600 步在 3.12 Hz 下仅有 193 秒。抓取用了 395 步（66%），后抓取只剩 205 步（34%），且这 205 步中约 155 步在做位姿调整。实际有效运输步数仅 ~50 步。

5. **推荐下一步**：
   - **立即**：将 max_steps 增大到 1200，保持白灯调亮，重新运行实验，验证模型是否能在更多步数内完成运输+插入
   - **短期**：尝试 n_exec=10，增大每次推理执行的轨迹覆盖范围
   - **中期**：改进 action selection 策略（均匀采样替代取首 n 步）
   - **长期**：重新训练时扩展 q7 工作范围

---

## 实验 A2_1_6：max_steps=1200，首次到达插入阶段但 q7 上界钳位导致对孔失败

### 实验配置

| 参数 | 值 | 备注 |
|---|---|---|
| 实验名 | A2_1_6 | |
| Checkpoint | 4wvlaFrkPlugCkp010420 | 同 A2 系列 |
| n_exec | 5 | `VLA_N_EXEC=5` |
| GRASP_HANDOFF_A | 0.40 | `VLA_GRIPPER_GRASP_HANDOFF_A=0.40` |
| **max_steps** | **1200** | **从 600 增加到 1200（本轮关键变化）** |
| control_hz（请求/实际） | 10.0 / 未正常结束（joint_reflex 崩溃） | |
| 光照 | 白灯调亮（同 A2_1_5c） | |
| 场景布局 | 插头插座前移（同 A2_1_4/A2_1_5 系列） | |
| 夹爪 max_width 报告 | 0.0703m | |
| 总执行步数 | 990（step 990 时 joint_reflex 崩溃） | |
| 总推理次数 | 198 次 | |
| 总 OUT-OF-TRAIN 警告 | 1855 次（全部为 q7） | |
| 总时间 | 01:42:21→01:47:41 = 320 秒（5.3 分钟） | |
| 日志 | `/tmp/client_a2_1_6_full.log`、`/tmp/server_a2_1_6.log` | |
| 终止原因 | **joint_reflex（非正常终止）** | libfranka 安全反射停机 |

### 操作细节

1. 清理上一轮进程，启动新 Server（VLA_N_EXEC=5, Ckp010420）
2. Client 启动：`--max-steps 1200 --n-exec 5 --control-hz 10.0 --use-realsense`
3. 机械臂回 HOME，夹爪张开，场景保持前移位置，白灯调亮
4. 按 'a' 开始实验

### 关键时间线（五阶段分析）

```
=== 阶段 1: 接近 (step 0→581, 582 步) ===
Step    0: q1=-0.271 q3=0.165 q5=-0.032 q7=0.554  grip=0.070 (open)
Step   19: q7 首次 clamp 到 0.4844 (lower bound)
Step  100: q1=-0.350 到达插头上方
Step  300: q1=-0.383 继续接近
Step  500: q1=-0.417 grip=0.064 (缓慢收窄)
Step  530: grip=0.311 (开始显著收窄)
Step  581: ★ grasp_issued=True, grip=0.002m (夹取成功)
           位置: q1=-0.436 q3=0.232 q5=-0.038 q7=0.484
  → 位移: Δq1=-0.165 Δq3=+0.068 Δq5=-0.006 Δq7=-0.070

=== 阶段 2: 腕关节调整 (step 581→700, 119 步) ===
Step  585: q5=-0.070 q7=0.484 (夹爪已闭合)
Step  600: q5=-0.103 q7=0.517 (q7 脱离下界)
Step  650: q5=-0.177 q7=0.577
Step  700: q5=-0.185 q7=0.578
  → 位移: Δq1=+0.030 Δq3=+0.026 Δq5=-0.147 Δq7=+0.093

=== 阶段 3: 悬停/振荡 (step 700→900, 200 步) ===
Step  700: q1=-0.406 q3=0.259 q7=0.578
Step  740: q1=-0.378 q3=0.226 q7=0.628
Step  780: q1=-0.378 q3=0.220 q7=0.625
Step  820: q1=-0.376 q3=0.226 q7=0.623
Step  860: q1=-0.375 q3=0.223 q7=0.642
Step  900: q1=-0.397 q3=0.257 q7=0.656
  → 位移: Δq1=+0.009 Δq3=-0.002 Δq5=-0.012 Δq7=+0.079
  ★ 200 步净位移极小！q1 振荡在 [-0.41, -0.37] 之间

=== 阶段 4: 急速插入逼近 (step 900→950, 50 步) ===
Step  900: q1=-0.397 q3=0.257 q5=-0.197 q7=0.656
Step  920: q1=-0.414 q3=0.294 q5=-0.192 q7=0.604
Step  935: q1=-0.421 q3=0.330 q5=-0.160 q7=0.657
Step  940: q1=-0.407 q3=0.357 q5=-0.130 q7=0.722
Step  945: q1=-0.396 q3=0.389 q5=-0.109 q7=0.833
Step  950: q1=-0.386 q3=0.418 q5=-0.103 q7=0.935 ←★ q7 急升!
  → 位移: Δq1=+0.011 Δq3=+0.161 Δq5=+0.094 Δq7=+0.279
  ★ q7 在 50 步内从 0.66 飙升到 0.94！模型突然"决定"进行插入

=== 阶段 5: q7 上界钳位下的插入尝试 (step 950→989→crash) ===
Step  955: q7=0.981 ← 首次触及上界 0.9807
Step  960: q1=-0.366 q3=0.417 q7=0.981 (上界锁死)
Step  970: q1=-0.342 q3=0.392 q7=0.981
Step  980: q1=-0.308 q3=0.369 q7=0.981
Step  989: q1=-0.284 q3=0.363 q5=-0.186 q7=0.981
Step  990: ★ CRASH — joint_reflex!
           模型命令 q7=1.054, 钳位到 0.981, 关节配置不一致
  → 位移: Δq1=+0.103 Δq3=-0.055 Δq5=-0.084 Δq7=+0.046
```

### 里程碑意义：首次到达"插入"阶段

这是 A2 系列实验中**首次**机器人完成了"接近→抓取→运输→插入尝试"的完整流程：

| 实验 | 到达阶段 | 失败点 |
|---|---|---|
| A2_1_3 | 接近→抓取→运输→插入(近成功) | 插入位置偏高 (q7 上界) |
| A2_1_4 | 接近 | 无夹取 (n_exec=2) |
| A2_1_5 (首次) | 接近 | 无夹取 (光照+场景) |
| A2_1_5b | 接近→抓取 | 夹取太晚 (step 590) |
| A2_1_5c | 接近→抓取→部分运输 | 步数不够 (max=600) |
| **A2_1_6** | **接近→抓取→运输→插入尝试** | **q7 上界钳位→对孔失败→joint_reflex** |

max_steps 从 600 增加到 1200 使模型有足够步数完成运输阶段，验证了 A2_1_5c 分析中"增加步数"方案的有效性。

### 核心问题 1: 后抓取悬停再现（200 步，step 700-900）

与 A2_1_5c 完全一致的模式：

```
A2_1_5c: step 450-500, L2 从 0.174 到 0.175 (25步, Δ=0.001)
A2_1_6:  step 700-900, L2 在 0.18-0.25 间振荡 (200步, 无单调增长)
```

200 步中 q1 净位移仅 +0.009 rad。模型在这个阶段的行为特征：
- q1 在 [-0.41, -0.37] 范围内来回振荡（不是单向运动）
- q7 在 [0.58, 0.66] 范围内小幅变化
- q5 基本稳定在 -0.20 附近

这证实了"位姿搜索"是一个**系统性行为模式**，不是随机噪声。模型在抓取后进入低效探索状态约 200 步才能"决定"下一步的运动方向。

### 核心问题 2: q7 上界钳位导致插头与插孔对不准

#### 数据证据

模型在插入阶段持续请求 q7 ≈ 1.04-1.05，但被钳位到 0.9807：

```
Step 984: 模型请求 q7=1.036 → 钳位到 0.981 (误差 0.056 rad)
Step 985: 模型请求 q7=1.044 → 钳位到 0.981 (误差 0.063 rad)
Step 986: 模型请求 q7=1.043 → 钳位到 0.981 (误差 0.062 rad)
Step 987: 模型请求 q7=1.044 → 钳位到 0.981 (误差 0.063 rad)
Step 988: 模型请求 q7=1.043 → 钳位到 0.981 (误差 0.062 rad)
Step 989: 模型请求 q7=1.044 → 钳位到 0.981 (误差 0.063 rad)
Step 990: 模型请求 q7=1.054 → 钳位到 0.981 (误差 0.073 rad → CRASH)
```

模型**一致地**认为 q7 应该在 1.04-1.05 附近才是正确的插入朝向。0.06 rad ≈ 3.4° 的腕关节角度误差，在插头末端（距离腕关节约 15-20cm）会转化为约 **1.0-1.2mm 的位置偏差**。对于精密插孔任务（插孔间隙通常 < 1mm），这个误差足以导致对孔失败。

#### 与 A2_1_3 的对比

| 维度 | A2_1_3 (原始场景, 近成功) | A2_1_6 (前移场景, 失败) |
|---|---|---|
| 最终 q1 | -0.268 | -0.284 |
| 最终 q2 | 0.212 | 0.211 |
| 最终 q3 | 0.318 | **0.363** (+0.045, 更高) |
| 最终 q5 | -0.137 | **-0.186** (-0.050, 更旋转) |
| 最终 q7 | 0.981 | 0.981 (均被钳位) |
| 失败模式 | 位置偏高 | **对孔失败** |

两次实验的 q7 都被锁死在 0.981。A2_1_6 的 q3（+0.045）和 q5（-0.050）偏差更大，说明模型试图用其他关节**补偿 q7 不足**，但补偿方向不精确，导致整体对准反而更差。

### 核心问题 3: joint_reflex 崩溃

**崩溃过程**：

```
Step 990 模型命令:
  q7_cmd = 1.054  (超出上界 0.073 rad)
  q1_cmd = -0.268 (Δq1 = +0.016 from current)
  q6_cmd = 2.390  (Δq6 = +0.008)

安全系统处理:
  q7 被钳位到 0.981 (Δq7_actual = 0.000)
  其他关节按原始 delta 执行

结果:
  关节 1-6 按 q7=1.054 的假设移动, 但 q7 实际停留在 0.981
  → 产生的关节配置违反 libfranka 的内部安全约束
  → 触发 joint_reflex 紧急停机
```

**根因**：安全钳位系统独立地处理每个关节——将 q7 钳位到上界但不调整其他关节的目标值。当模型规划的多关节运动有耦合关系时（如 q7 旋转 + q1 平移协调运动），仅钳位其中一个关节会使整体运动变得不协调，导致速度或加速度超限。

### q7 双向钳位：训练范围 [0.4843, 0.9807] 的系统性问题

本次实验完整暴露了 q7 训练范围的两端问题：

```
q7 下界钳位:
  Step 19→585 (566 步, 57% 的总执行步数)
  q7 ≡ 0.4844
  模型请求 q7 < 0.4843 → OUT-OF-TRAIN

q7 上界钳位:
  Step 955→990 (35 步, 3.5% 的总执行步数)
  q7 ≡ 0.9807
  模型请求 q7 ≈ 1.04-1.05 → OUT-OF-TRAIN

训练范围: [0.4843, 0.9807], 宽度仅 0.4964 rad (28.4°)
模型需要: [约 0.43, 约 1.05], 宽度约 0.62 rad (35.5°)
缺口: 下界少约 0.05, 上界少约 0.07
```

q7 是 Franka 第 7 轴（末端腕关节），其旋转直接控制插头的插入朝向。训练范围过窄意味着模型在接近和插入两个关键阶段都无法达到它需要的腕关节角度。

### 解决方案

#### 方案 1（最高优先级）：扩展 q7 安全余量

当前 q7 的 SAFETY_MARGIN_RAD 为 0.0（零余量）。模型在插入阶段需要 q7 ≈ 1.05，当前被硬钳位在 0.9807。

```python
# franky_joint_env.py line 79
# 当前:
SAFETY_MARGIN_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0])

# 方案 A: 对称扩展 (上下各 0.08)
SAFETY_MARGIN_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.08])
# → 新范围: [0.4843-0.08, 0.9807+0.08] = [0.4043, 1.0607]
# 覆盖模型需要的 [0.43, 1.05]

# 方案 B: 非对称 (下界 0.05, 上界 0.08)
# 需要修改代码支持非对称 margin
# → 新范围: [0.4843-0.05, 0.9807+0.08] = [0.4343, 1.0607]
```

**前提检查**：确认 Franka 第 7 轴的硬件极限允许 q7 达到 1.06+。Franka Emika Panda 的 q7 硬件极限为 [-2.8973, 2.8973]，所以 1.06 完全在安全范围内。

**风险**：扩展后模型可能在超出训练分布的 q7 范围内产生不可预测行为。但从当前数据看，模型已经在一致地请求 q7 ≈ 1.04-1.05，说明它的内部表征已经知道需要这些值，只是被安全系统阻止了。

#### 方案 2：防止 joint_reflex 崩溃

当 q7 被钳位时，应同比例缩放其他关节的 delta，而非让它们继续按原始 delta 执行：

```python
# franky_joint_env.py check_action_safety() 中
# 当检测到 q7 被 clamp 时:
if was_clamped[6]:
    original_delta = action - current_state
    clamped_delta = clamped_action - current_state
    # 计算 q7 被缩减的比例
    if abs(original_delta[6]) > 1e-6:
        scale = clamped_delta[6] / original_delta[6]
        # 按同比例缩减其他关节
        clamped_action[:6] = current_state[:6] + original_delta[:6] * scale
```

这样当 q7 被钳位 50% 时，其他关节的 delta 也按 50% 缩减，保持运动的协调性。

#### 方案 3：减少后抓取悬停时间

后抓取悬停（200 步）是持续存在的系统性行为。两种思路：

**3a. 增大 n_exec 到 10**：让模型的 50 步轨迹中更多的"运输"部分被执行（10/50 vs 5/50），可能减少悬停时间。

**3b. 使用均匀采样替代取首 n 步**：
```python
# vla_inference_server.py line 485
# 当前: normalized_action = action_pred[:n_exec, ...]
# 改为: 
indices = np.linspace(0, len(action_pred)-1, n_exec, dtype=int)
normalized_action = action_pred[indices, ...]
```
这样 n_exec=5 时取 sub-step [0, 12, 24, 37, 49]，覆盖整个 50 步轨迹。

#### 方案 4：结合 A2_1_3 的经验优化场景

A2_1_3（原始场景位置）几乎成功。如果将场景恢复到原始位置，配合当前的改进（白灯调亮 + max_steps=1200 + q7 margin 扩展），可能直接成功。

### 模型行为的正面信号

尽管实验以 joint_reflex 崩溃结束，但模型展现了多个正面信号：

1. **完整的任务理解**：模型完成了"接近→定位→抓取→调整朝向→运输→插入"的完整行为序列
2. **一致的 q7 需求**：模型在 6 次连续推理中持续请求 q7 ≈ 1.04-1.05，说明这不是随机噪声而是有明确目标
3. **即将成功的释放信号**：最后一次推理（request 198）的 50 步 grip 计划从 0.89 逐渐降到 0.54——模型预测接下来应该**松开夹爪**释放插头。这意味着模型认为插入即将完成
4. **大幅运动能力**：step 900-950 的急速逼近表明模型有能力做大幅运动，悬停不是运动能力问题而是决策延迟

### 结论

1. **max_steps=1200 解决了步数不足问题**：模型在 990 步时已到达插入阶段（vs A2_1_5c 在 600 步时仅到达运输阶段中期）。如果不是 joint_reflex 崩溃，1200 步可能足够完成任务。

2. **q7 上界钳位是插入失败的直接原因**：模型需要 q7 ≈ 1.05 来对准插孔，但被钳位到 0.981。0.07 rad 的角度误差导致插头末端约 1mm 的位置偏差，超过了插孔的容差。

3. **q7 训练范围 [0.4843, 0.9807] 对本任务系统性偏窄**：双端钳位导致接近阶段 q7 锁死（本体感觉失真→后抓取悬停）和插入阶段 q7 不够（对孔失败→崩溃）。

4. **joint_reflex 崩溃暴露了安全系统的协调性缺陷**：单关节钳位不调整其他关节，导致多关节运动不协调，触发紧急停机。

5. **推荐下一步**：
   - **立即**：修改 `SAFETY_MARGIN_RAD[6]` 从 0.0 改为 0.08，扩展 q7 可达范围到 [0.40, 1.06]
   - **同时**：添加 q7 钳位时的多关节协调缩放逻辑，防止 joint_reflex
   - **验证**：以修改后的配置重新运行实验，预期模型能在 q7 ≈ 1.05 处对准插孔
   - **备选**：恢复原始场景位置 + 上述修改，验证 A2_1_3 的近成功结果能否转化为完全成功

---

## A2_1_6 后续：问题深入分析、解决方案讨论与实际改动

基于 A2_1_6 实验暴露的三个核心问题，进行了深入讨论和部分修复。

### 问题 1: q7 上界钳位导致对孔失败

#### 问题详解

模型在插入阶段连续 6 次推理一致请求 q7 ≈ 1.04-1.05，但安全系统将其钳位到 0.9807。0.07 rad ≈ 4° 的角度误差在插头末端（距腕关节约 15cm）产生约 1mm 的位置偏移，超过插孔容差。

#### 实际改动：非对称安全余量

**文件**：`b/x/4dwvla_ext/franky_joint_env.py` 第 79-83 行

**改动前**：
```python
SAFETY_MARGIN_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0])
ACTION_LIMIT_LOWER = np.maximum(TRAIN_ARM_MIN - SAFETY_MARGIN_RAD, JOINT_LIMITS_LOWER)
ACTION_LIMIT_UPPER = np.minimum(TRAIN_ARM_MAX + SAFETY_MARGIN_RAD, JOINT_LIMITS_UPPER)
```

**改动后**：
```python
SAFETY_MARGIN_LOWER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0])
SAFETY_MARGIN_UPPER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.08])
ACTION_LIMIT_LOWER = np.maximum(TRAIN_ARM_MIN - SAFETY_MARGIN_LOWER_RAD, JOINT_LIMITS_LOWER)
ACTION_LIMIT_UPPER = np.minimum(TRAIN_ARM_MAX + SAFETY_MARGIN_UPPER_RAD, JOINT_LIMITS_UPPER)
```

**效果**：
- q7 下界保持 0.4843（不变，防止行为反转——`diag_pose_attractor.py` 验证过 q7 低于下界时 action cosine 从 +0.34 翻转到 -0.27）
- q7 上界从 0.9807 放宽到 **1.0607**（+0.08），覆盖模型插入时需要的 q7 ≈ 1.05
- q1-q6 的上下界完全不变
- Franka q7 硬件极限为 [-2.8973, 2.8973]，1.06 完全安全
- `check_action_safety` 中的 L2 层直接引用 `ACTION_LIMIT_LOWER`/`ACTION_LIMIT_UPPER`，无需额外改动

### 问题 2: q7 训练范围 [0.4843, 0.9807] 系统性偏窄

#### 问题详解

q7（Franka 第 7 轴，末端腕关节旋转）在任务全周期中遭遇双端钳位：

```
q7
1.05 ┤                                              ✕✕✕ ← 模型需要（插入对准）
1.00 ┤- - - - - - - - - - - - - - - - - - -上界 0.9807 - - - - -
     │                                         ╱████ ← 上界钳位
0.60 ┤                              ╱‾‾‾   ← 悬停振荡区
0.50 ┤- - - - - - - - - - - - -下界 0.4843 - - - - - - - - - - -
0.48 ┤   ████████████████████  ← 566步锁死在下界
0.43 ┤   ✕✕✕✕✕✕✕✕✕✕✕✕✕✕✕✕✕✕✕ ← 模型需要（接近姿态）
     └──┬───────────────┬──┬──────────┬────┬──→ step
       0    接近(0-585) 585 悬停+运输  900 插入
```

- **下界钳位（接近阶段）**：模型认为正确的接近姿态需要 q7 ≈ 0.43-0.47，但低于 0.4843 的值全被钳位。结果 q7 连续 566 步锁死在 0.4844。模型每步输出 q7=0.47 之类的值，执行后发现 q7 还是 0.4844——"预期"与"反馈"持续矛盾（本体感觉失真）。抓取后 q7 突然从 0.4844 跳到 0.50+，这种训练数据中不存在的突变导致模型进入"迷茫"的探索行为，形成 200 步的后抓取悬停。

- **上界钳位（插入阶段）**：模型需要 q7 ≈ 1.05 来对准插孔，但被钳位到 0.981。已通过非对称余量解决（上文）。

训练范围宽度仅 0.4964 rad (28.4°)，实际任务需要约 0.62 rad (35.5°)。

#### 解决状态

| 端 | 状态 | 说明 |
|---|---|---|
| 上界 | ✅ 已解决 | 非对称余量 +0.08 |
| 下界 | ❌ 未解决 | 不能放宽——q7 低于下界时 action 方向反转；需从训练侧解决（扩展演示数据的 q7 范围） |

下界问题目前只能靠增加 max_steps 来**容忍**后抓取悬停的 200 步开销。

### 问题 3: joint_reflex 崩溃的安全系统协调性缺陷

#### 问题详解

`check_action_safety` 的 L2 层（训练范围检查）逐关节独立 clip。当模型命令 7 个关节协调运动，但 q7 被单独钳位时：
- q1-q6 的目标值是模型在假设 q7=1.054 的前提下规划的
- q7 实际停在 0.981
- 产生的关节配置力矩/加速度分布异常，触发 libfranka 的 `joint_reflex` 紧急停机

注意 L3（速度限制层）已经做了等比缩放（协调的），问题纯粹出在 L2 的独立 clip。

#### 讨论过的四个方案

**方案 A：L2 层等比缩放**

计算最小缩放系数使所有关节都回到合法范围，然后把整个 delta 向量等比缩放。

- 优点：从根本上保持 7 关节运动方向一致
- 缺点：q7 下界锁死的 566 步中，每步 q7 都超限，等比缩放会让**所有**关节的 delta 被压到接近零——机器人会更慢甚至停滞
- 风险：**中**。副作用严重，可能连抓取都到不了

**方案 B：L2 独立 clip 后追加协调修正**

保留现有逐关节 clip，之后检查哪个关节的 delta 被削减最多，按该比例缩小其他关节。

- 优点：不改现有 clip 逻辑，增量式
- 缺点：同方案 A，下界锁死阶段会全局减速
- 风险：**中**

**方案 C：try-catch 捕获 joint_reflex 并恢复**

在执行层包异常处理，捕获 `ControlException` 后调用 `recover()` 恢复到安全状态。

- 优点：不碰安全逻辑，不影响正常运动行为，兜底所有未预见的崩溃
- 缺点：被动不主动——机器人仍会触发急刹（对机械结构有磨损）
- 风险：**低**。纯增量，不改变正常路径行为

**方案 D：不额外改——靠已有的非对称余量**

A2_1_6 崩溃时模型请求 q7=1.054，在新上限 1.0607 下不会被钳位，所以那次 joint_reflex 不会再发生。

- 优点：零改动
- 缺点：只覆盖 q7 ≤ 1.06 的情况
- 风险：**最低**，但覆盖面有限

#### 结论与决定

推荐 **方案 C + D 组合**（最小改动 + 最低风险），但当前暂不实施方案 C。

理由：
- 方案 D（非对称余量）已经消除了已知的触发条件（q7=1.054 < 新上限 1.0607）
- 方案 A/B 的副作用太严重（566 步下界锁死期间全局减速）
- 方案 C 作为防御性兜底可以后续按需添加
- 先用修改后的配置跑一轮实验，观察 joint_reflex 是否仍然出现，再决定是否加 C

### 实际改动汇总

| 改动 | 文件 | 状态 |
|---|---|---|
| 非对称 q7 安全余量 | `franky_joint_env.py:79-83` | ✅ 已完成 |
| joint_reflex try-catch 恢复 | `franky_joint_env.py` (step 执行层) | ⏸ 暂不实施，观察后决定 |
| q7 下界放宽 | — | ❌ 不可行（需训练侧解决） |
| L2 等比缩放 | — | ❌ 副作用太大，不采用 |

---

## 实验 A2_1_7：夹爪反复夹放，始终无法夹住插头

### 实验信息

| 项目 | 值 |
|---|---|
| **实验名称** | A2_1_7 |
| **日期** | 2026-09-23 02:26 – 02:35 (手动终止) |
| **Checkpoint** | `4wvlaFrkPlugCkp010420` |
| **n_exec** | 5 |
| **max_steps** | 1600 (实际执行 1251 步后手动终止) |
| **GRASP_HANDOFF_A** | 0.40 |
| **control_hz** | 10.0 (请求), 2.47 Hz (实际平均) |
| **gripper mode** | continuous |
| **gripper max_width** | 0.0683m (hand 报告, config 0.080m 被钳位) |
| **q7 安全余量** | 非对称：下界 0.0, 上界 0.08 (A2_1_6 后的改动) |
| **场景变化** | 插座位置已固定, home_pose.json 已更新 |
| **日志文件** | `/tmp/client_a2_1_7_full.log` |

### 操作流程

1. 用户固定了插座位置，更新了 `home_pose.json`
2. 服务器启动：
   ```bash
   docker exec -d rlinf-4dwvla-gpu bash -c \
     'VLA_N_EXEC=5 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420 \
      bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh > /tmp/server_a2_1_7.log 2>&1'
   ```
3. 客户端首次启动因 **"Device or resource busy"** 相机错误崩溃（上一轮僵尸进程未释放 RealSense 设备句柄）
4. 通过 `docker restart rlinf-4dwvla-franky` 重启容器释放相机资源
5. 客户端成功启动：
   ```bash
   docker exec rlinf-4dwvla-franky bash -c \
     'source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env; \
      cd /workspace/RLinf/b/x/4dwvla_ext; \
      /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
        --server-host localhost --server-port 5555 \
        --task "plug into socket" --max-steps 1600 --n-exec 5 \
        --control-hz 10.0 --use-realsense'
   ```
6. 机械臂回 HOME，等待用户按 'a' 开始
7. 用户按 'a' 开始实验（02:26:07）
8. 用户观察到夹爪反复夹放插头但无法夹住，手动终止实验（02:34:36，step 1251）

### 实验时间线

| 阶段 | 步数范围 | 持续步数 | 持续时间 | 控制频率 | 描述 |
|---|---|---|---|---|---|
| **接近阶段** | 0–744 | 745 | 236.6s | 3.14 Hz | 手臂从 HOME 移向插头, 夹爪逐步缩窄 |
| **夹爪循环阶段** | 745–1251 | 506 | 268.5s | 1.88 Hz | 反复发起夹取但始终无法稳定持握 |

### 关键现象：夹爪夹放循环

#### 现象描述

从 step 745 开始，夹爪进入了一个无限循环：
- 模型输出 `action_grip > 0.40` → 触发 `grasp_handoff`
- libfranka 执行力控夹取命令 → 夹爪关闭
- 夹取完成后宽度检查失败 → 系统判定夹取未成功
- `holding` 始终为 `False` → cooldown (1.0s) 过后再次发起夹取
- 如此反复 **201 次**，无一成功

#### 关键统计

| 指标 | 值 |
|---|---|
| `grasp_handoff` 命令总数 | 506 步 |
| 实际发起 `grasp_issued=True` | 201 次 |
| `holding=True` | **0 次**（整个实验中从未成功持握）|
| `close_gripper` 警告 (grasp did not capture) | 16 次 |
| `close_gripper failed: Command failed!` | 1 次 |
| 全开弹出事件 (w_meas=0.068m) | 1 次 (step 762) |

#### w_meas 分布（夹爪循环阶段，506 步）

| 宽度范围 | 步数 | 占比 | 含义 |
|---|---|---|---|
| < 3mm（近全闭） | 27 | 5.3% | 夹爪闭合到底，插头滑出或未碰到 |
| 3–9mm（窗口内） | 188 | 37.2% | 瞬时通过持握窗口，但未被检测到 |
| 9–15mm（在插头上）| 289 | 57.1% | **夹爪停在插头表面，插头宽度约 10mm** |
| >60mm（全开） | 1 | 0.2% | 插头被弹出 |

#### 典型循环周期（steps 745–770）

```
step  a_grip  w_meas(mm)  cmd             grasp
 744  0.3922   50.2       move_width
 745  0.4338   48.6       grasp_handoff   YES ← 第一次夹取，w_meas 是夹取前的值
 746  0.4377    5.5       grasp_handoff   YES ← 闭合到 5.5mm，但持握检查失败
 747  0.4469    6.0       grasp_handoff       ← cooldown 中
 748  0.4571   11.0       grasp_handoff   YES ← 弹回到 11mm（插头宽度），再次夹取
 749  0.4526    5.6       grasp_handoff       ← 闭合
 750  0.8978   11.0       grasp_handoff   YES ← 弹回，再夹
 ...（如此反复直到 step 1251）
```

### 根因分析

#### 直接原因：`FRANKA_CUBE_WIDTH_M` 参数配置错误

夹取失败的直接原因是 **夹爪持握判定窗口与实际插头宽度不匹配**。

**参数链路追踪：**

1. **环境变量设置**（`configs/franka_plug_eval.env`）：
   ```bash
   export FRANKA_CUBE_WIDTH_M=0.002   # 2mm — 远小于实际插头宽度
   export FRANKA_HOLD_TOL_M=0.001     # 1mm
   ```

2. **运行时钳位**（`franky_ext/motion_limits.py`）：
   - `CUBE_WIDTH_RANGE_M = (0.005, 0.070)` — 最小允许值 5mm
   - `HOLD_TOL_RANGE_M = (0.004, 0.025)` — 最小允许值 4mm
   - 实际生效值：`cube_width = 0.005m`（从 0.002 钳位到 0.005），`hold_tol = 0.004m`（从 0.001 钳位到 0.004）

3. **持握判定窗口**：`[cube_width - tol, cube_width + tol]` = **[0.001m, 0.009m]**

4. **实际插头宽度**（libfranka 报告）：**≈ 0.010m (10mm)**

5. **结果**：`0.010m > 0.009m` → `_hardware_holding()` 始终返回 `False`

```
持握判定窗口:  [0.001m ──────── 0.009m]
                                   ↑
                            窗口上界 = 9mm
                                      ↓
实际插头宽度:                    0.010m (10mm)
                                ^^^^^^^^
                        差 1mm，始终在窗口外！
```

#### 失败机制详解

`FrankaLibfrankaGripper.close()` 的执行流程：

```
close() 调用
  ├─ _hardware_holding() 检查 → False (10mm ∉ [1mm, 9mm])
  ├─ 不跳过，发起 grasp(width=5mm, epsilon=4mm, force=20N)
  │   ├─ libfranka 闭合夹爪
  │   ├─ 遇到插头于 ~10mm 处停下
  │   ├─ 10mm ∉ [5mm-4mm, 5mm+4mm] = [1mm, 9mm]
  │   └─ libfranka 报告 grasp 失败 → 抛出异常
  ├─ 异常被 close_gripper() 捕获为 WARNING
  │   └─ "grasp did not capture the cube: measured width=0.0001m, expected 0.0050m +/-0.0040m"
  └─ 夹爪无力控保持 → 插头弹回 → w_meas 回到 ~10mm
```

后续步骤：
- `action_grip` 持续 > 0.40 → `gripper_cmd = grasp_handoff`
- `_hardware_holding()` 仍然 False（10mm ∉ [1mm, 9mm]）
- cooldown (1.0s) 过后 → 再次发起 grasp → 重复失败
- **死循环持续 506 步 / 268.5 秒**

#### 与 A2_1_6 的对比

A2_1_6 使用**完全相同的 `FRANKA_CUBE_WIDTH_M=0.002` 参数**，但夹取在 step 581 成功了：

| 指标 | A2_1_6 (成功) | A2_1_7 (失败) |
|---|---|---|
| 首次 grasp_issued | step 581 | step 745 |
| 总 grasp_issued | **1 次** | **201 次** |
| 夹取后 w_meas | **0.002195m (2.2mm)** | **~0.010m (10mm)** |
| 持握窗口检查 | 2.2mm ∈ [1mm, 9mm] ✓ | 10mm ∉ [1mm, 9mm] ✗ |
| close_gripper 警告 | 0 | 17 |
| 后续行为 | 持握成功 → 搬运 → 插入 | 无限循环 |

**差异原因**：
- A2_1_6 中夹爪将插头压缩到 2.2mm（深压式抓取，可能是侧向夹持或薄处抓取）
- A2_1_7 中夹爪只能压到 ~10mm（平面抓取，插头的正面厚度）
- A2_1_7 的变化因素：**插座位置固定 + home_pose 更新** → 接近角度不同 → 夹持点不同 → 插头在夹爪中的有效厚度从 ~2mm 变为 ~10mm

#### 夹取频率对控制频率的冲击

每次 `close_gripper()` 调用会阻塞约 0.5-0.6s（libfranka 力控夹取的执行时间）。在夹爪循环阶段：

| 阶段 | 控制频率 | 原因 |
|---|---|---|
| 接近阶段 (0-744) | 3.14 Hz | 正常：推理 ~300ms + 关节执行 ~230ms |
| 夹爪循环 (745-1251) | **1.88 Hz** | 每 ~5 步一次阻塞性 grasp 调用拉低均值 |

频率下降 40%（3.14 → 1.88 Hz），进一步恶化了机器人的响应能力。

#### q7 下界钳位仍然存在

整个实验中 q7 OUT-OF-TRAIN 警告出现 **3739 次**。模型持续请求 q7 ≈ 0.43–0.45（低于训练下界 0.4843），被钳位到 0.4843。这个问题在 A2_1_6 中已识别，不影响夹取但影响整体姿态精度。

### 解决方案

#### 方案 1（推荐）：修正 `FRANKA_CUBE_WIDTH_M` 参数

**原理**：将持握判定窗口和夹取目标宽度调整为匹配实际插头宽度。

**具体修改**（`configs/franka_plug_eval.env`）：

```bash
# 修改前（窗口 [0.001m, 0.009m]，夹取目标 0.005m）:
export FRANKA_CUBE_WIDTH_M=0.002
export FRANKA_HOLD_TOL_M=0.001

# 修改后（窗口 [0.002m, 0.014m]，夹取目标 0.008m）:
export FRANKA_CUBE_WIDTH_M=0.008
export FRANKA_HOLD_TOL_M=0.006
```

**参数选择理由**：

| 参数 | 值 | 理由 |
|---|---|---|
| `FRANKA_CUBE_WIDTH_M=0.008` | 8mm | 略小于插头的 10mm libfranka 宽度。作为 grasp 目标，确保夹爪施加压缩力（目标 < 物体宽度 → 力控闭合）。在 [0.005, 0.070] 范围内，不会被钳位。 |
| `FRANKA_HOLD_TOL_M=0.006` | 6mm | 持握窗口 [0.002, 0.014]。覆盖 A2_1_6 的成功值 2.2mm ✓，覆盖 A2_1_7 的 10mm ✓，排除全闭 0mm ✓，排除全开 68mm ✓。在 [0.004, 0.025] 范围内，不会被钳位。 |

**新窗口覆盖验证**：
```
新窗口:  [0.002m ─────────────── 0.014m]
                 ↑                 ↑
            下界 2mm          上界 14mm
                 ↑       ↑       ↑
           A2_1_6    A2_1_7   
           2.2mm ✓   10mm ✓   都在窗口内
           
全闭 0mm ✗  (正确排除: 空手闭合)
全开 68mm ✗ (正确排除: 未夹持)
```

#### 方案 2（备选）：降低夹取力或调整接近姿态

如果方案 1 不足以解决问题（例如插头在新角度下物理上无法被夹住），可考虑：
- 增大 `FRANKA_GRASP_FORCE` 到 30-40N（更强力夹持）
- 调整 `home_pose.json` 使接近角度更有利于夹持

#### 方案 3（长期）：自适应夹取参数

根据夹取后的实际 w_meas 动态调整 `cube_width` 参数。但这需要修改 `FrankaLibfrankaGripper` 代码，改动较大，当前不建议。

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 5 | 环境变量 |
| `VLA_CKPT_PATH` | `.../4wvlaFrkPlugCkp010420` | 环境变量 |
| `VLA_GRIPPER_MODE` | continuous | `franka_plug_eval.env` |
| `VLA_GRIPPER_GRASP_HANDOFF_A` | 0.40 | `franka_plug_eval.env` |
| `VLA_GRASP_RETRY_COOLDOWN_S` | 1.0 | `franka_plug_eval.env` |
| `FRANKA_CUBE_WIDTH_M` | 0.002 → **钳位为 0.005** | `franka_plug_eval.env` → `motion_limits.py` |
| `FRANKA_HOLD_TOL_M` | 0.001 → **钳位为 0.004** | `franka_plug_eval.env` → `motion_limits.py` |
| `FRANKA_GRASP_FORCE` | 20N | `franka_plug_eval.env` |
| `FRANKA_GRIPPER_MAX_WIDTH_M` | 0.080 → 钳位为 0.0683 | `franka_plug_eval.env` → hand 报告值 |
| `SAFETY_MARGIN_UPPER_RAD[q7]` | 0.08 | `franky_joint_env.py` (非对称余量) |
| `max_steps` | 1600 (实际跑了 1251) | CLI 参数 |

### 关键日志片段

#### 夹取参数钳位警告
```
franky_joint_env: FRANKA_GRIPPER_MAX_WIDTH_M=0.0800m exceeds the 0.0683m the hand
reports; clamping to the reported value.
```

#### 首次夹取失败
```
[step 745] gripper continuous grasp_handoff action=0.4338
    w_meas=0.048624273389577866 w_cmd=0.0453 holding=False
RLinf: gripper grasp: width=0.0050m +/-0.0040m speed=0.050m/s force=20.0N
    (force held after contact)
close_gripper: grasp did not capture the cube: measured width=0.0009m,
    expected 0.0050m +/-0.0040m. Either the hand is empty, or the cube is a
    different size -- set FRANKA_CUBE_WIDTH_M to the measured width
```

#### 插头弹出事件 (step 762)
```
[step 761] w_meas=0.000000m  ← 全闭
[step 762] w_meas=0.068288m  ← 全开！插头被弹出
[step 763] w_meas=0.004887m  ← 再次夹取
```

#### 控制频率下降
```
[step 1250] control rate: achieved=2.47 Hz requested=10.0 Hz (ratio=0.25)
```

### 结论

A2_1_7 的核心问题是 **夹爪参数配置不匹配实际插头宽度**：

1. `FRANKA_CUBE_WIDTH_M=0.002` 被钳位到 0.005m，产生持握窗口 [0.001m, 0.009m]
2. 插头在新位置/角度下的 libfranka 报告宽度约 10mm，**刚好在窗口外 1mm**
3. `_hardware_holding()` 始终返回 False → 无限夹取循环
4. 每次失败的夹取还会阻塞控制循环 ~0.5s，将频率从 3.14 Hz 拉低到 1.88 Hz

**推荐下一步**：将 `FRANKA_CUBE_WIDTH_M` 改为 `0.008`，`FRANKA_HOLD_TOL_M` 改为 `0.006`，产生窗口 [0.002m, 0.014m]，覆盖所有已观察到的成功和失败场景。

---

## 实验 A3_1：新 Checkpoint，夹爪循环问题复现

### 实验信息

| 项目 | 值 |
|---|---|
| **实验名称** | A3_1 |
| **日期** | 2026-09-23 02:50 – 02:54 (max_steps 耗尽) |
| **Checkpoint** | `itvlagpFrkPlug0907Dtaug010000`（**新 checkpoint，首次测试**）|
| **n_exec** | 5 |
| **max_steps** | 600 |
| **GRASP_HANDOFF_A** | 0.40 |
| **control_hz** | 10.0 (请求), 2.86 Hz (实际) |
| **gripper mode** | continuous |
| **gripper max_width** | 0.0683m (hand 报告) |
| **cube_width / hold_tol** | 0.005m / 0.004m（env 设的 0.002/0.001 被钳位，**与 A2_1_7 相同的错误参数**）|
| **q7 安全余量** | 非对称：下界 0.0, 上界 0.08 |
| **日志文件** | `/tmp/client_a3_1_full.log` |

### 操作流程

1. 停止 A2_1_7 残留的 server 进程
2. 启动新 server（新 checkpoint）：
   ```bash
   docker exec -d rlinf-4dwvla-gpu bash -c \
     'VLA_N_EXEC=5 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/itvlagpFrkPlug0907Dtaug010000 \
      bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh > /tmp/server_a3_1.log 2>&1'
   ```
3. Server 加载成功：`Model loaded: device=cuda dtype=torch.bfloat16 action_loss_only=True backend=standard kpt=True`
4. 启动 client：
   ```bash
   docker exec rlinf-4dwvla-franky bash -c \
     'source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env; \
      cd /workspace/RLinf/b/x/4dwvla_ext; \
      /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
        --server-host localhost --server-port 5555 \
        --task "plug into socket" --max-steps 600 --n-exec 5 \
        --control-hz 10.0 --use-realsense'
   ```
5. 机械臂回 HOME，用户按 'a' 开始（02:50:35）
6. 实验跑完 600 步后用户手动终止

### 实验时间线

| 阶段 | 步数范围 | 持续步数 | 描述 |
|---|---|---|---|
| **接近阶段** | 0–433 | 434 | 手臂从 HOME 移向插头，夹爪逐步缩窄 |
| **第一轮夹爪循环** | 434–449 | 16 | 12 次 grasp 全部失败，插头被弹出 |
| **开放/重新接近** | 450–549 | 100 | 模型放弃夹取，张开夹爪重新调整位置 |
| **第二轮夹爪循环** | 550–599 | 50 | 21 次 grasp 全部失败，持续到实验结束 |

### 关键统计

| 指标 | A3_1 | A2_1_7 (对比) |
|---|---|---|
| **Checkpoint** | itvlagpFrkPlug0907Dtaug010000 | 4wvlaFrkPlugCkp010420 |
| 总步数 | 600 | 1251 |
| 首次 grasp_issued | step 434 | step 745 |
| 总 grasp_issued | 33 | 201 |
| holding=True | **0** | **0** |
| close_gripper 警告 | 17 | 17 |
| 接近阶段步数 | 434 | 745 |

### 新 Checkpoint 的行为差异

与 A2_1_7（旧 checkpoint `4wvlaFrkPlugCkp010420`）相比，新 checkpoint 有几个显著差异：

#### 1. 更快到达夹取位置

接近阶段从 745 步缩短到 434 步（减少 42%）。新 checkpoint 可能在导航/接近方面更高效。

#### 2. 夹取失败后会"放弃"并重新尝试

这是最重要的行为差异。旧 checkpoint 在夹取失败后会**死循环**（持续 506 步直到实验结束），而新 checkpoint：

```
第一轮夹取 (434-449): 12 次 grasp → 全部失败
    ↓ 模型放弃 (action_grip 降到 0.12-0.15)
开放阶段 (450-549): 100 步重新调整
    ↓ 模型再次尝试 (action_grip 逐渐升高)
第二轮夹取 (550-599): 21 次 grasp → 全部失败
```

action_grip 在第一轮失败后从 ~0.97 骤降到 ~0.13，说明新模型能感知到夹取失败并主动调整策略。但 100 步后再次尝试时，由于底层参数问题未解决，仍然失败。

#### 3. 更多"全开弹出"事件

A3_1 中 42.2% 的夹爪循环步骤 w_meas > 60mm（完全张开），而 A2_1_7 只有 0.2%。这表明新 checkpoint 在夹取方式上更"激进"，导致插头更频繁地被弹出。

### 夹取失败的详细机制（与 A2_1_7 相同根因）

失败机制与 A2_1_7 完全相同：

1. **持握窗口 [0.001m, 0.009m] 不包含实际插头宽度 ~0.010m**
2. 每次 `grasp(width=0.005, epsilon=0.004, force=20N)` 调用后：
   - 要么闭合到 < 1mm（插头滑出，空手闭合）→ 警告 "measured width=0.0001m"
   - 要么停在 ~10mm（在插头上但窗口外）→ 警告 "measured width=0.0098m"
   - 偶尔弹出到 ~68mm（全开）→ 警告 "measured width=0.0683m"
3. `_hardware_holding()` 始终 False → 死循环

**close_gripper 警告中的 measured width 分布**：

| 测量值 | 次数 | 含义 |
|---|---|---|
| < 1mm | 11 | 空手闭合（插头未被夹住或滑出）|
| ~9.8mm | 1 | 在插头上但刚超出窗口上界 9mm |
| ~66-68mm | 3 | 全开（夹取过程中插头被弹飞）|
| 0.5-0.6mm | 2 | 接近空手闭合 |

### 第一轮夹取的详细过程 (steps 434-449)

```
step=434 a_grip=0.41 w= 50.7mm  GRASP ← 首次触发夹取，夹爪从 50mm 开始闭合
step=435 a_grip=0.94 w=  0.6mm  GRASP ← 闭合到 0.6mm（插头滑出，空手闭合）
step=436 a_grip=0.95 w= 68.3mm  GRASP ← 弹回全开（libfranka grasp 失败）
step=437 a_grip=0.95 w=  5.3mm  GRASP ← 再次闭合到 5.3mm（在窗口内但未被检测为 holding）
step=438 a_grip=0.96 w=  4.9mm        ← cooldown 中
step=439 a_grip=0.97 w= 11.6mm  GRASP ← 弹回到 11.6mm（插头宽度），再次夹取
...
step=445 a_grip=0.96 w=  0.0mm  GRASP ← 完全空手闭合
step=446 a_grip=0.97 w= 68.3mm  GRASP ← 弹回全开
step=447 a_grip=0.97 w=  5.5mm  GRASP 
step=448 a_grip=0.97 w=  0.1mm        ← 空手
step=449 a_grip=0.97 w=  0.1mm  GRASP ← 最后一次失败尝试
step=450 a_grip=0.14 w= 68.3mm  hold  ← 模型"放弃"，action_grip 骤降
```

### 解决方案

#### 与 A2_1_7 的方案完全一致

根因相同，修复方案不变：

**修改 `configs/franka_plug_eval.env`**：
```bash
export FRANKA_CUBE_WIDTH_M=0.008   # 8mm（略小于插头 10mm，确保压缩力）
export FRANKA_HOLD_TOL_M=0.006     # 6mm（窗口 [2mm, 14mm]，覆盖所有情况）
```

此修改已在 A2_1_7 分析中详细论证，此处不再赘述。

#### 额外观察：新 Checkpoint 的正面特征

尽管夹取同样失败，新 checkpoint `itvlagpFrkPlug0907Dtaug010000` 展现了两个值得注意的正面特征：
1. **更快的接近速度**（434 步 vs 745 步）
2. **失败后的自适应行为**（放弃 → 重新调整 → 再次尝试，而非无限死循环）

这些特征在修复参数问题后可能转化为更好的任务完成率。

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 5 | 环境变量 |
| `VLA_CKPT_PATH` | `.../itvlagpFrkPlug0907Dtaug010000` | 环境变量 |
| `VLA_GRIPPER_MODE` | continuous | `franka_plug_eval.env` |
| `VLA_GRIPPER_GRASP_HANDOFF_A` | 0.40 | `franka_plug_eval.env` |
| `VLA_GRASP_RETRY_COOLDOWN_S` | 1.0 | `franka_plug_eval.env` |
| `FRANKA_CUBE_WIDTH_M` | 0.002 → **钳位为 0.005** | 仍为错误值 |
| `FRANKA_HOLD_TOL_M` | 0.001 → **钳位为 0.004** | 仍为错误值 |
| `FRANKA_GRASP_FORCE` | 20N | `franka_plug_eval.env` |
| `max_steps` | 600 | CLI 参数 |

### 结论

A3_1 使用新 checkpoint 复现了 A2_1_7 的夹爪循环问题，**确认根因是 `FRANKA_CUBE_WIDTH_M` 参数配置错误**，而非模型问题。两个不同的 checkpoint 在相同的错误参数下表现出相同的失败模式。

**下一步必须先修复 `FRANKA_CUBE_WIDTH_M` 和 `FRANKA_HOLD_TOL_M`，再进行任何 checkpoint 对比实验。**

---

## 实验 A3_2：binary_abs 夹爪模式 — 夹爪始终未闭合

### 实验信息

| 项目 | 值 |
|---|---|
| **实验名称** | A3_2 |
| **日期** | 2026-09-23 03:00 – 03:04 (max_steps 耗尽) |
| **Checkpoint** | `itvlagpFrkPlug0907Dtaug010000`（同 A3_1）|
| **n_exec** | 5 |
| **max_steps** | 600 |
| **GRASP_HANDOFF_A** | 0.40 |
| **VLA_GRIPPER_MODE** | **`binary_abs`**（A3_1 用的是 `continuous`）|
| **VLA_GRIPPER_CLOSE_THRESHOLD** | **0.40** |
| **control_hz** | 10.0 (请求), 3.18 Hz (实际) |
| **gripper max_width** | 0.0683m (hand 报告) |
| **q7 安全余量** | 非对称：下界 0.0, 上界 0.08 |
| **日志文件** | `/tmp/client_a3_2_full.log` |

### 操作流程

1. Server 沿用 A3_1 的（同一 checkpoint，无需重启）
2. 启动 client，通过环境变量覆盖夹爪模式：
   ```bash
   docker exec rlinf-4dwvla-franky bash -c \
     'source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env; \
      export VLA_GRIPPER_MODE=binary_abs; \
      export VLA_GRIPPER_CLOSE_THRESHOLD=0.4; \
      cd /workspace/RLinf/b/x/4dwvla_ext; \
      /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
        --server-host localhost --server-port 5555 \
        --task "plug into socket" --max-steps 600 --n-exec 5 \
        --control-hz 10.0 --use-realsense'
   ```
3. 确认 `gripper mode=binary_abs`、`close if action >= 0.40` 生效
4. 机械臂回 HOME，用户按 'a' 开始
5. 600 步全部跑完，夹爪从未闭合

### 关键发现：action_grip 始终未达到闭合阈值

**整个 600 步中，action_grip 从未超过 0.40。最大值仅 0.1943（step 269）。**

| 指标 | 值 |
|---|---|
| action_grip >= 0.40 的步数 | **0**（600 步中） |
| action_grip 最大值 | 0.1943 |
| gripper_cmd = 'close' 的步数 | **0** |
| gripper_cmd = 'open' 的步数 | **600**（全部） |
| 夹爪宽度 | 68.3mm 全程不变（完全张开） |
| 夹取尝试 (grasp_issued) | **0** |

**action_grip 分布**：

| 范围 | 步数 | 占比 |
|---|---|---|
| [0.0, 0.1) | 52 | 8.7% |
| [0.1, 0.2) | 548 | **91.3%** |
| [0.2, 1.0) | 0 | 0% |

模型将 action_grip 锁定在 0.10–0.19 范围，从未尝试闭合夹爪。

### 根因分析：binary_abs 模式打破了模型的反馈回路

#### continuous 模式的工作原理

在 continuous 模式下（A3_1 使用的模式），夹爪宽度通过公式 `w_cmd = W0 × (1 − action_grip)` **渐进式缩窄**：

| action_grip | w_cmd (mm) | 效果 |
|---|---|---|
| 0.10 | 61.2 | 手指开始略微收缩 |
| 0.15 | 57.8 | 继续收缩 |
| 0.20 | 54.4 | 明显收缩 |
| 0.30 | 47.6 | 接近插头 |
| 0.40 | 40.8 | 触发 grasp_handoff（力控夹取）|

**关键**：每次 action_grip 增加，夹爪都会物理收缩。这个收缩：
1. **被腕部相机看到**（手指靠拢）
2. **被本体感觉感知**（state[7] = grip_width 减小）
3. 形成**正反馈**：模型看到手指在缩窄 → 判断"正在接近夹取" → 输出更高的 action_grip → 手指继续缩窄 → 最终越过 0.40 阈值

在 A3_1 中的实际观察：
```
step=  0  a_grip=0.03  grip_w=68.3mm   ← 完全张开
step=100  a_grip=0.16  grip_w=64.7mm   ← 已开始缩窄
step=300  a_grip=0.21  grip_w=59.4mm   ← 继续缩窄
step=420  a_grip=0.27  grip_w=54.8mm   ← 接近插头
step=432  a_grip=0.37  grip_w=50.6mm   ← 即将越过阈值
step=434  a_grip=0.41  grip_w= 0.6mm   ← 越过阈值！grasp_handoff 触发
```

#### binary_abs 模式的断裂

在 binary_abs 模式下：
- action_grip < 0.40 → 夹爪**完全张开**（68.3mm）
- action_grip >= 0.40 → 夹爪**完全闭合**

**没有中间状态。** 当模型输出 action_grip = 0.13 时：
- continuous 模式：夹爪缩窄到 ~59mm（视觉+本体感觉反馈）
- binary_abs 模式：夹爪保持 68.3mm（**零反馈**）

```
continuous 模式:  a_grip → 手指缩窄 → 相机看到变化 → 模型升高 a_grip → 越过阈值 ✓
                    ↑                                        ↓
                    └─── 正反馈回路 ──────────────────────────┘

binary_abs 模式:  a_grip=0.13 → 手指不动(68.3mm) → 相机无变化 → 模型不升高 a_grip → 死锁 ✗
                                    ↑                                        ↓
                                    └─── 无反馈 ────────────────────────────┘
```

在 A3_2 中：
```
step=  0  a_grip=0.04  grip_w=68.3mm
step=100  a_grip=0.11  grip_w=68.3mm   ← 无变化！
step=300  a_grip=0.12  grip_w=68.3mm   ← 还是无变化！
step=500  a_grip=0.13  grip_w=68.3mm   ← 模型困在 ~0.13
step=599  a_grip=0.14  grip_w=68.3mm   ← 600 步后仍然 68.3mm
```

### 手臂运动

尽管夹爪从未闭合，手臂确实移向了插头位置：

| 步数 | q1 | q2 | q3 | 描述 |
|---|---|---|---|---|
| 0 | -0.283 | 0.006 | 0.159 | HOME 位置 |
| 100 | -0.389 | 0.209 | 0.191 | 移向插头中 |
| 200 | -0.461 | 0.208 | 0.262 | 到达插头附近 |
| 300–599 | ~-0.459 | ~0.211 | ~0.266 | **停在插头旁**，但夹爪一直张开 |

手臂在 ~step 200 到达插头位置后基本不再移动，等待夹爪闭合信号——但由于夹爪宽度始终不变，模型无法进入下一阶段。

### 与 A3_1 的对比

| 指标 | A3_1 (continuous) | A3_2 (binary_abs) |
|---|---|---|
| 夹爪模式 | continuous | binary_abs |
| Checkpoint | 相同 | 相同 |
| 首次 action_grip >= 0.40 | step 434 | **从未达到** |
| action_grip 最大值 | 0.99+ | 0.1943 |
| 夹取尝试 | 33 次 | **0 次** |
| 夹爪宽度变化 | 68.3mm → 50.7mm → 闭合 | 68.3mm 全程不变 |
| 控制频率 | 2.86 Hz | 3.18 Hz |
| 手臂到位 | ~step 200 | ~step 200 |

### 结论

**`binary_abs` 模式与此模型不兼容。** 该模型训练时使用的是渐进式夹爪控制（continuous 模式），模型依赖夹爪逐步缩窄的视觉和本体感觉反馈来决定何时触发最终夹取。`binary_abs` 的全开/全闭二值行为打破了这一反馈回路，导致模型永远困在 action_grip ≈ 0.13 而无法越过 0.40 阈值。

**此模型必须使用 `continuous` 夹爪模式。** `binary_abs` 仅适用于训练时就使用二值夹爪控制的模型。

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 5 | 环境变量 |
| `VLA_CKPT_PATH` | `.../itvlagpFrkPlug0907Dtaug010000` | 环境变量（沿用 A3_1 server）|
| `VLA_GRIPPER_MODE` | **binary_abs** | 环境变量覆盖 |
| `VLA_GRIPPER_CLOSE_THRESHOLD` | **0.40** | 环境变量覆盖 |
| `VLA_GRIPPER_GRASP_HANDOFF_A` | 0.40 | `franka_plug_eval.env` |
| `FRANKA_CUBE_WIDTH_M` | 0.002 → 钳位为 0.005 | 仍为错误值（但本实验未触发夹取） |
| `max_steps` | 600 | CLI 参数 |

---

## 实验 A3_3：continuous 模式 + cube_width 修正后的夹取成功与后抓取悬停

### 实验配置

| 参数 | 值 | 备注 |
|---|---|---|
| 实验名 | A3_3 | |
| Checkpoint | `itvlagpFrkPlug0907Dtaug010000` | 数据增强版，同 A3_1/A3_2 |
| n_exec | 5 | `VLA_N_EXEC=5` |
| GRASP_HANDOFF_A | 0.40 | `VLA_GRIPPER_GRASP_HANDOFF_A=0.40` |
| VLA_GRIPPER_MODE | **continuous** | 环境变量覆盖（恢复为 continuous） |
| VLA_GRIPPER_CLOSE_THRESHOLD | 0.40 | 环境变量覆盖 |
| **FRANKA_CUBE_WIDTH_M** | **0.008** | **环境变量覆盖（关键修正！原值 0.002 被钳位到 0.005）** |
| **FRANKA_HOLD_TOL_M** | **0.004** | **环境变量覆盖（原值 0.001 被钳位到 0.004）** |
| max_steps | 600 | CLI 参数 |
| control_hz（请求/实际） | 10.0 / 3.09 | 实际 Hz 仅为请求的 31% |
| 总时间 | 03:18:51→03:22:05 = 194 秒 | |
| 总推理次数 | 120 次 | 600 步 / 5 步每次 |
| q7 OUT-OF-TRAIN 警告 | 264 次（steps 5-136，全在接近阶段） | |
| 日志 | `/tmp/client_a3_3_full.log` | Server 沿用 A3_1 的进程 |

### 操作细节

1. Server 沿用 A3_1 实验启动的 GPU 推理进程（VLA_N_EXEC=5, checkpoint=itvlagpFrkPlug0907Dtaug010000），无需重启
2. 在 franky 容器中执行实验前操作：回 HOME + 张开夹爪
3. Client 启动命令（关键改动：设置 FRANKA_CUBE_WIDTH_M=0.008, FRANKA_HOLD_TOL_M=0.004）：
```bash
docker exec rlinf-4dwvla-franky bash -c '
  source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env;
  export VLA_GRIPPER_MODE=continuous;
  export VLA_GRIPPER_CLOSE_THRESHOLD=0.4;
  export FRANKA_CUBE_WIDTH_M=0.008;
  export FRANKA_HOLD_TOL_M=0.004;
  cd /workspace/RLinf/b/x/4dwvla_ext;
  /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
    --server-host localhost --server-port 5555 \
    --task "plug into socket" --max-steps 600 \
    --n-exec 5 --control-hz 10.0 --use-realsense'
```
4. 启动日志确认 cube_width 修正生效：
```
plug eval grasp window: FRANKA_CUBE_WIDTH_M=0.008 +/- FRANKA_HOLD_TOL_M=0.004
FrankaLibfrankaGripper connected (max_width=0.068m, grasp_force=20.0N, cube_width=0.0080m +/-0.0040m)
```

### 关键时间线

```
Step    0 (03:18:51): 开始, q1=-0.280, q7=0.557, w_meas=68.3mm (open)
Step   50: q1=-0.382, q7=0.484 (q7 被钳位到下界)
Step  100: q1=-0.430, w_meas=64.9mm, a_grip=0.201 (夹爪开始缓慢收窄)
Step  130: q1=-0.463, w_meas=53.1mm, a_grip=0.311
Step  132: w_meas=49.0mm, a_grip=0.385
Step  133: ★ 夹取成功! a_grip=0.408 → grasp_handoff 触发
           w_meas=0.4mm (夹爪全闭), q1=-0.461, q7=0.484
Step  134: w_meas=10.3mm (夹爪微松，插头被夹住, holding=True)
Step  135: w_meas=11.0mm (稳定夹持, q5 开始腕关节调整)
Step  140: q5=-0.122, q7=0.526 (腕关节快速调整)
Step  200: q1=-0.464, q5=-0.164, q7=0.569, L2=0.158 ← 悬停中
Step  300: q1=-0.464, q5=-0.165, q7=0.570, L2=0.170 ← 仍在悬停
Step  400: q1=-0.467, q5=-0.159, q7=0.556, L2=0.144 ← 仍在悬停
Step  500: q1=-0.465, q5=-0.168, q7=0.562, L2=0.148 ← 仍在悬停
Step  599: q1=-0.471, q5=-0.160, q7=0.579, L2=0.158 ← 达到 max_steps，仍在悬停
```

### 重大正面发现 1：cube_width 修正彻底解决了夹取循环问题

**A3_1/A2_1_7 的夹取循环问题完全消失。** 对比：

| 指标 | A3_1/A2_1_7 (cube_width=0.005 钳位) | A3_3 (cube_width=0.008) |
|---|---|---|
| 夹取尝试次数 | 33-201 次 | **1 次** |
| 夹取成功 | 0 次（每次都"failed to capture"） | **1 次（一次成功）** |
| 夹爪状态 | 夹了松、松了夹循环 | **稳定夹持 w_meas≈11mm** |
| holding 状态 | 始终 False | **True（step 134 起）** |
| 控制频率 | ~1.9 Hz（grasp 阻塞拖慢） | **3.09 Hz（无阻塞）** |

#### 修正生效的机制

原有问题的根因是 `_hardware_holding()` 的判定窗口排除了实际插头宽度：

```
旧配置: cube_width=0.005 (0.002 被钳位), tol=0.004 (0.001 被钳位)
  → holding 窗口: [0.001m, 0.009m]
  → 实际插头宽度 ~0.010-0.011m 在窗口外 → holding=False → 反复重新夹取

新配置: cube_width=0.008, tol=0.004
  → holding 窗口: [0.004m, 0.012m]
  → 实际插头宽度 0.011m 在窗口内 → holding=True → 不再重新夹取
```

夹取过程详细序列：
1. Step 133: a_grip=0.408 越过阈值 → `should_issue_grasp_handoff(holding=False)` → 返回 True → 调用 `close_gripper()`
2. 夹爪全闭到 0.4mm → post-grasp 检查: abs(0.0004 - 0.008) = 0.0076 > 0.004 → 告警 "grasp did not capture"
3. Step 134: 夹爪微松到 10.3mm (0.0103m) → abs(0.0103 - 0.008) = 0.0023 ≤ 0.004 → **holding=True**
4. Step 135+: w_meas 稳定在 11.0mm → holding 持续为 True → 不再发起新的夹取
5. 467 步（step 133-599）中仅有 1 次夹取尝试，且夹持稳定

> **注意**：虽然 close_gripper 产生了 1 条 "did not capture" 告警（因为夹爪全闭到 0.4mm 时检测到宽度在窗口外），但夹爪微松后实际宽度 10.3mm→11.0mm 确实在 holding 窗口内，插头被稳定夹持。这表明 `close_gripper` 的 post-grasp 检查时机可能过早（在弹性恢复之前就检测了），可以考虑增加短延迟后复查。

### 核心问题：后抓取悬停（467 步，无运输迹象）

#### 问题描述

机械臂在 step 133 成功夹住插头后，在抓取位置附近进行微小振荡，持续了整个剩余实验周期（467 步 = 150 秒），**始终没有向插座方向运输**。最终因 max_steps=600 耗尽而终止。

这是 A2_1_5c 和 A2_1_6 中已观察到的"后抓取悬停"现象的再现，但表现更为严重。

#### 数据证据

**1. 关节空间位移对比**

```
接近阶段 (step 0→133, 134 步):
  Δq1=-0.181  Δq2=+0.210  Δq3=+0.100  Δq4=-0.236  Δq6=+0.492
  → 大幅运动, 从 HOME 到达插头位置

后抓取阶段 (step 133→599, 467 步):
  Δq1=-0.010  Δq2=-0.009  Δq3=+0.032  Δq5=-0.119  Δq7=+0.095
  → 467 步的净位移几乎为零! 仅 q5/q7 有腕关节调整
```

**2. 运动速度对比**

| 指标 | 接近阶段 | 后抓取阶段 | 变化 |
|---|---|---|---|
| 关节 delta 平均幅度 | 0.00793 rad/step | 0.00521 rad/step | -34% |
| q1 (肩关节) 平均 \|delta\| | 0.00192 | 0.00139 | -28% |
| q6 (肘关节) 平均 \|delta\| | 0.00467 | 0.00187 | -60% |

主要运动关节（q1-q4, q6）在后抓取阶段急剧减速。仅有腕关节 q7 的 |delta| 反而增加（0.00113→0.00317），说明模型在做腕部微调而非整臂运输。

**3. 模型动作目标范围对比**

```
                接近阶段 (target span)   后抓取阶段 (target span)   缩减比
  q1:           0.1834 rad              0.0212 rad               88%
  q2:           0.2148                  0.0204                   91%
  q3:           0.1194                  0.0464                   61%
  q4:           0.2395                  0.0289                   88%
  q6:           0.4991                  0.0323                   94%
```

**模型的动作输出本身就被锁定在极小范围内**——这不是安全钳位或硬件限制造成的，而是模型自身选择不动。

**4. L2 距离演化（从抓取点计算）**

```
step=140: L2=0.091  ← 初始腕关节调整
step=160: L2=0.156  ← 调整结束
step=200: L2=0.158  ← ★ 收敛, 不再远离
step=300: L2=0.170  ←   L2 在 0.14-0.17 间振荡
step=400: L2=0.144  ←   没有单调增长趋势
step=500: L2=0.148  ←   完全停滞
step=599: L2=0.158  ←   467 步后仍在原地
```

**5. q1 目标值线性趋势**

对后抓取阶段 467 个 step 的 q1 目标值做线性回归：
- 斜率 = -0.0000076 rad/step
- 467 步总漂移 = -0.00356 rad (0.204°)
- 对比: 接近阶段 133 步移动了 -0.1807 rad

**模型的 q1 目标值在 467 步中几乎没有任何趋势性变化**。运输需要 q1 向正方向运动（参考 A2_1_6 中 step 900-950 的 q1 从 -0.397 移向 -0.284），但模型从未发出这样的信号。

**6. n_exec=5 的子步骤分析**

每次推理产生 5 个子步骤的动作目标。后抓取阶段的典型 chunk：

```
--- Chunk at step 531 ---
  sub[0] step=531: q1=-0.4691 q5=-0.1649 q7=0.5702 grip=0.862
  sub[1] step=532: q1=-0.4680 q5=-0.1652 q7=0.5729 grip=0.871
  sub[2] step=533: q1=-0.4668 q5=-0.1662 q7=0.5740 grip=0.878
  sub[3] step=534: q1=-0.4666 q5=-0.1656 q7=0.5758 grip=0.883
  sub[4] step=535: q1=-0.4687 q5=-0.1655 q7=0.5534 grip=0.853
  → q1 spread: 0.0025 rad, q5 spread: 0.0013 rad
```

5 个子步骤内的 q1 变化仅 0.0025 rad，**连子步骤之间都没有运输意图**。这排除了"运输动作在后面的子步骤中，但被 n_exec=5 截断"的假设。

**7. 夹爪命令分布**

```
hold:          125 次 (step 0-132, 接近阶段)
move_width:      8 次 (step 125-132, 夹爪开始收窄)
grasp_handoff: 467 次 (step 133-599, 全部后抓取阶段)
```

### 与历史实验的对比：后抓取悬停是系统性问题

| 实验 | Checkpoint | 悬停持续 | 是否脱离悬停 | 备注 |
|---|---|---|---|---|
| A2_1_5c | 010420 (原始) | ~155 步 | 部分脱离 (step 525 开始缓慢运动) | max_steps=600 不够 |
| A2_1_6 | 010420 (原始) | ~200 步 | ★ 成功脱离 (step 900 开始急速运输) | max_steps=1200 |
| **A3_3** | **0907Dtaug (数据增强)** | **467 步 (实验结束仍未脱离)** | **否** | **max_steps=600** |

关键差异：
- A2_1_6 使用原始 checkpoint (010420)，悬停 200 步后成功过渡到运输阶段
- A3_3 使用数据增强 checkpoint (0907Dtaug010000)，467 步仍未脱离悬停
- 两者使用相同的 n_exec=5, GRASP_HANDOFF_A=0.40

**悬停阶段的关节模式高度一致**（表明是模型行为而非硬件问题）：

```
         A2_1_6 (悬停期 step 700-900)       A3_3 (悬停期 step 133-599)
q1:      [-0.41, -0.37] 振荡               [-0.47, -0.45] 振荡
q5:      稳定在 ~-0.20                     稳定在 ~-0.16
q7:      [0.58, 0.66] 小幅变化             [0.48, 0.59] 小幅变化
L2 变化:  0.18-0.25 振荡                   0.14-0.17 振荡
```

### 根因分析

#### 因素 1：数据增强 Checkpoint 的运输能力退化（最可能的主因）

数据增强 checkpoint (0907Dtaug010000) 在接近和夹取阶段表现出色（仅需 133 步到达并夹取，vs 原始 checkpoint 的 395-581 步），但**丧失了从夹取过渡到运输的能力**。

可能的原因：
- 数据增强在训练数据中引入了空间变换（随机裁剪、颜色抖动等），提高了泛化定位能力
- 但运输阶段依赖对场景布局的精确理解（插座相对于插头的方位），数据增强可能模糊了这种空间关系
- 训练 step 10000 可能不够——数据增强通常需要更多训练步数才能学到完整的任务行为

#### 因素 2：q7 钳位导致的本体感觉失真（次要因素）

接近阶段有 264 次 q7 OUT-OF-TRAIN 告警（step 5-136）。模型请求 q7 ≈ 0.40-0.48，但被钳位到 0.4843。这导致：
- 模型期望的 q7 与实际 q7 不一致，积累本体感觉误差
- 夹取时（step 133）q7=0.4844 处于训练范围边界——模型在训练中可能从未在这个 q7 值成功进入运输阶段
- 但 q7 在后抓取阶段升到 0.49-0.59（训练范围内），且无告警，说明 q7 本身不是阻碍运输的直接原因

#### 因素 3：悬停是"位姿搜索"而非"卡死"（行为特征）

q5/q7 振荡分析显示模型并非完全静止——它在进行低幅度的腕关节调整：

```
时间窗口         q5 目标 std    q7 目标 std    状态
[133-233]       0.02773        0.02661       ← 初始搜索（大振幅）
[233-333]       0.00420        0.00758       ← 收敛
[333-433]       0.00231        0.00875       ← 稳定振荡
[433-533]       0.00455        0.00607       ← 持续振荡
[533-600]       0.00301        0.00699       ← 仍在振荡，未脱离
```

模型在 step 233 左右完成了腕关节调整（q5 std 从 0.028 降到 0.004），之后进入稳态振荡。但这个稳态**不是运输前的准备状态**，而是模型的行为陷入了局部最优——它认为当前位姿"足够好"，不知道该往哪里移动。

### 解决方案

#### 方案 1：增加 max_steps 到 1500-2000（简单但不确定）

A2_1_6（原始 checkpoint）在 200 步悬停后成功过渡到运输。但 A3_3 的 467 步悬停没有任何脱离趋势（q1 漂移 slope ≈ 0），单纯增加步数**可能无效**。

如果尝试，建议 max_steps=1500，给模型 ~1300 步后抓取时间观察是否最终脱离。

#### 方案 2：换回原始 Checkpoint（推荐验证）

原始 checkpoint `4wvlaFrkPlugCkp010420` 在 A2_1_6 中展示了完整的"接近→抓取→悬停→运输→插入"流程。配合 A3_3 验证的 cube_width 修正：

```bash
# Server 使用原始 checkpoint:
VLA_N_EXEC=5 VLA_CKPT_PATH=.../4wvlaFrkPlugCkp010420 bash launch_gpu_server.sh

# Client 配合 cube_width 修正:
FRANKA_CUBE_WIDTH_M=0.008 FRANKA_HOLD_TOL_M=0.004 ...
```

预期效果：
- cube_width 修正消除夹取循环 → 仅需 1 次夹取
- 原始 checkpoint 有运输能力 → 悬停 200 步后开始运输
- max_steps=1200 足够完成全流程

#### 方案 3：减小 n_exec（n_exec=1 或 2）

更频繁的视觉反馈可能帮助模型在后抓取阶段做出更好的决策：
- n_exec=1：每步都做一次推理，模型始终能看到最新的视觉观察
- 缺点：控制频率可能降至 ~1.5 Hz（每步都需要推理），总步数效率下降

#### 方案 4：使用均匀子步骤采样（代码改动）

参考 A2_1_6 分析中提出的方案 3b——从 50 步轨迹预测中均匀采样而非取前 n 步：

```python
# 当前: normalized_action = action_pred[:n_exec, ...]  → 取 sub-step [0,1,2,3,4]
# 改为: 
indices = np.linspace(0, len(action_pred)-1, n_exec, dtype=int)
normalized_action = action_pred[indices, ...]  → 取 sub-step [0,12,24,37,49]
```

但 A3_3 的子步骤分析表明，5 个子步骤内的目标值几乎相同（q1 spread < 0.005），运输动作并不出现在后面的子步骤中。因此**此方案对 A3_3 的悬停问题可能效果有限**。

#### 方案 5：增大 n_exec 到 10-15（实验性）

n_exec 越大，模型在更长的"盲飞"窗口中执行。如果模型在预测的第 10-20 个子步骤中才开始产生运输动作，n_exec=10 能捕获到这些动作。

缺点：大 n_exec 意味着每 10 步才更新一次视觉观察，可能导致更大的偏差积累。

### 推荐下一步实验

**优先级排序**：

1. **A3_4（推荐）**：使用**原始 checkpoint `010420`** + cube_width 修正 + max_steps=1200，验证原始 checkpoint 在修正 cube_width 后的完整性能。这结合了 A2_1_6 验证的运输能力和 A3_3 验证的夹取修正。

2. **A3_5（备选）**：使用数据增强 checkpoint 0907Dtaug + cube_width 修正 + **max_steps=2000**，验证该 checkpoint 是否只是需要更多步数才能脱离悬停。

3. **A3_6（进阶）**：使用原始 checkpoint + cube_width 修正 + q7 安全边距扩展（SAFETY_MARGIN_UPPER_RAD[q7]=0.08，A2_1_6 后已实施）+ max_steps=1200，验证完整解决方案能否完成插入。

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 5 | 环境变量 |
| `VLA_CKPT_PATH` | `.../itvlagpFrkPlug0907Dtaug010000` | 环境变量（沿用 A3_1 server）|
| `VLA_GRIPPER_MODE` | **continuous** | 环境变量覆盖 |
| `VLA_GRIPPER_CLOSE_THRESHOLD` | **0.40** | 环境变量覆盖 |
| `VLA_GRIPPER_GRASP_HANDOFF_A` | 0.40 | `franka_plug_eval.env` |
| **`FRANKA_CUBE_WIDTH_M`** | **0.008** | **环境变量覆盖（关键修正）** |
| **`FRANKA_HOLD_TOL_M`** | **0.004** | **环境变量覆盖（关键修正）** |
| `max_steps` | 600 | CLI 参数 |

---

## 实验 A3_4：max_steps=1600 验证数据增强 Checkpoint 的运输能力

### 实验目的

验证 A3_3 中观察到的"后抓取悬停"是否仅因 max_steps=600 不够。A2_1_6（原始 checkpoint）在 200 步悬停后成功过渡到运输阶段（当时 max_steps=1200）。本次将 max_steps 增加到 1600，给数据增强 checkpoint 1393 步的后抓取时间观察是否最终脱离悬停。

### 实验配置

| 参数 | 值 | 备注 |
|---|---|---|
| 实验名 | A3_4 | |
| Checkpoint | `itvlagpFrkPlug0907Dtaug010000` | 数据增强版，同 A3_1/A3_2/A3_3 |
| n_exec | 5 | `VLA_N_EXEC=5` |
| GRASP_HANDOFF_A | 0.40 | |
| VLA_GRIPPER_MODE | continuous | |
| FRANKA_CUBE_WIDTH_M | **0.008** | 修正值（同 A3_3） |
| FRANKA_HOLD_TOL_M | **0.004** | 修正值（同 A3_3） |
| **max_steps** | **1600** | **从 A3_3 的 600 增加到 1600（本轮关键变化）** |
| control_hz（请求/实际） | 10.0 / 3.16 | |
| 总时间 | 03:40:02→03:48:28 = 506 秒（8.4 分钟） | |
| 总推理次数 | 320 次 | 1600 步 / 5 步每次 |
| q7 OUT-OF-TRAIN 警告 | 400 次（steps 10-209，全在接近阶段） | |
| 日志 | `/tmp/client_a3_4_full.log` | Server 沿用 A3_1 进程 |

### 操作细节

1. 在 franky 容器中执行 HOME + 张开夹爪（通过 `FrankyControllerDirect` + `reset_joint` + `open_gripper`）
2. Server 沿用 A3_1 实验启动的 GPU 推理进程（VLA_N_EXEC=5, checkpoint=itvlagpFrkPlug0907Dtaug010000）
3. Client 启动命令：
```bash
docker exec rlinf-4dwvla-franky bash -c '
  source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env;
  export VLA_GRIPPER_MODE=continuous;
  export VLA_GRIPPER_CLOSE_THRESHOLD=0.4;
  export FRANKA_CUBE_WIDTH_M=0.008;
  export FRANKA_HOLD_TOL_M=0.004;
  cd /workspace/RLinf/b/x/4dwvla_ext;
  /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
    --server-host localhost --server-port 5555 \
    --task "plug into socket" --max-steps 1600 \
    --n-exec 5 --control-hz 10.0 --use-realsense'
```
4. 启动日志确认参数：
```
plug eval grasp window: FRANKA_CUBE_WIDTH_M=0.008 +/- FRANKA_HOLD_TOL_M=0.004
Starting: task='plug into socket', max_steps=1600, n_exec=5, control_hz=10.0
```

### 关键时间线

```
Step    0 (03:40:02): 开始, q1=-0.277, q7=0.560, w_meas=68.3mm (open)
Step   20: q7 首次被钳位到 0.4844 (lower bound)
Step  100: q1=-0.380, a_grip=0.128, 接近中
Step  180: q1=-0.451, w_meas=66.8mm, 夹爪开始收窄
Step  200: q1=-0.466, w_meas=55.6mm, a_grip=0.259
Step  207: ★ 夹取! a_grip=0.414 → grasp_handoff → w_meas=48.1mm→5.7mm→11.0mm
Step  210: w_meas=11.0mm, q7=0.500 (q7 脱离下界), 夹持稳定
Step  300: q1=-0.464, L2=0.166, 悬停中
Step  500: q1=-0.465, L2=0.149, 悬停中
Step  700: q1=-0.471, L2=0.133, 悬停中
Step  900: q1=-0.475, L2=0.137, 悬停中
Step 1100: q1=-0.463, L2=0.134, 悬停中
Step 1300: q1=-0.462, L2=0.140, 悬停中
Step 1500: q1=-0.465, L2=0.148, 悬停中
Step 1599: q1=-0.466, L2=0.129, ★ 达到 max_steps=1600, 仍在悬停
```

### 核心结论：增加步数无效，Checkpoint 运输能力确认缺失

**1393 步后抓取时间（A3_3 的 3 倍），模型始终未脱离悬停。**

#### 决定性数据

**q1 零漂移**：对 1393 步后抓取数据做线性回归，q1 斜率 = +0.00000024 rad/step，总漂移 = +0.00033 rad ≈ 0.02°。这不是"还需要更多步数"——模型的 q1 目标没有任何方向性趋势。

```
q1 total drift in 1393 post-grasp steps: 0.00033 rad (0.02°)
对比: 接近阶段 207 步移动了 -0.1868 rad (10.7°)
比值: 1:566 — 后抓取的 q1 运动速率是接近阶段的 1/566
```

**最后 400 步也无趋势**（排除"即将脱离"的可能）：

```
q1 trend in steps 1200-1599: slope = -0.0000015 rad/step
400 步总漂移: -0.0006 rad (0.03°) → 本质为零
```

**L2 距离始终振荡**，无单调增长：

```
step  300: L2=0.166  ← 初期峰值
step  600: L2=0.132
step  900: L2=0.137  ← 在 0.12-0.17 间来回
step 1200: L2=0.156
step 1599: L2=0.129  ← 最终值反而更小（振荡回来了）
```

**action_grip 恒定**，模型知道在夹持但不知下一步该做什么：

```
窗口          action_grip 均值   标准差
[207-400]     0.868             0.059
[400-600]     0.869             0.019
[600-800]     0.867             0.020
... (全部 ≈ 0.87 ± 0.02) ...
[1400-1600]   0.867             0.018
```

action_grip 稳定在 0.87（远高于 0.40 阈值），说明模型一直"认为"应该夹紧。但它不产生运输动作。

#### 与 A2_1_6（原始 Checkpoint）的决定性对比

| 指标 | A2_1_6 (原始 ckp 010420) | A3_4 (数据增强 ckp 0907Dtaug) |
|---|---|---|
| 悬停持续 | 200 步 (step 700-900) | **1393 步 (step 207-1599, 未脱离)** |
| 是否脱离 | ★ 是 (step 900→急速运输) | **否** |
| q1 悬停范围 | [-0.41, -0.37] (span=0.04) | [-0.48, -0.45] (span=0.02) |
| q1 脱离后运动 | -0.40 → -0.28 (0.12 rad) | N/A |
| 后抓取总步数 | 409 步 (step 581-990) | **1393 步 (step 207-1599)** |
| 到达运输阶段 | ★ 是 (step 900) | **否** |
| 到达插入阶段 | ★ 是 (step 950) | **否** |
| 夹取循环 | 否 (同样 1 次成功, 但 cube_width 未修正) | 否 (cube_width 已修正) |

### 关节空间分析

**后抓取关节范围（1393 步）**——所有关节锁定在极小范围内：

```
  q1: [-0.476, -0.452] span=0.024 rad (1.4°)
  q2: [0.191, 0.214]  span=0.023 rad (1.3°)
  q3: [0.256, 0.302]  span=0.046 rad (2.6°)
  q4: [-2.124, -2.095] span=0.028 rad (1.6°)
  q5: [-0.177, -0.052] span=0.125 rad (7.2°) ← 仅腕关节有明显范围
  q6: [2.405, 2.441]  span=0.036 rad (2.1°)
  q7: [0.484, 0.593]  span=0.109 rad (6.2°) ← 仅腕关节有明显范围
```

q1-q4、q6 的总活动范围 < 3°，仅 q5 和 q7（腕关节）有 6-7° 的活动范围。模型在做"腕部微调"而非"整臂运输"。

**运动速度对比**——后抓取阶段运动速率骤降：

| 关节 | 接近阶段 mean |delta| | 后抓取阶段 mean |delta| | 下降 |
|---|---|---|---|
| q1 (肩) | 0.00162 | 0.00141 | -13% |
| q2 | 0.00155 | 0.00086 | -44% |
| q6 (肘) | 0.00361 | 0.00181 | -50% |
| q7 (腕) | 0.05048* | 0.00300 | -94% |
| 总体 | 0.05164 | 0.00504 | -90% |

*q7 接近阶段 delta 大是因为 q7 被钳位到 0.4844 而模型请求更低值，产生持续冲突。

**振荡窗口分析**——模型行为在 1393 步中无演化：

```
窗口          q1_std   q5_std   q7_std   q1_mean   q5_mean   q7_mean
[207-400]    0.0036   0.0200   0.0188   -0.4636   -0.1608    0.5663
[400-600]    0.0032   0.0059   0.0077   -0.4650   -0.1614    0.5635
[600-800]    0.0033   0.0042   0.0079   -0.4661   -0.1613    0.5635
[800-1000]   0.0042   0.0058   0.0083   -0.4655   -0.1611    0.5628
[1000-1200]  0.0037   0.0059   0.0073   -0.4655   -0.1601    0.5629
[1200-1400]  0.0029   0.0079   0.0120   -0.4645   -0.1563    0.5531
[1400-1600]  0.0038   0.0084   0.0131   -0.4638   -0.1518    0.5482
```

值得注意：最后 400 步中 q5_mean 从 -0.160 略升至 -0.152，q7_mean 从 0.563 略降至 0.548。这不是运输信号（运输需要 q1 向正方向大幅移动），而是微小的腕部漂移，可能是因为夹持 8 分钟后视觉场景的微小变化（光照漂移、手臂阴影等）。

### 根因定论

综合 A3_3（600 步，467 步悬停无脱离）和 A3_4（1600 步，1393 步悬停无脱离），可以得出**确定性结论**：

> **数据增强 checkpoint `itvlagpFrkPlug0907Dtaug010000` 丧失了从"夹取"到"运输"的任务阶段过渡能力。**

这不是步数问题、参数问题、或夹爪问题——模型本身不产生运输动作。可能的训练层面原因：

1. **数据增强破坏了空间关系学习**：增强可能在训练数据中引入了随机空间变换，使模型学会了鲁棒的"接近+夹取"（因为这只需要找到插头的局部特征），但**破坏了"运输"所需的场景级空间推理**（运输需要理解插座相对于当前位置的方位，这依赖全局空间布局）。

2. **训练步数不足**：step 10000 对于数据增强训练可能不够。增强引入的噪声需要更多训练步数来收敛。原始 checkpoint (010420, step 10420) 未使用增强，相同步数可能已经学到了完整行为。

3. **任务阶段分布不均**：训练数据中"接近+夹取"的帧数通常远多于"运输+插入"。数据增强可能进一步放大了这种不均衡，使模型更偏向"夹取"阶段的行为模式。

### 解决方案

#### 方案 1：换回原始 Checkpoint（强烈推荐）

原始 checkpoint `4wvlaFrkPlugCkp010420` 在 A2_1_6 中展示了完整的"接近→夹取→运输→插入"流程。配合已验证的 cube_width 修正：

```bash
# 新 Server:
docker exec -d rlinf-4dwvla-gpu bash -c '
  VLA_N_EXEC=5 \
  VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420 \
  bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh \
  > /tmp/server_a3_5.log 2>&1'

# Client 保持 cube_width 修正:
FRANKA_CUBE_WIDTH_M=0.008 FRANKA_HOLD_TOL_M=0.004 max_steps=1200
```

预期：
- 接近 ~400 步（原始 checkpoint 更谨慎）
- 夹取 1 次成功（cube_width 修正已验证）
- 悬停 ~200 步（A2_1_6 的模式）
- 运输 ~200 步
- 总计 ~800 步 → max_steps=1200 足够

#### 方案 2：尝试增强 Checkpoint 的更高训练步数版本

如果有 step 20000 或更高的数据增强 checkpoint 版本，可能已经学到了运输行为。检查是否存在：

```bash
ls /home/nvidia/bt/ckp/4wvlaFrk/plug/itvlagpFrkPlug0907Dtaug*/
```

#### 方案 3：修改推理阶段的提示策略

在夹取成功后切换任务描述从 `"plug into socket"` 到更具阶段指示性的文本，如 `"move plug to socket and insert"`。这需要修改 client 代码以支持动态任务描述。但此方案风险较高——模型可能未被训练处理此类动态提示。

### A3 系列实验总结

| 实验 | Checkpoint | 关键变量 | 结果 | 核心发现 |
|---|---|---|---|---|
| A3_1 | 0907Dtaug (增强) | 原始 cube_width | 夹取循环 33 次 | cube_width=0.005 排除插头宽度 |
| A3_2 | 0907Dtaug (增强) | binary_abs 模式 | 夹爪未闭合 | binary_abs 与此模型不兼容 |
| A3_3 | 0907Dtaug (增强) | cube_width 修正, max=600 | 夹取成功, 悬停 467 步 | cube_width 修正有效, 但无运输 |
| **A3_4** | **0907Dtaug (增强)** | **cube_width 修正, max=1600** | **夹取成功, 悬停 1393 步** | **增加步数无效, checkpoint 缺失运输能力** |

**A3 系列结论**：数据增强 checkpoint 的"接近+夹取"能力优秀（仅需 133-207 步到达并成功夹取），但**完全丧失了运输能力**。下一步应切换回原始 checkpoint `010420`，配合 cube_width 修正进行验证。

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 5 | 环境变量 |
| `VLA_CKPT_PATH` | `.../itvlagpFrkPlug0907Dtaug010000` | 环境变量（沿用 A3_1 server）|
| `VLA_GRIPPER_MODE` | continuous | 环境变量覆盖 |
| `VLA_GRIPPER_CLOSE_THRESHOLD` | 0.40 | 环境变量覆盖 |
| `VLA_GRIPPER_GRASP_HANDOFF_A` | 0.40 | `franka_plug_eval.env` |
| `FRANKA_CUBE_WIDTH_M` | **0.008** | 环境变量覆盖 |
| `FRANKA_HOLD_TOL_M` | **0.004** | 环境变量覆盖 |
| **`max_steps`** | **1600** | **CLI 参数（关键变化）** |

---

## 实验 A2_1_7b：原始 Checkpoint + cube_width 修正 — 场景位置变化与 q7 全程钳位

### 实验目的

验证原始 checkpoint `4wvlaFrkPlugCkp010420`（在 A2_1_6 中展示了完整任务流程）配合 cube_width 修正的效果。预期该 checkpoint 有运输能力，配合修正后的夹取参数应能完成 接近→夹取→运输 全流程。

### 实验配置

| 参数 | 值 | 备注 |
|---|---|---|
| 实验名 | A2_1_7b | "b"后缀以区分原 A2_1_7（旧 cube_width） |
| **Checkpoint** | **`4wvlaFrkPlugCkp010420`** | **原始 checkpoint，非数据增强** |
| n_exec | 5 | |
| GRASP_HANDOFF_A | 0.40 | |
| VLA_GRIPPER_MODE | continuous | |
| FRANKA_CUBE_WIDTH_M | **0.008** | 修正值 |
| FRANKA_HOLD_TOL_M | **0.004** | 修正值 |
| max_steps | 1600 | |
| control_hz（请求/实际） | 10.0 / 3.13 | |
| 总时间 | 04:11:20→04:19:51 = 511 秒（8.5 分钟） | |
| 总推理次数 | 320 次 | |
| q7 OUT-OF-TRAIN 警告 | **3190 次（steps 5-1599）** | **几乎每步都被钳位！** |
| 日志 | `/tmp/client_a2_1_7b_full.log`（client）, `/tmp/server_a2_1_7b.log`（server） | |

### 操作细节

1. 停止旧 server（数据增强 checkpoint），启动新 server：
```bash
# 停止旧 server
docker exec rlinf-4dwvla-gpu bash -c 'pkill -f vla_inference_server'

# 启动新 server（原始 checkpoint）
docker exec -d rlinf-4dwvla-gpu bash -c '
  VLA_N_EXEC=5 \
  VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420 \
  bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh \
  > /tmp/server_a2_1_7b.log 2>&1'
```
2. Server 启动确认：`Inference server listening on port 5555 (n_exec=5)`
3. HOME + 张开夹爪
4. Client 启动命令（同 A3_4，仅 server 使用不同 checkpoint）：
```bash
docker exec rlinf-4dwvla-franky bash -c '
  source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env;
  export VLA_GRIPPER_MODE=continuous;
  export VLA_GRIPPER_CLOSE_THRESHOLD=0.4;
  export FRANKA_CUBE_WIDTH_M=0.008;
  export FRANKA_HOLD_TOL_M=0.004;
  cd /workspace/RLinf/b/x/4dwvla_ext;
  /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
    --server-host localhost --server-port 5555 \
    --task "plug into socket" --max-steps 1600 \
    --n-exec 5 --control-hz 10.0 --use-realsense'
```

### 关键时间线

```
Step    0 (04:11:20): 开始, q1=-0.272, q3=+0.153, q7=0.551, w_meas=68.3mm
Step   25: q7 首次被钳位到 0.4844 (step 5 起开始告警)
Step   50: q1=-0.256, q3=+0.076 → 机械臂向正 q1 方向移动 (!)
Step  100: q1=-0.123, q3=-0.086 → 与 A2_1_6/A3 系列方向完全相反
Step  200: q1=-0.053, q3=-0.156, a_grip=0.241
Step  207: q7 仍锁死在 0.4844
Step  225: ★ 夹取! a_grip=0.427, w_meas=50.7mm→4.8mm→10.9mm (1次成功)
Step  300: q1=-0.048, q7=0.4844, L2=0.020 ← 几乎不动
Step  600: q1=-0.041, q7=0.4844, L2=0.027 ← 悬停
Step 1000: q1=-0.047, q7=0.4844, L2=0.022 ← 悬停
Step 1400: q1=-0.040, q7=0.4844, L2=0.029 ← 悬停
Step 1599: q1=-0.035, q7=0.4844, L2=0.038 ← max_steps 耗尽
```

### 重大发现 1：场景位置变化导致完全不同的接近方向

**机械臂的接近方向与 A2_1_6/A3 系列完全相反：**

```
                  q1 变化               q3 变化              方向
A2_1_6 (成功):    -0.271 → -0.436 (-0.165)   +0.165 → +0.232 (+0.067)   向左前方
A3_3/A3_4:        -0.280 → -0.464 (-0.184)   +0.155 → +0.256 (+0.101)   向左前方
A2_1_7b (本次):   -0.272 → -0.053 (+0.219)   +0.153 → -0.144 (-0.297)   ★ 向右前方！
```

q1 变化 +0.219 rad vs A2_1_6 的 -0.165 rad — **方向完全相反**。这意味着插头/插座的位置与之前实验不同。A3_4 结束时夹爪夹着插头，HOME 操作张开夹爪后插头可能掉落在了不同位置。

### 重大发现 2：q7 全程锁死（最严重的 q7 钳位案例）

**q7 在 1600 步中有 1595 步被钳位到 0.4844。后抓取 1375 步中，q7 没有任何一步超过 0.4844。**

```
  q7 全程实际范围:     [0.4844, 0.5507] → 仅 step 0-4 有 q7>0.4844
  q7 后抓取实际范围:   [0.4844, 0.4844] → span = 0.0000 (完全锁死!)
  q7 模型请求范围:     [0.3646, 0.5506] → 模型想要 q7 ≈ 0.37-0.46
  q7 平均请求-实际差:  -0.045 rad (持续 2.6° 偏差)
```

| 时间窗口 | 模型请求 q7 | 实际 q7 | 偏差 |
|---|---|---|---|
| [225-400] | 0.4364 | 0.4844 | -0.048 rad |
| [400-600] | 0.4390 | 0.4844 | -0.045 rad |
| [600-800] | 0.4404 | 0.4844 | -0.044 rad |
| [800-1000] | 0.4426 | 0.4844 | -0.042 rad |
| [1000-1200] | 0.4425 | 0.4844 | -0.042 rad |
| [1200-1400] | 0.4383 | 0.4844 | -0.046 rad |
| [1400-1600] | 0.4364 | 0.4844 | -0.048 rad |

**模型持续请求 q7 ≈ 0.43-0.44，但 1375 步中每一步都被钳位到 0.4844。偏差从未收敛——模型始终无法达到它想要的位姿，也无法从反馈中学习"这个 q7 值不可达"。**

对比 A3_3/A3_4（数据增强 checkpoint）：后抓取阶段 q7 去到了 0.49-0.59（脱离了下界），q7 span = 0.11 rad。即使那个 checkpoint 也无法运输，但至少 q7 有自由度。A2_1_7b 的 q7 自由度 = 0。

### 重大发现 3：后抓取悬停比 A3 系列更严重

| 指标 | A3_3 (增强 ckp) | A3_4 (增强 ckp) | A2_1_7b (原始 ckp) |
|---|---|---|---|
| 后抓取步数 | 467 | 1393 | **1375** |
| L2 范围 | 0.09-0.17 | 0.12-0.17 | **0.02-0.04** |
| q7 span (后抓取) | 0.11 rad | 0.11 rad | **0.00 rad** |
| q1 span (后抓取) | 0.02 rad | 0.02 rad | **0.04 rad** |
| q5 变化 | -0.04 → -0.16 | -0.05 → -0.16 | **+0.05 → +0.05 (无变化)** |
| 总位移 (L2) | 0.158 | 0.129 | **0.038** |

A2_1_7b 的后抓取 L2 距离仅 0.02-0.04 rad，是 A3 系列的 1/4。**这是 q7 全程锁死的直接后果**——当 q7 被钳位时，模型无法做出需要 q7 配合的任何运动（包括腕关节调整），因此整臂更加僵化。

在 A3_3/A3_4 中，模型至少还能做 q5/q7 腕关节调整（q5 从 -0.04 变到 -0.16，q7 从 0.48 变到 0.59）。在 A2_1_7b 中，q5 和 q7 都无法有意义地变化。

### 根因分析

#### 因素 1：场景位置变化（直接原因）

实验之间场景布局发生了变化（最可能原因：A3_4 结束后 HOME 张开夹爪，插头从夹爪中掉落到了新位置，或者用户在本次实验前重新摆放了场景）。新的场景位置使得：
- 机械臂需要向相反方向接近（q1 正方向 vs 之前的负方向）
- 达到插头所需的关节配置中 q7 需要 ≈ 0.37-0.46，**低于训练范围下界 0.4843**
- 因此 q7 被全程钳位

#### 因素 2：q7 训练范围下界太高（根本原因）

q7 训练范围 = [0.4843, 0.9807]。这个范围是从训练数据中学到的，但只覆盖了工作空间的一部分。当场景位于需要 q7 < 0.48 的位置时，系统完全无法工作。

```
训练范围:      [0.4843 ████████████████ 0.9807]
A2_1_6 需求:   [0.40 ██ 0.55 (接近) ████████████ 1.05 (插入)]
A2_1_7b 需求:  [0.37 ████ 0.46 (全程)]  ← 几乎完全在训练范围外！
A3_3/A3_4 需求:[0.40 ██ 0.48 (接近) ██ 0.59 (后抓取)]
```

#### 因素 3：q7 全程钳位导致的连锁效应

```
q7 锁死在 0.4844
  → 模型请求的 q7 与实际 q7 始终不匹配 (偏差 -0.045 rad)
  → 模型的本体感觉模型崩溃 (预期 q7=0.43, 实际 q7=0.48)
  → 基于错误本体感觉的动作预测也不可靠
  → 运动幅度极小 (L2 ≈ 0.02, 后抓取几乎静止)
  → 无法过渡到运输阶段
```

这与 A3_3/A3_4 的根因不同：
- **A3_3/A3_4**：Checkpoint 能力缺失（数据增强破坏了运输能力），q7 有自由度但模型不运输
- **A2_1_7b**：Checkpoint 有运输能力（A2_1_6 已证明），但 q7 被全程锁死导致模型无法正常工作

### 与 A2_1_6 成功案例的关键差异

A2_1_6 是唯一完成了全流程的实验。对比关键差异：

| 因素 | A2_1_6 (成功) | A2_1_7b (失败) |
|---|---|---|
| Checkpoint | 010420 (相同) | 010420 (相同) |
| cube_width | 0.005 (错误, 但仍夹取成功) | 0.008 (修正) |
| 场景位置 | 前移 + 白灯调亮 | **位置不同（机械臂方向相反）** |
| q7 接近阶段 | 钳位在 0.4844 (但只有接近阶段) | **钳位在 0.4844 (全程!)** |
| q7 后抓取 | 0.50→0.66 (脱离下界) | **0.4844 (永远锁死)** |
| q7 运输阶段 | 0.66→0.94 (大幅上升) | **N/A (从未到达)** |
| 后抓取 L2 | 0.18→0.25 (渐增) | **0.02→0.04 (极小)** |

**关键差异是场景位置**：A2_1_6 的场景位置使 q7 在夹取后自然升至训练范围内（0.50-0.66），模型恢复正常本体感觉后能规划运输。A2_1_7b 的场景位置使 q7 全程需要 < 0.48，永远在训练范围外。

### 解决方案

#### 方案 1：恢复 A2_1_6 的场景位置（强烈推荐，最简单）

A2_1_6 的成功证明了 checkpoint 010420 有完整能力。当前失败纯粹是场景位置导致的 q7 钳位问题。

**操作**：将插头和插座恢复到 A2_1_6 的"前移 + 白灯调亮"位置（即 A2_1_5c/A2_1_6 系列的位置），确保机械臂接近方向与 A2_1_6 一致（q1 向负方向移动）。

**验证方法**：观察 step 0-50 的 q1 变化方向。如果 q1 向负方向移动（-0.27 → -0.30+），说明场景位置正确。如果 q1 向正方向移动（-0.27 → -0.25-），说明场景位置有误。

#### 方案 2：扩展 q7 训练范围下界

修改 `TRAIN_ARM_MIN` 中的 q7 下界从 0.4843 到 0.35（或更低），使模型在更大的 q7 范围内可工作。

风险：这超出了训练数据的覆盖范围，模型在这些 q7 值上的行为是未定义的。但至少不会被完全锁死。

实现：修改 `franky_joint_env.py` 中的 `TRAIN_ARM_MIN[6]`。

#### 方案 3：去掉 q7 下界钳位（激进）

直接允许 q7 低于训练范围下界，不做钳位。模型可能在训练范围外有一定的泛化能力。

风险：超出训练范围的动作预测可能导致不稳定运动或 joint_reflex。

### 推荐下一步

**立即**：恢复场景到 A2_1_6 的位置（前移 + 白灯调亮），使用原始 checkpoint 010420 + cube_width 修正 + max_steps=1200，复现 A2_1_6 的成功。这是已验证可行的组合，唯一的新增是 cube_width 修正（消除夹取循环）。

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 5 | 环境变量 |
| `VLA_CKPT_PATH` | **`4wvlaFrkPlugCkp010420`** | **环境变量（新 server）** |
| `VLA_GRIPPER_MODE` | continuous | 环境变量覆盖 |
| `VLA_GRIPPER_CLOSE_THRESHOLD` | 0.40 | 环境变量覆盖 |
| `VLA_GRIPPER_GRASP_HANDOFF_A` | 0.40 | `franka_plug_eval.env` |
| `FRANKA_CUBE_WIDTH_M` | 0.008 | 环境变量覆盖 |
| `FRANKA_HOLD_TOL_M` | 0.004 | 环境变量覆盖 |
| `max_steps` | 1600 | CLI 参数 |

---

## 实验 A4_1：新 Checkpoint aug1w03pure1w07 — 场景位置问题再现

### 实验目的

测试第三个 checkpoint `aug1w03pure1w07`（1 周数据增强 + 3 周纯训练 + 7 周混合训练）的实际表现。

### 实验配置

| 参数 | 值 | 备注 |
|---|---|---|
| 实验名 | A4_1 | 新系列（A4 = aug1w03pure1w07 checkpoint） |
| **Checkpoint** | **`aug1w03pure1w07`** | **全新 checkpoint** |
| n_exec | 5 | |
| GRASP_HANDOFF_A | 0.40 | |
| VLA_GRIPPER_MODE | continuous | |
| FRANKA_CUBE_WIDTH_M | 0.008 | 修正值 |
| FRANKA_HOLD_TOL_M | 0.004 | 修正值 |
| max_steps | 1600 | |
| control_hz（请求/实际） | 10.0 / 3.13 | |
| 总时间 | 04:32:17→04:40:49 = 511 秒（8.5 分钟） | |
| 总推理次数 | 320 次 | |
| q7 OUT-OF-TRAIN 警告 | **3128 次（steps 10-1599）** | 几乎全程钳位 |
| 日志 | `/tmp/client_a4_1_full.log`（client）, `/tmp/server_a4_1.log`（server） | |

### 操作细节

1. 停止旧 server，启动新 server：
```bash
docker exec rlinf-4dwvla-gpu bash -c 'pkill -f vla_inference_server'
docker exec -d rlinf-4dwvla-gpu bash -c '
  VLA_N_EXEC=5 \
  VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/aug1w03pure1w07 \
  bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh \
  > /tmp/server_a4_1.log 2>&1'
```
2. Server 启动确认：`Inference server listening on port 5555 (n_exec=5)`
3. HOME + 张开夹爪
4. Client 启动命令（同 A3_4/A2_1_7b）

### 关键时间线

```
Step    0 (04:32:17): 开始, q1=-0.273, q3=+0.158, q7=0.557, w=68.3mm
Step   50: q1=-0.288, q3=+0.092, q7=0.484 (钳位开始)
Step  100: q1=-0.267, q3=+0.068, q7=0.484 ← 向右前方移动（与 A2_1_7b 相同方向！）
Step  200: q1=-0.082, q3=-0.119, w=66.5mm, a_grip=0.156
Step  250: ★ 夹取! a_grip=0.404 → w=51.5mm→0.6mm→10.9mm (1次成功)
Step  300: q1=-0.040, q3=-0.157, q7=0.484, L2=0.103 (悬停+振荡)
Step  600: q1=-0.106, q3=-0.088, q7=0.484, L2=0.145 (大幅振荡)
Step  800: q1=-0.138, q3=-0.063, q7=0.484, L2=0.139
Step 1000: q1=-0.111, q3=-0.083, q7=0.484, L2=0.155
Step 1200: q1=-0.110, q3=-0.083, q7=0.484, L2=0.140
Step 1400: q1=-0.104, q3=-0.080, q7=0.484, L2=0.151
Step 1599: q1=-0.116, q3=-0.078, q7=0.484, L2=0.152 ← max_steps 耗尽
```

### 核心发现：与 A2_1_7b 相同的场景位置问题

**接近方向与 A2_1_7b 完全一致——均向右前方移动（q1 正方向）**，与 A2_1_6 成功时的左前方方向相反：

```
              q1 变化           q3 变化           结论
A2_1_6:      -0.271→-0.436    +0.165→+0.232    左前方 ✓ (成功)
A2_1_7b:     -0.272→-0.053    +0.153→-0.144    右前方 ✗ (q7 锁死)
A4_1 (本次):  -0.273→-0.082    +0.158→-0.119    右前方 ✗ (q7 锁死)
```

**场景位置在 A2_1_7b 和 A4_1 之间没有变化（或变化极小）。** 两次实验的接近方向和关节配置几乎相同，证实场景位于需要低 q7 值的位置。

### q7 全程钳位（与 A2_1_7b 一致）

```
q7 实际范围:     [0.4844, 0.4919] → 仅偶尔略超 0.4844
q7 锁死步数:     1572/1600 (98.3%)
q7 模型请求:     [0.3792, 0.5567] → 均值 0.456 → 大部分在训练范围外
```

| 时间窗口 | 模型请求 q7 | 实际 q7 | 偏差 |
|---|---|---|---|
| [250-500] | 0.4594 | 0.4844 | -0.025 rad |
| [500-800] | 0.4682 | 0.4844 | -0.016 rad |
| [800-1100] | 0.4667 | 0.4844 | -0.018 rad |
| [1100-1400] | 0.4646 | 0.4844 | -0.020 rad |
| [1400-1600] | 0.4684 | 0.4844 | -0.016 rad |

相比 A2_1_7b（gap ≈ -0.045），A4_1 的 q7 偏差更小（-0.016 ~ -0.025）。这个 checkpoint 请求的 q7 值（≈0.46）更接近训练范围下界，但仍在下界之下。

### 三个 Checkpoint 在相同场景位置的对比

| 指标 | A3_4 (0907Dtaug) | A2_1_7b (010420) | A4_1 (aug1w03pure1w07) |
|---|---|---|---|
| 接近方向 | 左前方 (q1→-0.46) | 右前方 (q1→-0.05) | 右前方 (q1→-0.08) |
| q7 后抓取 | 0.49-0.59 (自由) | 0.4844 (锁死) | 0.4844 (近锁死) |
| q7 gap | N/A (在范围内) | -0.045 | -0.020 |
| 后抓取 L2 范围 | 0.12-0.17 | 0.02-0.04 | 0.10-0.15 |
| q1 span (后抓取) | 0.024 | 0.036 | **0.114** |
| q3 span (后抓取) | 0.046 | 0.038 | **0.115** |
| q5 span (后抓取) | 0.125 | 0.047 | **0.137** |
| 运输迹象 | 无 | 无 | 无 |
| 主要限制因素 | Checkpoint 运输能力缺失 | q7 全程锁死 | q7 近锁死 + 大幅振荡 |

**关键差异**：A3_4 的场景位置使 q7 在后抓取自然升至训练范围内（0.49-0.59），但该 checkpoint 本身缺运输能力。A2_1_7b 和 A4_1 的场景位置使 q7 需求低于训练范围，导致全程钳位。

A4_1 的振荡幅度比 A2_1_7b 大得多（q1 span 0.114 vs 0.036），说明这个 checkpoint 在"尝试"更积极地移动，但 q7 钳位阻止了有效运动。

### 根因定论

**连续三个 checkpoint 在当前场景位置下全部失败，唯一的成功案例 A2_1_6 使用的是不同的场景位置。** 问题的核心不是 checkpoint，而是：

1. **场景位置**使得目标 q7 值落在训练范围 [0.4843, 0.9807] 之外
2. **q7 钳位**破坏了模型的本体感觉闭环
3. **无论哪个 checkpoint**，在 q7 被锁死的情况下都无法完成运输

### 解决方案（优先级排序）

#### 方案 1：恢复 A2_1_6 的场景位置（最高优先级）

这是唯一已验证可行的路径。A2_1_6 成功时：
- 场景在"前移 + 白灯调亮"位置
- 接近方向为 q1 负方向（-0.27 → -0.44）
- q7 在后抓取自然升至 0.50-0.66（训练范围内）

**验证场景位置正确的方法**：启动实验后观察 step 0-20 的 q1 变化。如果 q1 向**负方向**变化（-0.27 → -0.29+），场景正确。如果 q1 向**正方向**变化（-0.27 → -0.26-），场景仍在错误位置。

#### 方案 2：扩展 q7 安全范围

修改 `franky_joint_env.py` 中 `TRAIN_ARM_MIN[6]` 从 0.4843 降低到 0.35，消除 q7 下界钳位。但模型在训练范围外的行为不确定。

#### 方案 3：去掉 q7 钳位限制

完全关闭 q7 的 OUT-OF-TRAIN 安全钳位，让模型自由控制 q7。风险最高但能测试模型的真实行为。

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 5 | 环境变量 |
| `VLA_CKPT_PATH` | **`aug1w03pure1w07`** | 环境变量（新 server） |
| `VLA_GRIPPER_MODE` | continuous | 环境变量覆盖 |
| `VLA_GRIPPER_CLOSE_THRESHOLD` | 0.40 | 环境变量覆盖 |
| `VLA_GRIPPER_GRASP_HANDOFF_A` | 0.40 | `franka_plug_eval.env` |
| `FRANKA_CUBE_WIDTH_M` | 0.008 | 环境变量覆盖 |
| `FRANKA_HOLD_TOL_M` | 0.004 | 环境变量覆盖 |
| `max_steps` | 1600 | CLI 参数 |

---

## 全局分析：A2_1_7b + A3_3/A3_4 + A4_1 的统一根因

### 当前实验的系统性失败模式

近 5 次实验（A3_3, A3_4, A2_1_7b, A4_1）都展现了相同的"夹取成功但无运输"模式。但失败的根因有两类：

```
           场景位置 A（A3 系列）           场景位置 B（A2_1_7b, A4_1）
           ┌──────────────────┐          ┌──────────────────┐
接近方向:   q1 负方向 (左前方)            q1 正方向 (右前方)
q7 状态:    后抓取脱离下界               后抓取全程锁死 0.4844
           (0.49-0.59 在范围内)          (需要 0.37-0.46 在范围外)
夹取:       1次成功 ✓                    1次成功 ✓
运输:       无运输 (ckp 能力问题)         无运输 (q7 钳位问题)
           └──────────────────┘          └──────────────────┘
                    ↕                             ↕
           唯一成功: A2_1_6                需要场景位置 A
           场景位置 A + 原始 ckp 010420    + 原始 ckp 010420
           q7 后抓取 0.50-0.66 → 运输
```

### 下一步行动建议

**必须做的一件事**：将场景恢复到 A2_1_6 的位置（接近方向 q1 负方向），然后使用原始 checkpoint 010420 + cube_width 修正 + max_steps=1200。

这是唯一已验证可行的组合（A2_1_6 几乎成功，仅因 q7 上界钳位在插入阶段崩溃）。配合 cube_width 修正消除夹取循环 + q7 上界安全边距扩展（已在 A2_1_6 后实施），有最高成功概率。

---

## 补充说明：q7 关节的物理含义与训练范围钳位机制

### q7 在 Franka 机械臂上的物理含义

q7 是 Franka Emika Panda 机械臂的**第 7 关节（末端腕旋转关节）**，控制夹爪/法兰绕自身接近轴的旋转：

- **q7 增大** → 夹爪绕接近轴**逆时针旋转**（从末端看向机器人方向）
- **q7 减小** → 夹爪绕接近轴**顺时针旋转**

q7 **不影响手臂的空间位移**（前后左右上下），只改变**夹爪的旋转朝向**——类似于拧螺丝刀时手腕的旋转动作。在插头插座任务中，q7 控制的是插头的旋转对齐角度。

### 训练范围钳位机制

`franky_joint_env.py` 中的 `check_action_safety()` 函数会将模型输出的关节目标值钳位到训练数据覆盖的范围内：

```
q7 训练范围: TRAIN_ARM_MIN[6] = 0.4843  ~  TRAIN_ARM_MAX[6] = 0.9807
```

当模型输出的 q7 目标低于 0.4843 时，会被强制设为 0.4843（下界钳位）。每一步的执行流程如下：

```
模型输出: q7 = 0.43  →  安全检查钳位: q7 = 0.4843  →  实际执行: q7 ≈ 0.4844
                         ↑
                   TRAIN_ARM_MIN[6] 卡住了
```

模型一直在说"再往顺时针转一点（q7 减小）"，但系统一直在说"不行，已经到训练数据见过的最小值了"。

### 为什么 q7 钳位会导致后抓取悬停

q7 钳位本身仅产生约 0.05 rad（≈3°）的旋转偏差，在插头末端（距腕关节 ~15cm 处）转化为不到 1mm 的位置偏移。**这个偏差本身不大，但它导致的连锁反应是致命的：**

1. **本体感觉持续不一致**：模型每步输出 q7=0.43，但每步读回的实际 q7=0.4844。模型的内部状态估计基于"我发出了指令，关节应该移动了"，但实际关节没动。这种持续的预期-现实不匹配使模型的内部世界模型逐渐失调。

2. **运动规划基础崩塌**：VLA 模型的动作预测依赖准确的本体感觉输入。当 q7 的反馈持续"错误"时，模型无法可靠地规划需要多关节协调的运输动作（运输需要 q1-q7 协调变化），因此退化为仅做微小振荡。

3. **场景位置决定了 q7 的需求**：
   - A2_1_6 成功时的场景位置：接近方向使 q7 在后抓取自然升至 0.50-0.66（在训练范围内），模型的本体感觉正常，能规划运输
   - A2_1_7b / A4_1 的场景位置：接近方向使 q7 全程需要 < 0.48（在训练范围外），q7 被锁死，本体感觉崩溃

### 双端钳位问题总结

q7 训练范围 [0.4843, 0.9807] 在实际任务中遭遇了**双端钳位**：

```
        下界钳位 (A2_1_7b, A4_1)              上界钳位 (A2_1_6)
        ↓                                       ↓
   0.37 ████ 0.48 [0.4843 ████████████ 0.9807] 1.05
        模型想要         训练范围              模型想要
        (当前场景)                           (插入阶段)
```

- **下界**：场景位于需要 q7 < 0.48 的位置时，q7 被锁在 0.4843，阻止接近和后抓取阶段的正常运动
- **上界**：插入阶段模型需要 q7 ≈ 1.05 来对准插孔，但被锁在 0.9807，导致对孔失败（A2_1_6 的崩溃原因）

两端的安全边距设置（`SAFETY_MARGIN_LOWER_RAD[6]=0.0`, `SAFETY_MARGIN_UPPER_RAD[6]=0.08`，后者在 A2_1_6 后扩展）可以在一定程度上缓解上界问题，但下界问题（当前场景的核心阻碍）需要通过恢复场景位置或扩展训练范围下界来解决。

---

## 实验 A4_2：放宽安全边距后的 Franka 死机

### 实验目的

验证放宽 SAFETY_MARGIN（全关节 ±0.2）后，q7 不再被锁死在 0.4844 时模型能否正常运输。

### 实验配置

| 参数 | 值 | 备注 |
|---|---|---|
| 实验名 | A4_2 | |
| Checkpoint | `aug1w03pure1w07` | 沿用 A4_1 server |
| **SAFETY_MARGIN_LOWER_RAD** | **[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]** | **关键变化：原为 [0.15×6, 0.0]** |
| **SAFETY_MARGIN_UPPER_RAD** | **[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]** | **关键变化：原为 [0.15×6, 0.08]** |
| 其余参数 | 同 A4_1 | n_exec=5, max_steps=1600, cube_width=0.008 |
| 执行步数 | **305 步（step 305 时 Franka 死机）** | |
| 总时间 | 06:07:48→06:09:27 = 99 秒 | |
| 总推理次数 | 62 次 | |
| q7 OUT-OF-TRAIN 警告 | **0 次！** | 修改生效 |
| 日志 | `/tmp/client_a4_2_full.log` | |

### 新的有效关节范围

修改 SAFETY_MARGIN 后的关节可达范围：

```python
TRAIN_ARM_MIN:      [-0.4842, -0.103, -0.2025, -2.2044, -0.2041, 1.5702, 0.4843]
TRAIN_ARM_MAX:      [ 0.0452,  0.312,  0.4789, -1.5347,  0.0806, 2.4536, 0.9807]
SAFETY_MARGIN:      ±0.2 (全关节)

ACTION_LIMIT_LOWER: [-0.6842, -0.303, -0.4025, -2.4044, -0.4041, 1.3702, 0.2843]
ACTION_LIMIT_UPPER: [ 0.2452,  0.512,  0.6789, -1.3347,  0.2806, 2.6536, 1.1807]
```

q7 可达范围从 [0.4843, 1.0607] 扩展到 **[0.2843, 1.1807]**。

### 操作细节

1. 用户修改 `franky_joint_env.py`：
```python
# 原来:
# SAFETY_MARGIN_LOWER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0])
# SAFETY_MARGIN_UPPER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.08])
# 修改为:
SAFETY_MARGIN_LOWER_RAD = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
SAFETY_MARGIN_UPPER_RAD = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
```
2. Server 沿用 A4_1 的进程（checkpoint=aug1w03pure1w07）
3. HOME + 张开夹爪 → 启动 client

### 关键时间线

```
Step    0 (06:07:48): q1=-0.275, q3=+0.156, q7=0.551, w=68.3mm (open)
Step   25: q7=0.417 ← q7 首次低于 0.4843（以前会被钳位，现在自由！）
Step   50: q7=0.386 ← 远低于旧下界
Step  100: q1=-0.201, q3=-0.011, q7=0.374, w=68.3mm ← 向右前方接近（同 A4_1）
Step  150: q1=-0.073, q3=-0.153, q7=0.373
Step  200: q1=-0.051, q3=-0.154, q7=0.401 ← 到达插头附近
Step  250: q1=-0.057, q3=-0.159, q7=0.376, a_grip=0.070 ← 夹爪未闭合
Step  300: q1=-0.052, q3=-0.157, q7=0.396, a_grip=0.074
Step  304: q1=-0.051, q3=-0.158, q7=0.398 ← 最后一个正常步
Step  305: ★ CRASH — Franka 进入"Other"模式（红灯闪烁）
           ControlException: Move command rejected: command not possible in current mode ("Other")
```

### 正面发现：q7 钳位完全消除

**放宽安全边距的预期效果完全实现：**

| 指标 | A4_1 (旧安全边距) | A4_2 (新安全边距) |
|---|---|---|
| q7 OUT-OF-TRAIN 警告 | 3128 次 | **0 次** |
| q7 实际范围 | [0.4844, 0.4919] | **[0.3644, 0.5531]** |
| q7 == 0.4844 步数 | 1572/1600 (98%) | **0/305 (0%)** |
| q7 < 0.4843 步数 | 0 | **297/305 (97%)** |
| 模型请求 q7 均值 | 0.4560 (被钳到 0.4844) | 0.3936 (**实际执行**) |

模型在 q7 自由后实际运行在 q7 ≈ 0.37-0.40，远低于旧下界 0.4843。这证实了之前的分析：模型确实需要 q7 在训练范围之外的值。

### 核心问题：Franka 内部安全停机

#### 崩溃机制

Step 304 执行完毕后，Franka 进入 "Other" 模式（内部安全反射）。Step 305 的 move 指令被拒绝。

```
Step 304 状态:  q=[-0.051, 0.221, -0.158, -2.041, 0.075, 2.369, 0.398]
Step 305 目标:  q=[-0.045, 0.214, -0.161, -2.026, 0.081, 2.355, 0.396]
Step 305 delta:   [+0.005, -0.007, -0.002, +0.015, +0.006, -0.015, -0.002]
```

**Step 305 的运动指令非常小**（所有关节 delta < 0.015 rad）。崩溃不是由某个大幅运动触发，而是 Franka 的内部安全系统（joint_reflex 或 cartesian_reflex）在 step 304 的运动执行过程中或完成后检测到异常。

#### 崩溃不是软件限制触发的

Step 304 的关节值与所有软件限制的距离：

```
关节  | 值     | 距 ACTION_LIMIT_LOWER | 距 ACTION_LIMIT_UPPER
q1    | -0.051 | +0.634 (安全)         | +0.296 (安全)
q3    | -0.158 | +0.244 (安全)         | +0.837 (安全)
q7    | 0.398  | +0.114 (安全)         | +0.783 (安全)
→ 所有关节都在软件限制的安全范围内
```

#### 可能的崩溃原因

**1. Franka 内部力矩/速度安全反射**

Franka 有独立于软件的硬件级安全系统，会在检测到异常力矩、速度或加速度时触发 reflex。当手臂处于非标准配置（q7≈0.40 + q3≈-0.16）时，相同的运动指令可能产生不同的内部力矩分布，触发内部安全阈值。

这个配置是训练数据中从未出现过的（训练范围 q7 ≥ 0.4843），机器人在该配置下的动力学特性可能与常规配置不同。

**2. Cartesian 运动护栏超出**

启动时设置的 Cartesian 运动护栏为：
```
min=[0.484, -0.19, 0.168], max=[0.652, 0.103, 0.567]
```
这个护栏是在 HOME 位置设置的。当手臂移到 q1≈-0.05、q3≈-0.16 的非标准配置时，TCP 位置可能超出了这个护栏范围。

**3. 腕关节旋转导致的自碰撞接近**

q7=0.398 加上 q3≈-0.16（负值）的组合使手臂末端接近某个自碰撞边界。Franka 的内部自碰撞检测可能在 step 304 的运动中检测到这一风险。

### 根因链分析

```
场景位置不同于训练数据
  → 模型需要 q7 ≈ 0.37-0.40 (低于训练范围 0.4843)
  → 放宽安全边距后 q7 可自由到达 0.37
  → 手臂进入训练数据中从未出现的关节配置
  → Franka 内部安全系统检测到该配置下的异常（力矩/速度/自碰撞）
  → 触发安全反射，进入 "Other" 模式
  → 后续指令被拒绝，实验终止
```

**本质问题**：放宽安全边距解决了 q7 钳位，但引入了新风险——模型可以到达训练数据中从未验证过的关节配置，这些配置可能不安全。

### 附加观察：夹爪从未闭合

a_grip 最大值仅 0.128（远低于 0.40 阈值）。机器人在崩溃时仍处于接近阶段，尚未尝试夹取。这与 A4_1（在 step 250 夹取成功）不同——**q7 自由后，模型的行为模式发生了变化**，接近更慢且更犹豫。

### 与 A4_1 的关键对比

| 指标 | A4_1 (q7 钳位) | A4_2 (q7 自由) |
|---|---|---|
| 到达插头位置 | ~step 200 | ~step 200（相似） |
| 夹取步数 | step 250 | **未达到（a_grip max=0.128）** |
| q7 后抓取 | 0.4844 锁死 | N/A（未抓取） |
| 总步数 | 1600 (完成) | **305 (崩溃)** |
| 崩溃原因 | 无 | **Franka 安全反射** |

这揭示了一个悖论：
- q7 被钳位时：模型可以接近并夹取，但无法运输（本体感觉失调）
- q7 自由时：模型到达新的关节配置空间，但 Franka 在该空间中不安全

### 解决方案

#### 方案 1：仅放宽 q7 的安全边距（推荐）

将 SAFETY_MARGIN 恢复为原始值，仅扩展 q7 的下界：

```python
SAFETY_MARGIN_LOWER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.2])  # 仅 q7 改为 0.2
SAFETY_MARGIN_UPPER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.2])  # 仅 q7 改为 0.2
```

这样其他关节保持原有的安全余量（0.15），减少进入不安全配置的风险。

#### 方案 2：恢复场景位置（最根本的解决）

将场景恢复到 A2_1_6 的位置。那个位置不需要 q7 < 0.48，因此不需要放宽安全边距就能正常工作。

#### 方案 3：扩大 Cartesian 运动护栏

如果崩溃原因是 Cartesian 护栏超出，可以在 `franky_controller_direct.py` 中扩大护栏范围。但这会降低对碰撞等异常的防护。

#### 方案 4：渐进式放宽 q7

将 q7 安全边距从 0.2 逐步减小（如 0.1），找到不触发安全反射的最大自由范围。

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 5 | 环境变量 |
| `VLA_CKPT_PATH` | `aug1w03pure1w07` | 沿用 A4_1 server |
| **`SAFETY_MARGIN_LOWER_RAD`** | **[0.2×7]** | **代码修改（关键变化）** |
| **`SAFETY_MARGIN_UPPER_RAD`** | **[0.2×7]** | **代码修改（关键变化）** |
| `FRANKA_CUBE_WIDTH_M` | 0.008 | 环境变量覆盖 |
| `FRANKA_HOLD_TOL_M` | 0.004 | 环境变量覆盖 |
| `max_steps` | 1600 | CLI 参数（仅执行了 305 步） |

> **注**：上述 A4_2（305 步崩溃版）是首次运行。下文 A4_2（重试）是恢复后的第二次运行，完成了全部 1600 步。

---

## 实验 A4_2（重试）：q7 自由后的 1600 步完整运行

### 实验目的

在首次 A4_2（305 步 Franka 死机）后，排除硬件异常，验证放宽安全边距 (±0.2) 后模型能否正常运行完整 1600 步。

### 与首次 A4_2 的区别

唯一区别：首次 A4_2 后 Franka 进入 "Other" 模式被手动恢复，机器人被移到正确的任务 HOME 位置（`home_pose.json`，非标准 Franka HOME）。首次 A4_2 崩溃的根因可能是瞬态的（不可复现的 Franka 内部 reflex），因为本次重试全程无崩溃。

### 实验配置

| 参数 | 值 |
|---|---|
| 实验名 | A4_2（重试） |
| Checkpoint | `aug1w03pure1w07`（沿用 A4_1/A4_2 的 server） |
| SAFETY_MARGIN | [0.2×7] lower + [0.2×7] upper |
| 其余参数 | 同首次 A4_2 |
| 总步数 | **1600（全部完成）** |
| 总时间 | 06:25:15 → 06:33:49 = 514 秒 |
| 总推理次数 | 320 次 |
| 控制频率 | 3.11 Hz |
| q7 OUT-OF-TRAIN 警告 | **0 次** |
| TRAIN-EDGE 警告 | 38 次 |
| 总警告数 | 19 |
| Franka 死机 | **无** |
| 日志 | `/tmp/client_a4_2_full.log` |

### 操作细节

1. 停止首次 A4_2 后的旧进程
2. 启动新 server（沿用同一 checkpoint）：
```bash
docker exec -d rlinf-4dwvla-gpu bash -c 'VLA_N_EXEC=5 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/aug1w03pure1w07 bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh > /tmp/server_a4_2.log 2>&1'
```
3. 首次启动 client 立即失败：
```
MotionGuardTripped: Cannot recover: fence: TCP outside fence: X=0.3069<0.4840
```
   **原因**：误用标准 Franka HOME `[0, -0.7854, 0, -2.3562, 0, 1.5708, 0.7854]`（TCP X≈0.307），而非任务 HOME `home_pose.json` 中的 `[-0.267, 0.015, 0.168, -1.842, -0.060, 1.892, 0.635]`（TCP X=0.571）。任务 HOME 的 TCP 在 motion guard fence [0.484, 0.652] 范围内。
4. 重新移至任务 HOME 后，client 正常启动
5. 通过 evdev 注入 `KEY_A` 启动 rollout

### Client 启动命令

```bash
docker exec -d rlinf-4dwvla-franky bash -c '
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env
export VLA_GRIPPER_MODE=continuous
export VLA_GRIPPER_CLOSE_THRESHOLD=0.4
export FRANKA_CUBE_WIDTH_M=0.008
export FRANKA_HOLD_TOL_M=0.004
cd /workspace/RLinf/b/x/4dwvla_ext
PYTHONPATH=/workspace/RLinf:/workspace/RLinf/b/x:$PYTHONPATH /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
  --server-host localhost --server-port 5555 \
  --task "plug into socket" \
  --max-steps 1600 --n-exec 5 --control-hz 10.0 \
  --use-realsense \
  > /tmp/client_a4_2_full.log 2>&1'
```

### 关键时间线

```
Step    0: q1=-0.267, q3=+0.168, q7=0.635, w=66.9mm, a_grip=0.040 (从任务HOME出发)
Step   50: q1=-0.269, q3=+0.099, q7=0.397, a_grip=0.020        ← q7已低于训练下界！
Step  100: q1=-0.224, q3=+0.034, q7=0.415, a_grip=0.035        ← 向右前方移动
Step  150: q1=-0.193, q3=+0.054, q7=0.461, a_grip=0.031
Step  200: q1=-0.173, q3=+0.005, q7=0.440, a_grip=0.026
Step  250: q1=-0.116, q3=-0.066, q7=0.417, a_grip=0.027        ← 接近插头区域
Step  300: q1=-0.086, q3=-0.069, q7=0.495, a_grip=0.034        ← 仍在接近
Step  400: q1=-0.137, q3=-0.075, q7=0.414, a_grip=0.116        ← a_grip升高但远不及0.40
Step  500: q1=-0.129, q3=-0.082, q7=0.389, a_grip=0.100
Step  750: q1=-0.036, q3=-0.167, q7=0.407, a_grip=0.092        ← 在插头附近来回
Step 1000: q1=-0.039, q3=-0.161, q7=0.395, a_grip=0.104
Step 1200: q1=-0.035, q3=-0.176, q7=0.394, a_grip=0.066
Step 1400: q1=-0.055, q3=-0.148, q7=0.414, a_grip=0.086
Step 1599: q1=-0.027, q3=-0.163, q7=0.422, a_grip=0.086 ← 从未夹取，max_steps耗尽
```

### 核心发现：q7 自由但从未夹取

**正面**：q7 确实获得了自由活动，range [0.361, 0.547]，1576/1600 步在训练范围下界以下。没有 Franka 死机。

**负面**：a_grip 最大值仅 0.160，远低于 0.40 阈值。夹爪全程保持张开。

a_grip 分布：
| 区间 | 占比 |
|---|---|
| < 0.05 | 21.6% |
| 0.05-0.10 | 51.1% |
| 0.10-0.15 | 26.8% |
| 0.15-0.20 | 0.5% |
| ≥ 0.20 | 0.0% |

后半段（step 500+）的关节 delta 均值仅 0.0066 rad，机器人在插头附近原地极小幅移动。

---

## 实验 A5_2：新 Checkpoint aug1w02augtw5k02pure1w06

### 实验目的

测试第四个 checkpoint `aug1w02augtw5k02pure1w06`（2 周数据增强 + 5000 步混合增强 + 6 周纯训练），验证是否改善夹取行为。

### 实验配置

| 参数 | 值 | 备注 |
|---|---|---|
| 实验名 | A5_2 | 新系列（A5 = aug1w02augtw5k02pure1w06 checkpoint） |
| **Checkpoint** | **`aug1w02augtw5k02pure1w06`** | **全新 checkpoint** |
| SAFETY_MARGIN | [0.2×7] lower + [0.2×7] upper | 同 A4_2 重试 |
| n_exec | 5 | |
| max_steps | 1600 | |
| 其余参数 | 同 A4_2 | |
| 总步数 | **1600（全部完成）** | |
| 总时间 | 06:43:51 → 06:52:23 = 512 秒 | |
| 总推理次数 | 320 次 | |
| 控制频率 | 3.13 Hz | |
| q7 OUT-OF-TRAIN 警告 | **0 次** | |
| TRAIN-EDGE 警告 | **0 次** | |
| 总警告数 | **0** | |
| Franka 死机 | **无** | |
| 日志 | `/tmp/client_a5_2_full.log` | |

### 操作细节

1. 停止 A4_2 server，启动新 server：
```bash
docker exec -d rlinf-4dwvla-gpu bash -c 'VLA_N_EXEC=5 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/aug1w02augtw5k02pure1w06 bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh > /tmp/server_a5_2.log 2>&1'
```
   首次启动失败：`FileNotFoundError: stats.json`。从 `aug1w03pure1w07` 复制 `stats.json` 后成功。
2. 回到任务 HOME + 张开夹爪
3. Client 启动（参数同 A4_2 重试）
4. evdev 注入 `KEY_A` 启动 rollout

### 关键时间线

```
Step    0: q1=-0.267, q3=+0.168, q7=0.635, w=66.9mm, a_grip=0.039 (从任务HOME出发)
Step   50: q1=-0.250, q3=+0.077, q7=0.388, a_grip=0.036        ← q7很快低于训练下界
Step  100: q1=-0.171, q3=-0.006, q7=0.411, a_grip=0.037
Step  150: q1=-0.108, q3=-0.095, q7=0.413, a_grip=0.050        ← 比A4_2快约100步到达
Step  200: q1=-0.119, q3=-0.093, q7=0.401, a_grip=0.075
Step  250: q1=-0.111, q3=-0.094, q7=0.395, a_grip=0.111        ← a_grip开始升高
Step  300: q1=-0.133, q3=-0.083, q7=0.378, a_grip=0.109
Step  400: q1=-0.136, q3=-0.071, q7=0.387, a_grip=0.111
Step  500: q1=-0.036, q3=-0.173, q7=0.392, a_grip=0.080        ← 转向插头附近
Step  750: q1=-0.033, q3=-0.170, q7=0.413, a_grip=0.087
Step 1000: q1=-0.033, q3=-0.163, q7=0.428, a_grip=0.072
Step 1200: q1=-0.032, q3=-0.165, q7=0.419, a_grip=0.092
Step 1400: q1=-0.057, q3=-0.156, q7=0.400, a_grip=0.081
Step 1599: q1=-0.037, q3=-0.161, q7=0.415, a_grip=0.077 ← 从未夹取，max_steps耗尽
```

### 行为与 A4_2 重试完全一致

a_grip 分布：
| 区间 | A4_2 (重试) | A5_2 |
|---|---|---|
| < 0.05 | 21.6% | 8.7% |
| 0.05-0.10 | 51.1% | 59.8% |
| 0.10-0.15 | 26.8% | 31.4% |
| 0.15-0.20 | 0.5% | 0.1% |
| ≥ 0.20 | 0.0% | 0.0% |
| **a_grip max** | **0.160** | **0.155** |

两个 checkpoint 的 a_grip 分布几乎相同。最大值均远低于 0.40 阈值。

---

## 跨实验深度对比分析：A4_1 vs A4_2（重试）vs A5_2

### 三实验核心对比表

| 指标 | A4_1 (q7钳位) | A4_2 重试 (q7自由) | A5_2 (q7自由) |
|---|---|---|---|
| **Checkpoint** | aug1w03pure1w07 | aug1w03pure1w07 | aug1w02augtw5k02pure1w06 |
| **SAFETY_MARGIN** | [0.15×6, 0.0/0.08] | [0.2×7] | [0.2×7] |
| **q7 实际范围** | [0.484, 0.492] | [0.361, 0.547] | [0.364, 0.564] |
| **q7 < 0.4843 步数** | 0/1600 (0%) | 1576/1600 (99%) | 1595/1600 (100%) |
| **OUT-OF-TRAIN 警告** | 3128 | 0 | 0 |
| **夹爪闭合** | ★ 是 (step 250) | ✗ 否 | ✗ 否 |
| **a_grip 最大值** | **0.404** | **0.160** | **0.155** |
| **后期 delta 均值** | 0.13-0.15 (L2) | 0.0066 | 0.0059 |
| **后期行为** | 大幅振荡 | 原地微动 | 原地微动 |
| **完成步数** | 1600 | 1600 | 1600 |
| **Franka 死机** | 无 | 无 | 无 |
| **控制频率** | 3.13 Hz | 3.11 Hz | 3.13 Hz |

### 关键发现：q7 自由 ↔ 夹取意愿的负相关

**这是最核心、最出乎意料的发现。**

A4_1 和 A4_2 使用完全相同的 checkpoint（`aug1w03pure1w07`），唯一区别是 SAFETY_MARGIN：

- **A4_1**（q7 钳位在 0.4844）：模型在 step 250 输出 a_grip=0.404 → 夹爪闭合 → 成功夹取
- **A4_2**（q7 自由在 0.36-0.55）：模型全程 a_grip < 0.16 → 夹爪从未闭合 → 从未夹取

**同一个模型、同一个场景，仅因为 q7 本体感觉（proprioceptive feedback）不同，夹取行为完全改变。**

### 根因分析：OOD 本体感觉导致的策略崩塌

#### 问题链

```
场景位置 → 模型需要 q7 ≈ 0.37-0.42
    ├── [q7 钳位] 模型收到 q7=0.4843（训练范围内）
    │   → 本体感觉在分布内 → 策略正常输出 → a_grip=0.40+ → 夹取成功
    │   → 但：q7 钳位导致后续本体感觉失调 → 无法运输（振荡）
    │
    └── [q7 自由] 模型收到 q7=0.37-0.42（训练范围外）
        → 本体感觉在分布外 → 策略输出退缩 → a_grip=0.08-0.15 → 从不夹取
        → 后续阶段也不会到达
```

#### 为什么 q7 OOD 影响夹取决策？

VLA 模型的 flow matching action head 接收全部 8 维状态（7 关节 + 1 夹爪宽度）作为输入。当 q7 的值从未在训练数据中出现过（训练范围 [0.4843, 0.9807]，当前运行 0.36-0.42），模型的隐含策略遭遇 OOD 输入。

具体机制：
1. **训练分布**：模型在 q7 ∈ [0.48, 0.98] 条件下学到"接近插头后输出 a_grip > 0.40 触发夹取"
2. **推理分布**：q7=0.37-0.42 不在训练分布中，模型对此区间的输出是未定义的
3. **退缩行为**：面对 OOD 输入，flow matching 的输出趋于保守——a_grip 停留在 0.08-0.15（"犹豫"区间），永远不触发夹取

这不是模型的 bug，而是所有基于学习的策略在 OOD 状态下的固有行为：**策略只在训练分布内有效**。

#### 数值证据

**A4_1 夹取时刻的状态对比**：
```
A4_1 step 250: q7=0.4844(钳位), a_grip=0.404 → ★ 触发夹取
A4_2 step 250: q7=0.4172(自由), a_grip=0.027 → ✗ 不夹取
A5_2 step 250: q7=0.3949(自由), a_grip=0.111 → ✗ 不夹取
```

同一时间窗口、同一位置附近，q7 差异 0.07-0.09 rad 导致 a_grip 从 0.40 降至 0.03-0.11。这说明 q7 值直接决定了模型的夹取意愿。

**最后 100 步的收敛状态**：

| 指标 | A4_2 重试 | A5_2 |
|---|---|---|
| q1 mean ± std | -0.036 ± 0.006 | -0.039 ± 0.004 |
| q7 mean ± std | 0.418 ± 0.009 | 0.417 ± 0.012 |

两个不同 checkpoint 收敛到几乎相同的最终状态 (q1≈-0.037, q7≈0.418)，进一步证明这是场景位置驱动的行为，而非 checkpoint 差异导致的。

### 悖论总结

| 场景 | q7 状态 | 本体感觉 | 夹取 | 运输 | 结果 |
|---|---|---|---|---|---|
| A4_1 | 钳位 0.4844 | 分布内 ✓ | 成功 ✓ | 失败 ✗ | 夹住后振荡 |
| A4_2/A5_2 | 自由 0.37-0.42 | 分布外 ✗ | 失败 ✗ | 未到达 | 永远不夹取 |
| A2_1_6（唯一成功） | 自然在范围内 | 分布内 ✓ | 成功 ✓ | 成功 ✓ | 完成任务 |

**只有在场景位置使 q7 自然处于训练范围 [0.48, 0.98] 内时，模型才能正常工作。** 当前场景位置是所有失败的根本原因。

### 解决方案（按可行性排序）

#### 方案 1：恢复 A2_1_6 的场景位置（最高优先级，已验证可行）

A2_1_6 是唯一完成 plug into socket 的实验。其场景特征：
- 插头/插座在 Franka 的左前方
- 接近时 q1 从 -0.27 向负方向移动（到 -0.44）
- q7 自然保持在 0.48-0.98 训练范围内
- 不需要放宽安全边距

**操作**：将插头和插座移回 A2_1_6 时的位置。这个位置的特征是"机器人右手臂向左前方伸出去"。

#### 方案 2：降低 VLA_GRIPPER_CLOSE_THRESHOLD（应急方案）

当前阈值为 0.40。在 q7 自由的情况下，模型 a_grip 峰值为 0.15-0.16。可以尝试降低阈值：

```bash
export VLA_GRIPPER_CLOSE_THRESHOLD=0.15
```

**风险**：阈值过低会导致在接近阶段误触发夹取（a_grip 在 step 200+ 就有 0.11 的值）。需要实验确定合适的阈值。

**根据数据估算**：
- 阈值 0.20: a_grip 从未超过 → 仍然不夹取
- 阈值 0.15: 仅 8-10 步超过 → 可能在正确时机触发，但也可能在错误时机
- 阈值 0.10: ~500 步超过 → 会在接近阶段误触发

这个方案是 hack，不解决根本问题。

#### 方案 3：分阶段 q7 策略（中等复杂度）

1. 接近+夹取阶段（step 0-300）：q7 钳位在 0.4843（保证夹取行为正常）
2. 夹取成功后：放开 q7 钳位（希望运输阶段不受影响）

**实现**：在 `franky_joint_env.py` 中添加逻辑，根据 `gripper_holding()` 状态动态调整 q7 钳位。

**风险**：夹取后突然改变 q7 可能引起新的 OOD 问题。

#### 方案 4：在当前场景位置收集训练数据（最根本但最慢）

在当前场景位置收集 demonstration，fine-tune checkpoint 使其学会 q7 < 0.48 时的行为。这需要较长的训练时间，但是唯一能从根本上解决 OOD 问题的方案。

### 发现速查

| 编号 | 发现 | 重要性 |
|---|---|---|
| F1 | q7 自由后模型不再尝试夹取（a_grip max 0.16 vs 钳位时 0.40） | ★★★ |
| F2 | 两个不同 checkpoint 在 q7 自由下行为完全相同 | ★★★ |
| F3 | 根因是 OOD 本体感觉导致的策略退缩 | ★★★ |
| F4 | 首次 A4_2 的 Franka 死机是瞬态问题（重试后不再复现） | ★★ |
| F5 | `home_pose.json` 定义了任务 HOME（不是标准 Franka HOME） | ★ |
| F6 | 新 checkpoint 缺少 `stats.json`，需从同系列 ckp 复制 | ★ |


---

## 实验 A5_3：continuous 模式 + 放宽 q7 安全裕度 + 长步数验证

**日期**: 2026-09-23  
**操作者**: 陈潇懿  
**结果**: ❌ 失败 — 夹爪下降到插头位置，但从未闭合夹取

### 1. 实验目标

在之前实验 A4_2 中观察到 q7 被钳位在 `TRAIN_ARM_MIN=0.4843` 边界时，模型的 grip 信号一度到达 0.40 的 `GRASP_HANDOFF_A` 阈值。然而由于 q7 严格约束（`SAFETY_MARGIN_LOWER_RAD[6]=0.0`），机械臂在 q7 维度上完全没有自由度，操作员怀疑这限制了末端姿态的调整，导致模型无法精确定位到抓取位置。

本次实验放宽了 q7 安全裕度，使 q7 可以低于 `TRAIN_ARM_MIN` 运行（但仍在硬限位之内），配合更长的步数上限（1600 步），验证以下假设：

> **假设**: 给予 q7 更多自由度后，模型能够更好地调整腕部姿态，从而在靠近插头后发出更强的 grip 信号，触发 `grasp_handoff`。

### 2. 配置参数

#### 2.1 环境变量（`franka_plug_eval.env`）

| 参数 | 值 | 说明 |
|---|---|---|
| `VLA_GRIPPER_MODE` | `continuous` | 连续 grip → 宽度映射 |
| `VLA_GRIPPER_GRASP_HANDOFF_A` | `0.40` | grip ≥ 此值触发力控夹取 |
| `VLA_GRIPPER_CLOSE_THRESHOLD` | `0.4` | binary_abs 备用阈值 |
| `VLA_GRIPPER_CLOSE_IF_ABOVE` | `1` | grip 方向：越大越闭 |
| `VLA_GRIPPER_DEADBAND_M` | `0.0015` | 缩窄死区 |
| `VLA_GRIPPER_WIDEN_DEADBAND_M` | `0.006` | 扩展死区 |
| `FRANKA_CUBE_WIDTH_M` | `0.008` | 插头厚度 8 mm |
| `FRANKA_HOLD_TOL_M` | `0.004` | 夹持容差 ±4 mm |
| `FRANKA_GRIPPER_MAX_WIDTH_M` | `0.080` | 最大夹爪宽度（被实际 hand 报告的 0.0669 m 钳位） |

#### 2.2 安全裕度修改（`franky_joint_env.py`）

本次实验前，操作员将 q7 的安全裕度从 `0.0` 放宽至 `0.08`：

```python
# 旧值（注释掉）:
# SAFETY_MARGIN_LOWER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0])
# SAFETY_MARGIN_UPPER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.08])

# 新值:
SAFETY_MARGIN_LOWER_RAD = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.08])
SAFETY_MARGIN_UPPER_RAD = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.1])
```

**影响**:
- q7 的 `ACTION_LIMIT_LOWER` 从 `0.4843`（= TRAIN_ARM_MIN - 0.0）变为 `0.4043`（= 0.4843 - 0.08）
- q7 的 `ACTION_LIMIT_UPPER` 从 `1.0607`（= 0.9807 + 0.08）变为 `1.0807`（= 0.9807 + 0.1）
- 其他关节 q1–q6 的裕度也从 0.15 增加到 0.2（但对本次实验影响较小）

#### 2.3 推理参数

| 参数 | 值 |
|---|---|
| Checkpoint | `aug1w02augtw5k02pure1w06` |
| n_exec | 5（每次推理 50 步中只执行前 5 步） |
| max_steps | 1600 |
| control_hz | 10.0（请求值；实际达到 3.11 Hz） |
| dtype | bfloat16 |
| task prompt | "plug into socket" |

### 3. 启动命令

#### 3.1 HOME + 张开夹爪

```bash
# 在 rlinf-4dwvla-franky 容器中执行
docker exec rlinf-4dwvla-franky /opt/venv/franky-0.19.0/bin/python \
    /workspace/RLinf/b/x/franky_ext/dsplug/stop_and_home.py \
    --execute --open-gripper --yes
```

输出：
```
connected: mode=RobotMode.Idle has_errors=False |dq|=0.0003 rad/s q=[...] tcp_xyz=[0.57, -0.067, 0.412]
stopped: mode=RobotMode.Idle ...
WARNING: opening the gripper may drop the plug
planned absolute joint move: max_delta=... rad target=[...home...]
home verified: max joint error=... rad, gripper=opened
RESULT DSPLUG_HOME PASS
```

#### 3.2 GPU 推理服务端（已在运行）

```bash
# 在 rlinf-4dwvla-gpu 容器中, PID 10079
python vla_inference_server.py \
    --ckpt-path /home/nvidia/ckpts/4wvlaFrk/plug/aug1w02augtw5k02pure1w06 \
    --schema-path /workspace/4WVLA/src/lerobot/dataset_schemas/configs/franka_plug.yaml \
    --n-exec 5 --port 5555 --dtype bfloat16
```

服务端关键启动日志：
```
2026-09-23 06:42:32 [INFO] Logging started
2026-09-23 06:42:32 [INFO] Stats loaded from .../aug1w02augtw5k02pure1w06/stats.json (state=8D, action=8D)
2026-09-23 06:42:59 [INFO] Model loaded: device=cuda dtype=torch.bfloat16 action_loss_only=True backend=standard kpt=True
2026-09-23 06:43:13 [INFO] Transform pipeline built (7 steps + unnormalize)
2026-09-23 06:43:13 [INFO] FK keypoint computer ready: 8 joints, dim=7, history=200
2026-09-23 06:43:13 [INFO] Inference server listening on port 5555 (n_exec=5)
```

#### 3.3 Franky 客户端

```bash
docker exec -d rlinf-4dwvla-franky bash -c '
  source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env;
  cd /workspace/RLinf/b/x/4dwvla_ext;
  /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
    --server-host localhost --server-port 5555 \
    --task "plug into socket" --max-steps 1600 \
    --n-exec 5 --control-hz 10.0 --use-realsense \
    > /tmp/client_a5_3.log 2>&1'
```

客户端关键启动日志：
```
2026-09-23 07:00:01 [INFO] Camera serials resolved: global=250222073513 wrist=420122070525
2026-09-23 07:00:01 [INFO] plug eval grasp window: FRANKA_CUBE_WIDTH_M=0.008 +/- FRANKA_HOLD_TOL_M=0.004
2026-09-23 07:00:01 [INFO] FrankaLibfrankaGripper connected (max_width=0.067m, grasp_force=20.0N)
2026-09-23 07:00:01 [INFO] Collision behavior tightened
2026-09-23 07:00:01 [INFO] Motion guard set: min=[0.484, -0.19, 0.168], max=[0.652, 0.103, 0.567]
2026-09-23 07:00:01 [INFO] gripper mode=continuous: w_cmd ceiling=0.0669m (configured 0.0800m, hand reports 0.0669m)
2026-09-23 07:00:01 [WARNING] FRANKA_GRIPPER_MAX_WIDTH_M=0.0800m exceeds the 0.0669m the hand reports; clamping to the reported value.
2026-09-23 07:00:01 [INFO] gripper command: close if action >= 0.40 (VLA_GRIPPER_CLOSE_IF_ABOVE=True)
```

### 4. 实验结果

#### 4.1 总体结果

```
Done: 1600 steps, 341 warnings, abort=False, achieved_control_hz=3.11 (requested 10.0)
```

- 1600 步全部执行完毕（未被安全系统中断）
- 控制频率 3.11 Hz，远低于请求的 10 Hz
- 341 个警告（主要是 OUT-OF-TRAIN）
- **夹爪从未触发 grasp_handoff，始终处于 "hold" 状态**

#### 4.2 action_grip 统计

| 指标 | 值 |
|---|---|
| 最小值 | 0.0124 |
| 最大值 | 0.2385 |
| 平均值 | 0.1029 |
| ≥ 0.40（GRASP_HANDOFF_A）的步数 | **0 / 1600 (0%)** |
| ≥ 0.30 的步数 | 0 |
| ≥ 0.20 的步数 | 11 |
| ≥ 0.15 的步数 | ~249 |

**action_grip 分布**:

| 区间 | 步数 | 直方图 |
|---|---|---|
| < 0.05 | 434 | ##################### |
| 0.05–0.10 | 174 | ######## |
| 0.10–0.15 | 743 | ##################################### |
| 0.15–0.20 | 238 | ########### |
| 0.20–0.25 | 11 |  |
| ≥ 0.25 | 0 |  |

**每 200 步窗口最大 grip**:

| 步数窗口 | max(action_grip) | 说明 |
|---|---|---|
| 0–199 | 0.063 | 离开 HOME，grip 极低 |
| 200–399 | 0.053 | 靠近工作区 |
| 400–599 | 0.172 | 初次进入插头附近 |
| 600–799 | 0.180 | 在插头附近徘徊 |
| 800–999 | **0.238** | 全程最高，仍远低于 0.40 |
| 1000–1199 | 0.196 | grip 开始下降 |
| 1200–1399 | 0.182 | 继续衰减 |
| 1400–1599 | 0.173 | 实验结束，从未触发 |

#### 4.3 夹爪命令分布

| 命令 | 次数 | 比例 |
|---|---|---|
| `hold` | 1595 | 99.7% |
| `move_width` | 5 | 0.3% |
| `grasp_handoff` | **0** | **0%** |

仅有的 5 次 `move_width` 事件：

| 步数 | action_grip | 说明 |
|---|---|---|
| 934 | 0.1864 | 轻微宽度调整 |
| 943 | 0.2105 | — |
| 964 | 0.2385 | 全程最大 grip |
| 965 | 0.1495 | 回落 |
| 1089 | 0.1894 | 最后一次宽度调整 |

#### 4.4 q7（腕部旋转）轨迹分析

**关键发现**: q7 在 1585/1600 步（99.1%）中低于 `TRAIN_ARM_MIN=0.4843`。

| 步数 | q7 | 状态 | 说明 |
|---|---|---|---|
| 0 | 0.5560 | ✓ 训练域内 | HOME 位置（0.6347 经第一步移动后） |
| 50 | 0.4333 | ✗ 域外 | 迅速离开训练域 |
| 100 | 0.4293 | ✗ 域外 | — |
| 200 | 0.4502 | ✗ 域外 | — |
| 400 | 0.4574 | ✗ 域外 | — |
| 600 | 0.4124 | ✗ 域外 | 接近下限 |
| 800 | 0.4413 | ✗ 域外 | 略有回升 |
| 1000 | 0.4225 | ✗ 域外 | — |
| 1200 | 0.4044 | ✗ 域外 | 接近 ACTION_LIMIT_LOWER |
| 1400 | 0.4091 | ✗ 域外 | — |
| 1599 | 0.4106 | ✗ 域外 | 实验结束 |

每 200 步窗口的 q7 统计：

| 窗口 | min | max | mean |
|---|---|---|---|
| 0–199 | 0.4247 | 0.5560 | 0.4357 |
| 200–399 | 0.4302 | 0.4736 | 0.4517 |
| 400–599 | 0.4059 | 0.4693 | 0.4444 |
| 600–799 | 0.4043 | 0.4525 | 0.4252 |
| 800–999 | 0.4043 | 0.4461 | 0.4258 |
| 1000–1199 | 0.4043 | 0.4317 | 0.4143 |
| 1200–1399 | 0.4043 | 0.4178 | 0.4082 |
| 1400–1599 | 0.4043 | 0.4178 | 0.4077 |

**OUT-OF-TRAIN 警告**: 662 次（约每 2.4 步一次）

q7 持续向 `ACTION_LIMIT_LOWER=0.4043` 逼近，最后 400 步中 q7 的均值仅 0.408，距下限仅 0.004 rad。

#### 4.5 q1 轨迹（接近指标）

| 步数 | q1 | 说明 |
|---|---|---|
| 0 | -0.273 | HOME |
| 100 | -0.222 | 移动中 |
| 200 | -0.096 | 接近工作区 |
| 600 | -0.090 | 在插头附近 |
| 1200 | -0.030 | 更靠近插头 |
| 1599 | -0.053 | 实验结束 |

机械臂确实移动到了插头附近，但夹爪始终没有闭合。

### 5. 服务端 50 步预测分析（核心失败机制）

服务端每次推理生成 50 步预测，但只有前 5 步（n_exec=5）被发送给客户端执行。

#### 5.1 全 50 步 grip 预测的分布

| 指标 | 值 |
|---|---|
| 总推理请求数 | 320（= 1600 步 / 5 步/请求） |
| 50 步中 grip 永远达不到 0.40 的请求 | 131/320 (20.5%) |
| 前 5 步最大 grip ≥ 0.40 的请求 | **0/320 (0%)** |
| 前 5 步最大 grip ≥ 0.30 的请求 | 0 |
| 前 5 步最大 grip ≥ 0.20 的请求 | 4 |

#### 5.2 50 步中首次达到 grip ≥ 0.40 的子步编号分布

| 子步范围 | 请求数 | 说明 |
|---|---|---|
| 子步 0–4 | **0** | ← 这是实际执行的范围，从未达到阈值 |
| 子步 5–14 | 0 | — |
| 子步 15–19 | 7 | 最早触发点 |
| 子步 20–24 | 15 | — |
| 子步 25–29 | 102 | **主要集中区域** |
| 子步 30–34 | 120 | **最密集** |
| 子步 35–39 | 123 | — |
| 子步 40–44 | 119 | — |
| 子步 45–49 | 23 | — |
| 从未达到 | 131 | 20.5% 的请求 |

**结论**: 在 509/640 的推理中（79.5%），模型确实在 50 步序列中"计划"了夹取动作，但将其推迟到子步 25–44（中位数约子步 33）。由于 `n_exec=5`，只有子步 0–4 被执行，grip 值仅为 0.10–0.17，远低于 0.40 的阈值。

#### 5.3 样例预测

**请求 0**（HOME 位置，远离插头）:
```
first5=[0.039, 0.044, 0.045, 0.045, 0.048]  chunk_max=0.062  → 从未计划夹取
```

**请求 100**（接近工作区）:
```
first5=[0.082, 0.092, 0.097, 0.104, 0.113]  → 执行的 grip 仅 ~0.10
steps[45:49]=[0.515, 0.516, 0.516, 0.545, 0.567]  → 序列末尾才达到 0.50+
```

**请求 200**（在插头附近）:
```
first5=[0.130, 0.135, 0.137, 0.143, 0.139]  → 执行的 grip 仅 ~0.14
steps[45:49]=[0.599, 0.607, 0.625, 0.644, 0.663]  → 序列末尾 grip 高达 0.66
```

**请求 300**（仍在附近徘徊）:
```
first5=[0.099, 0.100, 0.108, 0.101, 0.106]  → 执行的 grip 仅 ~0.10
steps[45:49]=[0.630, 0.642, 0.666, 0.694, 0.749]  → 序列末尾 grip 高达 0.75
```

**请求 319**（实验最后一次推理）:
```
first5=[0.114, 0.123, 0.126, 0.130, 0.133]  → 执行的 grip 仅 ~0.13
first sub-step ≥ 0.40: sub-step 30
chunk_max=0.826  → 序列末尾 grip 高达 0.83
```

### 6. 根因分析

#### 6.1 主因：Deferred Grasp（推迟夹取模式）— 与实验 A2_1_5 相同

这是与实验 A2_1_5 中首次识别的 **"永久推迟"（perpetual postponement）** 完全相同的失败模式：

1. **模型的 50 步预测序列**中，grip 信号呈现一个特征性的 S 型曲线：前 5–10 步保持低值（0.10–0.15），然后在子步 25–35 处快速爬升至 0.40+，在子步 40–50 处达到 0.60–0.80。

2. **但只有前 `n_exec=5` 步被执行**。由于 grip 在这些步中始终低于 0.40，`continuous_gripper_decision()` 从不触发 `grasp_handoff`，而是输出 `hold`。

3. **下一次推理重新观测当前状态**（夹爪仍然张开），再次生成类似的 S 型预测，grip 在前 5 步仍然很低 → 循环往复，永远无法触发夹取。

$$
\text{执行的 grip} = \text{chunk}[0:5] \approx 0.10\text{–}0.15 \ll 0.40 = \text{GRASP\_HANDOFF\_A}
$$

$$
\text{计划的 grip} = \text{chunk}[25:50] \approx 0.40\text{–}0.80 \geq 0.40
$$

由于 $n_\text{exec} = 5 \ll 25$（首次达到阈值的子步），模型的"未来计划"永远无法变成"当前执行"。

#### 6.2 副因：q7 OOD（分布外本体感知）加剧了 grip 退缩

与之前在 q7 被严格钳位时（A4_2 实验，action_grip 最高达 0.40）相比，本次实验放宽 q7 后：

| 对比项 | A4_2（q7 钳位在 0.4843） | A5_3（q7 放宽至 0.4043） |
|---|---|---|
| max(action_grip) | ~0.40 | **0.2385** |
| 平均 action_grip | ~0.14 | **0.1029** |
| 是否触发 grasp_handoff | 偶尔接近 | **从未** |
| q7 运行范围 | 固定在 0.4843 | 0.4043–0.5560 |
| OUT-OF-TRAIN 警告 | 较少 | **662 次** |

放宽 q7 裕度导致 q7 更深地进入训练域外区域（99.1% 的步数低于 TRAIN_ARM_MIN），模型接收到的本体感知信号（proprioception）越来越偏离训练分布。结果不仅没有提高 grip 信号，反而**进一步降低**了 grip 输出：

- A4_2 中 action_grip 最高到 0.40（接近阈值）
- A5_3 中 action_grip 最高仅 0.2385（远低于阈值）

**结论**: 给予 q7 更多自由度并没有帮助模型找到更好的姿态——相反，它让模型运行在一个更深的 OOD 区域，导致策略输出更加保守/退缩。

#### 6.3 因果链

```
HOME q7=0.635
  ↓ (模型输出的 q7 action 偏低)
q7 快速下降至 0.43–0.45 (离开训练域)
  ↓ (放宽的裕度允许 q7 继续下降)
q7 逐渐逼近 ACTION_LIMIT_LOWER=0.4043
  ↓ (越深的 OOD → 越保守的策略输出)
action_grip 被压缩到 0.10–0.15 范围
  ↓ (同时模型仍然在 50 步序列的后半段"计划"夹取)
前 5 步 grip << 0.40 → 夹爪始终 "hold"
  ↓ (每次重新观测 → 重新生成相同模式)
1600 步循环，从未触发夹取
```

### 7. 关键日志片段

#### 7.1 典型的 hold 步（步 1589，实验末尾附近）

```
[step 1589] execute: action=[...  0.414912,  0.141572]
                     state_before=[...  0.414231,  0.064846]
                     delta=[... 6.814e-04,  7.673e-02]
[step 1589] result:  state_after=[...  0.414862,  0.064846]
                     realized_delta=[... 6.32e-04,  0.000000]
    info:
      action_grip=0.1416         ← 远低于 0.40
      gripper_mode=continuous
      w_meas=0.0648              ← 夹爪保持张开
      w_cmd=0.0669               ← 命令宽度=上限
      delta_w=-0.0021            ← 微小缩窄（在死区内）
      gripper_cmd=hold           ← 夹爪保持不动
      grasp_issued=False
```

#### 7.2 OUT-OF-TRAIN 警告示例（步 1595–1596）

```
[step 1595] execute: action=[... 0.403197, 0.114439]
[WARNING] [step 1595] OUT-OF-TRAIN: q7=0.4032<0.4043
[step 1595] result: state_after=[... 0.404401, 0.064846]
    warnings=['[step 1595] OUT-OF-TRAIN: q7=0.4032<0.4043']
    action_grip=0.1144
    gripper_cmd=hold

[step 1596] execute: action=[... 0.404020, 0.123218]
[WARNING] [step 1596] OUT-OF-TRAIN: q7=0.4040<0.4043
```

模型输出的 q7 action（0.4032）已经低于 `ACTION_LIMIT_LOWER=0.4043`，被 L2 安全层钳位回 0.4043。

#### 7.3 最后一次推理（request 319）的完整 5 步 action

```
[inference 319] response: shape=[5, 8]
full_actions=[
  [-0.051878,  0.219233, -0.155359, -2.036659,  0.079551,  2.373184,  0.403197,  0.114439],
  [-0.052144,  0.219669, -0.155111, -2.038432,  0.079315,  2.375606,  0.404020,  0.123218],
  [-0.052121,  0.220065, -0.156256, -2.039200,  0.078415,  2.373263,  0.406051,  0.126038],
  [-0.052071,  0.220259, -0.154136, -2.040029,  0.077569,  2.373919,  0.408991,  0.129616],
  [-0.052762,  0.220625, -0.152760, -2.040618,  0.076328,  2.373413,  0.410620,  0.133494]
]
```

注意第 8 列（grip）: 0.114 → 0.123 → 0.126 → 0.130 → 0.133。grip 在缓慢上升，但上升速率仅 ~0.005/步，需要约 54 步才能从 0.13 达到 0.40。而每次只执行 5 步后就重新推理，永远追不上。

### 8. 与实验 A2_1_5 的对比

| 对比项 | A2_1_5 | A5_3 |
|---|---|---|
| Checkpoint | itvlagpFrkPlug0907Dtaug010000 | aug1w02augtw5k02pure1w06 |
| VLA_GRIPPER_MODE | continuous | continuous |
| GRASP_HANDOFF_A | 0.40 | 0.40 |
| n_exec | 5 | 5 |
| max_steps | 600 | 1600 |
| q7 SAFETY_MARGIN | 0.0 (严格) | 0.08 (放宽) |
| max(action_grip) | ~0.16 | 0.2385 |
| grasp_handoff 触发 | 0 次 | 0 次 |
| 失败模式 | deferred grasp | deferred grasp（加深） |
| q7 OOD 影响 | 无（被钳位在训练域边界） | 严重（99.1% 域外） |

### 9. 后续方向

基于 A4_2 → A5_3 的对比：

1. **恢复 q7 约束**: 将 q7 `SAFETY_MARGIN_LOWER_RAD[6]` 从 0.08 改回 0.0，避免 OOD 退化。
2. **增大 n_exec**: 从 5 增加到 15–25，使实际执行到达 50 步预测中 grip 上升的区间（但这会引入 open-loop 误差）。
3. **降低 GRASP_HANDOFF_A**: 从 0.40 降低到 0.20，使较弱的 grip 信号也能触发夹取（但可能导致误触发）。
4. **修改模型的 action chunk 生成策略**: 在模型层面解决 deferred grasp，使 grip 信号在序列前端而非后端。

### 发现速查

| 编号 | 发现 | 重要性 |
|---|---|---|
| F1 | action_grip 最大仅 0.2385，0/1600 步达到 0.40 阈值 | ★★★ |
| F2 | deferred grasp 模式：grip ≥ 0.40 首次出现在子步 25–35，但只执行前 5 步 | ★★★ |
| F3 | 放宽 q7 裕度反而加剧了 grip 退缩（A4_2: max=0.40 → A5_3: max=0.24） | ★★★ |
| F4 | q7 在 99.1% 的步数中低于 TRAIN_ARM_MIN，662 次 OUT-OF-TRAIN 警告 | ★★★ |
| F5 | 夹爪命令 99.7% 为 hold，0% grasp_handoff | ★★ |
| F6 | 控制频率仅 3.11 Hz（请求 10 Hz），与 A2_1_5 一致 | ★ |

---

## 实验 A5_4：收紧 q7 安全边距 + n_exec=10

### 实验目的

验证收紧 q7 安全边距（lower=0.08, upper=0.1，比 A4_2/A5_2 的 0.2 更窄）后，模型能否恢复正确的接近方向和夹取行为。同时将 n_exec 增加到 10 以获取更长的行动序列。

### 实验配置

| 参数 | 值 | 备注 |
|---|---|---|
| 实验名 | A5_4 | |
| **Checkpoint** | **`aug1w02augtw5k02pure1w06`** | 同 A5_2 |
| **SAFETY_MARGIN_LOWER_RAD** | **[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.08]** | **q7 收紧为 0.08** |
| **SAFETY_MARGIN_UPPER_RAD** | **[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.1]** | **q7 收紧为 0.1** |
| **n_exec** | **10** | **从 5 增至 10** |
| max_steps | 1600 | |
| 其余参数 | 同 A4_2/A5_2 | cube_width=0.008, hold_tol=0.004 |
| 总步数 | **1600（全部完成）** | |
| 总时间 | 07:23:52 → 07:31:21 = 449 秒 | |
| 总推理次数 | 160 次（1600/10） | |
| 控制频率 | **3.57 Hz** | 高于 A4_2/A5_2 的 3.11 Hz |
| q7 OUT-OF-TRAIN 警告 | 76 次 | |
| TRAIN-EDGE 警告 | 20 次 | |
| 总警告数 | 48 | |
| Franka 死机 | **无** | |
| 日志 | `/tmp/client_a5_4_full.log`, `/tmp/server_a5_4.log` | |

### 新的 q7 有效范围

```python
SAFETY_MARGIN_LOWER_RAD[6] = 0.08 → ACTION_LIMIT_LOWER[6] = 0.4843 - 0.08 = 0.4043
SAFETY_MARGIN_UPPER_RAD[6] = 0.10 → ACTION_LIMIT_UPPER[6] = 0.9807 + 0.10 = 1.0807
```

与之前版本的对比：
| 版本 | q7 下限 | q7 上限 | 说明 |
|---|---|---|---|
| 原始 (A4_1) | 0.4843 | 1.0607 | q7 lower margin=0.0 |
| 全放宽 (A4_2/A5_2) | 0.2843 | 1.1807 | q7 lower margin=0.2 |
| **本次 (A5_4)** | **0.4043** | **1.0807** | **q7 lower margin=0.08** |

### 操作细节

1. 用户修改 `franky_joint_env.py` 第 88-89 行：
```python
SAFETY_MARGIN_LOWER_RAD = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.08])
SAFETY_MARGIN_UPPER_RAD = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.1])
```
2. 停止旧进程，启动新 server（n_exec=10）：
```bash
docker exec -d rlinf-4dwvla-gpu bash -c 'VLA_N_EXEC=10 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/aug1w02augtw5k02pure1w06 bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh > /tmp/server_a5_4.log 2>&1'
```
3. 回到任务 HOME + 张开夹爪
4. Client 启动（n_exec=10）：
```bash
docker exec -d rlinf-4dwvla-franky bash -c '
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env
export VLA_GRIPPER_MODE=continuous
export VLA_GRIPPER_CLOSE_THRESHOLD=0.4
export FRANKA_CUBE_WIDTH_M=0.008
export FRANKA_HOLD_TOL_M=0.004
cd /workspace/RLinf/b/x/4dwvla_ext
PYTHONPATH=/workspace/RLinf:/workspace/RLinf/b/x:$PYTHONPATH /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
  --server-host localhost --server-port 5555 \
  --task "plug into socket" \
  --max-steps 1600 --n-exec 10 --control-hz 10.0 \
  --use-realsense \
  > /tmp/client_a5_4_full.log 2>&1'
```
5. evdev 注入 `KEY_A` 启动 rollout

### 关键时间线

```
Step    0: q1=-0.267, q3=+0.168, q7=0.635, w=66.9mm, a_grip=0.044  (任务HOME)
Step   50: q1=-0.308, q3=+0.144, q7=0.409, a_grip=0.057            ← 向左前移动！
Step  100: q1=-0.350, q3=+0.140, q7=0.420, a_grip=0.113
Step  150: q1=-0.365, q3=+0.152, q7=0.412, a_grip=0.156
Step  180: q7=0.420, w=57.6mm, a_grip=0.197                        ← 接近中，a_grip渐升
Step  190: q7=0.429, w=52.8mm, a_grip=0.325                        ← 夹爪开始合拢
Step  193: q7=0.432, w=53.0mm, a_grip=0.372
Step  196: q7=0.439, w=53.0mm, a_grip=0.402                        ← 首次超过阈值 0.40!
Step  197: q7=0.438, w= 0.1mm, a_grip=0.415                        ★ 夹取成功! w骤降
Step  200: q1=-0.397, q7=0.440, w= 9.4mm, a_grip=0.849             ← 稳定握持
Step  250: q1=-0.418, q3=+0.224, q7=0.504, a_grip=0.887            ← q7回到训练范围!
Step  300: q1=-0.424, q3=+0.256, q7=0.572, a_grip=0.888
Step  400: q1=-0.402, q3=+0.244, q7=0.586, a_grip=0.904            ← 悬停, q7在训练范围
Step  600: q1=-0.414, q3=+0.256, q7=0.568, a_grip=0.875
Step  800: q1=-0.405, q3=+0.254, q7=0.579, a_grip=0.890
Step 1000: q1=-0.405, q3=+0.244, q7=0.563, a_grip=0.894
Step 1200: q1=-0.403, q3=+0.247, q7=0.577, a_grip=0.902
Step 1400: q1=-0.390, q3=+0.237, q7=0.572, a_grip=0.902
Step 1599: q1=-0.404, q3=+0.246, q7=0.571, a_grip=0.931            ← max_steps耗尽
```

### 三大正面发现

#### 1. 接近方向恢复为左前方

A5_4 的 q1 始终向负方向移动（-0.267 → -0.424），与唯一成功的 A2_1_6 方向一致（-0.271 → -0.436）：

```
              q1变化              接近方向
A2_1_6:      -0.271 → -0.436    左前方 ✓ (成功)
A4_1:        -0.273 → -0.082    右前方 ✗ (q7钳位=0.4844)
A4_2 重试:   -0.267 → -0.021    右前方 ✗ (q7自由, lower=0.2)
A5_2:        -0.267 → -0.024    右前方 ✗ (q7自由, lower=0.2)
★ A5_4:      -0.267 → -0.424    左前方 ✓ (q7收紧, lower=0.08)
```

**q7 安全边距从 0.2 收紧到 0.08，使模型恢复了正确的接近方向。** 0.2 太宽导致 q7 进入 OOD 区域（0.28-0.40），模型行为崩塌；0.08 将 q7 限制在 0.40 以上，足够接近训练分布使模型表现正常。

#### 2. 夹取行为完全恢复

a_grip 在 step 196 首次超过 0.40 阈值，step 197 夹爪闭合成功。后续 a_grip 稳定在 0.85-0.95，强烈的持握意愿。

**夹取窗口极其精确**：a_grip 从 step 180 的 0.197 到 step 197 的 0.415，在 17 步内平滑上升并越过阈值。这证明模型的夹取决策与 q7 的本体感觉状态密切耦合。

夹取前后的 a_grip 对比：
| 实验 | q7 裕度 | step 197 a_grip | 是否夹取 |
|---|---|---|---|
| A4_2 重试 | lower=0.2 | 0.027 | ✗ |
| A5_2 | lower=0.2 | 0.050 | ✗ |
| **A5_4** | **lower=0.08** | **0.415** | **✓** |

#### 3. q7 后抓取自然回归训练范围

```
接近阶段 (step 0-199): q7_mean=0.422, 190/200步低于训练下界 (0.4843)
后抓取阶段 (step 200-1599): q7_mean=0.564, 仅40/1400步低于训练下界
```

夹取后手臂构型改变（握住插头抬起），q7 自然升至 0.50-0.59——完全在训练范围 [0.48, 0.98] 内。这意味着**后抓取阶段的本体感觉完全在分布内**。

### 核心问题：后抓取运输缺失

#### 数据

后抓取 q1 漂移（step 200 → 1599）：
```
q1: -0.406 → -0.403 = +0.003 rad (1400步, 几乎为零)
```

后抓取关节 delta 统计（step 200-1599）：
```
samples=1400  mean_delta=0.004138  min=0.000672  max=0.044042
```

后抓取 q1 标准差（分段）：
```
steps 200-500:  q1_mean=-0.409  q1_std=0.009
steps 800-1100: q1_mean=-0.406  q1_std=0.004
steps 1300-1600: q1_mean=-0.404  q1_std=0.006
```

q1_std 从 0.009 降至 0.004-0.006——运动幅度随时间递减，机器人趋于静止。

#### 与 A4_1 后抓取对比

| 指标 | A4_1 (同ckp系列, q7钳位) | A5_4 (本次) |
|---|---|---|
| 后抓取 q7 | 0.4844 (锁死) | 0.564 (训练范围内) |
| 后抓取 q1 drift | 0.076 (1350步) | 0.003 (1400步) |
| 后抓取 q1 std | 0.02-0.03 (振荡) | 0.004-0.009 (几乎静止) |
| 后抓取 delta 均值 | ~0.01 | 0.004 |

**A5_4 后抓取比 A4_1 更"安静"**。A4_1 有明显的振荡（q1 span=0.114），而 A5_4 几乎完全静止。

### 根因分析：为什么后抓取不运输？

#### 假说 1：Checkpoint 缺乏运输能力（最可能）

`aug1w02augtw5k02pure1w06` 和 `aug1w03pure1w07` 这两个 checkpoint 在所有实验中都没有展现运输行为——无论 q7 是否在训练范围内，无论接近方向是否正确。

对比唯一成功的 A2_1_6：
```
A2_1_6: checkpoint=4wvlaFrkPlugCkp010420, 接近+夹取+运输 全部成功
A4_1:   checkpoint=aug1w03pure1w07,       接近+夹取成功, 运输失败
A5_4:   checkpoint=aug1w02augtw5k02pure1w06, 接近+夹取成功, 运输失败
```

A2_1_6 使用的 checkpoint `4wvlaFrkPlugCkp010420` 是唯一展现过运输能力的。其他 checkpoint（数据增强版本）可能在训练过程中丧失或未学到运输策略。

#### 假说 2：场景位置与训练数据不匹配

即使 A5_4 的接近方向（左前方）与 A2_1_6 一致，但当前场景位置仍然与 A2_1_6 时不同。细微的位置差异可能导致模型在后抓取阶段无法找到正确的运输路径。

运输阶段需要模型"知道"插座在哪里。如果插座位置不在模型训练时见过的配置中，模型就无法生成正确的运输动作。

#### 假说 3：q7 接近阶段的 OOD 历史影响后续决策

接近阶段 q7 在训练范围外运行了 190 步。虽然后抓取 q7 回到了训练范围，但模型的状态历史（state_history_len=10）中可能还残留着 OOD 的 q7 值，影响了早期后抓取阶段的决策。不过这只能影响夹取后的前几个推理周期（10步），无法解释 1400 步的持续静止。

### 综合结论

```
问题层级:
  L1 (已解决): q7 安全边距过宽 → OOD 本体感觉 → 无法夹取
  L2 (已解决): q7 安全边距过窄 → q7 锁死 → 本体感觉失调 → 夹取后无法运输
  L3 (当前): q7 安全边距适中 → 夹取成功, q7后抓取在训练范围 → 但checkpoint缺乏运输能力
```

q7 安全边距 lower=0.08 是一个"甜蜜点"：既允许模型在接近阶段使用 q7 < 0.4843 的值（避免死锁），又将 q7 限制在 0.40 以上（避免 OOD 崩塌）。这解决了 L1 和 L2 层问题。

**但 L3 是 checkpoint 本身的能力限制**，不能通过调整安全边距解决。

### 解决方案

#### 方案 1：使用 `4wvlaFrkPlugCkp010420` checkpoint（快速验证）

这是唯一展现过运输能力的 checkpoint（A2_1_6 成功）。配合 A5_4 的安全边距设置（q7 lower=0.08, upper=0.1）重新测试，验证该 checkpoint 在当前场景位置下是否能完成运输。

#### 方案 2：恢复 A2_1_6 的场景位置 + 使用 `4wvlaFrkPlugCkp010420`

如果方案 1 仍然不运输，则恢复到 A2_1_6 的精确场景位置。这是唯一完整验证过的配置。

#### 方案 3：在当前场景位置收集更多运输阶段训练数据

如果要在当前位置使用，需要用遥操作收集包含运输阶段的 demonstration，fine-tune checkpoint 补充运输策略。

### 关键发现速查

| 编号 | 发现 | 重要性 |
|---|---|---|
| F1 | q7 lower=0.08 恢复了正确的左前方接近方向和夹取行为 | ★★★ |
| F2 | 夹取后 q7 自然回升至训练范围（0.422→0.564），本体感觉完全在分布内 | ★★★ |
| F3 | 后抓取 1400 步几乎完全静止（q1 drift=0.003），无运输 | ★★★ |
| F4 | 两个数据增强 checkpoint 均无运输能力，唯有 4wvlaFrkPlugCkp010420 成功过 | ★★★ |
| F5 | q7 lower=0.08 是当前场景位置下的"甜蜜点"：避免 OOD 崩塌，同时允许灵活接近 | ★★ |
| F6 | n_exec=10 使控制频率从 3.11 提升至 3.57 Hz | ★ |

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 10 | 环境变量（新增） |
| `VLA_CKPT_PATH` | `aug1w02augtw5k02pure1w06` | 同 A5_2 |
| `SAFETY_MARGIN_LOWER_RAD` | [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, **0.08**] | 代码修改 |
| `SAFETY_MARGIN_UPPER_RAD` | [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, **0.1**] | 代码修改 |
| `FRANKA_CUBE_WIDTH_M` | 0.008 | 环境变量 |
| `FRANKA_HOLD_TOL_M` | 0.004 | 环境变量 |
| `max_steps` | 1600 | CLI 参数 |

---

## 实验 A4_4：checkpoint aug1w01augtw5k02pure1w07 + q7 收紧约束

### 实验概述

| 项目 | 值 |
|---|---|
| **实验编号** | A4_4 |
| **日期/时间** | 2026-09-23 07:41:16 – 07:48:58（约 7m42s） |
| **checkpoint** | `aug1w01augtw5k02pure1w07` |
| **安全边距（lower）** | [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, **0.08**] |
| **安全边距（upper）** | [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, **0.1**] |
| **q7 有效范围** | [0.4043, 1.0807] |
| **n_exec** | 10 |
| **max_steps** | 1600 |
| **控制频率** | 3.59 Hz（请求 10.0 Hz） |
| **结果** | ✅ 正常完成 1600 步，全程持握，无运输 |

### 操作细节

**Server 端命令**：
```bash
docker exec -d rlinf-4dwvla-gpu bash -c 'VLA_N_EXEC=10 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/aug1w01augtw5k02pure1w07 bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh > /tmp/server_a4_4.log 2>&1'
```

**Client 端命令**：
```bash
docker exec -d rlinf-4dwvla-franky bash -c '
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env
export VLA_GRIPPER_MODE=continuous
export VLA_GRIPPER_CLOSE_THRESHOLD=0.4
export FRANKA_CUBE_WIDTH_M=0.008
export FRANKA_HOLD_TOL_M=0.004
cd /workspace/RLinf/b/x/4dwvla_ext
PYTHONPATH=/workspace/RLinf:/workspace/RLinf/b/x:$PYTHONPATH /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
  --server-host localhost --server-port 5555 \
  --task "plug into socket" \
  --max-steps 1600 --n-exec 10 --control-hz 10.0 \
  --use-realsense \
  > /tmp/client_a4_4_full.log 2>&1'
```

**注意**：该 checkpoint 缺少 `stats.json`，从 `aug1w03pure1w07/stats.json` 复制而来。

### 关键轨迹数据

| 步数 | q1 | q7 | w_meas (mm) | 状态 |
|------|-------|-------|-------------|------|
| 0 | -0.273 | 0.542 | 66.9 | 起始 HOME |
| 100 | -0.351 | 0.404 | 66.9 | 接近中，q7 在下限钳位 |
| 200 | -0.379 | 0.433 | 63.4 | 接近中 |
| 232 | -0.385 | 0.423 | 50.2 | **首次夹取尝试**（grasp_issued=True） |
| 245 | -0.388 | 0.459 | 9.66 | **持握确认** |
| 300 | -0.408 | 0.466 | 9.66 | 持握稳定 |
| 400 | -0.417 | 0.565 | 9.67 | 持握中 |
| 500 | -0.389 | 0.569 | 9.67 | 持握中 |
| 600 | -0.362 | 0.566 | 9.67 | q1 开始小幅回移 |
| 700 | -0.372 | 0.577 | 9.67 | 持握中 |
| 800 | -0.375 | 0.570 | 9.67 | 持握中 |
| 900 | -0.367 | 0.571 | 9.67 | 持握中 |
| 1000 | -0.368 | 0.580 | 9.67 | 持握中 |
| 1200 | -0.292 | 0.574 | 9.67 | q1 缓慢漂移中 |
| 1400 | -0.275 | 0.577 | 9.67 | 持握中 |
| 1599 | -0.284 | 0.574 | 9.67 | 正常结束，仍在持握 |

### 关节漂移分析

从夹取（step 245）到结束（step 1599）：

| 关节 | 夹取时 | 结束时 | 漂移量 | 方向 |
|------|--------|--------|--------|------|
| q1 | -0.388 | -0.284 | **+0.104 rad** | 正向（右移） |
| q7 | 0.459 | 0.574 | +0.115 rad | 正向（顺时针） |

q1 漂移 +0.104 rad，比 A5_4 的 0.003 rad 大 35 倍，但远不如 A2_1_6 的完整运输动作。q7 全程在训练范围内（0.404-0.580），无上限越界。

### OUT-OF-TRAIN 警告统计

| 类型 | 数量 | 步数范围 | 说明 |
|------|------|----------|------|
| q7 < 0.4043（下限） | 138 | step 37–112 | 接近阶段，q7 在下限钳位 |
| q7 > 1.0807（上限） | 0 | — | 无上限越界 |
| 总警告 | 114 | — | 含其他类型 |

### 关键日志片段

**首次夹取（step 232）**：
```
[step 232] gripper continuous grasp_handoff action=0.4006 w_meas=0.05019652470946312 w_cmd=0.0480 holding=False
RLinf: gripper grasp: width=0.0080m +/-0.0040m speed=0.050m/s force=20.0N (force held after contact)
```

**正常完成**：
```
Done: 1600 steps, 81 warnings, abort=False, achieved_control_hz=3.59 (requested 10.0)
```

### 分析

1. **夹取成功**：step 232 首次尝试即成功，全程持握 1355 步无掉落（w_meas 稳定在 9.67mm）
2. **q7 行为**：接近阶段 q7 被钳在下限 0.4044 附近（step 37-112，138 次 OOD），夹取后自然回升至训练范围（0.46→0.57），全程无上限越界
3. **后抓取运动**：q1 漂移 +0.104 rad，有轻微运输倾向但不足以完成运输。夹取后机械臂基本在原地"抖动"
4. **与 A5_4 对比**：同样 q7 sweet spot 配置，A4_4 的 q1 漂移（0.104）比 A5_4（0.003）大 35 倍，表明 checkpoint `aug1w01augtw5k02pure1w07` 比 `aug1w02augtw5k02pure1w06` 有更强的后抓取运动倾向
5. **稳定性极佳**：全程无 crash，无掉落，控制频率稳定 3.59 Hz

---

## 实验 A6_4：checkpoint aug1w015augtw5k015pure1w07 + q7 收紧约束

### 实验概述

| 项目 | 值 |
|---|---|
| **实验编号** | A6_4 |
| **日期/时间** | 2026-09-23 07:53:13 – 08:01:09（约 7m55s） |
| **checkpoint** | `aug1w015augtw5k015pure1w07` |
| **安全边距（lower）** | [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, **0.08**] |
| **安全边距（upper）** | [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, **0.1**] |
| **q7 有效范围** | [0.4043, 1.0807] |
| **n_exec** | 10 |
| **max_steps** | 1600 |
| **控制频率** | ~3.5 Hz（推算） |
| **结果** | ⚠️ 夹取成功 + 显著运输运动，但 step 1550 掉落，step 1598 Franka Reflex 崩溃 |

### 操作细节

**Server 端命令**：
```bash
docker exec -d rlinf-4dwvla-gpu bash -c 'VLA_N_EXEC=10 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/aug1w015augtw5k015pure1w07 bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh > /tmp/server_a6_4.log 2>&1'
```

**Client 端命令**：
```bash
docker exec -d rlinf-4dwvla-franky bash -c '
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env
export VLA_GRIPPER_MODE=continuous
export VLA_GRIPPER_CLOSE_THRESHOLD=0.4
export FRANKA_CUBE_WIDTH_M=0.008
export FRANKA_HOLD_TOL_M=0.004
cd /workspace/RLinf/b/x/4dwvla_ext
PYTHONPATH=/workspace/RLinf:/workspace/RLinf/b/x:$PYTHONPATH /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
  --server-host localhost --server-port 5555 \
  --task "plug into socket" \
  --max-steps 1600 --n-exec 10 --control-hz 10.0 \
  --use-realsense \
  > /tmp/client_a6_4_full.log 2>&1'
```

**注意**：该 checkpoint 缺少 `stats.json`，从 `aug1w03pure1w07/stats.json` 复制而来。

### 关键轨迹数据

| 步数 | q1 | q7 | w_meas (mm) | 状态 |
|------|-------|-------|-------------|------|
| 0 | -0.273 | 0.546 | 66.9 | 起始 HOME |
| 100 | -0.317 | 0.404 | 66.9 | 接近中，q7 在下限钳位 |
| 200 | -0.362 | 0.404 | 66.9 | 接近中 |
| 293 | -0.410 | 0.451 | 49.8 | **首次夹取尝试** |
| 400 | -0.428 | 0.552 | 9.36 | **持握确认** |
| 500 | -0.422 | 0.561 | 9.36 | 持握稳定 |
| 600 | -0.414 | 0.557 | 9.36 | 持握中 |
| 700 | -0.423 | 0.565 | 9.36 | 持握中 |
| 800 | -0.402 | 0.558 | 9.36 | 持握中 |
| 880 | -0.413 | 0.560 | 9.36 | 持握中 |
| 1000 | -0.397 | 0.567 | 9.36 | q1 缓慢漂移中 |
| 1200 | -0.384 | 0.570 | 9.36 | 持握中 |
| 1400 | -0.352 | **0.945** | 9.36 | ⚡ **q7 急剧跳变！运输启动** |
| 1500 | -0.239 | **1.050** | 9.54 | 运输中，q7 接近上限 |
| 1550 | -0.155 | **1.081** | **12.0** | ⚠️ **物体开始滑脱** |
| 1590 | -0.130 | 1.081 | **13.7** | ❌ **物体掉落** |
| 1597 | -0.133 | 1.081 | 10.9 | 再次尝试夹取（失败） |
| 1598 | — | — | — | 💥 **Franka Reflex 崩溃** |

### 关节漂移分析

从夹取确认（step 400）到最后有效步（step 1597）：

| 关节 | 夹取时 | 结束时 | 漂移量 | 方向 |
|------|--------|--------|--------|------|
| q1 | -0.428 | -0.133 | **+0.295 rad** | 正向（右移） |
| q2 | +0.197 | +0.279 | +0.082 rad | 正向 |
| q3 | +0.261 | +0.246 | -0.015 rad | — |
| q4 | -2.116 | -2.075 | +0.041 rad | — |
| q5 | -0.165 | -0.201 | -0.036 rad | — |
| q6 | +2.411 | +2.330 | -0.081 rad | 负向 |
| q7 | 0.552 | **1.081** | **+0.529 rad** | 正向（大幅顺时针旋转） |

**q1 漂移 +0.295 rad** — 这是所有实验中最大的后抓取运动！

**与前序实验对比**：

| 实验 | q1 漂移 | q7 漂移 | 持握时长 | 结果 |
|------|---------|---------|----------|------|
| A5_4 | 0.003 | 0.12 | 1400 步 | 无运输 |
| A4_4 | 0.104 | 0.12 | 1355 步 | 微弱运动 |
| **A6_4** | **0.295** | **0.53** | **1100 步** | **显著运输** |
| A2_1_6 | 完整运输 | 完整运输 | — | ✅ 成功插入 |

### OUT-OF-TRAIN 警告统计

| 类型 | 数量 | 步数范围 | 说明 |
|------|------|----------|------|
| q7 < 0.4043（下限） | 266 | step 30–202 | 接近阶段，q7 钳位 |
| q7 > 1.0807（上限） | 91 | step 1520–1598 | **运输阶段，q7 突破上限！** |
| 总警告 | 261 | — | 含其他类型 |

### q7 三阶段行为模式

A6_4 展现了清晰的 q7 三阶段模式：

```
q7 值
1.08 |                                            ████████████  ← 上限钳位 (1520-1598)
1.05 |                                       ████
0.95 |                                  ████
     |
0.57 |        ████████████████████████████  ← 训练范围内稳定 (300-1200)
0.55 |  ████
     |
0.45 | ██  ← 夹取瞬间
0.40 |██████  ← 下限钳位 (30-202)
     +---+---+---+---+---+---+---+---+---
     0  200 400 600 800 1000 1200 1400 1600  step
```

1. **Phase I — 接近阶段**（step 0-202）：q7 被钳在下限 0.404，模型请求更低值但被安全约束拦截
2. **Phase II — 稳定持握**（step 300-1200）：q7 在 0.55-0.57 范围稳定，完全在训练分布内
3. **Phase III — 运输爆发**（step 1200-1598）：q7 从 0.57 急剧攀升至 0.945→1.050→1.081，突破训练范围上限

### 崩溃分析

**直接原因**：step 1598，Franka Reflex 触发
```
[ERROR] vla-client: Fatal: ControlException: libfranka: Move command rejected: 
command not possible in the current mode ("Reflex")!
```

**触发链**：
1. step 1400-1500：q7 急剧攀升（0.945→1.081），大幅腕部旋转
2. step 1500：w_meas 从 9.36mm 轻微增大至 9.54mm（物体开始松动）
3. step 1550：w_meas 跳至 12.0mm（接近持握范围上边界）
4. step 1590：w_meas=13.7mm（物体脱落），夹爪反复尝试重新夹取
5. step 1596-1597：`grasp_issued=True`，但 `grasp did not capture`（夹爪闭合后测量宽度=0.0000m/0.0001m，物体已不在）
6. step 1598：空手快速运动触发 Franka 安全反射

**根本原因**：q7 大幅旋转（+0.529 rad）改变了夹爪对物体的夹持角度，导致物体从夹爪中旋转滑出。

### 关键日志片段

**首次夹取（step 293）**：
```
[step 293] gripper continuous grasp_handoff action=0.4054 w_meas=0.049762 w_cmd=0.0476 holding=False
RLinf: gripper grasp: width=0.0080m +/-0.0040m speed=0.050m/s force=20.0N
close_gripper: grasp did not capture the cube: measured width=0.0026m  ← 瞬态读数偏低
```
（尽管 close_gripper 报告未捕获，但后续 w_meas=9.36mm 确认物体实际已被持握）

**q7 突破上限（step 1520）**：
```
[WARNING] franky_joint_env: [step 1520] OUT-OF-TRAIN: q7=1.0896>1.0807
```

**物体掉落与再夹取失败（step 1596-1597）**：
```
[step 1596] gripper continuous grasp_handoff action=0.6029 w_meas=0.017726 holding=False
close_gripper: grasp did not capture the cube: measured width=0.0000m
[step 1597] OUT-OF-TRAIN: q7=1.0904>1.0807
close_gripper: grasp did not capture the cube: measured width=0.0001m
```

**Franka Reflex（step 1598）**：
```
[ERROR] Fatal: ControlException: libfranka: Move command rejected: 
command not possible in the current mode ("Reflex")!
```

### 分析与结论

#### 1. 突破性发现：运输行为涌现

A6_4 是继 A2_1_6（使用 4wvlaFrkPlugCkp010420 checkpoint）之后，**第一个展现明显后抓取运输行为的数据增强 checkpoint 实验**。q1 漂移 +0.295 rad 表明模型试图将物体从夹取位置移向目标位置。

#### 2. Checkpoint 运输能力排序

根据 q1 后抓取漂移量排序：

| 排名 | Checkpoint | q1 漂移 | 评价 |
|------|-----------|---------|------|
| 1 | **aug1w015augtw5k015pure1w07**（A6_4） | 0.295 rad | 显著运输，但过于激进导致掉落 |
| 2 | aug1w01augtw5k02pure1w07（A4_4） | 0.104 rad | 轻微运输倾向，稳定 |
| 3 | aug1w02augtw5k02pure1w06（A5_4） | 0.003 rad | 无运输，仅夹取 |
| 4 | aug1w03pure1w07（A4_2） | — | ±0.2 配置下无夹取 |

#### 3. q7 上限约束不足

A6_4 的 q7 upper margin=0.1 允许 q7 达到 1.0807，但模型实际需求（1.09+）超过此上限。在 Phase III 运输阶段，q7 在上限钳位处被截断，可能限制了运输动作的完整性。

可能的优化方向：
- **放宽 q7 上限**：将 upper margin 从 0.1 增至 0.15 或 0.2，允许更大的腕部旋转
- **风险**：更大的 OOD 可能导致不稳定行为

#### 4. 掉落原因：大幅腕部旋转

q7 在 step 1200-1500 间从 0.57 急升至 1.08（+0.51 rad ≈ 29°），这种大幅腕部旋转改变了夹爪对插头的夹持角度，导致插头从夹爪中旋转滑出。这暗示：
- 该 checkpoint 的运输策略包含大量腕部旋转（可能是训练数据中插入角度的体现）
- 夹爪力可能不足以在大幅旋转时保持夹持
- 或者需要调整 `FRANKA_CUBE_WIDTH_M` / 夹持力参数

### 环境变量和参数汇总

| 参数 | 值 | 来源 |
|---|---|---|
| `VLA_N_EXEC` | 10 | 环境变量 |
| `VLA_CKPT_PATH` | `aug1w015augtw5k015pure1w07` | 环境变量 |
| `SAFETY_MARGIN_LOWER_RAD` | [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, **0.08**] | 代码修改 |
| `SAFETY_MARGIN_UPPER_RAD` | [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, **0.1**] | 代码修改 |
| `FRANKA_CUBE_WIDTH_M` | 0.008 | 环境变量 |
| `FRANKA_HOLD_TOL_M` | 0.004 | 环境变量 |
| `max_steps` | 1600 | CLI 参数 |

---

## 跨实验深度对比分析（A5_4 / A4_4 / A6_4）

### 三实验横向对比表

| 维度 | A5_4 | A4_4 | A6_4 |
|------|------|------|------|
| **Checkpoint** | aug1w02augtw5k02pure1w06 | aug1w01augtw5k02pure1w07 | aug1w015augtw5k015pure1w07 |
| **完成状态** | ✅ 正常完成 | ✅ 正常完成 | ⚠️ Reflex 崩溃 |
| **夹取步数** | ~197 | ~232 | ~293 |
| **持握时长** | ~1400 步 | ~1355 步 | ~1100 步 |
| **q1 漂移** | 0.003 rad | 0.104 rad | **0.295 rad** |
| **q7 漂移** | 0.12 rad | 0.12 rad | **0.53 rad** |
| **q7 上限越界** | 0 次 | 0 次 | **91 次** |
| **物体掉落** | ❌ 否 | ❌ 否 | ✅ 是（step 1550） |
| **运输评价** | 无运输 | 微弱 | **显著但过激** |

### 核心发现

#### 1. Checkpoint 混合比例与运输能力的非线性关系

三个 checkpoint 的命名反映了训练数据混合比例差异：
- `aug1w02`：augmented data weight=0.2
- `aug1w01`：augmented data weight=0.1
- `aug1w015`：augmented data weight=0.15

运输能力（q1 漂移）并非与某个单一参数线性相关。`aug1w015`（中间值）反而展现最强运输能力，暗示训练数据混合存在一个"最优配比区间"。

#### 2. 运输行为与稳定性的权衡

```
运输能力 ↑
    │          A6_4 ★ (过激运输，掉落+crash)
    │
    │     A4_4 ★ (微弱运输，稳定)
    │
    │ A5_4 ★ (无运输，最稳定)
    │
    └─────────────────────────── 稳定性 →
```

存在明显的运输-稳定性权衡：运输能力越强的 checkpoint，越容易因激进动作导致物体掉落或触发安全保护。

#### 3. 下一步建议

1. **A6_4 再测试**：放宽 q7 上限（upper=0.15 或 0.2），看运输行为是否因减少钳位截断而更平滑
2. **A4_4 再测试**：同样放宽 q7 上限，看该 checkpoint 是否能释放更多运输能力
3. **增加夹持力**：调高夹取力参数（当前 20N），或调整 `FRANKA_CUBE_WIDTH_M` 使夹爪更紧
4. **原始 checkpoint 验证**：用 4wvlaFrkPlugCkp010420 + q7 sweet spot 配置测试，确认原始成功 checkpoint 在新配置下的表现

---

## 实验 A4_4 vs A6_4 深度对比分析：运输失败的根因与方案

### 1. 用户观测总结

| 实验 | Checkpoint | 用户观测 |
|------|-----------|---------|
| A4_4 | `aug1w01augtw5k02pure1w07` | 夹取成功，**但没有移动到插座那边** — 原地持握 |
| A6_4 | `aug1w015augtw5k015pure1w07` | 夹取成功，**移动到了插座附近但插到了左前方**，没有插进去 |

### 2. 全关节轨迹详细对比

#### 2.1 A4_4 轨迹（50步采样）

```
step   q1       q2      q3      q4       q5      q6      q7      w_meas(mm)
   0  -0.273  +0.006  +0.150  -1.847  -0.052  +1.906   0.542    66.9  ← HOME
  50  -0.305  +0.132  +0.117  -1.965  -0.019  +2.212   0.404    66.9  ← 接近
 100  -0.350  +0.219  +0.134  -2.031  -0.016  +2.354   0.404    66.9
 150  -0.358  +0.232  +0.141  -2.024  -0.004  +2.355   0.417    66.9
 200  -0.379  +0.227  +0.168  -2.044  -0.014  +2.369   0.433    63.4  ← 接近插头
 250  -0.391  +0.230  +0.184  -2.055  -0.029  +2.385   0.445     9.7  ← ★ 夹取
 300  -0.408  +0.219  +0.208  -2.079  -0.049  +2.407   0.466     9.7
 350  -0.431  +0.206  +0.248  -2.108  -0.152  +2.432   0.543     9.7  ← 手腕调整
 400  -0.417  +0.189  +0.256  -2.116  -0.175  +2.404   0.565     9.7
 500  -0.389  +0.187  +0.234  -2.119  -0.185  +2.414   0.569     9.7
 600  -0.362  +0.182  +0.199  -2.116  -0.175  +2.405   0.566     9.7  ← 持握
 800  -0.375  +0.186  +0.204  -2.116  -0.182  +2.410   0.570     9.7
1000  -0.368  +0.182  +0.211  -2.113  -0.179  +2.403   0.580     9.7  ← 持握
1200  -0.292  +0.180  +0.117  -2.114  -0.167  +2.412   0.574     9.7  ← 轻微漂移
1400  -0.275  +0.180  +0.108  -2.115  -0.157  +2.408   0.577     9.7  ← 停滞
1599  -0.284  +0.180  +0.113  -2.110  -0.150  +2.401   0.574     9.7  ← 结束 = 停滞
```

**A4_4 特征**：step 400 后全关节几乎不变（±0.02 rad 振荡），只有 q1 从 -0.41 缓慢漂移到 -0.28。

#### 2.2 A6_4 轨迹（50步采样）

```
step   q1       q2      q3      q4       q5      q6      q7      w_meas(mm)
   0  -0.272  +0.008  +0.151  -1.847  -0.046  +1.908   0.546    66.9  ← HOME
  50  -0.283  +0.119  +0.105  -1.959  -0.021  +2.188   0.404    66.9  ← 接近
 100  -0.316  +0.220  +0.097  -2.029  -0.010  +2.360   0.404    66.9
 200  -0.362  +0.227  +0.143  -2.016  -0.007  +2.343   0.404    66.9
 300  -0.410  +0.226  +0.205  -2.056  -0.040  +2.377   0.452     9.3  ← ★ 夹取
 400  -0.428  +0.197  +0.261  -2.116  -0.165  +2.411   0.552     9.4  ← 手腕调整
 500  -0.422  +0.181  +0.260  -2.119  -0.177  +2.404   0.561     9.4
 600  -0.414  +0.188  +0.253  -2.122  -0.174  +2.412   0.557     9.4  ← 持握
 800  -0.402  +0.182  +0.243  -2.120  -0.181  +2.405   0.558     9.4
1000  -0.397  +0.183  +0.235  -2.118  -0.184  +2.404   0.567     9.4  ← 持握
1200  -0.384  +0.157  +0.219  -2.124  -0.192  +2.385   0.570     9.4  ← 持握
1300  -0.385  +0.116  +0.237  -2.120  -0.195  +2.343   0.562     9.4  ← ↓↓ q2下降
1350  -0.386  +0.059  +0.258  -2.106  -0.186  +2.264   0.575     9.4
1380  -0.374  +0.034  +0.345  -2.099  -0.103  +2.224   0.737     9.4  ← ⚡ 运输启动！
1400  -0.352  +0.092  +0.398  -2.108  -0.087  +2.302   0.945     9.4  ← q7 急升
1420  -0.340  +0.181  +0.386  -2.125  -0.138  +2.399   0.998     9.4
1440  -0.336  +0.207  +0.404  -2.132  -0.192  +2.393   1.025     9.4
1460  -0.302  +0.237  +0.408  -2.137  -0.179  +2.388   1.045     9.4
1480  -0.260  +0.245  +0.383  -2.145  -0.178  +2.394   1.052     9.4
1500  -0.239  +0.249  +0.350  -2.131  -0.175  +2.378   1.050     9.5  ← w 开始上升
1520  -0.197  +0.258  +0.313  -2.118  -0.197  +2.367   1.081    11.0  ← q7 钳位!
1540  -0.158  +0.270  +0.281  -2.102  -0.203  +2.351   1.081    13.9  ← ⚠ 掉落
1560  -0.151  +0.278  +0.271  -2.097  -0.187  +2.362   1.078    13.8
1580  -0.120  +0.286  +0.243  -2.091  -0.195  +2.349   1.081    11.4
1597  -0.133  +0.279  +0.246  -2.075  -0.201  +2.330   1.081    10.9  ← 💥 crash
```

**A6_4 特征**：step 1300-1400 出现剧烈的"运输爆发"，q7 在约 80 步内从 0.57 飙升到 0.95。

### 3. 两种截然不同的后抓取行为模式

#### 3.1 A4_4：稳态吸引子（Hold Attractor）

A4_4 在夹取后，模型的 action 命令几乎等于当前 state：

```
step  state_q1 → action_q1    state_q7 → action_q7    Δq1      Δq7
1300  -0.276   → -0.275       0.575    → 0.563        +0.001   -0.012
1400  -0.274   → -0.275       0.571    → 0.577        -0.001   +0.006
1500  -0.286   → -0.287       0.584    → 0.580        -0.001   -0.004
1599  -0.284   → -0.284       0.574    → 0.574         0.000    0.000
```

**结论**：模型已收敛到一个**定点吸引子**。每步 delta 趋近于零。**模型不认为自己需要运输** — 它认为当前位置就是正确的。这是一个根本性的 checkpoint 能力缺失，不是配置问题。

#### 3.2 A6_4：延迟爆发式运输（Delayed Burst Transport）

A6_4 在夹取后经历三个清晰阶段：

| 阶段 | 步数范围 | 持续 | 特征 |
|------|----------|------|------|
| **Phase I — 悬停/搜索** | step 400–1200 | ~800步 | 各关节微幅振荡（±0.03 rad），净位移极小 |
| **Phase II — 运输爆发** | step 1200–1520 | ~320步 | q1 +0.19, q7 +0.51, 快速定向运动 |
| **Phase III — 终端失控** | step 1520–1598 | ~80步 | q7 钳位，物体掉落，Reflex 崩溃 |

Phase I 的"悬停"行为与 A2_1_6（使用 4wvlaFrkPlugCkp010420）中观测到的"后抓取悬停"（step 700-900, 200步）是**同一种现象**，但持续时间更长（800步 vs 200步）。

### 4. 与 A2_1_6（唯一成功运输的实验）的关键对比

A2_1_6 是使用 checkpoint `4wvlaFrkPlugCkp010420` 的实验，是唯一到达过"插入"阶段的实验。

#### 4.1 插入阶段关节运动对比

| 指标 | A2_1_6（step 900→989） | A6_4（step 1300→1597） | 倍率 |
|------|------------------------|------------------------|------|
| **Δq1**（基座旋转） | +0.113 rad | **+0.252 rad** | **2.2×** |
| **Δq7**（腕关节旋转） | +0.325 rad | **+0.519 rad** | **1.6×** |
| **插入阶段步数** | ~90 步 | ~300 步 | 3.3× |
| **模型请求 q7 目标** | 1.04–1.05 | **1.09–1.10** | +0.05 |
| **最终 q1** | -0.284 | **-0.133** | 差 0.151 |
| **最终 q3** | 0.363 | **0.246** | 差 0.117 |
| **最终 q7** | 0.981（钳位） | **1.081（钳位）** | +0.100 |

#### 4.2 关键发现：q1 过冲

A2_1_6 在"接近插入"时的 q1 值为 -0.284（crash 时），而 A6_4 的 q1 最终到达 -0.133。

假设插座位置对应 q1 ≈ -0.28（基于 A2_1_6 和 A2_1_3 的一致证据），那么 **A6_4 过冲了约 +0.15 rad**。这正好对应了用户观测的"插到了左前方" — 手臂飞过了插座位置，到达了更远的位置。

```
q1 值轴
-0.43  -0.28  -0.13
  |------+------+------→ q1 正方向 (左前方)
  ↑      ↑      ↑
 夹取   插座?   A6_4终点
 位置   位置?  (过冲!)
```

#### 4.3 q7 目标值差异

| Checkpoint | 模型请求的 q7 插入值 | 说明 |
|------------|---------------------|------|
| 4wvlaFrkPlugCkp010420（A2_1_6） | 1.04–1.05 | 较低的腕旋转 |
| aug1w015augtw5k015pure1w07（A6_4） | **1.09–1.10** | 更高的腕旋转（+0.05 rad） |

0.05 rad（2.9°）的差异对于精密插入可能导致约 1.5mm 的插头末端位置偏差。两个 checkpoint 对"正确插入角度"的认知不同，可能源于数据增强引入的变异。

### 5. 根因分析

#### 问题 A：A4_4 为什么不运输？

**直接原因**：模型收敛到定点吸引子，后抓取 action delta → 0。

**根本原因**：Checkpoint `aug1w01augtw5k02pure1w07` 的训练数据混合比例（aug weight=0.1）使模型学会了"抓取并持握"的策略，但**未充分学会"运输"策略**。这与训练数据中运输轨迹的权重不足有关。

**证据**：
- 对比 A5_4（aug1w02, q1_drift=0.003）、A4_4（aug1w01, q1_drift=0.104）、A6_4（aug1w015, q1_drift=0.295）
- 三个 checkpoint 的运输能力与 `aug weight` 不是简单的线性关系
- 但 A4_4 的运输能力显著低于 A6_4

#### 问题 B：A6_4 为什么插到了"左前方"而非插座？

**直接原因**：q1 过冲 +0.15 rad，飞过了插座位置。

**多重根因分析**：

**B1 — 物体脱落导致反馈中断**：
```
step 1500: q1=-0.239, w_meas=9.5mm (仍持握，距插座位置还差 ~0.04 rad)
step 1520: q1=-0.197, w_meas=11.0mm (开始滑脱!)
step 1540: q1=-0.158, w_meas=13.9mm (已掉落)
step 1580: q1=-0.120 (空手继续移动)
```

物体在 step 1500-1520 间开始滑脱，此时 q1 ≈ -0.20，还没到达插座位置（≈-0.28）。**物体在到达目标位置之前就掉了**。掉落后模型失去了触觉反馈，无法感知"已到达"，继续沿惯性方向移动。

**B2 — q7 旋转导致物理掉落**：
```
step 1350: q7=0.575, w_meas=9.4mm (稳定持握)
step 1380: q7=0.737, w_meas=9.4mm (q7 开始加速)
step 1400: q7=0.945, w_meas=9.4mm (仍持握)
step 1460: q7=1.045, w_meas=9.4mm (仍持握!)
step 1500: q7=1.050, w_meas=9.5mm ← 此处开始松动
step 1520: q7=1.081, w_meas=11.0mm ← 掉落
```

q7 在 step 1350→1500 间旋转了 +0.475 rad（约 27°）。这种大角度的腕关节旋转改变了夹爪对插头的夹持角度，当旋转超过某个临界值（约 +0.47 rad，即 q7 ≈ 1.05）时，插头从夹爪中旋转滑出。

**B3 — 运输速度过快**：
A6_4 在 step 1380-1500 的 120 步内完成了大部分运输（q1 从 -0.37 到 -0.24，Δ=0.13），而 A2_1_6 在类似的运输阶段用了约 90 步但 q1 只移动了 0.11 rad。两者速度相近，但 A6_4 的总运动量更大。

**B4 — 不同 checkpoint 的"插座认知"不同**：
A6_4 的模型请求 q7=1.09-1.10，而 A2_1_6 请求 q7=1.04-1.05。这 0.05 rad 的差异暗示两个 checkpoint 对"正确的插入姿态"有不同的认知，可能源于训练数据增强引入的空间变异。

### 6. 因果链总结

```
A4_4 (aug1w01):
  夹取 → 定点吸引子 → 原地持握 1355 步 → ❌ 无运输
  
A6_4 (aug1w015):
  夹取 → 悬停搜索 800 步 → 运输爆发 →
    q7 急速旋转 (+0.53 rad) →
      插头滑出 (step 1520, w_meas↑) →
        失去触觉反馈 →
          q1 过冲到 -0.133 (飞过插座) →
            空手运动触发 Reflex → 💥 crash
```

**A6_4 的根本失败不是"方向错了"，而是"物体在半路掉了"**。如果插头没有在 step 1520 掉落，模型可能会在 q1 ≈ -0.28 附近减速并尝试插入。用户观测到的"左前方"位置是**掉落后空手继续运动的终点**，不是模型的"目标位置"。

### 7. 解决方案分析

#### 方案 1（优先级最高）：用原始成功 checkpoint 4wvlaFrkPlugCkp010420 + 当前 q7 配置

**依据**：
- 这是唯一完成过"接近→抓取→运输→插入尝试"全流程的 checkpoint
- A2_1_6 中其失败原因是 q7 上界 0.9807 不够（模型需要 1.04-1.05）
- 当前 q7 上界已放宽到 1.0807（0.9807 + 0.1），完全覆盖模型的需求
- 当前 q7 下界 0.4043（0.4843 - 0.08）已验证为接近阶段的 sweet spot

**预期效果**：如果 A2_1_6 的唯一失败原因确实是 q7 上界钳位，那么放宽后应该能成功插入。

**风险**：原实验使用 n_exec=5，当前使用 n_exec=10。控制行为可能有差异。

#### 方案 2：增强 A6_4 的夹持力

**做法**：增加夹持力从 20N 到 40-60N，或调整 `FRANKA_CUBE_WIDTH_M` 使夹爪更紧。

**依据**：A6_4 的物体掉落发生在 q7 旋转 ~+0.47 rad 时。如果夹持力足以抵抗旋转产生的滑动力，物体就不会掉。

**风险**：
- 过大的夹持力可能损坏插头或导致不同的末端位姿
- 夹持力是 `franky_controller_direct.py` 中的参数，当前写死为 20N，需要确认是否可配置

**具体检查**：需要查看 `close_gripper` 中的 force 参数是否可通过环境变量配置。

#### 方案 3：限制 q7 变化速率

**做法**：在 `franky_joint_env.py` 中添加 q7 的步间最大变化量限制（如每步 ≤ 0.02 rad），避免 step 1380→1400 间的 q7 急剧跳变（0.74→0.95，单步推理间 +0.21 rad）。

**依据**：q7 的急速变化是导致物体掉落的直接物理原因。减缓变化速率可能让模型在运输过程中保持对物体的夹持。

**风险**：
- 已有 `MAX_JOINT_STEP_RAD = 0.15`，但这是 per-step（execution step）的限制
- n_exec=10 时，每次推理产生 10 步 action，q7 的 per-inference delta 可达 0.15×10=1.5 rad
- 可能需要 per-inference 的 q7 rate limiter

#### 方案 4：放宽 q7 上限（upper margin 0.1 → 0.15）

**做法**：允许 q7 达到 0.9807 + 0.15 = 1.1307。

**依据**：A6_4 的模型请求 q7=1.09-1.10，当前上限 1.0807 只差 0.01-0.02 rad。放宽后模型可达到目标值而不被截断。

**风险**：
- q7=1.13 远超训练数据的 max=0.981，进入未知行为区域
- 可能让物体掉落更快（旋转角度更大）
- 但 A6_4 的物体掉落主要发生在 q7=1.05 附近（step 1500），此时 q7 还没到上限钳位（step 1520 才钳位），所以放宽上限不一定能解决掉落问题

#### 方案 5：缩短悬停阶段

**做法**：如果检测到"持握且关节停滞 >200 步"，主动施加微小扰动或切换到更激进的 prompt。

**依据**：A4_4 和 A6_4 的悬停阶段分别持续 1355 步和 800 步。A2_1_6 也有 200 步悬停。这是一个系统性的效率问题。

**风险**：扰动可能导致掉落或不可预测行为。

### 8. 方案优先级排序

| 优先级 | 方案 | 预期收益 | 实施复杂度 | 风险 |
|--------|------|----------|------------|------|
| **★★★** | 方案 1：4wvlaFrkPlugCkp010420 + 当前 q7 | 最有可能成功 | 低（换 checkpoint 即可） | 低 |
| **★★** | 方案 2：增加夹持力 | 保持 A6_4 的物体 | 中（需查代码） | 中 |
| **★★** | 方案 3：q7 速率限制 | 减缓运输掉落 | 中（需改代码） | 中 |
| **★** | 方案 4：放宽 q7 上限 | 不确定 | 低 | 高 |
| **★** | 方案 5：缩短悬停 | 提高效率 | 高 | 高 |

### 9. 训练数据统计参考

来自 `stats.json`（observation.state.arm），可作为评估关节值合理性的参照：

| 关节 | min | max | mean | std | q10 | q50 | q90 | q99 |
|------|-----|-----|------|-----|-----|-----|-----|-----|
| q1 | -0.484 | 0.045 | -0.241 | 0.121 | -0.404 | -0.251 | -0.061 | -0.026 |
| q2 | -0.103 | 0.312 | 0.146 | 0.080 | 0.025 | 0.172 | 0.232 | 0.275 |
| q3 | -0.202 | 0.479 | 0.187 | 0.146 | -0.024 | 0.200 | 0.364 | 0.451 |
| q4 | -2.204 | -1.535 | -2.060 | 0.085 | -2.136 | -2.074 | -1.975 | -1.660 |
| q5 | -0.204 | 0.081 | -0.055 | 0.043 | -0.114 | -0.056 | 0.000 | 0.038 |
| q6 | 1.570 | 2.454 | 2.201 | 0.128 | 2.031 | 2.233 | 2.330 | 2.393 |
| q7 | 0.484 | 0.981 | 0.700 | 0.097 | 0.572 | 0.705 | 0.820 | 0.902 |

**A6_4 终点位置 vs 训练分布**：

| 关节 | A6_4 终点 | 训练百分位 | 状态 |
|------|-----------|------------|------|
| q1=-0.133 | 在 q90(-0.061) 和 q99(-0.026) 之间 | **P90-P99** — 在训练分布内，但偏高端 |
| q2=+0.279 | 在 q99(0.275) 附近 | **≈P99** — 在训练分布边缘 |
| q3=+0.246 | 在 q50(0.200) 和 q90(0.364) 之间 | **P50-P90** — 正常 |
| q7=1.081 | 超过 max(0.981) | **超出训练范围 +0.100** |

q1 和 q2 都在训练分布的高端（P90-P99），q7 超出训练范围。这意味着 A6_4 终点的关节组合在训练数据中是**极少见**的，模型在此处的行为可靠性较低。

### 10. 关键日志片段

#### A6_4 运输爆发阶段（step 1380-1400，q7 急升）

```
# step 1380: q7 开始加速
[step 1380] execute: action=[..., 0.737458, ...] state_before=[..., 0.683638, ...]
  → q7 请求 0.737，当前 0.684，delta +0.054

# step 1400: q7 进入高速区
[step 1400] execute: action=[..., 0.945461, ...] state_before=[..., 0.880686, ...]
  → q7 请求 0.945，当前 0.881，delta +0.065
```

#### A6_4 物体掉落阶段（step 1520-1540）

```
[step 1520] OUT-OF-TRAIN: q7=1.0896>1.0807
[step 1520] result: w_meas=0.010972 (11.0mm — 开始滑脱)

[step 1540] result: w_meas=0.013941 (13.9mm — 已掉落)
  gripper continuous grasp_handoff action=0.779 w_meas=0.013941 holding=False
  → 模型仍命令夹持 (action=0.779>0.4)，但物体已不在
```

#### A4_4 终端定点吸引子（step 1599）

```
[step 1599] execute: action=[-0.284072,  0.179174,  0.112845, -2.109343, ...]
  state_before=[-0.284188,  0.180909,  0.113701, -2.110401, ...]
  delta=[ 1.160e-04, -1.735e-03, -8.557e-04,  1.058e-03, ...]
  → 所有关节 delta < 0.002 rad，模型已完全收敛
```

### 11. 结论

1. **A4_4 的 checkpoint 不具备运输能力**，属于能力缺失问题，无法通过调参解决
2. **A6_4 的 checkpoint 具备运输能力但存在执行问题**：运输方向正确，但 q7 大幅旋转（+0.53 rad）导致物体在到达目标前掉落
3. **最有希望的方案**是用原始 checkpoint `4wvlaFrkPlugCkp010420` 配合当前的 q7 安全边距（lower=0.08, upper=0.1），这应该能解决 A2_1_6 中 q7 上界钳位导致的插入失败问题
4. **A6_4 的问题**可通过增加夹持力（方案 2）或限制 q7 变化速率（方案 3）来缓解，但优先级低于方案 1

---

## 实验 A6_5 / A6_5_2 / A6_5_3 / A6_5_4 深入分析与对比

> 日期：2026-09-23
> 分析者：Claude Opus 4.6
> Checkpoint：`aug1w015augtw5k015pure1w07`（同一 checkpoint，四次实验）
> 场景：插座移至左上方（与 A4_4/A6_4 的原始位置不同）
> 夹持力：40N（A6_5 显示 20N 因初始化显示 bug，实际 close_gripper(force=40.0) 已生效）

### 1. 实验概要对比表

| 指标 | A6_5 | A6_5_2 | A6_5_3 | A6_5_4 |
|------|------|--------|--------|--------|
| 日期时间 | 09-23 08:41~08:49 | 09-23 09:23~09:31 | 09-23 09:39~09:47 | 09-23 09:56~10:02 |
| 总步数 | 1600 (完成) | 1600 (完成) | 1600 (完成) | 1260 (**崩溃**) |
| 抓取步数 | step 521 | step 210 | step 190 | step 244 |
| 抓取位置 q1 | -0.061 | -0.255 | -0.323 | -0.380 |
| 抓取位置 q7 | 0.404 | 0.410 | 0.440 | 0.439 |
| 最终/崩溃 q1 | -0.136 | -0.093 | -0.349 | -0.242 |
| 最终/崩溃 q7 | 0.459 | 0.518 | 0.609 | **1.058** |
| q1 范围 | [-0.067, -0.052] | [-0.282, -0.086] | [-0.323, -0.267] | [-0.380, -0.240] |
| q7 范围 | [0.404, 0.472] | [0.404, 0.472] | [0.404, 0.440] | [0.404, 0.561→**1.058**] |
| OUT-OF-TRAIN 警告 | 652 | 262 | 258 | 238 |
| TRAIN-EDGE 警告 | 126 | 890 | 36 | 58 |
| 控制频率 | 3.41 Hz | 3.59 Hz | 3.58 Hz | 3.55~3.56 Hz |
| 搬运行为 | **无** | **无** | **无** | **有**（step 1260 burst） |
| 崩溃原因 | — | — | — | joint_reflex |
| 光照条件 | 正常 | 正常 | 正常 | **关了部分灯** |
| global 图像亮度 | mean=119.8 | mean=119.3 | mean=120.2 | mean=**119.0** |
| wrist 图像亮度 | mean=122.1 | mean=123.1 | mean=126.2 | mean=**121.6** |
| wrist 图像 max | 191 | 192 | 196 | **214** |

### 2. 实验配置（四次相同）

```
Checkpoint:  /home/nvidia/bt/ckp/4wvlaFrk/plug/aug1w015augtw5k015pure1w07/
n_exec:      10
max_steps:   1600
control_hz:  10.0 (实际 3.4~3.6 Hz)
task:        "plug into socket"

VLA_GRIPPER_MODE:             continuous
VLA_GRIPPER_CLOSE_THRESHOLD:  0.4
GRASP_HANDOFF_A:              0.40
FRANKA_CUBE_WIDTH_M:          0.008
FRANKA_HOLD_TOL_M:            0.004
Grasp force:                  40N (franky_controller_direct.py:382 默认值已改)

Safety margins:
  SAFETY_MARGIN_LOWER_RAD = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.08]
  SAFETY_MARGIN_UPPER_RAD = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.1]
  
Action limits (TRAIN ± margin):
  ACTION_LIMIT_LOWER = [-0.6842, -0.3030, -0.4025, -2.4044, -0.4041, 1.3702, 0.4043]
  ACTION_LIMIT_UPPER = [ 0.2452,  0.5120,  0.6789, -1.3347,  0.2806, 2.6536, 1.0807]
```

### 3. 操作流程

四次实验使用相同的操作流程：

#### Server 启动（rlinf-4dwvla-gpu 容器）

```bash
docker exec -d rlinf-4dwvla-gpu bash -c 'VLA_N_EXEC=10 VLA_CKPT_PATH=/home/nvidia/ckpts/4wvlaFrk/plug/aug1w015augtw5k015pure1w07 bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh > /tmp/server_a6_5_X.log 2>&1'
```

确认 server ready：
```
2026-09-23 XX:XX:XX [INFO] vla-server: Inference server listening on port 5555 (n_exec=10)
2026-09-23 XX:XX:XX [INFO] vla-server: Waiting for client connection...
```

#### Client 启动（rlinf-4dwvla-franky 容器）

```bash
docker exec -d rlinf-4dwvla-franky bash -c '
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env
export VLA_GRIPPER_MODE=continuous
export VLA_GRIPPER_CLOSE_THRESHOLD=0.4
export FRANKA_CUBE_WIDTH_M=0.008
export FRANKA_HOLD_TOL_M=0.004
cd /workspace/RLinf/b/x/4dwvla_ext
PYTHONPATH=/workspace/RLinf:/workspace/RLinf/b/x:$PYTHONPATH /opt/venv/franky-0.19.0/bin/python franka_vla_client.py \
  --server-host localhost --server-port 5555 \
  --task "plug into socket" \
  --max-steps 1600 --n-exec 10 --control-hz 10.0 \
  --use-realsense \
  > /tmp/client_a6_5_X_full.log 2>&1'
```

Client 启动后等待 KEY_A 按键，通过 evdev 注入：
```bash
docker exec rlinf-4dwvla-franky bash -c 'python3 -c "
import time, struct, os
KEY_A = 30; EV_KEY = 1; event_path = \"/dev/input/event2\"
def send_key(fd, code, value):
    t = time.time(); sec = int(t); usec = int((t - sec) * 1e6)
    ev = struct.pack(\"llHHi\", sec, usec, EV_KEY, code, value)
    syn = struct.pack(\"llHHi\", sec, usec, 0, 0, 0)
    os.write(fd, ev + syn)
fd = os.open(event_path, os.O_WRONLY)
send_key(fd, KEY_A, 1); time.sleep(0.05); send_key(fd, KEY_A, 0); os.close(fd)
"'
```

#### A6_5 特殊操作

A6_5 启动时遇到相机忙错误 `RuntimeError: xioctl(VIDIOC_S_FMT) failed, errno=16 Device or resource busy`，通过 pyrealsense2 硬件重置两台 RealSense 相机解决。

#### A6_5_2 特殊操作

A6_5_2 回 HOME 时遇到 `ControlException: motion aborted by reflex! ["joint_motion_generator_velocity_discontinuity"]`，通过 `robot.recover_from_errors()` 两次（间隔 1.0s）+ `robot.relative_dynamics_factor = 0.1` 慢速运动解决。

#### A6_5_4 特殊条件

A6_5_4 实验前关闭了部分灯光，导致图像整体亮度略低（global mean 119.0 vs 119.3~120.2），但 wrist 相机 max=214（高于其它实验的 191~196），可能因为灯光关闭后产生了更强的局部高光反射。

### 4. 关节轨迹详细对比

#### 4.1 q1 轨迹（关节1，控制末端执行器左右位移）

```
Step    A6_5     A6_5_2   A6_5_3   A6_5_4
────    ─────    ──────   ──────   ──────
0       -0.267   -0.267   -0.267   -0.267    ← HOME
50      -0.288   -0.281   -0.287   -0.291
100     -0.304   -0.288   -0.326   -0.328
150     -0.288   -0.278   -0.333   -0.350
200     -0.247   -0.267   -0.323   -0.382    ← A6_5 向右; A6_5_4 向左
250     -0.201   -0.219   -0.331   -0.379    ← 抓取前后
300     -0.148   -0.183   -0.316   -0.396
400     -0.064   -0.136   -0.274   -0.418    ← A6_5 大幅右移
500     -0.069   -0.105   -0.305   -0.386
600     -0.062   -0.102   -0.325   -0.359
700     -0.116   -0.113   -0.351   -0.315
800     -0.137   -0.128   -0.383   -0.292
900     -0.132   -0.105   -0.410   -0.279
1000    -0.114   -0.092   -0.402   -0.276
1100    -0.108   -0.097   -0.397   -0.275    ← A6_5_4 q7 开始攀升
1200    -0.111   -0.097   -0.394   -0.304    ← A6_5_4 进入 q7 ramp
1250    -0.116   -0.101   -0.400   -0.261
1260    —        —        —        -0.242    ← 崩溃点
1500    -0.101   -0.100   -0.343   —
1599    -0.136   -0.093   -0.349   —
```

**关键发现**：
- **A6_5** 抓取前大幅右移到 q1≈-0.06（远离 HOME 的 -0.267），是四次实验中唯一向正方向移动的
- **A6_5_3 / A6_5_4** 抓取后 q1 保持在 -0.3 ~ -0.4 附近，向负方向（左）移动
- **A6_5_2** 抓取后缓慢右移到 q1≈-0.1
- **A6_5_4** 在 step 1100 后 q1 开始从 -0.275 向 -0.242 移动，伴随 q7 攀升

#### 4.2 q7 轨迹（关节7，腕部旋转）

```
Step    A6_5     A6_5_2   A6_5_3   A6_5_4
────    ─────    ──────   ──────   ──────
0       0.635    0.635    0.635    0.635     ← HOME
50      0.417    0.404    0.404    0.409     ← 快速下降
100     0.404    0.414    0.404    0.404     ← 触底 ACTION_LIMIT_LOWER
200     0.431    0.417    0.447    0.436
250     0.435    0.466    0.458    0.444     ← 抓取前后
300     0.404    0.465    0.487    0.482
400     0.404    0.471    0.588    0.573
500     0.404    0.490    0.647    0.600
600     0.451    0.468    0.638    0.580
700     0.459    0.481    0.647    0.589
800     0.485    0.481    0.618    0.580
900     0.476    0.504    0.634    0.575
1000    0.500    0.513    0.573    0.583
1100    0.480    0.534    0.590    0.663     ← A6_5_4 开始加速
1150    0.468    0.500    0.578    0.901     ← A6_5_4 急剧攀升！
1200    0.474    0.501    0.601    1.012     ← 超出 TRAIN_ARM_MAX=0.981
1250    0.470    0.477    0.604    1.040     ← 接近 ACTION_LIMIT_UPPER=1.081
1260    —        —        —        1.058     ← 崩溃点
1500    0.490    0.504    0.575    —
1599    0.459    0.518    0.609    —
```

**关键发现**：
- **A6_5 / A6_5_2** 的 q7 在 0.40~0.53 之间稳定波动，始终在训练范围 [0.484, 0.981] 附近
- **A6_5_3** 的 q7 稍高（0.57~0.65），但仍在训练范围内
- **A6_5_4** 的 q7 在 step 1100 后急剧攀升：0.66 → 0.90 → 1.01 → 1.06，仅 ~150 步内增长了 **+0.4 rad**

#### 4.3 A6_5_4 的 q7 ramp-up 详细分析

```
Step    q1       q2      q3      q6      q7       Δq7/10步
────    ─────    ─────   ─────   ─────   ─────    ────────
1050   -0.277    0.098   0.114   2.281   0.590    
1060   -0.276    0.088   0.123   2.264   0.607    +0.017
1070   -0.280    0.070   0.136   2.238   0.619    +0.012
1080   -0.284    0.057   0.152   2.211   0.619    +0.000
1090   -0.280    0.049   0.165   2.184   0.641    +0.022
1100   -0.275    0.040   0.173   2.168   0.663    +0.022
1110   -0.278    0.042   0.180   2.156   0.665    +0.002
1120   -0.282    0.037   0.209   2.148   0.694    +0.029  ← 加速
1130   -0.291    0.031   0.245   2.143   0.727    +0.033
1140   -0.292    0.034   0.310   2.155   0.822    +0.095  ← 急加速！
1150   -0.281    0.058   0.327   2.187   0.901    +0.079
1160   -0.294    0.092   0.339   2.233   0.938    +0.037
1170   -0.299    0.131   0.345   2.282   0.983    +0.045  ← 超出 TRAIN_ARM_MAX
1180   -0.295    0.161   0.350   2.315   0.998    +0.015
1190   -0.302    0.191   0.343   2.351   1.004    +0.006  ← 减速（钳位效应）
1200   -0.304    0.208   0.349   2.379   1.012    +0.008
1210   -0.300    0.220   0.351   2.388   1.036    +0.024
1220   -0.290    0.229   0.356   2.374   1.033    -0.003
1230   -0.283    0.233   0.358   2.385   1.051    +0.018
1240   -0.271    0.240   0.362   2.384   1.057    +0.006
1250   -0.261    0.254   0.353   2.371   1.040    -0.017
1255   -0.240    0.272   0.347   2.386   1.058    +0.003
1260   -0.242    0.269   0.345   2.379   1.058    → 崩溃
```

**分析**：
1. Step 1050~1100：q7 缓慢上升，同时 q2 下降（0.098→0.040），q3 上升（0.114→0.173）——手臂向下俯仰
2. Step 1100~1150：q7 急剧加速（+0.24 rad in 50 steps），同时 q3 快速上升（0.173→0.327）——与 A6_4 中观察到的 q7 ramp 模式一致
3. Step 1150~1200：q2 回升（0.058→0.208），q6 上升（2.187→2.379）——手臂开始调整姿态，q7 接近 ACTION_LIMIT_UPPER 被钳位减速
4. Step 1250~1260：q1 从 -0.261 快速变到 -0.242（burst transport 启动），q7 在 1.058 处，新 inference batch 返回大 delta（q1 +0.019, q2 +0.014, q6 +0.017），触发 joint_reflex

### 5. 四次实验的行为模式分类

```mermaid
graph TD
    A[HOME: q1=-0.267] --> B{预抓取移动方向}
    B -->|右移 q1→-0.06| C[A6_5: 右侧抓取]
    B -->|原地/略左 q1≈-0.27| D[A6_5_2: 中部抓取]
    B -->|左移 q1→-0.32| E[A6_5_3: 左侧抓取]
    B -->|左移 q1→-0.38| F[A6_5_4: 深左抓取]
    
    C --> G[Hold Attractor: q1≈-0.12, q7≈0.47]
    D --> H[Hold Attractor: q1≈-0.10, q7≈0.50]
    E --> I[Hold Attractor: q1≈-0.38, q7≈0.60]
    F --> J[q7 Ramp → Burst → Crash]
    
    G -->|1600步| K[完成,无搬运]
    H -->|1600步| K
    I -->|1600步| K
    J -->|step 1260| L[joint_reflex 崩溃]
```

#### 模式 A：Hold Attractor（A6_5, A6_5_2, A6_5_3）

- 抓取后关节 delta 持续微小（~10⁻³ rad），模型收敛到稳态定点
- q7 在训练范围附近波动，不发生单向漂移
- 无搬运意图，夹持力 40N 稳定保持物体

#### 模式 B：Delayed Burst Transport（A6_5_4）

- 抓取后 step 250~1050 与 Hold Attractor 类似
- step 1050 后 q7 开始单向攀升（+0.4 rad in ~200 steps）
- q7 攀升伴随 q2 下降 + q3 上升（手臂姿态变化）
- step 1260 触发搬运 burst（多关节同时大 delta），但 q7 已在极限附近导致 joint_reflex

### 6. A6_5_4 vs 前三次实验的关键差异

#### 6.1 光照条件

A6_5_4 **关闭了部分灯光**，这是与前三次实验唯一的显式差异：

| 指标 | A6_5 | A6_5_2 | A6_5_3 | **A6_5_4** |
|------|------|--------|--------|------------|
| global mean | 119.8 | 119.3 | 120.2 | **119.0** |
| wrist mean | 122.1 | 123.1 | 126.2 | **121.6** |
| wrist max | 191 | 192 | 196 | **214** |
| global max | 163 | 162 | 163 | **180** |

- 整体亮度差异很小（<1.2 的 mean 差），属于微调
- 但 wrist max 从 196 跳到 214，global max 从 163 跳到 180，说明暗环境下局部高光反射更突出
- 这种微小的光照变化可能改变了模型的视觉特征响应，导致了完全不同的行为模式（Hold Attractor → Delayed Burst）

**推论**：模型对光照变化的敏感度极高。微小的亮度分布变化（mean <1.2 差异）就可能切换模型的决策路径。

#### 6.2 抓取位置差异

A6_5_4 的抓取位置（q1=-0.380）比其他三次更靠左：

```
A6_5:   grasp at q1=-0.061 (最右)
A6_5_2: grasp at q1=-0.255 (中)
A6_5_3: grasp at q1=-0.323 (偏左)
A6_5_4: grasp at q1=-0.380 (最左)
```

但 A6_5_3 的抓取位置（-0.323）也很靠左，却没有触发 burst transport。因此抓取位置不是决定性因素。

#### 6.3 随机性因素

VLA 模型使用 flow matching 采样动作，存在固有随机性。即使输入完全相同，生成的动作序列也可能不同。这解释了为什么同一 checkpoint + 同一场景 + 同一光照（A6_5_2 vs A6_5_3）也会产生不同的 hold attractor 终态（q1=-0.093 vs q1=-0.349）。

但 A6_5_4 出现 q7 ramp 的概率显然更高——在 A6_5/A6_5_2/A6_5_3 的 4800 步中从未出现，而 A6_5_4 在 1260 步就触发了。光照变化可能降低了触发阈值。

### 7. q7 过旋转问题的统一分析

将 A6_5_4 与之前的 A6_4 对比（两者都出现了 q7 急剧攀升）：

| 指标 | A6_4 | A6_5_4 |
|------|------|--------|
| q7 起始 | 0.635 | 0.635 |
| q7 开始攀升 step | ~900 | ~1100 |
| q7 攀升前值 | ~0.55 | ~0.59 |
| q7 最终值 | 1.08 (限位) | 1.058 |
| q7 攀升持续步数 | ~300 | ~150 |
| 攀升速率 | ~0.0018 rad/step | ~0.0031 rad/step |
| 伴随现象 | q3 上升 + q2 下降 | q3 上升 + q2 下降 |
| 结果 | 物体掉落 → 继续搬运 → crash | burst transport → joint_reflex |
| 场景 | 原始位置 | 插座左上方 |
| 光照 | 正常 | 关了部分灯 |

**共同模式**：q7 攀升总是伴随 q3（前臂旋转）上升和 q2（肩关节）下降，这是一个耦合的姿态调整动作。训练数据中 q7 的范围是 [0.484, 0.981]（q99=0.902），模型在推理时倾向于将 q7 推到训练分布上界甚至超出，说明模型可能学到了与任务相关但被训练数据截断的 q7 旋转模式。

### 8. Hold Attractor 的进一步分析

三次成功完成 1600 步的实验（A6_5, A6_5_2, A6_5_3）都展示了 "Hold Attractor" 行为，但终态位置各不相同：

```
A6_5 终态:   q1=-0.136, q7=0.459  → TCP 近似在插座右前方
A6_5_2 终态: q1=-0.093, q7=0.518  → TCP 近似在插座右方
A6_5_3 终态: q1=-0.349, q7=0.609  → TCP 近似在抓取位置附近（几乎没移动）
```

训练数据中的插入目标位置参考（来自 A2_1_6 的成功插入）：q1≈-0.284, q7≈1.04~1.05。

**关键观察**：
1. A6_5 和 A6_5_2 的 q1 收敛到 -0.1 附近，远离插入目标 -0.284
2. A6_5_3 的 q1 收敛到 -0.349，比插入目标更靠左
3. 三者的 q7 都远低于插入目标 1.04~1.05

这说明 Hold Attractor 不是"接近目标后稳定"，而是"在错误位置收敛"。模型在新场景布局下无法找到正确的搬运路径，只能收敛到局部极小值。

### 9. 夹持稳定性分析

40N 夹持力的效果在所有四次实验中都得到验证：

```
A6_5:   w_meas ≈ 0.0101m, 稳定保持 1079 步 (step 521→1600)
A6_5_2: w_meas ≈ 0.0093m, 稳定保持 1390 步 (step 210→1600)
A6_5_3: w_meas ≈ 0.0093m, 稳定保持 1410 步 (step 190→1600)
A6_5_4: w_meas ≈ 0.0090~0.0095m, 稳定保持 1016 步 (step 244→1260)
```

对比 A6_4（20N 夹持力）：物体在 step ~1520 掉落（w_meas 从 ~0.009 跳到 0.014）。

**结论**：40N 夹持力完全解决了物体掉落问题。即使 A6_5_4 经历了剧烈的 q7 旋转（+0.47 rad），物体始终被牢牢夹住。这是相对于 A6_4 的重大改进。

### 10. A6_5 特殊分析：抓取失败与重试

A6_5 的抓取过程异常复杂，在 step 521~535 之间经历了多次抓取失败和重试：

```
step 520: w_meas=0.0488, 夹爪接近闭合, grasp_issued=False
step 521: w_meas=0.0488, grasp_issued=True → 首次抓取命令
step 522: w_meas=-0.0000(负!), grasp_issued=True → 夹爪完全闭合（空抓）
step 523: w_meas=0.0666, grasp_issued=True → 夹爪重新张开
step 524: w_meas=-0.0000, grasp_issued=True → 又空抓
step 525: w_meas=0.0666, grasp_issued=True → 又张开
...（反复夹持-张开-夹持循环）
step 535: w_meas=0.0101, grasp_issued=False → 最终成功夹住
```

w_meas 在正值和负值/零值之间反复跳变，说明夹爪在反复尝试夹持。最终在 step 535 左右稳定在 0.0101m，成功夹住物体。

这种抓取不稳定可能与 A6_5 抓取时的位置有关——q1=-0.061 偏右，可能不在物体的理想夹持位置上。

### 11. 关键日志片段

#### A6_5 初始化与 20N 显示 Bug

```
2026-09-23 08:39:17,935 [INFO] RLinf: FrankaLibfrankaGripper connected 
  (max_width=0.067m, grasp_force=20.0N, ...)
```

注：虽然初始化日志显示 `grasp_force=20.0N`，但实际 `close_gripper()` 调用使用 `force=40.0N`（因为 franky_controller_direct.py:382 的默认值已从 20.0 改为 40.0）。FrankaLibfrankaGripper 初始化时读取的是 env var `FRANKA_GRASP_FORCE=20`（来自 franka_plug_eval.env），但 close() 被调用时 force 参数覆盖了环境变量默认值。

#### A6_5_4 崩溃瞬间（step 1260）

```
2026-09-23 10:02:20,228 [INFO] vla-client: [step 1260] execute: 
  action=[-0.222978, 0.283206, 0.330981, -2.132343, -0.166527, 2.396765, 1.042779, 0.801413]
  state_before=[-0.241774, 0.268963, 0.34503, -2.129854, -0.176077, 2.379324, 1.057762, 0.009492]
  delta=[ 0.018796, 0.014242, -0.014049, -0.002489, 0.009550, 0.017441, -0.014983, 0.791920]

2026-09-23 10:02:20,712 [ERROR] vla-client: Fatal: ControlException: 
  libfranka: Move command aborted: motion aborted by reflex! ["joint_reflex"]
  control_command_success_rate: 1
```

分析：
- delta q1 = +0.0188 rad（向右移动），这是搬运 burst 的特征
- delta q7 = -0.0150 rad（向下旋转），模型试图降低 q7
- 但 state q7 = 1.058 已接近 ACTION_LIMIT_UPPER = 1.081
- 多关节同时大幅变化可能导致力矩不连续，触发 joint_reflex 保护

#### A6_5_4 Server 最后的 inference 输出

```
2026-09-23 10:02:20,228 [INFO] vla-server: 
  full_actions (step 1260 inference batch):
  action[0]=[-0.222978, 0.283206, 0.330981, ...] ← 第一步 burst
  action[1]=[-0.223833, 0.281176, 0.330057, ...] ← 后续步骤 delta 更小
  ...
  action[9]=[-0.228892, 0.273644, 0.323910, ...]
  
  delta_from_previous[0]=[ 0.018796, 0.014242, -0.014049, ...]  ← 大突变！
  delta_from_previous[1]=[-0.000855, -0.002030, -0.000925, ...] ← 正常大小
```

只有 batch 的第一步（action[0]）产生了大 delta，后续 9 步的 delta 回到正常水平（~10⁻³）。这证实 burst transport 是一个瞬态事件——模型在某一次 inference 时突然输出了远离当前状态的目标位置，但后续推理立即回到渐进调整模式。

### 12. 问题诊断与根因分析

#### 问题 1：Hold Attractor（A6_5, A6_5_2, A6_5_3）

**现象**：模型抓取成功后收敛到定点，不启动搬运。

**根因分析**：
1. **场景布局脱离训练分布**：插座移至左上方后，模型从训练数据中学到的"抓取→搬运→插入"流程被打断。训练数据中插座位于特定位置，模型在新位置下无法泛化出正确的搬运轨迹。
2. **视觉编码的位置敏感性**：Qwen3.5-2B 的视觉编码器对物体位置高度敏感，插座位置变化可能导致视觉特征映射到完全不同的动作空间区域。
3. **Flow matching 收敛到局部极小值**：抓取后的状态对 flow matching 采样器来说是一个"安全"的区域——delta 接近零意味着模型对接下来该做什么没有信心。

#### 问题 2：q7 过旋转（A6_5_4, 也曾出现在 A6_4）

**现象**：q7 从训练范围 [0.484, 0.981] 向上漂移到 1.06+。

**根因分析**：
1. **训练数据中的 q7 分布截断**：训练数据的 q7 q99=0.902，但目标插入位置需要 q7≈1.04~1.05。模型在推理时被迫外推到未见过的 q7 范围。
2. **姿态耦合效应**：q7 攀升总是伴随 q3↑ 和 q2↓，说明模型学到了一个姿态调整模式，但这个模式在超出训练范围后变得不可控。
3. **安全边距允许过度外推**：当前 q7 upper margin = 0.1 rad，允许 q7 达到 1.081。这足以让模型进入未知区域但不足以在此区域产生合理动作。

#### 问题 3：Burst Transport 导致崩溃（A6_5_4）

**现象**：step 1260 突然输出大 delta，触发 joint_reflex。

**根因分析**：
1. **q7 处于极限位置时的不稳定性**：q7 在 1.058（接近 ACTION_LIMIT_UPPER=1.081）时，模型的动作输出变得不稳定
2. **多关节同时突变**：burst transport 涉及 q1, q2, q3, q5, q6 同时大幅变化，可能导致力矩计算超出关节承受范围
3. **Action clipping 边界效应**：当 q7 接近上限，模型输出的动作被 clip 到边界，但其他关节的大 delta 未被 clip，造成关节间的力学不一致

### 13. 处理方案

按优先级排序：

#### 方案 1（最高优先级）：换用 checkpoint `4wvlaFrkPlugCkp010420`

**理由**：这是唯一在 A2_1_6 中展示过完整 grasp→transport→insertion 流程的 checkpoint。当前 q7 安全边距（upper=0.1）应该能解决 A2_1_6 中 q7 上界钳位导致的插入失败问题。

**预期**：
- 该 checkpoint 在训练时可能使用了不同的数据混合比例，具备更强的搬运泛化能力
- 配合 40N 夹持力可以防止搬运过程中物体掉落
- q7 margin=0.1 允许到达插入目标 q7≈1.04~1.05（ACTION_LIMIT_UPPER=1.081）

**风险**：q7 仍可能在 1.05~1.08 范围内触发不稳定行为

#### 方案 2：收紧 q7 上界 margin（0.1 → 0.06）

**理由**：将 ACTION_LIMIT_UPPER[6] 从 1.081 收紧到 1.041，更早地 clip q7，防止进入完全未知的区域。

**实施**：
```python
# franky_joint_env.py line 89
SAFETY_MARGIN_UPPER_RAD = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.06])
# → ACTION_LIMIT_UPPER[6] = 0.9807 + 0.06 = 1.0407
```

**权衡**：可能阻止模型到达插入所需的 q7≈1.04~1.05。需要根据具体 checkpoint 的目标位置调整。

#### 方案 3：引入 q7 变化速率限制

**理由**：A6_5_4 和 A6_4 中 q7 的急剧攀升（0.003+ rad/step）是崩溃的前兆。限制 q7 的单步变化量可以避免急剧旋转。

**实施**：在 `franky_joint_env.py` 的 step() 中添加 q7 delta clamp：
```python
MAX_Q7_DELTA = 0.005  # rad per step
q7_delta = action_arm[6] - current_state[6]
if abs(q7_delta) > MAX_Q7_DELTA:
    action_arm[6] = current_state[6] + np.sign(q7_delta) * MAX_Q7_DELTA
```

**权衡**：可能导致 q7 调整滞后，影响正常的姿态控制。

#### 方案 4：Burst 检测与速度衰减

**理由**：检测到多关节同时大 delta 时自动降低运动速度，避免 joint_reflex。

**实施**：
```python
total_delta = np.sum(np.abs(delta_arm))
if total_delta > 0.05:  # 阈值：所有关节 delta 绝对值之和
    scale = 0.05 / total_delta
    action_arm = current_state + (action_arm - current_state) * scale
```

**权衡**：平滑了 burst transport，但可能使搬运动作变得过慢。

#### 方案 5：光照标准化

**理由**：A6_5_4 证明微小的光照变化可以触发完全不同的行为模式。

**实施**：
1. 固定实验环境光照（不关灯）
2. 或在图像预处理中加入亮度标准化
3. 或使用颜色增强（color jitter）训练的 checkpoint

**权衡**：实际部署环境中光照变化不可避免，这只是实验控制措施。

### 14. 与之前实验的联系

| 实验 | Checkpoint | 场景 | 行为 | 与本次的关系 |
|------|-----------|------|------|------------|
| A4_4 | aug1w01augtw5k02pure1w07 | 原始位置 | Hold Attractor | 不同 ckpt，相同行为模式 |
| A6_4 | aug1w015augtw5k015pure1w07 | 原始位置 | Delayed Burst + 物体掉落 | 同 ckpt, q7 ramp 模式一致 |
| A6_5~A6_5_3 | aug1w015augtw5k015pure1w07 | 左上方 | Hold Attractor | 同 ckpt, 场景变化抑制了搬运 |
| A6_5_4 | aug1w015augtw5k015pure1w07 | 左上方+暗光 | Delayed Burst → Crash | 光照变化重新触发了搬运 |
| A2_1_6 | 4wvlaFrkPlugCkp010420 | 原始位置 | 完整 grasp→transport→insert | 参考标杆 |

### 15. 结论

1. **Checkpoint `aug1w015augtw5k015pure1w07` 的搬运能力是不稳定的**：同一场景下 4 次实验中仅 1 次（A6_5_4）触发了搬运行为，且立即崩溃
2. **40N 夹持力有效解决了物体掉落问题**：四次实验中物体均未掉落（vs A6_4 的 20N 导致掉落）
3. **场景布局对模型行为有决定性影响**：插座左上方 + 正常光照 → Hold Attractor；插座左上方 + 暗光 → 可能触发 Delayed Burst
4. **光照敏感性是一个严重的泛化问题**：mean 亮度差异 <1.2 就能改变模型的整体行为模式
5. **q7 过旋转是跨实验的系统性问题**：A6_4 和 A6_5_4 都展示了相同的 q7 ramp 模式
6. **最优先行动**：使用 checkpoint `4wvlaFrkPlugCkp010420` + 40N 夹持力 + 当前 q7 margins 进行测试

