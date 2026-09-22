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

