# set franka home
_Exported on 9/20/2026 at 10:45:34 GMT+8 from Cursor (3.15.19)_

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

