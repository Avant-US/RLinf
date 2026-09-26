# franka_4dwvla_client.py 使用说明

4DWVLA (InternVLA-A1.5) 实机 Franka 评估客户端. 基于 `franky_ext` 安全基础设施,
通过 IPC 连接远端 GPU 上的推理服务端, 读取机械臂与相机观测, 发送推理请求,
接收关节空间动作并在 8 层安全体系下执行.

---

## 1. 系统架构

```
                  ┌──────────────────────────────────────────┐
                  │         GPU 容器 (rlinf-4dwvla-gpu)       │
                  │                                          │
                  │  vla_inference_server.py                  │
                  │  ┌──────────────────────────────────┐    │
                  │  │ 4DWVLA (InternVLA-A1.5) model    │    │
                  │  │  ├─ Qwen3.5-2B VLM backbone      │    │
                  │  │  ├─ action expert (flow matching) │    │
                  │  │  └─ FK keypoint history           │    │
                  │  └──────────────────────────────────┘    │
                  │        ▲ TCP:5555 (multiprocessing)       │
                  └────────┼─────────────────────────────────┘
                           │ IPC (pickle over TCP)
                  ┌────────┼─────────────────────────────────┐
                  │        ▼                                  │
                  │  franka_4dwvla_client.py                  │
                  │  ┌────────────────────────────────────┐  │
                  │  │ VLAEvalController                  │  │
                  │  │  ├─ IPC client (send obs / recv act)│  │
                  │  │  ├─ ExecutedStateBuffer (修复 A)     │  │
                  │  │  ├─ action queue (chunk → per-step) │  │
                  │  │  └─ control-rate monitor (修复 B)    │  │
                  │  ├────────────────────────────────────┤  │
                  │  │ FrankaController                   │  │
                  │  │  ├─ franky.Robot (joint-space)     │  │
                  │  │  ├─ FrankaLibfrankaGripper         │  │
                  │  │  ├─ motion guard (TCP fence)       │  │
                  │  │  ├─ watchdog thread (50 Hz)        │  │
                  │  │  └─ collision behavior tightening  │  │
                  │  ├────────────────────────────────────┤  │
                  │  │ CameraCapture (pyrealsense2)       │  │
                  │  │  ├─ global camera (640x480 RGB)    │  │
                  │  │  └─ wrist camera  (640x480 RGB)    │  │
                  │  └────────────────────────────────────┘  │
                  │       Franky 容器 (rlinf-4dwvla-franky)   │
                  └───────────────┬───────────────────────────┘
                                  │ libfranka FCI (TCP:1337)
                           ┌──────┴──────┐
                           │  Franka FR3  │
                           │  (firmware   │
                           │   5.10.0)    │
                           └─────────────┘
```

### 1.1 数据流

每个控制步 (control step) 的数据流:

```
1. 读观测: FrankaController.get_state() → 7D joint angles + 1D gripper width
2. 读图像: CameraCapture.get_frames() → {global: (480,640,3), wrist: (480,640,3)}
3. 发推理请求: VLAEvalController → multiprocessing.connection → 服务端
   请求内容: {images, state, state_history, task, protocol: 2}
4. 收动作块: 服务端返回 {status: "ok", actions: [[8D] × n_exec]}
5. 逐步执行: action_queue 中每步取一个 8D 动作
   ├─ action[:7] → check_action_safety (L1-L3) → FrankaController.move_joints
   └─ action[7]  → 夹爪 close/open 决策
6. 记录历史: ExecutedStateBuffer.record(arm_q7)  → 下次推理时一并发送
```

### 1.2 动作格式

服务端返回的每个动作为 8 维向量:

| 维度    | 含义            | 单位    | 备注                          |
|---------|-----------------|---------|-------------------------------|
| `[0:7]` | 绝对关节角度    | rad     | FR3 q1-q7, absolute          |
| `[7]`   | 夹爪指令        | [0, 1]  | 1 = close, ~0 = open         |

---

## 2. 前置条件

### 2.1 硬件

| 项目         | 要求                                             |
|--------------|--------------------------------------------------|
| 机械臂       | Franka Research 3 (FR3v2.1), firmware >= 5.10.0  |
| 夹爪         | Franka Hand (原装平行夹爪)                        |
| 相机         | 2 台 Intel RealSense (global + wrist)             |
| GPU 主机     | CUDA GPU, 显存 >= 16 GB (加载 4DWVLA checkpoint) |
| 网络         | GPU 主机与 Franky 主机在同一网络, TCP:5555 可达    |

### 2.2 软件

**Franky 容器 (客户端):**

| 包              | 版本要求               | 安装方式                         |
|-----------------|------------------------|----------------------------------|
| Python          | 3.10+                  | 容器内已有                       |
| franky          | 0.19.0                 | `/opt/venv/franky-0.19.0`        |
| numpy           | >= 1.24                | pip                              |
| pyrealsense2    | >= 2.50                | pip (仅 `--use-realsense` 时需要) |
| RLinf (可选)    | `rlinf` package on path| 仅键盘控制需要 (`KeyboardListener`)|

**GPU 容器 (服务端):**

| 包                 | 版本要求    | 备注                        |
|--------------------|-------------|-----------------------------|
| InternVLA-A1.5     | 同 4WVLA repo| `pip install -e /workspace/4WVLA` |
| torch              | >= 2.10     | CUDA 版本                    |
| transformers       | 5.2.0       | 需打 patch (见 CLAUDE.md)    |
| flash-attn         | 2.8.3       | `--no-build-isolation`      |

### 2.3 目录结构

```
/workspace/RLinf/b/x/
├── franky_ext/                        # franky_ext 安全基础设施
│   ├── franka_4dwvla_client.py        # ← 本文件
│   ├── motion_limits.py               # 安全常数与函数
│   ├── franka_libfranka_gripper.py    # Franka Hand 夹爪驱动
│   ├── dsplug/
│   │   ├── home_pose.json             # HOME 关节位置
│   │   └── home_pose.py               # HOME 加载器
│   └── ...
├── 4dwvla_ext/
│   ├── vla_inference_server.py        # GPU 推理服务端
│   └── configs/
│       ├── franka_plug_eval.env       # 硬件/夹爪环境变量配置
│       ├── docker_run_4dwvla_gpu.sh   # GPU 容器启动脚本
│       ├── docker_run_4dwvla_franky.sh# Franky 容器启动脚本
│       ├── launch_gpu_server.sh       # 服务端启动脚本
│       └── launch_franky_client.sh    # 客户端启动脚本 (参考)
└── ...
```

---

## 3. 快速开始

### 步骤 1: 启动 GPU 容器并运行推理服务端

```bash
# 宿主机上启动 GPU 容器
bash /workspace/RLinf/b/x/4dwvla_ext/configs/docker_run_4dwvla_gpu.sh

# 容器内启动服务端
source /opt/venv/4dwvla/bin/activate
python /workspace/RLinf/b/x/4dwvla_ext/vla_inference_server.py \
    --ckpt-path /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp041680 \
    --schema-path /workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml \
    --kpt-meta-path /workspace/RLinf/b/d/frk1/plug/keypoints_meta.json \
    --urdf-path /workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf \
    --port 5555
```

等待输出 `Inference server listening on port 5555` 表示就绪.

### 步骤 2: 启动 Franky 容器并运行客户端

```bash
# 宿主机上启动 Franky 容器
bash /workspace/RLinf/b/x/4dwvla_ext/configs/docker_run_4dwvla_franky.sh

# 容器内: source 硬件配置 (相机序列号, 夹爪参数等)
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env

# 容器内: 激活 franky venv 并运行客户端
source /opt/venv/franky-0.19.0/bin/activate
python /workspace/RLinf/b/x/franky_ext/franka_4dwvla_client.py \
    --robot-ip 172.16.0.2 \
    --server-host localhost \
    --task "plug into socket" \
    --use-realsense
```

### 步骤 3: 操作

启动后程序进入键盘等待状态, 操作流程:

1. 将场景摆放到初始位姿
2. 按 **`a`** 键开始执行
3. 观察机械臂运动, 随时可按 **`r`** 中止
4. 任务完成后按 **`c`** (成功) 或 **`b`** (失败)
5. 重新摆放场景, 按 **`a`** 开始下一轮

---

## 4. 分级验证流程

> 必须按顺序 Level 0 → 1 → 2 → 3 逐级通过, 不可跳级.

### Level 0: Dry Run (通信测试, 机器人不动)

```bash
python /workspace/RLinf/b/x/franky_ext/franka_4dwvla_client.py \
    --server-host <gpu_host> \
    --task "plug into socket" \
    --max-steps 250 \
    --dry-run
```

**通过标准:**
- 服务端日志中 `his_len` 按 `0 → 10 → 20 → ...` 递增
- 服务端日志中 `full_chunk_grip` 每次包含 50 个值
- 客户端日志无报错

### Level 1: 保守实机测试 (30 步, 5 Hz)

> **握住 E-Stop!**

```bash
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env
python /workspace/RLinf/b/x/franky_ext/franka_4dwvla_client.py \
    --robot-ip 172.16.0.2 \
    --server-host <gpu_host> \
    --task "plug into socket" \
    --use-realsense \
    --max-steps 30 \
    --control-hz 5
```

**通过标准:**
- 无 MOTION GUARD TRIP
- 无空中夹爪误闭合
- 运动轨迹符合预期方向

### Level 2: 扩展测试 (300 步, 10 Hz)

```bash
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env
python /workspace/RLinf/b/x/franky_ext/franka_4dwvla_client.py \
    --robot-ip 172.16.0.2 \
    --server-host <gpu_host> \
    --task "plug into socket" \
    --use-realsense \
    --max-steps 300 \
    --control-hz 10
```

**通过标准:**
- 日志中至少出现一次 `gripper action → close`
- 夹爪宽度在 20 步内开始下降
- 运行结束后腕部相机可见手指夹住插头

### Level 3: 正式评估 (多 episode, 600 步)

```bash
source /workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env
python /workspace/RLinf/b/x/franky_ext/franka_4dwvla_client.py \
    --robot-ip 172.16.0.2 \
    --server-host <gpu_host> \
    --task "plug into socket" \
    --use-realsense \
    --max-steps 600 \
    --control-hz 10
```

键盘模式下每轮按 `a` 开始、`c`/`b` 结束, 循环执行多个 episode.

---

## 5. 命令行参数

| 参数                       | 默认值        | 说明                                                  |
|----------------------------|---------------|-------------------------------------------------------|
| `--robot-ip`               | `172.16.0.2`  | Franka 控制器 IP 地址                                  |
| `--server-host`            | `localhost`   | GPU 推理服务端主机名或 IP                               |
| `--server-port`            | `5555`        | 推理服务端端口                                         |
| `--task`                   | **(必填)**    | 任务指令文本, 如 `"plug into socket"`                   |
| `--n-exec`                 | `10`          | 每次推理取动作块中的前 N 个执行, **必须与服务端一致**    |
| `--control-hz`             | `10.0`        | 控制循环频率 (Hz), 每步执行后 sleep `1/control_hz` 秒   |
| `--max-steps`              | `300`         | 单 episode 最大步数                                    |
| `--use-realsense`          | `false`       | 启用 RealSense 相机采集                                |
| `--global-camera-serial`   | `$RS_GLOBAL_SERIAL`| 全局相机序列号; 缺省使用环境变量                  |
| `--wrist-camera-serial`    | `$RS_WRIST_SERIAL` | 腕部相机序列号; 缺省使用环境变量                  |
| `--log-dir`                | `$VLA_LOG_DIR` 或 `./logs`| 日志文件输出目录                          |
| `--dry-run`                | `false`       | 空跑模式: 不连接机器人, 用 HOME 位姿代替观测            |
| `--no-keyboard`            | `false`       | 禁用键盘控制, 直接运行至 max-steps 或 Ctrl-C            |

### 关键约束

- **`--n-exec` 必须与服务端 `--n-exec` 相同.** 服务端按此值截取动作块, 客户端按此值
  填充 action queue. 不匹配会导致 state history 时钟漂移.
- **`--task` 必须与训练时使用的指令文本一致.** 不同措辞会导致 VLM 编码不同.
- **`--dry-run` 模式不需要 `franky` 库**, 可在任何机器上运行通信测试.

---

## 6. 环境变量

在运行客户端前 source `franka_plug_eval.env` 可配置以下变量:

### 6.1 相机

| 变量                | 示例值          | 说明                          |
|---------------------|-----------------|-------------------------------|
| `RS_GLOBAL_SERIAL`  | `250222073513`  | 全局相机序列号                 |
| `RS_WRIST_SERIAL`   | `420122070525`  | 腕部相机序列号                 |

### 6.2 夹爪

| 变量                         | 默认值  | 说明                                            |
|------------------------------|---------|------------------------------------------------|
| `VLA_GRIPPER_CLOSE_THRESHOLD`| `0.5`   | 二值模式下, `action[7] >= 此值` 时闭合            |
| `VLA_GRIPPER_CLOSE_IF_ABOVE` | `1`     | `1` = action[7] >= threshold 时 close; `0` = 反向 |
| `FRANKA_CUBE_WIDTH_M`        | `0.010` | 夹持物体宽度 (m), 用于 `gripper_holding()` 判定   |
| `FRANKA_HOLD_TOL_M`          | `0.008` | 持有判定容差 (m)                                  |
| `FRANKA_GRASP_FORCE`         | `20`    | 力控抓取力 (N)                                    |
| `FRANKA_GRIPPER_MAX_WIDTH_M` | `0.080` | 夹爪最大宽度 (libfranka 报告值, 非卡尺值)          |

### 6.3 日志

| 变量          | 默认值                     | 说明              |
|---------------|----------------------------|-------------------|
| `VLA_LOG_DIR` | `<script_dir>/logs`        | 日志文件输出目录   |

---

## 7. 安全体系

8 层安全层级, 从软件到硬件逐级收紧:

```
         软件安全                             硬件安全
   ┌─────────────────────────┐       ┌──────────────────────┐
   │ L1  硬关节限位裁剪       │       │ L7  libfranka 硬件反射 │
   │     (每步, ±2.90 rad)   │       │     (1 kHz, 固件)     │
   │ L2  训练范围 + 余量裁剪  │       │ L8  E-Stop 急停按钮   │
   │     (每步, 见下表)      │       │     (即时, 硬件)      │
   │ L3  单步速度限制         │       └──────────────────────┘
   │     (每步, 0.15 rad)    │
   │ L4  TCP 围栏看门狗       │
   │     (50 Hz, 笛卡尔空间)  │
   │ L5  关节速度范数限制      │
   │     (50 Hz, 看门狗)     │
   │ L6  碰撞行为收紧         │
   │     (初始化时, libfranka)│
   └─────────────────────────┘
```

### L1 硬关节限位

FR3v2.1 关节限位, 超限部分被 clip 并发出 `HARD LIMIT` 警告:

| 关节 | 下限 (rad) | 上限 (rad) |
|------|-----------|-----------|
| q1   | -2.8973   | +2.8973   |
| q2   | -1.7628   | +1.7628   |
| q3   | -2.8973   | +2.8973   |
| q4   | -3.0718   | -0.0698   |
| q5   | -2.8973   | +2.8973   |
| q6   | -0.0175   | +3.7525   |
| q7   | -2.8973   | +2.8973   |

### L2 训练范围裁剪

动作被 clip 到训练数据范围 + 安全余量. **q7 的余量为 0** (零容差), 因为 q7 超出
训练边界 0.026 rad 就会导致预测动作方向反转 (cosine 从 +0.34 翻转到 -0.27):

| 关节 | 训练 min   | 训练 max   | 余量 (rad) | 有效下限   | 有效上限   |
|------|-----------|-----------|-----------|-----------|-----------|
| q1   | -0.4842   | +0.0452   | 0.15      | -0.6342   | +0.1952   |
| q2   | -0.1030   | +0.3120   | 0.15      | -0.2530   | +0.4620   |
| q3   | -0.2025   | +0.4789   | 0.15      | -0.3525   | +0.6289   |
| q4   | -2.2044   | -1.5347   | 0.15      | -2.3544   | -1.3847   |
| q5   | -0.2041   | +0.0806   | 0.15      | -0.3541   | +0.2306   |
| q6   | +1.5702   | +2.4536   | 0.15      | +1.4202   | +2.6036   |
| q7   | +0.4843   | +0.9807   | **0.00**  | **+0.4843**| **+0.9807**|

### L3 单步速度限制

单步关节变化量不超过 `MAX_JOINT_STEP_RAD = 0.15 rad`. 超限时按比例缩放整个
7D 位移向量, 保持方向不变.

### L4-L5 TCP 围栏看门狗

50 Hz 后台线程持续检查:
- **TCP 位置** 是否在围栏 `[TRAIN_TCP_MIN - margin, TRAIN_TCP_MAX + margin]` 内
- **关节速度范数** `|dq|` 是否低于阈值

围栏参数来自 `franky_ext.motion_limits`:
- `guard_margin_m()`: 围栏四周水平余量
- `guard_floor_margin_m()`: -Z 方向余量 (桌面方向, 更紧)
- `guard_max_dq_rad_s()`: 关节速度范数上限

触发后: 立即制动 → 锁存原因 → 日志报错 → 等待操作员复位.

### L6 碰撞行为收紧

初始化时调用 `robot.set_collision_behavior()`, 将 libfranka 的笛卡尔力/力矩阈值
从默认 100 N 收紧到与安全裁剪一致的水平, 使 1 kHz 硬件反射在软件安全层之前
就能介入.

---

## 8. 键盘控制

> 需要 RLinf 的 `KeyboardListener` (基于 evdev, 支持无头环境).
> 如不可用, 程序自动回退为无键盘模式.

| 按键 | 功能                                        |
|------|---------------------------------------------|
| `a`  | 开始 episode (reset 后等待此键)              |
| `r`  | 中止当前 episode: 立即停止, truncated=True    |
| `c`  | 标记成功: terminated=True, reward=1          |
| `b`  | 标记失败: terminated=True, reward=0          |
| `h`  | 回 HOME 位姿 (不结束 episode)                |

### 无键盘模式

```bash
python franka_4dwvla_client.py --task "..." --no-keyboard
```

直接运行到 `--max-steps` 或 Ctrl-C. 适用于:
- 没有 evdev 设备的环境
- 自动化批量测试

---

## 9. State History 机制 (修复 A)

### 问题

训练时 `Extract3DKeypointTransformFn` 每个 30 Hz 数据帧推进一次 `observation.his_len`.
推理时每 `n_exec` 个控制步才调用一次模型, 导致 `his_len` 比训练时慢 `n_exec` 倍.
训练数据中最早的夹爪闭合出现在 `his_len >= 120`, 而旧实现在 700 步时 `his_len` 才
到 70, 永远不会到达闭合区域.

### 修复

客户端在每个控制步记录当前 7D 关节角度到 `ExecutedStateBuffer`. 请求推理时,
将上次推理以来积累的所有 pose 一并发送给服务端:

```python
{
    "images": {...},
    "state": {"arm": [7 floats], "gripper": [1 float]},
    "state_history": [[7 floats], [7 floats], ...],  # n_exec 个 pose
    "task": "plug into socket",
    "protocol": 2
}
```

服务端按序 replay 每个 pose 到 `FKKeypointComputer`, 使 `his_len` 以正确的
每控制步速率递增.

---

## 10. 服务端 Reset 通知 (修复 E)

每次 episode 重置时, 客户端发送 `{"command": "reset"}` 给服务端:

- 清除服务端的 policy KV cache
- 清除 FK keypoint history
- 重置 `request_index = 0`

防止中止的 episode 的状态泄漏到下一个 episode.

---

## 11. 控制频率监测 (修复 B)

franky 的 `Robot.move()` 是阻塞调用, 实际控制频率可能远低于 `--control-hz`.
客户端每 50 步输出实际达到的频率:

```
[step 50] control rate: 3.58 Hz (requested 10.0 Hz, ratio=0.36)
```

当 `ratio < 0.5` 时以 WARNING 级别输出.

> 注意: 客户端仅**测量和报告**, 不改变运动行为. 切换到异步执行需要经过
> 四级实机验证流程.

---

## 12. HOME 位姿

从 `franky_ext/dsplug/home_pose.json` 加载, 内容:

```json
{
  "joint_position_rad": [
    -0.2576, -0.0209, 0.1674, -1.8850, -0.0551, 1.9064, 0.6436
  ],
  "tcp_pose_from_robot": {
    "position_m": [0.5611, -0.0598, 0.4123]
  }
}
```

每次 episode reset 时:
1. 打开夹爪 → 等待 0.3s
2. `reset_joint(HOME_JOINTS)` (关节空间运动, `relative_dynamics_factor=0.1`)
3. 等待 0.5s → 再次打开夹爪 → 等待 0.3s

---

## 13. 日志

### 日志文件

每次运行自动创建带时间戳的日志文件:

```
<log_dir>/4dwvla_client_20260922_143000_12345.log
```

同时输出到 stdout 和文件. 日志级别 INFO.

### 关键日志条目

| 日志关键字                  | 含义                                        |
|----------------------------|---------------------------------------------|
| `Inference (q1=... grip=...)`| 开始推理请求                               |
| `Received N actions`        | 收到动作块                                  |
| `execute: action=... delta=...` | 正在执行动作及与当前状态的差值           |
| `state_after=... realized_delta=...`| 执行后实际到达的状态               |
| `HARD LIMIT`                | L1: 动作超出硬关节限位, 已裁剪              |
| `OUT-OF-TRAIN`              | L2: 动作超出训练范围, 已裁剪                |
| `TRAIN-EDGE`                | L2b: 动作贴近训练范围边界 (尤其 q7)         |
| `VEL LIMIT`                 | L3: 单步位移过大, 已缩放                    |
| `MOTION GUARD TRIP`         | L4/L5: TCP 围栏或关节速度触发, 已制动       |
| `NEAR-SINGULAR`             | 机械臂接近奇异位形, 警告                    |
| `control rate: X Hz`        | 每 50 步报告实际控制频率                    |
| `gripper action=... → close/open` | 夹爪状态变更                          |
| `Server reset returned: ok` | 修复 E: episode 重置通知成功                |

---

## 14. 异常处理与恢复

### Motion Guard Trip

1. 看门狗线程检测到 TCP 越界或关节速度过快
2. 立即制动: `Robot.stop()` (围栏/朝向触发) 或 `freeze_at_current()` (滞后/速度触发)
3. 锁存原因, 拒绝后续运动指令
4. 日志输出 `MOTION GUARD TRIP [fence/dq]: ...`
5. 恢复流程:
   - 有限恢复预算 (`guard_recovery_budget`), 超出后终止运行
   - `recover_from_guard_trip()`: 确认停稳 → 清除错误 → 解除锁存
   - 操作员手动检查后按 Enter 继续

### Keyboard Abort (`r` 键)

1. `controller.stop()` 立即停止运动
2. episode 标记为 truncated
3. 等待操作员摆放场景, 按 `a` 开始下一轮

### Ctrl-C

1. SIGINT handler 设置 `_abort = True`
2. 主循环退出
3. `finally` 块依次: 断开 IPC → 关闭相机 → controller cleanup (停看门狗, freeze, 释放夹爪)

---

## 15. 与 franka_vla_client.py 的差异

本文件 (`franky_ext/franka_4dwvla_client.py`) 与 `4dwvla_ext/franka_vla_client.py`
功能相同, 主要差异在于依赖和代码组织:

| 方面            | `4dwvla_ext/franka_vla_client.py`          | `franky_ext/franka_4dwvla_client.py`    |
|-----------------|--------------------------------------------|-----------------------------------------|
| 机器人控制      | `FrankyJointEnv` (Gym env 封装)            | `FrankaController` (直接 franky 调用)   |
| 底层控制器      | `FrankyControllerDirect`                   | franky.Robot + FrankaLibfrankaGripper    |
| 安全常数来源    | 间接经 `FrankyControllerDirect`             | 直接从 `franky_ext.motion_limits` 导入  |
| HOME 位姿       | `franky_ext.dsplug.home_pose`              | 同左                                     |
| 键盘控制        | `KeyboardVLAEvalWrapper` (Gym wrapper)     | `run_with_keyboard()` 独立函数          |
| State History   | 导入 `state_history_buffer.py`             | 内联 `ExecutedStateBuffer` class        |
| 夹爪模式        | 支持 binary_abs / binary_delta / continuous| 仅 binary_abs (可通过环境变量切换阈值)  |
| 依赖            | 需要 `4dwvla_ext/` 的多个模块              | 自包含, 仅依赖 `franky_ext` 和 `franky`|

---

## 16. 常见问题排查

### Q: 客户端连不上服务端

```
ConnectionRefusedError: [Errno 111] Connection refused
```

- 确认服务端已启动并输出 `listening on port 5555`
- 确认 `--server-host` 和 `--server-port` 正确
- 确认网络连通: `nc -zv <server_host> 5555`
- 容器间通信: 确认两个容器均使用 `--network host`

### Q: 动作全是 NaN 或 Inf

```
ValueError: Actions contain NaN or infinite values
```

- 检查 checkpoint 路径是否正确
- 检查 `stats.json` 是否存在且包含 `franka_plug` 键
- 检查 schema YAML 与 checkpoint 的 `config.json` 是否匹配

### Q: 夹爪不闭合

- 检查 `VLA_GRIPPER_CLOSE_IF_ABOVE` (默认 `1` = action[7] >= 0.5 时 close)
- 查看日志中 `action_grip` 值是否达到阈值
- 若 `his_len` 增长过慢, 确认 `--n-exec` 与服务端一致
- 参考 `grperr_1.md` 分析

### Q: MOTION GUARD TRIP

- 查看日志中 trip 类型:
  - `fence`: TCP 越界 → 检查 `TRAIN_TCP_MIN/MAX` 和初始摆放
  - `dq`: 关节速度过快 → 检查动作幅度, 降低 `--control-hz`
- 清除错误: 在 Franka Desk 界面 clear fault
- 恢复后可按 `a` 继续

### Q: 控制频率过低

```
[step 50] control rate: 2.28 Hz (requested 10.0 Hz, ratio=0.23)
```

- franky 的 `Robot.move()` 是阻塞的, 每步等待运动完成
- 检查 `FRANKA_GRIPPER_MAX_WIDTH_M` 是否超过手实际报告值
  (超出会导致每步发阻塞的 "open wider" 指令)
- 检查步幅: 大的关节位移会导致 `move()` 阻塞更久
- 降低 `--control-hz` 可减少警告但不改善实际频率

### Q: 相机画面交换 (global/wrist 反了)

- 指定相机序列号:
  ```bash
  export RS_GLOBAL_SERIAL=250222073513
  export RS_WRIST_SERIAL=420122070525
  ```
- 或使用 CLI 参数:
  ```bash
  --global-camera-serial 250222073513 --wrist-camera-serial 420122070525
  ```
- 通过 `rs-enumerate-devices` 查看设备列表确认序列号
