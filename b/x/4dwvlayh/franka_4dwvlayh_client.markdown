# franka_4dwvlayh_client.py 使用说明

## 1. 概述

`franka_4dwvlayh_client.py` 是一个 Franka FR3 机械臂的 VLA (Vision-Language-Action) 推理客户端. 它通过 TCP IPC (`multiprocessing.connection`) 连接到运行在 GPU 容器中的 `vla_inference_server.py`, 将相机图像和机器人状态发送给 4DWVLA (InternVLA-A1.5) 模型, 接收预测的关节动作, 并通过 `JointImpedanceTracker` + Hermite 插值平滑地驱动机械臂执行.

### 1.1 与同目录其他脚本的关系

| 脚本 | 推理后端 | 通信协议 | 用途 |
|------|----------|----------|------|
| `deploy_plug_franka3.py` | `rlinf.serve.plug_franka3` (WebSocket) | WebSocket + msgpack | 原始 plug-into-socket 部署 |
| **`franka_4dwvlayh_client.py`** | **`vla_inference_server.py`** (TCP IPC) | **`multiprocessing.connection`** | **4DWVLA 推理服务部署** |

本脚本从 `deploy_plug_franka3.py` 导入了全部底层机器人控制组件 (相机、夹爪、关节跟踪器、插值器等), 仅替换了推理通信层.

### 1.2 系统架构

```
┌─────────────────────────────────────────────┐
│            GPU 容器 (rlinf-4dwvla-gpu)       │
│                                             │
│  vla_inference_server.py                    │
│  ├── InternVLA-A1.5 模型                    │
│  ├── FK Keypoint Computer                   │
│  └── TCP Listener (port 5555, authkey)      │
│                                 ▲           │
└─────────────────────────────────┼───────────┘
                                  │ multiprocessing.connection
                                  │ (Python pickle over TCP)
┌─────────────────────────────────┼───────────┐
│            Franky 环境 (host / 容器)         │
│                                 ▼           │
│  franka_4dwvlayh_client.py                  │
│  ├── VLAServerConnection (IPC 层)           │
│  ├── ExecutedStateBuffer (关键点历史)         │
│  ├── CameraThread ×2 (global + wrist)       │
│  ├── JointImpedanceTracker (关节控制)        │
│  ├── Hermite 插值 + 流水线推理              │
│  ├── 夹爪逻辑 (binary / width)              │
│  ├── WrenchSampler (力矩日志)               │
│  └── RunLogger (CSV + npz 日志)             │
│                                 │           │
│                    franky.Robot ▼            │
│                    Franka FR3 (172.16.0.2)   │
└─────────────────────────────────────────────┘
```

## 2. 环境准备

### 2.1 前提条件

- Franka FR3 机械臂已开机, FCI 接口就绪
- 两台 Intel RealSense D435i 相机已连接:
  - Global 相机 (serial: `250222073513`)
  - Wrist 相机 (serial: `420122070525`)
- GPU 容器中的 `vla_inference_server.py` 已启动并监听
- gello/franky Python 虚拟环境中已安装依赖: `franky`, `pyrealsense2`, `numpy`, `msgpack_numpy`, `websockets`

### 2.2 启动 GPU 推理服务器

在 GPU 容器中 (见 `4dwvla_ext/configs/docker_run_4dwvla_gpu.sh`):

```bash
# 启动 GPU 容器
bash b/x/4dwvla_ext/configs/docker_run_4dwvla_gpu.sh

# 容器内部: 启动推理服务
source /opt/venv/4dwvla/bin/activate
python /workspace/RLinf/b/x/4dwvla_ext/vla_inference_server.py \
    --ckpt-path /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp041680 \
    --schema-path /workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml \
    --kpt-meta-path /workspace/RLinf/b/d/frk1/plug/keypoints_meta.json \
    --urdf-path /workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf \
    --n-exec 10 \
    --port 5555 \
    --dtype bfloat16
```

或直接使用封装脚本:

```bash
bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh
```

### 2.3 环境变量 (可选)

服务器端相关环境变量可通过 `franka_plug_eval.env` 一次性设置:

```bash
source b/x/4dwvla_ext/configs/franka_plug_eval.env
```

关键变量:

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `VLA_CKPT_PATH` | `/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp041680` | 4DWVLA 模型 checkpoint 路径 |
| `VLA_SCHEMA_PATH` | `...4WVLA/b/s/Frk/cfg/franka_plug.yaml` | 数据集 schema |
| `VLA_KPT_META_PATH` | `.../keypoints_meta.json` | 关键点元数据 |
| `VLA_URDF_PATH` | `.../fr3v2_1_franka_hand.urdf` | Franka URDF (FK 计算) |
| `VLA_PORT` | `5555` | 服务器监听端口 |
| `VLA_N_EXEC` | `10` | 每次推理执行的动作步数 |
| `RS_GLOBAL_SERIAL` | `250222073513` | Global 相机 serial |
| `RS_WRIST_SERIAL` | `420122070525` | Wrist 相机 serial |

## 3. 使用方法

### 3.1 激活环境

```bash
source /home/nvidia/cxy_ws/gello_software/.venv/bin/activate
cd /path/to/RLinf/b/x/4dwvlayh
```

### 3.2 Dry Run (无机器人, 无相机)

仅测试与 GPU 服务器的通信是否正常, 不连接机器人和相机:

```bash
python franka_4dwvlayh_client.py \
    --server-host 127.0.0.1 \
    --server-port 5555 \
    --task "plug into socket" \
    --dry-run --no-robot --no-camera
```

### 3.3 Dry Run (有机器人, 无动作执行)

连接机器人读取状态, 但不启动 JointImpedanceTracker (不发送运动指令):

```bash
python franka_4dwvlayh_client.py \
    --server-host 127.0.0.1 \
    --server-port 5555 \
    --task "plug into socket" \
    --dry-run
```

### 3.4 真实机器人执行

**确保急停开关在手边, 可随时按下.**

```bash
python franka_4dwvlayh_client.py \
    --server-host 127.0.0.1 \
    --server-port 5555 \
    --task "plug into socket"
```

### 3.5 推荐的首次运行流程

```bash
# Step 1: 确认通信正常
python franka_4dwvlayh_client.py \
    --server-host 127.0.0.1 --server-port 5555 \
    --task "plug into socket" \
    --dry-run --no-robot --no-camera --once

# Step 2: 确认机器人连接和相机正常 (不执行动作)
python franka_4dwvlayh_client.py \
    --server-host 127.0.0.1 --server-port 5555 \
    --task "plug into socket" \
    --dry-run --once

# Step 3: 慢速真实执行, 随时准备按急停
python franka_4dwvlayh_client.py \
    --server-host 127.0.0.1 --server-port 5555 \
    --task "plug into socket" \
    --speed 0.35 --wait-enter --once

# Step 4: 正常持续执行
python franka_4dwvlayh_client.py \
    --server-host 127.0.0.1 --server-port 5555 \
    --task "plug into socket" \
    --speed 0.5 --wait-enter
```

## 4. 命令行参数详解

### 4.1 连接参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--server-host` | `127.0.0.1` | `vla_inference_server.py` 的主机地址 |
| `--server-port` | `5555` | `vla_inference_server.py` 的 TCP 端口 |
| `--robot-ip` | `172.16.0.2` | Franka FR3 的 IP 地址 |

### 4.2 任务参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--task` | `"plug into socket"` | 发送给 VLA 模型的自然语言任务指令 |

### 4.3 运行模式

| 参数 | 说明 |
|------|------|
| `--dry-run` | 连接机器人但不启动 JointImpedanceTracker (不发送关节运动指令) |
| `--no-robot` | 不连接 franky, 使用假的关节/夹爪数据 |
| `--no-camera` | 不打开 RealSense 相机, 使用全黑 480x640 图像 |
| `--once` | 执行一次推理后退出 |
| `--wait-enter` | 连接完成后等待用户按 Enter 才开始执行 |
| `--home-gripper` | 启动时先 homing 夹爪再打开到最大宽度 |

### 4.4 运动控制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--control-hz` | `15.0` | 策略控制频率 (Hz). 决定每步的时间间隔 `dt = 1/hz` |
| `--speed` | `0.5` | 回放速度系数. `0.5` = 每个 waypoint 用 2 倍时间; `1.0` = 原速 |
| `--execute-horizon` | `8` | 每次推理返回的 chunk 中实际执行的步数. 模型预测 10 步, 默认执行前 8 步 |
| `--interp-hz` | `100.0` | waypoint 之间 Hermite 插值的命令发送频率. `0` = 每步仅一次 `set_target` |
| `--blend-steps` | `4` | chunk 边界处的混合步数, 避免相邻 chunk 切换时关节突跳 |

**速度与时间的关系**:
- 策略产出每步间隔: `dt = 1 / control_hz` (默认 1/15 ≈ 66.7 ms)
- 实际回放每步时间: `step_dt = dt / speed` (默认 66.7 / 0.5 = 133.3 ms)
- 一个 chunk (8 步) 的执行时间: `8 × 133.3 ≈ 1067 ms`

### 4.5 流水线推理参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--no-pipeline` | (关闭) | 禁用流水线. 启用后: 在 chunk 最后几步时提前发起下一次推理请求, 减少 chunk 之间的空闲等待 |
| `--prefetch` | `0` | chunk 尾部预留给推理 RTT 的步数. `0` = 根据上次 RTT 自动计算 |

流水线模式示意:

```
时间 ──────────────────────────────────────────────────────────►

无流水线:
  [执行 chunk A 全部 8 步] [等 RTT] [执行 chunk B 全部 8 步] [等 RTT] ...
                           ↑ 空闲

有流水线 (prefetch=2):
  [执行 chunk A 前 6 步][推理B←→][继续步 7-8][执行 chunk B ...]
                        ↑ 重叠, 无空闲
```

### 4.6 夹爪参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--gripper-mode` | `binary` | `binary`: 二值模式 (全开/全关); `width`: 连续宽度模式 |
| `--gripper-close-g` | `0.18` | GELLO 值达到此阈值时触发闭合 (0=全开, 1=全关) |
| `--gripper-close-rise` | `0.10` | g[-1]-g[0] 上升量达到此值 (且 gmax>=0.10) 也触发闭合 |

**夹爪闭合检测逻辑**:

模型预测的夹爪值是一个渐变斜坡 (非阶跃), 峰值可能仅 0.16~0.24. 客户端使用两级确认:

1. **步级 (per-step)**: 连续 2 步 `g >= gripper_close_g` → 置位 `want_closed`
2. **推理级 (per-infer)**: 连续 2 次推理的 chunk 检测到闭合意图 → 锁存 `want_closed`
3. **开启**: 连续 2 次推理无闭合意图且 gmax <= 0.10 → 解锁

`binary` 模式下, 闭合 → `grasp(width=0, force=50N)`, 开启 → `move(max_width)`.

### 4.7 安全参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--max-joint-jump-deg` | `30.0` | chunk 首步关节目标与当前测量值的最大偏差 (度). 超过则 ABORT, 不发送指令. `0` 禁用 |
| `--max-tracking-error-deg` | `0.0` | 目标与测量的最大跟踪误差 (度). `0` 禁用. 过小 (如 2.9°) 会导致抖动 |
| `--no-rt` | (关闭) | 跳过 RT 加固 (mlockall / SCHED_FIFO / CPU 亲和性) |

### 4.8 日志参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--log-dir` | `./runs/` | 日志输出根目录 |
| `--no-log` | (关闭) | 不记录日志 |

## 5. IPC 协议

客户端通过 `multiprocessing.connection.Client` 连接到服务器, 使用 Python pickle 序列化, 认证密钥为 `b"4dwvla-eval"`.

### 5.1 Reset 消息

```python
# 客户端发送
{"command": "reset"}

# 服务器响应
{"status": "ok", "actions": []}
```

清除服务器端的策略 KV cache 和关键点历史.

### 5.2 推理消息 (Protocol v2)

```python
# 客户端发送
{
    "images": {
        "global": np.ndarray,   # (480, 640, 3) uint8 RGB
        "wrist":  np.ndarray,   # (480, 640, 3) uint8 RGB
    },
    "state": {
        "arm":     [float] * 7,  # 关节角度 (rad)
        "gripper": [float],      # GELLO 值 (0=开, 1=关)
    },
    "state_history": [[float]*7, ...],  # 上次推理以来执行的每步关节角度
    "task": "plug into socket",
    "protocol": 2,
}

# 服务器响应
{
    "status": "ok",
    "actions": [[float]*8, ...],   # N×8: 7 关节 + 1 夹爪
}
```

### 5.3 关键点历史 (state_history)

训练时, 关键点历史 (`observation.his_len`) 每个控制步 (30 Hz) 推进一次. 推理时每 `n_exec` 个控制步才调用一次推理, 如果不补偿, 关键点历史的推进速度会慢 `n_exec` 倍, 导致夹爪闭合时机延迟.

`state_history` 字段携带了上次推理以来每个控制步的实测关节角度, 服务器端通过 `FKKeypointComputer` 逐步回放, 使 `his_len` 以正确的速率推进.

### 5.4 Shutdown 消息

```python
{"command": "shutdown"}
```

客户端退出时发送, 通知服务器断开连接.

## 6. 控制流水线详解

### 6.1 主循环

```
启动
  │
  ├─ RT 加固 (mlockall, SCHED_FIFO, CPU 亲和性)
  ├─ 连接 vla_inference_server (TCP IPC)
  ├─ 连接 Franka FR3 (franky)
  │    ├─ recover_from_errors()
  │    ├─ 设置碰撞阈值
  │    ├─ 初始化 JointImpedanceTracker (保持当前姿态)
  │    └─ WrenchSampler (100 Hz 力矩采集)
  ├─ 启动 CameraThread ×2 (global + wrist, 30 fps)
  ├─ 发送 reset 到服务器
  ├─ [可选] 等待 Enter
  │
  ▼
  推理循环:
  ┌──────────────────────────────────────────────────┐
  │  1. 读取相机图像 + 机器人状态                      │
  │  2. drain state_history (上次推理以来的关节角度)    │
  │  3. 发送推理请求 → 接收 actions (N×8)             │
  │  4. consume_reply:                                │
  │     ├─ 检测夹爪闭合意图 (gmax/rise)               │
  │     ├─ 计算 execute_horizon                       │
  │     ├─ 安全检查 (max_joint_jump)                  │
  │     └─ blend_chunk (与上一个 chunk 的衔接混合)     │
  │  5. 逐步执行 chunk:                               │
  │     ├─ apply_gripper (binary/width)               │
  │     ├─ clip_target (关节限位 + 跟踪误差钳位)       │
  │     ├─ play_joint_segment (Hermite 插值执行)       │
  │     └─ state_history.record (记录实测关节角度)     │
  │  6. [流水线] 在 chunk 尾部提前发起下一次推理        │
  │  7. [流水线] chunk 执行完, coast_while 等推理返回   │
  └──────────────────────────────────────────────────┘
  │
  ▼
  清理:
  ├─ 停止 tracker
  ├─ 停止夹爪
  ├─ recover_from_errors()
  ├─ 关闭相机
  ├─ 关闭 IPC 连接
  └─ 保存日志
```

### 6.2 Hermite 插值

waypoint 之间使用三次 Hermite 样条插值, 在 `interp_hz` (默认 100 Hz) 频率下向 `JointImpedanceTracker.set_target(q, dq)` 发送位置和速度目标. 这确保:
- 关节运动连续平滑 (C1 连续)
- 与 franky 的阻抗控制器匹配 (需要持续的高频指令流)
- chunk 之间 velocity 平滑衔接

### 6.3 Chunk 混合

```
上一个 chunk 末尾: prev_q = [...实际位置...]
新 chunk:          raw_chunk[0] = [...模型预测...]

混合后: chunk[i, :7] += (prev_q - raw_chunk[0, :7]) × max(0, 1 - (i+1)/blend_steps)
```

前 `blend_steps` (默认 4) 步逐渐从当前位置过渡到模型预测, 避免 chunk 切换时的位置突跳.

## 7. 日志输出

### 7.1 目录结构

```
runs/
└── 20260922_143055_4dwvla-vla_inference_server/
    ├── meta.json       # 运行参数和配置快照
    ├── wrench.csv      # 100 Hz 力/力矩数据 (t_rel, t_mono, phase, infer_i, fx..tz)
    ├── wrench.npz      # 压缩的力矩数据 (运行结束时保存)
    └── infer.csv       # 每次推理的记录 (rtt, 夹爪状态, 关节角度等)
```

### 7.2 控制台输出示例

```
[   1] rtt=187ms history=0 q0=[-10.2  28.5  ... ] dq0=3.2deg g_obs=0.000 g0=0.012 gmax=0.045→76.8mm rise=0.030 exec=8/10 play=133ms pipe=Y prefetch=2 grip=open intent=n g=[0.012, 0.015, ...]
[   2] rtt=142ms history=8 q0=[-11.0  29.1  ... ] dq0=1.1deg ...
```

各字段含义:
- `rtt`: 推理请求往返时间 (ms)
- `history`: 本次推理携带的 state_history 长度
- `q0`: chunk 首步关节目标 (度)
- `dq0`: 首步目标与当前位置的偏差 (度)
- `g_obs`: 当前夹爪 GELLO 值
- `g0/gmax`: chunk 中夹爪值的首步 / 最大值
- `gmax→mm`: gmax 对应的物理宽度
- `rise`: chunk 中夹爪值的上升量 (g[-1]-g[0])
- `exec`: 实际执行步数 / 模型输出步数
- `play`: 每步回放时间 (ms)
- `pipe`: 流水线开关
- `prefetch`: 预取步数
- `grip`: 当前夹爪目标状态 (open/close)
- `intent`: 本次 chunk 是否检测到闭合意图

## 8. 安全注意事项

1. **急停**: 真实机器人运行时, 始终将急停按钮放在手边
2. **首次运行**: 按 Section 3.5 的递进流程, 从 dry-run 到慢速执行, 逐步验证
3. **关节跳变保护**: `--max-joint-jump-deg 30` 会在首步目标偏离当前位置超过 30° 时自动 ABORT. 常见原因: 模型输出的是 delta 动作但被当作 abs 处理
4. **碰撞阈值**: 已设置为 `[80, 80, 80, 80, 11, 11, 11]` (关节力矩) / `[100, 100, 100, 25, 25, 25]` (笛卡尔力), 外部碰撞会触发 franky 安全停止
5. **Ctrl+C**: 捕获 SIGINT/SIGTERM, 优雅停止 tracker → 停止夹爪 → recover_from_errors → 关闭连接
6. **`--speed`**: 首次使用新模型时建议设为 `0.35` (更慢), 确认动作合理后再调高

## 9. 常见问题

### Q: 连接服务器失败

```
ConnectionRefusedError: [Errno 111] Connection refused
```

- 确认 GPU 容器中的 `vla_inference_server.py` 已启动并监听
- 确认端口号一致 (客户端 `--server-port` = 服务器 `--port`)
- 如果使用 Docker `--network host`, 客户端用 `--server-host 127.0.0.1`

### Q: 认证失败

```
AuthenticationError: digest received was wrong
```

- 客户端和服务器必须使用相同的 authkey (`b"4dwvla-eval"`)

### Q: "ABORT: first joint target N deg from measured"

- 模型输出的首步关节目标距离当前位置过远, 可能原因:
  - 模型 checkpoint 使用了 delta 动作模式, 输出被误当 abs 处理
  - stats.json 的归一化参数与当前 checkpoint 不匹配
- 降低 `--max-joint-jump-deg` 的值可以更严格地保护, 但不能根治模型问题

### Q: 夹爪始终不闭合

- 检查服务器日志中 `full_chunk_grip` 是否有值上升到 0.18 以上
- 检查 `history` 字段是否在增长 (应随步数递增, 如 0→8→16→...)
  - 如果一直为 0, 说明 state_history 没有正确记录
- 如果 `his_len` 增长太慢, 夹爪闭合的关键点条件可能永远无法满足 (这就是 state_history 机制要解决的问题)

### Q: 机械臂运动抖动

- 尝试降低 `--speed` (如 `0.35`)
- 确认 `--interp-hz 100` (默认值, 不要设为 0)
- 增大 `--blend-steps` (如 6)
- 避免过小的 `--max-tracking-error-deg` (默认 0 = 禁用, 2.9° 会导致反复抖动)

### Q: Tracker stopped

- franky 的 JointImpedanceTracker 异常退出, 通常由外部碰撞或关节限位触发
- 检查 Franka Desk 上的错误信息
- 运行 `robot.recover_from_errors()` 后重试

## 10. 与 franka_vla_client.py 的对比

| 特性 | `franka_vla_client.py` | `franka_4dwvlayh_client.py` |
|------|------------------------|-----------------------------|
| 关节控制 | `FrankyJointEnv` (gym.Env, 阻塞式 `Robot.move()`) | `JointImpedanceTracker` (非阻塞, Hermite 插值) |
| 控制频率 | 受 `Robot.move()` 阻塞限制, 实测常低于目标 | 由 `interp_hz` (100 Hz) 精确控制 |
| 流水线推理 | 无 (推理与执行串行) | 有 (推理与末尾几步执行重叠) |
| Chunk 混合 | 无 | 有 (`blend_steps`) |
| 键盘控制 | 有 (`KeyboardVLAEvalWrapper`, evdev) | 无 (Ctrl+C 优雅退出) |
| RT 加固 | 无 | 有 (mlockall, SCHED_FIFO, CPU affinity) |
| 力矩日志 | 无 | 有 (100 Hz WrenchSampler) |
| IPC 协议 | 相同 (`multiprocessing.connection`) | 相同 |
| state_history | 相同 (Protocol v2) | 相同 |
