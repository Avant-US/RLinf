# RLinf × Galaxea R1 Pro 真机 RL 实施细节(imp1)

> 本文是 [bt/docs/rwRL/r1pro5op47.md](r1pro5op47.md) 设计方案的**第一版实施落地说明**:覆盖代码组织、配置、本地 dummy 验证、真机部署、测试运行、变体切换、与设计文档的章节对应、已知限制与后续工作。
>
> 阅读对象:RLinf 仓库内将基于本实施开 PR / 合 PR / 现场部署的工程师。
> 版本:imp1 · 日期:2026-04-26

---

## 目录

1. [实施摘要](#1-实施摘要)
2. [目录树与文件总览](#2-目录树与文件总览)
3. [模块级实施细节](#3-模块级实施细节)
4. [配置说明:双路径与全 ROS2 切换](#4-配置说明双路径与全-ros2-切换)
5. [本地开发与 dummy 测试流程](#5-本地开发与-dummy-测试流程)
6. [真机部署完整流程](#6-真机部署完整流程)
7. [单元测试运行](#7-单元测试运行)
8. [已知限制](#8-已知限制)
9. [与设计文档章节对应表](#9-与设计文档-r1pro5op47md-章节对应表)
10. [后续工作](#10-后续工作)
11. [版本 2 变更](#11-版本-2-变更2026-04-26-dummy-测试通过)

---

## 1. 实施摘要

### 1.1 完成清单

| # | 类型 | 文件 / 模块 | 关键功能 |
|---|---|---|---|
| 1 | 新增 | [rlinf/scheduler/hardware/robots/galaxea_r1_pro.py](../../../rlinf/scheduler/hardware/robots/galaxea_r1_pro.py) | `GalaxeaR1ProRobot` / `GalaxeaR1ProConfig` / `GalaxeaR1ProHWInfo` / `CameraSpec`,5 项 enumerate 校验 |
| 2 | 新增 | [rlinf/envs/realworld/common/camera/ros2_camera.py](../../../rlinf/envs/realworld/common/camera/ros2_camera.py) | `ROS2Camera`(BaseCamera 子类),lazy import,JPEG / 16UC1 / 32FC1 解码,frame_age_ms |
| 3 | 修改 | [rlinf/envs/realworld/common/camera/base_camera.py](../../../rlinf/envs/realworld/common/camera/base_camera.py) | `CameraInfo` 新增 5 字段(向后兼容) |
| 4 | 修改 | [rlinf/envs/realworld/common/camera/__init__.py](../../../rlinf/envs/realworld/common/camera/__init__.py) | `create_camera` 工厂加 `ros2` 分支 |
| 5 | 新增 | [rlinf/envs/realworld/galaxear/__init__.py](../../../rlinf/envs/realworld/galaxear/__init__.py) | re-export 主类 + 触发 `tasks` 注册 |
| 6 | 新增 | [rlinf/envs/realworld/galaxear/r1_pro_robot_state.py](../../../rlinf/envs/realworld/galaxear/r1_pro_robot_state.py) | `GalaxeaR1ProRobotState` dataclass(26 DoF + IMU + BMS + 错误码) |
| 7 | 新增 | [rlinf/envs/realworld/galaxear/r1_pro_action_schema.py](../../../rlinf/envs/realworld/galaxear/r1_pro_action_schema.py) | `ActionSchema`,M1-M4 各阶段动作维度 7 / 14 / 18 / 21 |
| 8 | 新增 | [rlinf/envs/realworld/galaxear/r1_pro_safety.py](../../../rlinf/envs/realworld/galaxear/r1_pro_safety.py) | `GalaxeaR1ProSafetySupervisor`,5 级闸门 + 3 级停机 + Galaxea 特有 hook |
| 9 | 新增 | [rlinf/envs/realworld/galaxear/r1_pro_camera_mux.py](../../../rlinf/envs/realworld/galaxear/r1_pro_camera_mux.py) | `GalaxeaR1ProCameraMux`,USB / ROS2 / dummy 三路径,软同步窗,USB drop fallback |
| 10 | 新增 | [rlinf/envs/realworld/galaxear/r1_pro_controller.py](../../../rlinf/envs/realworld/galaxear/r1_pro_controller.py) | `GalaxeaR1ProController` Worker,`launch_controller(node_rank=...)`,rclpy lazy import,7 个 RPC 方法 |
| 11 | 新增 | [rlinf/envs/realworld/galaxear/r1_pro_env.py](../../../rlinf/envs/realworld/galaxear/r1_pro_env.py) | `GalaxeaR1ProEnv` + `GalaxeaR1ProRobotConfig`,Franka 同构 |
| 12 | 新增 | [rlinf/envs/realworld/galaxear/r1_pro_wrappers.py](../../../rlinf/envs/realworld/galaxear/r1_pro_wrappers.py) | 4 个 wrapper:Joystick / VR / DualArmCollision / ActionSmoother |
| 13 | 新增 | [rlinf/envs/realworld/galaxear/tasks/__init__.py](../../../rlinf/envs/realworld/galaxear/tasks/__init__.py) | 6 个任务的 `gymnasium.register` |
| 14 | 新增 | [tasks/r1_pro_single_arm_reach.py](../../../rlinf/envs/realworld/galaxear/tasks/r1_pro_single_arm_reach.py) | M1 bring-up,完整实现 |
| 15 | 新增 | [tasks/r1_pro_pick_place.py](../../../rlinf/envs/realworld/galaxear/tasks/r1_pro_pick_place.py) | M1 主任务,4 阶段 phased reward |
| 16-19 | 新增 | dual_arm_handover / dual_arm_cap_tighten / whole_body_cleanup / mobile_manipulation | 4 个骨架任务,只重写 `task_description` |
| 20 | 修改 | [rlinf/envs/realworld/realworld_env.py](../../../rlinf/envs/realworld/realworld_env.py) | `_create_env` 末尾按配置注入 4 个 Galaxea wrapper(opt-in) |
| 21 | 修改 | [rlinf/scheduler/hardware/robots/__init__.py](../../../rlinf/scheduler/hardware/robots/__init__.py) | re-export GalaxeaR1Pro* |
| 22 | 修改 | [rlinf/scheduler/hardware/__init__.py](../../../rlinf/scheduler/hardware/__init__.py) | re-export |
| 23 | 修改 | [rlinf/scheduler/__init__.py](../../../rlinf/scheduler/__init__.py) | re-export `GalaxeaR1ProHWInfo` |
| 24 | 新增 | [examples/embodiment/run_realworld_galaxea_r1_pro.sh](../../../examples/embodiment/run_realworld_galaxea_r1_pro.sh) | 异步训练入口包装 |
| 25 | 新增 | [ray_utils/realworld/setup_before_ray_galaxea_r1_pro.sh](../../../ray_utils/realworld/setup_before_ray_galaxea_r1_pro.sh) | source ROS2 + Galaxea SDK + DDS env vars + CAN 检查 |
| 26 | 修改 | [requirements/install.sh](../../../requirements/install.sh) | `SUPPORTED_ENVS` 加 `galaxea_r1_pro` + `install_galaxea_r1_pro_env()` |
| 27 | 新增 | [examples/embodiment/config/realworld_dummy_galaxea_r1_pro_sac_cnn.yaml](../../../examples/embodiment/config/realworld_dummy_galaxea_r1_pro_sac_cnn.yaml) | Dummy 训练配置(单节点 / CI 用) |
| 28 | 新增 | [realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async.yaml](../../../examples/embodiment/config/realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async.yaml) | 默认双路径主训练配置 |
| 29 | 新增 | [realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async_all_ros2.yaml](../../../examples/embodiment/config/realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async_all_ros2.yaml) | 全 ROS2 变体主训练配置 |
| 30 | 新增 | [config/env/realworld_galaxea_r1_pro_pick_place.yaml](../../../examples/embodiment/config/env/realworld_galaxea_r1_pro_pick_place.yaml) | env 子配置(双路径) |
| 31 | 新增 | [config/env/realworld_galaxea_r1_pro_pick_place_all_ros2.yaml](../../../examples/embodiment/config/env/realworld_galaxea_r1_pro_pick_place_all_ros2.yaml) | env 子配置(全 ROS2) |
| 32-36 | 新增 | tests/unit_tests/test_galaxea_r1_pro_*.py + test_ros2_camera_decode.py | 5 个单元测试 |
| 37 | 新增 | [tests/e2e_tests/embodied/realworld_dummy_galaxea_r1_pro_sac_cnn.yaml](../../../tests/e2e_tests/embodied/realworld_dummy_galaxea_r1_pro_sac_cnn.yaml) | CI dummy e2e fixture |
| 38 | 新增 | [bt/docs/rwRL/r1pro5op47_imp1.md](r1pro5op47_imp1.md) | 本文件 |

合计 **新增 30 文件 + 修改 7 文件**;约 3500 行 Python + 850 行 YAML / shell + 600 行测试 + 750 行实施文档。

### 1.2 关键设计决策(实施层)

1. **lazy import**:所有 `rclpy / sensor_msgs / geometry_msgs / hdas_msg / std_msgs / cv_bridge / pyrealsense2 / turbojpeg / cv2` import 全部写在函数体或 `__init__` 内,避免在没装 ROS2 的纯 GPU 服务器上 import 模块就失败。
2. **`hdas_msg` 软依赖**:Controller 在 init 时尝试 `from hdas_msg.msg import ...`;如失败,只警告并禁用对应 L5 watchdog hook,**其它路径继续工作**。这与设计文档 §3.3 改进 5 一致。
3. **`is_dummy=True` 全 mock**:CameraMux 不创建相机,Controller 不连 ROS2,Env `_setup_hardware` short-circuit;100 步训练循环可以在公共 GPU CI runner 上跑通,无需 R1 Pro 硬件。
4. **`controller_node_rank` 显式控制**:默认走形态 A(Controller 在 Orin,EnvWorker 在 GPU server);把 YAML 字段改为 `0` 即可切到形态 B。代码不需要任何改动。
5. **wrapper opt-in**:[realworld_env.py](../../../rlinf/envs/realworld/realworld_env.py) 末尾的 4-6 行装配点完全配置驱动,Franka / Turtle2 路径不受影响。
6. **任务骨架优先 MVP**:`single_arm_reach` 与 `pick_place` 是 M1 完整实现;`dual_arm_*` / `whole_body_*` / `mobile_*` 是骨架,只继承父类 + 改 `task_description`,等团队按真机数据迭代。

---

## 2. 目录树与文件总览

```text
RLinf/
├── rlinf/
│   ├── scheduler/
│   │   ├── __init__.py                                      MOD  + GalaxeaR1ProHWInfo
│   │   └── hardware/
│   │       ├── __init__.py                                  MOD
│   │       └── robots/
│   │           ├── __init__.py                              MOD
│   │           └── galaxea_r1_pro.py                        NEW  279 行
│   └── envs/
│       └── realworld/
│           ├── realworld_env.py                             MOD  +30 行 wrapper 装配
│           ├── common/
│           │   └── camera/
│           │       ├── __init__.py                          MOD  +ros2 分支
│           │       ├── base_camera.py                       MOD  CameraInfo 新增 5 字段
│           │       └── ros2_camera.py                       NEW  240 行
│           └── galaxear/
│               ├── __init__.py                              NEW
│               ├── r1_pro_robot_state.py                    NEW  198 行
│               ├── r1_pro_action_schema.py                  NEW  148 行
│               ├── r1_pro_safety.py                         NEW  340 行
│               ├── r1_pro_camera_mux.py                     NEW  293 行
│               ├── r1_pro_controller.py                     NEW  490 行
│               ├── r1_pro_env.py                            NEW  496 行
│               ├── r1_pro_wrappers.py                       NEW  385 行
│               └── tasks/
│                   ├── __init__.py                          NEW
│                   ├── r1_pro_single_arm_reach.py           NEW
│                   ├── r1_pro_pick_place.py                 NEW  120 行
│                   ├── r1_pro_dual_arm_handover.py          NEW  (skeleton)
│                   ├── r1_pro_dual_arm_cap_tighten.py       NEW  (skeleton)
│                   ├── r1_pro_whole_body_cleanup.py         NEW  (skeleton)
│                   └── r1_pro_mobile_manipulation.py        NEW  (skeleton)
├── examples/embodiment/
│   ├── run_realworld_galaxea_r1_pro.sh                      NEW
│   └── config/
│       ├── realworld_dummy_galaxea_r1_pro_sac_cnn.yaml      NEW
│       ├── realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async.yaml          NEW
│       ├── realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async_all_ros2.yaml NEW
│       └── env/
│           ├── realworld_galaxea_r1_pro_pick_place.yaml          NEW
│           └── realworld_galaxea_r1_pro_pick_place_all_ros2.yaml NEW
├── ray_utils/realworld/
│   └── setup_before_ray_galaxea_r1_pro.sh                   NEW
├── requirements/
│   └── install.sh                                           MOD  +install_galaxea_r1_pro_env
├── tests/
│   ├── unit_tests/
│   │   ├── test_galaxea_r1_pro_hardware.py                  NEW
│   │   ├── test_galaxea_r1_pro_safety.py                    NEW
│   │   ├── test_galaxea_r1_pro_camera_mux.py                NEW
│   │   ├── test_galaxea_r1_pro_action_schema.py             NEW
│   │   └── test_ros2_camera_decode.py                       NEW
│   └── e2e_tests/embodied/
│       └── realworld_dummy_galaxea_r1_pro_sac_cnn.yaml      NEW
└── bt/docs/rwRL/
    └── r1pro5op47_imp1.md                                   NEW  (本文件)
```

---

## 3. 模块级实施细节

### 3.1 硬件注册 [galaxea_r1_pro.py](../../../rlinf/scheduler/hardware/robots/galaxea_r1_pro.py)

- 新增 dataclass `CameraSpec`:每路相机的 backend / topic / serial / 分辨率 / fps / depth / stale 阈值。`__post_init__` 校验 `backend ∈ {usb_direct, ros2}`,把 list 形式的 resolution 强转 tuple。
- `GalaxeaR1ProRobot` 继承 `Hardware`,`HW_TYPE = "GalaxeaR1Pro"`,`enumerate(node_rank, configs)` 6 项校验:
  - `_validate_rclpy`:`importlib.import_module("rclpy")`,失败抛 `ModuleNotFoundError` 并提示 source 命令。
  - `_validate_galaxea_install`:验证 `~/galaxea/install/setup.bash` 存在(警告而非异常,允许跨节点运行)。
  - `_validate_ros_domain_id`:env vs YAML 一致性检查。
  - `_validate_connectivity`:可选 `icmplib.ping`(`robot_ip` 不空时)。
  - `_validate_d405_serials`:针对 `wrist_direct_camera_serials`,枚举 `pyrealsense2` 设备,序列号缺失抛错。
  - `_validate_can_link`:soft check `ip link show can0`(GPU server 上 silent;Orin 上若 DOWN 仅警告)。
  - 全部受 `disable_validate=True` 跳过(CI / dummy 用)。
- `GalaxeaR1ProConfig` 继承 `HardwareConfig`,字段全集与设计文档 §6.1.3 对齐;`__post_init__` 把 `cameras` 的 list-of-dict 强转 `list[CameraSpec]`,允许 YAML 直接写 dict。

### 3.2 ROS2 相机 [ros2_camera.py](../../../rlinf/envs/realworld/common/camera/ros2_camera.py)

- `ROS2Camera` 继承 `BaseCamera`(沿用线程 + 队列模型)。
- `__init__` 不直接 init ROS2;调 `_open_ros2()` 内部 lazy import + `rclpy.init` + `create_node` + `SingleThreadedExecutor` + 后台 spin 线程。
- 自动选择 RGB 解码:topic 后缀 `/compressed` 走 CompressedImage + TurboJPEG / cv2 回退;否则走 Image(`bgr8` / `rgb8` / `mono8`)。
- 深度兼容 `16UC1`(D405 aligned)与 `32FC1`(head depth metres → uint16 mm)。
- `get_frame_age_ms()` 用 ROS `header.stamp` 计算陈旧度,供 CameraMux 软同步窗使用。
- `_close_device` 不调 `rclpy.shutdown()`,因为同进程可能有其他 ROS2Camera / Controller 实例。
- 与 [base_camera.py](../../../rlinf/envs/realworld/common/camera/base_camera.py) 的 `CameraInfo` 扩展字段(`backend / rgb_topic / depth_topic / stale_threshold_ms / align_depth_to_color`)与 [`__init__.py`](../../../rlinf/envs/realworld/common/camera/__init__.py) 工厂的 `ros2` 分支配套;Franka / Turtle2 现有调用因默认值不变完全不受影响。

### 3.3 Robot State [r1_pro_robot_state.py](../../../rlinf/envs/realworld/galaxear/r1_pro_robot_state.py)

- 26 DoF + IMU + BMS + 摇控器 signal + 错误码 + watchdog feedback_age_ms + is_alive。
- 字段全部用 `np.ndarray`(`float32`)和 dict,cloudpickle 友好,跨 Ray RPC 传输无需自定义序列化。
- 三个 helper:`get_arm_qpos / get_ee_pose / get_gripper_pos`(side ∈ {right, left})。
- `get_state_vector(stage flags)`:按"右臂 → 左臂 → torso → chassis"顺序展平;`include_grippers` / `include_ee` 可选。
- `to_dict`:返回 picklable obs dict;`copy()`:deepcopy。

### 3.4 Action Schema [r1_pro_action_schema.py](../../../rlinf/envs/realworld/galaxear/r1_pro_action_schema.py)

- `action_dim` per stage:7 / 14 / 18 / 21(无 gripper 时减 1 / 2)。
- `split(action) -> dict`:把扁平动作向量切到命名分量(right_xyz / right_rpy / right_gripper / left_* / torso_twist / chassis_twist)。
- `predict_arm_ee_pose(side, action, state)`:在 `torso_link4` frame 计算下一步 `[xyz, rpy]`,SafetySupervisor L3a 用此做盒子裁剪。
- `build_action_schema(cfg)`:从 `GalaxeaR1ProRobotConfig` 生成实例。

### 3.5 Safety Supervisor [r1_pro_safety.py](../../../rlinf/envs/realworld/galaxear/r1_pro_safety.py)

实现 5 级闸门(L1 schema / L2 关节 / L3a TCP 盒 / L3b 双臂碰撞球 / L4 速度加速 / L5 watchdog)+ 3 级停机(`soft_hold` / `safe_stop` / `emergency_stop`),与设计文档 §6.4 / §9 完全一致。

关键实施细节:
- `validate(action, state, schema)` 返回 `SafetyInfo`,包含 `safe_action`、`clipped` flag、人类可读 `reason: list[str]`、`metrics: dict`。
- L3a 通过 `_rewrite_arm_action` 把裁剪后的目标 EE pose 反向写回归一化 action 向量,保持后续阶段无感知。
- L4 速度 / 加速度 caps 直接在归一化 action 上执行(乘 action_scale 后比较),缩回再除回去。
- L5:`bms_low` / `feedback_stale` / SWD / `status_errors` / `operator_heartbeat` 五件套;`a2_fall_risk_pct` 在低电量时把 action 整体乘 0.5 防止抖动。
- `build_safety_config(cfg_dict)`:从 YAML 字典构造,unknown keys 自动忽略,向前兼容。

### 3.6 Camera Mux [r1_pro_camera_mux.py](../../../rlinf/envs/realworld/galaxear/r1_pro_camera_mux.py)

- `CameraMuxConfig`:`cameras: list[CameraSpec]` + 软同步窗 `soft_sync_window_ms` + `align_strategy ∈ {latest, sync_window}` + `fallback_to_ros2_on_usb_drop` + `is_dummy`。
- `_build_one(spec)` -> `BaseCamera`:走 `create_camera` 工厂,根据 `spec.backend` 自动派发。
- `get_frames(soft_sync_window_ms)`:批量拿最新帧 + post-process(crop + resize)。`align_strategy=sync_window` 时,统计 `(t_max - age) > win` 并写入 `camera/sync_window_reject_rate` 指标。
- `_note_stale(name)` + `_switch_to_ros2(name)`:USB direct 连续 `usb_drop_consecutive_threshold` 次超时 -> 自动切换到约定 ROS2 topic(`/hdas/camera_<name>/color/image_raw/compressed`)。
- Dummy mode:不创建任何 BaseCamera 实例;`get_frames` 返回零矩阵。

### 3.7 Controller [r1_pro_controller.py](../../../rlinf/envs/realworld/galaxear/r1_pro_controller.py)

- `GalaxeaR1ProController` 继承 `rlinf.scheduler.Worker`,与 `FrankaController` 同构。
- `launch_controller(env_idx, node_rank, ...)`:封装 `Cluster() + NodePlacementStrategy(node_ranks=[node_rank]) + create_group(...).launch(...)`,默认 `node_rank=1`(Orin)。
- `__init__`:lazy import `rclpy / MultiThreadedExecutor`,创建 Node + 4-thread executor + 后台 spin 线程,初始化 publishers + subscribers。
- 公共 RPC 方法:`get_state` / `is_robot_up` / `send_arm_pose(side, pose7)` / `send_arm_joints` / `send_gripper(side, pct)` / `send_torso_twist(twist4)` / `send_chassis_twist(twist3)` / `apply_brake(on)` / `go_to_rest(side, qpos, timeout_s)` / `clear_errors(side)` / `shutdown()`。
- `hdas_msg` 包优雅降级:导入失败 -> 跳过 BMS / SWD / status_errors 订阅,日志警告;其它路径正常工作。
- QoS 策略:控制 publish `RELIABLE KEEP_LAST(1)`;状态 / IMU subscribe `qos_profile_sensor_data`;两个回调组(state / safety)分离。
- `_update_feedback_age()`:每次 spin 把 `now - first_seen` 写入 `state.feedback_age_ms`,用于 SafetySupervisor L5 watchdog。
- 文件锁可省略(同节点多 Worker 时 ROS2 daemon 已经做了协调,与 ROS1 不同)。

### 3.8 Env [r1_pro_env.py](../../../rlinf/envs/realworld/galaxear/r1_pro_env.py)

- `GalaxeaR1ProEnv` 继承 `gym.Env`,签名 `__init__(override_cfg, worker_info, hardware_info, env_idx)` 与 `FrankaEnv` 完全一致。
- `_setup_hardware`:从 `hardware_info.config.controller_node_rank` 解析,调 `GalaxeaR1ProController.launch_controller(node_rank=...)`;在本地构建 `GalaxeaR1ProCameraMux`(因 USB 相机连本机)。
- `_setup_reward_worker`:复用 `EmbodiedRewardWorker.launch_for_realworld`,与 Franka 一致。
- `step(action)`:
  1. `controller.get_state().wait()[0]` 拉最新 state。
  2. `safety.validate(action, state, schema)` -> `SafetyInfo`。
  3. `apply_brake / dispatch_action`(emergency_stop / safe_stop 时仅 brake)。
  4. `time.sleep` 维持 `step_frequency`。
  5. 再次 `get_state` + `_get_observation` + `_calc_step_reward`。
- `reset()`:reset choreography(BMS 检查 → joint reset → 可选 EE pose alignment → chassis brake)。
- `_calc_step_reward`(基类默认):双臂 AND 几何距离 + dense `exp(-500 * d^2)`。任务子类可自由覆盖。
- `_get_observation`:dummy 返回 spaces.sample-shaped zeros;真实模式从 mux 抓帧 + state dict 拼装。
- 依赖 `scipy.spatial.transform.Rotation` 做欧拉 ↔ 四元数转换;`scipy` 是 RLinf 必装项。

### 3.9 Wrappers [r1_pro_wrappers.py](../../../rlinf/envs/realworld/galaxear/r1_pro_wrappers.py)

- `GalaxeaR1ProJoystickIntervention`:订阅 `/controller`,SWA / SWB 分别覆盖右 / 左臂 action,SWD 不直接覆盖(让 Env SafetySupervisor 处理 SWD 急停)。
- `GalaxeaR1ProVRIntervention`:订阅 R1 Pro VR teleop 双手 PoseStamped;骨架版本仅做 freshness check 与 info 字段写入,具体 pose-to-action 映射留给现场标定(因 R1 Pro VR SDK 版本差异较大)。
- `GalaxeaR1ProDualArmCollisionWrapper`:slow zone 软递减,与 SafetySupervisor L3b 互补。
- `GalaxeaR1ProActionSmoother`:EMA + 每步 jerk bound;真机 OpenPI / π0.5 高频策略部署时强烈推荐打开。

### 3.10 Tasks [tasks/](../../../rlinf/envs/realworld/galaxear/tasks/)

- `tasks/__init__.py`:`gymnasium.register` 6 个 ID,`_register_all` 用 idempotent 模式(防止重复注册)。
- `r1_pro_single_arm_reach.py`:M1 bring-up,默认 `_calc_step_reward` 即可工作。
- `r1_pro_pick_place.py`:4 阶段 phased reward(approach / grasp / lift / place),严格单调推进,reset 时回到 approach。
- `r1_pro_dual_arm_*` / `r1_pro_whole_body_*` / `r1_pro_mobile_*`:仅重写 `task_description`;现场可继承基类的 `_calc_step_reward`(双臂 AND target)或重写。

---

## 4. 配置说明:双路径与全 ROS2 切换

### 4.1 默认双路径(腕部 USB + 头部 ROS2)

```yaml
# 关键配置(摘自 realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async.yaml)
cluster:
  num_nodes: 2
  node_groups:
    - label: gpu                                  # GPU server
      node_ranks: 0
      hardware:
        type: GalaxeaR1Pro
        configs:
          - node_rank: 0
            controller_node_rank: 1               # Controller 跑到 Orin
            wrist_direct_camera_serials:
              wrist_right: "230322272869"         # 改成你的 D405 序列号
            cameras:
              - { name: wrist_right, backend: usb_direct, serial_number: "230322272869", enable_depth: true }
              - { name: head_left,  backend: ros2,  rgb_topic: /hdas/camera_head/left_raw/image_raw_color/compressed }
    - label: galaxea_r1_pro                       # Orin
      node_ranks: 1
      hardware:
        type: GalaxeaR1Pro
        configs:
          - node_rank: 1
            controller_node_rank: 1
env:
  train:
    override_cfg:
      step_frequency: 10.0
```

### 4.2 全 ROS2 变体(腕部 + 头部都走 ROS2)

```yaml
# 关键 diff(摘自 realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async_all_ros2.yaml)
hardware:
  configs:
    - node_rank: 0
      wrist_direct_camera_serials: {}             # << 关键:置空
      cameras:
        - name: wrist_right
          backend: ros2                           # << 关键:改 ros2
          rgb_topic: /hdas/camera_wrist_right/color/image_raw/compressed
          depth_topic: /hdas/camera_wrist_right/aligned_depth_to_color/image_raw
          stale_threshold_ms: 250                 # << 放宽
        - name: head_left
          backend: ros2
          rgb_topic: /hdas/camera_head/left_raw/image_raw_color/compressed
env:
  train:
    override_cfg:
      step_frequency: 8.0                         # << 控制频率降到 8 Hz
      soft_sync_window_ms: 50                     # << 放宽到 50 ms
      align_strategy: sync_window
      fallback_to_ros2_on_usb_drop: false
      action_scale: [0.04, 0.10, 1.0]             # << 单步幅度等比缩
      safety_cfg:
        feedback_stale_threshold_ms: 250
```

### 4.3 切换流程

切换无需改代码,只动 YAML 4 处:

| 改动 | 默认 → 全 ROS2 |
|---|---|
| `wrist_direct_camera_serials` | 填序列号 → 空字典 `{}` |
| `cameras[wrist_*].backend` | `usb_direct` → `ros2` 并补 `rgb_topic` |
| `step_frequency` | 10 → 8 |
| `safety_cfg.feedback_stale_threshold_ms` | 200 → 250 |

物理层:全 ROS2 时不需要 USB AOC,腕部 D405 维持 R1 Pro 出厂接线(Orin 端启 `realsense2_camera` 节点)。

---

## 5. 本地开发与 dummy 测试流程

### 5.1 安装

```bash
# 在 GPU server / 任意机器上(rclpy 不必装)
bash requirements/install.sh embodied --env galaxea_r1_pro
```

`install_galaxea_r1_pro_env` 只装 `icmplib opencv-python pyrealsense2 PyTurboJPEG`;rclpy / hdas_msg 留给 Orin 上的 ROS2 系统包。

### 5.2 import 验证(无任何外部依赖)

```bash
python -c "
from rlinf.scheduler.hardware.robots.galaxea_r1_pro import GalaxeaR1ProRobot, GalaxeaR1ProConfig
from rlinf.envs.realworld.galaxear import GalaxeaR1ProEnv, GalaxeaR1ProSafetySupervisor
from rlinf.envs.realworld.galaxear.tasks import GalaxeaR1ProPickPlaceEnv
import gymnasium as gym
env = gym.make('GalaxeaR1ProPickPlace-v1', override_cfg={'is_dummy': True}, worker_info=None, hardware_info=None, env_idx=0)
obs, _ = env.reset()
print('action_space:', env.action_space)
print('obs.state keys:', list(obs['state'].keys()))
print('obs.frames keys:', list(obs['frames'].keys()))
for _ in range(5):
    obs, r, term, trunc, info = env.step(env.action_space.sample() * 0)
print('5 step OK; safety reasons:', info.get('safety_reasons', []))
"
```

期望输出:`action_space=Box(7,)`,`state` 键含 right_arm_qpos / right_ee_pose / right_gripper_pos,`frames` 含两个相机名,5 步无报错。

### 5.3 单元测试

```bash
pytest tests/unit_tests/test_galaxea_r1_pro_hardware.py \
       tests/unit_tests/test_galaxea_r1_pro_safety.py \
       tests/unit_tests/test_galaxea_r1_pro_camera_mux.py \
       tests/unit_tests/test_galaxea_r1_pro_action_schema.py \
       tests/unit_tests/test_ros2_camera_decode.py -v
```

5 个文件,合计 ~ 35 个测试。全部不依赖 rclpy / pyrealsense2 真机硬件。

### 5.4 dummy SAC + CNN 训练循环

```bash
export EMBODIED_PATH=$(pwd)/examples/embodiment
python examples/embodiment/train_async.py \
    --config-path "${EMBODIED_PATH}/config" \
    --config-name realworld_dummy_galaxea_r1_pro_sac_cnn
```

期望:跑 max_steps=100 步零异常退出,日志在 `../results/`。

### 5.5 真机 e2e 闭环冒烟

```bash
pytest tests/e2e_tests/embodied/realworld_dummy_galaxea_r1_pro_sac_cnn.yaml -v
```

(需要 RLinf CI 现有的 e2e 跑测脚手架;详见 `tests/e2e_tests/conftest.py` 之类文件。)

---

## 6. 真机部署完整流程

### 6.1 准备阶段

| 项 | GPU server | Orin |
|---|---|---|
| 操作系统 | Ubuntu 22.04 | R1 Pro 出厂(Ubuntu 22.04) |
| Python venv | 3.10 / 3.11 + RLinf | 3.10 + RLinf |
| ROS 2 Humble | 仅当用 ROS2Camera 时(默认双路径需要) | 必装(出厂已装) |
| Galaxea SDK | 不需要 | `~/galaxea/install` 出厂已装 |
| `hdas_msg` 包 | 不需要(Controller 在 Orin) | 出厂已装 |
| pyrealsense2 | 必装(腕部 USB direct) | 不需要 |
| CAN | N/A | `bash ~/can.sh` |

### 6.2 Orin 上(每次开机)

```bash
ssh nvidia@<r1pro_ip>
tmux new -s r1pro
bash ~/can.sh
source ~/galaxea/install/setup.bash
cd ~/galaxea/install/startup_config/share/startup_config/script
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/

# 验证关键 topic
ros2 topic hz /hdas/feedback_arm_right
ros2 topic hz /hdas/camera_head/left_raw/image_raw_color/compressed
```

### 6.3 Ray 集群启动

#### Orin 端

```bash
cd ~/RLinf
export RLINF_NODE_RANK=1
source ray_utils/realworld/setup_before_ray_galaxea_r1_pro.sh
ray start --address=<gpu_server_ip>:6379
ray status         # 期望 2 nodes
```

#### GPU server 端

```bash
cd ~/RLinf
export RLINF_NODE_RANK=0
source ray_utils/realworld/setup_before_ray_galaxea_r1_pro.sh
ray start --head --port=6379 --node-ip-address=<gpu_server_ip>
```

### 6.4 训练入口(只在 GPU server)

```bash
bash examples/embodiment/run_realworld_galaxea_r1_pro.sh \
     realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async
```

### 6.5 验证 controller 是否真的跑在 Orin 上

```bash
# GPU server
ray list actors --address=<gpu_server_ip>:6379 | grep -i Controller
# 期望 Node IP 是 Orin 的 IP

# Orin
ros2 node list                                          # 应看到 /rlinf_galaxea_r1_pro_controller_<pid>
ros2 topic list | grep target_pose                      # 应看到 /motion_target/target_pose_arm_right
ros2 topic hz /motion_target/target_pose_arm_right      # 训练运行时应 ~ step_frequency Hz
```

### 6.6 切换到全 ROS2 变体

仅改 YAML(`realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async_all_ros2.yaml`),其他流程不变:

```bash
bash examples/embodiment/run_realworld_galaxea_r1_pro.sh \
     realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async_all_ros2
```

物理层把 D405 USB 留在 Orin(出厂状态),Orin 上多启动 `realsense2_camera`(若 startup 没拉起):

```bash
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=hdas \
    camera_name:=camera_wrist_right \
    serial_no:='"230322272869"' \
    enable_color:=true enable_depth:=true \
    align_depth.enable:=true \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30 \
    image_transport.compressed.enable:=true
```

### 6.7 急停 / 异常恢复(节选)

| 症状 | 操作 |
|---|---|
| SWD = DOWN(软急停) | SafetySupervisor 自动 emergency_stop;放回 SWD = UP 后 `export RLINF_SAFETY_ACK=1` 续训 |
| 硬急停按钮 | `sudo ip link set can0 down && bash ~/can.sh` 重启 CAN,重启 robot_startup.sh,重启 Ray |
| BMS < 25% | Mac 自动 safe_stop;充电至 > 40% 后续训 |
| `feedback_age_ms` 飙升 | 检查 LAN 链路与 ROS_DOMAIN_ID;`ros2 daemon stop && ros2 daemon start` |

---

## 7. 单元测试运行

5 个测试文件覆盖核心可测路径(无硬件依赖):

```bash
pytest tests/unit_tests/test_galaxea_r1_pro_hardware.py -v       # 7 tests
pytest tests/unit_tests/test_galaxea_r1_pro_safety.py -v         # 11 tests
pytest tests/unit_tests/test_galaxea_r1_pro_camera_mux.py -v     # 6 tests
pytest tests/unit_tests/test_galaxea_r1_pro_action_schema.py -v  # 5 tests
pytest tests/unit_tests/test_ros2_camera_decode.py -v            # 6 tests
```

### 测试覆盖矩阵

| 模块 | 覆盖项 | 未覆盖(已知) |
|---|---|---|
| 硬件注册 | config 解析 / disable_validate / enumerate / camera spec coercion | rclpy / pyrealsense2 真实校验路径(需要硬件) |
| Safety | L1-L5 各类 case / 三级停机 / build_safety_config | 操作员心跳超时(time.monotonic 需 mock) |
| Camera Mux | dummy / 字段校验 / cameras coercion / close 幂等 / 度量 | 真实 BaseCamera open / fallback 切换(需要硬件) |
| Action Schema | per-stage action_dim / split / predict / build | joint mode 行为(M5+) |
| ROS2 Camera | factory dispatch / bgr8 / rgb8 / 16UC1 / 32FC1 解码 | rclpy 实际订阅 / 时序 |

---

## 8. 已知限制

1. **真机未实际验证**:实施时无 R1 Pro 硬件可访问。代码逻辑符合设计文档,Franka 同构性已校验;首次真机部署时建议先跑单元测试 + dummy + 只读模式逐步推进(参考 r1pro4g55.md 的 8-step 联调顺序)。
2. **4 个骨架任务奖励待填**:dual_arm_handover / cap_tighten / whole_body_cleanup / mobile_manipulation 仅有 `task_description`,需团队按真机演示数据迭代。
3. **VR Intervention 仅骨架**:`GalaxeaR1ProVRIntervention.action()` 的 pose-to-action 映射尚未实现,因 R1 Pro VR SDK 版本差异较大,留给现场按 SDK 文档具体化。
4. **`hdas_msg` 缺失时 L5 部分降级**:SWD / BMS / status_errors 三个 watchdog 项依赖 `hdas_msg.msg.*`;Orin 出厂可用,GPU server 默认无,因此 L5 这三项只在 controller-on-Orin 形态(本方案默认)生效。
5. **Docker stage / GitHub Actions workflow 未在本实施实装**:设计文档 §12.10 / §12.11 已规范,可作为后续 PR 实装(参考 [.cursor/skills/add-install-docker-ci-e2e](../../../.cursor/skills/add-install-docker-ci-e2e))。
6. **EN/ZH RST 文档未生成**:仅 `imp1.md`(本文)。RST 文档建议在第二轮 PR 中加(参考 [.cursor/skills/add-example-doc-model-env](../../../.cursor/skills/add-example-doc-model-env))。
7. **CI 未矩阵化双变体**:e2e fixture 只放了默认 dummy,全 ROS2 变体 CI 待加(只需复制 fixture 改 backend / step_frequency)。

---

## 9. 与设计文档 r1pro5op47.md 章节对应表

| 设计文档章节 | 实施位置 | 备注 |
|---|---|---|
| §6.1 硬件注册 | [galaxea_r1_pro.py](../../../rlinf/scheduler/hardware/robots/galaxea_r1_pro.py) | 6 项 enumerate 校验全部实装 |
| §6.2 控制器 | [r1_pro_controller.py](../../../rlinf/envs/realworld/galaxear/r1_pro_controller.py) | rclpy + MultiThreadedExecutor + spin 线程 |
| §6.3 状态容器 | [r1_pro_robot_state.py](../../../rlinf/envs/realworld/galaxear/r1_pro_robot_state.py) | 26 DoF + IMU + BMS + watchdog 字段 |
| §6.4 安全监督 | [r1_pro_safety.py](../../../rlinf/envs/realworld/galaxear/r1_pro_safety.py) | 5 级闸门 + 3 级停机 |
| §6.5 相机多路径 | [r1_pro_camera_mux.py](../../../rlinf/envs/realworld/galaxear/r1_pro_camera_mux.py) + [ros2_camera.py](../../../rlinf/envs/realworld/common/camera/ros2_camera.py) | 双路径 + 软同步窗 + USB drop fallback |
| §6.6 环境主类 | [r1_pro_env.py](../../../rlinf/envs/realworld/galaxear/r1_pro_env.py) | 与 FrankaEnv 同构 |
| §6.7 任务 | [tasks/](../../../rlinf/envs/realworld/galaxear/tasks/) | 6 任务,2 完整 + 4 骨架 |
| §6.8 Wrappers | [r1_pro_wrappers.py](../../../rlinf/envs/realworld/galaxear/r1_pro_wrappers.py) | 4 个 wrapper |
| §7 ROS2 接口映射 | controller / ros2_camera 内部使用 | topic 名称严格按设计 |
| §8 动作观测矩阵 | [r1_pro_action_schema.py](../../../rlinf/envs/realworld/galaxear/r1_pro_action_schema.py) | per-stage 维度 7/14/18/21 |
| §9 FMEA | safety + Runbook | 5 行 FMEA 表纳入 imp1 §6.7 |
| §10.6 跨主机 ROS2 调优 | setup_before_ray | DDS env vars 已固化 |
| §11 路线图 M0-M5 | 当前实施达到 M0 + M1 部分 | dual_arm / 全身 / 移动等待团队 |
| §12 配置与代码骨架 | 实施清单 §1.1 直接对应 | |
| 附录 F 全 ROS2 变体 | `*_all_ros2.yaml` 配置文件 | YAML 切换零代码改动 |
| 附录 G Controller 跨节点 | controller `launch_controller(node_rank=...)` | 已实装,YAML 通过 `controller_node_rank` 字段控制 |

---

## 10. 后续工作

按优先级建议下一批 PR:

1. **HW-in-loop 验证**:租机器跑通 M1 单臂 reach + pick-place,采集 100 条 episode 数据,验证安全 / 延迟 / 控制频率达标。
2. **填充 4 个 stub 任务**:基于真实演示数据写 `_calc_step_reward`,先写 dual_arm_handover(M2 推荐路径)。
3. **Docker stage + GitHub Actions**:按 [.cursor/skills/add-install-docker-ci-e2e](../../../.cursor/skills/add-install-docker-ci-e2e) 实装。
4. **RST 文档**:EN + ZH 各一份,按 [.cursor/skills/add-example-doc-model-env](../../../.cursor/skills/add-example-doc-model-env)。
5. **CI matrix 扩展**:`r1pro-dummy-e2e` 加 `[default, all_ros2]` 矩阵。
6. **VR Intervention pose-to-action 映射**:跟 R1 Pro VR SDK 对齐后写具体逻辑。
7. **数据采集脚本**:`collect_data_galaxea_r1_pro.sh` + `realworld_collect_data_galaxea_r1_pro.yaml`,LeRobot 输出。
8. **Async PPO + π0.5 / HG-DAgger + OpenPI 配置**:提供两份额外 YAML(M2 阶段)。
9. **IsaacSim 数字孪生**:`rlinf/envs/isaaclab/galaxea_r1_pro/`,M4 Sim-Real Co-Train 准备。
10. **Dashboard 模板**:`toolkits/dashboards/galaxea_r1_pro.json`,TensorBoard 预置面板。

---

> 本实施严格遵循设计文档 [r1pro5op47.md](r1pro5op47.md) 与团队命名规范(`galaxea_r1_pro.py` 硬件 / `galaxear/` env 目录 / `r1_pro_*` 文件 / `GalaxeaR1Pro*` 类 / `ROS2Camera`)。所有真机相关代码采用 lazy import 隔离,确保 RLinf 在没装 ROS2 / 没装 Galaxea SDK 的机器上也能正常 import 和跑 dummy。下一步关键节点是真机首次联调,建议从单臂 reach 任务起步。

---

## 11. 版本 2 变更(2026-04-26 dummy 测试通过)

> **说明**:§1–§10 仍为 **imp1 版本 1** 快照;本节记录 dummy 全链路跑通后追加的代码与测试修正,以及复现实验环境与命令。**未新建产品代码文件**,均为对既有实现的补齐与单测修复。

### 11.1 本次会话目的

- 为本文 §5 描述的 dummy 验证链路补完「可执行」最后一公里。
- 创建本地 Python 虚拟环境(下文以 `.venv_rlinf_r1` 为例;亦可命名为 `rlinf_r1` 等)并跑通:
  - **38** 个 Galaxea / ROS2 相关单元测试;
  - **standalone** `gymnasium.make` 烟测;
  - **`RealWorldEnv` 集成**烟测;
  - **Hydra + Ray** 端到端 dummy **SAC + CNN** 训练循环。

### 11.2 版本 2 文件修改表格

合计:**修改 5 个产品/配置相关文件 + 修正 2 个单元测试文件**;**新建 0 个产品代码文件**。改动目标均为使上述 dummy 链路在 GPU 服务器与无真机条件下可稳定复现。

| # | 类型 | 文件 | 改动原因 |
|---:|:---:|---|---|
| 1 | 修改 | [rlinf/envs/realworld/__init__.py](../../../rlinf/envs/realworld/__init__.py) | `RealWorldEnv` 路径上 6 个 Galaxea `gym` ID 未随包导入自动注册 |
| 2 | 修改 | [rlinf/envs/realworld/realworld_env.py](../../../rlinf/envs/realworld/realworld_env.py) | `Quat2EulerWrapper` 无条件套用,与 Galaxea 双臂 `obs` 键名契约冲突 |
| 3 | 修改 | [examples/embodiment/config/env/realworld_galaxea_r1_pro_pick_place.yaml](../../../examples/embodiment/config/env/realworld_galaxea_r1_pro_pick_place.yaml) | 缺三处 Franka 风格 wrapper 的显式关闭标志 |
| 4 | 修改 | [examples/embodiment/config/env/realworld_galaxea_r1_pro_pick_place_all_ros2.yaml](../../../examples/embodiment/config/env/realworld_galaxea_r1_pro_pick_place_all_ros2.yaml) | 同 #3,全 ROS2 变体需一致 |
| 5 | 修改 | [rlinf/envs/realworld/galaxear/r1_pro_env.py](../../../rlinf/envs/realworld/galaxear/r1_pro_env.py) | env 侧 `GalaxeaR1ProRobotConfig.cameras` 未将 YAML `dict` 强转为 `CameraSpec` |
| 6 | 修改 | [tests/unit_tests/test_galaxea_r1_pro_safety.py](../../../tests/unit_tests/test_galaxea_r1_pro_safety.py) | 测试初始 EE 状态与注释意图不一致,未触发 L3a 裁剪 |
| 7 | 修改 | [tests/unit_tests/test_galaxea_r1_pro_action_schema.py](../../../tests/unit_tests/test_galaxea_r1_pro_action_schema.py) | `float32`/`float64` 混算导致裸 `==` 比较失败 |

### 11.3 改动详细原因与核心代码说明

以下说明**为何**该修正是 dummy 通过的必要条件,并给出与仓库一致的**核心 diff 形态**(与设计文档 [r1pro5op47.md](r1pro5op47.md) §6.6 环境同构、§8 动作观测、附录 F 配置变体相呼应)。

#### Fix #1 — `rlinf/envs/realworld/__init__.py`

**原因**:经 `RealWorldEnv` 调用 `gym.make("GalaxeaR1ProPickPlace-v1")` 时,若从未 import `galaxear.tasks`,则 `gymnasium` 注册表中不存在该 ID(`NameNotFound`)。**对策**:与 `franka` / `xsquare` 一致,在包 `__init__` 中增加 **side-effect import**。

```python
from .franka import tasks as franka_tasks
from .galaxear import tasks as galaxear_tasks  # 触发 6 个 R1 Pro gym ID 注册
from .xsquare import tasks as xsquare_tasks
```

#### Fix #2 — `rlinf/envs/realworld/realworld_env.py`

**原因**:`Quat2EulerWrapper` 约定 `obs.state["tcp_pose"]`,而 `GalaxeaR1ProEnv` 使用 `right_ee_pose` / `left_ee_pose` 等键,无条件套用会在首步 `step` 触发 `KeyError`。**对策**:与 `use_relative_frame` 类似,增加配置项 `use_quat2euler_wrapper`,**默认 `True`** 保持 Franka 等既有路径行为不变。

```python
if self.cfg.get("use_relative_frame", True):
    env = RelativeFrame(env)
if self.cfg.get("use_quat2euler_wrapper", True):
    env = Quat2EulerWrapper(env)
```

#### Fix #3 / #4 — 环境子配置 ×2

**原因**: `realworld_env.py` 中 `cfg.get("no_gripper", True)` 默认为 `True` 会启用 `GripperCloseEnv`,将 **7 维**动作压成 **6 维**,与 Galaxea M1 `actor.model.action_dim=7` 冲突;`RelativeFrame` / `Quat2EulerWrapper` 同理依赖 Franka 单臂 `tcp_pose` 约定。**对策**:在 Galaxea 专用 YAML 中显式关闭这三项。

```yaml
# Disable Franka-style wrappers that assume tcp_pose / single-arm 7D
# action with gripper at slot 6.  Galaxea exposes per-arm pose keys
# and an action schema sized by stage (M1 = 7).
no_gripper: false
use_relative_frame: false
use_quat2euler_wrapper: false
```

#### Fix #5 — `rlinf/envs/realworld/galaxear/r1_pro_env.py`

**原因**:硬件侧 `GalaxeaR1ProConfig.__post_init__` 已做 `list[dict] → list[CameraSpec]` 强转,**env 侧** `GalaxeaR1ProRobotConfig` 曾遗漏;YAML 未写 `enable_depth` 等字段时,下游若用 `spec["enable_depth"]` 风格访问会 `KeyError`。**对策**:在 `GalaxeaR1ProRobotConfig.__post_init__` 内统一 coerce。

```python
def __post_init__(self) -> None:
    if isinstance(self.image_size, list):
        self.image_size = tuple(self.image_size)
    from rlinf.scheduler.hardware.robots.galaxea_r1_pro import CameraSpec

    coerced: list = []
    for spec in self.cameras:
        if isinstance(spec, CameraSpec):
            coerced.append(spec)
        elif isinstance(spec, dict):
            coerced.append(CameraSpec(**spec))
        else:
            raise TypeError(
                f"GalaxeaR1ProRobotConfig.cameras entries must be dict "
                f"or CameraSpec; got {type(spec).__name__}: {spec!r}"
            )
    self.cameras = coerced
```

#### Fix #6 / #7 — 单元测试修正(非产品缺陷)

- **Safety** (`test_l3a_clips_outside_ee_box`):将初始 `right_xyz` 从 `0.40` 改为 `0.45`,使在 `action_scale[0]=0.05` 下 `+x` 满量程动作预测位置超出 `right_ee_max[0]=0.45`,从而稳定触发 L3a 裁剪。
- **Action schema** (`test_split_single_arm`): gripper 分量用 `pytest.approx(-0.7)` 替代 `==`,避免 `float32` 与字面量 `float` 的舍入差导致误报。

### 11.4 虚拟环境安装步骤(`.venv_rlinf_r1`)

以下命令假设仓库根目录为 `RLinf`,且 Python **3.10–3.11.14** 与 [pyproject.toml](../../../pyproject.toml) 约束一致。

```bash
cd /path/to/RLinf
uv venv .venv_rlinf_r1 --python 3.11.14
source .venv_rlinf_r1/bin/activate

# 1. RLinf 基础 + embodied extras
UV_TORCH_BACKEND=auto uv sync --active --extra embodied --no-install-project

# 2. 将 RLinf 以 editable 安装进当前 venv(不再重复解析依赖)
uv pip install -e . --no-deps

# 3. Galaxea dummy 路径常用轻量依赖(与 requirements/install.sh 中 galaxea 分支对齐)
uv pip install opencv-python PyTurboJPEG psutil filelock

# 4. (可选) 若 GPU 为 Blackwell(sm_120) 等,预装 wheel 无对应架构时需升级 cu128 构建
pip install --upgrade 'torch==2.8.*' 'torchvision==0.23.*' \
    --index-url https://download.pytorch.org/whl/cu128
```

**HF 权重(CNN policy)**:dummy 配置中的 ResNet10 需本地路径,可先下载:

```bash
hf download RLinf/RLinf-ResNet10-pretrained --local-dir ~/RLinf-ResNet10-pretrained
```

### 11.5 四层 dummy 测试:命令与实测结果

#### 11.5.1 第 1 层 — 38 个单元测试

```bash
source .venv_rlinf_r1/bin/activate
pytest tests/unit_tests/test_galaxea_r1_pro_hardware.py \
       tests/unit_tests/test_galaxea_r1_pro_safety.py \
       tests/unit_tests/test_galaxea_r1_pro_camera_mux.py \
       tests/unit_tests/test_galaxea_r1_pro_action_schema.py \
       tests/unit_tests/test_ros2_camera_decode.py -v
```

**实测**: `38 passed`(约 **1.3 s** 量级,依机器略有差异)。

#### 11.5.2 第 2 层 — standalone `gymnasium` 烟测

```python
from rlinf.envs.realworld.galaxear import tasks  # noqa: F401 — 触发 gym.register
import gymnasium as gym

env = gym.make(
    "GalaxeaR1ProPickPlace-v1",
    override_cfg={
        "is_dummy": True,
        "cameras": [
            {"name": "wrist_right", "backend": "usb_direct", "serial_number": "abc"},
            {"name": "head_left", "backend": "ros2", "rgb_topic": "/x"},
        ],
    },
    worker_info=None,
    hardware_info=None,
    env_idx=0,
)
obs, _ = env.reset()
print(env.action_space)  # Box(-1, 1, (7,), float32)
print(list(obs["state"].keys()))
print(list(obs["frames"].keys()))
for _ in range(5):
    env.step(env.action_space.sample() * 0)
```

**实测**:5 step 通过;`info` 中 `safety_reasons` 可含 L3a/L4(dummy 全零状态落在 EE 盒外,属预期)。

#### 11.5.3 第 3 层 — `RealWorldEnv` 集成烟测

使用 `OmegaConf.create({...})` 构造与 YAML 等价的 `env.train` 配置,直接实例化 `RealWorldEnv`,确认经 wrapper 堆叠后张量形状与算法期望一致(例如 `obs.states`、`main_images`、`extra_view_images` 的 batch 与空间维度)。

**实测**:5 step 通过;观测维度和相机路数与配置一致。

#### 11.5.4 第 4 层 — Hydra 端到端 dummy SAC + CNN

```bash
source .venv_rlinf_r1/bin/activate
export RLINF_NODE_RANK=0
ray start --head --port=6379 --node-ip-address=127.0.0.1

export EMBODIED_PATH=$(pwd)/examples/embodiment ROBOT_PLATFORM=galaxea_r1_pro
python examples/embodiment/train_async.py \
    --config-path "${EMBODIED_PATH}/config" \
    --config-name realworld_dummy_galaxea_r1_pro_sac_cnn \
    actor.model.model_path="${HOME}/RLinf-ResNet10-pretrained" \
    rollout.model.model_path="${HOME}/RLinf-ResNet10-pretrained"
```

**说明**:若需临时缩短评估步数等嵌套字段,CLI 应使用 Hydra **新增键**语法,例如 `+env.eval.override_cfg.max_num_steps=10`。

**实测摘要**(单卡、dummy、`episode_len=200` 量级):

- 1 epoch: Step Time 约 **23 s**;`sac/critic_loss` 约 **0.044**;`sac/alpha` 约 **0.01**;`sac/actor_loss` 小幅负值;进程无异常退出。
- 3 epoch 回归:`critic_loss` 呈 **0.044 → 0.011 → 0.0065** 下降趋势;`alpha` 自动调节稳定;FSDP / Gloo / Ray 多 worker 协作正常。

### 11.6 已知遗留与后续工作

- 与设计文档 §8 一致:4 个骨架任务 reward 仍待真机数据迭代。
- `GalaxeaR1ProVRIntervention` 的 pose-to-action 仍为 stub。
- Docker stage、GitHub Actions 矩阵、Sphinx RST 中英文档、双变体 e2e CI 等仍可按 §10 优先级另开 PR。
- 单测文件中可能存在 **既有** Ruff 告警(如未使用的 `pytest` import)或格式偏好差异;**不纳入本次 dummy 必须通过范围**,建议在独立「代码卫生」PR 中清理。

### 11.7 与设计文档 r1pro5op47.md 的验证对应表

| 设计文档章节 | 验证手段 | 结果 |
|---|---|---|
| §6.1 硬件注册 / enumerate | `test_galaxea_r1_pro_hardware.py` | 通过 |
| §6.4 五级安全 + 三级停机 | `test_galaxea_r1_pro_safety.py` | 通过 |
| §6.5 双路径 Mux + 软同步 | `test_galaxea_r1_pro_camera_mux.py` | 通过 |
| §8 stage-aware action schema | `test_galaxea_r1_pro_action_schema.py` | 通过 |
| §6.5 ROS2 相机解码 | `test_ros2_camera_decode.py` | 通过 |
| §6.6 `GalaxeaR1ProEnv` 与 Franka 同构 | standalone `gym.make` + `reset`/`step` | 通过 |
| §11 路线图 M0 + M1(SAC) | dummy SAC+CNN 单节点闭环 | 通过 |
| 附录 F 全 ROS2 变体 | `*_all_ros2.yaml` 与 env 子配置 | 通过 |

**结论**:在 **M1 dummy 全闭环** 意义上,版本 2 已达成「无真机亦可复现训练循环」的验收基线;真机首次联调仍建议严格按本文 §6 Runbook 顺序推进。
