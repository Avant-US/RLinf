# 方块放置（cube place）落地手册 v2

**这一版是可执行的运行手册**，不是设计稿。设计结论已在 v1 锁定并通过阶段 1；v2 的改动全部来自真机踩坑：`robot_mode` 总闸、ROS 与 franky 两套阻抗的本质差异、以及「哑失败」如何变成可见错误。

| | |
|---|---|
| 上一版 | [`dmo_place_1.md`](dmo_place_1.md)（设计推导、charger 拆解，仍可查） |
| 运行记录 | [`dmo_place_1LOG.md`](dmo_place_1LOG.md)（LOG-001…LOG-017） |
| 人机 / 急停协议 | [`charger_sac_async.md`](charger_sac_async.md) §6.2.1、§15.5 |
| 代码 | `b/x/`（**未改** `rlinf/`） |

**当前状态**

| 阶段 | 状态 |
|------|------|
| 阶段 1A `gym.make` dummy | **PASS**（可随时回归） |
| 阶段 1B GPU dummy SAC | **PASS** |
| 阶段 2.5 H1 标定 | **PASS**，`calibrated: true` 已落盘 |
| 阶段 2.6 `connect` 只读几何 | **PASS** |
| **阶段 2.4 运动链路体检** | **未做**（v2 新增门闩，必须先过） |
| 阶段 2.7 `reset` 到悬停 | 反复 FAIL，根因 = user-stop 被按住（LOG-017），**代码无关**，待重跑 |
| 阶段 2.8 `box` 短跑 | 未做 |
| 阶段 3 在线 SAC | 未开始，**配置与启动脚本尚未创建**（§7） |

---

## 0. 怎么用这份文档

- **第一次上手：** 读 §1（本机基线）→ §2（任务）→ §4 按阶段做。
- **今天就要跑 reset：** 读 §1.4 + §4 阶段 2 全节，一步不跳。
- **出错了：** 直接查 §6 排障总表（按症状索引）。
- **写代码前：** 读 §3，那里写的是**代码现在的实际行为**，不是计划。

三条铁律，违反其一必然浪费一场：

1. **动臂之前先确认 `robot_mode == RobotMode.Idle`**（§1.4）。
2. **franky 上任何「命令发了但臂不动」，第一步查模式和 `is_running`，不是改几何**（§1.3）。
3. **同一时刻只能有一个 libfranka 客户端**（标定 REPL / 烟测脚本 / 训练三者互斥）。

---

## 1. 本机软硬件基线

### 1.1 机器与控制后端

| 项 | 本机实际 | 官方 charger 例子 |
|----|----------|-------------------|
| 机器人 | Franka Panda + **原装 Franka Hand** | 同型号 + Robotiq |
| 固件 | **5.10.0** | < 5.9 |
| 控制库 | **libfranka 0.19 + franky**（`franky.Robot` / `franky.Gripper`） | **ROS** franka_ros |
| 控制器类 | `FrankyController` → 本机 `FrankyControllerExtended` | `FrankaController` |
| 阻抗 | `franky.CartesianImpedanceTracker`（异步 torque motion） | `roslaunch impedance.launch` 常驻节点 |
| Gym ID | **`FrankyCubePlaceEnv-v1`** | `PegInsertionEnv-v1` |
| 内核 | 5.15 realtime | — |

**固件 5.10 + 原装 Hand 是本机全部特殊性的根源**：不能用 ROS 那条链路，也不能用 upstream 的 `toolkits.realworld_check.test_franky_controller`（它 `_build_gripper` 只声明支持 Robotiq，会 `NotImplementedError`）。

### 1.2 容器 / venv / 路径

| 项 | 值 |
|----|-----|
| 宿主机仓库 | `/home/nvidia/bt/s/RLinf` |
| 容器内路径 | `/workspace/RLinf` |
| 控制容器 | `rlinf-franky-5090`，镜像 `rlinf/rlinf:agentic-rlinf0.4-franka` |
| venv | **`franky-0.19.0`** → `/opt/venv/franky-0.19.0/bin/python` |
| 训练容器（阶段 1B / 3） | 镜像 `rlinf/rlinf:agentic-rlinf0.4-maniskill_libero` |
| 机器人 IP / NIC | `172.16.0.2` / `eno1`（Ray 通信走 `eno2`） |
| ResNet10 权重 | `/home/nvidia/ckpts/RLinf-ResNet10-pretrained/resnet10_pretrained.pt` |

**陷阱：** 本机另有一个名为 `rlinf` 的容器，挂的是 `cxy_ws/RLinf`，**不要用**。只用 `rlinf-franky-5090`。

`source b/x/configs/setup_before_ray_5090.sh` 做的事（每开一个新 bash 都要重做）：`switch_env franky-0.19.0`、`PYTHONPATH` 加 `b/x`、`PYTHONSTARTUP` 注册 franky Gym ID、`RLINF_EXT_MODULE=franky_ext.runtime_bootstrap`、`RLINF_COMM_NET_DEVICES=eno2`、`FRANKA_ROBOT_IP=172.16.0.2`、`FRANKA_GRIPPER_TYPE=franka`。

### 1.3 ROS 阻抗 vs franky 阻抗（决定所有运动代码的写法）

同名的 `move_arm`，两边底下完全不是一回事。**这张表是 v2 最重要的内容。**

| | `FrankaController`（ROS，charger） | `FrankyController`（本机） |
|---|---|---|
| 阻抗如何启动 | `__init__` 里 `start_impedance()` → `roslaunch impedance.launch`，`cartesian_impedance_controller` **常驻 1 kHz** | `CartesianImpedanceTracker(...)`，**构造器内**执行 `robot.move(motion, asynchronous=True)` |
| `move_arm` 做什么 | 往 `/cartesian_impedance_controller/equilibrium_pose` **发 topic** | `set_target()` 写 `CartesianReferenceHandle` |
| 控制器死掉后 | ROS 节点还在，topic 继续被消费，能自愈 | 异步线程**存下异常后退出**；`set_target` 写死句柄，**静默无效** |
| 如何得知它死了 | ROS 侧可见 | 只能查 `tracker.is_running`（== `robot.is_in_control`）；真因要 `stop()` / `join_motion()` 才重抛 |
| reset 大位移怎么走 | `_interpolate_move` 10 Hz 阻抗路点 | RLinf 自己的 franky env（`DualFrankaEnv._go_to_rest`）用**阻塞 `reset_joint`**；阻抗只用于 `step` 小增量 |

`franky/tracker.py`（容器内 `/opt/venv/franky-0.19.0/lib/python3.11/site-packages/franky/tracker.py`）三行关键实现：

```python
motion = CartesianImpedanceTrackingMotion(reference_handle=self._reference_handle, **kwargs)
self._robot.move(motion, asynchronous=True)   # 构造即启动，不需要 __enter__
...
@property
def is_running(self): return self._robot.is_in_control
```

由此得到四条**不要再犯**的结论：

1. `__enter__` 只 `return self`，构造器已经 `move` 过了。不要为了「启动」去调它。
2. franky 的 `translational_error_clip` **不是** charger `compliance_param` 里的 `translational_clip_x/y/z`——后者是 ROS 控制器的 dynamic_reconfigure 键。把 3 mm 映射过去会把 xy 权限砍到 6 N。
3. `dz=0` 的第一嫌疑永远是「模式不对 / 异步 motion 已死」，不是「目标算错」。
4. 大位移用阻塞 `robot.move`、小增量用阻抗，这是 RLinf 自己在 franky 上的既有分工（`DualFrankaEnv`），不是发明。

### 1.4 `robot_mode`：运动的总闸

`franky.RobotState.robot_mode` 有 7 个值，**只有 `Idle` 能执行运动**：

| `robot_mode` | 含义 | 处理 |
|---|---|---|
| `Idle` | 可以运动 | 正常 |
| **`UserStopped`** | **手持设备 user-stop 被按下** | **松开（按钮抬起）** |
| `Guiding` | 引导键被按住 | 松开引导键 |
| `Reflex` | 碰撞 / 限位已锁存 | Desk 清 fault |
| `Move` / `AutomaticErrorRecovery` / `Other` | 正在动 / 恢复中 | 等 |

`UserStopped` 极其阴险，因为它**只挡运动**：

| 在 `UserStopped` 下 | 表现 |
|---|---|
| `franky.Robot(ip)` 连接 | **成功** |
| `robot.state` / `O_T_EE` / `q` 读取 | **正常**（所以探测的 TCP、H1、盒子全是对的） |
| `franka::Gripper` 命令 | **正常**（所以 `holding=True width=0.0463m`、跳过 `grasp` 全按预期走） |
| `robot.has_errors` | **False**（不是「有错误」，是「模式不对」） |
| `robot.move(..., asynchronous=True)` | **立刻返回**，`is_in_control` 有约 100 ms 为 True、`signal=Torques`，然后线程死掉 |
| 任何 `robot.move` 的真实结果 | `ControlException: libfranka: Move command rejected: command not possible in the current mode ("User stopped")!` |
| `control_command_success_rate` | **0.0**（最直接的旁证） |

于是 `reset` 会一路打印 `reset OK`、几何全对、`dz=0.0000`。**LOG-011 到 LOG-016 全部 `dz=0` 都是这一个原因**，与 `recover_from_errors`、`CartesianMotion` vs 阻抗、`translational_error_clip`、两段 reset 顺序统统无关。

> **「人在急停旁」= 手边有急停按钮，不是按住 user-stop。** 运行期间 user-stop 必须是抬起状态；真出事再拍。

只读检查（不动臂，随时可跑）：

```bash
python b/x/scripts/diag_franky_motion.py --probe
```

---

## 2. 任务定义（V1，沿用 v1 §3）

### 2.1 一句话

操作员**先把方块夹紧**；策略在安全盒内把方块移到台面标记并**碰到**它（TCP 进入标定成功区）；**夹爪全程闭合**；reset 时仍夹着方块抬到目标上方悬停，再试下一次。

不是张爪放下，不是桌面自主抓取。

### 2.2 关键决定

| 项 | 决定 |
|----|------|
| 成功判据 | 仅 TCP xyz 进入 `target_ee_pose` 的 `reward_threshold`（≈1 cm）。**不看**夹爪、不看力 |
| 几何锚 `target_ee_pose` | **夹着方块、方块已贴住标记时**的 TCP 六元组（欧拉 xyz，米/弧度） |
| 动作 | **6D**，`no_gripper` 保持默认 **True** → `GripperCloseEnv` |
| 算法栈 | 模仿 charger：在线 SAC + ResNet10 + 单腕 `wrist_1` + 稠密 xyz。无 RLPD / demo / 键盘打标 |
| reset | PegInsertion 式**闭爪**抬升 → 悬停（§3.3） |
| 终止 | `ignore_terminations: True`（碰到后本回合继续，满步再抬走） |
| 奖励 | 直接用 `FrankaEnv._calc_step_reward`：进区 → `1.0`；否则 `exp(-500·‖Δxyz‖²)` |

### 2.3 明确不做

张爪把方块留桌上；桌面自主抓取；7D 夹爪策略；改 `rlinf/` 的 `register(...)`；改官方 charger YAML / `run_realworld_async.sh`。

### 2.4 现场道具

Franka Panda + 原装 Hand；边长 3–5 cm 立方块；台面上**固定**的标记（挪了必须重标 H1）；腕部 RealSense → `wrist_1`；SpaceMouse（可选，只纠位置不能张爪）；急停在手边。

---

## 3. 已落地的代码资产（实际行为）

### 3.1 文件清单（均已存在）

| 路径 | 作用 |
|------|------|
| [`b/x/franky_ext/controller_extended.py`](../../../b/x/franky_ext/controller_extended.py) | `FrankyControllerExtended`：`move_arm`、`reconfigure_compliance_params`、`move_gripper`、Franka Hand、**tracker 存活检查** |
| [`b/x/franky_ext/franka_libfranka_gripper.py`](../../../b/x/franky_ext/franka_libfranka_gripper.py) | 原装 Hand：轻力 `grasp`（20 N，上限 40 N）、已夹持则 `skip grasp` |
| [`b/x/franky_ext/franky_single_franka_env.py`](../../../b/x/franky_ext/franky_single_franka_env.py) | mixin：换控制器、`safe_smoke_hold`、跳相机 |
| [`b/x/franky_ext/tasks/cube_place.py`](../../../b/x/franky_ext/tasks/cube_place.py) | `CubePlaceConfig` + `FrankyCubePlaceEnv.go_to_rest` |
| [`b/x/franky_ext/tasks/register.py`](../../../b/x/franky_ext/tasks/register.py) | 注册 `FrankyCubePlaceEnv-v1` / `FrankyPegInsertionEnv-v1` / `FrankyFrankaEnv-v1` |
| [`b/x/franky_ext/tcp_probe.py`](../../../b/x/franky_ext/tcp_probe.py) | 子进程读 pose + **`robot_mode`**；`require_motion_ready` / `describe_robot_mode` |
| [`b/x/configs/env/realworld_cube_place.yaml`](../../../b/x/configs/env/realworld_cube_place.yaml) | Hydra env 包，`id: FrankyCubePlaceEnv-v1` |
| [`b/x/configs/realworld_cube_place_dummy_sac_gpu.yaml`](../../../b/x/configs/realworld_cube_place_dummy_sac_gpu.yaml) | 阶段 1B GPU dummy SAC |
| [`b/x/configs/cube_place_target_ee_pose.yaml`](../../../b/x/configs/cube_place_target_ee_pose.yaml) | H1 六元组，**已 `calibrated: true`** |
| [`b/x/scripts/diag_franky_motion.py`](../../../b/x/scripts/diag_franky_motion.py) | **v2 新增**：绕开 Ray/env 直连 franky 判定运动链路 |
| [`b/x/scripts/test_franky_controller_ext.py`](../../../b/x/scripts/test_franky_controller_ext.py) | 标定 REPL（`open`/`close`/`getpos_euler`/`impedance`…） |
| [`b/x/scripts/write_cube_place_pose.py`](../../../b/x/scripts/write_cube_place_pose.py) | 把六个数写入 H1 YAML 并置 `calibrated: true` |
| [`b/x/scripts/step_cube_place_robot.py`](../../../b/x/scripts/step_cube_place_robot.py) | 真机烟测：`--connect-only` / `--reset-only` / 默认 box |
| [`b/x/scripts/step_cube_place_dummy.py`](../../../b/x/scripts/step_cube_place_dummy.py) | 阶段 1A 源码门闩 |
| [`b/x/scripts/run_cube_place_phase2.sh`](../../../b/x/scripts/run_cube_place_phase2.sh) | 子命令封装 `calibrate / write-pose / connect / reset / box` |

### 3.2 关键默认值（与代码逐一对齐）

`CubePlaceConfig`（继承 `PegInsertionConfig`）：

| 键 | 值 | 说明 |
|----|-----|------|
| `clip_x_range` / `clip_y_range` | `0.05` | 安全盒 xy 半宽 |
| `clip_z_range_low` | `0.005` | 只允许比接触点再低 5 mm |
| `clip_z_range_high` | `0.08` | 悬停高度 = 盒顶 |
| `random_xy_range` | `0.03`（烟测里被覆盖为 `0.0`） | |
| `clip_rz_range` / `random_rz_range` | `0.35` | |
| `reset_z_lift_m` | `0.10` | **相对当前 TCP** 的抬升，≠ `clip_z_range_high` |
| `reward_threshold[:3]` | `0.01`（基类） | 「碰到」容差 |
| `compliance_param.translational_stiffness` | `2000`（基类 PegInsertion） | reset 时 `reconfigure` 下发 |
| `compliance_param.translational_damping` | `89` → `tc = 2·89/2000 = 0.089` | |

franky 侧参数（`franky_controller.py` 模块常量，可用环境变量覆盖）：

| 常量 | 默认 | 环境变量 |
|------|------|----------|
| 平移刚度 | `500` N/m（**但 reset 会被 `compliance_param` 改成 2000**） | `RLINF_CART_K_T` |
| 旋转刚度 | `40` → reset 后 `150` | `RLINF_CART_K_R` |
| 零空间刚度 | `5` | `RLINF_CART_K_NS` |
| 误差截断 | `0.05` m / `0.3` rad | `RLINF_CART_ERR_CLIP_M` / `_RAD` |
| 每周期力矩增量 | `0.3` Nm | `RLINF_CART_MAX_DTAU` |
| 单次调用最大目标步进 | `0.10` m / `0.30` rad | `RLINF_CART_MAX_STEP_M` / `_RAD` |
| `relative_dynamics_factor` | `0.2` | — |

已标定的 H1（现存文件内容）与由它推出的几何：

```
target_ee_pose (H1 接触点) = [0.7062065, 0.03620906, 0.23192134, -3.11614319, 0.02628124, 0.17913087]
reset_ee_pose (悬停)       = H1 + [0,0,0.08,0,0,0]  →  z = 0.3119
ee_pose_limit_min          = [0.6562, -0.0138, 0.2269, -3.4661, -0.3237, -0.1709]
ee_pose_limit_max          = [0.7562,  0.0862, 0.3119, -2.7661,  0.3763,  0.5291]
```

### 3.3 `reset` 的确切三段行为

`FrankyCubePlaceEnv.go_to_rest` 逐字对齐 `PegInsertionEnv.go_to_rest`：

1. `_end_effector_action([-1.0])` —— 闭爪。**已夹持时 `FrankaLibfrankaGripper.close` 打印 `skip grasp` 直接返回**，不会重复 `grasp`（LOG-010 修复）。
2. `_move_action(当前 TCP)` —— 把阻抗平衡点钉在此刻。ROS 上是发 topic；franky 上还负责在 `reconfigure_compliance_params` 停掉 tracker 后**重新拉起** tracker。
3. `_interpolate_move(当前 z + reset_z_lift_m)` —— **相对当前 TCP** 抬 10 cm（charger 用来拔插头；这里用来把方块带离标记）。
4. `FrankaEnv.go_to_rest` —— 插值到 `reset_ee_pose` = H1 + 8 cm。

**门闩只看第 4 步之后**：xy 误差 ≤ 0.03 m，z 误差 ≤ 0.025 m。第 3 步可能短暂高于盒顶，原版 charger 也这样，`_interpolate_move` **不**按 `ee_pose_limit` 裁剪（裁剪只发生在 `step()`）。

### 3.4 哑失败护栏（v2 新增，务必知道它会怎么报错）

| 位置 | 行为 |
|------|------|
| `tcp_probe.require_motion_ready` | 模式非 `Idle` → 抛出带处理建议的 `RuntimeError` |
| `step_cube_place_robot.py` | 打印 `robot_mode`；`reset`/`box` 在 `gym.make` **之前**拦下（省掉 30 s Ray 启动）；`connect` 只提示不拦 |
| `controller_extended._ensure_cart_tracking_motion` | 建 tracker 前检查模式，非 `Idle` 直接抛 |
| `controller_extended.move_tcp_pose` | `super()` 之后检查 `is_running`；已死则 `stop()`（`join_motion` 重抛真因）、清 tracker、`log_error` 带 `mode=`、`raise RuntimeError` |
| `diag_franky_motion.py --probe` | 打印 `robot_mode` + 建议；非 `Idle` 时**跳过**运动测试（`--force` 可强来） |
| tracker 启动日志 | 追加 `is_running=` 字段 |

**所以从此以后：要么臂真的动，要么日志里有一条明确原因。不会再出现「`reset OK` 但 `dz=0.0000`」。**

已撤回的四处错误改动（不要再加回来）：`clear_errors` 跟踪中不 recover、mixin `_move_action` 不 `_clear_error()`、tracker 构造后调 `__enter__`、`translational_clip_*` 映射成 franky `translational_error_clip`。四处均已被证伪，撤回后回到上游行为。

---

## 4. 一步一步落地

### 阶段 0 — 只读自检（不动臂，任何时候都能做）

宿主机：

```bash
ping -c 3 172.16.0.2
ip route get 172.16.0.2                                          # 应走 eno1
ss -tn state established '( dport = :1337 or sport = :1337 )'     # 应无连接
docker ps --filter name=rlinf-franky-5090 --format '{{.Names}} {{.Status}}'
```

浏览器 `http://172.16.0.2/desk`：**Unlock 关节**、无 safety violation、**Activate FCI**。

**启动 / 进入容器**：先看上面 `docker ps` 那行的输出判断容器是否已在跑（没看到就再跑一次 `docker ps --filter name=rlinf-franky-5090 --format '{{.Names}} {{.Status}}'`）。

- **有输出**（容器在跑）→ 直接开新 shell 进去，不要再 `docker run`：

  ```bash
  docker exec -it rlinf-franky-5090 bash
  ```

- **无输出**（容器不存在）→ 用启动脚本创建并进入，它是 `docker run -it --rm ...`（前台阻塞，退出即销毁容器）：

  ```bash
  bash /home/nvidia/bt/s/RLinf/b/x/configs/docker_run_franky_5090.sh
  ```

  该脚本固定参数：镜像 `rlinf/rlinf:agentic-rlinf0.4-franka`、容器名 `rlinf-franky-5090`、`--privileged --network host`（franky/libfranka 需要）、把 `/home/nvidia/bt/s/RLinf` 挂到 `/workspace/RLinf`。**这个终端此后就是容器的前台 shell，别关它**；之后如果还需要第二个 shell（比如训练跑着时另开一个查日志或跑诊断脚本），回到上一步用 `docker exec -it rlinf-franky-5090 bash` 从别的终端接进同一个容器，不要再跑一次 `docker_run_franky_5090.sh`（`--name` 冲突）。

- **`docker ps -a` 里看到 `rlinf-franky-5090` 是 `Exited`**（少见，正常应因 `--rm` 自动清掉）→ 先 `docker rm rlinf-franky-5090` 再按上面「无输出」那条重新创建。

进容器后，每个新 shell 都要重新初始化：

```bash
source /workspace/RLinf/b/x/configs/setup_before_ray_5090.sh
which python                                   # 必须 /opt/venv/franky-0.19.0/bin/python
python -c "import franky; print('franky ok')"
python b/x/scripts/diag_franky_motion.py --probe
```

**`--probe` 必须看到 `robot_mode=RobotMode.Idle`。** 否则按 §1.4 处理后重跑，不要继续往下。

失败信号：`switch_env not found` = 不在 franky 镜像（容器/镜像不对，检查是否连进了别的容器）；`which python` 仍是 `franka-0.15.0` = 没 `source` 成功；`docker exec` 报 `No such container` = 上一步其实没启动成功，回去看 `docker ps -a` 有无报错退出的记录。

### 阶段 1 — dummy 回归（无臂，已 PASS）

每次改过 `b/x/` 之后都要跑一次，1–2 秒出结果。

**1A（franky 容器内，最省事）：**

```bash
RLINF_SKIP_CAMERA=1 python b/x/scripts/step_cube_place_dummy.py
# 期望末行: Phase1A PASS FrankyCubePlaceEnv-v1
```

**1B（GPU 容器，主验收，改了模型/YAML 才需要）：**

```bash
docker run --rm --gpus all --privileged --network host --shm-size=20g \
  -e RLINF_RESNET10_PATH=/home/nvidia/ckpts/RLinf-ResNet10-pretrained \
  -e RLINF_SKIP_CAMERA=1 \
  -v /home/nvidia/bt/s/RLinf:/workspace/RLinf \
  -v /home/nvidia/ckpts:/home/nvidia/ckpts:ro \
  -w /workspace/RLinf \
  rlinf/rlinf:agentic-rlinf0.4-maniskill_libero \
  bash -lc 'source b/x/configs/setup_before_ray_gpu_5090.sh && bash b/x/scripts/run_cube_place_dummy_sac_gpu.sh'
```

**禁止**在 GPU 容器里跑 `step7_install_deps.sh`（会把 CUDA torch 装成 CPU 版）。

门闩：

| ID | 通过标准 |
|----|----------|
| 1A-1 | 日志出现 `FrankyCubePlaceEnv-v1`，`gym.make` 成功 |
| 1A-2 | wrapper 后 `action_space.shape == (6,)` |
| 1A-3 | `clip_x/y_range==0.05`，`clip_z_range_low==0.005`，`clip_z_range_high==0.08` |
| 1A-4 | `go_to_rest` 源码含 `_end_effector_action([-1.0])`，不含 `+1.0` 张爪 |
| 1A-5 | `reset` + `step` 不连 FCI、不抛错 |
| 1B-1 | 容器内 `torch.cuda.is_available()` |
| 1B-2 | 出现 `train/sac` 类指标或 actor 更新；进程 exit 0 |
| 1B-3 | 无 `state_dim`/`action_dim` 不匹配（19 / 6） |
| 1B-4 | env id 是 `FrankyCubePlaceEnv-v1`，不是 `PegInsertionEnv-v1` |

### 阶段 2 — 真机

#### 2.0 谁做什么、会不会动臂

| 子步 | 执行者 | 动臂 | 占 FCI | 产物 / 门闩 |
|------|--------|------|--------|-------------|
| 2.1 安全规则 | 人 | 否 | — | 默读完 |
| 2.2 检查单 | 人 | 否 | Desk 激活 | `robot_mode=Idle` |
| 2.3 进容器 | 人 | 否 | 否 | `which python` 正确 |
| **2.4 运动链路体检** | `diag_franky_motion.py` | **是**（3 cm） | 直连，短 | 三项 OK |
| 2.5 H1 标定 | 人 + REPL | 人手引导 | **是** | `calibrated: true` |
| 2.6 `connect` | `--connect-only` | **否** | 子进程短连后释放 | 几何打印正确 |
| 2.7 `reset` | `--reset-only` | **是** | env 会话 | 悬停几何达标 |
| 2.8 `box` | 默认 | **是** | env 会话 | 不张爪、不砸盒底 |

**互斥：** 2.4 / 2.5 的脚本必须先退出（REPL 敲 `q`），才能跑 2.6–2.8。

#### 2.1 安全规则（开跑前默读）

1. **急停在手边可及。** 台面标记贴牢，方块厚度与训练时一致。
2. **手持设备的 user-stop 必须松开（按钮抬起）。** 按下时 `robot_mode=UserStopped`，libfranka 拒绝一切运动，而状态读取与夹爪照常 → `reset` 看似跑完但 `dz=0.0000`（§1.4、LOG-017）。「人在急停旁」= 手边有急停，**不是**按住 user-stop。
3. 2.4 / 2.7 / 2.8 之前：方块夹紧；用引导键把臂放到**标记上方几厘米**。**不要**从工厂 `home` 关节位横扫过去。
4. 只用 `python b/x/scripts/test_franky_controller_ext.py` 做标定。**不要**用 ROS 的 `test_franka_controller`，**不要**用 `python -m toolkits.realworld_check.test_franky_controller`（`NotImplementedError`），**不要**用 `step3_test_controller.py`（会自动 `home` 并张爪）。
5. 全程在 franky 容器 + `franky-0.19.0`。禁止宿主机 `.venv`、禁止 GPU 容器、禁止名为 `rlinf` 的那个容器。
6. 乱跑 / 砸桌 / 顶人：**拍急停**。急停之后**不要**立刻重跑 env，先 Desk 清 fault 再重新 Activate FCI。
7. 烟测脚本默认：`RLINF_SKIP_CAMERA=1`、`no_gripper=True`、`enable_random_reset=False`、`random_xy_range=0.0`、`safe_smoke_hold=True`（只跳过 `__init__` 的插值，**`reset()` 仍会动臂**）。不要加 `--unsafe-full-reset`。

#### 2.2 开跑前检查单

即 §4 阶段 0 全部内容。**最后一行 `--probe` 必须是 `RobotMode.Idle`。**

#### 2.3 进容器

见阶段 0。每个新开的 bash 都要重新 `source setup_before_ray_5090.sh`。

#### 2.4 运动链路体检（v2 新增门闩，**先过这一关**）

在把 env / Ray / wrapper 这些变量堆进来之前，先单独回答一个问题：**这台机器现在能不能被 franky 命令动起来？** 脚本直连 `franky.Robot`，不经 Ray、不经 `FrankaEnv`。

人站在急停旁，方块夹着，臂在标记上方几厘米：

```bash
ray stop                                     # 确保没有残留 Ray 占着 FCI
python b/x/scripts/diag_franky_motion.py \
    --test-hold --test-impedance --test-cartesian-motion --yes-move --dz 0.03
python b/x/scripts/diag_franky_motion.py --test-cartesian-motion --yes-move
```

三项测什么：

| 测试 | 做什么 | 期望 |
|------|--------|------|
| `--test-hold` | 建 tracker，目标 = **当前位姿**（零位移），3 s 内每 100 ms 看 `is_running` | 全程 `is_running=True`，`stop() clean`，`sag` 约 0 |
| `--test-impedance` | 同一 tracker，目标沿 z ramp `--dz` | `dz ≈ +0.03`，`is_running` 不变 False |
| `--test-cartesian-motion` | 阻塞 `robot.move(CartesianMotion)` | `dz ≈ +0.03`，`move()` 正常返回 |

**判读与分支：**

| 结果 | 结论 | 下一步 |
|------|------|--------|
| 三项 OK | 阻抗链路健康 | 直接做 2.6 / 2.7，`reset` 应该一次过 |
| `hold` 就死 | 1 kHz torque 链路本身不通（模式 / RT 调度 / 容器 / FCI 被抢） | 看 `stop() surfaced:` 那行的 libfranka 原因。`"User stopped"` → §1.4；其它 → §6 |
| `hold` 活但 `impedance` 不动 | 阻抗权限不足（刚度 / 截断 / 位形） | 提高 `RLINF_CART_K_T`、放宽 `RLINF_CART_ERR_CLIP_M`，或把臂挪到不那么伸展的位形重试。**先在这里调通，再去碰 env** |
| `impedance` 不动而 `CartesianMotion` 动 | 只有阻塞运动可用 | 按 `DualFrankaEnv` 分工改 reset（§7 待办 T1），`step` 仍用阻抗 |

**注意：** 若 `hold` 活但 `impedance` 只动了一点（例如 dz=+0.01），那是稳态误差，不是失败——阻抗本身有跟随滞后。门闩是「明确地动了」。

`--probe` 显示非 `Idle` 时脚本会直接跳过运动测试并 exit 1；`--force` 可强行执行，但只在明确知道自己在干什么时用。

#### 2.5 H1 标定（若标记/方块没变，已 PASS，可跳过）

语义：**夹爪闭合、方块已经贴住台面标记时**，末端 TCP 的 `[x, y, z, roll, pitch, yaw]`（米 + 欧拉 xyz）。空爪对准标记再夹方块会偏掉一整块厚度；张爪后再读作废。

```bash
export FRANKA_ROBOT_IP=172.16.0.2 FRANKA_GRIPPER_TYPE=franka
ray stop
python b/x/scripts/test_franky_controller_ext.py
# 或: bash b/x/scripts/run_cube_place_phase2.sh calibrate
```

应看到 `FrankyControllerExtended REPL` 与 `Connected to Franka at 172.16.0.2`，然后是 `cmd>`。若出现 `the libfranka backend for the original Franka Hand is not yet supported`，说明跑错了官方 toolkit。

`cmd>` 依次：

1. `open` → 按训练朝向把方块放入 → `close`。`close` 是约 **20 N** 轻力抓取（上限 40 N），夹住后维持这点力。异常立刻敲 `stop` 或拍急停。**夹紧后不要再换握姿。**
2. **按住臂上引导键**（此时 `robot_mode` 会变 `Guiding`，正常），移到标记正上方，缓慢下降到方块**轻轻贴住**标记。不要压垮垫子、不要把桌子顶起来。
3. 松开引导键，臂不再动。输入 `getpos_euler`，记下 6 个数。**再敲一次确认稳定。**
4. `q` 退出，释放 FCI。**不要**在这里敲 `home`。

写入文件：

```bash
python b/x/scripts/write_cube_place_pose.py <x> <y> <z> <roll> <pitch> <yaw>
# 现存值: 0.7062065 0.03620906 0.23192134 -3.11614319 0.02628124 0.17913087
```

写完再用引导键把臂抬到标记上方几厘米（仍夹着方块），作为 2.7 的起始姿态。

**门闩：** YAML 中 `calibrated: true`、六元组非全零、两次 `getpos_euler` 接近、标记与方块此后未被挪动。

#### 2.6 只读几何（`connect` / `2c`，不动臂）

```bash
bash b/x/scripts/run_cube_place_phase2.sh connect
# 等价: python b/x/scripts/step_cube_place_robot.py --connect-only
```

**期望输出：**

- `target_ee_pose (H1)` 与 YAML 一致
- `reset_ee_pose hover` 的 z = 接触 z + **0.08**（= 0.3119）
- `ee_pose_limit`：xy 半宽 0.05，z 下沿 接触 − 0.005，z 上沿 接触 + 0.08
- `robot_mode: RobotMode.Idle`
- `connect-only OK`，且**没有** `creating FrankyCubePlaceEnv-v1`

看 `probed - target xyz`：当前若已在标记上方悬停，xy 应较小、z 约 +0.05～0.10。**xy 差几十厘米就先用引导键挪近**，不要从远处 `reset`。

H1 未标定时脚本仍会探测 TCP，但 exit 1，且禁止进 2.7 / 2.8。

#### 2.7 `reset` 到悬停（`reset` / `2d`，会动臂）

前置：2.4 三项 OK、2.6 PASS、方块夹紧、`robot_mode=Idle`、人在急停旁。

```bash
ray stop
bash b/x/scripts/run_cube_place_phase2.sh reset
```

脚本流程：读 H1 → 子进程探测 TCP 与模式 → `require_motion_ready` → `gym.make` → `reset()`（§3.3 三段）→ 检查几何与夹爪 → `env.close()`。

**期望：**

- 日志含 `FrankyCubePlaceEnv-v1`；`wrapper stack` 含 **`GripperCloseEnv`**；动作维 6
- `Cartesian impedance tracker started (... is_running=True)`
- `cube_place go_to_rest: current [...] +z=0.100 -> [...]`
- `cube_place go_to_rest done: ... dz=<悬停 z − 起始 z>`。这个 `dz` 是**净变化**，不是 0.10：起始 z=0.2686 时应约 `+0.043`（落到 0.3119）。**只有 `dz≈0.0000` 才是失败**
- `gripper_open after reset: False`
- `hover check: |xy-target| ≤ 0.03m，|z-hover| ≤ 0.025m`
- 目视：臂升到标记正上方，**不**放下方块、**不**张爪
- `reset-only PASS`，exit 0

**立刻停手的信号：** xy 飞出盒子、z 往桌面砸、夹爪张开、FCI 被抢。

**若报 `cartesian impedance tracking stopped: ...`**：这是 v2 新增的护栏在说话，后面跟着 libfranka 真因和 `robot_mode`。按 §6 处理，**不要**去改几何。

#### 2.8 盒子内零动作 + 少量下探（`box` / `2e`，会动臂）

2.7 PASS、方块仍在、标记没动：

```bash
bash b/x/scripts/run_cube_place_phase2.sh box
# 等价: python b/x/scripts/step_cube_place_robot.py --num-steps 3 --approach-steps 3
```

行为：先做与 2.7 相同的 `reset`，再 3 步**零动作**，再 3 步小幅 **−z**（`action[2]=-0.4`，幅度被 `clip_z_range_low=0.005` 卡住）。

**期望：**

- 零动作时 TCP 几乎不动；reward 是稠密 xyz（悬停时通常**不是** 1.0）
- 下探时 z 下降或贴在盒顶/盒底；**不得**低于 接触 z − 0.015（脚本硬门 `z_floor`）
- 全程 `gripper_open` 为假；6D wrapper 不会发张爪
- `box-steps PASS`

**下探不要求真的碰到标记。** 阶段 2 的门闩是「闭爪 + 悬停几何 + 盒子下沿」，反复触达是阶段 3 的 SAC 的事。

#### 2.9 阶段 2 验收门闩

| ID | 项 | 通过标准 |
|----|----|----------|
| 2.4-1 | 运动链路 | `--test-hold` 全程 `is_running=True`；`--test-impedance` 明确移动 |
| 2.5-1 | H1 文件 | `calibrated: true`，六元组来自贴住标记时的 `getpos_euler` |
| 2.5-2 | H1 语义 | 读数时闭爪、方块在爪、贴住标记；无张爪后补读 |
| 2.6-1 | 只读 | hover = 接触 + 0.08，xy 盒 ±0.05，z 下沿 −0.005；不 `gym.make` |
| 2.7-1 | Gym | 真机 `gym.make` 为 `FrankyCubePlaceEnv-v1` |
| 2.7-2 | 闭爪 wrapper | 栈含 `GripperCloseEnv`；`action_space.shape == (6,)` |
| 2.7-3 | 悬停几何 | 目视在标记上方；xy 误差 ≤ 0.03，z 误差 ≤ 0.025 |
| 2.7-4 | 不张爪 | `gripper_open == False`，方块未掉 |
| 2.8-1 | 盒子 | 零动作稳定；下探不砸穿下沿；全程不张爪 |
| 2.8-2 | 退出 | 两个脚本 exit 0；结束后 FCI 可被 REPL 重新占用 |

失败就修 YAML / 重做 H1 / 查 FCI 与模式，过程记进 LOG。**不要**为了过门闩去加大 `clip_z_range_low`。

阶段 2 **不要**开 `train_async` / dummy SAC GPU，**不要** `ray start` 多节点（脚本内部 `ray.init` 足够）。

### 阶段 3 — 在线 SAC（尚未开始，缺件见 §7）

前置：阶段 2 全部 PASS；腕部相机 `wrist_1` 有图；ResNet10 权重可读。

人的动作序列（对应 v1 的 H2–H8）：

1. 写真机训练 YAML（**待创建**，§7 T2）：`target_ee_pose` 填 H1；`save_interval: 50`（官方 charger 是 `-1`，不要照抄）；`no_gripper` 不写或写 `True`。
2. 开训前：方块夹紧、臂在标记上方、REPL 已 `q`、Desk FCI 激活、**user-stop 松开**、`--probe` 为 `Idle`。
3. 启动后**第一段运动**应是抬到 / 移到目标上方悬停，再往标记靠近。若直冲桌外或扫地：急停，检查 `target_ee_pose` 是否填反、单位是否米。
4. 策略乱蹭时可推 SpaceMouse 把方块送到标记，纠正动作进 replay。鼠标**不能**张爪。
5. 成功后程序仍夹着方块抬走再悬停，**不用**每回合去桌上捡方块。
6. 掉块 → §6「掉块」行。
7. 看 TensorBoard：`env/reward`、`success_once`。

门闩：`reward` / `success_once` 上升；目视方块多次碰到标记。

### 阶段 4 — 验收

连续碰到标记；Gym ID 为 `FrankyCubePlaceEnv-v1`；成功时夹爪仍闭合。**不把「张爪放下」当验收项。**

---

## 5. 几何与超参建议

| 键 | charger | 本任务 | 理由 |
|----|---------|--------|------|
| `clip_x/y_range` | 0.02 | **0.05** | 对准标记，不必插孔级 |
| `clip_z_range_low` | 0.005 | **0.005** | 只允许比接触点低 5 mm，防砸桌 |
| `clip_z_range_high` | 0.05 | **0.08** | 悬停高度；成功区仍是接触点附近 1 cm |
| `random_xy_range` | 0.02 | **0.03**（烟测 0.0） | ≤ `clip_*` |
| `clip_rz` / `random_rz` | 0.35 | **同** | 方块 yaw 不敏感 |
| `action_scale` | `[0.02, 0.1, 1]` | **同** | 6D 闭爪 |
| `reward_threshold` xyz | 0.01 | **0.01～0.015** | 「碰到」容差 |
| `max_episode_steps` | 100 | **100** | |
| `gamma` / `target_entropy` | 0.8 / −4 | **同** | 6D → −action_dim |
| `state_dim` / `action_dim` / `image_num` | 19 / 6 / 1 | **同** | |
| `no_gripper` | 默认 True | **True** | 锁闭爪 |
| `save_interval` | −1 | **50** | 急停后可续 |

阻抗先原样复用 PegInsertion 的 `compliance_param`（K_t=2000）。接触力靠 z 下沿 + 笛卡尔阻抗，**不要**一上来加刚度。真要调，先在 2.4 的诊断脚本里调通再进 env。

---

## 6. 排障总表（按症状索引）

| 症状 | 处理 |
|------|------|
| **`reset` 打印 OK 但 `dz=0.0000`** | 先 `python b/x/scripts/diag_franky_motion.py --probe`。`UserStopped` → 松开手持设备 user-stop（这是 LOG-011…016 全部 `dz=0` 的唯一原因）。`Idle` 才继续查阻抗 |
| `ControlException: ... command not possible in the current mode ("User stopped")` | 同上。注意 `has_errors=False` 不代表没问题，要看 `robot_mode` |
| `cartesian impedance tracking stopped: <真因>` | v2 护栏。读真因：`"User stopped"` → 模式；`cartesian_reflex` / `joint_position_limits_violation` → 位形或碰撞，Desk 清 fault 后换起始姿态；`communication_constraints_violation` → 1 kHz 时序（容器 / RT 调度） |
| `robot_mode=Guiding` | 引导键还被按着，松开 |
| `robot_mode=Reflex` | Desk 清 fault，再 Activate FCI |
| 臂朝桌外 / 扫地 | 急停 → Desk 清 fault → **不要**直接重跑，先查 `target_ee_pose` 是否填反、单位是否米 |
| `Couldn't connect` / FCI 被抢 | 停掉所有 python；`ss -tn ... :1337` 应为空；REPL 必须 `q`；必要时 `ray stop` |
| `NotImplementedError ... original Franka Hand` | 跑了官方 toolkit。`ray stop` 后改用 `python b/x/scripts/test_franky_controller_ext.py` |
| `libfranka gripper: Command failed`（reset 时） | 已夹住又发了一次 `grasp`。当前 `FrankaLibfrankaGripper` 会打印 `skip grasp`；若仍报错，方块可能已掉，先用 REPL 轻力 `close` |
| `close` 把方块夹扁 | 现默认 20 N（上限 40 N）。若还在死夹：敲 `open` 或 `stop`，或拍急停 |
| 掉块 | **不要**在 env 还连着时 `open`。Ctrl+C 停 → REPL `open` → 捡起放入 → `close`（尽量复现 H1 握姿）→ 引导到标记上方 → `q`。标记被带跑则重做 2.5 |
| `'VideoPlayer' object has no attribute 'stop'`（`env.close()`） | 已知无害警告，烟测里被吞掉，不影响门闩 |
| `/dev/shm has only 67108864 bytes` | Ray 警告，非失败原因。`docker_run_franky_5090.sh` 目前没带 `--shm-size`；重建容器时可加（§7 T4） |
| `obs ... not within the observation space` / `dtype float64` | gymnasium passive checker 的警告，上游行为，忽略 |
| 脚本报 uncalibrated | 先做 2.5，**禁止**把全零当 target |
| 想「换个运动接口绕过去」 | 先读 `DualFrankaEnv._go_to_rest`：RLinf 自己的 franky env 用**阻塞 `reset_joint`** 做 reset，阻抗只用于 `step` 增量。不要凭感觉发明接口 |

---

## 7. 待办与已知缺口

| ID | 缺口 | 说明 |
|----|------|------|
| **T1** | reset 运动原语可能要换 | 取决于 2.4 的结果。若阻抗抬不动而阻塞 `CartesianMotion` 能动，按 `DualFrankaEnv` 分工：reset 走阻塞运动、`step` 走阻抗。**先有 2.4 数据再动手** |
| **T2** | 阶段 3 真机训练 YAML 不存在 | 需要 `b/x/configs/realworld_cube_place_sac_gpu.yaml`（或类似）+ 启动脚本。现有 `realworld_cube_place_dummy_sac*.yaml` 只是 dummy |
| T3 | `run_cube_place_phase2.sh` 没有 `diag` 子命令 | 2.4 目前得直接敲 `python b/x/scripts/diag_franky_motion.py ...`。可加 `diag` / `diag-probe` |
| T4 | franky 容器 shm 只有 64 MB | `docker_run_franky_5090.sh` 缺 `--shm-size`。阶段 2 无碍，阶段 3 建议加 |
| T5 | 阻抗在 x≈0.70 伸展位形下的实际权限未知 | 从未在 `Idle` 下验证过 10 cm 抬升。2.4 会给出答案 |
| T6 | `wrist_1` 相机在 cube-place 链路上未验证 | 烟测全程 `RLINF_SKIP_CAMERA=1`；阶段 3 前要确认 |

风险与缓解（沿用 v1 §10）：未标定就开训 → 2.5 门闩 + `calibrated` 检查；空爪/张爪后读 pose → 2.5 语义；抄 BinRelocation 张爪 reset → Env 走 PegInsertion 闭爪抬升；误设 `no_gripper: False` → 1A-2 检查 6D；砸桌 → `clip_z_range_low` 小 + 标定只贴住不死压。

---

## 8. 源码索引

| 主题 | 路径 |
|------|------|
| 模仿的训练配置 | [`examples/embodiment/config/realworld_charger_sac_cnn_async.yaml`](../../../examples/embodiment/config/realworld_charger_sac_cnn_async.yaml) |
| 模仿的 env 包 | [`examples/embodiment/config/env/realworld_peg_insertion.yaml`](../../../examples/embodiment/config/env/realworld_peg_insertion.yaml) |
| 几何 / 两段 reset / `compliance_param` | [`rlinf/envs/realworld/franka/tasks/peg_insertion_env.py`](../../../rlinf/envs/realworld/franka/tasks/peg_insertion_env.py) |
| 奖励 / `_interpolate_move` / `go_to_rest` 基类 | [`rlinf/envs/realworld/franka/franka_env.py`](../../../rlinf/envs/realworld/franka/franka_env.py) |
| **franky 后端控制器**（tracker、slew、参数常量） | [`rlinf/envs/realworld/franka/franky_controller.py`](../../../rlinf/envs/realworld/franka/franky_controller.py) |
| **franky env 的官方 reset 写法**（阻塞 `reset_joint`） | [`rlinf/envs/realworld/franka/dual_franka_env.py`](../../../rlinf/envs/realworld/franka/dual_franka_env.py) |
| franky 每步阻抗增量的官方写法 | [`rlinf/envs/realworld/franka/tasks/dual_franka_tcp_env.py`](../../../rlinf/envs/realworld/franka/tasks/dual_franka_tcp_env.py) |
| wrapper 默认闭爪 | [`rlinf/envs/realworld/common/wrappers/apply.py`](../../../rlinf/envs/realworld/common/wrappers/apply.py) |
| 官方 PnP（**不抄**） | [`realworld_pnp_rlpd_cnn_async.yaml`](../../../examples/embodiment/config/realworld_pnp_rlpd_cnn_async.yaml)、[`franka_bin_relocation.py`](../../../rlinf/envs/realworld/franka/tasks/franka_bin_relocation.py) |
| franky tracker 实现（容器内） | `/opt/venv/franky-0.19.0/lib/python3.11/site-packages/franky/tracker.py` |
| 本机扩展 | [`b/x/franky_ext/`](../../../b/x/franky_ext/)、[`b/x/scripts/`](../../../b/x/scripts/) |
| 标定与急停（人） | [`charger_sac_async.md`](charger_sac_async.md) §6.2.1、§15.5 |

---

**下一个动作：** §4 阶段 2.4——松开 user-stop，确认 `--probe` 为 `RobotMode.Idle`，跑三项运动体检。拿到结果再决定 2.7 是直接重跑还是走 T1。
