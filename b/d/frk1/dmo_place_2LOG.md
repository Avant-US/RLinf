# cube place 落地日志 v2

手册：[`dmo_place_2.md`](dmo_place_2.md)（v2 运行手册）
设计：[`dmo_place_1.md`](dmo_place_1.md)　前序记录：[`dmo_place_1LOG.md`](dmo_place_1LOG.md)（LOG-001…LOG-017）

代码只改 `b/x/`，不改 `rlinf/`。编号接 v1 日志，从 **LOG-018** 起。按时间顺序记录操作、命令、错误、根因与修复方案。

---

## 2026-08-19 — 阶段 2.4 / 2.7

### LOG-018 | 阶段 2.4 | 运动链路体检：hold + impedance PASS，CartesianMotion 假失败

**操作：** 用户松开 user-stop（LOG-017 根因），按手册 §2.4 一条命令串跑三项体检。

**命令（容器内，已 `source setup_before_ray_5090.sh`）：**
```bash
python b/x/scripts/diag_franky_motion.py \
    --test-hold --test-impedance --test-cartesian-motion --yes-move --dz 0.03
```

**起始状态（只读探测）：**

| 项 | 值 | 判定 |
|----|----|------|
| `robot_mode` | **`RobotMode.Idle`** | LOG-017 的 user-stop 已松开，总闸打开 |
| `has_errors` | `False` | 无锁存错误 |
| TCP | `[0.5545, -0.0001, 0.5204]` | **不是** H1 起始位形（H1 接触点 `[0.7062, 0.0362, 0.2319]`） |
| `q` | `[-0.0000, 0.0007, -0.0000, -1.5714, -0.0001, 1.5704, 0.0001]` | 近 home、**q2≈0** |
| `relative_dynamics_factor` | 0.2 | 脚本常量 |

**三项结果：**

| 测试 | 观测 | 判定 |
|------|------|------|
| `--test-hold` | 3 s 内每 100 ms `is_running=True`；`tracker.stop() clean`；`sag=-0.0012 m` | **PASS**。K_t=2000 下 1.2 mm 重力下沉正常。异步 torque motion 存活 |
| `--test-impedance` | 30 步 ramp（1 mm/100 ms），`is_running` 全程 True，末端 `dz=+0.0253`（目标 +0.030）；`stop() clean` | **PASS**。手册 §2.4 注已写明部分跟随 = 稳态滞后，不算失败 |
| `--test-cartesian-motion` | `ControlException: Move command aborted: motion aborted by reflex! ["cartesian_motion_generator_joint_velocity_discontinuity"]`；`dz=+0.0016`；事后 `has_errors=True` | **FAIL**，但见下：是假失败 |

`control_command_success_rate: 1` → 1 kHz 通信完好，不是 comms 问题。

**门闩判定（手册 2.4-1）：** 门闩只看 hold 存活 + impedance 明确移动 → **PASS**。

**顺带结论：T1 关闭。** 阻抗链路可用，reset 不需要改成阻塞 `CartesianMotion`，[`cube_place.py`](../../../b/x/franky_ext/tasks/cube_place.py) 的两段 reset 保持不动。

#### `CartesianMotion` 为什么挂（当时判为无需修，**事后看这个判断本身没错，但掩盖了另一件事**）

`cartesian_motion_generator_joint_velocity_discontinuity` = 笛卡尔**位置**运动发生器（内部做 IK 的那条链路）算出的关节速度，相对当前实测关节速度不连续，libfranka 第一个控制周期即抛 reflex。

本次同时具备两个触发条件：

| 因素 | 证据 | 说明 |
|------|------|------|
| **起始速度非零**（主因，充分） | 上一段 impedance ramp 结束在 `dz=+0.0254`，仍落后目标 4.6 mm；[`diag_franky_motion.py:185`](../../../b/x/scripts/diag_franky_motion.py#L185) 在 `tracker.stop()` 后仅隔几十毫秒就 `robot.move()` | 臂仍在运动/沉降，位置运动发生器要求起始关节速度连续 → 当场 reflex |
| **近奇异位形**（很可能的加重因素） | `q2≈0` → joint 1 与 joint 3 转轴共线（肩部/零空间退化） | IK 条件数极差，微小笛卡尔位移要求巨大关节速度跳变 |

**为什么阻抗不受影响：** `CartesianImpedanceTracker` 是力矩控制（雅可比转置），**不做 IK**，且 `translational_error_clip` + `max_delta_tau` 天然平滑指令。「impedance 过、CartesianMotion 挂」完全自洽。

**当时给出的处理（未改代码）：**
1. 清 latched error（Desk 清 fault + 重新 Activate FCI）。
2. 指出位形不对：测试在 `x=0.55, z=0.52` 近 home 位形做的，**T5（阻抗在 x≈0.70 伸展位形的实际权限）仍未回答**；要求把臂引导回「夹着方块、标记上方几厘米」再重跑 hold+impedance。
3. 提醒手册 §2.4 三项串跑有顺序副作用，`--test-cartesian-motion` 应单独跑。

#### 事后复盘：这一条的判读**漏了最关键的一项**

手册 §2.4 判读表写的是「三项 OK → 阻抗链路健康 → 直接做 2.6 / 2.7，`reset` 应该一次过」。我据此放行了 2.7。**这个推断是错的**，因为体检和 reset 跑在**完全不同的速度与幅度区间**：

| | 2.4 体检 | 2.7 `reset` 实际 |
|---|---|---|
| 指令速度 | 1 mm / 100 ms = **1 cm/s** | 1 cm / 100 ms = **10 cm/s** |
| 幅度 | 3 cm | 10 cm（抬升）+ 约 9 cm（落到 hover） |
| 位形 | x=0.55，近 home | x=0.707，伸展 |
| 负载 | 空载（方块状态未知） | 夹着方块，且压在标记上 |
| 实测滞后 | 4.7 mm | 未测 → 线性外推约 **4.7 cm** |

4.7 cm 已经吃满 `translational_error_clip = 0.05 m`。**即「体检 84% 跟随、看起来健康」与「reset 时阻抗全程力饱和」是同一组参数下的两个不同工况**，前者根本不能预测后者。这是 LOG-019 事故的直接前因。

---

### LOG-019 | 阶段 2.7 | `reset` 冲高 36 cm、超出最高指令值 26 cm | **FAIL（安全事故）**

**操作：** 用户在 LOG-018 之后执行 2.7。

**命令：**
```bash
bash b/x/scripts/run_cube_place_phase2.sh reset
```

**现象：** 机械臂**突然大幅度向上移动**；操作员按下手持设备 user-stop 制动；进程以 `RuntimeError` 退出。

#### 完整时间线（取自日志）

| 时刻 | 事件 |
|------|------|
| — | `probed tcp (current): [0.7074, 0.0445, 0.2254, -3.1143, 0.0671, 0.1845]` |
| — | `probed - target xyz (m): [0.0012, 0.0083, -0.0066]` |
| — | `robot_mode: RobotMode.Idle has_errors: False` → `require_motion_ready` 放行 |
| 02:29:27 | Ray 本地 cluster（`/dev/shm` 64 MB 警告，非失败原因） |
| 02:29:57 | `FrankaLibfrankaGripper connected (grasp_force=20.0N, holding=True width=0.0470m)` |
| 02:29:57 | `safe_smoke_hold: skip __init__ _interpolate_move` |
| 02:29:57 | `wrapper stack: OrderEnforcing -> PassiveEnvChecker -> Quat2EulerWrapper -> GripperCloseEnv -> FrankyCubePlaceEnv` |
| 02:29:58 | `Cartesian impedance tracker started (K_t=2000 K_r=150.0 tc=0.089 clip=[0.05, 0.05, 0.05] is_running=True)` |
| 02:29:58 | `cube_place go_to_rest: current [0.7074, 0.0445, 0.2254] +z=0.100 -> [0.7074, 0.0445, 0.3254]` |
| 02:30:01 | `ERROR ... cartesian impedance control thread died (mode=RobotMode.Other has_errors=True tcp=[0.6483, 0.0356, 0.5841])` |
| 02:30:01 | `RuntimeError: cartesian impedance tracking stopped: ControlException: libfranka: Move command aborted: User Stop pressed!; robot_mode=RobotMode.Other` |
| 02:30:01 | `Exiting main process due to a failure upon worker execution.` |

#### 关键量化

| 量 | 值 |
|----|----|
| 起始 z | `0.2254` |
| H1 接触 z | `0.23192134` → **起始比接触点低 6.6 mm**（方块压在标记上） |
| 抬升指令目标 z | `0.2254 + 0.10 = 0.3254` |
| hover 目标 z | `0.3119` |
| **全过程最高指令 z** | **`0.3254`** |
| 失败时实测 z | **`0.5841`** |
| 相对起始上升 | **+0.3587 m（35.9 cm）** |
| **超出最高指令值** | **+0.2587 m（25.9 cm）** |
| 超出安全盒上沿 `ee_pose_limit_max[2]=0.3119` | **+0.2722 m** |
| x 漂移 | `0.7074 → 0.6483`，**−5.9 cm** |
| 耗时 | 约 3 s → 平均垂直速度约 **12 cm/s** |

#### 三条与 LOG-017 的本质区别（不要混为一谈）

| | LOG-011…017 | **本次 LOG-019** |
|---|---|---|
| 症状 | 命令发了臂**不动**（`dz=0.0000`） | 臂动了，**远超命令值**（+26 cm） |
| `robot_mode`（开跑前） | `UserStopped` | **`Idle`** |
| user-stop | **原因**（被按住） | **结果**（操作员正确制动） |
| 结论 | 与代码无关 | **就是我们的参数与运动写法问题** |

**user-stop 是操作员的正确反应，不是故障原因。** `Move command aborted: User Stop pressed!` 与 `robot_mode=RobotMode.Other` 都是制动过程的表现。

#### 已按预期工作的部分（不要回改）

1. `require_motion_ready` 在 `gym.make` 前放行且判断正确（确实是 `Idle`）。
2. `FrankaLibfrankaGripper` 的 `skip grasp`：`holding=True width=0.0470m`，未重复 `grasp`（LOG-010 修复有效）。
3. `safe_smoke_hold` 只跳过 `__init__` 插值。
4. wrapper 栈 6D 闭爪正确。
5. **v2 §3.4 护栏生效**：[`controller_extended.py:93`](../../../b/x/franky_ext/controller_extended.py#L93) 把死掉的 tracker 变成带真因 + `robot_mode` 的 `RuntimeError`，进程立刻退出，没有继续往下跑 box。哑失败已经彻底消失。

#### 根因分析

代码路径（自上而下）：

```
FrankyCubePlaceEnv.go_to_rest              b/x/franky_ext/tasks/cube_place.py:40
  ├─ _end_effector_action([-1.0])          → skip grasp（正常）
  ├─ _move_action(current TCP)             → 建 tracker，K_t=2000 K_r=150 clip=0.05
  ├─ _interpolate_move(current+0.10, timeout=1)
  │    └─ FrankaEnv._interpolate_move      rlinf/.../franka_env.py:854
  │         linspace(measured_once → target, 11)，for 循环 + time.sleep(0.1)
  │           └─ _move_action              rlinf/.../franka_env.py:871
  │                ├─ _clear_error()       → controller.clear_errors() → robot.recover_from_errors()
  │                └─ controller.move_arm  → move_tcp_pose → cart_tracker.set_target
  └─ FrankaEnv.go_to_rest                  rlinf/.../franka_env.py:504
       while not np.allclose(tcp, reset_pose, 0.02): _interpolate_move(reset_pose)  ×≤3
```

**R1 — 速度区间失配（主因，已量化）**

- `step_frequency = 10.0`（[`franka_env.py:62`](../../../rlinf/envs/realworld/franka/franka_env.py#L62)）。
- `_interpolate_move(reset_pose, timeout=1)` → `num_steps = 1 × 10 = 10` → 10 cm 分 10 个路点、每 100 ms 一个 = **指令速度 10 cm/s**。
- 2.4 体检验证过的是 **1 cm/s**，实测滞后 4.7 mm。线性外推到 10 cm/s → 滞后约 **4.7 cm**。
- `translational_error_clip = 0.05 m`（[`franky_controller.py:58`](../../../rlinf/envs/realworld/franka/franky_controller.py#L58)），`K_t = 2000 N/m` → **力上限 = 2000 × 0.05 = 100 N**。
- 即：整个抬升过程中，误差基本吃满 clip，阻抗**全程输出接近 100 N 向上的力**。这是可以把臂甩上去的量级，也解释了 12 cm/s 的实际速度和之后的巨大超调（弹簧储能 ≈ 100 N × 0.1 m ≈ 10 J）。

**R2 — `K_t` 抬到 2000 时 `error_clip` 没跟着收（力上限被无声放大 4 倍）**

- franky 默认 `K_t = 500`（[`franky_controller.py:52`](../../../rlinf/envs/realworld/franka/franky_controller.py#L52)）→ 力上限 500 × 0.05 = **25 N**。
- `reset()` 调 `reconfigure_compliance_params`，把 PegInsertion 的 `compliance_param.translational_stiffness = 2000` 灌进去（[`controller_extended.py:121`](../../../b/x/franky_ext/controller_extended.py#L121)），但 `_ensure_cart_tracking_motion` 仍取 `fc._CART_TRANS_ERROR_CLIP_M = 0.05`（[`controller_extended.py:156`](../../../b/x/franky_ext/controller_extended.py#L156)）→ 力上限变 **100 N**。
- 旋转同理：`K_r` 40 → 150，`rot_clip = 0.3 rad` → 力矩上限 12 → **45 N·m**（j5–j7 的关节限值只有约 12 N·m 量级）。
- **这两个「4 倍」从来没人明确决定过**，是两处独立取默认值相乘的结果。

**R3 — `_interpolate_move` 是纯开环定时，没有任何收敛/超调检查**

- `linspace` 的起点是**开始前采样一次**的位姿；之后只按 `time.sleep` 发路点，从不回读「臂到了吗」。
- 臂落后 → 目标继续前进 → 误差增大 → 力饱和；臂冲过 → 也没有任何一处会中止。
- `_interpolate_move` **不**按 `ee_pose_limit` 裁剪（这是手册 §3.3 明确记录的上游行为，裁剪只发生在 `step()`）→ 冲到盒顶上方 27 cm 全程无人叫停。
- `FrankaEnv.go_to_rest:525` 的 `np.allclose(a, b, 0.02)` 把 `0.02` 传给的是 **rtol** 而非 atol（atol 默认 1e-8），所以那是「相对 2%」（z=0.3119 时约 6 mm）的松容差，且最多重试 3 次；它是收敛**判据**，不是运行中的安全监控。

**R4 — `translational_damping → gains_time_constant` 的映射是错的（待测其影响）**

[`controller_extended.py:130-133`](../../../b/x/franky_ext/controller_extended.py#L130)：
```python
if trans_d is not None and trans_k > 0:
    tc = max(0.01, 2.0 * float(trans_d) / trans_k)     # 89 → 2*89/2000 = 0.089
```
- charger 的 `compliance_param.translational_damping = 89` 是 **ROS `cartesian_impedance_controller`** 的阻尼系数（N·s/m）。
- franky 的 `gains_time_constant` 是**增益斜坡滤波时间常数（秒）**，与阻尼不是同一个量。
- `2·89/2000 = 0.089 s` 恰好接近默认 `0.1 s`，所以数值上无害；但**这与 LOG-016 已经撤回过两次的 `translational_clip_*` 是同一类错误**（把 ROS 键硬映射到 franky 参数）。真正的阻尼是否由 franky 内部按刚度推导、K_t=2000 时是否仍临界阻尼，**未读源码确认** → 是超调幅度的候选解释之一，必须实测。

**R5 — LOG-013 的假说必须重新打开（其「证伪」是无效的）**

- `_move_action` 每个路点都调 `_clear_error()` → `clear_errors()` → `robot.recover_from_errors()`（[`franky_controller.py:225`](../../../rlinf/envs/realworld/franka/franky_controller.py#L225)），即**在运行中的 torque motion 里以 10 Hz 反复调用 `automaticErrorRecovery`**。
- LOG-013 提出「recover 会掐死 tracker」，LOG-014 声称「去掉后仍 dz=0 → 证伪」。但 **LOG-014 是在 `UserStopped` 下跑的（LOG-017）**，那次测试根本不可能动 → **该证伪无效，假说重新有效**。
- 需要在 `Idle` 下单独测：路点间 recover vs 不 recover，对 tracker 存活与跟随的影响。

**R6 — 操作侧：起始位形违反手册 §2.1 规则 3**

- 手册要求「方块夹紧；用引导键把臂放到**标记上方几厘米**」。
- 实际起始 z = 0.2254，**比 H1 接触点低 6.6 mm** → 方块是**压在标记上**开跑的，臂带着接触载荷。
- 这不是超调的主因，但它使抬升幅度、初始受力、以及「臂在盒底」这些条件都偏离设计，且我在 LOG-018 里明确要求过先引导到上方而未被执行/未被核对。
- 另外 x≈0.707 是伸展位形，`nullspace_stiffness = 5 Nm/rad` 极软 → 肘部几乎自由，这解释了 **z +35.9 cm 同时 x −5.9 cm 的圆弧轨迹**：那不是干净的笛卡尔 z 运动，是臂被大力向上「甩」出的摆臂。

#### 现场恢复步骤（安全优先，先做这个再谈重跑）

```bash
# 0. 保持 user-stop 按下。目视：方块在爪里还是掉了？周围有无被挤压物？标记有没有被带走？
# 1. 确认软件侧已完全断开（日志已 Exiting main process）
ray stop
ss -tn state established '( dport = :1337 or sport = :1337 )'   # 必须为空
# 2. Desk http://172.16.0.2/desk：查 safety violation / errors → 清 fault
# 3. 再松开 user-stop（按钮抬起）。无运动指令臂不会自己动
# 4. Desk：Unlock 关节（若被锁）→ 重新 Activate FCI
# 5. 只读确认
python b/x/scripts/diag_franky_motion.py --probe
#    期望 robot_mode=RobotMode.Idle、has_errors=False；记录当前 TCP 与 q（臂现在很高，z≈0.58）
# 6. 引导回起始位形（方块掉了就重夹）
python b/x/scripts/test_franky_controller_ext.py
#    cmd> open → 放入方块 → close        （20 N）
#    按住引导键把臂带到标记上方 3–5 cm
#    cmd> getpos_euler
#         门闩: z 必须在 0.262~0.282 之间，即**高于** H1 的 0.2319，绝不能再低于它
#    cmd> q
```

**第 6 步的 z 核对是新增硬门闩** —— 本次事故的起始条件（z 低于接触点 6.6 mm）本应在这里被拦住。

#### 重跑前必须先做的测量（不改代码，只用现有脚本 + 环境变量）

**M1 — 降力上限。** 唯一不改代码就能收紧的旋钮是 error clip（`K_t` 被 `compliance_param` 强制成 2000，`RLINF_CART_K_T` 对 reset 无效）：

```bash
export RLINF_CART_ERR_CLIP_M=0.01      # 2000 × 0.01 = 20 N（原 100 N）
```
**自校验：** tracker 启动日志会打印 `clip=[0.01, 0.01, 0.01]`。若仍是 `0.05`，说明环境变量没进 Ray actor，**不要继续**。

**M2 — 在真实起始位形上补测 T5，按速度/幅度阶梯escalate。** 这是 LOG-018 漏掉的那一步：

```bash
python b/x/scripts/diag_franky_motion.py --probe                                   # Idle
python b/x/scripts/diag_franky_motion.py --test-hold      --yes-move --seconds 3
python b/x/scripts/diag_franky_motion.py --test-impedance --yes-move --dz 0.03 --seconds 3   # 1   cm/s 基线
python b/x/scripts/diag_franky_motion.py --test-impedance --yes-move --dz 0.03 --seconds 1   # 3   cm/s
python b/x/scripts/diag_franky_motion.py --test-impedance --yes-move --dz 0.10 --seconds 5   # 2   cm/s，10 cm 幅度
```
每一档之间回读 `--probe`，臂只准向上（远离桌面），人手在 user-stop 上但**不按**。

**判读：** 记录每档 `dz_measured / dz_commanded` 与是否出现 `dz > dz_commanded`。若任一档 `dz` 超过指令值 → 超调已复现，`RLINF_CART_ERR_CLIP_M` 继续降到 0.005。

**M2 的已知盲区：** 见下表 T7 —— 现在的脚本 ramp 跑完**立刻** `tracker.stop()`，**看不到 ramp 结束后的超调峰值**，而这恰恰是本次事故的关键量。所以 M2 只能测「跟随」，测不到「冲高」。这是必须补的脚本能力。

**M3 — 重开 LOG-013 假说（R5）。** 需要脚本支持「路点间是否 recover」的对照，属 T8。

#### 提议的增删改（**尚未执行**，等 M1/M2 数据）

| 文件 | 改动 | 为什么 |
|------|------|--------|
| [`b/x/scripts/diag_franky_motion.py`](../../../b/x/scripts/diag_franky_motion.py) | **新增** ramp 后观测窗（`--settle-seconds`，不立刻 `stop()`），记录 `max(dz)` 与 `argmax` 时刻；新增按 cm/s 指定速度的 `--ramp-speed`；每步打印 `目标−实测` 误差与 `O_F_ext_hat_K` | 现在 ramp 结束就 `stop()`，**超调峰值测不出来**。事故的核心量诊断脚本盲区 → 这是最高优先级的一处改动 |
| [`b/x/scripts/diag_franky_motion.py`](../../../b/x/scripts/diag_franky_motion.py) | **新增** `--max-overshoot` 硬门：实测超过指令值该值即立刻 `stop()` 并 exit 1 | 诊断脚本自己也要有超调看门狗，不能靠人眼 |
| [`b/x/franky_ext/controller_extended.py`](../../../b/x/franky_ext/controller_extended.py) | `_ensure_cart_tracking_motion`：把 `translational_error_clip` 与 `translational_stiffness` **绑定**，令 `K_t × clip ≤ F_max`（建议 25 N，与 franky 默认 500×0.05 一致）；旋转同理 `K_r × rot_clip ≤ 12 N·m` | R2。K_t 从 500 抬到 2000 时力上限被无声放大 4 倍（100 N / 45 N·m），无人决定过 |
| [`b/x/franky_ext/controller_extended.py`](../../../b/x/franky_ext/controller_extended.py) | `reconfigure_compliance_params`：**删除** `tc = 2·trans_d/trans_k` 映射，改为忽略 `translational_damping` 并 `log_warning` 说明理由 | R4。与 LOG-016 已撤回两次的 `translational_clip_*` 同类错误：ROS 键硬映射到 franky 参数 |
| [`b/x/franky_ext/franky_single_franka_env.py`](../../../b/x/franky_ext/franky_single_franka_env.py) | **覆写** `_move_action`：每个路点后回读 live TCP，若 `\|live − target\| > error_clip` 或 live 超出 `ee_pose_limit_max` + 余量（如 3 cm）→ 立刻 `_stop_cart_tracking_motion()` 并抛错 | R3。上游 `_interpolate_move` 纯开环、无收敛检查、不按盒子裁剪；这次就是开环把目标推满、力饱和、无人叫停 |
| [`b/x/franky_ext/tasks/cube_place.py`](../../../b/x/franky_ext/tasks/cube_place.py) | `go_to_rest`：`_interpolate_move(reset_pose, timeout=1)` → 按位移算时间（`timeout = max(1.5, dz / v_max)`，`v_max ≈ 0.02 m/s`）；`reset_z_lift_m` 0.10 → **0.02~0.03** | R1。10 cm/10 步/10 Hz = 10 cm/s，是 2.4 验证过速度的 10 倍。charger 抬 10 cm 是为了**拔插头**；本任务只需把方块抬离标记，2–3 cm 足够 |
| [`b/x/scripts/step_cube_place_robot.py`](../../../b/x/scripts/step_cube_place_robot.py) | 新增 `--reset-z-lift` / `--interp-speed` / `--cart-err-clip` 三个显式旋钮，写进 `_build_override_cfg`；新增起始 z **必须高于** `target_z` 的前置门闩 | 现在这三个量只能改代码或靠 `RLINF_CART_*` 隐式生效，烟测无法先在低权限下试；起始位形也没有任何检查（R6） |
| [`b/d/frk1/dmo_place_2.md`](dmo_place_2.md) | §2.4 判读表删掉「三项 OK → reset 应该一次过」，改为「体检只覆盖 1 cm/s / 3 cm，必须按速度幅度阶梯逐档验证到 reset 实际区间」；§3.3 标注 `_interpolate_move` 为**开环无收敛检查、不按盒子裁剪**；§5 新增 `K_t × clip = 力上限` 一行；§6 新增「臂冲高 / 超出指令值」症状行；§2.1 新增起始 z 必须高于接触点的规则；§7 更新 T1（关闭）/T5（部分）、新增 T7/T8 | 手册当前的推断链被本次事故否掉了，必须先改手册再重跑，否则下一个人会重犯 |

#### 待办表更新

| ID | 状态 | 说明 |
|----|------|------|
| T1（reset 运动原语要不要换） | **关闭** | LOG-018：阻抗 hold+ramp 均 OK，不需要换成阻塞 `CartesianMotion`。而且 `CartesianMotion` 在近奇异位形 + 非零起始速度下反而会 reflex |
| T5（阻抗在 x≈0.70 的权限） | **部分，仍未回答** | LOG-018 在 x=0.55 测的，不算。LOG-019 反而给出了另一个答案：在 x≈0.707、K_t=2000、clip=0.05 下权限**过大**（100 N），不是不足 |
| **T7（新）** | 未做 | `diag_franky_motion.py` 测不到 ramp 结束后的超调峰值（ramp 完即 `stop()`）。事故核心量存在诊断盲区 |
| **T8（新）** | 未做 | LOG-013 假说（路点间 `recover_from_errors()` 干扰 torque motion）的「证伪」因 LOG-014 在 `UserStopped` 下进行而无效，需在 `Idle` 下重测 |
| T2/T3/T4/T6 | 不变 | 见 [`dmo_place_2.md`](dmo_place_2.md) §7 |

#### 教训

1. **「体检 PASS」只对体检覆盖的工况有效。** 1 cm/s / 3 cm 通过，不能推出 10 cm/s / 10 cm 也通过；而手册的判读表恰好写了这个推断。速度和幅度必须逐档 escalate 到实际使用区间。
2. **刚度和误差截断必须一起看。** 两个独立的「默认值」相乘出来的力上限（100 N）从来没人决定过。凡是改 `K_t`，就必须同时检查 `K_t × clip`。
3. **开环插值 + 高力上限 = 没有刹车的加速踏板。** 上游 `_interpolate_move` 只按时钟发目标、不看臂在哪、也不按安全盒裁剪，这在 ROS 常驻控制器上问题不大，在 franky 高刚度阻抗上就是超程。
4. **v2 的哑失败护栏是有效的，但它只在事后报错。** 它把「静默不动」变成了「明确报错」，却拦不住「动过头」。需要的是运行中的超调看门狗（上表 mixin 那条）。
5. **起始位形没有机器检查就等于没有要求。** LOG-018 明确写了「引导到标记上方几厘米」，实际起跑点比接触点还低 6.6 mm，脚本一声不响地放行了。

---

**当前状态（截至 LOG-019）**

| 项 | 状态 |
|----|------|
| 阶段 2.4 hold / impedance | PASS（但仅覆盖 1 cm/s、3 cm、x=0.55） |
| 阶段 2.4 CartesianMotion | FAIL（假失败，且 T1 已关闭，不需要它） |
| 阶段 2.7 `reset` | **FAIL —— 冲高 36 cm，超最高指令值 26 cm，操作员 user-stop 制动** |
| 机器人 | user-stop 按下、`has_errors=True`、TCP 约 `[0.6483, 0.0356, 0.5841]`，**需按上文「现场恢复步骤」处理** |
| 代码 | **未改动**（本轮只读代码 + 分析） |

**下一个动作：** 现场恢复（含起始 z 高于接触点的核对）→ `export RLINF_CART_ERR_CLIP_M=0.01` 并从 tracker 日志确认生效 → M2 速度/幅度阶梯测量 → 拿到超调数据后再决定上表哪几条改动落地。**在 M1/M2 完成之前不要重跑 `reset`。**
