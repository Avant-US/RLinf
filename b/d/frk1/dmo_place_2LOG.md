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

**R1 — 速度区间失配（事实，但**不是**已证实的机理，见下方 LOG-021 更正）**

- `step_frequency = 10.0`（[`franka_env.py:62`](../../../rlinf/envs/realworld/franka/franka_env.py#L62)）。
- `_interpolate_move(reset_pose, timeout=1)` → `num_steps = 1 × 10 = 10` → 10 cm 分 10 个路点、每 100 ms 一个 = **指令速度 10 cm/s**。
- 2.4 体检验证过的是 **1 cm/s**，实测滞后 4.7 mm。
- `translational_error_clip = 0.05 m`（[`franky_controller.py:58`](../../../rlinf/envs/realworld/franka/franky_controller.py#L58)），`K_t = 2000 N/m` → **逐轴弹簧力上限 = 2000 × 0.05 = 100 N**。

> ⚠️ **本条当时的推论「线性外推滞后 4.7 cm → 吃满 clip → 全程输出 100 N」在 LOG-021 中被证伪。** 滞后并不随速度线性增长（摩擦项占主导且与速度无关），10 cm/s 时实际滞后约 11.7 mm，远未饱和。100 N 是**可用权限**，不是**实际输出**。26 cm 超调的真实机理仍未确定。详见 LOG-021 §21.1。

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

> **LOG-021 之后，这一段有三处行为变了**（本条按当时状态保留，执行时按下面这版）：
>
> 1. **第 6 步的 `open` 在持物时需要写 `open yes`。** 若方块还在爪里，裸 `open` 会被拒并提示「会掉」。这正是本次恢复的场景。
> 2. **第 6 步的 `close` 会按测量宽度复核。** 现在按标定宽度 `FRANKA_CUBE_WIDTH_M`（默认 0.046 ± 0.012 m）抓取，抓完不符就**抛错**并告知设该环境变量。空爪或换了尺寸的方块会在这里被挡住，而不是像以前那样报成功。
> 3. **第 5 步的 `--probe` 输出变多了**：除 `robot_mode` / `has_errors` 外还有夹爪 `width`/`holding`、`|F_ext|`、`|dq|`、`cmd_success_rate`，以及一行 `authority:`——应为 `<= 20.0N/axis`。REPL 另有 `mode` / `health` 命令可随时查模式，不必退出。
>
> 另外**第 6 步现在不是可选的**：臂停在 z≈0.584，而围栏顶是 0.392、起始位形硬门上限是 `盒顶+0.01 = 0.322`，从当前位置跑 `reset` 会被这两道门先后拒绝，臂根本不会动。
>
> 若需要重建 franky 容器：`docker_run_franky_5090.sh` 现在会先检查 1337 是否已被占用，被占时直接拒绝启动（防第二个容器抢 FCI）。

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

> **后续修正（见 LOG-020）：** 上面这条「先测量再改」的次序被用户改为「先把护栏落地，再测量」。这是对的——事故后的第一轮测量本身就需要一个能刹车的诊断脚本，而当时的脚本连超调都测不到。M1 里的 `RLINF_CART_ERR_CLIP_M=0.01` 也已被更好的办法取代（力上限恒定，clip 由刚度反推），该环境变量现在不再被使用。

---

### LOG-020 | 落地全部护栏 + 代码风险普查 | 1A PASS，真机待测

**用户指令：** 按 LOG-019 的优先级顺序开始改；顺便普查代码中其它会导致风险的地方一并改；改动与理由记入本日志；涉及手册的一并改。

**本轮**未接触真机**：机器人仍处于 LOG-019 之后的状态（user-stop 按下、`has_errors=True`、TCP 约 `[0.6483, 0.0356, 0.5841]`）。所有验证都在 dummy 路径（`is_dummy: True`，不连 FCI）完成。

#### 20.1 设计决定：把「危险的量是乘积」变成代码里的不变量

事故的结构性原因不是某一个数字错了，而是**两个各自合理的默认值相乘**：

| 危险量 | = | 因子 A（来自配置） | × | 因子 B（来自库默认） |
|---|---|---|---|---|
| 指令力 100 N | | `compliance_param.translational_stiffness = 2000` | | `_CART_TRANS_ERROR_CLIP_M = 0.05` |
| 指令速度 10 cm/s | | 位移 0.10 m | | `_interpolate_move(timeout=1)` 硬编码 |

所以修法不是「把 clip 调小」（那只是把同一个陷阱挪了个位置），而是**反转谁是自由变量**：

- 恒定的是**力**（25 N），`clip = 力上限 ÷ 刚度` 由它反推 → 改刚度不再改力。
- 恒定的是**速度**（2 cm/s），`时长 = 位移 ÷ 速度` 由它反推 → 改位移不再改速度。

这两条加上一条**运行时围栏**（测量值越界就刹车），构成本轮的全部主线。新建 [`b/x/franky_ext/motion_limits.py`](../../../b/x/franky_ext/motion_limits.py) 作为唯一的权限来源，纯算术、无硬件依赖，因此 1A dummy 能覆盖它。

#### 20.2 改动清单（11 个文件，+1598 / −182）

```
新增  b/x/franky_ext/motion_limits.py
改    b/x/franky_ext/controller_extended.py       +328
改    b/x/franky_ext/franky_single_franka_env.py  +137
改    b/x/franky_ext/tasks/cube_place.py          + 76
改    b/x/franky_ext/tcp_probe.py                 +105
改    b/x/scripts/diag_franky_motion.py           +677（基本重写）
改    b/x/scripts/step_cube_place_robot.py        +183
改    b/x/scripts/step_cube_place_dummy.py        +103
改    b/x/scripts/test_franky_controller_ext.py   + 59
改    b/x/scripts/run_cube_place_phase2.sh        + 33
改    b/d/frk1/dmo_place_2.md                     （手册，见 20.6）
```

**`rlinf/` 一行未改。** 所有对上游行为的修正都通过 `b/x/` 的 mixin / 子类覆写完成。

##### F1 — 新增 `motion_limits.py`：权限集中化

| 量 | 默认 | 环境变量 | 硬范围 |
|---|---|---|---|
| 指令力上限 | **25 N** | `RLINF_CUBE_FORCE_CEILING_N` | [5, 60] |
| 指令力矩上限 | **10 N·m** | `RLINF_CUBE_TORQUE_CEILING_NM` | [2, 25] |
| 插值速度上限 | **2 cm/s** | `RLINF_CUBE_INTERP_SPEED` | [0.2, 6] cm/s |
| 插值位移上限 | **0.35 m**（超过→拒绝） | — | — |
| 围栏余量 | **0.05 m** | `RLINF_CUBE_GUARD_MARGIN` | [0.01, 0.15] |
| 围栏跟随上限 | **0.05 m** | `RLINF_CUBE_GUARD_MAX_LAG` | [0.01, 0.20] |
| 刚度硬夹 | `K_t∈[50,3000]`、`K_r∈[5,300]` | — | — |

关键函数：`error_clips_for_stiffness`（力恒定）、`interp_duration_s`（速度恒定）、`clamp_stiffness`、`describe_authority`（把**乘积**打进日志，而不只是因子）。

**为什么每个环境变量都被夹在硬范围内：** 事故的直接触发条件之一就是一个可配置量取了默认值。允许覆盖但不允许覆盖到危险值，是让「on-robot 调参」和「不能重演 100 N」同时成立的唯一办法。畸形值（写错格式）被忽略而不是抛异常——这段代码跑在 Ray actor 里，import 期的 traceback 很难归因。

**25 N 是否够用（量化，不是猜）：** LOG-018 实测 1 cm/s 下滞后 4.7 mm，即实际只用了 `2000 × 0.0047 ≈ 9.4 N`；hold 测试的 1.2 mm 下沉对应约 2.4 N。2 cm/s 下滞后线性外推约 9.4 mm，仍小于 `25/2000 = 12.5 mm` 的 clip → **不饱和，仍有余量**。若真「抬不动」，`--force-ceiling 35` 在 2.4b 里验，不改代码。

##### F2 — `controller_extended.py`

| 改动 | 为什么 |
|---|---|
| `_ensure_cart_tracking_motion` 用 `error_clips_for_stiffness` 推 clip | **R2 根治**。K_t=2000 → clip=0.0125 → 力仍 25 N |
| `reconfigure_compliance_params` **删除** `tc = 2·trans_d/trans_k` | **R4**。ROS 的 `translational_damping`(N·s/m) 与 franky 的 `gains_time_constant`(s) 不是同一个量；`2·89/2000=0.089` 恰好落在默认 0.1 附近，所以「看起来合理」而毫无意义。现在改为**忽略并 warn**，并把所有 ROS-only 键（`*_damping`、`translational_clip_*`、`rotational_clip_*`、`Ki`）一起列进警告 |
| `reconfigure_compliance_params` 调 `clamp_stiffness` | REPL 手抖 `impedance 99999` 或 YAML 打错也建不出怪物 tracker |
| 新增 `set_motion_guard` / `_check_motion_guard` | **R3 的运行时补救**。围栏比对**测量** TCP 与 `ee_pose_limit ± margin`（`+z` 另加 `reset_z_lift_m`，容许 `go_to_rest` 那一次合法越顶），以及 `\|测量−指令\|` |
| 围栏放在 `move_arm` 里、**控制器进程内** | 这里有 `self._robot` 直连，**没有 Ray 往返**，所以每个路点都能零成本检查并立即刹车。放在 env 侧要多两次 RPC，延迟正好落在最不该延迟的地方 |
| 新增 `freeze_at_current`，刹车顺序 = **先 `set_target(测量位姿)` 再 `stop()`** | 把目标设到臂**当前所在**处等于消掉位置误差 → 阻抗主动减速。直接 `stop()` 会把带着动量的臂交给受控停止。顺序是有意的 |
| 新增 `motion_health()` / `gripper_holding()` | 让 Ray 另一侧的调用者能廉价读到模式/围栏/夹爪状态，用于烟测打印与开跑前拦截 |
| 围栏 z 上沿 = `盒顶 + 0.05 + reset_z_lift_m` | 算术：接触 0.23192 → 盒顶 0.31192 → 围栏顶 **0.39192**。同样的事故条件下会在 0.392 m 触发，比人快 **19 cm** |

##### F3 — `franky_single_franka_env.py`（mixin）

| 改动 | 为什么 |
|---|---|
| `_interpolate_move` 用 `interp_duration_s` 推时长 | **R1 根治**。上游把 `timeout` 硬编码为 1.0/1.5 s 而不看位移，等于把速度当自由变量。调用方给的 `timeout` 只被当作**下限**，永远不能让运动变快 |
| `_interpolate_move` 位移 > **0.35 m** 直接 `raise` | 我自己查出的漏洞：`INTERP_DURATION_S_RANGE` 上限 20 s 意味着位移超过 `20×0.02 = 0.4 m` 时速度又会突破上限。与其把时长上限调大，不如**拒绝**——0.35 m 已远超本任务安全盒，且 RLinf 自己的 franky env（`DualFrankaEnv._go_to_rest`）对大位移用阻塞 `reset_joint`，从不用阻抗。1A 现在断言 `0.35 ≤ 20 × 0.02` 这个不变量 |
| `_setup_hardware` 末尾 `arm_motion_guard()` | **我自己查出的潜伏风险**：`FrankaEnv.__init__` 第 212 行会自己跑一次 `_interpolate_move(self._reset_pose)`。烟测靠 `safe_smoke_hold=True` 跳过它，但**阶段 3 训练时该字段默认 False**，于是训练启动后的**第一个动作**就是一次从任意位姿出发、无限速、无围栏的插值。现在它同时被限速、被围栏、被位移上限约束 |
| `arm_motion_guard` 在 `ee_pose_limit` 全零时拒绝装并 warn | 装一个 `[0,0,0]` 的围栏比不装更糟（任何真实位姿都在界外，第一个路点就误报） |
| 新增 `_move_action` 覆写 + `clear_error_per_waypoint` 旋钮 | **R5，但刻意不改默认值。** LOG-013 提出「路点间 `recover_from_errors()` 干扰 torque motion」，LOG-014 声称证伪——但 LOG-014 是在 `UserStopped` 下跑的，那次**任何**运动都不可能发生，所以**该证伪无效，假说重新有效**。默认保持上游行为，旋钮 + `diag --test-recover-loop` 留给测量（T8）。在没有数据的事情上改默认值，正是 LOG-011…016 那七轮的错误模式 |

##### F4 — `tasks/cube_place.py`

| 改动 | 为什么 |
|---|---|
| `reset_z_lift_m` **0.10 → 0.03** | charger 抬 10 cm 是为了把插头**拔出插座**；本任务只需让方块脱离一个平面标记。0.10 还把抬升目标顶到盒顶上方 10 cm，超出任何几何授权 |
| `go_to_rest` 的抬升时长由 `interp_duration_s` 算 | 让日志里能直接看到 `over 1.50s`，而不是一个隐含的 10 cm/s |
| 新增 `_check_start_pose()`，越界 **warn** | R6。低于盒底 = 方块压在标记上（事故起始条件）；高于盒顶 / xy 出盒也各有警告。这里用 warn 而非 refuse：训练中一次成功触达之后沉降几毫米到盒底之下是正常的，硬拦要放在有人在场的烟测脚本里 |
| `clear_error_per_waypoint: bool = True` 字段 | 同 F3 |

##### F5 — `tcp_probe.py`

| 改动 | 为什么 |
|---|---|
| 探测子进程additionally返回 `q`、`cmd_success_rate`、`joint_vel_norm`、**夹爪 `width` / `holding`** | 让「空爪跑闭爪任务」和「臂还在动」能在 **Ray 启动前**被发现。夹爪是独立的 libfranka 连接，只读，连不上不算致命 |
| 新增 `check_start_pose()` | R6 的硬门实现。纯算术 → 1A 覆盖。返回**给操作员的指令**（「用引导键把臂挪到标记上方几厘米」），不是一句 `AssertionError` |
| 起始 z 上限只给 **1 cm** 越顶余量 | 这个数字不是随手取的：围栏顶 = `盒顶 + margin + lift`，而 `go_to_rest` 是从**起点**抬升的，所以这里每放宽 1 cm，就从围栏的跟随余量里吃掉 1 cm。取 1 cm 时余量约 4 cm |

##### F6 — `diag_franky_motion.py`（**基本重写**，T7）

| 改动 | 为什么 |
|---|---|
| **ramp 后新增 settle 观测窗**（`--settle-seconds`，默认 2 s），tracker 不立刻 stop | **T7 的核心**。旧脚本最后一个路点之后立刻 `tracker.stop()`，**事故那 26 cm 超调在物理上就发生在这个窗口里，而脚本把窗口关掉了**。这是本轮最高优先级的一处修改 |
| 采样从「每路点一次」改为 **20 ms 连续采样** | 事故只持续 3 s。20 ms 给约 150 个样本，能看到峰值；旧的 100 ms/路点只能看到端点 |
| 记录 `peak_overshoot`（沿运动方向超过**终点目标**的距离）、`peak_lag`、`peak_dz`、`min_dz`、`settle_drift`、`peak\|F_ext\|`、`peak\|dq\|` | 事故的核心量此前一个都没被采集 |
| 四道 abort：超调 / 绝对 z 天花板 / 跟随 / tracker 死亡 | 诊断脚本自己也必须能刹车。它带 `--yes-move` 命令真机，此前唯一的「保护」是人眼 |
| abort 时的刹车 = 与控制器同一套（先 `set_target(测量)` 再 `stop`） | 同 F2 |
| clip 改用 `error_clips_for_stiffness` | 旧脚本硬编码 `K_TRANS=2000` + `clip=0.05`，即**它复现的正是造成事故的 100 N**，却在注释里自称「与 env 参数一致」。现在两边共用同一个推导 |
| 新增 `--test-waypoints` | 复现 `_interpolate_move` 真正的指令形状：从**一次采样**的位姿出发、10 Hz 零阶保持的绝对路点阶梯。平滑 ramp 不是 reset 在做的事 |
| 新增 `--test-recover-loop` | T8 的对照实验：同上但每个路点前 `recover_from_errors()` |
| 新增 `--ramp-speed` | 直接按 cm/s 指定，而不是心算 `dz/seconds` |
| `--test-cartesian-motion` 前加 dwell 并检查 `\|dq\|` | LOG-018 的假失败：紧接 ramp 启动会因残余关节速度触发 `joint_velocity_discontinuity`。现在它会先等、再检查，检查不过就拒绝而不是把 reflex 记成「原语坏了」 |
| `--probe` 增打 `\|F_ext\|`、`\|dq\|`、`cmd_success_rate` | `cmd_success_rate=0.0` 曾是 LOG-017 最直接的旁证，但当时没被打印 |

##### F7 — `step_cube_place_robot.py`

| 改动 | 为什么 |
|---|---|
| **起始位形硬门**（`check_start_pose`），在 `gym.make` 前 | R6。失败信息是可执行的操作指令；`--allow-start-outside-box` 是显式逃生门 |
| **夹爪硬门**：`holding is False` → 拒绝 | 闭爪任务空爪起跑会对空气 `grasp`，且 hover/奖励几何是按已知方块高度标定的。`--skip-gripper-check` 可绕 |
| 新增 `--reset-z-lift` / `--interp-speed` / `--force-ceiling` / `--guard-margin` / `--min-start-clearance` / `--no-waypoint-clear-error` | 事故前这些量只能改代码或靠隐式环境变量；现在能在**低权限**下先试一遍 |
| 环境变量在 `ray.init` **之前**导出 | Ray actor 继承 driver 环境。而且 tracker 启动日志会回显乘积 → 「生效了吗」可从日志回答，不靠信任 |
| 整段 env 使用改为 `try: _run_env(...) finally: _close_env(env)` | 我自己查出的风险：此前每个 `raise` 各自记得调 `_close_env`，唯一漏掉的那个正是 **`env.reset()` 自己抛错**——也就是事故当次。结果 Ray actor 与 1 kHz 力矩线程只能靠进程死亡回收 |
| `_close_env` 从只 catch `AttributeError` 改为 catch 广泛 | 它现在跑在 `finally` 里，二次异常会**顶掉真正的失败原因**。丢证据正是 LOG-011…017 那七轮的成本来源 |
| 打印 `controller health` | 确认围栏真的装上了 |

##### F8 — `test_franky_controller_ext.py`（标定 REPL，我自己查出的四处）

| 改动 | 为什么 |
|---|---|
| `nudge` 单次 ≤ **0.2 rad** | `stream` 早就有 0.5 rad 总位移上限，而 `nudge` **完全没有上限**：手误 `nudge 2 1.5` 会让肩关节在一次阻塞运动里转 86° |
| `home` 必须写 `home yes` | 它是从**当前任意位形**到工厂关节位的一次阻塞 `JointMotion`。夹着方块贴在桌面附近时，那是一次会把方块拖过整个台面的横扫；而标定流程本来就明确说「不要敲 home」 |
| 夹着东西时 `open` 必须写 `open yes` | H1 第一步的 `open` 是空爪，正常；但方块夹好、臂已引导到标记上方之后，一个 `open` 就是把方块摔下去 |
| `impedance` 回显 `describe_authority(...)` | 请求的刚度会被 clamp、clip 会被反推，所以**不能假设输入的 kt 原样生效** |
| 新增 `mode` / `health` 命令 | 标定过程中随手查 `robot_mode`，不必退出去跑别的脚本 |

##### F8b — `configs/docker_run_franky_5090.sh`（T4）

加 `--shm-size`（默认 10g，`RLINF_FRANKA_SHM_SIZE` 可改）、`set -euo pipefail`、`REPO` 存在性检查，以及说明为什么必须 `--privileged`（1 kHz 力矩循环需要 RT 调度 + `mlockall`）和 `--network host`（libfranka 走 `172.16.0.2:1337`）。**注意当前运行中的容器是旧参数起的，`/dev/shm` 仍是 64 MB**，重建后才生效。

##### F9 — `run_cube_place_phase2.sh`（T3）

新增 `diag-probe` / `diag` / `diag-replay` 子命令；**所有**子命令都把多余参数透传给 python（`"$@"`），这样 `reset --force-ceiling 15` 之类不必绕过封装。`diag` 刻意**不含** `--test-cartesian-motion`（LOG-018 的顺序副作用 + T1 已关闭）。

##### F10 — `step_cube_place_dummy.py`（1A 现在真的守住这些不变量）

新增断言：`reset_z_lift_m == 0.03`；围栏/健康 API 存在；**力上限跨刚度恒定**（500/2000/3000 三档都 ≤25 N、≤10 N·m）；刚度被夹到 `[50,3000]`/`[5,300]`；**速度上限成立**（10 cm 需 5 s，且 `0.10/duration ≤ 2 cm/s`），且「比上限更慢的请求会被尊重」；**起始位形门**能拒 LOG-019 那个位姿、能拒 xy 过远与 z 过高、能放行一个正常起点；**位移上限不变量** `0.35 ≤ 20 × 0.02`。

之所以要把这些做成 1A 断言：它们此前全部依赖「没人犯错」。现在 1–2 秒就能验证。

**另外补了一个 1A 覆盖不到的缺口（R18）：** `arm_motion_guard()` 在 dummy 路径下**根本不会被执行**（`is_dummy` 时不调 `_setup_hardware`，没有控制器可调），所以关键字参数写错只会在**真机上、恰好该装围栏的那一刻**暴露。已核实 `rlinf/scheduler/worker/worker_group.py:436` 是 `.remote(*args, **kwargs)`，kwargs 会原样转发；再在 1A 里用 `inspect.signature(...).bind(...)` 按**真实调用形式**绑一遍 `set_motion_guard` / `motion_health` / `freeze_at_current` / `gripper_holding`，把 arity 与参数名错误挡在无臂阶段。

顺带注意：上游 franka 代码里**所有** `self._controller.*` 调用都是位置参数，没有任何现成的 kwargs 调用点——也就是说这条 RPC 路径在本仓库里是我第一次用关键字参数走，值得这一步核实。

#### 20.3 除 R1–R6 之外自查出的风险（均已修）

| ID | 风险 | 位置 | 后果 | 处理 |
|---|---|---|---|---|
| **R7** | `FrankaEnv.__init__` 自带一次无界 `_interpolate_move(self._reset_pose)` | `rlinf/.../franka_env.py:212` | 阶段 3 训练启动后的**第一个动作**就是从任意位姿出发、无限速、无围栏的插值。烟测靠 `safe_smoke_hold` 躲过了，训练默认躲不过 | mixin 限速 + `_setup_hardware` 末尾装围栏 + 位移超 0.35 m 拒绝 |
| **R8** | 时长上限 20 s 让位移 > 0.4 m 时速度重新突破上限 | 我新写的 `motion_limits` | 「限速」在最大的位移上失效——恰好是最危险的场合 | `MAX_INTERP_DISTANCE_M = 0.35` 直接拒绝，并把 `0.35 ≤ 20×0.02` 做成 1A 断言 |
| **R9** | 烟测脚本无 `try/finally`，`env.reset()` 抛错就漏掉 `env.close()` | `step_cube_place_robot.py` | 事故当次就是这条：Ray actor + 1 kHz 力矩线程只能靠进程死亡回收 | 单一 `finally` 出口 |
| **R10** | `_close_env` 只 catch `AttributeError` | 同上 | `finally` 里的二次异常会顶掉真正的失败原因 | catch 广泛并打印类型 |
| **R11** | REPL `nudge` 无幅度上限 | `test_franky_controller_ext.py` | 手误一次转 86° | 0.2 rad/次 |
| **R12** | REPL `home` 无确认 | 同上 | 夹着方块从任意位形横扫工厂位 | 需 `home yes` |
| **R13** | REPL `open` 无持物检查 | 同上 | 在高处直接摔方块 | 持物时需 `open yes` |
| **R14** | REPL `impedance` 刚度无上限 | 同上 | 任意刚度 × 默认 clip = 任意大的力 | `clamp_stiffness` + 回显真实力上限 |
| **R15** | 空爪也能跑闭爪任务 | `step_cube_place_robot.py` | 对空气 `grasp`；hover/奖励几何按方块高度标定，空爪整体失真 | 探测夹爪 + Ray 启动前拒绝 |
| **R16** | `ee_pose_limit` 全零时若装围栏会全程误报 | `arm_motion_guard` | 第一个路点就 abort，护栏反成故障源 | 全零则拒装并 warn |
| **R17** | 围栏只管 xyz，**姿态无围栏** | `set_motion_guard`（我第一版） | `K_r × rot_clip` 虽已从 45 降到 10 N·m，但 10 N·m 已经贴着 j5–j7 的关节限值，旋转失稳会甩腕。平移有围栏而旋转没有 | 新增第三道：测量姿态与目标姿态的**最短弧角** > `clip_rz×√3 + 0.2 ≈ 0.806 rad` → 刹车。**刻意不用欧拉比较**：本任务 roll ≈ −3.116 rad，离 −π 只差 0.026，欧拉读数落到 +π 那侧就会把正常姿态判成严重越界 → 误报。四元数最短弧角无绕回。1A 断言：2π 等价 → 0 rad，90° 扭转 → 1.571 rad > 0.806 |

> R17 的取舍值得单独说明：在安全护栏里写角度绕回逻辑，写错的两种结果分别是「误报把好运动打断」和「漏报给人虚假的安全感」，都不可接受。所以选了一个**结构上不可能绕回**的量（四元数最短弧角），而不是去把欧拉比较写对。

**已知但**不改**、只记录的上游行为**（改它们需要动 `rlinf/` 或缺乏证据）：

| 现象 | 位置 | 为什么不动 |
|---|---|---|
| `np.allclose(tcp, reset_pose, 0.02)` 里 `0.02` 是 **rtol** 而非 atol | `franka_env.py:525` | 上游行为；它是收敛判据而非安全监控，真正的安全由围栏承担。记入手册 §1.5 |
| `step()` 用上一步的 `_franka_state` 算 `next_position` | `franka_env.py:330` | 上游行为；`step` 的目标会被裁到盒内，且增量很小 |
| 零空间刚度仅 5 N·m/rad，肘部近乎自由 | `franky_controller.py:54` | 事故中 z+36 cm 伴随 x−6 cm 的圆弧与它有关，但**没有数据**支持某个具体新值。`RLINF_CART_K_NS` 可调 → 列为 T10，先测再调 |

#### 20.4 验证：1A dummy 回归 PASS

```bash
docker exec -e RLINF_SKIP_CAMERA=1 rlinf-franky-5090 bash -lc \
  'source /workspace/RLinf/b/x/configs/setup_before_ray_5090.sh; \
   cd /workspace/RLinf && python b/x/scripts/step_cube_place_dummy.py'
```

输出（节选，全部为新断言的实测值）：

```
gym_id=FrankyCubePlaceEnv-v1
wrapper_stack=['OrderEnforcing','PassiveEnvChecker','Quat2EulerWrapper','RelativeFrame','GripperCloseEnv','FrankyCubePlaceEnv']
action_space=Box(-1.0, 1.0, (6,), float32)
clips xy=(0.05,0.05) z_low=0.005 z_high=0.08 rand_xy=0.03
reset_z_lift_m=0.03
go_to_rest uses close (-1.0), not open (+1.0)
motion guard API present, and arm_motion_guard's call signature binds
force ceiling holds across stiffness: <= 25.0N / 10.0Nm
stiffness clamped to [50,3000] / [5,300]
interpolate speed capped at 2.0cm/s (10cm needs 5.00s)
start-pose gate rejects the LOG-019 pose and accepts a good one
interp distance cap 0.35m <= 20.0s x 0.02m/s
orientation fence 0.806rad is wrap-free (2pi-equivalent -> 0.00e+00rad, 90deg twist -> 1.571rad)
Phase1A PASS FrankyCubePlaceEnv-v1
```

**事故前后的权限对照：**

| 量 | LOG-019 当时 | 现在 |
|---|---|---|
| 指令力上限 | 100 N | **25 N** |
| 指令力矩上限 | 45 N·m | **10 N·m** |
| reset 抬升 | 0.10 m | **0.03 m** |
| 抬升指令速度 | 10 cm/s | **2 cm/s** |
| 平移超程时谁叫停 | 无（人拍 user-stop，+36 cm 时） | 围栏，**0.392 m**（约 +12 cm 处） |
| 姿态超程时谁叫停 | 无 | 围栏，与目标姿态夹角 > **0.806 rad** |
| 「没在跟随」时谁叫停 | 无（哑失败护栏只查 tracker 死没死） | 围栏，`\|测量−指令\| > 0.05 m` |
| 插值位移无上限 | 是 | **> 0.35 m 直接拒绝** |
| 起始位形检查 | 无 | Ray 启动前硬门（z / xy / 夹爪） |
| 诊断能否看到超调 | **不能** | settle 窗 + `peak_overshoot` + 四道 abort |
| `env.reset()` 抛错时是否 `env.close()` | **否**（漏了） | 是（单一 `finally`） |

**未跑：** 1B（GPU dummy SAC）——本轮改了 env/控制器/配置字段，应在阶段 3 前重跑一次，列为 **T9**。真机验证全部待 2.4b。

#### 20.5 一次失败的尝试也记下来（多视角审计 workflow）

先起了一个 6 视角 × 2 对抗校验 × 1 综合的审计 workflow。它**失败了**：6 个 finder 里 1 个重试后返回 `None`，另有 4 个 key 反复重试。原因是我给的 structured-output schema 太重（每条 finding 9 个必填字段，还要求每个 agent 产出 5–12 条），加上 verify 阶段要把全部 finding 拼成一个巨大的 digest。已 `TaskStop`，改成更精简的两路（复查我新写的代码 + 审计尚未手工细看的配置/夹爪文件），schema 减到 5 个必填字段。

记这一条的理由和记 LOG-007 一样：**下次别再这么设计 schema**。

#### 20.6 手册 `dmo_place_2.md` 的改动

| 位置 | 改了什么 | 为什么 |
|---|---|---|
| 抬头状态表 | 2.4 改「部分 PASS，不足以放行 2.7」；2.7 改「FAIL（安全事故）」；新增 2.4b 行 | 旧表把 2.7 的失败归因于 user-stop（LOG-017），已经不对 |
| §0 铁律 | 三条 → **四条**，新增「危险的量永远是两个量的乘积」+「体检 PASS 只对覆盖过的速度和幅度有效」 | 这是事故唯一的结构性教训 |
| **新增 §1.5** | 「相反的失效模式：运动权限被两个默认值放大」——与 §1.4 `UserStopped` 对称成篇：两个因子的表、10 倍速度、日志全绿的现象表、为什么没人拦、现在谁在拦 | 手册此前只教了「不动」这一种失效，读者会以为动了就是好的 |
| §2.1 安全规则 | 规则 3 补「**机器强制**」与具体阈值；新增 3b「诊断三项不要串跑」 | 旧规则只是文字要求，没有任何代码检查；串跑是 LOG-018 假失败的原因 |
| §2.0 分工表 | 拆成 2.4a/2.4b，加「顺序不可跳」 | — |
| **§2.4 整节重写** | **删掉「三项 OK → reset 应该一次过」**，换成速度/幅度四档阶梯 + `peak_overshoot` 门闩 + 分支表 | **就是这句推断放行了 2.7。** 不删掉它，下一个人会重犯 |
| §2.7 | 期望值全部更新（0.03 抬升、`over 1.50s`、`authority ≈ 25 N`、`motion guard armed`、`dz≈+0.040`）；新增三种 abort 的处理 | 与代码对齐 |
| §2.9 门闩 | 新增 2.4-0（力上限）、2.4-2（无超调）、2.4-3（真实位形）、2.6-2（起始位形）、2.7-2b（围栏已装）、2.7-3b（不超程）；末尾加「不要为了过门闩放宽安全量」 | — |
| §3.1 / §3.2 / §3.3 / §3.4 | 加 `motion_limits.py`；权限表重写（乘积在前）；damping 行改「被忽略并 warn」；reset 改五段；§3.4 拆成 A 哑失败 / B 超程两组 | §3 声明写的是「代码现在的实际行为」，必须逐一对齐 |
| §3.4 末尾 | 明确指出 LOG-014 对两条已撤回改动的「证伪」**无效**（在 `UserStopped` 下做的） | 否则下一个人会以为 R5 已经排除 |
| §5 | 新增 `reset_z_lift_m` / 力上限 / 插值速度三行 + 25 N 够用的量化论证 | — |
| §6 排障 | 新增 7 行：臂突然大幅移动、两种 `motion guard abort`、位移拒绝、起始位形拒绝、空爪、`authority` 不对 | 按症状索引才能在现场用 |
| §7 待办 | T1 关闭、T3/T7 完成、T5 改「部分」、新增 T8（R5 测量）/T9（1B 重跑）/T10（零空间刚度）；风险表补三条 | — |
| 文末「下一个动作」 | 改为「恢复 → 2.4a → 2.4b 四档 → 才回 2.7」，并写明「2.4b 之前不要跑 reset」 | — |

#### 20.7 教训（补充 LOG-019 的五条）

6. **修法要改变谁是自由变量，而不是把同一个陷阱挪个位置。** 「把 clip 调小」只是又选了一个魔数；「让 clip 由力上限反推」才让「改刚度会改力」这件事在结构上不可能发生。
7. **在没有数据的事情上不要改默认值。** R5 有充分的怀疑理由（而且它的「证伪」确实无效），但我只加了旋钮和对照测试，默认仍是上游行为。LOG-011…016 那七轮就是靠猜改代码烧掉的。
8. **诊断脚本本身也是安全设备。** 它带 `--yes-move` 命令真机，却既没有 abort 门也测不到关键量。事故后的第一轮测量必须先有一个能刹车、能看到峰值的脚本，否则又是一次盲跑。
9. **`finally` 里的二次异常会吃掉真正的失败原因。** 这类「丢证据」的代码不是整洁度问题，它直接换算成调试轮次。
10. **护栏报错是它在干活，不是它挡路。** 所以门闩里专门写了「不要为了过门闩放宽 `--guard-margin` / `--force-ceiling` / `clip_z_range_low`」。

#### 20.8 本轮之后的状态

| 项 | 状态 |
|---|---|
| 阶段 1A | **PASS**（含全部新增安全断言） |
| 阶段 1B | 未重跑（T9） |
| 阶段 2.4a/2.4b | **未做**，是下一个动作 |
| 阶段 2.7 | 护栏已就位，**待 2.4b 之后重跑** |
| 机器人 | 仍在 LOG-019 之后的状态，**需先做 LOG-019 的现场恢复六步** |
| 代码 | `b/x/` 11 个文件（+1519/−182），`rlinf/` **一行未改** |
| 手册 | 已同步（20.6） |

**下一个动作：**

```bash
# 0) 现场恢复：见 LOG-019「现场恢复步骤」，务必把臂引导回标记上方 3-5 cm、方块夹好
ray stop
bash b/x/scripts/run_cube_place_phase2.sh diag-probe
#    必须 Idle + has_errors=False，且 authority 一行约 = 25.0N

# 1) 2.4b 四档，一次一档，档间回 diag-probe
bash b/x/scripts/run_cube_place_phase2.sh diag --dz 0.03 --seconds 3     # 1 cm/s
bash b/x/scripts/run_cube_place_phase2.sh diag --dz 0.03 --seconds 1     # 3 cm/s
bash b/x/scripts/run_cube_place_phase2.sh diag --dz 0.10 --ramp-speed 0.02
bash b/x/scripts/run_cube_place_phase2.sh diag-replay --dz 0.03          # 10 Hz 阶梯

# 2) 四档都 alive 且 peak_overshoot <= 0.02 才回 2.7，首次压低权限
bash b/x/scripts/run_cube_place_phase2.sh connect
bash b/x/scripts/run_cube_place_phase2.sh reset --force-ceiling 15 --interp-speed 0.01
```

每档记录 `peak_overshoot` / `peak_lag` / `final_dz` / `peak|F_ext|`，回填本日志。**2.4b 之前不要跑 `reset`。**

---

### LOG-021 | 两轮独立审计 + 第二波修复 | 1A / 1B 双 PASS，真机仍待测

**背景：** LOG-020 落地后，起了两路独立审计复查我刚写的这批安全代码（它当时没有任何人看过）。第一路审计尚未手工细看的配置/夹爪/引导文件，出 24 条；第二路复查新代码本身，**读了 franky 的 C++ 源码**（`cartesian_impedance_base.cpp`、`torque_control_utils.hpp`、`tracker.py`、`robot.hpp`），出 15 条并**推翻了我的一个核心论断**。

#### 21.1 我说错的地方（必须先记这个）

##### C1 — 「`刚度 × clip` 是力上限」只对了一半

读 `cartesian_impedance_base.cpp`：

```cpp
error.head(3) = O_T_EE.translation() - target.translation();
error.head(3) = error.head(3).cwiseMax(-clip).cwiseMin(clip);   // 逐轴！
wrench = -K * error - D * (measured_twist - desired_twist);      // 阻尼不裁剪
```

| 我原来的说法 | 实际 |
|---|---|
| `K × clip` = 指令力上限 | 只是**弹簧项**，而且是**逐轴**的。三轴同时饱和是 `K·clip·√3` |
| 「25 N」 | 逐轴 25 N → worst case **43.3 N** |
| 旋转「10 N·m，因为 j5–j7 限值约 12 N·m」 | `150 × 0.0667 × √3 = 17.3 N·m`，**反而超过了我用来定这个值的那个 12 N·m** |
| 力上限恒定就等于降低了权限 | **阻尼项完全不被裁剪**，且 franky 的阻尼是由刚度推导的临界阻尼（`K_t=2000` 时约 155 N·s/m）。因 `desired_twist≡0` 它只阻碍运动、不驱动超程，但：**抬高刚度会以 √K 抬高阻尼，无论 clip 怎么推导**。推导 clip **不能**让 `K_t=2000` 等价于 `K_t=500` |

**修法：** 同时设**逐轴**上限（20 N / 6 N·m）与**范数**上限（40 N / 12 N·m），取先约束者；`describe_authority` 现在同时打印两个数并注明阻尼不在内；1A 断言两者都成立。旋转的逐轴默认从 10 N·m 降到 **6 N·m**（`6×√3 = 10.4 < 12`）。

##### C2 — 「全程力饱和」这个机理论断是错的

LOG-019 R1 我写的是：「1 cm/s 滞后 4.7 mm，线性外推到 10 cm/s → 4.7 cm → 吃满 5 cm clip → 阻抗全程输出 100 N」。

按真实阻尼反算：`K·e = D·v + F_friction`。1 cm/s 时 `D·v = 155 × 0.01 = 1.55 N`，实测弹簧力 9.4 N → **摩擦约 7.85 N，与速度无关**（franky 的摩擦前馈默认关闭）。所以：

| 速度 | 需要的力 | 对应滞后 | 是否吃满 50 mm clip |
|---|---|---|---|
| 1 cm/s | 9.4 N | 4.7 mm | 否 |
| 2 cm/s | ≈11 N | ≈5.5 mm | 否 |
| **10 cm/s（事故）** | **≈23.4 N** | **≈11.7 mm** | **否，远未饱和** |

**所以 100 N 是「可用权限」而不是「实际输出」，26 cm 超调不是饱和造成的。** 真实机理（更像是 10 Hz 零阶保持目标 + 高刚度 + 极软零空间 + 伸展位形下的失稳）**仍未确定** → 新增 **T11**。

这条特别值得记：我在 LOG-019 里批评过 LOG-011…016「靠猜改代码」，然后自己在 LOG-020 里给出了一个**没验算过的定量机理**。护栏本身仍然正确（它们降低权限、增加拦截，与机理无关），但**结论的确定性被我夸大了**。手册 §1.5 与 LOG-019 R1 都已加更正框。

##### C3 — 刹车方向对超调是**反的**

`_abort_motion` 原来对所有情况都「先 `set_target(测量位姿)` 再 `stop()`」，理由是「消掉位置误差让阻抗减速」。但在**超调**情形下臂已经冲到目标**前面**，误差 `live − target` 本身就在产生**向回**的弹簧力——`set_target(测量)` 恰好把这个复位力清零，只剩阻尼。**比什么都不做更弱。** 而且后面还固定 `sleep(0.25)`，按事故的 26 cm/s 又多走 6.5 cm 才执行真正会减速的 `stop()`。

| 失效类型 | 谁在跑 | 正确顺序 |
|---|---|---|
| `fence` / `orient`（臂冲过目标） | **臂** | **先 `stop()`**（libsranka 的受控停止主动减速），别把弹簧扔掉 |
| `lag`（目标跑在臂前面） | **目标** | **先 `set_target(测量)`**，消掉跑飞的目标 |

已改为按类型分向，且固定 `sleep` 换成按 `|dq|` 轮询到停住（上限 0.25 s），并把刹车期间走过的距离记进日志。诊断脚本的 `_brake` 也是同一问题，一并说明。

#### 21.2 第二波修复（19 个文件，+3273 / −286）

按严重度，只列有实质后果的。

##### 关键（三条）

| ID | 问题 | 修法 |
|---|---|---|
| **W1** | **libfranka 自己的 1 kHz 反射从未收紧。** 上游 `_FORCE_THRESHOLD = [100,100,100,25,25,25]`，是软件力上限的 5 倍，所以**硬件界永远不可能先响**——于是所有实际的界都在 Python 里：一个 GIL 约束的 50 Hz 守护线程监督 1 kHz 力矩环。按事故的位移速率，一个看门狗周期约 5 mm，一次 100 ms 的 GIL 停顿就是 2.6 cm | `__init__` 里 `set_collision_behavior`，笛卡尔力/力矩降到 **40 N / 12 N·m**（= 范数上限，两个数由同一处推出所以不会再漂）。关节力矩阈值**刻意不动**（收紧会招来自身动力学误反射，且 j5–j7 已是 11 N·m）。失败只 log 不阻断构造，但会明确说「只剩 Python 护栏」 |
| **W2** | **`step()` 仍在命令 20 cm/s。** 限速只覆盖 `_interpolate_move`；`FrankaEnv.step` 是 `next_position += action × action_scale[0]` 后直接 `_move_action`。PegInsertion 的 `action_scale[0]=0.02` @10 Hz = **20 cm/s**，比事故那次还快一倍。而且因为 `step` 每周期都从**实测位姿**重算目标，滞后不累积，**围栏的跟随判据也看不见它** | `CubePlaceConfig.__post_init__` 把 `action_scale[0]` 夹到 **5 cm/s**（0.005 m/步）并大声 warn；mixin `_clamp_step_slew` 再兜一层。5 cm/s 也低于臂在该力上限下的终端速度（约 13 cm/s），所以动作空间此前**大部分是物理上到不了的**——这对策略学习也是坏事。**注意这改变了 RL 动作空间语义** → T12 |
| **S3** | **夹爪的 `epsilon=0.05` 配 `width=0.01` 让空爪也报「抓到了」。** 接受窗是 `[-0.04, 0.06]`，而手爪全程行程只有 0–0.08 m → **任何小于 6 cm 的终止宽度都满足**，包括合到 0 的空爪。于是 `is_grasped` 为真 → `_hardware_holding` 为真 → `close()` 永远走 `skip grasp`、`is_open` 永远为假、**我刚加的「夹爪必须夹着东西」硬门被毒化**，臂会带着按方块高度标定的几何空爪下探 | `_hardware_holding` **只看测量宽度**落在标定窗内（`0.046 ± 0.012`），**彻底不用 `is_grasped`**；`grasp` 改用标定宽度 + 紧 epsilon，事后**按宽度复核**，不符就抛错并告知设 `FRANKA_CUBE_WIDTH_M` |

##### 高（十条，摘要）

| ID | 问题 | 修法 |
|---|---|---|
| **W4** | 控制器里 `raise` **不会**传到调用方：`WorkerGroupFuncResult` 吞掉、发 SIGUSR1、处理器 `ray.kill` 所有 actor。所以烟测的 `try/finally` 对它要处理的那个情形**根本跑不到**，env 侧也永远看不到原因 | 新增非抛出的 `guard_tripped()` RPC；看门狗只**锁存**，mixin 在每次 `_move_action` 后轮询并在 **env 侧**抛普通异常（`finally` 才有效）。控制器内的即时刹车保持不变——那才是真正起作用的部分 |
| **S2** | `arm_motion_guard` 在盒子不可用时只 **warn** 然后继续 | 真机上 **`raise`**。「保护机制自己退场、运行继续」正是事故的同类错误，而 `ee_pose_limit` 全零恰是最可能出错的形状；紧接着就是 `FrankaEnv.__init__` 自己那次无界插值 |
| **S1** | `realworld_franky_camera.yaml` 带 `is_dummy: false` + 真 IP 却**没有任何几何** → `_reset_pose` 是基座原点，围栏无盒可围，`step()` 会把目标裁到 `(0,0,0)` | 删掉那几个真机键（两个消费者只读相机字段，从不建 env），并写明为什么 |
| **S2b** | franky 的**软关节限位斥力**一直是关闭的：只有同时给出两个限位向量才生效，上游两个都不给 | 传入 `fc.JOINT_LIMITS_LOWER/UPPER` + 激活距离/刚度/阻尼/上限；老版本 franky 若不认这些 kwarg 就退回不带并 warn |
| **S6** | 围栏只在**命令路点时**采样，路点之间、`sleep` 期间、回合之间全是空档——**而事故的超调物理上就发生在这种空档里** | 50 Hz 守护线程 + 在 `get_state()` 里也查（env 每次 `_interpolate_move` 和每个 `step` 之后都会调它） |
| **S5/S10** | 打印给操作员的安全盒**不是**实际生效的那个：`PegInsertionConfig.__post_init__` 会覆写 `ee_pose_limit_*`，且 roll/pitch 半宽是**硬编码 0.01 rad**，不是 `clip_rz`。旧的打印把姿态窗说宽了 **35 倍**，还是以「开跑前核对」的名义 | 新增 `effective_ee_pose_limits` 镜像真实推导（`connect` 不建 env，只能这样）；构造后再打印一次 `config` 里的真值。**没有**去「恢复显式传入的限位」——那会把 roll/pitch 从 ±0.01 放宽到 ±0.35，方向正好反了 |
| **S7** | 起始位形门不查**姿态**，但 `go_to_rest` 的第一条命令是「原地不动」→ 腕部偏离目标姿态就会让围栏在**臂完全没动**的情况下报「腕部被甩」 | `check_start_pose` 加姿态检查（四元数角，Ray 启动前拦）；`_check_start_pose` 同步加 warn。同时修正 `orientation_fence_rad`：三个半宽**不相等**（roll/pitch 0.01、yaw `clip_rz`），旧版按 `√3×clip_rz` 算出的围栏比自己的推理宽 1.5 倍 → 0.806 rad 收到 **0.550 rad** |
| **S9** | 只有位移上限，**没有转角上限**。可达路径：标定写成四元数前三位 → 目标姿态落在单位姿态附近 → 盒子与姿态围栏都跟着重新居中 → **没有任何围栏会响**，而 reset 会命令腕部翻 180° | `MAX_INTERP_ANGLE_RAD = 0.6`，超过直接拒绝，并在错误信息里点名「是不是把四元数当欧拉角了」 |
| **S10b** | `write_cube_place_pose.py` 只 warn 且**无条件**写 `calibrated: true`——而那个 flag 是唯一的「人从机器上读过」凭证 | 改为**拒写**（可达域按肩部半径 30–90%、接触高度、欧拉范围、roll 必须接近 ±π），`--force` 时 `calibrated` 写 **false** 并记下失败项，覆写前留 `.bak` |
| **S11/S12** | 围栏下沿是接触点**下方 5.5 cm**——那一侧有桌子；`--reset-z-lift` 还能把绝对天花板一起抬走 | −z 单独用 **1 cm** 余量；新增不随 lift 变动的**绝对 z 天花板**（盒顶 + 0.10 m） |
| **S15/S11b** | 没有任何 `close()` 覆写 → **即使成功收尾**，阻抗 tracker 也会一直发力矩到进程退出（`Cluster._shutdown_ray_at_exit` 走 `os._exit(0)`，atexit 也救不了） | mixin `close()`：`freeze_at_current` + `cleanup()`（**不**开夹爪——Franka Hand 断连后机械保持夹持，进程被杀方块不会掉，这是刻意的） |
| **S16** | 所有夹爪调用都是无超时阻塞 → 手爪不响应时 `reset` 永久挂住，同时占着 FCI、臂在阻抗下带电 | 6 s 超时执行器，超时后尽力 `stop()` 再抛错 |
| **S7b** | `RLINF_CUBE_*` 与 `FRANKA_GRASP_FORCE` **到不了训练路径的 actor**（Ray actor 继承 raylet 环境；训练路径先 `ray start`）。操作员在 `ray start` 之后 export 会被**静默忽略** | 两个 `setup_before_ray_*.sh` 都导出并回显，并写明改了要 `ray stop` → re-source → `ray start` |

##### 中 / 低（已修，不逐条展开）

`S4` `move()` 的 0–255 映射与 Robotiq 约定**相反**（255 = 全开）→ 不静默翻转语义（两个脚本依赖现行为），改为**持物时拒绝变宽**并在错误信息里说明约定差异；`S8` `open()` 不限速且上游 `open_gripper` 用 `speed=1.0`（手指上限约 0.1 m/s）→ 限速 + 覆写 `open_gripper` + 持物时警告；`S13` `close()` 的异常吞咽范围过宽 → 改为按宽度复核后才继续；`S14` 畸形 `FRANKA_GRASP_FORCE` 会在 actor 构造期抛 `ValueError` → 用容错读取器，且实际力超过默认值时 warn；`S18` `ray stop --force` 会 SIGKILL 活着的控制器 actor → 先检查后拒绝；`S17` `docker_run_franky_5090.sh` 加 1337 占用预检（防第二个容器抢 FCI）；`S19` `is_dummy`/`robot_ip` 一致性断言 + 把安全默认值下移到 env defaults；`S20` `safe_smoke_hold` 在所有 cube-place YAML 里都是 False → env defaults 钉 `True`；`S21` `use_spacemouse` 泄漏到 `env.eval` → 钉 `False`；`S22` `no_gripper` 从未被钉住（只靠上游默认值） → 钉 `True`；`S23` `ray_register_startup.py` 是**死代码**（`PYTHONSTARTUP` 只对交互式解释器生效，其 docstring 的说法是错的）→ 改写 docstring 说明真实机制是 `RLINF_EXT_MODULE`，并把三处 `except Exception: pass` 改成打印；`S24` `--clip-rz` 是唯一没有范围检查的几何参数（负值会让 `np.clip(x, lo, hi)` 在 `lo > hi` 时恒返回 `hi`）→ 加检查。

#### 21.3 审计确认「其实没问题」的地方（也记下来，免得重复怀疑）

- `FrankyControllerExtended.__init__` 在 `super().__init__()` **之前**设字段：安全。上游 `__init__` 先设 `_logger`，再 `_franky`/`_robot`，再 `_build_gripper`，最后才 `_cart_tracker = None`；没有任何一步碰 `_guard_*` / `_compliance_*`。
- 围栏不会误报：算术核对过——最高合法指令 z = 起始上限 0.32192 + lift 0.03 = 0.35192，围栏顶 0.39192，余量 4 cm；`step()` 满幅三轴指令距实测位姿 `0.005×√3 = 0.0087 m`，远小于 0.05 m 的跟随门。
- `_check_motion_guard` 拿**裁剪后**的目标比对是对的：那才是真正交给 tracker 的值；`_prev_cart_target_xyz` 在 `move_tcp_pose` 里必然已被 `super()` 赋值，不会是 `None` 也不会陈旧。
- `franky.Affine(xyz, quat)` 与 4×4 两种构造都合法；四元数是从 franky 自己的 `Affine.quaternion` 往返的，系数约定不会错。
- tracker 刚建好时查 `is_running` 不是竞态：`moveInternal` 在**启动控制线程之前**就把 `motion_generator_running_` 置真。
- `_in_waypoint_move` 标记无重入问题；`FrankaEnv.reset` 里那次直接的 `_clear_error()` 在其作用域之外，仍会走到上游。
- 进程被杀时方块**不会掉**：没有任何收尾路径会 `open()`，`FrankaLibfrankaGripper.cleanup()` 是刻意的 no-op，Franka Hand 断连后机械保持夹持力。已在代码里注明「不要把这个『修』成 `open()`」。
- `diag_franky_motion.py` 的 `peak_overshoot`、零阶保持、`target_period == 0` 分支、以及「至少调用一次 `set_target`」都被逐一核对为正确。

#### 21.4 验证

**1A（franky 容器，无臂）：PASS**，现在覆盖 9 组不变量：

```
force ceiling holds across stiffness: <= 20.0N/axis (40.0N norm) / 6.0Nm/axis (12.0Nm norm)
clip shortfall reported at low stiffness, silent at the shipping values
stiffness clamped to [50,3000] / [5,300]
interpolate speed capped at 2.0cm/s (10cm needs 5.00s)
interp distance cap 0.35m <= 20.0s x 0.02m/s
interp angle cap 0.6rad
step action_scale cap 0.0050m/step at 10Hz = 5cm/s (upstream 0.02 = 20cm/s)
cartesian collision thresholds 40N / 12Nm (upstream 100N / 25Nm)
orientation fence 0.550rad is wrap-free (2pi-equivalent -> 0.00e+00rad, 90deg twist -> 1.571rad)
start-pose gate rejects the LOG-019 pose and accepts a good one
effective_ee_pose_limits matches the constructed config (roll/pitch half-width 0.01rad, not clip_rz)
is_dummy / robot_ip invariant enforced
config fields in sync: ['clear_error_per_waypoint', 'safe_smoke_hold']
motion guard API present, and arm_motion_guard's call signature binds
Phase1A PASS FrankyCubePlaceEnv-v1
```

**1B（GPU 容器 dummy SAC）：PASS**，改动后跑了两次（改 env/控制器之后、改 `action_scale` 之后），均 `docker exit=0`，`sac/actor_loss` / `sac/critic_loss` / `sac/alpha` 齐全，env worker 里可见 `clamping to 0.0050 m/step`。**T9 关闭。**

**权限对照（三个阶段）：**

| 量 | LOG-019 事故时 | LOG-020 之后 | 现在 |
|---|---|---|---|
| 逐轴弹簧力上限 | 100 N | 25 N | **20 N** |
| 力范数 worst case | 173 N | 43 N | **34.6 N** |
| 逐轴力矩上限 | 45 N·m | 10 N·m | **6 N·m** |
| 力矩范数 worst case | 78 N·m | 17.3 N·m | **10.4 N·m** |
| **libfranka 硬件反射** | 100 N / 25 N·m | 100 N / 25 N·m（**没动**） | **40 N / 12 N·m** |
| reset 指令速度 | 10 cm/s | 2 cm/s | 2 cm/s |
| **`step()` 指令速度** | 20 cm/s | 20 cm/s（**没管**） | **5 cm/s** |
| 围栏采样 | 无围栏 | 仅命令路点时（10 Hz，空档为 0） | **+ 50 Hz 看门狗 + 每次 `get_state`** |
| 超调时的刹车 | 无 | **方向反了**（清掉复位力）+ 固定 0.25 s | **按类型分向 + 按 `\|dq\|` 轮询** |
| 软关节限位斥力 | 关 | 关 | **开** |
| 「是否夹着方块」 | `is_grasped`（空爪也为真） | 同（未察觉） | **只看测量宽度落在标定窗** |
| 成功收尾后 tracker | 一直发力矩到进程退出 | 同 | **`close()` 里显式停** |

#### 21.5 教训（补 LOG-019 的 5 条与 LOG-020 的 5 条）

11. **我批评过的错误，我自己在下一条日志里就犯了。** LOG-019 教训 1 写着「体检 PASS 只对覆盖过的工况有效」，然后我在 R1 里给了一个**没验算过的定量机理**（线性外推滞后）。防御性的改动不需要机理就成立，但**结论的置信度必须与证据匹配**——写「100 N 是可用权限」和写「全程输出 100 N」是两件完全不同的事。
12. **读被包装库的源码，不要只读它的 Python 层。** 「clip 是逐轴的」「阻尼不被裁剪」「软关节限位默认关闭」这三件都只在 C++ 里写着，而每一件都改变了结论。
13. **刹车这类操作有方向性。** 同一个动作（`set_target(测量)`）对「目标跑飞」是刹车，对「臂冲过头」是**松刹车**。安全动作必须按失效类型分派，不能只有一种。
14. **纵深防御要问「哪一层不依赖软件被调度」。** 我加了三层 Python 护栏却没动 1 kHz 的硬件反射——而在 GIL 里，一次 100 ms 停顿就是 2.6 cm。
15. **限速只覆盖了我看的那条路径。** `step()` 不走 `_interpolate_move`，于是整套「2 cm/s」在**训练路径上完全不成立**，反而是那条路径在命令 20 cm/s。改了一个瓶颈之后要问「还有哪条路绕过它」。

#### 21.6 现在的状态

| 项 | 状态 |
|---|---|
| 阶段 1A | **PASS**（9 组不变量） |
| 阶段 1B | **PASS**（重跑两次） |
| 阶段 2.4a / 2.4b | **未做，是下一个动作** |
| 阶段 2.7 | 护栏已就位，**待 2.4b 之后重跑** |
| 机器人 | 仍在 LOG-019 之后的状态，**需先做 LOG-019 的现场恢复六步** |
| 代码 | `b/x/` 20 个文件（+3273/−286），`rlinf/` **一行未改** |
| 手册 | 已同步（含 §1.5 的机理更正） |
| 未决 | T2、T5、T6、T8、T10、**T11（机理）**、T12、T13、T14 |

**下一个动作不变**，只是期望值更新：现场恢复 → `diag-probe`（核对 `authority` 为 `<= 20.0N/axis`、且出现 `collision behavior tightened: ... 40.0 ... 12.0`）→ 2.4b 四档 → 全部 `peak_overshoot ≤ 0.02` 才回 2.7。

---

### LOG-022 | 阶段 2.5 重标定 + 2.7 `reset` | 臂动了但差 4.2 cm 到不了悬停 | 根因 = **我引入的 slew clamp 掐死了插值**

**操作（用户，约 08:25–08:33 UTC）：** 松开 user-stop 后重夹方块、重做 H1 标定、`connect`、`reset`。**跳过了 2.4b**（本条不追究，因为 reset 的结果本身就把问题暴露得更清楚）。

#### 22.1 先说好消息：护栏与运动链路全部按设计工作

| 观测 | 判定 |
|---|---|
| `collision behavior tightened: cartesian force/torque thresholds=[40.0, 40.0, 40.0, 12.0, 12.0, 12.0] (was [100.0, ...])` | **硬件反射收紧生效**（W1） |
| `authority: spring force <= 20.0N/axis (34.6N worst-case 3-axis) [K_t=2000 x clip=0.0100m]; torque <= 6.00Nm/axis (10.39Nm worst-case)` | 力上限按设计，且逐轴与范数都打印了（C1） |
| `joint_repulsion=True` | **软关节限位斥力启用成功**，franky 接受了那两个限位向量（S2b） |
| `motion guard armed: xyz in [[0.5713,-0.1081,0.2145],[0.7713,0.0919,0.3895]] ... ceiling 0.4095, max_lag=0.050m, orient<=0.550rad` + `motion guard confirmed` | 围栏装上并被回读确认（S2/S8） |
| `reconfigure_compliance_params: ignoring ROS-only keys [...]` | damping/clip 映射已删除并明确 warn（C1/R4） |
| `robot_mode: RobotMode.UserStopped` 时 `connect` 只提示不拦；松开后 `reset` 看到 `Idle` | 模式门按设计分级（LOG-017） |
| `gripper: holding=True width=0.0365` → `close` 未抛错 | 宽度判定与复核生效（S3） |
| `watchdog_alive: True`、`guard_tripped: None`、全程无 abort | 看门狗在跑且没误报（S6） |
| `controller tracker stopped on close()`，且在 `RuntimeError` 之后仍执行 | `close()` 覆写 + `try/finally` 生效（S15/R9） |
| `dz=0.0255`（不是 `0.0000`） | **臂真的动了。** 与 LOG-011…017 的哑失败彻底不同 |
| `reach: worst box corner ... 86%; worst fence corner ... 92% NEAR-SINGULAR` | 新增的可达域提示生效（S11）。新 H1 的 x=0.6713 比旧的 0.7062 更靠近基座，是改善 |

**没有任何一处 abort、没有超程、没有夹爪意外张开。**

#### 22.2 失败点与根因

```
hover check: |xy-target|=0.0057m |z-hover|=0.0423m
RuntimeError: after reset, |z-hover|=0.0423 > tol 0.0250; arm did not go to mark+0.08m
```

日志里那串 `step slew clamped` 是全部证据 —— 请求距离**单调增长**而夹后值恒为 0.0087：

```
0.0102 → 0.0122 → 0.0143 → ... → 0.0306   全部夹到 0.0087
```

**根因：`_clamp_step_slew` 用 `self._franka_state` 作参考位姿，而上游 `_interpolate_move` 只在路点循环**之前**读一次状态（[`franka_env.py:856`](../../../rlinf/envs/realworld/franka/franka_env.py#L856)），循环里（864–867 行）从不刷新。**

于是路点 *k* 是 `起点 + k·Δ`，`|路点 − 陈旧状态|` 线性增长，我的 clamp 把**整个调用的有效目标钉在 `起点 + 0.0087`**，轨迹其余部分被夹掉。逐段核算与日志完全吻合：

| 阶段 | 起点 z | 夹后可达上限 | 实际到达 |
|---|---|---|---|
| 抬升（+0.030 指令） | 0.2416 | 0.2503 | ≈0.2476 |
| rest attempt 1 | 0.2476 | 0.2563 | ≈0.2540 |
| rest attempt 2 | 0.2540 | 0.2627 | ≈0.2606 |
| rest attempt 3 | 0.2606 | 0.2693 | **0.2672** |

每轮只前进约 6.5 mm；三次重试各自「跑完了」，所以**每一行日志单独看都正常**，只有增长模式暴露了它。

**这是 LOG-021 的 W2 修复引入的回归。** 它作为 `step()` 路径的兜底是对的（那里 `_franka_state` 每周期都刷新，`FrankaEnv.step` 正是从它算目标），但套到插值路径上参考位姿是陈旧的。而且在插值路径上它**本来就多余**：`interp_duration_s` 已经把路点间距压成 `插值速度 ÷ step_frequency = 2 mm`，比 8.7 mm 的步进预算小一个量级。

#### 22.3 修复

| 文件 | 改动 | 理由 |
|---|---|---|
| [`franky_single_franka_env.py`](../../../b/x/franky_ext/franky_single_franka_env.py) | `_interpolate_move` 用 `_in_interpolate` 标记作用域；`_move_action` 在该作用域内**跳过** slew clamp | 参考位姿在插值期间陈旧，且插值本身已限速 |
| [`franky_single_franka_env.py`](../../../b/x/franky_ext/franky_single_franka_env.py) | `_clamp_step_slew` docstring 写明它**只对 `step()` 路径有效**及为什么 | 免得下一个人再把它套到插值上 |
| [`cube_place.py`](../../../b/x/franky_ext/tasks/cube_place.py) | `_go_to_rest_pose` 在最后一次移动后**重新判定**，成功就不再 warn；失败信息里加上「查是否有 `step slew clamped` 洪流」 | 循环是移动**前**判定，末次成功也会打 NOT reached |
| [`step_cube_place_dummy.py`](../../../b/x/scripts/step_cube_place_dummy.py) | 新增不变量断言 `interp_speed ≤ step_speed` | 跳过 clamp 只在「插值是两条路径里更慢的那条」时安全。若有人把 `RLINF_CUBE_INTERP_SPEED` 抬到步进速度以上，这个推理就失效——现在会被 1A 拦住 |

**1A 回归 PASS**，新增断言输出：`interp 2.0cm/s <= step 5.0cm/s (so skipping the slew clamp mid-interp is safe)`。

#### 22.4 另一件要处理的：方块宽度标定与实际不符

| 量 | 值 |
|---|---|
| `FRANKA_CUBE_WIDTH_M` 默认 | 0.046 |
| 判定窗 | [0.034, 0.058] |
| **实际夹持宽度** | **0.0365** |
| 距窗下沿余量 | **仅 2.5 mm** |

现在能通过，但太靠边：方块稍微滑动或指垫压缩就会掉出窗外 → `holding=False` → `close()` 会在回合中途尝试重抓。应设成实测值：

```bash
bash b/x/scripts/run_cube_place_phase2.sh reset --cube-width 0.0365
```

（`--cube-width` 会在 `ray.init` 之前写 `os.environ`，所以能进 actor。）

#### 22.5 教训

16. **兜底措施也要问「它在哪条路径上被调用」。** clamp 对 `step()` 正确、对插值有害，差别只在「参考位姿是否新鲜」这一个上游实现细节里。加护栏时列一遍所有调用点，不只是我脑子里那一个。
17. **「每一行都正常，只有趋势不对」是这类 bug 的指纹。** 三次重试各自都报「跑完了」，是那串请求距离的**单调增长**把它交出来的。日志里保留原始数值（而不是只报成败）正是为此。
18. **注意：本条修复期间有一轮审计 workflow 正在读这些文件**，所以它针对 `franky_single_franka_env.py` / `cube_place.py` 行号的发现可能已过期，需按当前内容复核。

#### 22.6 下一步

```bash
ray stop
# 起始位形：臂应仍在 0.6658,-0.0066,0.2672（标记上方约 3.8 cm），方块夹着 → 门闩会过
bash b/x/scripts/run_cube_place_phase2.sh reset --cube-width 0.0365
```

**期望：** 不再有 `step slew clamped` 洪流；`rest pose reached on attempt 1`（或最多 2）；`|z-hover| ≤ 0.025`；`reset-only PASS`。

若这次过了，2.4b 仍应补做——它测的是超调（`peak_overshoot`），而 reset 只能证明「到得了」，证不了「不会冲过头」。

---

### LOG-023 | 代码 ↔ 手册对账（无臂） | 代码 7 处错漏 + 手册 12 处不符 | 核心是「控制器里的 `raise` 会把 driver 打掉」

**触发：** 用户要求「深入分析已改变的代码，检查 `dmo_place_2.md` 有没有不符合实际代码的地方，同时检查实际代码有什么错漏，一并修正」。**全程无臂**，只读代码 + 1A 桌面回归 + ruff。

**方法：** 通读 `b/x/franky_ext/`（`motion_limits` / `controller_extended` / `franky_single_franka_env` / `tasks/cube_place` / `tcp_probe` / `franka_libfranka_gripper`）与 `b/x/scripts/` + `b/x/configs/`，再逐节比对手册。凡是手册里写了具体数字、具体日志字符串、具体调用顺序的地方，都回代码核一遍——这类断言最容易在改代码时悄悄过期，而它们恰恰是操作员在真机前用来对账的东西。

---

#### 23.1 代码错漏（按严重度）

##### 发现 1（关键）—— 控制器里的 `raise` 让「为它写的那个 `finally`」永远跑不到

LOG-021 的 W4 已经识别出「Worker 方法里 `raise` 不会传到调用方」，并加了 `guard_tripped()` 轮询。**但只有看门狗线程被改成不抛，主线程这条路没改完**：

| 谁 | 当时的行为 |
|---|---|
| `_abort_motion` | 刹车之后 **`raise RuntimeError`** |
| `_check_motion_guard` | 已锁存时**也 `raise`** |
| `get_state()` | **每次**都调 `_check_motion_guard()` |
| `motion_health()` | 直接读字段，任何一处出错就抛 |

把这四条摆在一起，后果是链式的：

1. 围栏跳闸 → `_abort_motion` 刹车后抛 → 这个 `raise` 发生在 `move_arm` **内部**，而 env 侧的 `guard_tripped()` 轮询写在 `_move_action` **返回之后** → **轮询永远执行不到**，W4 的整套设计在主线程这条路上是死的。
2. 那个 `raise` 落到 `WorkerGroupFuncResult`：打印 → 置 `Cluster._run_failed` → 给主进程发 `SIGUSR1` → 处理器 `ray.kill` 掉**所有** actor → `exit(-1)`。
3. 于是烟测脚本的 `finally: env.close()` 跑不完；`close()` 里那句「停 tracker」——LOG-021 的 S15 专门加的——**不会执行**。
4. 就算侥幸跑到 `_close_env`，它第一件事是 `controller.motion_health()`，那会再走一次 `_check_motion_guard()`，锁存着就**再抛一次**，把 driver 打掉。

也就是说：**一次围栏跳闸的最终效果，是阻抗 tracker 带着最后一个目标继续发力矩，直到进程被杀。** 这正是 S15 要消灭的状态，而围栏跳闸是最需要它别发生的时刻。

**修法**（`controller_extended.py`）：把「检测 + 刹车」和「传播」彻底分开。

| 方法 | 现在的契约 |
|---|---|
| `_abort_motion` | 刹车 + 拆 tracker + **锁存**原因；**绝不抛**。日志末尾明说 `arm braked and latched; ... the env-side poll will raise` |
| `_check_motion_guard` | **返回**锁存原因（或 `None`）；只在**新**违规时调 `_abort_motion` |
| `motion_health` | 所有读取过一个 `_safe()` 包装，**保证不抛**；内部失败以 `guard_check_error` 字段报出 |
| `move_tcp_pose` | 先查锁存：已锁存则 `log_error` 后**直接返回**，不下发。这不是宽容——`super()` 会调 `_ensure_cart_tracking_motion` 把刚拆掉的 tracker 重建起来，等于在失控刹车几秒后重新给臂上电 |
| `_ensure_cart_tracking_motion` | **唯一**允许抛的地方，且只为拦住「跳闸后重建 tracker」。`move_tcp_pose` 已经先拒了，所以正常路径到不了这里；留着是因为它防的那件事是这个文件里后果最坏的 |
| env 侧 `_raise_if_guard_tripped` | 把锁存转成**普通 Python 异常**，在 `finally` 还有效的地方抛 |

##### 发现 2（高）—— 插值最后一段的跳闸会被「几何没达标」盖住

轮询点原本只有 `_move_action` 之后。但上游 `_interpolate_move` 在**最后一个路点之后**还有一次 `time.sleep` 和一次收尾 `get_state()`；这段时间里看门狗跳闸，逐路点轮询看不到，要等**下一次**命令运动才浮出来。

而 reset 的最后一段插值之后，先跑的是悬停几何判定。于是屏幕上只有一句 `|z-hover|=... > tol`——**真正的跳闸原因被一个下游症状盖住了**，这跟 LOG-022 那次「每一行都正常，只有趋势不对」是同一类误导。

**修法：** `_interpolate_move` 在 `super()` 返回后（`finally` 之外）补一次 `self._raise_if_guard_tripped()`。

##### 发现 3（高，本轮新引入的审计发现）—— 主线程和看门狗可能**同时**刹车

`_watchdog_loop` 在循环顶部查 `_guard_trip_reason is not None` 就退出，看起来「只刹一次」。但**原因是在刹完车之后才锁存的**（它要记录刹车期间走过的距离）。于是存在一个约 0.25 s 的窗口：主线程 `_check_motion_guard` 发现违规 → 进 `_brake`（按 `|dq|` 轮询，最长 0.25 s）→ 期间看门狗每 20 ms 醒一次，看到 `_guard_trip_reason` 仍是 `None`、tracker 仍在，于是**也**判定同一个违规、**也**开始刹车。

两个刹车交错的后果不只是日志难看：`fence` 走 `stop()` 优先，`lag` 走 `set_target(测量)` 优先，两条序列如果穿插，一个线程的 `set_target` 可能把目标重新指到另一个线程正在 `stop()` 离开的位姿上。

**修法：** 新增 `self._guard_trip_lock`。`_abort_motion` 和看门狗都在锁内「再查一次锁存 → 刹车 → 写锁存」。`_stop_watchdog()` 移到锁**外**调用——否则主线程持锁去 join 一个正阻塞在这把锁上的线程，会一直卡到 1 s 的 join 超时。

1A 新增断言：已锁存时 `_abort_motion` 直接返回且**不覆盖**原有原因。

##### 发现 4（中）—— `__file__` 路径算术在四个脚本里少数一层

```python
REPO = os.environ.get("REPO_PATH", os.path.abspath(os.path.join(__file__, "../../..")))
```

`__file__` 是**文件**不是目录，所以第一个 `..` 只是脱掉文件名。`b/x/scripts/foo.py` 往上三层得到的是 `<repo>/b`，不是 `<repo>`。后面 `os.path.join(REPO, "b", "x")` 就成了 `<repo>/b/b/x`。

之所以一直没炸：手册里所有路径都经 `run_cube_place_phase2.sh`，而它（间接）导出了 `REPO_PATH`，`os.environ.get` 走的是第一分支。**这是一个只在「不按手册跑」时才会现身的 bug**，也就是排障时最可能出现的那种跑法。

`step_cube_place_robot.py`、`step_cube_place_dummy.py`、`write_cube_place_pose.py`、`test_franky_controller_ext.py` 四个已改成 `../../../..`。

顺带在 `b/x/scripts/` 里发现同一个 bug 还留在 franka_3 阶段的 `step2_test_gripper.py` / `step3_test_controller.py` / `step5_test_env_robot.py` / `step6_test_peg_env_robot.py` 里（不属本手册范围，但同一目录、同一错法，一并修了）。其中 `step3_test_controller.py` 还有第二处：`sys.path.insert(0, os.path.join(REPO, "b", "d"))` —— 扩展包早就从 `b/d` 搬到 `b/x` 了，`b/d` 现在只有文档。这行一直是死代码，靠 `setup_before_ray_5090.sh` 的 `PYTHONPATH` 兜着才没暴露，已改指 `b/x`。

##### 发现 5（中）—— 错误信息里让人用一个**不存在**的旗标

`motion_limits.clip_shortfall` 的提示写着「抬 `RLINF_CUBE_TORQUE_NORM_CEILING_NM`」，而 `--force-ceiling` / `--force-norm-ceiling` 都有 CLI 旗标、唯独力矩范数没有。操作员照提示敲 `--torque-norm-ceiling` 会得到 `unrecognized arguments`。已给 `step_cube_place_robot.py` 和 `diag_franky_motion.py` 都补上该旗标与对应的环境变量导出。

##### 发现 5b（中）—— `connect` 号称是开跑前的几何核对，却看不到可达域体检

写手册 §2.6 时想把 `reach:` 一行列进期望输出，回代码一查：`reach_report` / `worst_reach_corner` **只**在 `controller_extended.set_motion_guard` 里被调用，也就是 `gym.make` 之内。`--connect-only` 明确**不**建 env，所以它从来没打印过这一行。

这正好把 `connect` 的定位打了个折扣：它存在的意义是「在臂动之前、人还有时间挪标记的时候」发现几何不对，而「最坏角落已接近奇异」恰恰是这类问题里最该早知道的一个——等到 `set_motion_guard` 打印时，臂已经在 `reset` 的路上了。

**修法：** `_print_geometry` 里补一行盒角可达域（纯几何，不需要连机器人）。围栏角仍只在装围栏时报，因为围栏余量要到那时才确定。核对过数值与 LOG-022 完全一致：盒角 `r=0.732m (86%)`、围栏角 `r=0.788m (92%) NEAR-SINGULAR`。

##### 发现 6（低）—— `run_cube_place_phase2.sh` 依赖调用者的环境

它假定 `REPO_PATH` 已被 `setup_before_ray_5090.sh` 导出。多数时候成立，但这让「脚本能不能用」取决于调用者做过什么。已在脚本内 `export REPO_PATH="${REPO_PATH:-${ROOT}}"`，`ROOT` 由脚本自身位置解析。

##### 发现 7（低）—— 关于 Ray 环境变量继承的注释是错的（手册里也是）

`step_cube_place_robot.py` 原注释说这些 `os.environ` 赋值靠「在 `ray.init` 之前设好、被继承下去」生效。真实机制更绕，而且**必须写对，否则下一个人会以为 CLI 旗标不可靠**：

`_RemoteNodeProbe` 是 `@ray.remote` actor，它采到的 `default_env_vars` 是 **raylet** 的环境；`ClusterInfo._configure_node_envs` 拿 **driver** 的 `os.environ` 与之**做差**，把差异并进 `node.env_vars`；`Cluster.allocate` 再把它作为 `runtime_env["env_vars"]` 下发给每个 worker。所以 driver 在 `ray.init` 前改的环境变量**确实能到 actor**，但走的是显式转发，不是继承。注释与手册 §3.2 都已按这个机制重写。

---

#### 23.2 核对为「不是问题」的（记下来免得重复怀疑）

- **跳闸后的收尾链是安全的。** `_abort_motion` 拆掉 tracker 后，`close()` 里的 `freeze_at_current()` 走「没有 tracker，无可刹」分支并 `return False`；`cleanup()` 的 `_stop_tracking_motion` / `_stop_cart_tracking_motion` 对 `None` 都是空操作。**读代码确认，未在真机上走过** → 新增 **T15**。
- `setup_before_ray_*.sh` 里 `export RLINF_CUBE_Z_CEILING=""` 不会让 `float("")` 炸：`_env_float` 用 `raw.strip()` 判空。
- `use_dense_reward` 没漏：env 包 `env/realworld_cube_place.yaml` 和 `step_cube_place_robot.py._build_override_cfg` **都**显式给了 `True`，两个 dummy SAC 配置也通过 `defaults: env/realworld_cube_place@env.train` 继承。（`FrankaRobotConfig` 默认 `False`，而 `_calc_step_reward` 在 `False` 时除进区外恒返回 0.0——漏了它会让阶段 3 被喂恒零奖励。）
- 姿态围栏 0.550 rad 与 `setup_before_ray_5090.sh` 的 `RLINF_CUBE_GUARD_ORIENT_SLACK=0.20` 对得上：`√(0.01²+0.01²+0.35²) + 0.20 = 0.550`。
- `--cube-width 0.0365` 确实能到 actor（按发现 7 的机制：raylet 有 `0.046`、driver 有 `0.0365`，做差命中）。
- `_in_interpolate` 用 `getattr(..., False)` 读，`_move_action` 先于任何 `_interpolate_move` 被调用（`go_to_rest` 第 3 步）时不会 `AttributeError`。

---

#### 23.3 手册（`dmo_place_2.md`）改良清单

| # | 位置 | 原来 | 现在 |
|---|---|---|---|
| D1 | §0 状态表 | 2.7 只记了 LOG-019 的冲高 | 记两次失败，并写明第二次的根因已修、下次要带 `--cube-width 0.0365` |
| D2 | §0 状态表 | 1A「9 组不变量」 | **18 组**（实测打印行数），并点名新增的「锁存后拒绝下发」 |
| D3 | §1.2 | 没提 `setup_*.sh` 会导出权限变量 | 补上，并说明是 `${VAR:-默认}` 写法（不会覆盖你先设的值）+ 末尾两行回显 |
| D4 | §1.5 | 「围栏会在 **0.392 m** 处触发」 | 那是**旧 H1** 的数。改成几何相对表述（盒顶 +8 cm 触发、+10 cm 绝对天花板、比人早约 19 cm），并给出现 H1 的 0.3895 / 0.4095 |
| D5 | §2.5 | 「现存值」写的是旧 H1，新 H1 反而被注释掉 | 两者对调，并补 `write_cube_place_pose.py` 的拒写规则与 `--force` 会把 `calibrated` 写成 `false` |
| D6 | §2.6 期望 | hover z = 0.3119 | **0.3095**；并补 `reach:` 一行（该行本身是发现 5b 新加的） |
| D7 | §2.7 流程 | 「起始位形硬门 → `require_motion_ready`」 | 顺序反了，代码是**先模式后位形**，再加夹爪门，三道都在 Ray 之前 |
| D8 | §2.7 期望 | `motion guard armed` 格式与代码不符 | 按代码写全（`floor` / `ceiling` / `max_lag` / `orient`），并补 `motion guard confirmed` 回读、`reach:`、`rest pose reached on attempt N` |
| D9 | §2.7 期望 | `dz` 举例用旧几何（起 0.2719 → 0.3119） | 改为现几何（起 ≈0.2672 → 0.3095，`dz≈+0.042`），并加第三种失败形态：`dz` 只有 0.02 出头且每轮只进几毫米 = LOG-022 那种被夹住 |
| D10 | §3.2 | 「这些环境变量**必须**在 `ray start` 之前导出」 | 过强。改成三行对照表（`ray start` 前 export / CLI 旗标 / driver 侧 export 各自为什么能到 actor），并列出**哪些量有旗标、哪些只能用环境变量** |
| D11 | §3.2 | H1 与几何全是旧值 | 换成 YAML 现值，并附围栏坐标与 `reach` 判读 |
| D12 | §3.2 默认值表 | 缺 `safe_smoke_hold` / `use_dense_reward` | 补上，并写明「数据类默认 `False`、env 包钉 `True`」这个容易踩的差别 |
| D13 | §3.3 | 第 5 步写成 `FrankaEnv.go_to_rest` | 实际是 `_go_to_rest_pose`：为什么要替换（上游 `np.allclose` 第三参是 **rtol**，`target_y≈0` 时容差塌缩成 `atol=1e-8`，循环必跑满 3 轮）、绝对容差 0.01 m、最多 3 次、**末次之后再判一次**、三种收尾日志、env 门与脚本门是两层 |
| D14 | §3.4 A/B 表 | 围栏「刹车并抛错」 | 全面改写成「刹车 + 锁存 + env 侧轮询抛」，新增两行专讲「为什么控制器里一律不抛」和「mixin 在哪些点轮询」 |
| D15 | §6 | 两行重复且**过期**（还写着已被 C3 推翻的刹车顺序「先 `set_target(测量)` 再 `stop`」） | 删掉重复行；首行补新日志措辞；新增四行：`refusing to command motion`、`guard_check_error`、`step slew clamped` 洪流、`rest pose NOT reached`、夹爪宽度贴窗沿 |
| D16 | §7 | T2 说 `safe_smoke_hold` 默认 False | 补清：数据类默认 `False`，env 包钉 `True`，不走那个 env 包才需要担心 |
| D17 | §7 | 无 | 新增 **T15**（跳闸后的收尾路径从未在真机上走过，2.4b 若真触发 abort 要顺手核对四句日志） |
| D18 | 文末「下一个动作」 | 还停在 LOG-019 后的「现场恢复 → 2.4a → 2.4b」 | 改成 LOG-022 的下一步，并**显式承认**这是跳过 2.4b 的一个有意例外、给出理由与退回条件；§2.7 前置和 §2.0「顺序不可跳」都加了指回这个例外的交叉引用，免得手册里两处互相打架 |

---

#### 23.4 验证

**1A（franky 容器，无臂）：PASS**，现为 18 组不变量，新增两条：

```
a latched guard trip refuses further motion instead of raising
interp 2.0cm/s <= step 5.0cm/s (so skipping the slew clamp mid-interp is safe)
```

**ruff（容器内）：** 改动前后 `b/x/franky_ext` + `b/x/scripts` 的告警构成完全一致（27 CPY001 / 24 E402 / 12 I001 / 4 C408 / 2 F401 / 1 F541），**没有新增**。这些都是 `b/x/` 全目录的既有形态（缺版权头、`sys.path` 之后再 import），不是本轮引入。

**未做：** 任何真机验证。本条全部结论要么来自读代码，要么来自 1A 桌面断言。

---

#### 23.5 教训（接 LOG-022 的第 18 条）

19. **一个修复只覆盖了它的示例路径。** LOG-021 的 W4 认定「Worker 里 `raise` 到不了调用方」并让**看门狗**改成锁存——完全正确——但同一份代码里 `_abort_motion` 和 `_check_motion_guard` 仍在抛，而后者挂在**每一次** `get_state()` 上。修一个类别的 bug 时，要把这个类别在文件里**全部**列出来，不能只改触发这次讨论的那一处。这与 LOG-022 教训 16（「兜底措施也要问它在哪条路径上被调用」）是同一条，换了个方向而已。

20. **「只做一次」如果靠读一个共享变量来保证，就要问它是什么时候被写的。** 看门狗查 `_guard_trip_reason is not None` 看着很像互斥，但那个值要等刹完车才写，于是留下一个和刹车时长一样宽的窗口。检查与置位之间隔着 0.25 s 的物理动作，就必须上锁。

21. **手册里的每一个具体数字都是会过期的断言。** 这次一次性发现旧 H1 的数字散落在四节里（0.3119 / 0.392 / 0.2719 / 六元组本身），全是「重标定之后没人回来改」。凡是能用几何关系表述的（盒顶 +8 cm、接触 +0.08），就不要写死绝对值；必须写绝对值的地方，要在同一处标明它是从哪份文件的哪一次标定推出来的。

22. **两处互相矛盾的指示，比一处错误的指示更糟。** §2.7 写着「前置：2.4b 四档全过」，文末却让人直接跑 2.7。这不是笔误——是一次**真实的、有理由的例外**，但如果不把理由和退回条件写进去，读者只会挑他喜欢的那一条执行。例外要写成例外，不能靠沉默。

---

#### 23.6 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 1A | **PASS**（18 组不变量） |
| 阶段 1B | PASS（LOG-021 后两次），本轮**未重跑**——改动只在控制器/脚本的错误路径上，不碰模型与 YAML |
| 阶段 2.4b | **仍欠着**（`peak_overshoot` 从未在真实位形上测过） |
| 阶段 2.7 | 待重跑，带 `--cube-width 0.0365` |
| 机器人 | 未动。臂应仍在 LOG-022 结束位置 `0.6658, -0.0066, 0.2672`，夹着方块 |
| 代码 | `b/x/` 12 个文件（`controller_extended` / `franky_single_franka_env` / `step_cube_place_robot` / `step_cube_place_dummy` / `write_cube_place_pose` / `test_franky_controller_ext` / `diag_franky_motion` / `run_cube_place_phase2.sh`，加 franka_3 阶段的 `step2` / `step3` / `step5` / `step6`）；`rlinf/` **一行未改** |
| 手册 | 已按 §23.3 的 18 项改良 |
| 未决 | T2、T5、T6、T8、T10、T11、T12、T13、T14、**T15（新）** |

**下一步不变**（LOG-022 §22.6）：

```bash
ray stop
bash b/x/scripts/run_cube_place_phase2.sh reset --cube-width 0.0365
```

**本轮额外要看的：** 若这次出现任何 `motion guard abort`，请完整核对屏幕上是否**依次**出现这四句——它们是发现 1 那条修复链的现场证据，而这条链至今只有桌面断言（T15）：

1. `motion guard abort [...]: ... arm braked and latched`
2. `motion guard tripped and the arm was braked: ...`（**env 侧**抛的）
3. `controller tracker stopped on close()`
4. `controller health at teardown: {...}`

缺任何一句，都说明这条链在真机上还有一段没通。

---

### LOG-024 | 标定 REPL：「明明夹住了却报没夹住」 | 根因 = 宽度窗口标定值 vs 实测值不符 + Worker 异常会打掉整个进程

#### 24.1 现象

在 `test_franky_controller_ext.py` 的 `cmd>` 里执行 `close` 夹取方块，肉眼可见夹爪已经贴住方块并维持力，但命令报错退出：

```
RuntimeError: grasp did not capture the cube: measured width=0.0325m, expected 0.0460m +/-0.0120m
```

随后整个进程连带 Ray 一起退出，回不到 `cmd>`。用户提问：「分明夹住了, 为什么报错说没夹住」。

#### 24.2 根因一：判定逻辑只看测量宽度是否落在窗口内，与「有没有物理接触」无关

`b/x/franky_ext/franka_libfranka_gripper.py` 的 `close()`：闭合到位后读回 `measured width`，判定条件是

```
cube_width_m() - hold_tolerance_m() <= measured_width <= cube_width_m() + hold_tolerance_m()
```

默认 `FRANKA_CUBE_WIDTH_M=0.046`、`FRANKA_HOLD_TOL_M=0.012`，窗口是 `[0.034, 0.058]` m。这次实测 `0.0325` m 比窗口下沿还小 0.0015 m——不是空爪（空爪会显示接近夹爪全闭合的宽度，通常远小于 0.03 或直接是 0），而是**这块具体方块的实际厚度比默认假设的 0.046 m 更薄**。同一份代码在 LOG-022 标定另一块方块时也测出过 `0.0365` m，同样明显小于默认值——两次独立测量都指向同一个结论：**`0.046 m` 从来就不是任何实物方块的实测值，只是代码里的占位默认**，每次换方块或重新标定都必须用真实读数覆盖它，而不是依赖默认窗口。

#### 24.3 根因二：为什么这个异常会把整个 REPL 进程带走，而不是被本地 `try/except` 接住

`test_franky_controller_ext.py` 的 `cmd>` 循环包着一层 `try/except`，但接不住这个异常，原因和 LOG-023 发现 1 里描述的 motion-guard 场景是**同一条机制，且不局限于运动围栏**：

- `close()` 是控制器 Ray actor 上的一个远程方法调用（RPC）；
- 它在 actor 进程里抛出的 `RuntimeError` 先被 `rlinf/scheduler/worker/worker_group.py` 里的 `WorkerGroupFuncResult` 捕获，打印错误并把 `Cluster._run_failed` 置位；
- 一个后台线程检测到这个状态后向主进程发 `SIGUSR1`；
- 主进程的信号处理器对所有 actor 调用 `ray.kill`，然后 `exit(-1)`；
- 这一整条链和「是不是 motion guard 触发」**完全无关**——只要是任何一个 Ray Worker 方法抛出未捕获异常，都会走这条路。REPL 脚本里对 `cmd>` 单条命令包的 `try/except` 只能捕获**本地**抛出的异常，接不住已经在另一个进程里被 `WorkerGroupFuncResult` 处理并转化成信号的这种情况。

也就是说，LOG-023 把控制器内部（`_abort_motion` / `_check_motion_guard` / `motion_health`）改成不抛，只解决了「围栏路径」这一类调用；`franka_libfranka_gripper.close()` 这种业务校验失败仍然是**普通 `raise`**，同样会触发整条链条。这不是本次要修的 bug（校验失败确实应该让调用方知道），但**必须被文档明确记录**，否则下一次任何 Worker 方法（不管是夹爪、控制器还是别的）抛错，都会有人诧异「为什么我 `try/except` 包了还是整个炸了」。

#### 24.4 判定：不是代码 bug，是标定值需要按实物重新设置 + 一个必须补文档的行为说明

本次未改代码。核对过 `close()` 的判定逻辑本身没有问题（窗口判定是有意为之，用来防止「爪子合到底但方块其实没夹住/滑走」这种静默失败）；`RuntimeError` 会连带整个进程退出也是既有的、经 LOG-023 定性过的 Ray 行为，不是这次新引入的缺陷。需要修的是**文档**：

1. `dmo_place_2.md` §2.5 步骤 1：补充「`close` 判定看测量宽度是否落在窗口内，与物理接触无关」的说明，给出 LOG-022（0.0365）与本次（0.0325）两个实测对照，并给出「拿报错里的 `measured width` 直接定档、`open`/`close` 反复 2–3 次取中间值」的操作建议。
2. `dmo_place_2.md` §3.2 默认值表：在 `FRANKA_CUBE_WIDTH_M` 行下补注「`0.046` 只是占位默认，不是任何实物的实测值」。
3. `dmo_place_2.md` §3.4：新增一行，把 LOG-023 「为什么控制器里一律不抛」的结论**泛化**——这不是围栏专属机制，是所有 Ray Worker RPC 的通用行为；夹爪 `close()` 的 `RuntimeError` 是另一个具体例子。
4. `dmo_place_2.md` §6：改写 `grasp did not capture the cube` 一行，说明「不一定是空爪，先看报出来的 `measured width` 数值判断是标定值不对还是真的没夹住」；新增一行专讲「REPL/命令报错后整个进程退出、回不到 `cmd>`」这一通用症状及其处理方式（重启，不要期待本地异常处理能接住）。

#### 24.5 教训（接 LOG-023 第 22 条）

23. **一个「为什么会崩」的解释，如果只讲了触发这次讨论的那条调用路径，就还没讲完。** LOG-023 把 Worker 异常摧毁整个进程这件事，写成了「围栏为什么不抛」的专属背景板；这次同一机制在夹爪校验上原样重演，才发现手册里从没有把它当成**通用规则**写出来。任何涉及「Ray actor 方法里 raise 会怎样」的解释，都要在第一次写的时候就点明它对**所有**远程方法成立，而不是只在每次踩到新的具体方法时补一次。

24. **默认值是给「没有更好信息时」用的，不是给「这次」用的。** `FRANKA_CUBE_WIDTH_M=0.046` 这种带物理量纲的默认值，只要下游有校验会拿它做判定阈值，就必然会在第一次遇到具体实物时报错——这不是异常情况，是这类默认值的正常使用模式的一部分，应当在标定步骤里**预先**提示，而不是等用户报错后才解释。

#### 24.6 状态与下一步

| 项 | 状态 |
|---|---|
| 本次是否改代码 | 否，`close()` 判定逻辑保持不变（有意为之的静默失败防护） |
| 本次改文档 | `dmo_place_2.md` §2.5 / §3.2 / §3.4 / §6，共 4 处 |
| 机器人 | 未涉及新的真机动作，状态延续 LOG-022 结束时 |
| 后续标定建议 | 下次执行 §2.5 时，`close` 报错不要当成失败重来——直接用报出的 `measured width` 定 `FRANKA_CUBE_WIDTH_M`，多测 2–3 次取中间值，再继续走 H1 标定后续步骤 |
| 未决 | 与 LOG-022/023 相同：T2、T5、T6、T8、T10、T11、T12、T13、T14、T15；阶段 2.4b 仍欠着；阶段 2.7 待重跑（`--cube-width 0.0365`，见 LOG-022/023） |

---

### LOG-025 | `write_cube_place_pose.py` 吃掉负数科学计数法参数 | 根因 = argparse 的「像负数」正则不认指数记法

#### 25.1 现象

`getpos_euler` 读出的 roll 接近 −π，按 numpy 的默认格式化打印成科学计数法：

```bash
python b/x/scripts/write_cube_place_pose.py  7.26857066e-01  2.49468647e-02  2.50846744e-01 -3.13747483e+00 3.54833569e-02 -2.68804519e-03
# write_cube_place_pose.py: error: the following arguments are required: pitch, yaw
```

6 个位置参数原样传了 6 个，报错却说少了 `pitch, yaw`。

#### 25.2 根因

`argparse` 判断一个以 `-` 开头的 token 到底是「未知选项」还是「负数（应该当位置参数吃）」，靠的是内置的 `_negative_number_matcher`：

```python
_negative_number_matcher = re.compile(r'^-\d+$|^-\d*\.\d+$')
```

这个正则只认得纯小数形式的负数（`-3`、`-3.14`），**不认指数记法**（`-3.13747483e+00`、`-2.68804519e-03`）。而这个解析器没有任何形如 `-1` 的短选项，所以本来所有负数都应该被当成位置参数——但 `-3.13747483e+00` 和 `-2.68804519e-03` 因为带指数，匹配不上这条正则，于是被 argparse 当成「看起来像未知选项」的 token 处理，打乱了位置参数的分组，导致 `roll`/`yaw` 这两个负值把后面的参数顶飞，`pitch`、`yaw` 收不到值。

用最小复现验证：

```python
import argparse
p = argparse.ArgumentParser()
for n in ["x","y","z","roll","pitch","yaw"]:
    p.add_argument(n, type=float)
p.parse_args(["7.26857066e-01","2.49468647e-02","2.50846744e-01",
              "-3.13747483e+00","3.54833569e-02","-2.68804519e-03"])
# error: the following arguments are required: pitch, yaw
```

这不是偶发：**roll 只要接近 −π（H1 标定要求的姿态，见 §2.5 的检查项 `ROLL_FROM_PI_TOL_RAD`），且数值大到 numpy/Python 的 `repr` 切换成科学计数法，这个命令就必炸**，跟具体是哪个方块、哪次标定无关。

#### 25.3 修复

`b/x/scripts/write_cube_place_pose.py`：在构造 `ArgumentParser` 之后，把 `parser._negative_number_matcher` 换成一条同时接受指数记法的正则：

```python
_NEGATIVE_NUMBER_RE = re.compile(r"^-\d+\.?\d*(?:[eE][+-]?\d+)?$")
...
parser._negative_number_matcher = _NEGATIVE_NUMBER_RE
```

这个解析器里没有任何 `-x` 形式的短选项（`--path` / `--notes` / `--force` / `--no-backup` 都是长选项），所以放宽这条匹配是安全的——不会有真正的选项被误判成数字。

修复后用原样报错的那一行命令重新跑（写到 `/tmp` 避免覆盖真实标定），六个值全部正确落位、无告警之外的错误：

```
target_ee_pose: ['0.726857', '0.024947', '0.250847', '-3.137475', '0.035483', '-0.002688']
```

`_negative_number_matcher` 是 `argparse` 的私有属性（下划线开头），CPython 各版本一直存在且行为稳定，但严格来说是内部实现细节；已在代码里加注释说明为什么这么做、以及为什么在这个解析器上安全。

#### 25.4 教训

25. **命令行工具的参数格式假设，要覆盖「数据实际长什么样」，不能只覆盖「测试时手打的那几个数」。** 这个脚本上线时大概率只用手动敲的、位数不多的小数测过（`0.706`、`-3.116` 这种），从没喂过 `getpos_euler` 在极端值下真实吐出来的科学计数法字符串。**输入格式的边界要按「上游实际产出」去试，而不是按「看起来像负数」的直觉去试。**

#### 25.5 状态

| 项 | 状态 |
|---|---|
| 代码改动 | `b/x/scripts/write_cube_place_pose.py`：加 `import re` + `_NEGATIVE_NUMBER_RE` + 挂到 `parser._negative_number_matcher` |
| 验证 | 最小复现（纯 argparse）确认根因；修复后用原始报错命令重跑（`--path /tmp/...` 避免覆盖真实标定）成功落盘且六元组数值正确；`ReadLints` 无告警 |
| 影响面 | 只影响这一个脚本的命令行解析，不影响 `_check`/写盘逻辑本身 |
| 后续提醒 | 无论 roll/pitch/yaw 是否恰好落在会触发科学计数法的范围，`write_cube_place_pose.py` 现在都能正确解析；不需要用户手动把 `e+00` 展开成小数 |

---

### LOG-026 | 阶段 2.7 + 2.8 一次跑通（`box`） | 逐条门闩核对 + 手册阶段 3 章节细化

#### 26.1 输入

用户在 LOG-025 修好标定脚本、按新的 H1 重标定之后，直接跑了 `box`（它内含完整的 2.7 `reset`），并问「下面日志是否正常」。

```bash
bash b/x/scripts/run_cube_place_phase2.sh box
```

单容器、单节点（`1 node and 0 accelerator`、`python_interpreter_path: /opt/venv/franky-0.19.0/bin/python`），`safe_hold=True`。**结论：正常，而且这是阶段 2.7 与 2.8 的第一次通过。**

#### 26.2 逐条门闩核对（对照手册 §2.9）

| 门闩 | 日志证据 | 判定 |
|---|---|---|
| 2.4-0 力上限 | `authority: spring force <= 20.0N/axis (34.6N worst-case 3-axis) [K_t=2000 x clip=0.0100m]; torque <= 6.00Nm/axis`；`collision behavior tightened: ... [40.0×3, 12.0×3] (was [100.0×3, 25.0×3])` | PASS（**不是** LOG-019 的 100 N） |
| 2.6-1 只读几何 | `reset_ee_pose hover: [0.7269, 0.0249, 0.3308]` = H1 + 0.08；`ee_pose_limit_min/max` xy 半宽 0.05、z 下沿 −0.005、上沿 +0.08 | PASS |
| 2.6-2 起始位形 + 夹爪 | `probed - target xyz = [-0.0038, 0.0002, 0.0764]`（在标记上方 7.6 cm，盒内）；`robot_mode: RobotMode.Idle has_errors: False`；`gripper: holding=True width=0.0315786749124527`；无 `start-pose note` | PASS |
| 2.7-1 / 2.7-2 Gym 与 wrapper | `creating FrankyCubePlaceEnv-v1`；`wrapper stack: OrderEnforcing -> PassiveEnvChecker -> Quat2EulerWrapper -> GripperCloseEnv -> FrankyCubePlaceEnv` | PASS |
| 2.7-2b 围栏已装 | `motion guard armed: xyz in [[0.6269,-0.0751,0.2358],[0.8269,0.1249,0.4108]] ... ceiling 0.4308`，随后 `motion guard confirmed:`（回读），`controller health` 里 `guard_enabled: True`、`watchdog_alive: True`、`guard_tripped: None`、`guard_check_error: None` | PASS |
| 2.7-3 悬停几何 | `hover check: |xy-target|=0.0046m |z-hover|=0.0019m`（门是 0.03 / 0.025） | PASS，余量很大 |
| 2.7-3b 不超程 | 全程无 `motion guard abort`、无 `step slew clamped`、无 `cartesian impedance tracking stopped` | PASS |
| 2.7-4 不张爪 | `gripper_open after reset: False`；全程 `gripper_position` 稳定在 0.03158 | PASS |
| 2.8-1 盒子 | 零动作三步 TCP 漂移 ≤1.2 mm；下探三步 `approach dz=-0.0106m`（z 0.3289 → 0.3163，仍远高于 z 下沿 0.2458）；`box-steps PASS` | PASS |
| 2.8-2 退出 | `controller tracker stopped on close()`；脚本正常结束 | PASS |

#### 26.3 值得记下的四个细节

1. **`rest pose reached on attempt 2` 也算过。** 第一段是 `+z=0.030` 的抬升（`interpolate_move: 0.0308m in 1.50s would be 2.1cm/s; stretching to 1.54s to respect the 2.0cm/s cap` —— 限速在这里是**拉长时间**而不是提速），第二段补残差 `err=0.0241m`，收敛到 `err=0.0050m <= 0.0100m`。手册里原来把「attempt 1」当成期望值，实际两段是正常的。
2. **`dz=0.0016` 不是失败。** 起点 z=0.3272 已经离悬停 0.3308 只差 3.6 mm，所以净变化必然很小。手册原来写「应约 +0.042」，那是按 LOG-022 的起点算的——**这类「期望值」必须写成几何关系（终点落在 `hover ± 0.025`），不能写成某次实测的数字**（同教训 21）。
3. **实测方块宽度第三次不同：0.0316 m**（LOG-022 是 0.0365，LOG-024 是 0.0325）。窗口 `cube_width=0.0325 +/- 0.0150` 覆盖住了。三次读数散布约 5 mm，说明夹取角度的影响比想象大，`FRANKA_HOLD_TOL_M=0.015` 是合理档位。
4. **`reach` 现在两处都是 `NEAR-SINGULAR`**：`worst box corner r=0.785m (92%)`、`worst fence corner r=0.842m (98%)`。新 H1 的 x=0.7269 比 LOG-022 的 0.6713 更靠外，所以 T5（伸展位形权限）比上一次更紧，不是更松。已在手册 §3.2 改正原先「新 H1 更靠近基座、对 T5 是改善」的说法。

无害警告三条，均已在手册 §6 有行：`/dev/shm has only 67108864 bytes`（当前容器仍是旧 `--shm-size` 起的）、gymnasium passive checker 的 `float64` / `not within the observation space`、`env.close()` 的 `AttributeError: 'VideoPlayer' object has no attribute 'stop'`（来自 `FrankaEnv.close` 里的 `camera_player.stop()`）。

#### 26.4 阶段 3 章节细化：调研结论

用户要求把手册的「阶段 3 — 在线 SAC」从 7 条纲要细化到能指导工程师逐步操作，并以 RLinf 自带的 charger 例子为参照。先与用户确认了两个形态选择：**双容器双 Ray 节点** 与 **第一版就带相机**。随后逐项对代码核实，得到以下结论（都写进了新章节）：

| # | 结论 | 依据 |
|---|---|---|
| 1 | charger 的 `cluster` 写法可直接照搬：`component_placement` 用 `{node_group, placement}` 形式，`node_groups` 里 `4090`(rank 0) / `franka`(rank 1) 各带 `python_interpreter_path` | `examples/embodiment/config/realworld_charger_sac_cnn_async.yaml` L16-43 |
| 2 | 必须双容器：franky venv 是 `torch==2.5.1+cpu` 且容器无 `--gpus`；GPU 容器无 franky/libfranka | `b/x/scripts/step7_install_deps.sh` |
| 3 | **环境变量转发会反向覆盖机器人侧**：优先级为「各节点 ray start 前的环境 < head 侧 ray start 之后的改动（广播到所有节点）< YAML `env_configs.env_vars`」。具体地雷：`setup_before_ray_gpu_5090.sh` 有 `RLINF_SKIP_CAMERA:-1`，而启动脚本 source 它发生在 `ray start` 之后 → 会把 franky 节点的相机关掉。所以 `RLINF_SKIP_CAMERA` / `FRANKA_CUBE_WIDTH_M` 必须写进 franky 组的 `env_configs.env_vars` | `rlinf/scheduler/cluster/node.py` L388-421（docstring 列了三级优先级）、`b/x/configs/setup_before_ray_gpu_5090.sh` L11 |
| 4 | 两容器同 IP 不冲突（按 `RLINF_NODE_RANK` 排序、按 Ray `NodeID` 钉节点），但 rank 必须**唯一且从 0 连续**，否则直接 assert；且要么所有节点都设 rank，要么都不设 | `rlinf/scheduler/cluster/node.py` `_sort_nodes` L428-486 |
| 5 | `hardware: type: Franka` 的枚举只做 `icmplib.ping` + 相机 SDK/serial 校验，**不碰 1337**；`disable_validate: true` 会跳过 | `rlinf/scheduler/hardware/robots/franka.py` L98-144 |
| 6 | `RLINF_SKIP_CAMERA=1` 时返回的是**全零 stub 帧**（不是缺键、不报错），所以「训练在跑但策略看黑图」不会自己暴露 → 相机验收必须前置 | `b/x/franky_ext/franky_single_franka_env.py` L62-63、L395-407 |
| 7 | 单相机的默认名就是 `wrist_1`（`wrist_{i}`，i 从 1），`camera_names` 只是把它写明 | `rlinf/envs/realworld/franka/franka_env.py` `_build_camera_infos` |
| 8 | **不要在 YAML 写 `reset_ee_pose` / `ee_pose_limit_*`**：`PegInsertionConfig.__post_init__` 会从 `target_ee_pose + clip_*` 重新推导并覆盖 | `rlinf/envs/realworld/franka/tasks/peg_insertion_env.py` L82-106、`b/x/franky_ext/tasks/cube_place.py` L67-76 |
| 9 | 训练时 `enable_random_reset` 继承上游默认 **True**（env 包未覆盖），reset 的 xy/yaw 会带 `random_*` 抖动——与烟测（显式 False）不同 | 同上 |
| 10 | `action_scale[0]` 被 `RLINF_CUBE_STEP_SPEED/step_frequency` 夹到 5 mm/步，那句 `clamping to 0.0050` 是正常信息 | `b/x/franky_ext/tasks/cube_place.py` |
| 11 | **训练路径没有 Ray 前硬门**：`tcp_probe` 的三道门只在烟测/诊断脚本里，env 侧只剩 `_check_start_pose` 的 warning → pre-flight 只能靠先跑一次 `run_cube_place_phase2.sh connect` | `b/x/scripts/step_cube_place_robot.py` vs `examples/embodiment/train_async.py` |
| 12 | `step_cube_place_robot.py` **无条件**写 `RLINF_SKIP_CAMERA=1`，所以它现在不能用来验相机 → 需要一个 `--with-camera` 旗标（列为第四件待创建物） | 同上 |
| 13 | 指标命名：`env/{reward,return,episode_len,success_once,intervened_once,intervened_steps,success_no_intervened}`、`train/sac/{critic,actor,alpha}_loss`、`train/sac/alpha`、`train/actor/*`、`train/replay_buffer/*`、`train/replay_channel_qsize`、`time/*` | `rlinf/envs/realworld/realworld_env.py` L145-192、`rlinf/workers/actor/fsdp_sac_policy_worker.py` L644-670、`rlinf/runners/async_embodied_runner.py` L205-292 |
| 14 | embodied runner **不支持** `resume_dir: auto`，必须给 `global_step_<N>` 完整路径；恢复内容含 actor/optimizer/alpha/target/replay buffer | `rlinf/runners/embodied_runner.py` L175-185、`fsdp_sac_policy_worker.py` L803-845 |
| 15 | `save_interval` 照抄 charger 的 `-1` 是危险的（真机会被急停打断）→ 定为 50 | charger YAML `runner` 节 |

#### 26.5 文档改动清单

`dmo_place_2.md`：

- **`### 阶段 3` 全节重写**，扩为 §S3.0–§S3.10（写作时把上面第 3 条的地雷同时写进启动脚本正文：`source setup_before_ray_gpu_5090.sh` 之后补一句 `export RLINF_SKIP_CAMERA=0` 作第二层防线）：与 charger 的照抄/必改对照表、四件待创建物的**完整可抄正文**（训练 YAML、启动脚本、相机 YAML、`--with-camera` 旗标）、双容器拓扑图与不可换的启动顺序、相机验收（8a/8c + cube-place 链路自检）、开训前 pre-flight、启动后前五分钟的八条期望、SpaceMouse 介入语义、掉块/急停/续训、指标表、验收门闩 3-0…3-6、已核实的七个坑。
- **小节号加 `S` 前缀**（`S3.x`）：因为第 3 章已经占用 `§3.1`–`§3.4`，而文档里大量引用「§3.2」「§3.4」指的是第 3 章。§0 加了一句说明。
- §0 状态表：2.7 与 2.8 由 FAIL/未做 改为 **PASS（LOG-026）**；2.4b 明确为「不再是放行门闩，但欠账仍在」；阶段 3 改为「流程已细化、四个文件待创建」。
- §3.2：H1 六元组、hover z、`ee_pose_limit_*`、围栏与 `reach` 百分比全部换成本次实测；删掉「新 H1 更靠近基座、对 T5 是改善」的错误结论。
- §2.5：`getpos_euler` 的科学计数法可直接粘贴（LOG-025）；旧两组 H1 保留为对照。
- §2.6 / §2.7 期望值：hover z 0.3095 → 0.3308；`reach` 86% → 92%（且已带 `NEAR-SINGULAR`）；围栏盒改为本次数值；`--cube-width 0.0365` 改为「用实测值」；`attempt 1` 补上「attempt 2 也算过」；`dz` 的判据改为几何关系而非固定数字；删掉「2.7 的例外框」。
- §2.4b 段落与 §7 T2/T6：改为「欠账仍在」与「正文/步骤已备，见 §S3.1 / §S3.3」。
- §8 源码索引新增 5 行（`train_async.py` / `run_realworld_async.sh`、异步 runner、SAC actor、cluster config + node、Franka 硬件校验）。
- 文末「下一个动作」：从「重跑 2.7」改为「阶段 3 第一步 = 相机验收（§S3.3）」，并给出可直接执行的命令块。

代码：**本轮无改动**（唯一「该改」的是 `step_cube_place_robot.py --with-camera`，按 T2 与其余三个文件一起做）。

#### 26.6 教训（接 LOG-025 第 25 条）

26. **「期望输出」写成上一次的实测数字，就会在下一次误报失败。** §2.7 的 `dz≈+0.042` 是按 LOG-022 结束时的臂高算的；这次臂停在离悬停 3.6 mm 处，`dz=0.0016` 完全正确却长得像 §1.4 的「哑失败」。**凡是依赖初始条件的量，期望值只能写成关系式**（「终点落在 `hover ± 0.025`」），把具体数字降格为举例并注明来自哪一次。同理适用于 `--cube-width 0.0365` 这种「上次的实测值」被写成命令行默认动作。

27. **同一份文档里两套同号小节是引用事故的温床。** 第 3 章的 `§3.1`–`§3.4` 与阶段 3 的 `3.0`–`3.10` 撞号，而文档里已有十几处「见 §3.4」指的是前者。发现得早只花一次改名（`S3.x`）；发现得晚就是读者按错的章节去操作机器人。**新增小节前先扫一遍现有编号空间。**

#### 26.7 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 | 2.4（部分）/ 2.5 / 2.6 / 2.7 / 2.8 全 PASS；**2.4b 仍欠着**（`peak_overshoot` 至今无真机数据，T5/T11） |
| 机器人 | 结束时 TCP `[0.7194, 0.0250, 0.3134]`（标记上方约 6.3 cm）、夹着方块、tracker 已 `stop()`、FCI 已释放 |
| 本轮代码改动 | 无 |
| 本轮文档改动 | `dmo_place_2.md`（阶段 3 全节重写 + 9 处一致性修正）、本条 LOG-026 |
| 下一步 | 阶段 3 第一步：相机验收（手册 §S3.3 的 8a → 8c）。之后建 §S3.1 的四个文件、按 §S3.2 起双容器双节点集群、按 §S3.4 做 pre-flight |
| 未决 | T2（正文已备，文件未建）、T5、T6（步骤已备）、T8、T10、T11、T12、T13、T14、T15 |

---

### LOG-027：把 §S3.1 的四件东西从文档正文落成真实文件

**上下文与方法**：用户在 LOG-026 把阶段 3 的正文写完之后，明确要求「根据 §阶段 3 进行编码吧」——把 §S3.1 描述的四个「待创建物」真正建出来，而不是只留在文档里。这四个文件此前只以 markdown 代码块的形式存在于 `dmo_place_2.md`，本轮把它们原样（或按静态检查纠正后）写入仓库，并做了能在**没有 franky/GPU venv 的开发环境里**做到的最大限度验证：YAML `yaml.safe_load` 解析、脚本 `bash -n` 语法检查、Python `py_compile` + `ReadLints`。**没有**做的：任何需要真机、双容器 Ray 集群、CUDA/franky venv 的端到端验证——那些留给 §S3.2–§S3.4 描述的手工验收流程。

**改动 1／4：`b/x/configs/realworld_cube_place_sac.yaml`（新建）**

内容与 §S3.1 ① 的正文一致，另外做了两处比文档草稿更完整的补强：

- 文件顶部加了三条 `EDIT ME` 注释（GPU 容器 `python_interpreter_path`、`FRANKA_CUBE_WIDTH_M`、`camera_serials`），因为原文档正文里这三处是硬编码的「示例值」（来自 LOG-026 的当次实测），直接照抄到别的机器上会是错的标定值，而不是「换成你自己的」。
- `env.eval.override_cfg` 没有像文档草稿写的 `{ }`（「照抄 train 的那一份」），而是把 train 的完整 `override_cfg`（含 H1、`clip_*`、相机字段）原样复制过去——因为 Hydra 的合并语义下 `{ }` 不会自动继承 `env.train` 分支的值，写 `{ }` 会让 `env.eval` 退回 dataclass 默认值（`is_dummy: True`），与文档本意（“同一条臂、同一个标记”）相反。`val_check_interval: -1` 时这份配置不会被用到，但错误的默认值一旦哪天 `val_check_interval` 被打开，就是一个访问 `is_dummy=True` 的评估工位，静默地什么都不做。
- 额外把 `camera_type` / `camera_serials` / `camera_names` 也写进了 `env.train/eval.override_cfg`（文档正文的草稿只在 `cluster.node_groups.franky.hardware.configs` 里写了相机字段）。`hardware.configs` 里的相机信息是给 `FrankaHWInfo`／硬件校验用的；真正决定 env 内部 `_build_camera_infos` 用哪些 serial、叫什么名字的是 `FrankaRobotConfig.camera_serials/camera_type/camera_names`，也就是 `override_cfg` 里的同名字段（见 `rlinf/envs/realworld/franka/franka_env.py` `_build_camera_infos`，L660-670）。只写在 `hardware.configs` 里，env 会因为 `override_cfg` 没给这些字段而退回 dataclass 默认（`camera_serials=None`，见 `FrankaRobotConfig`），`_build_camera_infos` 直接 `return []`——相当于又造出一个新的「相机字段名字对但没接上」的坑，正是 §S3.10 想避免的那一类问题。

静态验证：

```
$ python3 -c "import yaml; yaml.safe_load(open('b/x/configs/realworld_cube_place_sac.yaml'))"
parsed OK, top keys: ['defaults', 'hydra', 'cluster', 'runner', 'algorithm', 'env', 'rollout', 'actor', 'reward', 'critic']
cluster.node_groups labels: ['gpu', 'franky']
```

**改动 2／4：`b/x/scripts/run_cube_place_sac.sh`（新建，`chmod +x`）**

与 §S3.1 ② 的正文基本一致，额外照抄了 `run_cube_place_dummy_sac_gpu.sh` 里两个本文档草稿没写但显然该有的安全检查（这两条不是新决定，是把已有脚本的安全性搬过来，避免阶段 3 的脚本比阶段 1B 的还弱）：

- resnet 权重存在性检查（`RLINF_RESNET10_PATH/resnet10_pretrained.pt`）与 CUDA 可用性检查，放在 `ray status` 检查之前——失败要快，不要等到 Ray/Hydra 起来一半才报错。
- 起训练前检查是否已有 `state=ALIVE` 的 `FrankyControllerExtended` actor，如果有就拒绝启动（见该脚本内注释：一个正在 1 kHz 力矩控制中的 actor 被粗暴打断，得到的是 libfranka 的通信超时急停，而不是 `freeze_at_current()` 的受控减速）。dummy 脚本这条检查是在**自己 `ray start` 之前**用来决定「要不要先 `ray stop --force`」；阶段 3 脚本不 `ray start`，所以这里改成单纯的「检测到就拒绝启动，不做 `ray stop`」——因为集群是否要停、什么时候停，阶段 3 里是操作者在另一个终端手工决定的事（§S3.2），这个脚本不该替他做这个决定。

语法与权限验证：

```
$ bash -n b/x/scripts/run_cube_place_sac.sh && echo "syntax OK"
syntax OK
$ ls -l b/x/scripts/run_cube_place_sac.sh
-rwxrwxr-x 1 nvidia nvidia 4357 ... run_cube_place_sac.sh
```

**改动 3／4：`b/x/configs/realworld_cube_place_camera.yaml`（新建）**

按 §S3.1 ③ 的说明写：以 `realworld_franky_camera.yaml` 为骨架，`init_params.id` 换成 `FrankyCubePlaceEnv-v1`，`override_cfg` 里补 `is_dummy: false` + H1 六元组 + `clip_*`（`CubePlaceConfig.__post_init__` 继承自 `PegInsertionConfig`，会从这些值推导 `reset_ee_pose` / `ee_pose_limit_*`，所以给了这些就是可构造的真机配置——这一点与 `realworld_franky_camera.yaml` 注释里解释的 `FrankySingleFrankaEnvConfig`「不推导，必须显式给」不同，是两条环境线的既有差异，不是本轮引入的不一致）。`env.train` 和 `env.eval` 都写了同一份，而不是像旧版 `realworld_franky_camera.yaml` 那样只写 `env.eval`——因为这份文件的第二个用途（文档里写的）是被 `run_cube_place_phase2.sh reset --with-camera` 一类走 `env.train` 分支的脚本引用，只写 `env.eval` 会让那条路径读到默认值。

静态验证：

```
$ python3 -c "import yaml; d=yaml.safe_load(open('b/x/configs/realworld_cube_place_camera.yaml')); print(d['env']['train']['override_cfg']['target_ee_pose'])"
parsed OK
[0.726857066, 0.0249468647, 0.250846744, -3.13747483, 0.0354833569, -0.00268804519]
```

**改动 4／4：`step_cube_place_robot.py` 加 `--with-camera`（编辑既有文件）**

比文档草稿描述得更细，因为写代码时发现「置 0、塞 serial」这句话背后有三个需要落地的决定：

1. **serial 从哪来。** 没有直接写死 §S3.3 的那个 serial，而是复用 `step8_test_env_camera.py` 已经用的 `b/x/configs/camera_detected.json`（Step 8a 的输出）作为默认来源，新增 `--camera-serials`（可传多个）/ `--camera-type` 允许显式覆盖、`--camera-json` 换路径。理由：这个脚本本来就是「H1 标定 + 抓取校准」之后的**冷启动**脚本，不应该要求维护者提前记住/硬编码 serial；而 8a 已经把 serial 写进了 JSON，复用它比新增一个命令行必填参数更符合"§S3.3 是按顺序做"的假设。解析到的 serial 会用 `step8_checks.is_placeholder_serial` 拒绝占位符（同 `step8_check_yaml.py` 的检查逻辑），避免「忘了跑 8a、拿着 `SERIAL1` 就去连硬件」。
2. **camera_names 怎么给。** `_build_override_cfg` 新增 `camera_serials` / `camera_type` 两个可选参数：给了 serial 就同时生成 `camera_names={serial: f"wrist_{i}"}`（`i` 从 1）。这与 `franka_env.py` `_build_camera_infos` 的默认命名规则完全一致（未显式给 `camera_names` 时它自己也会退回 `wrist_{i}`），显式写出来是为了让 `obs['frames']` 里的键名不依赖 `camera_serials` 列表的枚举顺序这种隐式细节。
3. **验完怎么报。** 新增 `_print_frame_stats(obs)`：`reset()` 之后遍历 `obs['frames']`，打印每路 `shape/dtype/min/max/mean`，`max<=0` 的额外标 `ALL-ZERO (stub or capture failure)`。**故意不做断言**（不像 `step8_test_env_camera.py` 的 `_check_frame` 那样在校验失败时抛异常）：这个脚本此时已经真的移动过机械臂、正处在 `finally: _close_env` 敏感的收尾阶段，让相机诊断信息去中断一个已经复位成功的运行没有意义——全零帧本身就是最有用的诊断结果，交给操作者判断即可。

修改点：模块级新增 `import json`、`from pathlib import Path`、`from step8_checks import is_placeholder_serial`（新增一行 `sys.path.insert(0, os.path.dirname(__file__))` 使其可导入，因为原有的两条 `sys.path.insert` 只到 `<repo>` 和 `<repo>/b/x`，不含 `b/x/scripts` 本身）；`_build_hardware` / `_build_override_cfg` 各新增 `camera_serials` / `camera_type` 关键字参数（默认值维持旧行为：`["000000000000"]` 占位符，`RLINF_SKIP_CAMERA=1` 时这个值从不会被真正打开）；`parse_args` 新增 `--with-camera` / `--camera-serials` / `--camera-type` / `--camera-json` 四个参数；`main()` 按 `args.with_camera` 决定 `RLINF_SKIP_CAMERA` 是 `"0"` 还是 `"1"`，并在开头打印一行汇总（含 serial 列表）；`_run_env` 在 `env.reset()` 之后、`time.sleep(0.5)` 之前调用 `_print_frame_stats`（放在 settle 之前，因为诊断的是 reset 落地那一刻的帧，不是之后又轮询到的新帧）。

静态验证：

```
$ python3 -m py_compile b/x/scripts/step_cube_place_robot.py && echo "compiles OK"
compiles OK
```

`ReadLints` 只报告 `gymnasium` / `ray` 导入无法解析（本机没有 franky/GPU venv，这两个警告在改动前就存在，`step8_checks` 的新导入没有引入新的 lint）。

**做了但值得单独指出的交叉检查**：`camera_names` 字段确实被 `FrankaRobotConfig` 接受（`rlinf/envs/realworld/franka/franka_env.py` L49、L665-670），且 `FrankaConfig`（硬件层，`rlinf/scheduler/hardware/robots/franka.py` L209-223）确实有 `camera_type` 字段——这两点是新代码能否工作的关键假设，写代码前专门读源码确认过，不是照抄文档就假定成立。

**未做，留给真机阶段**：

- 三个 YAML/脚本里的 `EDIT ME` 三处占位值（`python_interpreter_path`、`FRANKA_CUBE_WIDTH_M`、`camera_serials`）——这些本来就该用当次实测值，不能在没有真机的环境里替维护者填。
- `step_cube_place_robot.py --with-camera` 从未真正连过相机；`_print_frame_stats` 的字段名、`obs['frames']` 的 key 是否真如预期，要等 §S3.3 第一次实跑才能确认。
- `run_cube_place_sac.sh` 里 `ray list actors --filter ...` 的输出格式假设（沿用自 dummy 脚本，本轮未改动这部分逻辑，只是保留）。
- Hydra 组合校验（`train_async.py --config-name realworld_cube_place_sac --cfg job` 之类）——本机没有 hydra/omegaconf，只做了 YAML 层面的语法检查，`ClusterConfig` / `PegInsertionConfig` 等 dataclass 校验（字段名拼写、类型）要等真机环境跑一次才能发现。

**教训**：

28. **文档草稿里的“照抄 train 的那一份”“见 §xx”这类占位描述，落成真实文件时必须替换成具体值，否则会把文档层面的省略带进可执行配置。** 本轮 `env.eval.override_cfg: { }` 就是一个例子：文档正文写 `{ }` 是为了省篇幅，但 Hydra 合并语义下 `{ }` 不等于「继承 train」，字面照抄会做出一个「eval 配置存在但 `is_dummy=True`」的哑弹。**把文档转成代码时，每一处“同上”都要重新推一遍字面语义**，而不是当成合法的 YAML 语法抄过去。

29. **相机相关字段有两处容易漏写其中一处：`cluster.node_groups.*.hardware.configs`（硬件校验/连接用）和 `env.*.override_cfg`（env 内部真正建相机用）。** 只写前者会通过硬件校验、构造出的 env 却拿不到任何相机（`_build_camera_infos` 因 `camera_serials is None` 直接 `return []`），且没有任何报错——又是一个"配置存在但没接上"的静默故障，模式与 §S3.10 第 6 条（`RLINF_SKIP_CAMERA` 的 stub 帧）相同：这条产品线里，"相机没插对"从不报错，只会安静地返回空/全零。

#### 27.1 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 | 不变，见 26.7 |
| 阶段 3 | §S3.1 的四个文件**已创建**（`realworld_cube_place_sac.yaml`、`run_cube_place_sac.sh`、`realworld_cube_place_camera.yaml`、`step_cube_place_robot.py --with-camera`），**均未在真机/双容器集群上跑过**；三处 `EDIT ME` 待按当次实测值填 |
| 机器人 | 本轮无机器人操作（无 shell 访问真机环境，纯静态编码+校验） |
| 本轮代码改动 | 新增 2 个 YAML、1 个 shell 脚本；编辑 `step_cube_place_robot.py`（+445/-37 行，含新增 `--with-camera` 及其三个辅助旗标、`_print_frame_stats`、`_resolve_camera_serials`、`_load_camera_serials`） |
| 本轮文档改动 | `dmo_place_2.md`（§0 状态表、§S3.1 前言与④正文、§7 T2/T6、文末"下一个动作"）、本条 LOG-027 |
| 下一步 | 在真机上按 §S3.3 走相机验收（先跑 `step_cube_place_robot.py --reset-only --with-camera`），通过后填 `EDIT ME` 三处，再按 §S3.2 起双容器集群 |
| 未决 | T2（文件已建、真机未验）、T5、T6（工具已备、真机未验）、T8、T10、T11、T12、T13、T14、T15 |

---

### LOG-028：给 `dmo_place_2.md` 的所有命令块标注执行环境

**上下文与方法**：用户指着 §S3.2 的启动块（`docker_run_franky_5090.sh` 与 `export RLINF_NODE_RANK=1` 写在同一个 ```bash 块里）问「这段是在宿主机上执行还是容器里执行，请标注清楚。其它的命令或代码也是」。这是一个真实的歧义：§S3.2 的块里第一行 `bash b/x/configs/docker_run_franky_5090.sh` 必须在**宿主机**跑（它会 `docker run` 进容器），而紧跟其后的 `export` / `source` / `ray start` 必须在**容器内**跑——同一个代码块里混了两种环境，读者照着从上往下敲会在宿主机上 `export RLINF_NODE_RANK=1`，然后 `ray start` 找不到 `ray` 命令。

**改动**（全部在 `dmo_place_2.md`，纯文档，无代码改动）：

1. **§0「怎么用这份文档」新增一段「命令的执行环境」**：定义了本文档只有三种执行环境（宿主机 / franky 容器 / GPU 容器）、各自跑什么类型的命令、每个新 shell 要 source 哪个 setup 脚本，并明确说「每个 bash 代码块的第一行注释都会标」。
2. **给全部 21 个 ```bash 代码块的第一行加了环境注释**：
   - 宿主机：阶段 0 的 `ping`/`ss`/`docker ps` 块、`docker exec` / `docker_run_franky_5090.sh` 两个块、1B 的 `docker run` 块、§S3.8 的 `tensorboard` 块。
   - franky 容器：§1.4 的 `diag --probe`、阶段 0 的进容器初始化块、1A、2.4a、2.4b、2.5（两处）、2.6、2.7、2.8、§S3.1 ④ 的 `--with-camera` 示例、§S3.3 的 8a 和 8c、文末「下一个动作」的两个块。
   - GPU 容器：§S3.5 的 `run_cube_place_sac.sh`。
   - §S3.2 的双容器启动块（用户指出的那一处）改成了**块内逐段标注**：`# ↓ 宿主机：创建并进入 franky 容器` / `# ↓ franky 容器内（上面那条命令进来之后，或 docker exec ...）` / `# ↓ 宿主机：创建并进入 GPU 容器` / `# ↓ GPU 容器内`，因为这一块本身就是「两种环境交替」的最典型例子，只在块首标一个环境反而会误导。
3. **§S3.4 开训前准备**的编号列表：第 1、3 条标了「franky 容器内」，第 4 条标了「宿主机」（`ss` 查 1337 在宿主机查一次就覆盖两个 `--network host` 的容器）。
4. **§S3.7 掉块/急停/续训表**：掉块行的「Ctrl+C 停训练」标了「GPU 容器」、REPL 标了「franky 容器」；急停行的 `ray stop` 改为「两个容器各自」、`diag-probe` 标了「franky 容器」；续训行的命令标了「GPU 容器内」。
5. **§6 排障总表**的 6 处：`臂突然大幅移动` 行的 `ray stop` 改为「每个在跑 ray 的容器各自」、`ss` 标了「宿主机」、`diag-probe` 标了「franky 容器」；`Couldn't connect` 行同样处理；`REPL 报错后进程退出` 行的 `ray stop` 标了「franky 容器内」；`reset 打印 OK 但 dz=0` 行的 `diag --probe` 标了「franky 容器内」；`NotImplementedError` 行的 `ray stop` 标了「franky 容器内」；`掉块` 行的 Ctrl+C 标了「GPU 容器」、REPL 标了「franky 容器」。

**没改的**：§S3.1 的四个文件正文块（```yaml / ```bash 里写的是**文件内容**而不是要执行的命令，不适用环境标注）；§1.3 的 ```python 块（是 franky 源码引用，不是要执行的命令）。

**教训**：

30. **「命令在哪执行」是操作手册的一等信息，不能靠上下文推断。** 这份文档在阶段 2 时代只有一个容器，「进容器后跑」是全文默认假设，不写也不会错；阶段 3 引入双容器 + 宿主机三种环境后，这个默认假设失效了，但文档里没有任何一处显式说过「现在有三种环境」。**每当文档引入一个新的执行环境，就要回头给所有既有的命令块补标签**——否则旧读者按旧假设操作新流程，新读者根本不知道该用哪个假设。这和 LOG-026 教训 27（小节号撞号）是同一类问题：**文档的隐式命名空间（章节号、执行环境、文件路径的相对基准）在扩写时要显式化，否则新内容和旧内容会互相误读。**

#### 28.1 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 / 3 | 不变，见 27.1 |
| 机器人 | 本轮无机器人操作（纯文档编辑） |
| 本轮代码改动 | 无 |
| 本轮文档改动 | `dmo_place_2.md`（§0 新增「命令的执行环境」段、21 个 bash 块加环境注释、§S3.2 启动块改为块内逐段标注、§S3.4 / §S3.7 / §6 的内联命令补环境标签）、本条 LOG-028 |
| 下一步 | 不变：真机上按 §S3.3 走相机验收 |
| 未决 | 同 27.1 |

---

### LOG-029：按 §S3.2 起双容器双节点 Ray 集群，并新增验收脚本 `verify_ray_cluster.sh`（14 项全 PASS）

**上下文与方法**：用户要求按 `dmo_place_2.md` §S3.2 实际执行双容器集群启动，并「补充验证脚本以检查是否执行成功，比如容器、ray 集群等等有没有正常启动」。本轮在宿主机上真实执行了全部步骤。**没有碰机械臂**（`ray start` 本身不做硬件校验、不连 1337；硬件校验是 RLinf 的 `Cluster` 初始化即 `train_async.py` 起跑时才发生的）。

**执行前环境检查（宿主机）**：

- `docker ps`：只有旧的 `rlinf` 容器（42 小时，franka 镜像但挂 `cxy_ws/RLinf`，文档 §1.2 明确「不要用」的那个）在跑；`rlinf-franky-5090` / `rlinf-gpu-5090` 都不存在。旧容器内**没有** raylet/gcs_server 进程，不持 1337，不碍事，保持原样未动。
- 端口：6379 空闲；1337 无 established 连接（`docker_run_franky_5090.sh` 的 FCI 前置检查条件满足）。
- 镜像：`rlinf/rlinf:agentic-rlinf0.4-franka` 与 `rlinf/rlinf:agentic-rlinf0.4-maniskill_libero` 都在本地。
- 宿主机 IP：`eno2 = 10.229.18.21`（Ray 通信网卡，两个 setup 脚本都导出 `RLINF_COMM_NET_DEVICES=eno2`），`eno1 = 172.16.0.1`（机器人网段）。`--node-ip-address` / `--address` 用 **10.229.18.21**。

**与 §S3.2 正文的两处执行偏差（均为环境约束，非流程改动）**：

1. **容器以后台方式起**：`docker_run_franky_5090.sh` / `docker_run_gpu_5090.sh` 是 `docker run -it --rm ... bash` 前台交互脚本，本执行环境无法持有前台 TTY，改用**参数完全相同的** `docker run -dit --rm ... bash`（同 name、同镜像、同挂载、同 `--network host` / `--privileged` / `--shm-size` / `-e`），后续命令用 `docker exec <name> bash -lc '...'` 进容器执行。效果与「脚本进容器后再敲命令」等价，且 `--rm` 保留（容器停即销毁）。
2. **顺序换成先 head 后 worker**：§S3.2 正文写的是 ① franky ② GPU，但注明「先起 worker 还是先起 head 都行」。实际执行选了先 GPU head（`ray start --head --port=6379 --node-ip-address=10.229.18.21`）再 franky worker（`ray start --address=10.229.18.21:6379`），避免 worker 先起时对未就绪 head 的连接重试。两边的 `export RLINF_NODE_RANK`（GPU=0、franky=1）都严格在各自 `ray start` 之前，且 franky 侧同时 `export ROBOT_IP=172.16.0.2`。

**执行结果**：

- GPU 容器 head：`Ray runtime started`，Local node IP 10.229.18.21，dashboard 8265。
- franky 容器 worker：`Ray runtime started`；setup 脚本回显确认权限与夹爪环境变量已进 raylet 启动环境（`force=20N/axis (norm<=40N) torque=6Nm/axis interp=0.02m/s step=0.05m/s`、`cube_width=0.046m +/-0.012m`——注意 0.046 仍是**默认值**，训练前必须按 §S3.1 的 EDIT ME 改成实测值）。
- `ray status`（GPU 容器内）：`Active: 2 node_*`，资源 `0.0/128.0 CPU`、`0.0/1.0 GPU`（GPU 只出现在 GPU 容器节点上，符合预期）。

**新增文件 `b/x/scripts/verify_ray_cluster.sh`**（宿主机执行，只需 docker + ss，14 项 CHECK，任一 FAIL 则 exit 1）：

| # | CHECK | 验什么 |
|---|-------|--------|
| 1-2 | `container_*` | 两个容器在跑且镜像正确 |
| 3 | `fci_free` | 1337 无连接（集群不该碰臂） |
| 4-5 | `raylet_*` | 每个容器里 raylet 进程的 `/proc/<pid>/environ` 中 `RLINF_NODE_RANK` 是**捕获对了的**（rank 错只能重启该节点 ray，事后 export 无效——这正是 §S3.2 强调的顺序问题，所以直接读 raylet 的环境而不是 shell 的） |
| 6 | `ray_status_2_nodes` | head 上 `ray status` 恰 2 个 alive 节点 |
| 7-8 | `interpreter_*` | YAML `python_interpreter_path` 指向的解释器真实存在；GPU 侧与 `which python`（source setup 后）一致，不一致会报「fix the EDIT ME」 |
| 9-10 | `franky_import` / `gpu_cuda_torch` | franky 在 franky 容器可 import；GPU 容器 `torch.cuda.is_available()` 且报出 `NVIDIA GeForce RTX 5090 D` |
| 11-12 | `gym_id_*` | `FrankyCubePlaceEnv-v1` 在**两个**容器都能注册（`RLINF_EXT_MODULE` 路径双侧可用） |
| 13 | `e2e_pinned_tasks` | 端到端：`ray.init(address="auto")` 后用 `NodeAffinitySchedulingStrategy(soft=False)` 把 task 钉到每个节点，回读该节点的 `RLINF_NODE_RANK` / hostname / `sys.executable`——证明调度能落到两个节点、且各节点 raylet 捕获的环境正确 |
| 14 | `resnet10_weights` | `resnet10_pretrained.pt` 在 GPU 容器内可见 |

**首跑结果：14/14 PASS。** 其中 `interpreter_gpu` 实测为 `/opt/venv/openvla/bin/python`，与 `realworld_cube_place_sac.yaml` 里 LOG-027 填入的占位值**恰好一致**——该处 `EDIT ME` 已实测确认，YAML 注释已改为「Verified on this machine by verify_ray_cluster.sh (LOG-029)」。

**验证脚本自身修过一个 bug**：`interpreter_gpu` 与 `ray_status` 两处最初用裸 `bash -lc 'which python' / 'ray status'`，没先 source setup 脚本，venv 未激活导致误报 FAIL（`ray: command not found`）。改为先 `source ... >/dev/null 2>&1` 再执行后通过。这本身验证了「GPU 容器的 venv 靠 setup 脚本激活」这一前提是真的。

**文档改动**：`dmo_place_2.md` §S3.2 启动块之后新增「起完之后必须跑验收脚本」小节（含命令块与 14 项检查的内容说明）；§0 状态表阶段 3 行更新为「双容器集群已起过并通过 14 项验收（LOG-029）」；`realworld_cube_place_sac.yaml` 的 `python_interpreter_path` 注释改为已验证。

**教训**：

31. **「rank 设对没有」的权威证据在 raylet 进程的 `/proc/<pid>/environ` 里，不在你敲命令的那个 shell 里。** Ray 抓的是 raylet 启动时的环境，之后在任何 shell 里 `echo $RLINF_NODE_RANK` 都只能证明那个 shell 的状态。验收脚本直接读 raylet 环境（check 4-5）+ 用 pinned task 回读（check 13），两层都是「从 Ray 视角看」，而不是「从操作者视角看」。

32. **验证脚本本身也要被验证。** 首跑 13/14 通过、唯一 FAIL 是脚本自己漏 source setup 导致的误报——如果当时直接信脚本报「GPU 容器没有 python」去改 YAML，就是一次由验证工具引入的错误修复。新写的验收脚本第一次跑出 FAIL 时，先怀疑脚本，再怀疑系统。

#### 29.1 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 | 不变，见 26.7 |
| 阶段 3 | §S3.1 四文件已建（LOG-027）；**§S3.2 双容器集群已起并通过 14 项验收**；GPU 解释器路径已实测（`/opt/venv/openvla/bin/python`）。仍欠：`FRANKA_CUBE_WIDTH_M` / 相机 serial 两处 EDIT ME、§S3.3 相机验收、训练 YAML 真机首跑 |
| 机器人 | 本轮未碰（无 1337 连接、无运动）；集群保持着「不碰臂」的状态 |
| 集群 | `rlinf-gpu-5090`（head, rank 0）+ `rlinf-franky-5090`（worker, rank 1）均在跑，ray 2 节点 alive。**注意：两个容器是 `-dit` 后台模式起的，关容器即 `docker stop <name>`（`--rm` 会自动清理）；容器内 ray 用 `docker exec <name> bash -lc '... ray stop'` 停** |
| 本轮代码改动 | 新增 `b/x/scripts/verify_ray_cluster.sh`（可执行）；`realworld_cube_place_sac.yaml` 一处注释 |
| 本轮文档改动 | `dmo_place_2.md`（§S3.2 新增验收小节、§0 状态表）、本条 LOG-029 |
| 下一步 | §S3.3 相机验收（franky 容器内，先 8a 再 8c 再 cube-place 链路自检）。集群可以留着不用重启——8c 需要单节点 ray 时先 `docker exec rlinf-franky-5090 ... ray stop`，验完再按 §S3.2 重新加入 |
| 未决 | T2（文件已建、真机未验）、T5、T6（工具已备、真机未验）、T8、T10、T11、T12、T13、T14、T15 |

---

### LOG-030：§S3.3 相机验收自动化脚本 `run_cube_place_camera_accept.sh`（含两处配套小修）

**上下文与方法**：用户要求在 `dmo_place_2.md` §S3.3 加入自动执行和验证脚本，「完成该章节的任务，并验证是否成功完成」。写脚本前通读了 §S3.3 涉及的全部四个程序（`step8_detect_cameras.py` / `step8_check_yaml.py` / `step8_test_env_camera.py` / `step_cube_place_robot.py --with-camera`），发现**文档里 8a 的手动命令照抄必挂**，两处：

1. `step8_check_yaml.py` 把 `env.eval.init_params.id` **硬编码**为 `FrankyFrankaEnv-v1`，而 cube-place 载体（`realworld_cube_place_camera.yaml`）的 id 是 `FrankyCubePlaceEnv-v1`——8b 对 cube-place 载体永远 FAIL。
2. `step8_detect_cameras.py --write-yaml` 默认写 `realworld_franky_camera.yaml`（不是 cube-place 载体），且 `_write_yaml` 只更新 `env.eval.override_cfg`——cube-place 载体 train/eval 两段都有 serial，只写 eval 会自相矛盾。

**改动**：

- `step8_check_yaml.py`：新增 `--expect-gym-id`（默认 `FrankyFrankaEnv-v1`，原 Step 8 流程行为不变）。
- `step8_detect_cameras.py::_write_yaml`：遍历 `env.train` / `env.eval` 两个 scope（存在的才更新），cube-place 载体两段保持一致；对只有 eval 的原载体是 no-op。
- **新增 `b/x/scripts/run_cube_place_camera_accept.sh`**（franky 容器内执行）：依次跑 8a（`--write-yaml --yaml-out` 直接写 cube-place 载体）→ 8b（`--expect-gym-id FrankyCubePlaceEnv-v1`）→ mode gate（`diag --probe` 必须报 `robot_mode=RobotMode.Idle`，否则中止、不动臂）→ 8c（`--save-jpeg --require-live`）→ cube-place 链路自检（`run_cube_place_phase2.sh reset --with-camera`）。每步 tee 到 `b/x/logs/camera_accept/<时间戳>/`，验证的是**机器可读标记**而不是让人盯日志：`RESULT Step8a/8b/8c PASS`、`reset-only PASS`、链路自检额外要求出现 `camera frame wrist_1:` 且无 `ALL-ZERO`（`step_cube_place_robot.py` 里 ALL-ZERO 只是诊断打印不是断言，所以断言由本脚本补）。ray 生命周期：已有集群则复用且不动它，没有则自己起单节点 head、EXIT trap 里 `ray stop`。8c 不过则跳过链路自检（先修相机，再查接线）。`--no-fci` 只跑 8a+8b（完全不碰机器人）。终态一行 `CAMERA_ACCEPT PASS/FAIL` + 失败步骤清单，退出码 0/1。
- `dmo_place_2.md` §S3.3：新增「自动执行（推荐）」小节（命令、通过标准、脚本内置安全行为、两处配套小修的说明）；原手动命令保留为「手动分步（排障时用）」，并修正了 8a 的 `--yaml-out` 与 8b 的 `--expect-gym-id`。

**验证**：`bash -n` 与 `py_compile` 过。真机首跑未做（需要相机 USB + FCI + 臂在起始位形），首跑时按 LOG-029 教训 32 先怀疑脚本再怀疑系统。

**教训**：

33. **把「文档里的命令」当代码审。** §S3.3 的 8a 命令在文档里躺了一个 LOG 周期，照抄必挂（gym id 硬编码 + 写错载体文件）——因为它从没被执行过。自动化脚本的价值不只是省手，而是强迫把每一步的「通过」定义成可断言的东西，这个过程本身就会把文档里的死命令挖出来。

#### 30.1 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 | 不变，见 26.7 |
| 阶段 3 | §S3.1 四文件已建（LOG-027）；§S3.2 双容器集群已起并通过 14 项验收（LOG-029）；**§S3.3 自动化脚本已备（LOG-030），真机未跑**。仍欠：`FRANKA_CUBE_WIDTH_M` / 相机 serial 两处 EDIT ME、§S3.3 真机首跑、训练 YAML 真机首跑 |
| 机器人 | 本轮未碰（纯脚本 + 文档改动） |
| 本轮代码改动 | 新增 `b/x/scripts/run_cube_place_camera_accept.sh`；`step8_check_yaml.py`（`--expect-gym-id`）；`step8_detect_cameras.py`（`_write_yaml` 同步 train+eval） |
| 本轮文档改动 | `dmo_place_2.md` §S3.3（自动执行小节 + 手动命令修正）、本条 LOG-030 |
| 下一步 | franky 容器内跑 `bash b/x/scripts/run_cube_place_camera_accept.sh`（先 `--no-fci` 验 8a/8b，再全量）。注意 LOG-029 留下的双节点集群：8c 与链路自检都是单节点自足（脚本会复用已有 ray 或自起自停），跑完集群状态不变 |
| 未决 | T2、T5、T6（工具已备、真机未验）、T8、T10–T15 |

---

### LOG-031：§S3.3 自动化脚本真机首跑——8a/8b/gate PASS，8c 被 Ray OOM 杀掉；三道新门 + 双相机意外

**上下文与方法**：用户在 franky 容器内首跑 `run_cube_place_camera_accept.sh`。结果：8a PASS、8b PASS、mode gate PASS（`RobotMode.Idle`），8c FAIL。本轮据此修脚本与文档。**没有动臂**（8c 在 worker 初始化阶段就被杀，env 没建成）。

**8c 失败根因（两层）**：

1. **直接原因：Ray OOM kill。** 宿主机 93.81GB 内存用到 90.4GB（96.4%）越过 Ray 95% 阈值，raylet 把正在 `__init__` 的 `FrankyControllerExtended`（16.13GB）连同 PortLockManager 一起杀了。内存大头是 LOG-029 留下的双节点集群：`rlinf-gpu-5090` 容器 52GB（其中 61 个 idle ray worker 约 28GB + dashboard 约 5GB）、`rlinf-franky-5090` 21.7GB——8c 脚本「复用已有集群」的设计在这种内存水位下必然踩线。
2. **更深一层：复用双节点集群本身就是错的。** §S3.3 原文写死「单容器单节点」，LOG-029 的下一步也写了要先停双节点集群，但 LOG-030 的脚本「已有集群则复用」。双节点集群里 `node_rank=0` 是 **GPU 容器**，而 `step8_test_env_camera.py` 的 `_build_hardware` 硬编码 `FrankaConfig(node_rank=0)`——OOM 日志里被杀的节点 ID `3436ab0a` 正是 rank 0 的 GPU head，证明控制器 worker 被调度到了**没有相机 USB、也没有 franky venv 的容器**。即使内存够，相机也采不到。

**8a 的意外：机器上插着两个 D435I**（`250222073513` 新出现，`420122070525` 是原腕相机）。连锁问题：(a) 8a 把两个 serial 都写进了载体 YAML 和 `camera_detected.json`，而训练 YAML 只配了 `420122070525`；(b) 枚举按 serial 排序，`250222073513` 排前面会变成 `wrist_1`——张冠李戴；(c) `_write_yaml` 的 `setdefault` 把 `camera_names` 写成两个键都映射 `wrist_1`；(d) `yaml.safe_dump` 回写把载体文件的注释全丢了。

**改动**：

- `run_cube_place_camera_accept.sh`：新增**集群门**（检测到已有集群默认中止并给出两个容器的 `ray stop` 命令，显式 `--reuse-cluster` 才复用）与**内存门**（`MemAvailable ≥ 20GB`，可用 `CAMERA_ACCEPT_MIN_AVAIL_MB` 调）；8b 加 `--expect-serials-from realworld_cube_place_sac.yaml`，且该检查 FAIL 时打印双相机处置指引。
- `step8_check_yaml.py`：新增 `--expect-serials-from`（与检测集合**含顺序**比对——顺序决定谁是 `wrist_1`）与 `yaml_camera_names_unique` 检查。
- `step8_detect_cameras.py::_write_yaml`：`camera_names` 改为按写入的 serial 重建（不再 setdefault 到残留键）。
- `realworld_cube_place_camera.yaml`：恢复为单相机 `420122070525` + 原注释（以训练 YAML 为准），并加注释说明 `--write-yaml` 会丢注释、会写入全部探测到的 serial。
- `dmo_place_2.md` §S3.3：自动执行小节的「内置安全行为」改写为四道门（mode / 内存 / 集群 / serial 三方一致），含停双节点集群的宿主机命令；手动 8b 命令补 `--expect-serials-from`；8c 手动块补前提注释。

**离线验证**：单相机 JSON → 8b 全 PASS；当前真实 `camera_detected.json`（双相机）→ `train_yaml_serials_match FAIL`、`RESULT Step8b FAIL`——正是首跑时该有的拦截。

**教训**：

34. **「复用现有集群」在共享内存的主机上不是中性操作。** 两个容器 `--network host`、无内存限制，看到的是同一份 93.81GB；GPU 容器的 61 个 idle worker（28GB）平时无害，一旦新 worker 要 16GB 就越过 95% 阈值。验收脚本默认不该复用不是为自己起的集群——集群的 node_rank 语义和内存预算都是它的上下文的一部分。
35. **枚举顺序即语义。** 「探测到几个相机就全写进配置」看似无害，但 serial 排序决定了 `wrist_1` 是谁；多插一个相机不只多了路帧，还会抢走腕相机的名字。验收的锚点必须是训练配置（它定义期望集合），探测结果向它对齐，而不是反过来。

#### 31.1 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 | 不变，见 26.7 |
| 阶段 3 | §S3.1 四文件已建（LOG-027）；§S3.2 集群验收过（LOG-029）；§S3.3 脚本首跑 8a/8b/gate PASS、8c OOM（LOG-031），修完后**未重跑** |
| 机器人 | 本轮未动（8c 在 worker 初始化即被杀，env 未建成） |
| 集群 | LOG-029 的双节点集群**仍在跑**（占约 74GB），重跑 §S3.3 前必须按 §S3.3 集群门先停掉 |
| 相机 | **机器上插着两个 D435I**：`420122070525`（原腕相机，训练 YAML 要的）与 `250222073513`（来路待确认）。当前训练配置是单相机；要么拔掉后者，要么改训练 YAML 走双相机 |
| 本轮代码改动 | `run_cube_place_camera_accept.sh`（集群门 + 内存门 + 8b 交叉核对）、`step8_check_yaml.py`（`--expect-serials-from` + names 唯一性）、`step8_detect_cameras.py`（names 重建）、`realworld_cube_place_camera.yaml`（恢复单相机 + 注释） |
| 本轮文档改动 | `dmo_place_2.md` §S3.3（四道门、手动命令修正）、本条 LOG-031 |
| 下一步 | ① 决定第二个相机的去留；② 宿主机停双节点集群（两个 `docker exec ... ray stop`）；③ franky 容器重跑 `run_cube_place_camera_accept.sh` |
| 未决 | T2、T5、T6（8c 仍未过）、T8、T10–T15 |

---

### LOG-032：第二只相机「插着但不配」——8a 加 `--serials` 白名单，默认锚定训练 YAML

**上下文与方法**：用户确认 `250222073513` 不用，问「是不是不在 YAML 内配置就可以」。答案分两层：

- **运行时：是的。** `FrankaRobot` 硬件校验（`rlinf/scheduler/hardware/robots/franka.py`）只查「配置的 serial ⊆ 在线 serial」，多插一只不配的相机无害；env 也只按 serial 打开配置的那只。唯一的坑是 `camera_serials` 留空/`null` 会触发 auto-detect 分支，把**全部**探测到的相机填进去——所以是「配死一只」，不是「不写」。
- **验收流程：不够。** 8a 是「探测到什么写什么」，第二只相机插着就会被写进 `camera_detected.json` 和载体 YAML，8b 的三方一致检查必 FAIL。

**改动**：`step8_detect_cameras.py` 新增 `--serials` 白名单（枚举后过滤；要的 serial 不在线则 `requested_serials_present` FAIL）；`run_cube_place_camera_accept.sh` 默认从 `realworld_cube_place_sac.yaml` 解析 `camera_serials` 作为白名单传给 8a（可 `--serials` 显式覆盖）。这样「训练 YAML 是期望集合的锚点」成立端到端：多插的相机自动忽略，少插/插错在 8a 就 FAIL，8b 的三方核对变成兜底。离线模拟验证：双相机枚举 + 白名单 `420122070525` → 只保留它；白名单要不存在的 serial → 正确报 missing。

**文档**：§S3.3 第 4 道门改写为「serial 以训练 YAML 为锚」（含 `camera_serials` 不能留空的警告）；载体 YAML 注释同步。

**结论**：第二只相机**不用拔**，训练 YAML 保持单相机 `420122070525` 即可，重跑验收脚本会自动忽略它。重跑前仍要先停 LOG-029 的双节点集群（§S3.3 集群门）。

---

### LOG-031：训练前检查脚本 `preflight_cube_place_sac.sh`（§S3.5 第 0 步，真机三层全 PASS）

**上下文与方法**：用户要求在 `dmo_place_2.md` §S3.5 加入「训练前要跑的脚本」，确保开训前一切条件满足，且任一条件不满足时提示用户「参考文档哪一节、怎么做」。动机：训练前条件原本散在三处——§S3.4 的人工六步、阶段 3 开头的放行门闩表、`verify_ray_cluster.sh`（只覆盖集群层，LOG-029）——而训练路径本身没有 Ray 前硬门（§S3.4 已强调），缺一扇一键的门。

**编号说明**：本条原计划记为 LOG-030，落笔时发现 LOG-030 已被 §S3.3 相机验收脚本占用，故顺延为 LOG-031；`dmo_place_2.md` 中相应引用已同步。

**新增文件 `b/x/scripts/preflight_cube_place_sac.sh`**（宿主机执行，只需 docker + python3 + ss，不需要任何 venv）。输出沿用 `step8_checks.py` / `verify_ray_cluster.sh` 的 `CHECK <name> OK|FAIL <detail>` 风格，末行 `RESULT preflight PASS|FAIL`，任一 FAIL 则 exit 1；**每条 FAIL 的 detail 末尾带「-> see dmo_place_2.md §X.Y：具体动作」**。分三层，按代价从低到高，前面的层 FAIL 就跳过后面的层（静态配置都没对，碰集群和臂没有意义）：

| 层 | CHECK | FAIL 指向 |
|---|-------|-----------|
| A 静态（宿主机 python3 + PyYAML，复用 `step8_checks.is_placeholder_serial`） | `h1_calibrated`（H1 `calibrated: true` 且六元组非全零） | §2.5 重做 H1 标定 |
| | `h1_matches_sac_yaml_{train,eval}`（H1 与训练 YAML 的 `target_ee_pose` 逐位一致） | §S3.1 顶部 EDIT ME |
| | `cube_width_pinned`（franky 组 env_vars 里 `FRANKA_CUBE_WIDTH_M` 存在且 ≠ 默认 0.046——LOG-024：默认值不是任何实物） | §2.5 步骤 1（REPL 量 `measured width`） |
| | `skip_camera_pinned`（同处 `RLINF_SKIP_CAMERA == "0"`） | §S3.10 第 1 条（环境变量覆盖链） |
| | `camera_serials_match`（`camera_detected.json` 存在、serial 非占位、与 YAML 的 `hardware.configs[].camera_serials` 和 `env.train.override_cfg.camera_serials` 三处一致） | §S3.3 8a |
| | `eval_not_dummy` / `save_interval_set` / `camera_player_off` | §S3.10 第 6 条 / §S3.0 对照表 / §S3.1 ① |
| B 集群 | `cluster_verify`：直接调用 `verify_ray_cluster.sh` 全部 14 项 | §S3.2（重起集群）/ §S3.1（改 EDIT ME） |
| C 机器人（只读、不动臂；`--skip-robot` 可整层跳过） | `no_live_controller`（GPU 容器内 `ray list actors` 无存活 `FrankyControllerExtended`） | §S3.4 步骤 4 |
| | `fci_free`（宿主机 `ss` 查 1337 无连接） | §6「Couldn't connect」行 |
| | `connect_preflight`（franky 容器内 `run_cube_place_phase2.sh connect` exit 0，内含 robot_mode / 起始位形 / 夹爪 holding 三道硬门） | §S3.4 步骤 1–3 |
| | `authority_echo`（从 connect 输出 grep `authority:` 行，必须是 20.0N/axis 而非 100.0N/axis——LOG-019 的权限，命中即停） | §6「authority: ... 100.0N/axis」行 |

**明确不查的三条**（脚本无法替代人，已写进 §S3.5 正文）：2.4b 的 `peak_overshoot` 实测、标记/方块是否被挪动、人是否站在急停旁。

**实现要点**：A 层用单个 python3 heredoc 完成（shell 变量先 `export` 再进 heredoc 的 `os.environ`，这是本轮修的唯一一个实现 bug——初版漏了 export，python 里读不到路径）；C 层的 connect 在 franky 容器内跑，`--connect-only` 不碰 ray（`step_cube_place_robot.py` 在 `ray.init` 之前就返回），所以即使集群没起也能查机器人——但脚本仍把 C 排在 B 之后，因为「集群没起」时训练反正开不了。

**验证（全部真跑）**：

1. `bash -n` 语法检查通过。
2. **真机全量跑**（集群为 LOG-029 所起、仍在跑；臂在线）：A 层 9 项全 OK（`FRANKA_CUBE_WIDTH_M=0.0325`、serial `420122070525` 三处一致——两处 EDIT ME 已在此前填好），B 层 14/14 PASS，C 层 4 项全 OK（connect 三道硬门过、`authority: spring force <= 20.0N/axis` 回显正确）。末行 `RESULT preflight PASS`，exit 0。
3. **负测试**：临时把 YAML 的 `FRANKA_CUBE_WIDTH_M` 改回 `0.046` → `CHECK cube_width_pinned FAIL ... -> see dmo_place_2.md S2.5 step 1: ...`，B/C 层被跳过，`RESULT preflight FAIL`，exit 1。改回 `0.0325` 后复跑恢复 PASS。
4. **`--skip-robot` 降级路径**：A+B 跑完、C 层打印 SKIPPED 提示，PASS，exit 0。

**文档改动**：`dmo_place_2.md` §S3.5 在启动命令前插入「第 0 步：训练前检查脚本（宿主机）」小节（命令块含 `# 宿主机` 标注与 `--skip-robot` 说明、三层 FAIL 指针速查表、三条人工确认清单，原启动命令改为「第 1 步」）；§S3.4 步骤 3 补「该 connect 检查已并入 preflight 脚本 C 层，开训前以脚本为准」；§7 T2 行补「另新增 `preflight_cube_place_sac.sh`（LOG-031）」；§8 源码索引新增「集群 / 训练前检查脚本」一行。

**教训**：

34. **检查脚本的分层顺序就是修复顺序。** A（静态）→ B（集群）→ C（机器人）不只是「便宜的先跑」：静态 YAML 错了，集群验收和机器人门过得再漂亮，训练起来也是错的（黑图、错宽度、错位姿都不会在 B/C 层暴露）。前层 FAIL 跳过后层，是在强迫操作者按「先配置、再集群、最后碰臂」的顺序修，而不是哪里红了修哪里。

35. **FAIL 信息的价值在于「下一步去哪」，不在于「什么错了」。** 「`FRANKA_CUBE_WIDTH_M=0.046`」只告诉你错了；「-> see dmo_place_2.md S2.5 step 1: 0.046 is the upstream default, not any real cube (LOG-024); measure with the REPL ...」才告诉你怎么办。每条 FAIL 都带文档指针的代价是脚本里维护一份「检查 → 章节」映射，收益是凌晨两点开训前不用翻 1300 行文档找该看哪节。

#### 31.1 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 | 不变，见 26.7 |
| 阶段 3 | §S3.1 四文件已建（LOG-027）；§S3.2 集群已起并通过 14 项验收（LOG-029）；§S3.3 相机验收脚本已备（LOG-030）；**§S3.5 训练前检查脚本已建且真机三层全 PASS（LOG-031，含负测试与降级路径）**。仍欠：§S3.3 相机验收真机首跑、训练 YAML 真机首跑 |
| 机器人 | C 层 connect 检查真机通过（robot_mode=Idle、起始位形、夹爪 holding、authority 20 N/axis），臂未动 |
| 集群 | 保持 LOG-029 状态（双容器双节点 alive），本轮未重启 |
| 本轮代码改动 | 新增 `b/x/scripts/preflight_cube_place_sac.sh`（可执行）；`realworld_cube_place_sac.yaml` 负测试用临时改动已还原 |
| 本轮文档改动 | `dmo_place_2.md`（§S3.5 第 0 步、§S3.4 步骤 3、§7 T2、§8 索引）、本条 LOG-031 |
| 下一步 | franky 容器内跑 `bash b/x/scripts/run_cube_place_camera_accept.sh`（LOG-030 已备）；相机验收过后，开训前在宿主机跑 `bash b/x/scripts/preflight_cube_place_sac.sh` 作为 §S3.5 第 0 步 |
| 未决 | T2、T5、T6（工具已备、真机未验）、T8、T10–T15 |

---

### LOG-032：preflight 脚本加「星号横幅 + 自动修复 + 修不了才转人工」（真机全路径调试通过）

**上下文**：用户要求优化 LOG-031 的 `preflight_cube_place_sac.sh`：条件不满足时先用 `*****` 星号横幅括出提示，然后**自动修改到条件满足**，自动修不了才提示人工；调试到通过，过程记入本 LOG；最后用改好的脚本完整跑一遍。

**新行为设计**（`b/x/scripts/preflight_cube_place_sac.sh` 重写）：

- 每项检查 FAIL 时打印 75 字符星号横幅：`FAIL <名>` / `detail` / `auto-fix: attempting... 或 not possible (<理由>) -- MANUAL ACTION REQUIRED` / `manual: 见 dmo_place_2.md §X.Y 做什么`。
- 有修复器的检查：修复 → 重跑该项 → 报 `AUTO-FIXED`（计入 PASS）或仍 `FAIL`。改 YAML 前一次性备份到 `realworld_cube_place_sac.yaml.preflight-bak`。
- **可自动修**（A 层）：H1→YAML 位姿同步（H1 是物理标定的产物，是真值源）、`RLINF_SKIP_CAMERA` 钉 `"0"`（缺失则插行）、相机 serial 从 `camera_detected.json` 同步到 hardware + train/eval override + `camera_names` 键、`is_dummy: True→False`、`save_interval→50`、`enable_camera_player: True→False`。
- **可自动修**（B 层，仅当失败项全部属于「启动顺序」类：`container_*`/`raylet_*`/`ray_status_*`/`e2e_pinned_tasks`/`gym_id_*`）：`docker start` 停掉的容器 + 按 §S3.2 顺序双节点 ray 重启（`ray stop --force` 两边 → head rank 0 → worker rank 1，rank 在 `ray start` 前导出）→ 自动复验 `verify_ray_cluster.sh`。
- **拒绝自动修**（必须人来做，脚本说明理由）：H1 标定、方块宽度（物理测量，猜错会悄悄弄坏夹爪 holding 判定）、占臂的存活 actor（杀进程是人的决定）、1337 被占（持有者可能在带臂运动）、connect 三道硬门（重新夹块/引导臂是物理动作）、authority 回显错（说明跑着的代码/环境不是你以为的那个，要查原因不是糊过去）、ResNet10 权重缺失、venv/解释器路径错。

**调试过程（按发生顺序，全部真机真跑）**：

1. `bash -n` 过；先存 golden 快照 `/tmp/sac_yaml_golden.yaml` 用于校验自动修复的还原度。
2. **自发现 bug 1（写文件前）**：`camera_serials_match` 初版把首次评估结果包在闭包里，`run_check` 修复后复评拿到的仍是修复前的旧值，会永远报「still fails」。改为 lambda 内重新评估。
3. **破坏测试 1**（5 处：train 位姿改 0.7、`RLINF_SKIP_CAMERA: "1"`、`save_interval: -1`、eval 的 `enable_camera_player: True`、eval 的 serial 改 `999999999999`）：前三个正确 AUTO-FIXED；**但 eval 的两处破坏没被发现**——`camera_serials_match` 和 `camera_player_off` 只查 train 段。**这是检查覆盖缺口（bug 2）**：eval 段是 train 的拷贝，同样会被训练用到（`val_check_interval` 一开就用）。把两个检查扩到 train+eval 双段后重跑：两处都正确 FAIL→AUTO-FIXED。
4. **golden diff 发现 bug 3（两个子问题）**：(a) `camera_names` 正则 `\s*$` 在多行模式下 `\s` 吃掉了后面的换行，把空行吞了——改 `[ \t]*$`；(b) `camera_names` 的 serial 映射用「文件顺序第 i 行 ↔ serials[i]」，单相机时 eval 段第二行拿不到 serial 没修——改为按 `wrist_<i>` 的数字索引映射 `serials[i-1]`（YAML 的既有约定）。重跑后又发现替换串漏了数字组（`wrist_1` 变成 `wrist_`，bug 3c），补上 `m.group(3)`。**最终 eval 破坏 → AUTO-FIXED 后 YAML 与 golden 逐字节一致。**
5. **B 层自动修复实测**：`docker exec rlinf-franky-5090 ... ray stop --force` 故意停掉 worker 节点 → preflight 检出 `raylet_rlinf-franky-5090` + `e2e_pinned_tasks` FAIL → 星号横幅 → 自动双节点 ray 重启 → 复验 14/14 PASS → `cluster_verify AUTO-FIXED`。整个修复约 70 秒，无需人工。
6. **人工路径实测**：`FRANKA_CUBE_WIDTH_M` 改回默认 0.046 → 星号横幅明确写「auto-fix: not possible（方块宽度是物理测量，猜的值会悄悄弄坏夹爪 holding 判定）-- MANUAL ACTION REQUIRED」+ §2.5 步骤 1 指引 → exit 1，B/C 层跳过。改回 0.0325 后确认 YAML 与 golden 一致。
7. **最终全量跑**（用户要求的「让脚本自己做一遍检查与自动修复」）：A 8 项 + B 14 项 + C 4 项全 OK（C 层含真机 connect 三道硬门与 `authority: 20.0N/axis` 回显），`RESULT preflight PASS`，exit 0。

**文档改动**：`dmo_place_2.md` §S3.5 第 0 步——三层表格加「自动修复」列（可/不可及理由），FAIL 行为段落改写为「星号横幅 → 自动修复 → AUTO-FIXED 或 MANUAL ACTION REQUIRED」，提及 `.preflight-bak` 备份；标题行标注「LOG-031 创建、LOG-032 加自动修复」。

**教训**：

36. **自动修复的验收标准是「修复后与 golden 逐字节一致」，不是「检查变绿」。** 检查变绿只证明值对了；bug 3 吞空行、改错 `camera_names` 键都是「检查绿了但文件被改伤」——因为检查只看解析后的值，不看文件格式。凡是自动改人维护的文本文件，修复后必须 diff 原始快照。

37. **破坏测试要打在「检查不看的那个分支」上。** 破坏测试 1 里打在 train 段的三处都被抓住了，打在 eval 段的两处全漏——如果当初只破坏 train，就会带着「eval 没人查」的缺口上线。对称配置（train/eval 双段）要破坏**靠后的**那段，那里才是覆盖盲区。

38. **「能不能自动修」的分界线是「改错了谁负责」。** 配置值抄真值源（H1 文件、detected json）→ 可以自动；物理量（标定、宽度、臂的位置）→ 自动修错了就是安全事故，必须人测；进程生死（占臂的 actor、1337 持有者）→ 杀错了可能正在带臂运动，必须人杀。脚本里每个「refused」都写明理由，让操作者知道这不是偷懒是刻意。

#### 32.1 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 | 不变，见 26.7 |
| 阶段 3 | §S3.1 四文件已建（LOG-027）；§S3.2 集群已起并通过 14 项验收（LOG-029）；§S3.3 相机验收脚本已备（LOG-030）；**§S3.5 preflight 脚本已具自动修复能力并全路径实测通过（LOG-031 创建、LOG-032 优化）**。仍欠：§S3.3 相机验收真机首跑、训练 YAML 真机首跑 |
| 机器人 | C 层 connect 真机通过（Idle / 起始位形 / holding / 20 N/axis），臂未动 |
| 集群 | B 层自动修复实测中被脚本自动重启过一次（双节点 ray stop → start），最终 14/14 PASS，状态与 LOG-029 相同 |
| 本轮代码改动 | `b/x/scripts/preflight_cube_place_sac.sh` 重写（星号横幅 + 6 个 YAML 自动修复器 + B 层集群自动重启 + C 层拒绝自动修并说明理由）；调试中修的 3 个脚本自身 bug 见上 |
| 本轮文档改动 | `dmo_place_2.md` §S3.5 第 0 步（自动修复行为、三层表格加列）、本条 LOG-032 |
| 副作用 | `b/x/configs/realworld_cube_place_sac.yaml` 经多轮破坏-修复后与 golden 逐字节一致；留有 `realworld_cube_place_sac.yaml.preflight-bak`（脚本设计内的一次性备份） |
| 下一步 | franky 容器内跑 `bash b/x/scripts/run_cube_place_camera_accept.sh`（LOG-030）；之后开训前宿主机跑 `preflight_cube_place_sac.sh`（§S3.5 第 0 步） |
| 未决 | T2、T5、T6（工具已备、真机未验）、T8、T10–T15 |

---

### LOG-033：按 §S3.2 重建集群 + preflight 抓到并纠正「相机检查方向性错误」（自动修复把第二只相机加进了训练 YAML）

**编号说明**：另一会话在本 LOG 的 LOG-030 与本条之间插入了它自己的 LOG-031（相机验收真机首跑）与 LOG-032（第二只相机白名单），与本序列的 LOG-031/032（preflight 创建/自动修复）撞号。本条顺延为 LOG-033；引用时以标题为准。

**任务**：按 §S3.2 启动 Ray 集群，再按 §S3.5 第 0 步跑训练前自动检查与修复。

**执行过程与结果**：

1. **集群已不在**：两个 5090 容器是 `--rm` 起的，已消失（`docker ps` 无）。按 §S3.2 重建：先查 1337 空闲 → `docker run -dit`（LOG-029 的分离模式等价命令）起 franky + GPU 容器 → GPU 容器内 `export RLINF_NODE_RANK=0` + source setup + `ray start --head` → franky 容器内 `export RLINF_NODE_RANK=1 ROBOT_IP=172.16.0.2` + source setup + `ray start --address=10.229.18.21:6379`。
2. **§S3.2 验收**：`verify_ray_cluster.sh` 14/14 PASS（含 raylet 捕获的 rank、pinned task e2e）。
3. **§S3.5 preflight**：`camera_serials_match` FAIL 并被 AUTO-FIXED——但**修复方向错了**，见下。其余全过（C 层 connect 三道硬门 + authority 20 N/axis 真机回显）。

**抓到的一个真 bug（检查语义与既定设计相反）**：`camera_detected.json` 在当日 08:47 被重新生成（另一会话的相机验收），列出了**两只** RealSense：`250222073513`（新插的）+ `420122070525`（原来的腕相机）。preflight 的旧检查要求「json == hardware == override 三者一致」，于是自动修复把 YAML 同步成了双相机——而另一会话的 LOG-032（「第二只相机插着但不配」）刚刚确立了**相反**的设计：**训练 YAML 是期望集合的锚点**，保持单相机 `420122070525`，多插的相机由验收脚本的 `--serials` 白名单自动忽略。也就是说：

- 旧语义的后果：训练 YAML 被加成双相机 → `camera_names` 把**新相机**映射成 `wrist_1`（json 枚举顺序在前），真正的腕相机反而丢了名字 → 策略看到的「wrist_1」是错的相机，又一种「训练在跑但输入不对」的静默故障。
- 处置：从 `.preflight-bak` 还原单相机 YAML（与 golden 逐字节一致），重写 `camera_serials_match` 的语义——**YAML 为锚**：hardware/train/eval/`camera_names` 四处一致、非占位、且每个 serial 都被 USB 实测到；json 里的多余相机只在 detail 里标注「extras plugged but ignored」。自动修复方向反转：override/`camera_names` 与 hardware 不一致时**从 hardware 锚点同步**（不再从 json 往 YAML 加相机）；仅当 YAML 为空/占位且**只插了一只**相机时才采纳 json 值；YAML 里的 serial 没被实测到 → 转人工（插相机是物理动作）。
- 顺带补了覆盖缺口：旧检查不看 `camera_names` 键与 serials 的一致性，现在四处一起查。

**验证**：

- 新语义下全量跑：单相机 YAML + 双相机 json → `OK ... (extras plugged but ignored: ['250222073513'])`，YAML 未被改动。
- 破坏测试（eval 段 serial + camera_names 改成 `999999999999`）→ FAIL 横幅 → 从 hardware 锚点 AUTO-FIXED → 修复后 YAML 与 golden **逐字节一致**。
- 最终全量跑（A 8 项 + B 14 项 + C 4 项真机）：全 OK，`RESULT preflight PASS`，exit 0。

**文档改动**：`dmo_place_2.md` §S3.5 第 0 步表格 A 层行改写为「serial 以训练 YAML 为锚」语义（含「多插的相机忽略」与自动修复方向说明）。

**教训**：

39. **自动修复的方向就是架构的所有权方向，写反了比不修更危险。** 「json → YAML」和「YAML → json」看起来都是「同步」，但前者把「物理上插了什么」当成「训练该用什么」，后者才是既定设计。自动修复器落笔前要先回答「这个字段的真值源是谁」——答错的话，修复器越勤，配置漂移越快。这次是靠 `.preflight-bak` 备份 + 另一会话的 LOG 才兜住的。

40. **跨会话撞号会发生，引用 LOG 要带标题。** 两个会话并发追加日志时编号会撞（本次 LOG-031/032 各有两条）。本条起引用格式改为「LOG-0XX（标题关键词）」，纯编号不再可靠。

#### 33.1 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 | 不变，见 26.7 |
| 阶段 3 | §S3.1 四文件已建；§S3.2 集群**本轮重建**并通过 14 项验收；§S3.3 相机验收已真机首跑（另一会话 LOG-031：8a/8b/gate PASS，8c 被 Ray OOM 杀掉后已加门）+ 双相机白名单（另一会话 LOG-032）；§S3.5 preflight 语义修正后全 PASS（本条）。仍欠：训练 YAML 真机首跑 |
| 机器人 | C 层 connect 真机通过（Idle / 起始位形 / holding / 20 N/axis），臂未动 |
| 集群 | 本轮重建（容器曾被 `--rm` 清掉）：`rlinf-gpu-5090`（head, rank 0）+ `rlinf-franky-5090`（worker, rank 1），14/14 PASS |
| 本轮代码改动 | `preflight_cube_place_sac.sh`：`camera_serials_match` 语义反转（YAML 为锚、json 多余忽略、修复方向 hardware→override、补 `camera_names` 一致性检查） |
| 本轮文档改动 | `dmo_place_2.md` §S3.5 表格 A 层行、本条 LOG-033 |
| 副作用 | `realworld_cube_place_sac.yaml` 一度被错误自动修复成双相机，已从 `.preflight-bak` 还原并与 golden 逐字节一致（备份机制第一次真派上用场） |
| 下一步 | 开训：宿主机 `preflight_cube_place_sac.sh`（第 0 步）→ GPU 容器 `run_cube_place_sac.sh realworld_cube_place_sac`（第 1 步），前五分钟盯 §S3.5 的 8 条表 |
| 未决 | T2、T5、T6（8c 真机仍欠）、T8、T10–T15 |

---

### LOG-034：训练 YAML 真机首跑 → 围栏 lag 看门狗在第 10 秒跳闸（T15 首次真机验证）；补做 2.4b 四档（T5 关闭、T11 未复现、嫌疑收敛到旋转授权）

**任务**：接 LOG-033（按标题引用，见教训 40）的下一步——GPU 容器内跑 §S3.5 第 1 步开训。这是训练 YAML 的**真机首跑**（T2 欠的那一项）。

#### 34.0 结论先写

启动链路全部正确，**不是配置故障**：双节点、env 落 franky 容器、相机 serial 正确、`authority: 20.0N/axis`、围栏装好并回读、`go_to_rest` 两次尝试收敛。**策略接手约 10 秒后运动围栏按设计刹车**，`RuntimeError` 经 env 侧抛出，Ray actor 依次退出。事后只读探针：`RobotMode.Idle`、`has_errors=False`、`|dq|=0.0026`、Desk 无 fault 要清。**围栏做了它该做的事**；问题在于「策略的哪一部分授权把臂推到了跟不上」。

#### 34.1 事件时间线（取自当次日志，非手算）

| 时刻 | 事件 |
|---|---|
| 01:18:38 | `motion guard armed: xyz in [[0.6269,-0.0751,0.2358],[0.8269,0.1249,0.4108]] ... max_lag=0.050m, orient<=0.550rad` |
| 01:18:38 | `reach: worst box corner r=0.785m (92% of 0.855m reach) NEAR-SINGULAR; worst fence corner r=0.842m (98%) NEAR-SINGULAR` |
| 01:18:38 | 夹爪：`cube_width=0.0460m +/-0.0120m, holding=False width=0.0314m` ← **这里已经不对了，见 34.4** |
| 01:18:44 | `rest pose reached on attempt 2 (err=0.0037m)`，悬停 `[0.7183, 0.0154, 0.3294]`；SAC 开始 |
| 01:18:49 | `step slew clamped: 0.0112m -> 0.0087m` ×2（平移限速正常工作） |
| 01:18:53 | `freeze_at_current: target <- measured [0.7092, 0.0657, 0.2907]` |
| 01:18:54 | `brake (lag, freeze-then-stop): \|dq\| 2.6942 -> 0.0211 rad/s, stop=clean` |
| 01:18:54 | `WATCHDOG trip [lag]: \|measured-commanded\|=0.0719m > 0.0500m`；commanded `[0.7181, 0.0221, 0.2724]` |
| 事后探针 | `Idle` / `has_errors=False` / tcp `[0.7180, 0.0222, 0.2720]` / `\|F_ext\|=3.41N` / `\|dq\|=0.0026` |

**一个容易漏掉的取证细节**：事后臂**停在了当时的指令位姿上**（`0.7180,0.0222,0.2720` ≈ commanded `0.7181,0.0221,0.2724`）。所以那 7.2 cm 不是「臂卡住了、指令跑了」，而是**臂自己飞出去 7 cm 又被拉回**——`|dq|=2.69 rad/s` 是在飞行途中量到的。方向上也印证：measured y=0.0657 而 commanded y=0.0221、悬停 y=0.0154，臂是往 **+y 侧甩了 4.4 cm**。

#### 34.2 根因分析：平移解释不了，只剩旋转

`FrankaEnv.step` 每周期都用**新鲜实测位姿**重算目标（`next_position = measured + action*action_scale`），`_clamp_step_slew` 再把合成位移掐到 `max_action_scale_xyz*√3 = 0.0087 m`（5 cm/s @10 Hz）。也就是说**平移方向上合法的领先量上限是 8.7 mm**，而实测滞后 71.9 mm 是它的 **8.3 倍**。日志里那两条 `step slew clamped` 恰好证明这条限速在工作。所以：随机换向、横向运动都在 8.7 mm 的合成预算内，**都不可能产生这个滞后**。

剩下唯一的解释是**旋转**，而它恰好是 LOG-019 之后**唯一没有被重新推导过**的授权：

- `CubePlaceConfig.__post_init__` 只 clamp `action_scale[0]`（`b/x/franky_ext/tasks/cube_place.py`），`action_scale[1] = 0.1 rad` 原样从 charger 继承；
- `_clamp_step_slew` 只改 `out[:3]`（`b/x/franky_ext/franky_single_franka_env.py`），姿态分量不碰；
- `MAX_INTERP_ANGLE_RAD = 0.6` / `INTERP_SPEED_RAD_S_DEFAULT = 0.15` 只作用于 `_interpolate_move`，**`step()` 这条路走不到**；
- `motion_limits.py` 有 `max_action_scale_xyz()`，**没有**旋转的对应函数。

于是未训练的 SAC 每步可命令**每轴 0.1 rad**（三轴合成 0.17 rad ≈ **1.7 rad/s** 的腕部角速度），叠加控制器自己报的 **NEAR-SINGULAR（盒角 92%、围栏角 98% 臂展）**——近奇异处一点末端转动就要求很大的关节速度，与 `|dq|=2.69 rad/s` 完全吻合。

**这是 T12（`step()` 平移上限是推算值）的旋转孪生问题**，而且比 T12 更糟：平移至少有个推算过的 5 cm/s，旋转连推算都没有。

#### 34.3 补做 2.4b 四档（用户选择的处置路径）

前置：人在急停旁（已确认）、方块夹着、FCI 空闲、无存活 actor、`Idle`。起点 `z=0.2720`（接触点上方 2.1 cm，正好落在事故发生的 0.272–0.291 区间，代表性好）。**未 `ray stop`**：LOG-033 的双容器集群仍要用，探针已证明没有进程占 FCI。**档间不降臂**——`_guards_for` 对负 `dz` 的默认地板是「目标下方 5 cm」，在本机会落到接触点以下 3 cm，等于授权把方块压进标记；四档全部向上（向上还会让臂折起来、离奇异更远），最后用一次带显式 `--z-floor 0.315` 的受控下降回悬停。顺序改成 2 → 4 → 3（10 cm 档结束时臂最高，放最后）。

| 档 | 命令 | 速度 / 幅度 | `peak_lag` | `peak\|dq\|` | `peak_overshoot` | `peak\|F_ext\|` | 判定 |
|---|---|---|---|---|---|---|---|
| hold | `diag --dz 0.03 --seconds 3` | 零位移 3 s | 0.0009 | 0.011 | 0.0000 | 5.6 N | OK（sag −0.8 mm） |
| 1 | 同上 | 1 cm/s / 3 cm | 0.0043 | 0.028 | 0.0000 | 6.8 N | OK（final +0.0265） |
| 2 | `diag --dz 0.03 --seconds 1` | 3 cm/s / 3 cm | 0.0057 | 0.068 | 0.0000 | 7.1 N | OK（final +0.0268） |
| 4 | `diag-replay --dz 0.03 --seconds 1.5` | 10 Hz 阶梯 2 cm/s / 3 cm | 0.0065 | 0.102 | 0.0000 | 7.7 N | OK，`stop=clean` |
| 3 | `diag --dz 0.10 --ramp-speed 0.02` | 2 cm/s / 10 cm | 0.0092 | 0.097 | 0.0000 | 10.7 N | OK（final +0.0950） |
| 回降 | `diag --dz -0.109 --ramp-speed 0.02 --z-floor 0.315` | 2 cm/s / −10.9 cm | 0.0043 | 0.112 | 0.0007 | 2.9 N | OK，回到 `z=0.3328` |
| **事故** | — | — | **0.0719** | **2.69** | — | — | 跳闸 |

**T5 关闭。** `peak_overshoot` 终于在**真实位形**上量到了（这正是 LOG-018 覆盖不到、LOG-019 出事的那个缺口）：门闩线 ≤0.02，六档实测**全部 0.0000**。滞后随速度增长很慢（1→3 cm/s 只从 4.3 到 5.7 mm），与 LOG-018「滞后由静摩擦主导、约 8 N 与速度无关」一致；`peak|F_ext|` 随幅度爬到 10.7 N，仍在 20 N/轴授权内。

**T11 未复现，但假说空间被砍掉一块。** `--test-waypoints` 是专门设计来复现 LOG-019 那 26 cm 的（10 Hz 零阶保持 + `K_t=2000` + 软零空间 + 伸展位形），在真实位形上跑出来是 `overshoot=0.0000`、`peak|dq|=0.102`。所以**「这个位形下 10 Hz ZOH 阻抗本身就不稳」不成立**——至少在纯平移、单调、慢速的条件下不成立。26 cm 的机理仍未确定，但现在可以确定它需要 2.4b 覆盖不到的某个因素。

**嫌疑收敛。** 事故滞后是最差档的 7.8 倍、`|dq|` 是 26 倍。2.4b 覆盖不到的只有三件：旋转（`target_at` 只改 xyz，姿态恒为起点姿态）、方向反转（斜坡是单调的）、横向运动（只动 z）。后两件都被 8.7 mm 的合成平移预算掐住，**只有旋转的授权上限从来没被推导过**——与 34.2 的代码结论独立地指向同一处。

#### 34.4 顺带暴露的独立问题 A：`env_configs.env_vars` 到不了控制器 actor

日志第一行夹爪回显是 `cube_width=0.0460m ... holding=False width=0.0314m`，而训练 YAML 明明钉了 `FRANKA_CUBE_WIDTH_M: "0.0325"`（preflight 的 `cube_width_pinned` 还专门查过并 OK）。根因在启动方式：

```python
# b/x/franky_ext/controller_extended.py::launch_controller
placement_strategy=NodePlacementStrategy(node_ranks=[node_rank]),
```

`NodePlacementStrategy` 不带 `node_group`，落进保留的 `node`/`cluster` 组（日志里就是 `node_group_label='cluster'`），而该组 `env_configs: null`。`Cluster.allocate` 只合并**当前组**的 `env_vars`（`node_group.get_node_env_vars(node_rank)`），于是：

| 谁 | 走哪个组 | 拿到 `FRANKA_CUBE_WIDTH_M` |
|---|---|---|
| `EnvGroup`（env worker） | `franky` 组（`component_placement`） | **0.0325**（YAML 生效） |
| `FrankyControllerExtended`（控制器 actor） | `node`/`cluster` 组 | **0.046**（franky 节点 raylet 环境 + head 侧广播） |

**而夹爪和 holding 判定窗口住在控制器里。** 后果：窗口是 `[0.034, 0.058]`，实测 0.0314 落在窗外 → `holding=False`。这次它没有引发跳闸（`GripperCloseEnv` 每步发的夹爪动作是 0，日志里全程没有 `gripper grasp:` / `skip grasp`），但掉块恢复、`is_open`、`move()` 的「持物时拒绝变宽」全部读 `_hardware_holding()`，现在都是错的。相机不受影响：`_skip_camera()` 在 env worker 里读，那边拿到的是 `"0"`。

**同一个洞还会漏掉 §3.2 表里的其它权限量**（`RLINF_CUBE_*`、`FRANKA_GRASP_FORCE`）——它们这次「碰巧对」，是因为 franky 容器 `ray start` 前 source 过 setup 脚本，而不是因为 YAML 生效了。

**preflight 挡不住它**：A 层查的是 YAML 里写了什么（意图），不是 actor 里实际读到什么（效果）。

#### 34.5 顺带暴露的独立问题 B：T15 的四句只到了三句，且 driver 的 handler 自己崩了

T15 要核对的链是「刹车 → 锁存 → env 侧抛 → `finally` 停 tracker → 打印 teardown health」。真机实际出现的：

| 环 | 出现？ | 证据 |
|---|---|---|
| 刹车 | ✅ | `brake (lag, freeze-then-stop): \|dq\| 2.6942 -> 0.0211 rad/s, stop=clean` |
| 锁存（不抛） | ✅ | `WATCHDOG trip [lag]: ... -- arm braked; the next command and the next guard_tripped() poll will report this` |
| env 侧转普通异常 | ✅ | `_raise_if_guard_tripped` 抛出的 `RuntimeError`，堆栈完整 |
| 锁存后拒绝下发 | ✅ | `refusing to command motion: the motion guard is tripped ...` |
| `finally` 停 tracker + teardown health | ❌ | **没有出现** |

原因是 driver 侧：`Cluster.signal_handler` 收到 SIGUSR1 后 `list_actors` 再逐个 `ray.kill`，把控制器 actor 直接杀了；而且它遍历的是已经过期的存活列表，自己抛了 `ValueError: Failed to look up actor with name 'FrankyControllerExtended-0-0:0'`。所以 LOG-023 设计的「让 `finally` 有机会跑完」在**训练路径**上仍然不成立——它只在烟测脚本（driver 就是脚本本身、有自己的 `try/finally`）里成立。本次没造成安全问题（tracker 在 `_abort_motion` 里已经拆掉、臂已刹住），但**teardown health 这道取证仍然没拿到**，T15 只能记「部分验证」。

#### 34.6 验证与现场状态

- 2.4b 六段运动全部 `tracker.stop() clean`、`has_errors=False`，无一次 `ABORT`。
- 臂最终在 `z=0.3328`（悬停目标 0.3308，高 2 mm）、`Idle`、`|F_ext|=2.57N`、`|dq|=0.0011`，方块仍夹着。
- 集群：LOG-033 起的双容器双节点未动，本轮未重启。
- 本轮**未改任何代码**（改 `b/x/` 要连带重跑 1A 的 18 组不变量，留给下一轮）。

#### 34.7 教训

41. **「跑通」和「测到」是两件事，而且欠账会精确地在它覆盖的那一档爆掉。** 2.4b 从 LOG-019 起欠了三轮（LOG-022、LOG-026 都跳过，理由都是「2.7 已经一次跑通」）。这次真机首跑第 10 秒就跳闸，而补做 2.4b 的结果是**全过、余量极大**——看似「白跑」，实则价值最高：正因为平移档全部干净，才能把嫌疑从「阻抗链路有问题」压缩到「旋转授权没推导过」这一处。**没有基线的异常值只是一个数字，有基线的异常值是一条线索。**

42. **限速要按动作空间的每一个自由度过一遍，不能按「主要那个」过。** LOG-019 之后我们给平移加了 `max_action_scale_xyz` + `_clamp_step_slew` + 5 cm/s 上限 + 围栏 lag 门，唯独旋转一路原样继承 charger 的 `action_scale[1]=0.1`——而 `_clamp_step_slew` 的 `out[:3]` 那个切片，就是这个盲区在代码里的样子。凡是「给动作加上限」，检查清单必须是动作向量的**维数**，不是「我记得的那几个」。

43. **配置检查要查「生效值」，不是「配置值」。** `cube_width_pinned` 检查 YAML 写了 0.0325 并 OK，而 actor 里读到的是 0.046——检查项和被检查的对象之间隔着一整条 Ray 环境变量传递链，链上任何一环（这次是 `NodePlacementStrategy` 不带 `node_group`）都能让两者脱钩。凡是「环境变量必须钉住」的检查，正确形态是**在目标进程里回读一次**（如控制器启动时把关键量连同来源一起打印，preflight 去 grep 那行），而不是在宿主机上解析 YAML。这与教训 39（自动修复的方向）是同一个问题的两面：39 问「真值源是谁」，43 问「真值到了没有」。

44. **优雅收尾的「最后一环」要在它真正的宿主里验证。** LOG-023 把「围栏不抛异常、由 env 侧转普通异常」这条链设计出来，1A 断言了契约、烟测脚本里跑通过——但训练路径的 driver 是 `train_async.py` + `Cluster.signal_handler`，它的错误处理是「杀光所有 actor」，`finally` 根本来不及跑。**在 A 宿主里验证过的收尾链，不能推断在 B 宿主里也成立**；而这次连 handler 自己都抛了 `ValueError`，说明那条路径本身也没被真机走过。

#### 34.8 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 2 | **2.4b 本轮补做完成、六档全过**（此前三轮欠着）；2.4 一节的「只覆盖 1 cm/s / 3 cm / x≈0.55 空载」限定条件解除 |
| 阶段 3 | 训练 YAML **已在真机首跑**（T2 的真机验证部分完成）：启动链路全部正确，策略接手第 10 秒围栏 lag 跳闸。**未产出任何训练步**（buffer 里只有几十个 transition，无 checkpoint） |
| 机器人 | `Idle`、`has_errors=False`、`z=0.3328`（悬停高度）、方块夹着、Desk 无 fault |
| 集群 | LOG-033 状态未变（`rlinf-gpu-5090` head rank 0 + `rlinf-franky-5090` worker rank 1） |
| 本轮代码改动 | **无** |
| 本轮文档改动 | 本条 LOG-034；`dmo_place_2.md`（§0 状态表两行、§2.4b 状态、§7 T5/T11/T12/T15/T2、§S3.5 第五分钟表与跳闸行、§S3.7 跳闸行、§6 新增症状行、§8 无变化） |
| 未决 | **T5 关闭**；T11 未复现（假说空间已缩小）；**新增 T16（`step()` 旋转授权无上限）与 T17（`env_configs.env_vars` 到不了控制器 actor）**；T2（首跑已做、训练未成）、T6、T8、T10、T12–T15 |
| 下一步（三件，按依赖排序） | ① 给 `diag_franky_motion.py` 加**旋转阶梯档**（10 Hz 阶梯，`--drx/--dry/--drz`，复现 `step()` 的姿态指令形状），在真实位形上量 `peak\|dq\|` 与滞后 → 这是唯一能证实/证伪 34.2 的实验；② 据实测给 `step()` 加旋转上限（`max_action_scale_rot` + `_clamp_step_slew` 同时 clamp 姿态），**先测再定值**；③ 修 34.4 的环境变量洞（控制器 actor 走 franky 组，或把关键量随 actor 参数显式传下去 + 启动时回读打印，preflight 改查回读行）。三件都动 `b/x/`，每件都要重跑 1A 的 18 组不变量 |

## LOG-035：三件事全部落地为代码（T16 上限 + 诊断阶梯，T17 env_var 路由修复），1A 全过，真机阶梯待人到场后再跑

用户原话：「三件都做了吧，最好现成自动执行和检验脚本。」—— 对应 LOG-034 §34.8「下一步」的三项。本条只记**代码/脚本改动与无实机风险的验证**；真正在真机上跑新姿态阶梯、以及用新上限重启训练，都还没做，见 §35.6。

### 35.1 T16：`step()` 路径的旋转授权补上限

**新增** `motion_limits.py`：
- `STEP_ROT_SPEED_RAD_S_DEFAULT = 0.3`（rad/s）+ `STEP_ROT_SPEED_RAD_S_RANGE = (0.02, 1.0)`，环境变量 `RLINF_CUBE_STEP_ROT_SPEED`。这个默认值**不是测出来的**，是按平移那对默认值的比例推的：`STEP_SPEED_M_S_DEFAULT / INTERP_SPEED_M_S_DEFAULT = 0.05/0.02 = 2.5`，套到旋转就是 `INTERP_SPEED_RAD_S_DEFAULT(0.15) * 2 ≈ 0.3`（取整数、略保守于 2.5x）。**先有栏栏、后有实测**，栏栏本身不能当作「已验证」——这正是本轮新增 §7 T18 要追的账，见 §35.5。
- `step_rot_speed_rad_s()` / `max_action_scale_rot(step_frequency)`，与平移那对函数结构对称。

**改** `franky_ext/tasks/cube_place.py::CubePlaceConfig.__post_init__`：新增一段与 `action_scale[0]` 完全对称的检查，`action_scale[1]` 超过 `max_action_scale_rot(step_frequency)` 就打印 WARNING 并夹到上限（`0.1 rad/step` → `0.03 rad/step`，即 `1.0rad/s` → `0.3rad/s`）。

**改** `franky_ext/franky_single_franka_env.py::_clamp_step_slew`：这是 LOG-034 §34.2 指出的真正缺口——即使 `action_scale[1]` 被夹住，一个绕过 `action_scale` 直接调 `_move_action` 的调用者（或以后改错了 `action_scale` 的默认值）仍然不受这层保护，正如原来 `out[:3]` 那一行只夹平移、姿态原样传下去。现在函数：
1. 平移逻辑不变（`xyz` 超预算按比例拉回）；
2. 新增姿态逻辑：用 `quat_angle_rad(测量姿态, 目标姿态)` 算夹角，超过 `max_action_scale_rot(step_frequency)*sqrt(3)` 的预算就用 `scipy.spatial.transform.Slerp` 在测量姿态与目标姿态之间按比例插值，取代目标姿态；
3. 两条日志合并成一行 `step slew clamped: xyz ...; rot ...`，而不是原来平移专用的单行——为姿态新增专门的日志格式没有意义，两者本来就是同一次限幅决策的两个分量。

`franky_single_franka_env.py` 新增 import：`scipy.spatial.transform.Slerp`、`motion_limits.max_action_scale_rot`、`motion_limits.step_rot_speed_rad_s`。

### 35.2 T16 的诊断工具：`diag_franky_motion.py` 旋转阶梯 + `run_2_4b_rotation_ladder.sh`

**为什么不复用 `run_tracker_motion`**：那个函数的 `Trace`/`Guards` 语义（`overshoot`/`lag` 都定义在 xyz 上）已经被 2.4b 验证过、被本文档多处引用，硬塞一个「有时测 xyz 有时测姿态」的模式进去，代价是让一个已经被信任的脚本变得难以推理。所以新增一套并行的 `RotGuards` / `RotTrace` / `run_tracker_rotation`，结构照抄 `run_tracker_motion`（同样的 `_brake` 刹车函数、同样的 fence-vs-lag 顺序、同样的有界 poll 而非固定 sleep），只是测的量换成角度：

- `--test-rotation`：绕当前姿态按 `--drx/--dry/--drz`（弧度）平滑 ramp，再 hold+watch（`--test-impedance` 的旋转版）。
- `--test-rotation-waypoints`：10 Hz 零阶保持阶梯，**这是复现 `step()` 逐周期指令形状的那个**（`--test-waypoints` 的旋转版），也是唯一能证实/证伪 `STEP_ROT_SPEED_RAD_S_DEFAULT` 的实验。
- 额外加了一层这两个函数专用的 `|dq|` 熔断（`--max-dq`，默认 `1.0 rad/s`）——这是一个从未在这条臂上量过的授权，比平移阶梯多一道保险；LOG-034 事故的 `|dq|` 峰值是 2.69 rad/s，这道熔断在那之前一个数量级就会先动。
- 结束后额外打印一行「本档命令了多少 rad/step，相对当前 `max_action_scale_rot` 是余量还是超限」，把阶梯结果直接和上限挂钩，不需要人工换算。

**新增** `run_cube_place_phase2.sh` 子命令 `diag-rot` / `diag-rot-replay`（默认 `--drz 0.05 --seconds 1`，即单档小幅度试探）。

**新增** `run_2_4b_rotation_ladder.sh`：全自动阶梯，4 档（ramp 慢/中、10Hz 阶梯慢/快），每档失败即停（复用 2.4b 教训 40：每个测试开头都会 `recover_from_errors()`，失败后接着跑下一档等于清完故障立刻再动一次），每档独立落日志到 `logs/2_4b_rotation_ladder/`，跑完打印汇总表。默认幅度上限 `0.15 rad`（在姿态围栏 `0.55 rad` 松弛量以内留足余量）、速度上限 `0.15 rad/s`（=`INTERP_SPEED_RAD_S_DEFAULT`，这条臂唯一实测过没出事的旋转速度），**故意不测到 `STEP_ROT_SPEED_RAD_S_DEFAULT`（0.3 rad/s）本身**——2.4b 当年测平移也没测到 `STEP_SPEED_M_S_DEFAULT`（T12 至今未关）。也**故意在悬停位形（非接近伸展极限的箱体角）跑**——LOG-034 跳闸发生的位形是「箱体角 92-98% 伸展」的近奇异点，脚本头部注释明确写了这是刻意的分阶段暴露，逼近奇异点是另一项更谨慎、需要专门设计的实验，不混进这个脚本。

### 35.3 T17：控制器 actor 的 `env_configs.env_vars` 路由

根因（LOG-034 §34.4 已诊断）：`FrankyControllerExtended.launch_controller` 用 `NodePlacementStrategy(node_ranks=[node_rank])`，不带 `node_group_label` 时解析到 `cluster.get_node_group()`（保留的默认组，`env_configs: null`），而 `WorkerGroup._launch_worker` 是按 `placement.node_group_label` 去查 `env_vars` 的（`worker_group.py:292`）——控制器因此对 YAML 里瞄准它的所有 `env_configs.env_vars` 都是瞎的。

**改** `franky_ext/franky_single_franka_env.py::FrankySingleFrankaEnvMixin.__init__`：在调用 `super().__init__` 之前，从 `kwargs["worker_info"]`（一个 `Placement`）取出 `node_group_label`，存到 `self._worker_node_group_label`。这个属性是 env 自己的 Worker 被放进哪个组（例如 `franky`）——日志证实过这条链是对的（`EnvGroup ... node_group_label='franky'`）。

**改** `_setup_hardware`：只有当 `controller_node_rank == self.node_rank`（控制器与 env 共址，当前配置唯一支持的情形）才把 `self._worker_node_group_label` 传给 `launch_controller`；否则传 `None`，走回退路径并打日志说明为什么。这是保守选择——猜错组比不传更危险。

**改** `franky_ext/controller_extended.py::launch_controller`：新增 `node_group_label` 形参。给定时，用 `Cluster().get_node_group(label).node_ranks.index(node_rank)` 把全局 node_rank 换算成该组的**局部** rank（`NodePlacementStrategy` 文档明确写了带 `node_group_label` 时 `node_ranks` 是组内局部下标），再构造 `NodePlacementStrategy(node_ranks=[local_rank], node_group_label=label)`。任何解析失败（比如给了一个不包含该 node_rank 的组）都不抛异常、退回旧行为并打 WARNING——启动失败比环境变量没生效严重得多。

**改** `franky_ext/franka_libfranka_gripper.py`：connected 日志新增 `raw_env FRANKA_CUBE_WIDTH_M=... FRANKA_HOLD_TOL_M=...`，直接回读这个**进程自己的** `os.environ`，不经过任何配置解析——这是 LOG-034 教训 43「查生效值不是配置值」的具体落地：以后 `cube_width` 不对时，不用再从 YAML 一路追到 actor，看这一行就知道该进程到底收到了什么（`None` = 完全没收到，仍在用硬编码默认）。

preflight 侧**没有新增检查**：这个问题只能在训练真的启动、控制器真的打了这行日志之后才能验证，preflight 跑在训练启动前，天然看不到。核对方式仍是 §S3.5「前五分钟要盯的」表里已有的那一行（LOG-034 已经改过，指向这行日志）——只是现在这行日志多了 `raw_env` 字段，核对更直接。

### 35.4 1A 回归（18 组 → 20 组，纯数值/纯 dummy，无机器人）

`step_cube_place_dummy.py` 新增两组不变量：
- `max_action_scale_rot(10.0)` 与 `step_rot_speed_rad_s()` 的乘除关系、且严格小于 upstream 的 `0.1 rad/step`；
- 用 `action_scale=[0.02, 0.1]` 构造一个 `CubePlaceConfig`，断言两个分量都被夹住；再用 `inspect.getsource` 断言 `_clamp_step_slew` 源码里还含 `max_action_scale_rot`（防止以后有人重构时把姿态那段删掉又不触发任何断言失败）。

franky 容器内跑（不接触机器人，纯 dummy env）：

```bash
# franky 容器内
source b/x/configs/setup_before_ray_5090.sh
python b/x/scripts/step_cube_place_dummy.py
```

结果：`Phase1A PASS FrankyCubePlaceEnv-v1`，新增两行分别打印
`step action_scale[1] cap 0.0300rad/step at 10Hz = 0.30rad/s (upstream 0.1 = 1.0rad/s)` 与
`_clamp_step_slew still clamps rotation, not just translation`。
构造 dummy env 时也能看到 `CubePlaceConfig.__post_init__` 的新 WARNING 按预期打印（`action_scale[1]=0.1000 rad/step ... clamping to 0.0300 rad/step`）。

另外用 `--probe`（只读，连接机械臂但不发任何运动指令）确认新增的 import（`Slerp`、`quat_angle_rad`、`max_action_scale_rot`、`step_rot_speed_rad_s`）在真实 franky 容器里都能正常加载、连接：

```bash
# franky 容器内
python b/x/scripts/diag_franky_motion.py --probe
```

`robot_mode=RobotMode.Idle`、`has_errors=False`、方块仍夹着（`tcp` 与 LOG-034 收尾时几乎一致），确认这段时间机器人处于安全静止状态，没有被本轮任何操作触碰。

### 35.5 §7 台账更新

- **T16**：状态改为「代码已落地（`max_action_scale_rot` + `_clamp_step_slew` 姿态分量 + 1A 断言），阶梯诊断工具已备（`--test-rotation[-waypoints]` + `run_2_4b_rotation_ladder.sh`），**真机验证未做**」。
- **T17**：状态改为「根因已修（`launch_controller` 按 `node_group_label` 路由 + 局部 rank 换算），回读日志已加（`raw_env` 字段），**真机重跑验证未做**（需要训练真的连一次控制器才能看到那行日志）」。
- 新增 **T18**：「`STEP_ROT_SPEED_RAD_S_DEFAULT=0.3rad/s` 是按比例推算的，不是测出来的（对齐 T12：平移那侧同样的账，`STEP_SPEED_M_S_DEFAULT` 至今没有被实测验证过，只验证到 `INTERP_SPEED_M_S_DEFAULT`）。`run_2_4b_rotation_ladder.sh` 目前默认只跑到 `0.15rad/s`，且刻意避开近奇异点位形（悬停，非箱体角）。要关闭这一项需要：① 在悬停位形跑完阶梯到 `0.3rad/s`；② 在真正跳闸时的箱体角位形（92-98% 伸展）单独设计一次更谨慎的复现实验。」

### 35.6 尚未做的（需要人在现场）

**本轮全程没有对机器人下发任何运动指令**（只做了 `--probe`）。以下两件都需要人在机器人旁边、急停在手才能做，本条日志到此为止，留给下一次会话或用户明确授权后再执行：

1. `bash b/x/scripts/run_2_4b_rotation_ladder.sh`（或先 `run_cube_place_phase2.sh diag-rot --drz 0.05 --seconds 1` 单档试探）——验证 T16 的旋转限幅在真机上确实生效、且不会在小幅度下自己触发熔断。
2. 用新代码重启一次 `bash b/x/scripts/run_cube_place_sac.sh realworld_cube_place_sac`——同时验证 T17（控制器 connected 日志里 `raw_env FRANKA_CUBE_WIDTH_M=0.0325`）和 T16 对训练场景的实际效果（旋转分量现在应该在 `0.3rad/s` 被限幅，`step slew clamped: ... rot ...` 应该在日志里出现而不是直接跳闸）。

| 项 | 状态 |
|---|---|
| 阶段 3 | 代码修复（T16/T17）已落地并过 1A；真机验证（阶梯 + 重训）待人到场 |
| 机器人 | 未被本轮任何操作触碰，状态与 LOG-034 收尾时一致（`Idle`、方块仍夹着） |
| 集群 | LOG-033 起的双容器双节点未动，`ray status` 确认 2 节点 alive、0 资源占用 |
| 本轮代码改动 | `motion_limits.py`（+2 函数+2 常量）、`tasks/cube_place.py`（`__post_init__` 新增旋转夹紧段）、`franky_single_franka_env.py`（`_clamp_step_slew` 姿态分量 + `__init__`/`_setup_hardware` 的 `node_group_label` 传递）、`controller_extended.py`（`launch_controller` 新增 `node_group_label` 解析）、`franka_libfranka_gripper.py`（connected 日志加 `raw_env`）、`diag_franky_motion.py`（`RotGuards`/`RotTrace`/`run_tracker_rotation`/`test_rotation[_waypoints]` + CLI）、`step_cube_place_dummy.py`（+2 组不变量）、`run_cube_place_phase2.sh`（`diag-rot[-replay]`）、新增 `run_2_4b_rotation_ladder.sh` |
| 本轮文档改动 | 本条 LOG-035；`dmo_place_2.md` §0/§7/§8/`下一个动作`（见下方 commit） |
| 未决 | T16/T17 均改「代码已修，真机未验」；新增 T18（旋转速度上限的实测缺口）；T2/T6/T8/T10/T12–T15 未变 |

## LOG-036：真机验证 T16/T17——诊断脚本一个测量口径 bug（已修）、旋转阶梯全过、但重训仍在第 3.5 分钟二次跳闸，根因下修到腕部奇异点

用户原话：「我在现场，执行这两步吧」——对应 LOG-035 §35.6 的两步（旋转阶梯 + 重启训练）。**结论先说：T16 只是部分缓解，不是修复；T17 的代码修复本身还有一个新 bug，本条已修但尚未在训练里验证。机器人全程安全（每次都 `Idle`/`has_errors=False`），但训练仍会跳闸，不建议在下一步动作前再次开训。**

### 36.1 前置状态确认

跑之前先 `--probe`：`Idle`、`has_errors=False`、`|dq|=0.0013rad/s`，位姿与 LOG-034/LOG-035 收尾时一致，方块仍夹着。确认安全后才开始。

### 36.2 诊断脚本本身的一个 bug：`ang_overshoot` 测的是「离目标还有多远」，不是「超过目标多少」

单档试探（`diag-rot --drz 0.05 --seconds 1`）第一次跑，**在 t=0.00s 就被围栏中止**：`ABORT [fence]: orientation overshoot 0.0500rad > 0.0500rad`，此时臂几乎没转（`ang=+0.0002rad`）。这不是真实超调，是测量口径反了——

```python
# 改前（b/x/scripts/diag_franky_motion.py::run_tracker_rotation）
ang_overshoot = quat_angle_rad(live_quat, final_quat)   # = 离终点的角距离
```

在 t=0 时 `live_quat == start_quat`，这个值天然就等于命令的总幅度（本例 0.05rad），跟真正的「转过头了多少」毫无关系。对照平移侧一直在用、已被信任的写法（`run_tracker_motion`）：

```python
motion = final_target - start_xyz
along = float(np.dot(live - final_target, motion / norm))   # 有符号：越过终点后才为正
trace.peak_overshoot = max(trace.peak_overshoot, along)
```

平移用「沿运动方向的有符号投影」，越过终点之前恒为负、不计入 `peak_overshoot`。旋转没有天然的方向向量，但对单调单轴 ramp，「已转过的角度」和「命令的总幅度」都是标量，两者之差就是同一个角色的替代品。**修法**：

```python
target_ang = float(quat_angle_rad(final_quat, start_quat))   # 循环外算一次：命令的总幅度
...
ang_overshoot = ang - target_ang   # ang = quat_angle_rad(live_quat, start_quat)，循环内已有
```

`b/x/scripts/diag_franky_motion.py` 已按此修改（`run_tracker_rotation` 新增 `target_ang`，第 690 行附近）。语法检查通过后，同一条命令重跑：**四个采样点全绿**（`t=0.00s ang=+0.0002 ... peak_ang_overshoot=+0.0000` 一路到 `final_ang=+0.0455rad`），`tracker.stop() clean`，`rotation ramp + settle: OK`。

**教训 45：新写的诊断代码，写完之后要先自己复盘「t=0 时这个式子的值应该是什么」。** 这条 bug 只要在心里代一次 t=0 就能发现，却是靠真机第一次运行才暴露——不是因为没法静态发现，是因为写完之后没有对着自己刚写的口径反问一遍「这在最保守的输入下应该等于什么」。好在它保守偏向了安全的一侧（提前中止而不是漏检）。

### 36.3 旋转阶梯（`run_2_4b_rotation_ladder.sh`）四档全过

修复后重新 `--probe` 确认仍 `Idle`/`|dq|≈0`，再跑全阶梯：

| 档 | 幅度/速度 | `peak_ang_overshoot` | `peak_ang_lag` | `peak\|dq\|` | 结果 |
|---|---|---|---|---|---|
| 1 ramp 慢 | 0.05rad, 0.025rad/s | 0.0000 | 0.0142 | 0.051 | PASS |
| 2 ramp 中 | 0.10rad, 0.100rad/s | 0.0000 | 0.0380 | 0.230 | PASS |
| 3 step 慢 | 10Hz, 0.05rad/s | 0.0000 | 0.0232 | 0.130 | PASS |
| 4 step 快 | 10Hz, 0.15rad/s（=`INTERP_SPEED_RAD_S_DEFAULT` 上限） | 0.0000 | 0.0600 | 0.379 | PASS |

四档全部 `alive=True`、`tracker.stop() clean`，`peak|dq|` 最差 0.379rad/s，远低于事故的 2.69rad/s，也在脚本自带 1.0rad/s 熔断以内。**这一段本身是干净的、可信的基线**——但它跑在悬停位形（REST），不是训练实际会到达的近奇异位形，这一点在下面被证实是关键限定。

### 36.4 重开训练：`raw_env` 仍是错的（T17 代码有第二个 bug），且训练本身在第 3.5 分钟二次跳闸（T16 只是部分缓解）

`preflight_cube_place_sac.sh` 全 PASS 后，GPU 容器内重启 `run_cube_place_sac.sh realworld_cube_place_sac`。

**先看好消息：** `EnvGroup` 侧的日志证实 T16 的限幅代码确实在跑（这是**这次和 LOG-034 唯一的实质差别**）：

```
WARNING: action_scale[0]=0.0200 m/step at 10 Hz is 20 cm/s; clamping to 0.0050 m/step (5 cm/s).
WARNING: action_scale[1]=0.1000 rad/step at 10 Hz is 1.00 rad/s; clamping to 0.0300 rad/step (0.30 rad/s).
```

训练跑到 **Global Step 58–65/200、Elapsed 02:09–02:25**（LOG-034 是第 10 秒），期间反复出现 `step slew clamped: rot ...` 与 `step slew clamped: xyz ...`，说明限幅确实在逐周期把策略的命令拉回预算内，不是摆设。

**但训练在 02:16:55（约第 3.5 分钟）再次跳闸：**

```
brake (lag, freeze-then-stop): |dq| 2.0315 -> 0.0321 rad/s, stop=clean
motion guard WATCHDOG trip [lag]: not tracking: |measured-commanded|=0.0512m > 0.0500m
tcp=[0.7788, 0.0249, 0.2639] -> [0.7787, 0.0311, 0.2636]
```

跳闸前最后三条日志分别是 `xyz 0.0121m -> 0.0087m (5.0cm/s cap)`、`rot 0.0600rad -> 0.0520rad (0.30rad/s cap)`、`xyz 0.0098m -> 0.0087m (5.0cm/s cap)`——**两个分量都被限幅代码钳在预算内**，可命令的末端速度不可能超过设计上限，然而关节速度仍冲到 **2.03rad/s**。

**与 LOG-034 对比：**

| 项 | LOG-034 首跑 | 本次重训 |
|---|---|---|
| 跳闸时刻 | 第 10 秒 | 第 ~205 秒（Global Step ~60） |
| `\|measured-commanded\|` | 0.0719m | 0.0512m（刚过门槛） |
| `\|dq\|`（刹车前） | 2.69rad/s | 2.03rad/s |
| 跳闸时 tcp | 近奇异盒角，92–98% 伸展 | `x=0.7788`（比 LOG-034 更远，接近伸展极限） |
| 跳闸时 q | 未记录 | `q=[0.033, 1.136, -0.034, -0.423, **0.085**, 1.497, 0.540]` |

**T16 的效果是真实的（推后 20 倍时间、降低 25% 的 `\|dq\|` 峰值），但没有解决问题**——本质原因很可能不是「旋转授权太大」，而是**腕部关节 5（`q[4]`）已经跑到 0.085rad，接近 0**。Franka Panda 在 `q5≈0` 时是一个**经典的腕部奇异位形**（关节 4 与关节 6 的转轴近似共线，雅可比病态）：在这个位形附近，**任何方向、任意大小合规的末端笛卡尔指令（哪怕严格卡在 5cm/s 平移 + 0.3rad/s 旋转的预算内）都可能反解出成倍放大的关节速度**——这解释了为什么「把末端指令卡住」这个思路（T16 的整个设计前提）注定只能延后、缓解，而不能根治：**它限制的是错误的空间**。真正需要限制的是关节速度本身，或者是让任务的目标位形远离这个奇异点。

这也修正了 LOG-034 §34.3 的假说：当时把「近奇异」归因为 Cartesian 意义上的「盒角 92–98% 伸展」；现在看，更准确的机制是**姿态奇异**（`target_ee_pose` 的 roll ≈ −π 这个设计本身，很可能让 `q5` 长期在 0 附近打转），跟盒子在 xy 平面上离中心多远关系不大——2.4b 与本次旋转阶梯都在**悬停位形**（`q5` 远离 0，见 §36.3 probe 输出 `q=[0.230, ..., 0.180, ...]`，`q5=0.18`）跑得干净，而两次真实跳闸都发生在策略把臂带到 `x≈0.72–0.78` 一带的路上、`q5` 被压到接近 0 的时候。

### 36.5 T17 仍未验证：修复代码本身有第二个 bug（本条已修，未再验证）

这次训练的 `raw_env` 仍是旧值：`raw_env FRANKA_CUBE_WIDTH_M=0.046`（不是 YAML 钉的 0.0325），说明 LOG-035 的 T17 修复没生效。审查代码发现 LOG-035 那次修复本身有两层错误：

1. `kwargs.get("worker_info")`——`worker_info` 是 `FrankaEnv.__init__` 的**第二个位置参数**（`override_cfg, worker_info, hardware_info, env_idx`），从不是按关键字传的，`kwargs.get(...)` **恒为 `None`**。
2. 即使拿到了，它的类型是 `WorkerInfo`（`rlinf/scheduler/manager/worker_manager.py`），这个 dataclass **根本没有 `node_group_label` 字段**——跟真正带这个字段的 `Placement`（`rlinf/scheduler/placement/placement.py`）是两个不同的类型。`getattr(..., "node_group_label", None)` 静默退回 `None`，退回旧行为，没有任何报错或警告——**这正是教训 43 想防的那类洞，这次是在补洞的代码里自己又开了一个同类的洞**。

真正的真值源：`NODE_GROUP_LABEL` 是 `WorkerGroup._launch_worker`（`rlinf/scheduler/worker/worker_group.py:267`）直接写进 worker **进程自身环境变量**的，`Worker._init_node_group`（`rlinf/scheduler/worker/worker.py:1500`）也是靠这个环境变量取自己的 `self._node_group`——`FrankaEnv` 是在这个 env worker 进程*内部*构造的，读 `os.environ.get("NODE_GROUP_LABEL")` 不需要猜测任何参数传递方式或 dataclass 形状，是最短路径。

**改** `franky_ext/franky_single_franka_env.py::FrankySingleFrankaEnvMixin.__init__`：

```python
self._worker_node_group_label = os.environ.get("NODE_GROUP_LABEL") or None
```

**未再验证**：本条修复完成于分析这次训练崩溃日志之后，没有再开一次训练去确认 `raw_env FRANKA_CUBE_WIDTH_M=0.0325` 会出现——鉴于 §36.4 的跳闸尚未解决，不建议现在为了单独验证这一行日志而再开一次训练（那样又要再承担一次奇异点风险）。

**教训 46：「代码修了」和「代码在生效」之间，隔着的不只是一层——每一层都要单独有证据。** LOG-035 把 T17 标成「代码已修，真机未验」，而这次真机验证直接告诉我们「代码修了」本身就是错的。往前看，唯一可信的证据形式还是教训 43 说的那句话：**在目标进程里回读一次**，而这次给出了它的推论：**回读也要在目标进程真的启动过一次之后才算数**——写完代码但没跑过，不能算「已修」，只能算「意图修」。

### 36.6 收尾状态

- 机器人：每次操作前后都 `--probe`，全程 `Idle`/`has_errors=False`；跳闸后确认 `|F_ext|=0.00N`、`|dq|≈0`，`stop=clean`。
- 集群：`ray status` 确认跳闸后两节点仍 `Active`，`0.0/128.0 CPU 0.0/1.0 GPU` 资源已全部释放——训练崩溃没有拖垂集群，可以直接复用做下一次尝试（一旦有修复方案）。
- T15（收尾链）：这次崩溃再次证实 LOG-034 的发现——driver 侧 `ray.kill` 直接杀 actor，`finally` 的 teardown health 打印没有出现。**T15 仍未解决，已连续两次真机复现同一模式**。
- 本轮代码改动：`diag_franky_motion.py`（修 `ang_overshoot` 测量口径）、`franky_single_franka_env.py`（修 `NODE_GROUP_LABEL` 读取方式）。
- 本轮**没有**再改任何限幅数值——在把「到底该限制哪个空间」搞清楚之前，调大或调小 `RLINF_CUBE_STEP_ROT_SPEED` 都是在猜。

### 36.7 教训

45. **新写的诊断代码，写完之后要先自己复盘边界值。** 见 §36.2——t=0 时的取值只要口头代一次就能发现口径反了，不需要等真机。

46. **「代码修了」和「代码在生效」之间的每一层都要单独有证据，包括修复代码本身。** 见 §36.5——上一轮把 T17 标成「代码已修」，这次证明修复代码本身有 bug，「已修」的证据从来没有真的出现过，只是「看起来应该对」。

47. **限幅要按「命令生效之后的物理量」验收，不是按「命令本身」验收。** T16 把末端笛卡尔空间的位移/转角卡在预算内，本身完全按设计工作（日志逐周期证实），但保护的目标量错了——事故量是关节速度，不是末端速度，两者之间的比例由雅可比条件数决定，在腕部奇异位形附近可以任意放大。**卡住 A 不能保证卡住 B，除非能证明 A 和 B 之间的比例有界**；这次两次事故都发生在这个比例失控的区域。

48. **基线要在事故会发生的位形上测，不能只在安全位形上测（教训 41 的推论）。** §36.3 的旋转阶梯干净得不像话（`peak\|dq\|` 最差 0.379），但它测的是悬停位形，`q5≈0.18`；训练路径上真正出事的位形 `q5≈0.085`——**基线覆盖不到事故位形，基线就只是「证明了别处没问题」，不能反过来证明「这里也没问题」**。

### 36.8 状态与下一步

| 项 | 状态 |
|---|---|
| 阶段 3 | **训练第二次真机跳闸**（LOG-034 第 10 秒，本次第 ~205 秒）；T16 有效但不充分；T17 修复代码本身有 bug，已改但未验证 |
| 机器人 | 全程 `Idle`/`has_errors=False`，方块状态未检查（建议下次人到场时顺手确认） |
| 集群 | 两节点 `Active`、0 资源占用，可直接复用 |
| 根因假说（更新） | 不再是「旋转授权无上限」（已限、仍跳），而是 **`q5≈0`（Panda 关节 5）腕部奇异**——目标姿态 `roll≈−π` 的设计可能让任务在几何上天然靠近这个奇异面。笛卡尔空间限幅（T16 的整个思路）在雅可比病态区间不能保证关节速度有界 |
| 新增 T19 | **关节速度本身需要一道独立于笛卡尔限幅的软件上限**（在 `_clamp_step_slew` 或控制器里加 `\|dq\|` 预测/回读式的钳制，而不是只信任末端空间的预算），或者：审查 `target_ee_pose` 的 roll 选择是否可以避开 `q5≈0` 这个奇异面（需要机器人学背景的人核对，不是纯数据能定的） |
| 新增 T20 | 2.4b 与旋转阶梯都只在悬停位形（`q5` 远离 0）测过；**需要在训练实际会到达的位形（`q5` 接近 0 一带）专门设计一次更谨慎的复现实验**，才能拿到覆盖事故位形的基线（教训 48） |
| 不建议 | 在 T19/T20 有实质进展前**不要再次开训**——目前唯一的「已知安全」区域是悬停附近，训练本身的探索会把臂带向未知安全的奇异面 |

## LOG-037：把 T19 从假说变成实测——雅可比条件数量化，推翻上一条自己写的「q5 腕部奇异」诊断，定出正解

用户原话：「请深入分析，参考 RLinf 的原代码，或者广度探索，然后回答一下如何解决 T19 这个问题」。**本条全程只读，没有对机器人下发任何运动指令。**

### 37.1 先推翻自己：LOG-036 §36.4 的「`q5≈0` 腕部奇异」是错的

那条诊断是从关节角**看出来**的，不是量出来的，而且错了两处：

1. **Panda 的腕部奇异不在关节 5，在关节 6**（`q6≈0` 时关节 5 与关节 7 转轴共线）。事故位形 `q6=1.4967`，离得很远。而且 `q6` 的下限是 −0.0175rad——腕部奇异面几乎贴着关节限位，正常根本到不了。
2. **`q5=0.0853` 本身不构成任何奇异**。它只是「关节 5 接近零位」，跟雅可比退化没有必然关系。

**教训 49：从关节角"看出"奇异，和"算出"奇异，不是一回事。** 上一条日志里我把一个未经计算的几何直觉写成了根因，还据此给出了「换 `roll` 姿态」的修法建议——如果照着做，会花大量时间去改一个和问题无关的东西。**凡是「根因是某个几何/数值性质」的结论，在写进文档之前必须有一次真的计算。**

### 37.2 新工具：`b/x/scripts/diag_franky_jacobian.py`（只读，不动臂）

关键发现让这件事变得很便宜：`franky.Model.zero_jacobian` 有一个**直接接受 `q` 的重载**（`_franky.pyi:1207` 附近），所以**任意位形的雅可比都能在不把臂开过去的前提下算出来**——只需要连上读一次 `F_T_EE` / `EE_T_K` 和模型。

脚本报告的量：完整 `J`（6×7）的奇异值 / 条件数 / `sigma_min`、平移块与旋转块各自的条件数、可操作度 `sqrt(det(J Jᵀ))`、**把本任务允许的合规指令换算成的最小范数关节速度需求 `|J⁺v|`**、最坏方向的同幅度需求、以及每个关节到限位的距离（对照 franky 斥力带 0.1rad）。内置两个已记录的位形：`trip_log036` 与 `hover_log036`。

### 37.3 实测结果

| 量 | 事故位形（LOG-036 跳闸处） | 悬停位形（2.4b / 旋转阶梯测的地方） | 比 |
|---|---|---|---|
| `sigma_min(J)` 完整 6×7 | **0.0062** | 0.1115 | **18×** |
| `cond(J)` | 310.7 | 16.95 | 18× |
| 可操作度 `sqrt(det(J Jᵀ))` | **0.00606** | 0.0883 | **14.6×** |
| 平移块 `cond(J[:3])` | 4.28 | 3.52 | 1.2× |
| 旋转块 `cond(J[3:])` | **1.41** | 1.57 | 0.9× |
| 合规指令（每轴 5cm/s + 0.3rad/s）的 `\|dq\|` 需求 | **11.77rad/s** | 0.81rad/s | **14.6×** |
| 同一需求的逐关节最大值 | **9.64rad/s**（关节 4） | 0.54 | — |
| 最坏方向同幅度的 `\|dq\|` 需求 | 49.35rad/s | 2.73 | 18× |
| 进入 0.1rad 关节限位斥力带的关节 | **无** | 无 | — |

**三条结论，都是数据说的：**

1. **机理确认：这确实是雅可比放大。** 同一条完全合规的指令，在事故位形要求 11.77rad/s 的关节速度——**逐关节最大 9.64rad/s，是 Panda 自身关节限速 2.075rad/s 的 4.6 倍**。实测观察到的 2.03rad/s 只是这个需求的一小部分，原因正是它是**力矩控制**：跟不上就体现为位姿误差，于是 `lag` 涨到 0.0512m 触发围栏。**所以 T16 那条思路（限末端笛卡尔量）在原理上就不可能充分**——它限的量与事故量之间的比例，在工作空间内可以差 14.6 倍。
2. **退化方向是「平移与旋转耦合」的，不是单纯某一类。** 平移块和旋转块**单独看都很健康**（`cond` 4.28 / 1.41），只有完整 6 维雅可比的第 6 个奇异值塌了。几何上对应的是**接近满臂展**（`q4=-0.4233`，距上限 −0.0698 仅 0.3535rad），不是任何腕部姿态问题。**这直接作废了 §36.8 里「换 `roll≈−π` 姿态」那条修法**——旋转恰恰是条件数最好的部分。
3. **零空间自运动与关节限位斥力都不是来源。** 两个位形都没有任何关节进入 0.1rad 斥力带。所以不必去调 `RLINF_CART_K_NS`（T10）或斥力参数，那是另一个问题。

### 37.4 查上游代码：这个洞是继承来的，且 franky 没有暴露可用的在环旋钮

**上游 `rlinf/envs/realworld/franka/franky_controller.py`**：`JOINT_VEL_LIMITS`（`[2.075]*4 + [2.51]*3`，polymetis 同款、留 0.1rad/s 余量）确实存在，但**只在关节阻抗路径 `move_joints` 里用了一次**（`np.clip(dq_ff, ±JOINT_VEL_LIMITS)`）。本任务走的是**笛卡尔路径 `move_tcp_pose`**——那条路上有 `_CART_MAX_STEP_M=0.10` / `_CART_MAX_STEP_RAD=0.30` 的每调用限幅，全是笛卡尔量，**没有任何关节空间的界**。T19 是上游架构的空缺，不是我们改坏的。

顺带一个好消息：上游 `get_state()` **每步已经在算 `self._robot.model.zero_jacobian(...)`**（还用它算 `tcp_vel = J @ dq`）。也就是说做关节空间需求检查所需的数据**已经在手上**，一次 6×7 的伪逆是微秒级——这个修法不需要引入任何新依赖或新的实时开销。

**franky 侧（`_franky.pyi` 逐参数核对 `CartesianImpedanceTrackingMotion`）**：1kHz 回路里可调的只有 `translational/rotational_stiffness`、`force_constraints`、`nullspace_target/stiffness`、`max_delta_tau`、`lower/upper_joint_limits` + `joint_limit_activation_distance/stiffness/damping/max_torque`、两个 `error_clip`、`gains_time_constant`。**没有关节速度上限，也没有关节阻尼项**（文档明确写「Cartesian damping is chosen internally as critically damped」——顺带再次确认 T14）。**所以「在 1kHz 回路里钳制 `|dq|`」这条路在当前依赖下不存在**，只能在命令侧（10Hz）预防 + 在看门狗侧（50Hz）兜底。

另外确认：**franky 没有暴露 IK**（只有 `Affine.inverse()`）。这意味着「启动时检查安全盒八个角的条件数」做不到——无法从笛卡尔角点反解 `q`。这个限制直接决定了修法必须基于**实时位形**。

### 37.5 结论：T19 的正解

按优先级（详细版已写入 `dmo_place_2.md` 文末「下一个动作」）：

1. **命令侧的关节空间需求限幅**（正解）：每步用当前雅可比算 `dq_req = J⁺·(Δpose/Δt)`，按 `max|dq_req| ≤ JOINT_VEL_LIMITS·margin` 缩小该步。限的是事故量本身；数据已备；**靠近奇异位形自动降速而不是跳闸**，对训练友好。
2. **看门狗加 `|dq|` 判据**（纵深防御）：50Hz 已在采样，加阈值 1.0–1.5rad/s（旋转阶梯实测最差 0.379，悬停位形合规需求 0.81）。不预防，但让诊断名字正确，且能抓不产生滞后的失控。
3. **收缩安全盒**：事故点 `x=0.7788` 已在盒外 1.9mm，而盒 x 上界 0.7769 本身就落在 `sigma_min≈0.006` 的坏区。第 1 条做完后训练会在这里持续被减速，所以应把 `clip_x_range` 收小或把 `target_ee_pose` 往内移——「别把工作区定在坏条件区」和「限幅」是互补的两件事。
4. **明确作废**：改 `roll≈−π` 姿态（§37.3 结论 2）、调零空间刚度或斥力参数（结论 3）、继续调 `RLINF_CUBE_STEP_ROT_SPEED`（任何固定的笛卡尔上限，要在事故位形安全就必然在别处慢得没必要）、放宽 `lag` 门槛（掩盖）。

### 37.6 教训

49. **从几何"看出"的根因，和"算出"的根因，不是一回事。** 见 §37.1——上一条日志把未经计算的直觉写成根因，并据此给了一条会浪费大量时间的修法建议。凡是「根因是某个几何/数值性质」的结论，写进文档前必须真的算一次；**而这次「算一次」的成本是一个只读脚本 + 3 秒**。
50. **限幅要限「事故量」，代理量只有在能证明比例有界时才可用。** 这是教训 47 的定量版：末端速度与关节速度之间的比例，实测在工作空间内差 14.6 倍——这个数字就是「为什么代理量不可用」的完整答案。**要用代理量，先量出比例的上界；量不出来，就说明选错了量。**
51. **先查依赖暴露了什么旋钮，再设计修法。** §37.4 逐参数核对了 franky 的 1kHz 回路参数表与 IK 的有无，才排除了「在环钳制」和「配置期查盒子角点」两条看起来最自然的路。**在不知道底层给了什么接口的情况下设计的修法，有可能整条都不成立。**

### 37.7 状态

| 项 | 状态 |
|---|---|
| 机器人 | 全程未动（只读连接），停在事故位形 `Idle`、`has_errors=False` |
| 本轮代码改动 | 新增 `b/x/scripts/diag_franky_jacobian.py`（只读诊断，无 lint 错误） |
| 本轮文档改动 | 本条 LOG-037；`dmo_place_2.md` §7 T19 行重写（含实测数据、两条已排除假说）、文末「下一个动作」重写为 5 条 |
| T19 | **机理已实测确认，修法已定，代码未实现** |
| T20 | 工具已备（本条的脚本），实验未做 |
| 不建议 | 仍然不建议开训——T19 的第 1、2 条都还没写成代码 |

---

## LOG-038 — T19 落地：命令侧关节需求限幅 + 看门狗 `|dq|` 判据（2026-08-20）

用户指令：**「实现 ①② 并测试直到通过」**——即 LOG-037 §37.5 的第 1 条（命令侧关节空间需求限幅）与第 2 条（看门狗 `|dq|` 判据）。

### 38.1 ① 命令侧关节需求限幅

新增 `motion_limits.joint_demand_scale(J, Δxyz, Δrotvec, dt, limits)`，返回 `(scale, worst_ratio, worst_joint)`：把这一步想要的笛卡尔位移换成它**在当前位形下所需求的**关节速度 `dq_req = J⁺·(Δpose/dt)`，再报出把最坏关节拉回 `JOINT_VEL_LIMITS × margin` 所需的缩放系数。

三个设计点，都是有意的：

- **限「需求」而不是「实测」。** 臂是力矩控制的，一条不可能的指令不会表现为关节转得快，而是表现为位姿误差一直涨到 `lag` 门限（这正是 LOG-036）。等 `|dq|` 测得出来时，指令早已被接受。**雅可比是唯一能在下令之前就知道「这个 5cm/s 在关节 4 上值 9.64rad/s」的东西。**
- **一次除法，不迭代。** `dq_req = J⁺v` 对 `v` 线性，`v` 对步长线性，所以缩放系数是精确解，且缩放后需求正好落在预算上（不是「小一些」）。测试里专门把这条线性性做成断言——若哪天有人给伪逆加了阻尼，单次缩放就失效，那条断言会说话。
- **缩小而不是拒绝。** 靠近坏条件位形时臂自动变慢、回合继续，而不是跳闸把整个训练带走。

`margin` 取 `JOINT_VEL_DEMAND_FRACTION_DEFAULT = 0.5`（env `RLINF_CUBE_DQ_DEMAND_FRAC`）。这个数是**照实测挑的**：悬停位形满速指令的逐关节最坏需求只有限速的 0.26，所以 0.5 让好位形**完全不受影响**；而事故位形同一条指令是 9.29 倍预算，会被缩到 **10.8%**。这种「好位形看不见、坏位形咬得狠」的不对称正是要的效果。

接入点 `FrankySingleFrankaEnvMixin._clamp_joint_demand`，在 `_clamp_step_slew` 里**最后**跑，作用在已经过笛卡尔限幅的位姿上，两道界因此可以叠加（谁更紧谁生效）。数据来自 `state.arm_jacobian`——上游 `get_state()` 每步已经在算，`FrankaEnv.step` 每周期刷新，所以**零额外通信、只多一次 6×7 伪逆**。

### 38.2 实现过程中实测发现的一个真实漏洞：插值路径没有被覆盖

准备做活体验证时先只读算了一遍：机器人此刻**正停在 LOG-036 跳闸的那个位形上**，于是顺手把各条路径的需求都算了出来（`dt=0.1s`，三轴同时饱和的最坏合规指令）：

| 指令 | 最坏关节 | 需求 / 预算 |
|---|---|---|
| 插值 纯平移 2cm/s | j4 | **1.73×** |
| 插值 纯旋转 0.15rad/s | j4 | **2.48×** |
| 插值 平移+旋转 | j4 | **4.21×** |
| step 纯平移 5cm/s | j4 | 4.32× |
| step 纯旋转 0.3rad/s | j4 | 4.97× |
| step 平移+旋转 | j4 | 9.29× |
| 插值 单轴 +z 2cm/s | j4 | 0.50×（合规） |

**即插值速度本身在这个位形上就超预算 1.73–4.21 倍**，而 `_move_action` 在插值期间是**故意跳过** slew 限幅的（LOG-020 的原因：上游 `_interpolate_move` 在 waypoint 循环外只读一次 `_franka_state`，位置型限幅会把有效目标钉在 `start + budget`，臂差 4.2cm 停住）。也就是说 reset 这条路——**训练启动时第一个动的东西**——原本完全不设防。

修法必须换机制而不是换阈值：**位置限幅不行，时长限幅行。** 关节需求与时长成反比，所以在 `interp_duration_s` 算出 `capped` 之后再乘 `1/scale` 拉长时长（`_stretch_interp_for_joint_demand`），是精确解、不需要逐 waypoint 状态、且**臂仍然到得了目标，只是走得慢**。超过 `INTERP_DURATION_S_RANGE` 上限（20s）则拒绝并提示手动引导——那种情况下笛卡尔阻抗本来就不该动这条臂。

诚实边界（已写进 docstring）：**只在起点的雅可比上评估**。路径中途条件数会变，那一段由 ② 的看门狗兜。把这一点写明，因为「以为覆盖了其实没覆盖」正是 T19 第一次被漏掉的方式。

### 38.3 ② 看门狗 `|dq|` 判据

`_evaluate_guard` 加 `dq` 判据，阈值 `GUARD_MAX_DQ_RAD_S_DEFAULT = 1.2` rad/s（env `RLINF_CUBE_GUARD_MAX_DQ`）。1.2 是**夹在两个实测总体之间**、两侧都留空隙：

| 实测值 | 性质 |
|---|---|
| 0.379 rad/s | 整条 2.4b-rot 旋转阶梯的最差峰值 |
| 0.81 rad/s | 悬停位形上满速合规指令的需求（合法满速） |
| **1.2 rad/s** | **本阈值** |
| 2.03 / 2.69 rad/s | LOG-036 / LOG-034 两次跳闸 |
| 2.075 rad/s | Panda 自身 j1–j4 关节限速 |

两个顺序问题比阈值本身更重要，都做了测试断言：

1. **`dq` 必须排在 `lag` 之前判。** 关节失控是**间接**表现为 lag 的：臂跟不上→位姿误差涨→lag 门限触发→报「not tracking」。**两次真实跳闸都因此被归错档**，LOG-036 还为一个关节空间问题跑了一整条笛卡尔旋转阶梯。先判 `dq` 意味着跳闸消息直接说出机理。
2. **`dq` 必须走 `lag` 的刹车顺序**（`freeze_at_current` 先、`stop` 后）。这是同一情形提前一步看到：关节快是**因为**在追一个追不上的目标，所以先撤掉那个目标仍然是第一件有用的事。顺序搞反会让刹车比不刹更糟（LOG-021）。

`_brake` 的分派因此从 `kind == "lag"` 改成 `kind in ("lag", "dq")`；`motion_health()` 增加 `guard_max_dq` / `joint_speed`；`describe_authority` 与 `arm_motion_guard` 的日志行都带上新的两个限幅，避免出现「生效了但日志里看不见」。

### 38.4 测试

新增 `b/x/scripts/test_t19_joint_demand.py`，分两半（因为这两半失效方式不同）：

- **数学半（无硬件）**：合成雅可比上的闭式答案、缩放后需求**正好**落在预算（`ratio=1.000000000`）、线性性、平移与旋转两侧都会被限（只看一侧会漏掉 LOG-036 那种旋转驱动的）、零/错形雅可比必须是 no-op（dummy 环境带的就是零雅可比，当成「无穷刚」会把每一步都限成零）。
- **实测半（`--robot`，只读、不下发任何运动）**：在 LOG-037 那两个位形上算真雅可比。这一半才能抓住「雅可比取错帧」或「矩阵转置了」这类合成测试永远发现不了的错。

外加三条源码级断言（写错时是静默的）：`dq` 排在 `lag` 前、`dq` 走 lag 刹车顺序、关节限幅排在笛卡尔限幅之后。同样的断言也加进了常设闸门 `step_cube_place_dummy.py`。

**结果（`--robot`，全部 PASS）：**

```
=== real Jacobians (READ-ONLY, no motion commanded) ===
  [PASS] hover pose: full-speed step passes unclamped  worst j4 at 0.517x budget
  [PASS] LOG-036 trip pose: the same step IS clamped   worst j4 at 9.29x budget -> step x0.1077
  [PASS] trip pose worst joint is j4 (LOG-037 measured 9.64 rad/s there)
  hover ratio 0.517x vs trip ratio 9.287x = 18.0x apart
  clamped step at trip pose: 0.54 mm, 3.23 mrad per cycle
  [PASS] clamped step is non-degenerate (>0.1 mm/cycle)
  [PASS] trip pose: even the INTERP speed is over budget  worst j4 at 4.21x -> stretch x4.21
  [PASS] a 5 cm reset hop stretches to a sane duration, not a refusal  2.50s -> 2.50s (cap 20s)
  [PASS] hover pose: interp speed needs no stretch
```

`0.1077` 与 LOG-037 §37.5 设计时预估的 `0.108` 一致——**实测把设计时的算术验证了一遍**。`step_cube_place_dummy.py`（Phase1A）在改动后仍然 PASS，两条新闸门都跑到了。

### 38.5 教训

52. **「这条路径为什么跳过了那道限幅」的答案，往往是「换个机制」而不是「别跳过」。** §38.2：插值路径跳过位置限幅有正当理由（状态陈旧），但它需要同一道界——把界从**位置**改成**时长**，两个问题就都没有了。**遇到「这里不能用 X」时，先问被限的量还能用什么形式表达，再考虑放弃这道界。**
53. **实现完了别急着上硬件，先用只读算一遍全部路径。** §38.2 那个漏洞是在准备活体测试时顺手算出来的，而不是在设计时想出来的——**如果直接跑 reset，第一个动作就走在没设防的路径上。** 只读诊断脚本的边际成本是 3 秒。
54. **阈值要能被实测总体夹住，并把这个夹逼写成测试。** §38.3 那张表里，阈值上下各留了实测数据做锚。测试里断言的不是「阈值等于 1.2」，而是「阈值高于已知正常、低于已知事故」——**前者只锁住一个字面量，后者锁住的是这个数存在的理由。**

### 38.6 状态

| 项 | 状态 |
|---|---|
| 机器人 | 全程未动（只读连接）；`Idle`、`has_errors=False`、`\|dq\|≈0.002`；仍停在事故位形，夹爪含 0.0325m 方块（`width=0.0304m`，holding=True） |
| 代码改动 | `motion_limits.py`（+`joint_demand_scale` / `guard_max_dq_rad_s` / `joint_vel_demand_fraction`，`describe_authority` 带上新限幅）、`franky_single_franka_env.py`（+`_clamp_joint_demand`、+`_stretch_interp_for_joint_demand`）、`controller_extended.py`（`_evaluate_guard` 加 `dq` 且排在 `lag` 前、`_brake` 分派、`motion_health`、guard 日志）、`step_cube_place_dummy.py`（两条新闸门）、新增 `b/x/scripts/test_t19_joint_demand.py` |
| ① 命令侧限幅 | **已实现，静态 + 实测雅可比全部通过；未经动臂验证** |
| ② 看门狗 `\|dq\|` | **已实现，阈值有实测锚，顺序有断言；未经动臂验证** |
| 插值路径 | 漏洞已发现并已补（时长拉长），未经动臂验证 |
| T20 | 仍未做（安全盒内 `sigma_min` 分布扫描） |
| 下一步 | 活体验证 `reset` → `box`：机器人正停在坏位形上，是最好的测试台。预期能看到 `interpolate_move: joint demand j4 ...x budget; stretching to ...s` 与 `step slew clamped: joint demand ...` 两条新日志，且**不跳闸** |

---

## LOG-039 — T19 活体验证通过：reset + box 全过，两道界在真机上可见地生效（2026-08-20）

承 LOG-038。软件层面全过之后做动臂验证。机器人当时正停在 LOG-036 跳闸的那个位形上、夹着方块、`Idle`——**这是唯一已知会触发限幅的真实位形，所以是最好的测试台**，不是退而求其次的选择。

### 39.1 起始位姿闸门先拦了一次（这是它该做的）

```
START-POSE PROBLEM: start is 0.0520 m from the mark in xy, outside the 0.050 m box.
```

臂离标记 52.0mm，比 50mm 的盒子超 **2mm**。这不是新代码的问题，是 LOG-019 那道既有闸门在正常工作，同时它**恰好印证了 LOG-037 §37.5 第 3 条**：两次跳闸都把臂顶到盒子 +x 边缘外一点点，而那个边缘本身就落在坏条件区（同一份输出里 `reach: worst box corner ... 92% of 0.855m reach NEAR-SINGULAR`）。

用 `--allow-start-outside-box` 越过（超出仅 2mm、移动方向是朝标记往里即条件数变好的方向、人在现场手放急停）。**这是有意识地绕一道安全闸门，记在这里以便日后审计。**

### 39.2 两个环境问题挡在前面，都与本次改动无关

**问题 1：Ray OOM，两次。** 控制器 actor 在 `__init__` 里被 Ray 的 OOM killer 杀掉，报 `Memory on the node was 90.42GB / 93.81GB (96.4%)`。**机器人全程没动。**

第一次以为是前两次训练崩溃留下的残留，`ray stop --force` 后内存从 64GB 掉到 8GB，重起集群——**同一个 OOM 又出现了**。这次数字说了实话：

| 观测 | 值 |
|---|---|
| 全新集群空转时的内存占用 | **66 GB** |
| 两个容器里的 `ray::IDLE` 进程数 | **61 + 62 = 123** |
| 每个 idle worker | ~0.45 GB |
| `/dev/shm`（Ray 对象存储） | 18–19 GB |
| 报告里控制器 `__init__` 的占用 | 11.87 GB → 19.63 GB |

**根因是配置而非泄漏**：两个容器都 `--network host` 跑在**同一台 93GB 主机**上，各自的 raylet 都按 `nproc=64` 预热工作进程池，于是 123 个 idle worker + 19GB 对象存储 ≈ 74GB **在任何任务开始之前就已经占掉了**。修法是给两边都加上界：

```bash
ray start ... --num-cpus=16 --object-store-memory=4000000000
```

空转占用降到 17GB（可用 75GB），`ray::IDLE` 降到 0，`verify_ray_cluster.sh` 14 项仍全 OK。

**问题 2：控制器被放到了 GPU 节点。** 加了上界之后 OOM 消失，露出真正的错误：

```
Placement(rank=0, cluster_node_rank=0, ..., node_group_label='cluster')
  File ".../franky_controller.py", line 107, in __init__
    import franky
ModuleNotFoundError: No module named 'franky'
```

`step_cube_place_robot.py:119` 把 `FrankaConfig(node_rank=0)` **写死**了。阶段 2 的脚本一直是在**没有集群**的情况下跑的——那时它自己在 franky 容器里起单节点 Ray，rank 0 就是 franky 容器。而 §S3.2 的两节点集群里 **franky 是 rank 1**，于是控制器落到 GPU 节点，那里没有 franky。

**这也顺带解释了「控制器 `__init__` 占 19.63GB」**：那个进程根本不是控制器，是 GPU venv 里的 CUDA torch。两个"异常"其实是同一件事的两面。

**所以：§2.x 的阶段 2 脚本与 §S3.2 的两节点集群目前是互斥的。** 本轮先 `ray stop` 回到已知可用的单节点配置（LOG-033 的 `box` 就是这么过的）完成验证，训练前再把两节点集群起回来。**根治要给 `step_cube_place_robot.py` 加一个 `--controller-node-rank`**（见 §39.5）。

### 39.3 `reset` 通过：插值时长拉长在真机上生效

`reset-only PASS`，exit=0，**未跳闸**。四段插值里三段被拉长，**每次都是关节 4**——正是 LOG-037 雅可比分析预测的那个关节：

```
interpolate_move: joint demand j4 1.56x budget at 3.20s; stretching to 4.99s (ill-conditioned pose, LOG-037 T19)
interpolate_move: joint demand j4 3.01x budget at 1.50s; stretching to 4.52s
interpolate_move: joint demand j4 3.04x budget at 1.50s; stretching to 4.57s
```

终点 `hover check: |xy-target|=0.0216m |z-hover|=0.0022m`；`guard_max_dq: 1.2` 已装载，`joint_speed: 0.0040`（离阈值三个数量级），`guard_tripped: None`、`has_errors: False`。

**一个预期内的副作用**：`rest pose NOT reached after 3 attempts: err=0.0208m > 0.0100m`。臂在这个位形上确实追不上目标——它要求 3 倍的关节速度预算，阻抗弹簧驱不动。这正是限幅的既定权衡（"降速而非跳闸"），而且这条警告的原文自己就写着「如果臂还在慢慢爬，说明阻抗在跟但很慢」。**注意这个残差不是拉长时长造成的**——拉长给的是更多时间，只会减小残差；这个位形从来没成功跑过 `reset`（闸门一直拦着），所以没有对照基线。

### 39.4 `box` 通过：一行日志把整件事说完

`box-steps PASS`，exit=0，**未跳闸**。六步全部完成（3 个零动作 + 3 步下探），奖励单调上升 0.0266 → 0.0313，`approach dz=-0.0012m`。关键是这一行：

```
step slew clamped: rot 0.0734rad -> 0.0520rad (0.30rad/s cap);
                   joint demand j4 21.20x budget (0.50 of 2.075rad/s) -> step x0.047
                   (ill-conditioned pose, see LOG-037 T19) at 10Hz
```

**两道界叠加的完整证据在同一行里**：T16 的笛卡尔旋转限幅**先**把 0.0734rad 钳到 0.0520rad（此时这一步已经完全合规），然后关节限幅发现这个**已经合规**的步子仍然要求关节 4 的 **21.2 倍**预算，把它缩到 **4.7%**。

**这就是 LOG-036 的失效形态被当场抓住**：一个通过了全部笛卡尔检查、却要求约 22rad/s 关节速度的指令。旧代码会把它发下去，然后在几秒内第三次跳闸。六步全程 `joint_speed` 稳定在 0.004、`guard_tripped: None`。

（顺带：21.2 倍高于 LOG-038 在事故位形量到的 9.29 倍，因为这一步同时要修正 0.0734rad 的姿态残差——比"三轴同时饱和"的合成最坏指令更凶。这说明**实际训练里出现的指令可以比设计时假想的最坏指令更极端**，也说明限幅的必要性比预估的还高。）

### 39.5 教训

55. **同一台主机上跑多个 Ray 节点时，工作进程池是按 `nproc` 各算一份的。** §39.2：两个 `--network host` 容器 × 64 CPU = 123 个 idle worker，空转吃掉 66GB / 93GB，**任何稍大的 actor 都会 OOM，而报错指向的是那个 actor，不是真正的原因**。多节点单主机的部署必须显式给 `--num-cpus` 与 `--object-store-memory` 设上界。
56. **两个看起来不同的异常，可能是同一件事的两面。** §39.2：「控制器 `__init__` 占 19.63GB」和「`ModuleNotFoundError: franky`」被当成两个问题看了一阵，实际上都是「控制器被放到了 GPU 节点」——19.63GB 是 CUDA torch，不是控制器。**在把两个异常当成两个问题之前，先问它们能不能被同一个原因解释。**
57. **写死的 `node_rank=0` 只在"恰好只有一个节点"时是对的。** §39.2：阶段 2 脚本一直靠"自己起的单节点 Ray 里 rank 0 就是本容器"这个隐含前提工作，加入第二个节点后前提静默失效。**凡是隐含"我就是 0 号"的代码，都要在多节点出现的那一天重新审。**
58. **活体测试的价值在于它能给出设计时想不到的数字。** §39.4：设计时按"三轴同时饱和"算出最坏 9.29 倍，真机上实测到 **21.2 倍**——因为真实指令还要同时修正姿态残差。**合成的最坏情况不一定是真实的最坏情况。**

### 39.6 状态

| 项 | 状态 |
|---|---|
| 机器人 | `reset` + `box` 两轮共动臂约 40 秒，**全程未跳闸**；结束时 `RobotMode.Move`、`has_errors=False`、`joint_speed=0.0017`、TCP `[0.7464, 0.0248, 0.3315]`、夹爪仍持方块 |
| ① 命令侧关节需求限幅 | **真机验证通过**（`box`，j4 21.2× → 步长 ×0.047，未跳闸） |
| ② 看门狗 `\|dq\|` 判据 | **已装载并在真机上运行**（`guard_max_dq: 1.2`，`watchdog_alive: True`）；**未被触发**——这是好事，说明 ① 在它之前就把问题解决了。它作为兜底的有效性仍未被真实事件检验 |
| 插值时长拉长 | **真机验证通过**（`reset`，三段被拉长，均为 j4） |
| 集群 | 为做阶段 2 验证已 `ray stop`；**开训前需按 §S3.2 重起，并加 `--num-cpus=16 --object-store-memory=4000000000`** |
| 遗留 | `step_cube_place_robot.py` 的 `node_rank=0` 写死（§39.2）；`rest pose NOT reached` 残差 ~20mm（§39.3，坏位形上的既定权衡）；T20 仍未做 |
| 开训 | **软件条件已具备**。第三次真机开训的阻塞项已清 |

## LOG-040 — 第三次真机开训：肘部漂移到病态位形（新发现）+ 手引导恢复 + dq 看门狗按设计跳闸但拖垮整条训练进程（2026-08-20）

承 LOG-039。按 §S3.2 顺序重启双节点集群（`--num-cpus=16 --object-store-memory=4000000000`），`verify_ray_cluster.sh` 14 项全过，`preflight_cube_place_sac.sh` 三层全过（含 `connect_preflight OK`），随后按 §S3.5 敲 `run_cube_place_sac.sh realworld_cube_place_sac`。

### 40.1 第一次尝试：reset 中被①正确拒绝——但暴露了肘部姿态漂移

env 初始化时的 `go_to_rest` 重试到第 2 次尝试就被 `_stretch_interp_for_joint_demand` 拒绝：

```
RuntimeError: refusing to interpolate from this configuration: the move demands 25.9x joint
j4's velocity budget (0.50 of 2.075 rad/s), and slowing it enough would take 38.9s > the 20s
cap. The arm is in an ill-conditioned (near-singular) configuration where cartesian impedance
cannot move it safely -- guide it back near the mark with the enabling device.
```

这不是回归——① 正确拒绝了一个不安全指令，机器人全程未动，`diag-probe` 确认 `Idle has_errors=False |dq|=0.0023`。但拒绝本身指向一个新问题：用 `diag_franky_jacobian.py --live` 量了一下，机械臂当前位形比 LOG-036/037 那次事故位形**还差** 7 倍：

| | `sigma_min` | `cond` | 合规指令下 min-norm `\|dq\|` |
|---|---|---|---|
| LOG-037 事故位形 | 0.0062 | ~305 | 11.77 rad/s |
| 本次（第一次 reset 失败时） | **0.0009** | **2121** | **82.60 rad/s** |
| hover_log036 参考良态 | 0.1115 | 16.95 | 0.81 rad/s |

**根因：纯笛卡尔阻抗控制不约束关节冗余方向。** `_interpolate_move`/`_move_action` 只管末端位姿误差，7 轴机械臂在给定 TCP 位姿下有一维冗余（肘部姿态），阻抗跟踪对这一维完全不施加偏置。连续几次跳闸+复位的动作積累下来，肘部姿态在不知不觉中越漂越差，直到末端位姿看起来完全正常（在安全盒内部，不靠边）时，关节解仍然病态。

代码里本有 `joint_reset`（`reset_joint(joint_reset_qpos)`，关节空间移动，能摆脱这种病态）能解决冗余漂移，但：
- `joint_reset_cycle` 默认 20000，**实际上从第一次 reset 起就永远不会触发**；
- 默认 `joint_reset_qpos=[0,0,0,-1.9,0,2,0]` 是从 charger 任务继承的，**从未针对 cube_place 这套桌面/工装验证过安全性**，不能盲目调用。

两者都不是能自动修的问题（软件不知道"安全的关节目标"是什么），于是问了用户，用户选择**现场手引导**（enabling device）把肘部姿态调回良态。给用户的参考目标是 hover_log036 的 `q≈[0.23,0.55,-0.29,-1.30,0.18,1.80,1.05]`，重点关节2、关节4。

### 40.2 手引导后复核：条件数回到参考值附近

```
q_now = [0.1187, 0.6011, -0.0995, -1.3072, 0.0458, 1.8885, 0.7611]
sigma_min=0.1141  cond=16.75  min-norm |dq|=0.76rad/s
```

对照 hover_log036（`sigma_min=0.1115 cond=16.95`），基本一致。`run_cube_place_phase2.sh connect` 复核：`robot_mode: Idle has_errors: False`、`holding=True`、起始位形闸门**未触发**（不需要 `--allow-start-outside-box`）。确认安全后重新起训练。

### 40.3 第二次尝试：训练正常起步，reward 有输出，第 2 个 rollout epoch 被 dq 看门狗按设计跳闸

这次 `go_to_rest` 顺利完成（`rest pose NOT reached after 3 attempts: err=0.0117m > 0.0100m`，只差 1.7mm，属于既定权衡内的温和提示，非故障）。训练循环开始跑：第 1 个 rollout `reward=0.302`，第 2 个 `reward=0.314`，无跳闸。第 3 个 rollout 中段：

```
[ERROR ...][controller_extended.py:792] motion guard WATCHDOG trip [dq]: joint runaway:
|dq|=1.2942 rad/s > 1.2000 (worst j6=+1.2114); q=[0.0606, 0.668, -0.113, -1.2071, 0.0998,
1.8369, 1.0819]; tcp=[0.732, -0.0062, 0.282] -> [0.7346, -0.0051, 0.2587] mode=RobotMode.Move
has_errors=False braked=True stop=clean
```

**这正是 T19 ② 设计要做的事**：跳闸类型是 `[watchdog:dq]`，不是旧的 `[lag]`；`braked=True stop=clean`；`diag-probe` 复核 `Idle has_errors=False |dq|=0.0016 |F_ext|=4.31N`——机器人干净地停下，没有失控。这条 j6 的跳闸也印证了一个新观察：即便肘部已回到良态（`cond=16.75`），**探索期策略仍能在良态区域内命中局部的关节速度峰值**（未训练策略的动作幅度本就较大），说明 ② 作为兜底不是纸面上的摆设，是会被真实探索命中的。

**新暴露的问题：跳闸的 `RuntimeError` 顺着 `_move_action → env.step → gymnasium wrapper 链` 一路往上抛，被 Ray 判定为 actor 异常退出，进而把 `RolloutGroup`、`ActorGroup` 一起 `ray.kill`，整条训练进程退出。** 集群本身（`verify_ray_cluster.sh` 14 项）复核仍 PASS，只是这次训练任务（第 2 个 rollout epoch，未到 `save_interval=50` 的第一个存档点）需要整体重新启动，损失的进度可忽略，但如果换成训练已进行数百 epoch 时跳闸，损失就不是"可忽略"了。

### 40.4 遗留问题（未决，需要产品/工程决策，本次未动代码）

- **T21（新）：跳闸后训练任务整体死亡，而不是 env 一层 reset 后继续。** 目前 `_raise_if_guard_tripped` 的契约是"抛出即整条链路失败"，对模拟环境的偶发异常是合理的（重启 episode 即可自愈），但对真机探索期"预期会偶尔跳闸"的场景，代价是丢掉未存档的训练进度。是否要在 `env.step`/`env.reset` 一层捕获跳闸异常、清故障、重新武装看门狗、按失败 episode 处理（而不是让异常穿透到 Ray actor 生命周期），需要用户决定——这会改变"跳闸=安全事件"与"跳闸=任务终止"的语义边界，不应该由脚本自动决定。
- **T22（新，非阻塞）：肘部姿态在纯阻抗控制下会随时间漂移，且没有自动检测。** 目前只有事后（reset 被拒绝，或 diag 脚本手测）才能发现。可以考虑在 `EnvGroup` 每次 reset 时用 `diag_franky_jacobian` 同款的 `sigma_min` 计算做一次快速自检（几毫秒的纯计算，不需要额外硬件动作），过低就在日志里预警，给操作员一个"该手引导一下了"的信号，而不是等到 25.9x 被拒绝才发现。
- `joint_reset_cycle=20000` 与 `joint_reset_qpos=[0,0,0,-1.9,0,2,0]`（继承自 charger 任务）在 cube_place 上事实上是死代码——如果要让它派上用场，需要先为这套工装验证一个安全的关节目标位并调低 cycle，这本身也是需要人工验证的物理动作，本次未做。

### 40.5 教训

59. **末端位姿正常不代表关节位形正常。** §40.1：7 轴臂的冗余方向纯阻抗控制完全不管，末端在安全盒中央、关节却能病态到 `cond=2121`。**只查笛卡尔位姿的健康检查是不够的**，需要同时查关节位形（哪怕只是偶尔抽查）。
60. **"能自动修复"和"知道正确目标"是两件事。** §40.1：`joint_reset` 机制本可以自动解决这次的漂移，但它的目标位是从另一个任务继承来的、从未在这套工装上验证过——**有代码路径不等于能安全调用它**，物理安全验证不能被跳过。
61. **看门狗按设计跳闸是好事，但"好事"和"训练不中断"是两个独立的目标，不能混为一谈。** §40.3：`[watchdog:dq]` 干净刹停证明了 T19 ② 有效，但同一个异常也杀死了整条训练任务——**评价一次跳闸时要分开回答"机器人安全吗"和"训练该继续吗"这两个问题**，前者这次是满分，后者目前是不及格，且没人替我们做过这个决定。

### 40.6 状态

| 项 | 状态 |
|---|---|
| 机器人 | 手引导恢复良态位形后两次开训均正常起步；当前 `Idle has_errors=False`，因 dq 跳闸清洁刹停，未失控 |
| 肘部位形漂移（T22，新） | **已发现，未修复**：纯阻抗控制下关节冗余方向会漂移到病态，无自动检测；本次靠人手引导恢复 |
| T19 ①②在真实训练中的效果 | ① 在 reset 阶段正确拒绝了危险指令（25.9x）；② 在训练 rollout 中正确捕捉了一次 j6 越限（1.29 vs 1.20 rad/s）并清洁刹停——**两道界都被真实探索命中过，而不只是设计推演** |
| 跳闸后的训练任务生命周期（T21，新） | **未处理**：跳闸异常会杀死整条 Ray 训练任务（`ActorGroup`+`RolloutGroup`），而不是被 env 一层捕获后继续；本次进度损失可忽略（第2个rollout epoch），但需要用户决定是否要改这个语义边界 |
| 集群 | 仍然存活（`verify_ray_cluster.sh` 复核 PASS），可直接用于下一次训练尝试，无需重启双节点 |
| 下一步 | 待用户决定：(a) 直接重开训练，把偶发跳闸当作训练期的正常代价，人工盯着重启；(b) 先实现 T21 的 env 层捕获与自动 reset-continue；(c) 先实现 T22 的肘部位形自检 |

---

## LOG-041 — T21/T22 落地：跳闸后自动 reset 续训（有预算上限）+ reset 时的肘部位形自检（2026-08-20）

**触发**：用户决策 LOG-040 §40.4 的两个遗留问题——"让训练在跳闸后自动 reset 继续，同时在 reset 时加一次 `sigma_min` 快速自检（复用 `diag_franky_jacobian.py` 的计算，几毫秒、不动臂），过低就预警操作员该手引导了"。

### 41.1 先摸清现状：跳闸闩锁根本没有解除的入口

改之前先把三条链路读清楚（`franky_single_franka_env.py` / `controller_extended.py` / `diag_franky_jacobian.py`），得到四个决定了后面所有设计的事实：

| 事实 | 出处 | 对设计的约束 |
|---|---|---|
| 跳闸是**闩锁**不是异常：控制器 `_abort_motion` 只刹车 + 写 `_guard_trip_reason`，一路不抛（LOG-023 定下的契约） | `controller_extended.py:575` | 恢复 = 清这个闩锁，不是 catch 一个异常 |
| **没有任何解除闩锁的 API**：`clear_motion_guard()` 只是 disarm 围栏、**不清闩锁**；唯一会清的是 `set_motion_guard()` 第 333 行的副作用 | `controller_extended.py:333,361` | 必须新写一个显式方法。**不能靠"再调一次 set_motion_guard"**——凡是能解除安全闩锁的代码都应该 grep 得出来，而不是藏在装围栏的副作用里 |
| 刹车时 tracker 被拆掉（`_cart_tracker=None`），且闩锁未清时 `_ensure_cart_tracking_motion` 会**拒绝重建** | `controller_extended.py:1015` | 只要清了闩锁，tracker 与看门狗会在下一条运动指令时自动一起回来——**恢复方法里不需要、也不应该自己重启它们**，否则会出现"能动但没人看着"的窗口 |
| `get_state()` 返回的 `FrankaRobotState` 已经带 `arm_jacobian`（6×7）与 `arm_joint_position` | `franky_controller.py:195` | `sigma_min` 自检**不需要额外往返、不需要连 franky.Model**，一次 SVD 就够 |

### 41.2 改了什么

**`b/x/franky_ext/motion_limits.py`**

| 新增 | 值 / 作用 | 依据 |
|---|---|---|
| `jacobian_conditioning(J)` | 一次 SVD 返回 `sigma / sigma_min / sigma_max / cond / manipulability`；沿用 `joint_demand_scale` 的"全零=无数据→返回 None"约定；接受 `(m,7)` 以便诊断脚本复用它算平移/旋转分块 | 在线自检与离线诊断**必须是同一份计算**：操作员照着警告去跑 `diag_franky_jacobian.py --live`，读到的得是同一个数 |
| `SIGMA_MIN_WARN_DEFAULT = 0.04` + `sigma_min_warn()`（`RLINF_CUBE_SIGMA_MIN_WARN`） | 落在实测的两群之间：良态 0.1115（悬停）/ 0.1141（LOG-040 手引导后）；限幅开始咬人约 0.055（由悬停位形最差需求占预算 0.52 外推）；坏态 0.0062（LOG-036）/ 0.0009（LOG-040 漂移） | 只预警不拒绝，所以取低一点的代价只是"沉默"，不是"不安全" |
| `GUARD_RECOVERY_BUDGET_DEFAULT = 10` + `guard_recovery_budget()`（`RLINF_CUBE_GUARD_RECOVERY_BUDGET`） | 每个 env 允许自动恢复的跳闸次数，**设 0 即完全退回 LOG-040 之前的行为** | 见 41.3 第 3 条 |

**`b/x/franky_ext/controller_extended.py`**：新增 `recover_from_guard_trip() -> dict`，**永不抛异常**（沿用围栏路径一律不抛的契约，返回 `recovered/refusal/previous_reason/...` 报告）。顺序是安全属性，不是流程：① 确认臂真停稳（`|dq| <= 0.02`，轮询 0.25s；`_abort_motion` 是先刹车后锁存，臂还在动就意味着刹车没生效）→ ② 若还有活着的 tracker 先停掉 → ③ `recover_from_errors()` 清故障并 `require_motion_ready` 复核 → ④ **最后**才清闩锁。围栏全程保持武装，未武装时直接拒绝恢复。

**`b/x/franky_ext/franky_single_franka_env.py`**

- 新增异常类 `MotionGuardTripped(RuntimeError)`，`_raise_if_guard_tripped` 改抛它。继承 `RuntimeError` 所以既有的 `except RuntimeError`（烟测脚本 teardown）行为完全不变。
- 新增 `_recover_from_trip()`：打星号横幅 → 查预算 → 调控制器恢复 → 成功后顺手做一次 `sigma_min` 自检。预算用尽或恢复被拒都返回 `False`（=让异常终止训练），并在日志里指明这通常是**工作区**问题（§7 T19/T20）而不是"把预算调大"。
- 新增 `_warn_if_ill_conditioned(when)` + `_safe_state()`：`reset()` 开头与每次恢复后各跑一次；读不到状态就跳过（**advisory 不能成为 reset 失败的原因**，尤其是刚跳闸完那次）；警告里直接给出要瞄准的良态关节角与验证命令。
- 新增 `step()` / `reset()` 覆盖：`step()` 把跳闸变成 `truncated=True, reward=0.0, info["motion_guard_trip"]`；`reset()` 最多重试 `RESET_TRIP_ATTEMPTS=2` 次，仍不行就抛出指向"用使动装置手引导"的消息（LOG-040 §40.1 那一类）。

**`b/x/scripts/diag_franky_jacobian.py`**：`_block_report` 改调 `jacobian_conditioning`（删掉自己那份 SVD），并新增打印 env 侧会给出的判定（`VERDICT: ok / ILL-CONDITIONED`）。

### 41.3 三个"不这样做会更糟"的设计决定

1. **捕获的必须是 `MotionGuardTripped` 而不是裸 `RuntimeError`。** 几行之外的 `_interpolate_move` / `_stretch_interp_for_joint_demand` refusal 也是 `RuntimeError`，但它们**必须保持致命**——那是"臂在一个笛卡尔阻抗开不出去的位形里"，自动重试只会再被拒一次，变成刷屏死循环。测试里专门断言这两个 `except` 子句中没有裸 `RuntimeError`。
2. **reward 给 0，且不编造惩罚。** 跳闸那一步没有按指令执行完（臂在半路被刹住），拿它停下的位姿打分等于给一个没做完的动作记账；而在错误路径里加惩罚项，是奖励函数开始名不副实的经典方式。要惩罚应当是显式的设计决定，信息已经放在 `info["motion_guard_trip"]` 里。用 `truncated` 而非 `terminated`，因为回合是被装置打断的，SAC 的 bootstrap 据此才对。
3. **必须有预算上限。** "恢复了接着跑"的失败模式比崩溃更坏：一条每几步就跳闸的臂会被反复怼在同一个坏位形上，围栏尽职地一次次刹车，日志一路滚过去，而没人看着。10 次够扛住探索期的偶发跳闸（LOG-040 是前两个 rollout epoch 内一次），又不足以让一个系统性坏掉的配置无人值守地跑一整夜。

### 41.4 调试与验收

新增 `b/x/scripts/test_t21_t22_recovery.py`（40 项，风格对齐 `test_t19_joint_demand.py`）。调试中只出过一个问题：

**问题**：第一版桩件用 `SimpleNamespace` 拼 `self`，跑到 `_recover_from_trip` 内部调 `self._warn_if_ill_conditioned` 时 `AttributeError`。
**根因**：被测方法之间有互相调用（`_recover_from_trip → _warn_if_ill_conditioned → _safe_state`），假 `self` 只有属性没有方法。
**修法**：改成真正 `class _StubEnv(FrankySingleFrankaEnvMixin)` + `object.__new__` 跳过 `__init__`（真 `__init__` 会拉起 Ray actor 并连臂），只手工塞 5 个属性。**这本身也是一项检查**：将来若恢复路径开始依赖只有 `FrankaEnv.__init__` 才提供的状态，测试会立刻炸而不是静默漂移。

```text
# franky 容器内
$ python b/x/scripts/test_t21_t22_recovery.py          →  40/40 ALL PASS
$ python b/x/scripts/test_t21_t22_recovery.py --robot  →  含真雅可比复核，ALL PASS
$ python b/x/scripts/test_t19_joint_demand.py --robot  →  回归，ALL PASS
$ python b/x/scripts/step_cube_place_dummy.py          →  Phase1A PASS（含新增常设闸门）
```

真雅可比复核（只读，未动臂）四个已记录位形**全部精确复现 LOG 里的数**，说明共享出来的 `jacobian_conditioning` 与原先脚本内联的算法等价：

```text
hover      (LOG-037, known-good): sigma_min=0.1115 (logged 0.1115), cond=17.0    → ok
recovered  (LOG-040 §40.2):       sigma_min=0.1141 (logged 0.1141), cond=16.8    → ok
trip       (LOG-036):             sigma_min=0.0062 (logged 0.0062), cond=310.7   → warn
drifted    (LOG-040 §40.1):       sigma_min=0.0009 (logged 0.0009), cond=2115.7  → warn
live: q=[0.0655, 0.6682, -0.1123, -1.2737, 0.1014, 1.9302, 1.0796] sigma_min=0.1100 → ok
```

最后一行是当前真机位形：**`sigma_min=0.1100`，与悬停参考值同档**，说明 LOG-040 §40.2 的手引导恢复至今仍然保持着，可以直接开训。

`step_cube_place_dummy.py` 同步加了常设闸门（异常类型、两个 `except` 子句、控制器恢复 API 存在、零雅可比静默、阈值夹在 0.0062 与 0.1115 之间），输出新增一行：

```text
trip recovery wired (budget 10 per env) and elbow self-check warns below sigma_min=0.0400
```

### 41.5 教训

62. **"没有 API 能做 X"往往意味着 X 曾被刻意设计成做不到——先弄清是不是，再决定加不加。** §41.1：跳闸闩锁没有解除入口不是遗漏，是 LOG-023 定下的"控制器只刹车 + 锁存"契约的一部分。所以加的是一个**显式命名、会拒绝、有前置条件**的恢复方法，而不是给 `set_motion_guard` 加个参数——凡是能解除安全闩锁的代码都应该 grep 得出来。
63. **自动恢复必须带预算，否则它就是"安全机制自己退场"的另一种写法。** §41.3：无上限的 recover-and-continue 会把一条反复跳闸的臂无人值守地磨一整夜。预算用尽时的日志要指向**工作区**（收 `clip_x_range`），而不是指向"把预算调大"。
64. **在错误路径里改奖励函数，是奖励函数开始名不副实的起点。** §41.3：跳闸给 0 分是因为那一步没执行完（不是因为它"该被罚"）；想罚要另行显式设计。
65. **桩件不要伪造被测对象的方法。** §41.4：`SimpleNamespace` 假 `self` 一遇到方法间互调就散架；用真类 + `object.__new__` 既能测到真实的 MRO 与方法绑定，又能把"这个路径依赖哪些状态"钉成一项检查。

### 41.6 状态

| 项 | 状态 |
|---|---|
| T21（跳闸后续训） | **已实现**：控制器 `recover_from_guard_trip()` + env 侧 `MotionGuardTripped` 捕获 + 预算上限；静态与桩件验收 40/40 通过，**真机触发验证待下次开训** |
| T22（肘部位形自检） | **已实现**：`reset()` 开头与每次恢复后各一次 `sigma_min` 自检，只预警不拒绝；四个已记录位形的真雅可比复核全部精确复现 |
| 回归 | T19 验收（含 `--robot`）、1A dummy 全过，未破坏既有路径 |
| 机器人 | 未动臂。当前位形 `sigma_min=0.1100`（良态，与悬停同档），LOG-040 §40.2 的手引导恢复仍然保持 |
| 集群 | 本次未触碰；开训前照例先 `verify_ray_cluster.sh` + `preflight_cube_place_sac.sh` |
| 下一步 | 重开训练，让 T21/T22 在真机上各被触发一次：reset 前应出现 `joint configuration OK before reset: sigma_min=...`；万一再跳闸应出现 `guard trip recovered (1 of 10 used)` **且训练继续** |

---

## LOG-042 — 第四次真机开训：T21/T22 上线后首个稳定训练（2026-08-20）

**触发**：用户指示"开训，重启 ray 集群，执行 §S3.5 启动训练与前五分钟要盯的"。这是 T21/T22（§S3.12）落地后的第一次真机训练，也是它们第一次有机会在真实训练里被触发。

### 42.1 前置：FCI 曾短暂掉线，自行恢复

上一轮验收收尾时（LOG-041 末）`franky.Robot` 连接报 `Connection to FCI refused`，本次开训前复查已自行恢复为 `RobotMode.Idle has_errors=False`——Desk 侧的 FCI 会话状态变了，期间无人下发运动、无安全后果。开训前确认通过。

### 42.2 执行序列（全部按 §S3.2 / §S3.5）

1. **重启 Ray 集群**（§S3.2）：两容器各自 `ray stop --force` → GPU 容器 `ray start --head`（rank 0）→ franky 容器 `ray start --address`（rank 1），两边都带 `--num-cpus=16 --object-store-memory=4000000000`（LOG-039 的 OOM 教训）。
2. **集群验收**（§S3.2 末）：`verify_ray_cluster.sh` 14 项全 PASS（rank 捕获、双节点 alive、解释器、gym id、端到端钉节点、ResNet10 权重）。
3. **训练前检查**（§S3.5 第 0 步）：`preflight_cube_place_sac.sh` 三层全 PASS——A 静态（H1 标定与 YAML 逐位一致、方块宽度 0.0325 已钉、相机 serial 以 YAML 为锚、第二只插着的相机按设计忽略）、B 集群、C 机器人（无存活控制器 actor、1337 空闲、`connect` 三道硬门、authority 回显 20N/axis）。**无任何自动修复被触发**。
4. **开训**（§S3.5 第 1 步）：GPU 容器 `bash b/x/scripts/run_cube_place_sac.sh realworld_cube_place_sac`。

### 42.3 前五分钟九条逐项核对（§S3.5 表）

| # | 期望 | 实测 |
|---|---|---|
| 1 | `RLinf is running on a cluster with 2 nodes` | ✅ |
| 2 | env worker 落 franky 容器、actor/rollout 落 GPU 容器 | ✅（`EnvGroup(rank=0) pid=169017` 在 franky 侧） |
| 3 | `FrankaLibfrankaGripper connected (... cube_width=<实测值> ... holding=True)` | ✅ `cube_width=0.0325m +/-0.0120m [raw_env FRANKA_CUBE_WIDTH_M=0.0325] holding=True width=0.0234m`——**T17 的 env_vars 这次真的到了控制器**（LOG-034 首跑这里是 0.046/holding=False） |
| 4 | `authority: ... 20.0N/axis` + `collision behavior tightened: 40.0/12.0` | ✅ |
| 5 | `motion guard armed` + `motion guard confirmed` + `reach:` | ✅（`max_dq=1.200rad/s` 也在） |
| 6 | 第一段运动是抬到悬停、`rest pose reached` | ✅ `rest pose reached on attempt 2 (err=0.0030m)` |
| 7 | `env/reward` 非恒 0 | ✅ `reward=0.7412486 return=74.12`（dense reward 生效） |
| 8 | 每回合约 100 步自动 reset、方块始终夹着 | ✅ `episode_len=100.0`，holding 全程 True |
| 9 | 策略接手头 30 秒无 `WATCHDOG trip` | ✅ **零跳闸**；`step slew clamped` 仅 1 次（偶发，限幅在边缘工作，未刷屏） |

### 42.4 T22 自检首次在真机训练里按预期工作

每次 reset 前都打出了新日志，且全部良态、无一次预警：

```text
joint configuration OK before reset: sigma_min=0.1225 (>= 0.0400), cond=15.6
joint configuration OK before reset: sigma_min=0.1355 (>= 0.0400), cond=14.1
joint configuration OK before reset: sigma_min=0.0933 (>= 0.0400), cond=20.6
joint configuration OK before reset: sigma_min=0.0964 (>= 0.0400), cond=19.9
joint configuration OK before reset: sigma_min=0.0962 (>= 0.0400), cond=20.0
joint configuration OK before reset: sigma_min=0.0989 (>= 0.0400), cond=19.4
joint configuration OK before reset: sigma_min=0.0890 (>= 0.0400), cond=21.6
```

`sigma_min` 全程在 0.089–0.136 之间，远高于 0.04 的预警线，也高于 LOG-040 漂移前的水平——**肘部没有再往坏方向漂**。T21 的恢复路径本次未被触发（零跳闸），属预期：它只在跳闸时才运行。

### 42.5 状态

| 项 | 状态 |
|---|---|
| 训练 | 稳定运行中，reward 有输出、无回归性跳闸、无限幅刷屏 |
| T21 | 已上线待命，本次未触发（零跳闸） |
| T22 | **已真机验证**：每次 reset 前自检正常打印、全部良态、无误报 |
| T17 | **本次顺带闭环**：`raw_env FRANKA_CUBE_WIDTH_M=0.0325` 与 `holding=True` 同时正确，env_vars 确实到了控制器 |
| 机器人 | 全程安全，无跳闸、无 Desk fault |
| 下一步 | 让训练继续跑；盯 `guard trip recovered (N of 10 used)` 的计数（预算被稳定消耗 = 该收工作区）与 `sigma_min` 是否缓慢下行（肘部漂移的早期信号） |

---

## LOG-043 — 第四次开训正常跑完 200 epoch：return 明显上升、9/20 回合碰到标记、全程零跳闸（2026-08-20）

**触发**：训练进程退出（exit 0）。初判"中途停了"，核查后确认是**正常跑完**——终端捕获文件在 162 步被显示缓冲截断，权威日志 `run_embodiment.log` 记录 200/200 步、100%，`global_step_200` 检查点已存。**教训：判断训练是否结束要看 `logs/<时间戳>/run_embodiment.log`，不要看终端捕获文件。**

### 43.1 结果（200 epoch，约 8 分钟真机）

| 指标 | 值 | 读法 |
|---|---|---|
| 完成度 | 200/200 步，100% | 正常结束，非中断 |
| 检查点 | `global_step_50/100/150/200` 四个 | `save_interval=50` 按预期工作 |
| **return 趋势** | 37.8 → 3.7 → 23.6 → 65.0 → … → **88.7 / 89.8 / 91.3 / 90.7 / 88.5 / 87.7** → 末回 66.6 | **明显上升后稳定在高位**：前 7 回合均值约 39，第 10–18 回合均值约 86，策略确实在学 |
| **success_once=1** | **9 / 20 回合** | 近半数回合至少碰到过一次标记；`success_at_end` 仍多为 0（碰到≠稳住，符合早期 SAC） |
| 跳闸 / 恢复 | **0 次** | T19 两道界 + T21 恢复全程未被触发；T22 自检每次 reset 前正常打印 |
| `sigma_min` 范围 | 0.0890 – 0.1362 | 全程远高于 0.04 预警线，**肘部无漂移** |
| replay buffer | 20 回合 / 2000 样本（`cache_size=100` 远未满） | 真机 10 Hz 下的正常积累速度 |

### 43.2 与前三次开训的对比

| | LOG-034 | LOG-036 | LOG-040 | **LOG-043** |
|---|---|---|---|---|
| 存活时间 | 第 10 秒跳闸 | 第 ~205 秒跳闸 | 第 2 个 rollout epoch 跳闸 | **跑完 200 epoch 全程** |
| 结局 | `[lag]` 跳闸杀训练 | `[lag]` 跳闸杀训练 | `[watchdog:dq]` 跳闸杀训练 | **正常结束，无跳闸** |
| return | — | — | 有输出但中断 | **37.8 → ~90，9/20 碰到标记** |

T19（关节需求限幅 + `|dq|` 看门狗）+ T21（跳闸恢复）+ T22（肘部自检）四层叠加后，训练第一次既能**跑完**又能**学到东西**。T21 本次未被触发（零跳闸），其真机验证仍待一次真实跳闸。

### 43.3 状态

| 项 | 状态 |
|---|---|
| 训练 | **正常跑完 200 epoch**，return 37.8→~90，9/20 回合碰到标记 |
| 检查点 | `logs/20260820-041122-realworld_cube_place_sac/cube_place_sac/checkpoints/global_step_200/`（可续训） |
| T19/T22 | 真机验证通过（限幅偶发工作、自检每次 reset 正常、零跳闸、无肘部漂移） |
| T21 | 已上线待命，**真机触发验证仍缺**（本次零跳闸） |
| 机器人 | 全程安全，无跳闸、无 Desk fault |
| 集群 | 仍存活，可直接续训 |
| 下一步 | ① 续训更久（`max_epochs` 从 200 往上抬，`runner.resume_dir=<global_step_200 目录>`）观察 return 是否继续涨、`success_at_end` 是否出现；② 若再跳闸，确认 T21 的 `guard trip recovered` 后训练**继续**而不是死掉 |
