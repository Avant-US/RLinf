# 方块放置（cube place）落地手册 v2

**这一版是可执行的运行手册**，不是设计稿。设计结论已在 v1 锁定并通过阶段 1；v2 的改动全部来自真机踩坑：`robot_mode` 总闸、ROS 与 franky 两套阻抗的本质差异、「哑失败」如何变成可见错误，以及（LOG-019 之后）**运动权限如何被两个独立默认值悄悄放大四倍**。

| | |
|---|---|
| 上一版 | [`dmo_place_1.md`](dmo_place_1.md)（设计推导、charger 拆解，仍可查） |
| 运行记录 | [`dmo_place_1LOG.md`](dmo_place_1LOG.md)（LOG-001…LOG-017）、[`dmo_place_2LOG.md`](dmo_place_2LOG.md)（LOG-018 起） |
| 人机 / 急停协议 | [`charger_sac_async.md`](charger_sac_async.md) §6.2.1、§15.5 |
| 代码 | `b/x/`（**未改** `rlinf/`） |

**当前状态**

| 阶段 | 状态 |
|------|------|
| 阶段 1A `gym.make` dummy | **PASS**（已扩展为覆盖力上限/限速/围栏/起始位形/配置一致性/围栏锁存后拒绝下发等 **18 组不变量**，改过 `b/x/` 必跑） |
| 阶段 1B GPU dummy SAC | **PASS**（LOG-021 后重跑两次） |
| 阶段 2.5 H1 标定 | **PASS**，`calibrated: true` 已落盘（**标记挪过、已重标定两次**，当前值见 §3.2） |
| 阶段 2.6 `connect` 只读几何 | **PASS** |
| 阶段 2.4 运动链路体检（hold + ramp） | **PASS**（LOG-018 只覆盖 1 cm/s / 3 cm / x≈0.55 空载；LOG-034「围栏 lag 跳闸 + 2.4b」已在**真实位形**上补齐 1–3 cm/s、3–10 cm、10 Hz 阶梯、以及一次 10.9 cm 受控下降） |
| **阶段 2.4b 速度/幅度阶梯 + 阶梯复现** | **PASS**（LOG-034）。六档全过，`peak_overshoot` 实测**全部 0.0000**（门闩线 ≤0.02），最差 `peak_lag=0.0092`、`peak\|dq\|=0.112`、`peak\|F_ext\|=10.7N`。**T5 关闭**；但 `--test-waypoints` **没有复现** LOG-019 的 26 cm 超调（T11 仍未定，假说空间已缩小），且四档**全程不带旋转**——旋转阶梯（`--test-rotation[-waypoints]` + `run_2_4b_rotation_ladder.sh`）已在 LOG-035 补上工具，**真机未跑**，见 §7 T16/T18 |
| 阶段 2.7 `reset` 到悬停 | **PASS**（LOG-026）。此前 FAIL 两次：① LOG-019 冲高 36 cm（安全事故）；② LOG-022 差 4.2 cm，根因是 slew clamp 掐死插值，已修 |
| 阶段 2.8 `box` 短跑 | **PASS**（LOG-026，与 2.7 同一次 `box` 跑完：`rest pose reached on attempt 2`、`hover check` 双项达标、零动作稳定、下探 −1.06 cm 未触底、全程无 `motion guard abort`） |
| 阶段 3 在线 SAC | **已真机三次跳闸**（LOG-034 第 10 秒 `[lag]`；LOG-036 第 ~205 秒 `[lag]`；LOG-040 训练中 `[watchdog:dq]`），前两次根因是**雅可比放大导致的关节速度需求爆炸**（LOG-037），已由 T19 的两道界（命令侧关节需求限幅 + 看门狗 `\|dq\|` 判据）解决并真机验证通过（LOG-038/039）。**第三次开训已跑通训练循环**（reward 有输出），第 2 个 rollout epoch 中 `[watchdog:dq]` **按设计**清洁刹停（LOG-040）——但这次暴露了两个新问题：① 纯阻抗控制会让肘部关节位形悄悄漂移到病态（无自动检测，本次靠手引导恢复，T22）；② 跳闸异常会杀死整条训练进程而不是被 env 捕获后继续（T21）。机器人本身全程安全。**T21/T22 已按 §S3.12 实现并通过 40 项验收（LOG-041），真机触发验证待下次开训** |

---

## 0. 怎么用这份文档

- **第一次上手：** 读 §1（本机基线）→ §2（任务）→ §4 按阶段做。
- **今天就要跑 reset：** 读 §1.4 + §1.5 + §4 阶段 2 全节，一步不跳。
- **出错了：** 直接查 §6 排障总表（按症状索引）。
- **写代码前：** 读 §3，那里写的是**代码现在的实际行为**，不是计划。
- **要开阶段 3（在线 SAC）：** 读 §4 阶段 3 的 §S3.0–§S3.10。它的小节号带 `S` 前缀（S = Stage），是为了跟第 3 章的 §3.1–§3.4 区分开——文档里不带 `S` 的 `§3.x` 一律指第 3 章。

**命令的执行环境（每个 bash 代码块的第一行注释都会标）：** 这份文档里只有三种执行环境——**宿主机**（跑 `docker` / `ping` / `ss` / `tensorboard` 这类命令，不需要任何 venv）、**franky 容器**（`rlinf-franky-5090`，跑一切碰机械臂的命令；每个新 shell 先 `source b/x/configs/setup_before_ray_5090.sh`）、**GPU 容器**（阶段 1B 与阶段 3 的训练侧，跑 CUDA torch；每个新 shell 先 `source b/x/configs/setup_before_ray_gpu_5090.sh`）。阶段 3 起，同一条流程里会交替用到三种环境，**看错环境标签是阶段 3 最常见的操作错误**（在宿主机跑 `ray start`、在 GPU 容器跑 REPL、在 franky 容器跑 `train_async.py` 都是真实发生过的错误类型）。

四条铁律，违反其一必然浪费一场（第 4 条是 LOG-019 用一次 36 cm 冲高换来的）：

1. **动臂之前先确认 `robot_mode == RobotMode.Idle`**（§1.4）。
2. **franky 上任何「命令发了但臂不动」，第一步查模式和 `is_running`，不是改几何**（§1.3）。
3. **同一时刻只能有一个 libfranka 客户端**（标定 REPL / 烟测脚本 / 训练三者互斥）。
4. **危险的量永远是两个量的乘积。** `刚度 × 误差截断` 是**力**；`位移 ÷ 时长` 是**速度**。永远不要让一对里的一个来自配置、另一个来自库默认值——那正是 100 N 和 10 cm/s 的来历（§1.5）。**「体检 PASS」只对体检覆盖过的速度和幅度有效。**

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

`source b/x/configs/setup_before_ray_5090.sh` 做的事（每开一个新 bash 都要重做）：`switch_env franky-0.19.0`、`PYTHONPATH` 加 `b/x`、`PYTHONSTARTUP` 注册 franky Gym ID、`RLINF_EXT_MODULE=franky_ext.runtime_bootstrap`、`RLINF_COMM_NET_DEVICES=eno2`、`FRANKA_ROBOT_IP=172.16.0.2`、`FRANKA_GRIPPER_TYPE=franka`，**以及全套运动权限与夹爪环境变量**（`RLINF_CUBE_*`、`FRANKA_GRASP_FORCE` / `FRANKA_CUBE_WIDTH_M` / `FRANKA_HOLD_TOL_M`，见 §3.2）。它们都用 `${VAR:-默认}` 写法，所以**你先 export 的值不会被覆盖**；末尾会把力上限、限速、围栏余量、夹爪三项回显出来——开跑前扫一眼这两行，比事后翻日志便宜得多。

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
# franky 容器内（已 source setup_before_ray_5090.sh）
python b/x/scripts/diag_franky_motion.py --probe
```

### 1.5 相反的失效模式：运动权限被两个默认值放大（LOG-019）

§1.4 讲的是「命令了但不动」。**它有一个完全相反的孪生失效，而且更危险：动得远超命令值。** 2026-08-19 一次 `reset` 命令抬升 10 cm，臂实际到了**最高指令值上方 26 cm**（相对起点 +36 cm，超安全盒上沿 27 cm），软件里没有任何一处叫停，最后是操作员拍 user-stop 制动的。

两个「合理的默认值」相乘出了没人决定过的权限：

| 量 | 来自哪里 | 值 |
|---|---|---|
| `translational_stiffness` | PegInsertion 的 `compliance_param`，每次 `reset` 下发 | 500 → **2000** N/m |
| `translational_error_clip` | franky 模块默认值，**没跟着改** | 0.05 m |
| **乘积 = 逐轴指令弹簧力上限** | — | 25 N → **100 N**（franky 自己的配对是 500×0.05 = 25 N） |
| `rotational_stiffness × rot_clip` | 同理 | 12 → **45 N·m**（j5–j7 关节限值仅约 12 N·m） |

同时速度也被放大了十倍：`_interpolate_move(pose, timeout=1)` 在 `step_frequency=10` 下把 10 cm 切成 10 个路点、每 100 ms 一个 = **10 cm/s**，而当时唯一验证过的速度是 1 cm/s。

> ⚠️ **机理仍未确定，不要把上面两条当成已证实的原因。** 早期版本的本节声称「滞后线性外推到 4.7 cm → 吃满 clip → 全程输出 100 N」。读 franky 的 `cartesian_impedance_base.cpp` 之后这个推论被证伪：滞后由**摩擦**主导（实测 9.4 N 里约 8 N 是静摩擦，与速度无关），10 cm/s 时滞后只有约 11.7 mm，**远未吃满 50 mm 的 clip**。
>
> 100 N 是**可用权限**，不是**实际输出**。真正把臂送上去 26 cm 的机理（更像是 10 Hz 零阶保持目标 + 高刚度 + 极软零空间 + 伸展位形下的失稳）**仍未确定**，这正是 §4 阶段 2.4b 的 `--test-waypoints` 要回答的问题。护栏是防御纵深，不是「已经找到原因了」。

**还有两件事，`刚度 × clip` 并**不**界定：**

| | |
|---|---|
| clip 是**逐轴**的（franky 用 `cwiseMax/cwiseMin`） | 三轴同时饱和时指令力是 `K·clip·√3`。所以现在同时设**逐轴**上限和**范数**上限，取先约束者。旧版只按逐轴设 10 N·m，worst case 达 17.3 N·m，**反而超过了当初用来定这个值的 12 N·m 关节限值** |
| 阻尼项 `−D·(twist)` **完全不被裁剪** | franky 的阻尼是由刚度推导的临界阻尼（`K_t=2000` 时约 155 N·s/m）。因为 `desired_twist` 恒为 0，它**只会阻碍运动**，不会驱动超程——但这意味着人手拦住臂时感受到的峰值力**不受这些数字约束**，而且**抬高刚度会以 √K 抬高阻尼，无论 clip 怎么推导**。推导 clip **不能**让 `K_t=2000` 等价于 `K_t=500` |

`UserStopped` 阴险在「只挡运动」；这一条阴险在**几何全对、日志全绿**：

| 现象 | 值 |
|---|---|
| `robot_mode` / `has_errors`（开跑前） | `Idle` / `False`（门闩正确放行） |
| H1、hover、盒子打印 | 全部正确 |
| 夹爪 | `holding=True width=0.0470m`，`skip grasp` 正常 |
| wrapper 栈 | 6D 闭爪，正确 |
| `cube_place go_to_rest` 日志 | 目标 `0.3254`，正确 |
| 实际到达 | **`0.5841`** |

**为什么没有任何东西拦住它：** 上游 `FrankaEnv._interpolate_move` 是**纯开环定时**——`linspace` 的起点是开跑前采样一次的位姿，之后只按 `time.sleep` 发目标，从不回读臂在哪，而且**不按 `ee_pose_limit` 裁剪**（裁剪只发生在 `step()`）。`go_to_rest` 末尾 `np.allclose(tcp, reset_pose, 0.02)` 里的 `0.02` 传给的是 **rtol 而非 atol**，那是收敛判据，不是运行中的安全监控。

**现在有什么在拦：** 见 §3.4。控制器进程内的围栏会在测量 TCP 越出 `ee_pose_limit ± margin` 时立刻**刹车并锁存原因**（env 侧轮询后抛出，见 §3.4 B）；软件力上限恒定 20 N/轴（范数 40 N）；libfranka 硬件反射也从 100 N 降到 40 N；插值速度封顶 2 cm/s、`step()` 封顶 5 cm/s；位移超 0.35 m、转角超 0.6 rad 直接拒绝；50 Hz 看门狗覆盖路点之间的空档。

围栏顶在**盒顶上方 8 cm**（= 余量 0.05 + `reset_z_lift_m` 0.03），绝对天花板再高 2 cm（盒顶 + 0.10）。事故那次臂到了盒顶上方约 **27 cm** 才被人拍下 —— 即围栏会比人**早约 19 cm** 叫停。按现在这份 H1，这两个数分别是 **0.3895 m** 与 **0.4095 m**（LOG-022 的 `motion guard armed` 已回读确认）。

> **教训写在这里免得再犯：** 凡是改 `K_t`，必须同时看 `K_t × clip`。凡是「体检通过了」，必须问「在什么速度和幅度下通过的」。

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
| [`b/x/franky_ext/motion_limits.py`](../../../b/x/franky_ext/motion_limits.py) | **LOG-020 新增**：力/力矩上限、插值速度上限、位移上限、围栏余量集中在这里。`error_clips_for_stiffness` 让 `K×clip` 恒定（逐轴 20 N，范数 40 N）；`interp_duration_s` 让**速度**而非时长成为常量。纯算术，1A 覆盖 |
| [`b/x/franky_ext/controller_extended.py`](../../../b/x/franky_ext/controller_extended.py) | `FrankyControllerExtended`：`move_arm`、`reconfigure_compliance_params`、`move_gripper`、Franka Hand、**tracker 存活检查**、**运动围栏**（`set_motion_guard` / `freeze_at_current` / `motion_health` / `gripper_holding` / **`guard_tripped`**）、50 Hz 看门狗、收紧 libfranka 碰撞阈值 |
| [`b/x/franky_ext/franka_libfranka_gripper.py`](../../../b/x/franky_ext/franka_libfranka_gripper.py) | 原装 Hand：轻力 `grasp`（20 N，上限 40 N）、已夹持则 `skip grasp` |
| [`b/x/franky_ext/franky_single_franka_env.py`](../../../b/x/franky_ext/franky_single_franka_env.py) | mixin：换控制器、`safe_smoke_hold`、跳相机、**插值限速 + 位移上限 + 装围栏**、`clear_error_per_waypoint` 旋钮 |
| [`b/x/franky_ext/tasks/cube_place.py`](../../../b/x/franky_ext/tasks/cube_place.py) | `CubePlaceConfig` + `FrankyCubePlaceEnv.go_to_rest` + `_check_start_pose` |
| [`b/x/franky_ext/tasks/register.py`](../../../b/x/franky_ext/tasks/register.py) | 注册 `FrankyCubePlaceEnv-v1` / `FrankyPegInsertionEnv-v1` / `FrankyFrankaEnv-v1` |
| [`b/x/franky_ext/tcp_probe.py`](../../../b/x/franky_ext/tcp_probe.py) | 子进程读 pose + **`robot_mode`** + q + **夹爪 width/holding** + `cmd_success_rate`；`require_motion_ready` / `describe_robot_mode` / **`check_start_pose`** |
| [`b/x/configs/env/realworld_cube_place.yaml`](../../../b/x/configs/env/realworld_cube_place.yaml) | Hydra env 包，`id: FrankyCubePlaceEnv-v1` |
| [`b/x/configs/realworld_cube_place_dummy_sac_gpu.yaml`](../../../b/x/configs/realworld_cube_place_dummy_sac_gpu.yaml) | 阶段 1B GPU dummy SAC |
| [`b/x/configs/cube_place_target_ee_pose.yaml`](../../../b/x/configs/cube_place_target_ee_pose.yaml) | H1 六元组，**已 `calibrated: true`** |
| [`b/x/scripts/diag_franky_motion.py`](../../../b/x/scripts/diag_franky_motion.py) | **v2 新增**：绕开 Ray/env 直连 franky 判定运动链路 |
| [`b/x/scripts/test_franky_controller_ext.py`](../../../b/x/scripts/test_franky_controller_ext.py) | 标定 REPL（`open`/`close`/`getpos_euler`/`impedance`…） |
| [`b/x/scripts/write_cube_place_pose.py`](../../../b/x/scripts/write_cube_place_pose.py) | 把六个数写入 H1 YAML 并置 `calibrated: true` |
| [`b/x/scripts/step_cube_place_robot.py`](../../../b/x/scripts/step_cube_place_robot.py) | 真机烟测：`--connect-only` / `--reset-only` / 默认 box |
| [`b/x/scripts/step_cube_place_dummy.py`](../../../b/x/scripts/step_cube_place_dummy.py) | 阶段 1A 源码门闩 |
| [`b/x/scripts/run_cube_place_phase2.sh`](../../../b/x/scripts/run_cube_place_phase2.sh) | 子命令封装 `calibrate / write-pose / connect / reset / box / diag-probe / diag / diag-replay`，多余参数一律透传给 python；自己 `export REPO_PATH`，不依赖调用者的环境 |

### 3.2 关键默认值（与代码逐一对齐）

`CubePlaceConfig`（继承 `PegInsertionConfig`）：

| 键 | 值 | 说明 |
|----|-----|------|
| `clip_x_range` / `clip_y_range` | `0.05` | 安全盒 xy 半宽 |
| `clip_z_range_low` | `0.005` | 只允许比接触点再低 5 mm |
| `clip_z_range_high` | `0.08` | 悬停高度 = 盒顶 |
| `random_xy_range` | `0.03`（烟测里被覆盖为 `0.0`） | |
| `clip_rz_range` / `random_rz_range` | `0.35` | |
| `reset_z_lift_m` | **`0.03`** | **相对当前 TCP** 的抬升，≠ `clip_z_range_high`。0.03 而非 PegInsertion 的 0.10：charger 抬 10 cm 是为了**拔插头**，本任务只需让方块脱离平面标记（LOG-019） |
| `reward_threshold[:3]` | `0.01`（基类） | 「碰到」容差 |
| `compliance_param.translational_stiffness` | `2000`（基类 PegInsertion） | reset 时 `reconfigure` 下发，**会被 `clamp_stiffness` 限在 ≤3000** |
| `compliance_param.translational_damping` | **被忽略并 warn** | ROS 控制器的阻尼系数（N·s/m），franky 无对应项。旧的 `tc = 2·d/k` 映射已删除（LOG-019 R4） |
| `clear_error_per_waypoint` | `True`（= 上游行为） | 路点间是否 `recover_from_errors()`。**尚未定论**，旋钮留给测量（T8） |
| `safe_smoke_hold` | 数据类里是 `False`，**env 包 `env/realworld_cube_place.yaml` 钉成 `True`** | 只跳过 `FrankaEnv.__init__` 自己那次 `_interpolate_move(reset_pose)`（从臂**当前所在**的任意位置出发的一次运动）；`reset()` 照常动臂。凡是 `defaults:` 里带这个 env 包的配置都继承 `True` |
| `use_dense_reward` | **`True`**（env 包 + 烟测脚本都显式给） | `FrankaRobotConfig` 默认 `False`，而 `_calc_step_reward` 在 `False` 时**除进区的 1.0 之外一律返回 0.0**——本任务没有别的奖励代码。漏了它的话每一步都打印 `reward=0.0000`，和「奖励链路坏了」长得一模一样，阶段 3 的 SAC 也会被喂恒零信号 |

**运动权限（`b/x/franky_ext/motion_limits.py`，环境变量可覆盖但都会被夹到硬范围内）：**

| 量 | 默认 | 环境变量 | 硬范围 |
|---|---|---|---|
| **逐轴指令弹簧力上限** `K_t × clip` | **20 N** | `RLINF_CUBE_FORCE_CEILING_N` | [5, 60] |
| **力范数上限**（三轴同时饱和） | **40 N** | `RLINF_CUBE_FORCE_NORM_CEILING_N` | [10, 120] |
| **逐轴指令力矩上限** `K_r × rot_clip` | **6 N·m** | `RLINF_CUBE_TORQUE_CEILING_NM` | [1, 15] |
| **力矩范数上限** | **12 N·m**（= j5–j7 关节限值） | `RLINF_CUBE_TORQUE_NORM_CEILING_NM` | [4, 30] |
| **libfranka 硬件反射阈值** | 笛卡尔力/力矩降到 **40 N / 12 N·m**（上游 100 N / 25 N·m） | 由上面两个范数上限推出 | — |
| 插值速度上限（reset 路径） | **2 cm/s** | `RLINF_CUBE_INTERP_SPEED` | [0.2, 6] cm/s |
| **`step()` 速度上限（策略路径）** | **5 cm/s** → `action_scale[0]` 被夹到 **0.005 m/步** | `RLINF_CUBE_STEP_SPEED` | [0.2, 20] cm/s |
| 插值单次位移上限 | **0.35 m**（超过直接**拒绝**） | — | — |
| 插值单次转角上限 | **0.6 rad**（超过直接**拒绝**） | — | — |
| 围栏余量（测量 TCP 可越出盒子多少） | **0.05 m** | `RLINF_CUBE_GUARD_MARGIN` | [0.01, 0.15] |
| **围栏下沿余量（−z）** | **0.01 m**（桌面在那儿，不能给 5 cm） | `RLINF_CUBE_GUARD_FLOOR_MARGIN` | [0.002, 0.05] |
| 围栏绝对 z 天花板 | 盒顶 + **0.10 m**（不随 `reset_z_lift_m` 变动） | `RLINF_CUBE_Z_CEILING` | — |
| 围栏跟随上限 `|测量−指令|` | **0.05 m** | `RLINF_CUBE_GUARD_MAX_LAG` | [0.02, 0.20] |
| 姿态围栏 | `√(0.01²+0.01²+clip_rz²) + 余量` ≈ **0.550 rad** | `RLINF_CUBE_GUARD_ORIENT_SLACK` | [0.05, 0.80] |
| 刚度硬夹 | `K_t ∈ [50, 3000]`，`K_r ∈ [5, 300]` | — | — |
| 夹爪抓取力 / 标定方块宽度 | **20 N** / **0.046 m ± 0.012** | `FRANKA_GRASP_FORCE` / `FRANKA_CUBE_WIDTH_M` | [5,40] / [0.005,0.070] |

**`0.046 m` 只是占位默认值，不是任何一块真实方块的实测值**（LOG-022 实测 0.0365 m，LOG-024 实测 0.0325 m，LOG-026 实测 0.0316 m，都明显更小，而且三次彼此也不同——夹取角度会影响读数）。每换一个物理方块 / 每次 H1 重标定都要重新量一次，见 §2.5 步骤 1 的说明；不重新量、直接用默认窗口大概率触发 `grasp did not capture the cube`（详见 §6）。

**这些环境变量怎么才能到 actor 里**（早期版本这里写成「必须在 `ray start` 之前导出」，过强了）：

| 设法 | 到不到 actor | 说明 |
|---|---|---|
| `ray start` **之前** export（`setup_before_ray_*.sh` 干的事） | **到** | Ray actor 继承 raylet 的环境。这是训练路径唯一可靠的办法，因为 `train_async` 那条链路不会替你改 `os.environ` |
| 烟测脚本的 CLI 旗标（`--force-ceiling` 等） | **到** | 脚本在 `ray.init` **之前**写 `os.environ`；RLinf 会把 driver 环境与 raylet 环境**做差**（[`node.py::_configure_node_envs`](../../../rlinf/scheduler/cluster/node.py)），把差异并进 `node.env_vars`，再在 `Cluster.allocate` 里作为 `runtime_env["env_vars"]` 下发。**不是**靠继承 |
| `ray start` **之后**在 shell 里 export，然后跑训练启动器 | **到**（同上，因为启动器是 driver） | 但**别依赖它**：`setup_before_ray_*.sh` 会回显这些值，绕过它就没人回显，出事时无从对账 |

结论：**改了权限旗标就 `ray stop` → 重新 `source` → `ray start`**，这条操作纪律不变（省得去推理走的是哪条路），但「CLI 旗标没生效」不该再是第一嫌疑。

**哪些量有 CLI 旗标**（`step_cube_place_robot.py` / `diag_franky_motion.py`，`run_cube_place_phase2.sh` 会透传）：`--force-ceiling`、`--force-norm-ceiling`、`--torque-norm-ceiling`、`--interp-speed`、`--guard-margin`、`--guard-max-lag`、`--cube-width`。**没有**旗标的（只能用环境变量）：逐轴力矩上限 `RLINF_CUBE_TORQUE_CEILING_NM`、`RLINF_CUBE_STEP_SPEED`、`RLINF_CUBE_INTERP_SPEED_RAD`、`RLINF_CUBE_GUARD_FLOOR_MARGIN`、`RLINF_CUBE_Z_CEILING`、`RLINF_CUBE_GUARD_ORIENT_SLACK`、`FRANKA_GRASP_FORCE`。

`clip` **不是独立旋钮**：由 `力上限 ÷ 刚度` 算出（`K_t=2000` → `clip=0.01 m`），且同时满足范数上限。**低刚度时 clip 会被下界夹住而达不到设定力上限**（`K_t=50` 时实际只有 2.5 N，臂克服不了静摩擦 → 看起来正常其实不动），所以 `clip_shortfall()` 会在这种情况下 warn。

为什么 20 N/轴够用：LOG-018 实测 1 cm/s 下滞后 4.7 mm，即只用了约 9.4 N（其中约 8 N 是静摩擦，与速度无关）；2 cm/s 下约 11 N，仍小于 `20/2000 = 10 mm`…… 边际很薄，**若 2.4b 出现「抬不动」，用 `--force-ceiling 30` 复测，不要改代码**。

**姿态围栏的三个半宽不相等**：`PegInsertionConfig.__post_init__` 把 roll/pitch 钉在 **±0.01 rad**，只有 yaw 拿到 `clip_rz_range`。早期版本假设三轴都是 `clip_rz`，算出的围栏比自己的推理宽 1.5 倍。

franky 侧其余参数（`franky_controller.py` 模块常量）：

| 常量 | 默认 | 环境变量 | 说明 |
|------|------|----------|------|
| 平移 / 旋转刚度 | `500` / `40` | `RLINF_CART_K_T` / `_K_R` | **reset 会被 `compliance_param` 改成 2000 / 150**，所以这两个环境变量对 reset 无效 |
| 零空间刚度 | `5` | `RLINF_CART_K_NS` | 很软，肘部近乎自由（事故中 z+36 cm 伴随 x−6 cm 的圆弧就是这个） |
| 误差截断 | 已由刚度推导 | ~~`RLINF_CART_ERR_CLIP_M`~~ | **不再使用**，改用 `RLINF_CUBE_FORCE_CEILING_N` |
| 每周期力矩增量 | `0.3` Nm | `RLINF_CART_MAX_DTAU` | |
| 单次调用最大目标步进 | `0.10` m / `0.30` rad | `RLINF_CART_MAX_STEP_M` / `_RAD` | 相对**上一个目标**，不是相对测量值 |
| `relative_dynamics_factor` | `0.2` | — | 只影响阻塞 `robot.move`，不影响阻抗 |

已标定的 H1（[`cube_place_target_ee_pose.yaml`](../../../b/x/configs/cube_place_target_ee_pose.yaml) 的现存内容，**LOG-025 之后又重标定过一次，下面是当前值**）与由它推出的几何（数字取自 LOG-026 那次 `box` 的实际打印，不是手算）：

```
target_ee_pose (H1 接触点) = [0.726857066, 0.0249468647, 0.250846744, -3.13747483, 0.0354833569, -0.00268804519]
reset_ee_pose (悬停)       = H1 + [0,0,0.08,0,0,0]  →  z = 0.3308
ee_pose_limit_min          = [0.6769, -0.0251, 0.2458, -3.1475, 0.0255, -0.3527]
ee_pose_limit_max          = [0.7769,  0.0749, 0.3308, -3.1275, 0.0455,  0.3473]
围栏（+ 余量）             = xyz in [[0.6269, -0.0751, 0.2358], [0.8269, 0.1249, 0.4108]]，绝对天花板 0.4308
# roll/pitch 半宽是硬编码 0.01 rad（不是 clip_rz），只有 yaw 拿到 clip_rz=0.35（见上一段）
```

**这组数字比上一次标定更靠外**（`x` 从 0.6713 挪到 0.7269），所以 T5（伸展位形权限）不是改善而是变紧了：可达域体检里 `connect` 报的**盒角**是 `reach: worst box corner [0.7769, 0.0749, 0.2458] r=0.785m (92% of 0.855m reach) NEAR-SINGULAR`，装围栏时（`gym.make` 内）再报的**围栏角**是 `r=0.842m`（**98%，同样 `NEAR-SINGULAR`**）。两处都已带标记 —— 围栏一旦触发，**绝对不要**靠「把围栏放宽一点」化解，那个方向上臂本来就没有余量了；要挪的是标记，不是围栏。

### 3.3 `reset` 的确切三段行为

`FrankyCubePlaceEnv.go_to_rest` 序列对齐 `PegInsertionEnv.go_to_rest`：

1. `_end_effector_action([-1.0])` —— 闭爪。**已夹持时 `FrankaLibfrankaGripper.close` 打印 `skip grasp` 直接返回**，不会重复 `grasp`（LOG-010 修复）。
2. `_check_start_pose()` —— 起点在盒子外时 **warn**（低于盒底 = 方块压在标记上，这是 LOG-019 的起始条件）。硬拦在烟测脚本里，见 §3.4。
3. `_move_action(当前 TCP)` —— 把阻抗平衡点钉在此刻。ROS 上是发 topic；franky 上还负责在 `reconfigure_compliance_params` 停掉 tracker 后**重新拉起** tracker。
4. `_interpolate_move(当前 z + reset_z_lift_m)` —— **相对当前 TCP** 抬 **3 cm**，时长由位移推导（≤2 cm/s，约 1.5 s）。
5. `_go_to_rest_pose()` —— 插值到 `reset_ee_pose` = H1 + 8 cm，同样限速。**这一步不再走上游 `FrankaEnv.go_to_rest`**：上游的循环条件是 `np.allclose(tcp[:3], reset_pose[:3], 0.02)`，而 `np.allclose` 的第三个位置参数是 **rtol 不是 atol** —— `target_y ≈ 0` 时那一轴的容差塌缩成 `atol=1e-8`，循环**必然**跑满 3 轮。上游只是浪费，但配上「时长由位移推导」之后，每次 reset 会多出两段约 1.5 s 的、重复命令臂已经在的位姿的插值；训练里就是几分钟的空转，看起来像挂住。现在改成**绝对**容差 `REST_POSE_TOL_M = 0.01 m`，最多 `REST_POSE_ATTEMPTS = 3` 次，**并在最后一次移动之后再判定一次**（循环是移动**前**判定，末次成功也会被报成 `NOT reached` —— LOG-022）。

日志上因此有三种收尾：`rest pose reached on attempt N`、`rest pose reached on the final attempt`、`rest pose NOT reached after 3 attempts`（后者的提示里会点名去查有没有 `step slew clamped` 洪流）。

**门闩看两层**：env 侧是上面的 0.01 m 绝对收敛；烟测脚本另有一道更宽的几何门（xy ≤ 0.03 m、z ≤ 0.025 m），所以「env 报 NOT reached 但脚本 PASS」是可能的，反之不可能。第 4 步可能短暂高于盒顶，原版 charger 也这样，`_interpolate_move` 本身**不**按 `ee_pose_limit` 裁剪（裁剪只发生在 `step()`）——所以围栏（§3.4）给 `+z` 留了 `reset_z_lift_m` 的额外余量，容许这一次、且仅这一次越顶。

### 3.4 护栏（务必知道它们会怎么报错）

**A. 哑失败护栏（「命令了但不动」，§1.4）**

| 位置 | 行为 |
|------|------|
| `tcp_probe.require_motion_ready` | 模式非 `Idle` → 抛出带处理建议的 `RuntimeError` |
| `step_cube_place_robot.py` | 打印 `robot_mode`；`reset`/`box` 在 `gym.make` **之前**拦下（省掉 30 s Ray 启动）；`connect` 只提示不拦 |
| `controller_extended._ensure_cart_tracking_motion` | 建 tracker 前检查模式，非 `Idle` 直接抛。**围栏已锁存时也直接抛**——否则一次刹车之后还能把 tracker 重建起来，等于把刹车撤销 |
| `controller_extended.move_tcp_pose` | 先查围栏锁存：已锁存则 `log_error` 后**直接返回，不下发、不抛**（§3.4 B）。否则 `super()` 之后检查 `is_running`；已死则 `stop()`（`join_motion` 重抛真因）、清 tracker、`log_error` 带 `mode=`、`raise RuntimeError` |
| `diag_franky_motion.py --probe` | 打印 `robot_mode` + 建议；非 `Idle` 时**跳过**运动测试（`--force` 可强来） |

**B. 超程护栏（「动过头」，§1.5，LOG-020 新增）**

| 位置 | 行为 |
|------|------|
| **libfranka 硬件反射**（`set_collision_behavior`，1 kHz） | 笛卡尔力/力矩阈值从上游 **100 N / 25 N·m** 降到 **40 N / 12 N·m**（= 范数上限）。**这是唯一不依赖 Python 被调度的界**——其余全是 GIL 里的 Python，50 Hz 看门狗监督 1 kHz 力矩环。关节力矩阈值刻意不动（收紧会招来自身动力学的误反射，且 j5–j7 已是 11 N·m） |
| `motion_limits.error_clips_for_stiffness` | `clip = 力上限 ÷ 刚度`，同时满足逐轴与范数两个上限 → 改刚度**不会**改力上限。tracker 启动日志打印**乘积**（逐轴与 worst-case 各一个），并注明阻尼项不在内 |
| `motion_limits.clip_shortfall` | 低刚度下 clip 被下界夹住、达不到设定力上限时 warn（否则「臂不动」会被当成正常运行） |
| `motion_limits.clamp_stiffness` | `K_t ≤ 3000`、`K_r ≤ 300`，REPL 手抖或 YAML 打错也建不出怪物 tracker |
| mixin `_interpolate_move` | 时长由位移推导（≤2 cm/s）；**位移 > 0.35 m 或转角 > 0.6 rad 直接拒绝** |
| `CubePlaceConfig.__post_init__` | `action_scale[0]` 被夹到 **5 cm/s**（上游 0.02 m/步 @10 Hz = **20 cm/s**，比事故那次还快一倍，而且 `step()` **不走** `_interpolate_move`，此前完全没有限速）；`is_dummy` 与 `robot_ip` 必须一致 |
| mixin `_clamp_step_slew` | 单次命令位移超出每周期预算就按比例缩回并 warn（绕过 `action_scale` 的调用者也被兜住）。**只在 `step()` 路径上有效**：`_move_action` 在 `_in_interpolate` 作用域内跳过它，因为上游 `_interpolate_move` 只在路点循环**之前**读一次 `_franka_state`，参考位姿在整个调用期间是陈旧的（LOG-022 就是被这个夹到差 4.2 cm）。跳过它安全的前提是「插值比步进慢」，1A 有断言盯着 |
| `controller._check_motion_guard`（**控制器进程内**，每个路点 + 每次 `get_state`） | 三道：① 测量 TCP 越出 `ee_pose_limit`（xy/+z 余量 0.05、**−z 仅 0.01**、`+z` 另加 `reset_z_lift_m`、外加绝对天花板）→ 刹车；② `\|测量−指令\| > 0.05` → 「没在跟随」→ 刹车；③ 姿态最短弧角 > **0.550 rad** → 刹车。**返回锁存原因而不是抛**（见下面「为什么控制器里一律不抛」） |
| `controller._check_requested_target` | **指令**位姿出围栏就在动之前拒绝。测量值围栏只有在臂已经走出去之后才会响，而 `_CART_MAX_STEP_M` 会把一个 30 cm 外的请求分成 10 cm 一步慢慢走过去 |
| `controller._watchdog_loop`（50 Hz 守护线程） | 路点之间、`time.sleep` 期间、回合之间的空档也在查。**事故的超调物理上就发生在这种空档里** |
| `controller._brake`（**按失效类型分向**） | `fence`/`orient` → **先 `stop()`**：臂已冲过目标，弹簧本来就在往回拉，`set_target(测量)` 恰好会把这个复位力清零。`lag` → **先 `set_target(测量)`**：那时是目标在跑，先消掉它。旧版对两者都用后一种顺序，等于在超调时把刹车拆了 |
| 刹车等待 | 不是固定 `sleep`，而是按 `\|dq\|` 轮询到停住（上限 0.25 s），并把刹车期间走过的距离记进日志 |
| **`_guard_trip_lock`：刹车只发生一次**（LOG-023 发现 3） | 跳闸原因是**刹完车才**锁存的（它要记录刹车期间的位移），所以看门狗在循环顶部查 `guard_trip_reason` 挡不住并发：主线程正在 `_brake` 的那 0.25 s 里，看门狗仍会看到 `None` 并**也**开始刹车。而 `fence` 走「先 `stop()`」、`lag` 走「先 `set_target(测量)`」，两条序列交错时一个线程的 `set_target` 可能把目标重新指到另一个线程正 `stop()` 离开的位姿上。现在两条路径都在同一把锁内「再查 → 刹车 → 锁存」；`_stop_watchdog()` 刻意放在锁**外**（持锁去 join 一个正等这把锁的线程会卡满 1 s） |
| `controller.guard_tripped()` + mixin `_raise_if_guard_tripped` | 控制器里 `raise` **不会**传到调用方：`WorkerGroupFuncResult` 吞掉它、发 SIGUSR1、处理器 `ray.kill` 所有 actor 并 `exit(-1)`。所以围栏路径只**刹车 + 锁存**，env 侧轮询后抛出**普通 Python 异常**，`finally` 才能真的跑 |
| **为什么控制器里一律不抛**（LOG-023 发现 1） | 上一版只让**看门狗**不抛，`_abort_motion` 自己仍在抛 —— 而 `_check_motion_guard` 又在**每次** `get_state()` 里跑。于是一次锁存之后，紧接着的任何一次 `get_state()`（**包括 `_close_env` 从 `finally` 里发的那一次**）都会把整个 driver 打掉，`env.close()` 还没来得及停 tracker。现在：`_abort_motion` 刹车 + 锁存**不抛**；`_check_motion_guard` **返回**原因；`move_tcp_pose` 锁存时拒绝下发；`motion_health` **绝不抛**（所有读取过 `_safe()`，失败以 `guard_check_error` 字段报出）；`_ensure_cart_tracking_motion` 是**唯一**会抛的地方，因为它要阻止「刹车后重建 tracker」。1A 有对应断言 |
| **这条规则不是围栏专用的——任何 Worker 方法抛异常，整个进程都会被打掉**（LOG-024 发现） | 上一行的机制（`WorkerGroupFuncResult` 吞异常 → 后台线程 SIGUSR1 → `ray.kill` 全部 actor + `exit(-1)`）是 `rlinf/scheduler/worker/worker_group.py` 里**所有** Ray actor RPC 的通用行为，不是专门为运动围栏做的特例。所以像夹爪 `close()` 在 `franka_libfranka_gripper.py` 里因 `measured width` 落在容差窗外而抛的 `RuntimeError("grasp did not capture the cube: ...")`，同样会把 REPL 进程和 Ray 一起带走——**用户在 `cmd>` 层面的 `try/except`（如果有）根本接不住**，因为异常是在另一个进程 / 另一条 Ray 内部线程里被发现并处理掉的。实操含义：**任何** `cmd>` 报错退出（不只是 motion guard 那几条）都要当成整个会话已经结束，重启 REPL，而不是期待还能在同一个 `cmd>` 里继续下一条命令 |
| mixin 在哪些点轮询 | 每次 `_move_action` 之后；**以及 `_interpolate_move` 的最后**——最后一个路点之后的 `time.sleep` 和收尾 `get_state` 期间发生的看门狗跳闸，逐路点轮询看不到，不补这一次的话原因要等到**下一次**命令运动时才浮出来，而 reset 的最后一段插值之后先跑的是悬停几何判定，于是屏幕上只剩一句「几何没达标」，真正的跳闸原因被盖住（LOG-023 发现 2） |
| 姿态围栏为什么用四元数角 | 本任务 roll ≈ **−3.116 rad**，离 −π 只差 0.026。欧拉读数落到 +π 那侧就会把正常姿态判成严重越界 → 误报。四元数最短弧角**没有绕回问题**。1A 断言：2π 等价 → 0 rad，90° 扭转 → 1.571 rad |
| `arm_motion_guard` 在盒子不可用时 | **真机上直接 `raise`**，不再只 warn。「保护机制自己退场、运行继续」正是事故的同类错误，而 `ee_pose_limit` 全零恰恰是最可能出错的配置形状 |
| `step_cube_place_robot.py` 起始位形硬门 | reset/box 前要求：z 在 `[接触+0.005, 盒顶+0.01]`、xy 在盒内、**姿态**在围栏内、**夹爪必须夹着标定尺寸的方块**。全部在 Ray 启动前 |
| mixin `close()` + `_close_env` | 停 tracker、打印最终 health（`controller health at teardown`）。此前**即使成功收尾**，阻抗 tracker 也会一直发力矩到进程退出。收尾这条路上的 `motion_health()` 现在保证不抛，所以「围栏跳过闸」与「读不到 health」不会再互相掩盖 |
| 夹爪 | 「是否夹着」只看**测量宽度**落在标定窗内，**不看 `is_grasped`**（`epsilon=0.05` 配 `width=0.01` 时空爪合到 0 也会报成功 → 毒化所有「方块在不在」的判断）；`grasp` 后按宽度**复核**；`open` 限速并在持物时警告；`move` 拒绝在持物时变宽；所有阻塞调用带 6 s 超时 |
| `diag_franky_motion.py` | 20 ms 连续采样、ramp 后 settle 观测窗、超调/z 天花板/跟随/tracker 死亡四道 abort |
| REPL 护栏 | `nudge` 单次 ≤0.2 rad；`home` 要写 `home yes`；持物时 `open` 要写 `open yes`；`impedance` 回显实际力上限 |
| 容器 / 启动脚本 | `docker_run_franky_5090.sh` 先查 1337 是否已被占（防第二个容器抢 FCI）；训练启动器在 `ray stop --force` 前拒绝 SIGKILL 掉活着的控制器 actor |

**所以从此以后：要么臂动到该去的地方，要么日志里有一条明确原因。「`reset OK` 但 `dz=0.0000`」和「悄悄冲高 26 cm」两种都会报错。**

已撤回的四处错误改动（不要再加回来）：tracker 构造后调 `__enter__`、`translational_clip_*` 映射成 franky `translational_error_clip`、`clear_errors` 跟踪中不 recover、mixin `_move_action` 无条件不 `_clear_error()`。

> 但注意：后两条的「证伪」（LOG-014）是在 `RobotMode.UserStopped` 下做的，那次**任何**运动都不可能发生，所以那个证伪**无效**。假说重新有效，现在用 `clear_error_per_waypoint` 旋钮 + `diag --test-recover-loop` 去**测**，默认仍保持上游行为（T8）。

---

## 4. 一步一步落地

### 阶段 0 — 只读自检（不动臂，任何时候都能做）

```bash
# 宿主机（不需要任何 venv / 容器）
ping -c 3 172.16.0.2
ip route get 172.16.0.2                                          # 应走 eno1
ss -tn state established '( dport = :1337 or sport = :1337 )'     # 应无连接
docker ps --filter name=rlinf-franky-5090 --format '{{.Names}} {{.Status}}'
```

浏览器 `http://172.16.0.2/desk`：**Unlock 关节**、无 safety violation、**Activate FCI**。

**启动 / 进入容器**：先看上面 `docker ps` 那行的输出判断容器是否已在跑（没看到就再跑一次 `docker ps --filter name=rlinf-franky-5090 --format '{{.Names}} {{.Status}}'`）。

- **有输出**（容器在跑）→ 直接开新 shell 进去，不要再 `docker run`：

  ```bash
  # 宿主机
  docker exec -it rlinf-franky-5090 bash
  ```

- **无输出**（容器不存在）→ 用启动脚本创建并进入，它是 `docker run -it --rm ...`（前台阻塞，退出即销毁容器）：

  ```bash
  # 宿主机
  bash /home/nvidia/bt/s/RLinf/b/x/configs/docker_run_franky_5090.sh
  ```

  该脚本固定参数：镜像 `rlinf/rlinf:agentic-rlinf0.4-franka`、容器名 `rlinf-franky-5090`、`--privileged --network host`（franky/libfranka 需要）、把 `/home/nvidia/bt/s/RLinf` 挂到 `/workspace/RLinf`。**这个终端此后就是容器的前台 shell，别关它**；之后如果还需要第二个 shell（比如训练跑着时另开一个查日志或跑诊断脚本），回到上一步用 `docker exec -it rlinf-franky-5090 bash` 从别的终端接进同一个容器，不要再跑一次 `docker_run_franky_5090.sh`（`--name` 冲突）。

- **`docker ps -a` 里看到 `rlinf-franky-5090` 是 `Exited`**（少见，正常应因 `--rm` 自动清掉）→ 先 `docker rm rlinf-franky-5090` 再按上面「无输出」那条重新创建。

进容器后，每个新 shell 都要重新初始化：

```bash
# franky 容器内（刚 docker exec / docker run 进来的新 shell）
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
# franky 容器内（已 source setup_before_ray_5090.sh）
RLINF_SKIP_CAMERA=1 python b/x/scripts/step_cube_place_dummy.py
# 期望末行: Phase1A PASS FrankyCubePlaceEnv-v1
```

**1B（GPU 容器，主验收，改了模型/YAML 才需要）：**

```bash
# 宿主机（这条命令自己 docker run 起一个一次性 GPU 容器，跑完即销毁）
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
| 2.4a `diag-probe` | `diag_franky_motion.py` | **否** | 直连，短 | `Idle`、力上限 20 N/轴 |
| **2.4b 速度/幅度阶梯** | `diag` / `diag-replay` | **是**（3–10 cm） | 直连，短 | 四档 `alive` 且 `peak_overshoot ≤ 0.02`（LOG-034 已全过，实测全 0.0000） |
| 2.5 H1 标定 | 人 + REPL | 人手引导 | **是** | `calibrated: true` |
| 2.6 `connect` | `--connect-only` | **否** | 子进程短连后释放 | 几何 + 起始位形 + 夹爪都正确 |
| 2.7 `reset` | `--reset-only` | **是** | env 会话 | 悬停几何达标 |
| 2.8 `box` | 默认 | **是** | env 会话 | 不张爪、不砸盒底 |

**互斥：** 2.4 / 2.5 的脚本必须先退出（REPL 敲 `q`），才能跑 2.6–2.8。

**2.4b 已在 LOG-034 补做完成（六档全过）。** 它欠了三轮：LOG-018 在别的位形上过了 2.4，LOG-019 在 2.7 出了事故，之后 LOG-022 与 LOG-026 都跳过了它（理由都是「2.7 已经一次跑通」）。**跑通不等于测到**——阶段 3 真机首跑第 10 秒就围栏跳闸，回头补做 2.4b 才拿到基线：`peak_overshoot` 全 0.0000、最差 `peak_lag` 9.2 mm、最差 `peak|dq|` 0.112 rad/s。**正因为平移档全部干净**，才能把事故的嫌疑从「阻抗链路有问题」压缩到「旋转授权没推导过」这一处（§7 T16）。这就是基线的价值：**没有基线的异常值只是一个数字，有基线的异常值是一条线索。**

**它现在还剩一个覆盖缺口：四档全程不带旋转**（`target_at` 只改 xyz，姿态恒为起点姿态），而 `step()` 每步可转每轴 0.1 rad。补一档 10 Hz 旋转阶梯是 T16 的第一步。

#### 2.1 安全规则（开跑前默读）

1. **急停在手边可及。** 台面标记贴牢，方块厚度与训练时一致。
2. **手持设备的 user-stop 必须松开（按钮抬起）。** 按下时 `robot_mode=UserStopped`，libfranka 拒绝一切运动，而状态读取与夹爪照常 → `reset` 看似跑完但 `dz=0.0000`（§1.4、LOG-017）。「人在急停旁」= 手边有急停，**不是**按住 user-stop。
3. 2.4 / 2.7 / 2.8 之前：**方块必须夹紧**；用引导键把臂放到**标记上方 3–5 cm**。**不要**从工厂 `home` 关节位横扫过去。
   **这一条现在是机器强制的**：脚本会在 Ray 启动前拒绝 `z ≤ 接触点`（方块压在标记上）、`z > 盒顶+1 cm`、`xy` 出盒、或**空爪**。LOG-019 就是从 `z = 接触点 − 6.6 mm` 起跑的，当时没有任何检查。用 `connect` 先确认，别靠目测。
3b. **诊断三项不要串在一条命令里跑。** ramp 结束时臂还在动，紧接着的 `--test-cartesian-motion` 必然踩 `joint_velocity_discontinuity`，而且它留下的 latched error 会污染后续（LOG-018）。用 `diag` / `diag-replay` 子命令，一次一项。
4. 只用 `python b/x/scripts/test_franky_controller_ext.py` 做标定。**不要**用 ROS 的 `test_franka_controller`，**不要**用 `python -m toolkits.realworld_check.test_franky_controller`（`NotImplementedError`），**不要**用 `step3_test_controller.py`（会自动 `home` 并张爪）。
5. 全程在 franky 容器 + `franky-0.19.0`。禁止宿主机 `.venv`、禁止 GPU 容器、禁止名为 `rlinf` 的那个容器。
6. 乱跑 / 砸桌 / 顶人：**拍急停**。急停之后**不要**立刻重跑 env，先 Desk 清 fault 再重新 Activate FCI。
7. 烟测脚本默认：`RLINF_SKIP_CAMERA=1`、`no_gripper=True`、`enable_random_reset=False`、`random_xy_range=0.0`、`safe_smoke_hold=True`（只跳过 `__init__` 的插值，**`reset()` 仍会动臂**）。不要加 `--unsafe-full-reset`。

#### 2.2 开跑前检查单

即 §4 阶段 0 全部内容。**最后一行 `--probe` 必须是 `RobotMode.Idle`。**

#### 2.3 进容器

见阶段 0。每个新开的 bash 都要重新 `source setup_before_ray_5090.sh`。

#### 2.4 运动链路体检（**先过这一关**）

在把 env / Ray / wrapper 这些变量堆进来之前，先单独回答一个问题：**这台机器现在能不能被 franky 命令动起来，而且只动到该动的地方？** 脚本直连 `franky.Robot`，不经 Ray、不经 `FrankaEnv`。

> **LOG-019 的教训就在这一节。** 上一版这里写着「三项 OK → 阻抗链路健康 → 直接做 2.6 / 2.7，`reset` 应该一次过」，我据此放行了 2.7，然后臂冲高 36 cm。那个推断是错的：体检当时跑的是 **1 cm/s、3 cm、x≈0.55 空载**，而 `reset` 实际跑的是 **10 cm/s、10 cm、x≈0.707 夹着方块**。**「体检 PASS」只对体检覆盖过的速度和幅度有效。**

**前置：** 人站在急停旁（手**不**按 user-stop），方块夹着，臂在**标记上方 3–5 cm**——即 2.7 的真实起始位形。在别的位形上测出来的数不算。

##### 2.4a 只读

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
ray stop                                    # 确保没有残留 Ray 占着 FCI
bash b/x/scripts/run_cube_place_phase2.sh diag-probe
```

必须 `robot_mode=RobotMode.Idle`、`has_errors=False`。顺便核对打印的 `authority:` 一行——它是本次会用的**力上限**，应为 `<= 20.0N/axis (34.6N worst-case 3-axis)` 左右，不是 100 N。

##### 2.4b 速度 / 幅度阶梯（新门闩）

一次一档，档间回 `diag-probe`。臂只准向上（远离桌面）。

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
# 1) 存活 + 3 cm 斜坡 @ 1 cm/s（diag = --test-hold --test-impedance，不是零位移；
#    --test-hold 单独测的才是零位移，这里 --test-impedance 会按 --dz/--seconds 斜坡）
bash b/x/scripts/run_cube_place_phase2.sh diag --dz 0.03 --seconds 3

# 2) 提速到 3 cm/s（注意 --seconds 同时也把 --test-hold 的存活观测窗从 3 s 缩到 1 s）
bash b/x/scripts/run_cube_place_phase2.sh diag --dz 0.03 --seconds 1

# 3) 加幅度：2 cm/s 走 10 cm（--ramp-speed 0.02 是 2 cm/s，不是 3 cm/s；这才是 reset 第 5 步的量级）
bash b/x/scripts/run_cube_place_phase2.sh diag --dz 0.10 --ramp-speed 0.02

# 4) 复现 reset 真正的指令形状：10 Hz 开环阶梯，且要用 reset 实际的路点数/速度
#    （reset 日志里 `interpolate_move: 0.0305m in 1.50s ... 2.0cm/s` → 15 路点 @ 10Hz；
#    diag-replay 默认 --seconds 3 会变成 30 路点 @ 1cm/s，达不到这个量级，必须显式给 --seconds）
bash b/x/scripts/run_cube_place_phase2.sh diag-replay --dz 0.03 --seconds 1.5
```

每档看什么：

| 测试 | 做什么 | 期望 |
|------|--------|------|
| `--test-hold` | 建 tracker，目标 = **当前位姿**，每 20 ms 看 `is_running` | 全程 `is_running=True`，`stop() clean`，`sag` 约 1 mm |
| `--test-impedance` | 目标沿 z ramp，**ramp 完不停 tracker**，再观测 `--settle-seconds`（默认 2 s） | `final_dz ≈ dz`，且 **`peak_overshoot ≤ 0.02`** |
| `--test-waypoints` | 复现 `_interpolate_move` 的 10 Hz 绝对路点阶梯 | 同上。这是最接近 `reset` 的一档 |
| `--test-recover-loop` | 同上 + 每个路点前 `recover_from_errors()`（= `_move_action` 的行为） | 与上一档**没有明显差别**。有差别就说明 R5 是真的（T8） |

**`peak_overshoot` 是这一节的核心量**，它就是事故里那 26 cm。旧脚本 ramp 完立刻 `stop()`，根本测不到它。

**LOG-034 的实测基线**（真实位形，起点 `z=0.2720` 即接触点上方 2.1 cm，方块夹着）。**把它当对照表用**：以后任何一次跳闸，先跟这几行比，就知道异常出在哪个量上。

| 档 | 速度 / 幅度 | `peak_lag` | `peak\|dq\|` | `peak_overshoot` | `peak\|F_ext\|` |
|---|---|---|---|---|---|
| hold（零位移） | — | 0.0009 | 0.011 | 0.0000 | 5.6 N |
| 1) ramp | 1 cm/s / 3 cm | 0.0043 | 0.028 | 0.0000 | 6.8 N |
| 2) ramp | 3 cm/s / 3 cm | 0.0057 | 0.068 | 0.0000 | 7.1 N |
| 4) 10 Hz 阶梯 | 2 cm/s / 3 cm | 0.0065 | 0.102 | 0.0000 | 7.7 N |
| 3) ramp | 2 cm/s / 10 cm | 0.0092 | 0.097 | 0.0000 | 10.7 N |
| 回降 | 2 cm/s / −10.9 cm | 0.0043 | 0.112 | 0.0007 | 2.9 N |
| **阶段 3 首跑跳闸** | 策略自由动（含旋转） | **0.0719** | **2.69** | — | — |

滞后随速度增长很慢（1→3 cm/s 只从 4.3 到 5.7 mm），与 LOG-018「滞后由静摩擦主导、约 8 N 与速度无关」一致。**`--test-waypoints` 那一档没有复现 LOG-019 的 26 cm**——所以「这个位形下 10 Hz 零阶保持阻抗本身就不稳」这条假说，至少在纯平移、单调、慢速条件下不成立（T11 仍未定）。

**档间不要降臂回工作高度。** `_guards_for` 对负 `dz` 的默认地板是「目标下方 5 cm」，在本机会落到接触点以下约 3 cm——等于授权把方块压进标记。四档一路向上（向上还会让臂折起来、离奇异更远），最后用**一次带显式 `--z-floor` 的受控下降**回悬停：

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
# 从四档累加后的高度回悬停；--z-floor 必须明显高于接触点
bash b/x/scripts/run_cube_place_phase2.sh diag --dz -0.109 --ramp-speed 0.02 --z-floor 0.315
```

**判读与分支：**

| 结果 | 结论 | 下一步 |
|------|------|--------|
| 四档都 `alive` 且 `peak_overshoot ≤ 0.02` | 在 reset 的真实速度/幅度/位形下阻抗可控 | 可以做 2.6 / 2.7。**仍要人在急停旁** |
| 任一档 `ABORT: overshoot ...` | 超调已复现，围栏起作用了 | 降力上限：`--force-ceiling 15`（甚至 10）再跑该档。调通了才进 env |
| `ABORT: not tracking` | 阻抗跟不上（力上限太低 / 位形权限不足 / tracker 死） | 反向：`--force-ceiling 35`。若同时 `is_running=False`，先看 `stop() surfaced:` 的 libfranka 真因 |
| `hold` 就死 | 1 kHz torque 链路本身不通（模式 / RT 调度 / 容器 / FCI 被抢） | 看 `stop() surfaced:`。`"User stopped"` → §1.4；其它 → §6 |
| `--test-recover-loop` 明显比 `--test-waypoints` 差 | LOG-013 的假说成立 | `reset --no-waypoint-clear-error` 复测，记进 LOG，更新 T8 |

**注意：** `final_dz` 比指令值小一点（例如 +0.025 / +0.030）是**稳态跟随滞后**，不是失败——阻抗天生有滞后。门闩是「明确地动了」+「没有冲过头」。

`--probe` 显示非 `Idle` 时脚本会直接跳过运动测试并 exit 1；`--force` 可强行执行，但只在明确知道自己在干什么时用。

**`--test-cartesian-motion` 现在是可选的、且要单独跑。** T1 已关闭（reset 用阻抗，不需要阻塞运动），而且它在近奇异位形（`q2≈0`）或臂还有残余速度时会抛 `cartesian_motion_generator_joint_velocity_discontinuity`（LOG-018）——脚本现在会先 dwell 并检查 `|dq|`，但没必要为了它污染前面几档。

#### 2.5 H1 标定（若标记/方块没变，已 PASS，可跳过）

语义：**夹爪闭合、方块已经贴住台面标记时**，末端 TCP 的 `[x, y, z, roll, pitch, yaw]`（米 + 欧拉 xyz）。空爪对准标记再夹方块会偏掉一整块厚度；张爪后再读作废。

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
export FRANKA_ROBOT_IP=172.16.0.2 FRANKA_GRIPPER_TYPE=franka
ray stop
python b/x/scripts/test_franky_controller_ext.py
# 或: bash b/x/scripts/run_cube_place_phase2.sh calibrate
```

应看到 `FrankyControllerExtended REPL` 与 `Connected to Franka at 172.16.0.2`，然后是 `cmd>`。若出现 `the libfranka backend for the original Franka Hand is not yet supported`，说明跑错了官方 toolkit。

`cmd>` 依次：

1. `open` → 按训练朝向把方块放入 → `close`。`close` 是约 **20 N** 轻力抓取（上限 40 N），夹住后维持这点力。异常立刻敲 `stop` 或拍急停。**夹紧后不要再换握姿。**

   **`close` 判定「夹住了没有」不是看有没有东西在指间，是看最终宽度是否落在 `FRANKA_CUBE_WIDTH_M ± FRANKA_HOLD_TOL_M` 这个窗口里**（默认 `0.046 ± 0.012` m）。这个默认值是通用假设，不是给你手上这个方块量的——LOG-022 实测过 0.0365 m，LOG-024 实测过 0.0325 m，LOG-026 实测过 0.0316 m，**三次都比默认值小一截、且彼此有 5 mm 的散布**（所以窗宽 `FRANKA_HOLD_TOL_M` 放到 0.015 是合理的），说明第一次标定新方块时大概率会撞见 `grasp did not capture the cube: measured width=...`。**这不代表没夹住**：错误信息里的 `measured width` 本身就是一次有效读数。处理方式：
   - `q` 退出（这个错误会让整个进程连带 Ray 一起退出，回不到 `cmd>`，见 §3.4「所有 Worker 方法都是这样」和 §6）；
   - 用报出来的宽度重新导出再重启：`export FRANKA_CUBE_WIDTH_M=<measured width>`（容差 `FRANKA_HOLD_TOL_M` 可以先放宽到 `0.015`～`0.02` 覆盖夹取角度的正常波动，但不要放到能覆盖「空爪合到底」或「完全张开」的地步）；
   - 建议 `open`/`close` 反复 2–3 次，每次记下 `measured width`，取中间值定档，而不是只信第一次。
2. **按住臂上引导键**（此时 `robot_mode` 会变 `Guiding`，正常），移到标记正上方，缓慢下降到方块**轻轻贴住**标记。不要压垮垫子、不要把桌子顶起来。
3. 松开引导键，臂不再动。输入 `getpos_euler`，记下 6 个数。**再敲一次确认稳定。**
4. `q` 退出，释放 FCI。**不要**在这里敲 `home`。

写入文件：

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
python b/x/scripts/write_cube_place_pose.py <x> <y> <z> <roll> <pitch> <yaw>
# 现存值（当前标记位置，文件里就是这一组；LOG-026 那次 box 用的正是它）:
# python b/x/scripts/write_cube_place_pose.py  7.26857066e-01  2.49468647e-02  2.50846744e-01 -3.13747483e+00 3.54833569e-02 -2.68804519e-03
# 已作废，仅供对照: LOG-022 那次 0.67129487 -0.00806926 0.2294566 -3.10852762 0.07677764 0.15125061
#                  更早一次      0.7062065   0.03620906 0.23192134 -3.11614319 0.02628124 0.17913087
```

`getpos_euler` 是 numpy 打印，接近 −π 的 roll 会写成 `-3.13747483e+00`。**这个形式可以直接粘贴**：LOG-025 修掉了 argparse 把「负数 + 科学计数法」当成未知选项、于是报「明明给了 6 个数却缺 pitch/yaw」的那个 bug。

`write_cube_place_pose.py` 会**拒写**不像真机读数的位姿（可达域按肩部半径 30–90%、接触高度、欧拉范围、roll 必须接近 ±π），覆写前留 `.bak`；`--force` 时 `calibrated` 会被写成 **false** 并记下失败项 —— 也就是说 `--force` 之后 `connect` 仍会 exit 1，这是刻意的。

写完再用引导键把臂抬到标记上方几厘米（仍夹着方块），作为 2.7 的起始姿态。

**门闩：** YAML 中 `calibrated: true`、六元组非全零、两次 `getpos_euler` 接近、标记与方块此后未被挪动。

#### 2.6 只读几何（`connect` / `2c`，不动臂）

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
bash b/x/scripts/run_cube_place_phase2.sh connect
# 等价: python b/x/scripts/step_cube_place_robot.py --connect-only
```

**期望输出：**

- `target_ee_pose (H1)` 与 YAML 一致
- `reset_ee_pose hover` 的 z = 接触 z + **0.08**（当前 H1 → **0.3308**）
- `ee_pose_limit`：xy 半宽 0.05，z 下沿 接触 − 0.005，z 上沿 接触 + 0.08
- `robot_mode: RobotMode.Idle`
- `reach: worst box corner ... 92% of 0.855m reach NEAR-SINGULAR`（**LOG-023 新增**：此前只有装围栏时才打印，也就是臂已经要动了才看得到）。当前标记位置下这行**就是带 `NEAR-SINGULAR` 的**，它是提示不是失败，但意味着该方向上没有余量了
- `connect-only OK`，且**没有** `creating FrankyCubePlaceEnv-v1`

看 `probed - target xyz`：当前若已在标记上方悬停，xy 应较小、z 约 +0.05～0.10。**xy 差几十厘米就先用引导键挪近**，不要从远处 `reset`。

H1 未标定时脚本仍会探测 TCP，但 exit 1，且禁止进 2.7 / 2.8。

#### 2.7 `reset` 到悬停（`reset` / `2d`，会动臂）

前置：2.6 PASS、方块夹紧、臂在标记上方 3–5 cm、`robot_mode=Idle`、人在急停旁（手不按 user-stop）。

> **2.7 已在 LOG-026 一次跑通（与 2.8 同一次 `box`），2.4b 已在 LOG-034 补齐。** 它们测的是不同的东西：reset 回答「到不到得了」，2.4b 回答「会不会冲过头」——2.7 通过**不等于**量过超调。**任何一次出现 `motion guard abort`、或 reset 再度失败，就退回 2.4b 并跟 §2.4b 那张实测基线表逐列对比，不要在 2.7 上反复试。**

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
ray stop
bash b/x/scripts/run_cube_place_phase2.sh reset
# 第一次重跑建议先压低权限：
# bash b/x/scripts/run_cube_place_phase2.sh reset --force-ceiling 15 --interp-speed 0.01
```

脚本流程（Ray 之前的三道门是**这个顺序**，先模式后位形）：读 H1 → 子进程探测 TCP / 模式 / **夹爪** → 打印几何 → ① `require_motion_ready`（模式）→ ② 起始位形硬门 → ③ 夹爪必须夹着方块 → `ray.init` → `gym.make`（此时装围栏并回读）→ `reset()`（§3.3）→ 检查几何与夹爪 → `finally:` 打印 `controller health at teardown` 并 `env.close()`。

三道门全在 Ray 之前，所以配置不对只花几秒，不会浪费一次机器人会话。`gym.make` 本身会拉起控制器 actor（且在 `safe_smoke_hold=False` 时真的插值一次），所以它失败时脚本会**大声**说「actor 可能已经活着」，而不是静默退出。

**期望：**

- `authority: spring force <= 20.0N/axis (34.6N worst-case 3-axis) [K_t=2000 x clip=0.0100m]; torque <= 6.00Nm/axis ...`（**若逐轴是 100 N，立刻停**）
- `collision behavior tightened: cartesian force/torque thresholds=[40.0, 40.0, 40.0, 12.0, 12.0, 12.0] (was [100.0, 100.0, 100.0, 25.0, 25.0, 25.0])`
- `gripper: holding=True width=<实测>`（LOG-026 那次是 `0.0316`，配的窗口是 `cube_width=0.0325 +/- 0.0150`）。**换方块或换夹取角度就要重量**，见 §2.5 步骤 1 与 §6「实测宽度贴着窗沿」一行
- `motion guard armed: xyz in [[0.6269,-0.0751,0.2358], [0.8269,0.1249,0.4108]] (box +/- 0.050m, floor -0.010m, +z headroom 0.030m, ceiling 0.4308), max_lag=0.050m, orient<=0.550rad`
- `reach: worst box corner ... 92% NEAR-SINGULAR; worst fence corner ... 98% NEAR-SINGULAR`（当前标记位置下**两处都带标记**，是已知的，不是本次新问题）
- `motion guard confirmed: ...` —— 这是**回读**，只打印 `armed` 不算，`motion_health()` 里 `guard_enabled` 为假会直接抛 `motion guard did not arm`
- 日志含 `FrankyCubePlaceEnv-v1`；`wrapper stack` 含 **`GripperCloseEnv`**；动作维 6
- `Cartesian impedance tracker started (robot_in_control=True, tc=0.100, K_ns=5.0, joint_repulsion=True) spring force <= 20.0N/axis ...`（`is_running=` 这个字面写法只在 2.4b 的 `--test-hold` 里出现，例如 `tracker created: is_running=True`；env 路径上打印的字段是 `robot_in_control`，franky 自己的 `is_running` 就是 `robot.is_in_control`，改名是有意的，别拿错关键字去 grep）
- `interpolate_move: 0.0300m in 1.50s (2.0cm/s)`；若该段距离让 2 cm/s 超标，会看到 `would be 2.1cm/s; stretching to 1.54s`——**拉长时间而不是提速**，这是权限在起作用
- `cube_place go_to_rest: current [...] +z=0.030 -> [...] over 1.50s`
- `rest pose reached on attempt 1`（LOG-026 那次是 `on attempt 2`，也算过：第二段补的是第一段剩下的残差）。**不应该**看到成排的 `step slew clamped`——那是 LOG-022 的回归，已修；再出现说明跑的是旧代码
- `cube_place go_to_rest done: ... dz=<悬停 z − 起始 z>`。这个 `dz` 是**净变化**，取决于你把臂引导到了多高：起点已接近悬停（LOG-026 那次起点 z≈0.3272，悬停 0.3308）时 `dz` 只有 `+0.0016`，起点在标记上方 3 cm 时应约 `+0.05`。判据不是某个具体数字，而是**终点 z 是否落在 `hover ± 0.025`**（下一行的 `hover check` 就是这么判的）。**`dz≈0.0000` 且终点离悬停仍差几厘米**才是失败（§1.4）；`dz` 冲过悬停也是失败（§1.5）；`attempt` 之间每轮只前进几毫米、最终仍差 2 cm 以上，是 LOG-022 那种「被夹住」的失败
- `gripper_open after reset: False`
- `controller health: {... 'robot_in_control': True, 'tracker_object_present': True, 'guard_enabled': True, 'watchdog_alive': True, 'guard_tripped': None, ...}`（没有 `tracker_running` 这个键；`watchdog_alive` 是唯一能看到 50 Hz 看门狗还活着的地方，建议连同 `guard_enabled` 一起进门闩 2.7-2b）
- `hover check: |xy-target| ≤ 0.03m，|z-hover| ≤ 0.025m`
- 目视：臂升到标记正上方，**不**放下方块、**不**张爪
- `reset-only PASS`，exit 0

**立刻停手的信号：** xy 飞出盒子、z 往桌面砸、z 明显冲过标记上方 8 cm、夹爪张开、FCI 被抢。

**若报 `motion guard abort: measured TCP left the guard fence ...`**：围栏起作用了，臂已被刹住。**不要**放宽围栏。回 2.4b 用 `--force-ceiling` 降权限，把超调调掉再回来。

**若报 `motion guard abort: not tracking ...`**：阻抗跟不上（力上限太低、位形权限不足，或 tracker 已死）。看同一段日志里的 `mode=` 与 libfranka 真因。

**若报 `cartesian impedance tracking stopped: ...`**：哑失败护栏在说话，后面跟着 libfranka 真因和 `robot_mode`。按 §6 处理，**不要**去改几何。

**若报 `refusing to reset from this pose ...`**：起始位形硬门。按提示用引导键把臂挪到标记上方 3–5 cm，`connect` 复核，再重跑。**不要**直接加 `--allow-start-outside-box`。

#### 2.8 盒子内零动作 + 少量下探（`box` / `2e`，会动臂）

2.7 PASS、方块仍在、标记没动：

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
bash b/x/scripts/run_cube_place_phase2.sh box
# 等价: python b/x/scripts/step_cube_place_robot.py --num-steps 3 --approach-steps 3
```

行为：先做与 2.7 相同的 `reset`，再 3 步**零动作**，再 3 步 **−z**（`action[2]=-1.0` 满幅；`action_scale[0]` 现在被步进速度上限夹到 5 mm/步，幅度再被 `clip_z_range_low=0.005` 卡住）。

**期望：**

- 零动作时 TCP 几乎不动；reward 是稠密 xyz（悬停时通常**不是** 1.0）
- 下探时 z 下降或贴在盒顶/盒底；**不得**低于 接触 z − 0.015（脚本硬门 `z_floor`）
- 全程 `gripper_open` 为假；6D wrapper 不会发张爪
- `box-steps PASS`

**下探不要求真的碰到标记。** 阶段 2 的门闩是「闭爪 + 悬停几何 + 盒子下沿」，反复触达是阶段 3 的 SAC 的事。

#### 2.9 阶段 2 验收门闩

| ID | 项 | 通过标准 |
|----|----|----------|
| 2.4-0 | 力上限 | 所有涉及运动的日志里 `authority:` 为 `<= 20.0N/axis`（范数 34.6 N）/ `<= 6.00Nm/axis`，**不是** 100 N / 45 N·m；且出现 `collision behavior tightened: ... 40.0 ... 12.0` |
| 2.4-1 | 运动链路存活 | `--test-hold` 全程 `is_running=True`，`stop() clean` |
| **2.4-2** | **无超调** | 2.4b 四档（1 cm/s·3 cm、3 cm/s·3 cm、2 cm/s·10 cm、10 Hz 阶梯）全部 `peak_overshoot ≤ 0.02` 且 `alive` |
| **2.4-3** | **真实位形** | 上述四档是在「夹着方块、标记上方 3–5 cm」这个位形上做的，不是在 home 附近 |
| 2.5-1 | H1 文件 | `calibrated: true`，六元组来自贴住标记时的 `getpos_euler` |
| 2.5-2 | H1 语义 | 读数时闭爪、方块在爪、贴住标记；无张爪后补读 |
| 2.6-1 | 只读几何 | hover = 接触 + 0.08，xy 盒 ±0.05，z 下沿 −0.005；不 `gym.make` |
| **2.6-2** | **起始位形** | `connect` 不打印任何 `start-pose note`；`gripper: holding=True` |
| 2.7-1 | Gym | 真机 `gym.make` 为 `FrankyCubePlaceEnv-v1` |
| 2.7-2 | 闭爪 wrapper | 栈含 `GripperCloseEnv`；`action_space.shape == (6,)` |
| **2.7-2b** | **围栏已装** | 日志有 `motion guard armed: ...`；`controller health` 里 `guard_enabled: True` |
| 2.7-3 | 悬停几何 | 目视在标记上方；xy 误差 ≤ 0.03，z 误差 ≤ 0.025 |
| **2.7-3b** | **不超程** | 全程无 `motion guard abort`；目视 z 未明显冲过标记上方 8 cm |
| 2.7-4 | 不张爪 | `gripper_open == False`，方块未掉 |
| 2.8-1 | 盒子 | 零动作稳定；下探不砸穿下沿；全程不张爪 |
| 2.8-2 | 退出 | 两个脚本 exit 0；结束后 FCI 可被 REPL 重新占用 |

失败就修 YAML / 重做 H1 / 查 FCI 与模式，过程记进 LOG。

**不要为了过门闩放宽安全量：** 不要加大 `clip_z_range_low`，不要放宽 `--guard-margin`，不要抬 `--force-ceiling` 去「让它动起来」而不先看 `peak_overshoot`。围栏报错是它在干活，不是它挡路。

阶段 2 **不要**开 `train_async` / dummy SAC GPU，**不要** `ray start` 多节点（脚本内部 `ray.init` 足够）。

### 阶段 3 — 在线 SAC（配置与脚本待创建，正文见 §S3.1）

这一节是**照 charger（`realworld_charger_sac_cnn_async`）改写的可执行流程**，不是设计稿。两个已定的形态选择贯穿全节：

- **双容器双 Ray 节点**（照 charger 的 `node_groups` + `python_interpreter_path`）：franky 容器跑 env worker + 控制器，GPU 容器跑 actor + rollout。原因见 §S3.2。
- **第一版就带相机**（`cnn_policy` + ResNet10 + `wrist_1`），所以 §S3.3 的相机验收是阶段 3 的**第一步**，在任何训练之前。

> **本节小节号写作 `S3.x`**（S = Stage），与第 3 章的 §3.1–§3.4（已落地资产）不是同一套编号。文档其它地方出现的 `§3.2` / `§3.3` / `§3.4` 一律指第 3 章。

前置门闩（缺一不可）：

| 项 | 怎么确认 |
|----|----------|
| 阶段 2.6 / 2.7 / 2.8 PASS | 最近一次 `box` 的日志里有 `reset OK`、`hover check` 双项达标、`box-steps PASS`，且**全程无** `motion guard abort` |
| H1 与实物一致 | `b/x/configs/cube_place_target_ee_pose.yaml` 的 `calibrated: true`，且标记与方块自那次标定后没被挪动 |
| 夹爪窗口按实物标定 | `FRANKA_CUBE_WIDTH_M` 用实测值（LOG-024：默认 0.046 不是任何实物的值），`connect` 打印的 `holding=True` 且 `width` 离窗沿有余量 |
| 相机 | §S3.3 全过 |
| 视觉权重 | `${RLINF_RESNET10_PATH}/resnet10_pretrained.pt` 存在可读 |
| 2.4b | **已做完，六档全过**（LOG-034，T5 关闭）。实测基线见 §2.4b，跳闸时拿它逐列对比 |
| **旋转授权** | **已闭环，但不是靠笛卡尔上限闭的**。首跑第 10 秒死在这里；旋转阶梯（LOG-036）在悬停位形四档全过、`step()` 旋转上限 0.3rad/s 已生效（T16），**但仍二次跳闸**——根因是雅可比放大（§7 T19），已由 §S3.11 的关节需求限幅解决。**不要再去调 `RLINF_CUBE_STEP_ROT_SPEED`**：任何固定的笛卡尔上限，要在事故位形安全就必然在别处慢得没必要 |

#### S3.0 阶段 3 与 charger 的对照（哪些照抄、哪些必须改）

| 键 / 项 | charger | 本任务 | 为什么 |
|---|---|---|---|
| `defaults` 的 env 包 | `env/realworld_peg_insertion` | `env/realworld_cube_place` | 后者已钉 `no_gripper: True` / `safe_smoke_hold: True` / `use_dense_reward: True` / `use_spacemouse: False`，照抄 charger 的包会同时丢掉这四个决定 |
| gym id | `PegInsertionEnv-v1` | `FrankyCubePlaceEnv-v1` | 前者走官方 ROS 控制器，后者才走 `FrankyControllerExtended` + 围栏 |
| `hydra.searchpath` | 只有 `EMBODIED_PATH/config` | **再加** `REPO_PATH/b/x/configs` | `env/realworld_cube_place` 在 `b/x/configs` 下 |
| `algorithm.*`（SAC 超参） | 见 charger | **原样照抄**（`update_epoch: 32`、`gamma: 0.8`、`tau: 0.01`、`target_entropy: -4`、`bootstrap_type: always`） | 1B 已用同一组跑通；此刻不要同时改算法和硬件 |
| `runner.save_interval` | `-1`（只在训练正常结束时存） | **50** | 真机会被急停/掉块打断，`-1` 意味着断了就什么都没有 |
| `runner.max_epochs` | 8000 | **8000**，首跑建议先 `200` 试水 | embodied runner 每 epoch 一个 global_step |
| `runner.val_check_interval` | `-1` | **`-1`** | 真机没有独立评估工位，验证就是看训练回合 |
| 几何 `clip_*` / `random_*` | 0.02 / 0.02（插孔级） | 0.05 / 0.005 / 0.08 / 0.03（§5 表） | 对准平面标记不需要插孔精度，但 z 下沿必须小 |
| `no_gripper` | 默认 True | **True（env 包已钉）** | 闭爪任务，动作 6D |
| `enable_random_reset` | 开 | **开**（代码默认 True） | 与烟测不同，见 §S3.10 |
| `demo_buffer` | 无 | **无** | 纯在线 SAC，不需要先采示教 |
| 运动权限 | 隐式 100 N / 20 cm/s | 20 N/轴、5 cm/s（`step`）、2 cm/s（插值） | LOG-019。这些**不在 YAML 里**，靠 franky 容器 `ray start` 前的环境变量，见 §S3.10 第 1 条 |

#### S3.1 要创建的四件东西（T2，LOG-027 已落盘，但未在真机上跑过）

> 下面四段正文已按原样写入仓库对应文件。**开工前务必检查文件顶部标了 `EDIT ME` 的三处**：GPU 容器的 `python_interpreter_path`、`FRANKA_CUBE_WIDTH_M`（换成你的实测值）、`camera_serials`（换成 §S3.3 8a 探测到的 serial）。`realworld_cube_place_sac.yaml` / `realworld_cube_place_camera.yaml` 里的 H1 六元组也要用 `run_cube_place_phase2.sh connect` 的当次输出核对一遍——那是本次实测值，不是你机器上的值。

**① `b/x/configs/realworld_cube_place_sac.yaml`** —— 以 `realworld_cube_place_dummy_sac_gpu.yaml` 为底，只改 `cluster` / `runner` / `env.*.override_cfg` 三处：

```yaml
# 真机在线 SAC：env+controller 在 franky 容器(rank1)，actor+rollout 在 GPU 容器(rank0)。
defaults:
  - env/realworld_cube_place@env.train
  - env/realworld_cube_place@env.eval
  - model/cnn_policy@actor.model
  - hybrid_engines/fsdp@actor.fsdp_config
  - weight_syncer/patch_syncer@weight_syncer
  - override hydra/job_logging: stdout

hydra:
  run:
    dir: .
  output_subdir: null
  searchpath:
    - file://${oc.env:EMBODIED_PATH}/config/
    - file://${oc.env:REPO_PATH}/b/x/configs

cluster:
  num_nodes: 2
  component_placement:
    actor:
      node_group: gpu
      placement: 0-0        # GPU hardware rank 0
    rollout:
      node_group: gpu
      placement: 0-0
    env:
      node_group: franky
      placement: 0          # Franka 硬件 rank 0，不需要 accelerator
  node_groups:
    - label: gpu
      node_ranks: 0-0
      env_configs:
        - node_ranks: 0-0
          # 在 GPU 容器里 `which python` 实测填入；rank 0 是 head，
          # 不写也会退回 driver 自己的解释器
          python_interpreter_path: /opt/venv/openvla/bin/python
    - label: franky
      node_ranks: 1-1
      env_configs:
        - node_ranks: 1-1
          python_interpreter_path: /opt/venv/franky-0.19.0/bin/python
          env_vars:
            # 钉死而不是依赖 shell：这两个是「训练在跑但看的是黑图 / 判定窗口被 GPU 侧覆盖」
            # 这两类静默故障的唯一防线，见 §S3.10
            - RLINF_SKIP_CAMERA: "0"
            - FRANKA_CUBE_WIDTH_M: "0.0325"   # 换成你的实测值
      hardware:
        type: Franka
        configs:
          - robot_ip: 172.16.0.2
            node_rank: 1
            camera_type: realsense
            camera_serials: ["420122070525"]  # 用 §S3.3 的 8a 实测值
            gripper_type: franka

runner:
  task_type: embodied
  logger:
    log_path: "../results"          # 启动脚本会用带时间戳的目录覆盖
    project_name: rlinf
    experiment_name: "cube_place_sac"
    logger_backends: ["tensorboard"]
  max_epochs: 200                   # 首跑试水；顺利后再抬到 8000
  max_steps: -1
  only_eval: False
  val_check_interval: -1
  save_interval: 50                 # 不要照抄 charger 的 -1
  resume_dir: null
  ckpt_path: null

# algorithm / rollout / actor / reward / critic 四节与
# realworld_cube_place_dummy_sac_gpu.yaml 完全相同，直接照抄。

env:
  group_name: "EnvGroup"
  train:
    rollout_epoch: 1
    total_num_envs: 1
    init_params:
      id: "FrankyCubePlaceEnv-v1"
    auto_reset: True
    ignore_terminations: True
    use_spacemouse: False           # 有 SpaceMouse 才改 True，见 §S3.6
    override_cfg:
      is_dummy: False
      robot_ip: 172.16.0.2
      # H1：必须与 b/x/configs/cube_place_target_ee_pose.yaml 逐位一致，
      # 用 `run_cube_place_phase2.sh connect` 打印的 target_ee_pose (H1) 对一遍。
      target_ee_pose: [0.726857066, 0.0249468647, 0.250846744,
                       -3.13747483, 0.0354833569, -0.00268804519]
      clip_x_range: 0.05
      clip_y_range: 0.05
      clip_z_range_low: 0.005
      clip_z_range_high: 0.08
      random_xy_range: 0.03
      random_rz_range: 0.35
      clip_rz_range: 0.35
      reset_z_lift_m: 0.03
      use_dense_reward: True
      enable_camera_player: False    # 无头容器，别开播放窗
  eval:
    # 与 train 同一套 override_cfg（同一条臂、同一个标记）；val_check_interval=-1 时不会被用到
    rollout_epoch: 1
    total_num_envs: 1
    init_params:
      id: "FrankyCubePlaceEnv-v1"
    override_cfg: { }               # 照抄 train 的那一份
```

**不要在 `override_cfg` 里写 `reset_ee_pose` / `ee_pose_limit_min` / `ee_pose_limit_max`。** `PegInsertionConfig.__post_init__` 会从 `target_ee_pose` + `clip_*` **重新推导并覆盖**它们（`reset = target + [0,0,clip_z_range_high,0,0,0]`；xy 用 `clip_x/y_range`，z 用 `clip_z_range_low/high`，roll/pitch 只给 ±0.01 rad，yaw 用 `clip_rz_range`），`CubePlaceConfig.__post_init__` 的注释已经写明这是刻意保留的上游行为。写了只会让 YAML 和实际生效的围栏不一致——那正是烟测脚本要打印 `effective ee_pose_limit_*` 的原因。

**② `b/x/scripts/run_cube_place_sac.sh`** —— 与 1B 的 `run_cube_place_dummy_sac_gpu.sh` 的**唯一实质差别是不 `ray start`**：双节点集群必须先按 §S3.2 手工起好，脚本只做断言。

```bash
#!/bin/bash
# 阶段 3：真机在线 SAC。只在 GPU 容器(rank 0, head)里跑。
set -euo pipefail
RUN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${RUN_SCRIPT_DIR}/../configs/setup_before_ray_gpu_5090.sh"
# 那个 setup 脚本把 RLINF_SKIP_CAMERA 默认成 1，而 source 发生在 ray start 之后，
# 会被当成「head 侧改动」广播到 franky 节点、把相机关掉（见 §S3.10 第 1 条）。
# YAML 的 env_configs 会压过它，这一行只是第二层防线。
export RLINF_SKIP_CAMERA=0

export EMBODIED_PATH="${REPO_PATH}/examples/embodiment"
export CONFIG_PATH="${REPO_PATH}/b/x/configs"
CONFIG_NAME="${1:-realworld_cube_place_sac}"; shift || true

[[ -f "${RLINF_RESNET10_PATH}/resnet10_pretrained.pt" ]] \
  || { echo "ERROR: missing resnet10_pretrained.pt" >&2; exit 1; }
python -c 'import torch; assert torch.cuda.is_available(), "need CUDA torch (not franky venv)"'

# 集群必须已经是两节点：rank0=本容器(head)，rank1=franky 容器。
ray status >/dev/null 2>&1 || { echo "ERROR: no ray cluster; see dmo_place_2 S3.2" >&2; exit 1; }
NODES=$(python - <<'PY'
import ray; ray.init(address="auto", logging_level="ERROR")
print(sum(1 for n in ray.nodes() if n["Alive"]))
PY
)
[[ "${NODES}" == "2" ]] || { echo "ERROR: expected 2 alive ray nodes, got ${NODES}" >&2; exit 1; }

LOG_DIR="${REPO_PATH}/logs/$(date +'%Y%m%d-%H%M%S')-${CONFIG_NAME}"
mkdir -p "${LOG_DIR}"
python "${EMBODIED_PATH}/train_async.py" \
  --config-path "${CONFIG_PATH}" --config-name "${CONFIG_NAME}" \
  runner.logger.log_path="${LOG_DIR}" "$@" 2>&1 | tee -a "${LOG_DIR}/run_embodiment.log"
```

**③ `b/x/configs/realworld_cube_place_camera.yaml`** —— §S3.3 的 serial 载体，照 `realworld_franky_camera.yaml` 写，但 `init_params.id` 换成 `FrankyCubePlaceEnv-v1`，并**补上 `is_dummy: false` 与 H1 六元组**（`step8_check_yaml.py` 会检查 `is_dummy is False`；而 cube-place 这条线的配置类会自己推导围栏，所以给了 `target_ee_pose` + `clip_*` 就是可构造的真机配置，和 `FrankyFrankaEnv-v1` 不同）。

**④ `step_cube_place_robot.py` 的 `--with-camera` 旗标**（**已实现**）—— 该脚本原先**无条件**写 `os.environ["RLINF_SKIP_CAMERA"] = "1"`，没法用来验 cube-place 链路上的相机。现在 `--with-camera` 会把它改成 `"0"`，并解析相机 serial / type：默认读 §S3.3 8a 写出的 `b/x/configs/camera_detected.json`（同 `step8_test_env_camera.py` 的默认 JSON），也可以用 `--camera-serials`（一个或多个）/ `--camera-type` 显式覆盖、用 `--camera-json` 换一个 JSON 路径。解析到的 serial 会被拒绝占位符（复用 `step8_checks.is_placeholder_serial`），并连同按 `wrist_{i}` 生成的 `camera_names` 一起塞进 `override_cfg` 与硬件配置。`reset()` 之后打印每路 `frames` 的 `shape/dtype/min/max/mean`（全零帧额外标 `ALL-ZERO`，但不中断脚本——这是诊断信息，不是断言）。这样 §S3.3 的第二步不必新写脚本：

```bash
# franky 容器内（已 source setup_before_ray_5090.sh）
python b/x/scripts/step_cube_place_robot.py --reset-only --with-camera
# 或显式给 serial（跳过 camera_detected.json）：
python b/x/scripts/step_cube_place_robot.py --reset-only --with-camera \
  --camera-serials 420122070525 --camera-type realsense
```

#### S3.2 为什么必须两个容器，以及启动顺序

```
        单台 5090 主机（两个容器都用 --network host，仓库都挂在 /workspace/RLinf）
  ┌──────────────────────────────┐        ┌────────────────────────────────────┐
  │ GPU 容器  RLINF_NODE_RANK=0  │        │ franky 容器  RLINF_NODE_RANK=1     │
  │ ray start --head             │◀──────▶│ ray start --address=<ip>:6379      │
  │  driver: train_async.py      │        │  env worker: FrankyCubePlaceEnv-v1 │
  │  actor + rollout (CUDA torch,│        │  FrankyControllerExtended ─► 1337  │
  │  ResNet10)                   │        │  RealSense wrist_1 (USB)           │
  └──────────────────────────────┘        └──────────────┬─────────────────────┘
                                                          ▼  Franka 172.16.0.2
```

单容器跑不通，两边各缺一半：franky 容器的 venv 是 `torch==2.5.1+cpu`（`step7_install_deps.sh` 故意装的 CPU wheel）且容器起来时没有 `--gpus`；GPU 容器里没有 franky/libfranka，也不该由它去占 1337。RLinf 恰好为这种情形提供了 `node_groups[].env_configs[].python_interpreter_path`——每个 worker actor 用它自己节点的解释器启动，所以「控制器在 franky venv、训练在 CUDA venv」是配置问题而不是打包问题。

**顺序不可换**（`RLINF_NODE_RANK` 必须在 `ray start` **之前**导出，Ray 抓的是 raylet 启动时的环境）。下面每一块都标了执行环境——**这是阶段 3 最容易看错的地方**：`docker_run_*.sh` 在**宿主机**跑（它会 `docker run` 进容器，之后的命令都在那个容器里），`export` / `source` / `ray start` 都在**各自容器内**跑：

```bash
# ① franky 容器（先起 worker 还是先起 head 都行，但两边 rank 必须不同）
#    ↓ 宿主机：创建并进入 franky 容器（这个终端此后就是容器的前台 shell）
bash b/x/configs/docker_run_franky_5090.sh
#    ↓ franky 容器内（上面那条命令进来之后，或 docker exec -it rlinf-franky-5090 bash）
export RLINF_NODE_RANK=1                            # 关键：默认是 0，会和 GPU 容器撞
export ROBOT_IP=172.16.0.2                          # 供 FrankaConfig 自动填充
source b/x/configs/setup_before_ray_5090.sh         # 运动权限 + PYTHONPATH + RLINF_EXT_MODULE
ray start --address=<宿主机IP>:6379

# ② GPU 容器
#    ↓ 宿主机：创建并进入 GPU 容器
bash b/x/configs/docker_run_gpu_5090.sh
#    ↓ GPU 容器内，或 docker exec -it rlinf-gpu-5090 bash
export RLINF_NODE_RANK=0
source b/x/configs/setup_before_ray_gpu_5090.sh
ray start --head --port=6379 --node-ip-address=<宿主机IP> --disable-usage-stats
ray status                                          # 必须看到 2 个 alive 节点
```

两个容器 `--network host`，所以 `node_ip` 一样——**这不影响**：RLinf 用 `RLINF_NODE_RANK` 排序、用 Ray 的 `NodeID` 把 worker 钉到物理节点。真正会炸的是**两边 rank 相同**（`_sort_nodes` 要求 rank **唯一且从 0 连续**，不满足直接 assert）或**只有一边设了 rank**（要么全设、要么全不设，混着也 assert）。driver 只在 rank 0 跑。

**起完之后必须跑验收脚本（宿主机上，LOG-029 首次通过）：**

```bash
# 宿主机（只需要 docker 和 ss，不需要任何 venv）
bash b/x/scripts/verify_ray_cluster.sh
# 期望末行: RESULT verify_ray_cluster PASS（14 项 CHECK 全 OK）
```

它检查：两个容器在跑且镜像正确；1337 无人占用（集群不该碰臂）；**每个容器里 raylet 进程的环境中 `RLINF_NODE_RANK` 是捕获对了的**（rank 错了只能重启该节点的 ray，事后改环境变量无效）；head 上 `ray status` 恰有 2 个 alive 节点；YAML 里 `python_interpreter_path` 指向的解释器在两个容器里都真实存在且与 `which python` 一致；franky 在 franky 容器可 import、CUDA torch 在 GPU 容器可用；`FrankyCubePlaceEnv-v1` 在**两个**容器都能注册；**端到端**用 `NodeAffinitySchedulingStrategy` 把一个 task 钉到每个节点上，回读该节点的 `RLINF_NODE_RANK` / hostname / python 路径（证明调度与环境捕获都对）；ResNet10 权重在 GPU 容器内可见。任何一项 FAIL 都不要进 §S3.3。

#### S3.3 相机验收（阶段 3 的第一步，臂几乎不动）

烟测至今全程 `RLINF_SKIP_CAMERA=1`（§7 T6），而 skip 时 `_get_camera_frames` 返回的是**按 obs space 形状造的全零 uint8 帧**，不是缺键、不是报错。也就是说：**训练可以一路跑下去，而策略看到的是纯黑图**，指标上只表现为「学不动」。所以这一步必须在开训前独立做完。

**自动执行（推荐）：** 一条命令跑完 8a → 8b → mode gate → 8c → 链路自检，逐步验证机器可读的通过标记（`RESULT Step8x PASS` / `reset-only PASS` / 无 `ALL-ZERO`），失败时列出失败步骤并给出日志目录：

```bash
# franky 容器内或docker exec -it rlinf-franky-5090 bash 进入后（已 source b/x/configs/setup_before_ray_5090.sh）
bash b/x/scripts/run_cube_place_camera_accept.sh
# 只验 8a+8b（完全不碰机器人，连 FCI 都不占）：
bash b/x/scripts/run_cube_place_camera_accept.sh --no-fci
```

**通过标准：最后一行 `CAMERA_ACCEPT PASS: 8a + 8b + mode gate + 8c + cube-place 链路`，退出码 0**；每步完整输出在 `b/x/logs/camera_accept/<时间戳>/`。脚本内置的门（任何一道不过即中止，不动臂）：

1. **mode gate**：`diag --probe` 必须报 `robot_mode=RobotMode.Idle`。
2. **内存门**（LOG-031）：`MemAvailable ≥ 20GB`——控制器 worker 初始化约 16GB，宿主机内存用过 95% 时 Ray 会直接 OOM 杀掉它（8c 的首个真实失败就是这么挂的）。
3. **集群门**（LOG-031）：检测到已有 Ray 集群**默认中止**。8c/链路自检必须跑在 franky 容器的**单节点**集群上——复用 §S3.2 的双节点集群会让 `node_rank=0` 变成 GPU 节点（`step8_test_env_camera.py` 的 `FrankaConfig(node_rank=0)` 把控制器调度到没有相机 USB 的容器），且其 idle worker 占几十 GB 内存。没有集群时脚本自起单节点、结束 `ray stop`；确认现有集群就是本容器的单节点集群时才可加 `--reuse-cluster`。**如果 §S3.2 的双节点集群还在跑，先在宿主机停掉再执行**（这也顺带释放约 50GB）：

```bash
# 宿主机
docker exec rlinf-franky-5090 /opt/venv/franky-0.19.0/bin/ray stop
docker exec rlinf-gpu-5090    /opt/venv/openvla/bin/ray stop
```

4. **serial 以训练 YAML 为锚**（LOG-031/032）：8a 默认按 `realworld_cube_place_sac.yaml` 的 `camera_serials` 做白名单过滤（`--serials`），**多插的相机自动忽略、不用拔**；要用的相机不在线会在 8a 的 `requested_serials_present` 处 FAIL。8b 再兜底核对 检测 JSON == 载体 YAML == 训练 YAML（`--expect-serials-from`，顺序也算——第一个 serial 就是 `wrist_1`）。想改用别的相机或双相机：先改训练 YAML 的 `camera_serials` / `camera_names` 再重跑。注意运行时硬件校验只查「配置的 serial 都在线」，插着不配的相机本身无害——但 `camera_serials` 千万别留空/`null`，那会自动填上**全部**探测到的相机（`franka.py` 的 auto-detect 分支）。

此外 8c 不过则跳过链路自检（先修相机本身，再查 cube-place 链路的接线）。注意 8c 与链路自检仍会占 FCI 并 reset 臂（8c 位移≈0，链路自检是真的 reset 到悬停）：方块夹紧、user-stop 松开、人在急停旁。

> 该脚本依赖的配套小修（LOG-030/LOG-031）：`step8_check_yaml.py` 新增 `--expect-gym-id`（原来硬编码 `FrankyFrankaEnv-v1`，对 cube-place 载体必挂）与 `--expect-serials-from`（三方 serial 核对 + `camera_names` 唯一性）；`step8_detect_cameras.py --write-yaml` 同步 `env.train` 与 `env.eval` 两段 serial，且 `camera_names` 改为按写入的 serial 重建（原来 setdefault 到残留键上，双相机时会把两个 serial 都映射成 `wrist_1`）。注意 `--write-yaml` 用 `yaml.safe_dump` 回写，**会丢掉载体文件里的注释**并把当时探测到的所有 serial 都写进去。

**手动分步（排障 / 想看中间输出时用）：**

**8a 检相机（无 FCI，无臂）：**

```bash
# franky 容器内或docker exec -it rlinf-franky-5090 bash 进入后（已 source b/x/configs/setup_before_ray_5090.sh；相机 USB 挂在这个容器）
source b/x/configs/setup_before_ray_5090.sh
python b/x/scripts/step8_detect_cameras.py --write-yaml \
  --yaml-out b/x/configs/realworld_cube_place_camera.yaml
python b/x/scripts/step8_check_yaml.py \
  --json-in  b/x/configs/camera_detected.json \
  --yaml-in  b/x/configs/realworld_cube_place_camera.yaml \
  --expect-gym-id FrankyCubePlaceEnv-v1 \
  --expect-serials-from b/x/configs/realworld_cube_place_sac.yaml
```

期望：`RESULT Step8a PASS` / `RESULT Step8b PASS`（退出码均为 0）；`camera_detected.json` 里 serial 非占位（本机是 `420122070525`，D435I）；YAML 与 JSON 的 serial 一致；`wrist_1` 命名存在。注意 `_build_camera_infos` 对单相机的**默认**名就是 `wrist_1`（`wrist_{i}`，i 从 1），`camera_names` 只是把它写明。

**8c 真机 + 相机（要 FCI，臂只做零位移 reset）：**

```bash
# franky 容器内或docker exec -it rlinf-franky-5090 bash 进入后（已 source setup_before_ray_5090.sh；单容器单节点，验完就 ray stop）
# 前提（LOG-031）：没有其他 ray 集群在跑（有就先停，见上面集群门），且 MemAvailable >= 20GB
export RLINF_SKIP_CAMERA=0 FRANKA_ROBOT_IP=172.16.0.2
ray start --head --port=6379 --disable-usage-stats
python b/x/scripts/step8_test_env_camera.py --save-jpeg --require-live
ray stop
```

期望：`obs['frames']['wrist_1']` 是 `uint8`、`128×128×3`、**`max>0`**（全零就是 skip stub 或采集失败）；`--require-live` 要求零动作步之间帧有变化（能排除「拿到的是同一张缓存」）。注意该脚本用的是 `FrankyFrankaEnv-v1` 且 `target/reset` 都取当前实测位姿，所以 `reset()` 的位移≈0——但它**确实会 reset**，臂上仍要夹着方块、人在急停旁。

**cube-place 链路自检（franky 容器内）：** 用 §S3.1 的 ④ 跑一次 `bash b/x/scripts/run_cube_place_phase2.sh reset --with-camera`，确认 `FrankyCubePlaceEnv-v1` 这条线上 `wrist_1` 同样非全零。8c 过而这一步不过，说明是 override/YAML 的相机键没传到，不是相机坏。

#### S3.4 开训前的人工准备（**训练路径没有 Ray 前硬门**）

这是阶段 3 与阶段 2 最危险的一处差别：`tcp_probe` 的那三道 Ray 前硬门（`robot_mode`、起始位形、夹爪 holding）**只存在于烟测和诊断脚本里**，`train_async.py` 一条都不跑。env 侧只剩 `FrankyCubePlaceEnv._check_start_pose` 的一句 **warning**（「起始位置低于盒底、方块被压在标记上」）。所以顺序是：

1. **franky 容器内** REPL：`open` → 放方块 → `close`，记下 `measured width`，需要就更新 `FRANKA_CUBE_WIDTH_M`（§2.5 步骤 1）。
2. 按引导键把臂送到**标记上方 3–5 cm**，`q` 退出 REPL（释放 1337）。
3. **franky 容器内**跑一次 `bash b/x/scripts/run_cube_place_phase2.sh connect` —— 这就是阶段 3 的 pre-flight：它会跑那三道硬门、打印 `effective ee_pose_limit_*` / `reach:` / `holding=True`，且不动臂。这一步 exit 0 才继续。（该 connect 检查已并入宿主机脚本 `preflight_cube_place_sac.sh` 的 C 层，开训前以脚本为准，见 §S3.5 第 0 步。）
4. **宿主机**确认没有别的 libfranka 客户端：`ss -tn state established '( sport = :1337 or dport = :1337 )'` 应为空（两个容器都 `--network host`，在宿主机查一次就覆盖两边）；REPL、烟测、8c 的 ray 都已退出。
5. 按 §S3.2 起双节点集群（此时才 `ray start`；顺序反了运动权限就进不去 raylet）。
6. 人站在急停旁，手不按 user-stop。

#### S3.5 启动训练与前五分钟要盯的

**第 0 步：训练前检查脚本（宿主机，LOG-031 创建、LOG-032 加自动修复）。** 在敲启动命令之前，先在宿主机跑一次一键检查——它把 §S3.4 的人工六步里所有能机器化的部分、§S3.2 的集群验收、以及训练 YAML 的静态一致性集中成一次运行：

```bash
# 宿主机（只需 docker + python3 + ss，不需要任何 venv）
bash b/x/scripts/preflight_cube_place_sac.sh
# 臂不在线 / 只想查配置与集群时：bash b/x/scripts/preflight_cube_place_sac.sh --skip-robot
# 期望末行: RESULT preflight PASS
```

脚本分三层，按代价从低到高，前面的层 FAIL 就跳过后面的层。**FAIL 时的行为（LOG-032 起）**：先用 `*****` 星号横幅把失败项、文档指针括出来，然后**自动尝试修复**——修成了报 `AUTO-FIXED` 并继续，修不了（或该项本质上不可自动修）才留下 `FAIL` + `MANUAL ACTION REQUIRED` 和人工指引。自动改 YAML 前会留一次性备份 `realworld_cube_place_sac.yaml.preflight-bak`：

| 层 | 覆盖 | 自动修复 | 修不了时的人工指针（脚本会打印，此处只列大类） |
|---|---|---|---|
| A 静态 | H1 已标定且与训练 YAML 逐位一致、`FRANKA_CUBE_WIDTH_M` 已钉且非默认 0.046、`RLINF_SKIP_CAMERA: "0"` 已钉、相机 serial **以训练 YAML 为锚**（hardware/train/eval/`camera_names` 四处一致、非占位、且每个都被 USB 实测到；`camera_detected.json` 里多插的相机**忽略**，见 LOG「第二只相机插着但不配」）、eval 非 dummy、`save_interval` 已设、camera player 已关 | **可**：H1→YAML 位姿同步、`RLINF_SKIP_CAMERA` 钉 0、serial 以 hardware.configs 为锚同步 override 与 `camera_names`（**不是**从 json 往 YAML 加相机）、`is_dummy`/`save_interval: 50`/`enable_camera_player: false` 改值。**不可**：H1 标定本身、方块宽度（都是物理测量，猜错了会悄悄弄坏夹爪判定）、YAML 里的 serial 没被 USB 实测到（插相机是物理动作） | §2.5、§S3.1 顶部 EDIT ME、§S3.3 8a、§S3.10 第 1/6 条、§S3.0 对照表 |
| B 集群 | 直接调用 `verify_ray_cluster.sh` 的全部 14 项（§S3.2） | **可**（仅当失败项都是「启动顺序」类：`container_*`/`raylet_*`/`ray_status_*`/`e2e_pinned_tasks`/`gym_id_*`）：`docker start` 停掉的容器 + 按 §S3.2 顺序双节点 ray 重启（rank 在 `ray start` 前导出），然后自动复验。**不可**：1337 被占、ResNet10 权重缺失、venv/解释器路径错 | §S3.2（重起集群）/ §S3.1（改 EDIT ME）/ §6「Couldn't connect」 |
| C 机器人 | 无存活 `FrankyControllerExtended` actor、1337 空闲、franky 容器内 `run_cube_place_phase2.sh connect` 三道硬门（robot_mode / 起始位形 / 夹爪 holding）、`authority:` 回显是 20 N/axis 而非 100 N | **全部不可**——杀占臂进程、重新夹块、引导臂回位都是人的决定，脚本拒绝代劳并说明理由 | §S3.4 步骤 1–4、§6 对应行 |

**脚本不覆盖、仍需人工确认的三条**（脚本 PASS 之后逐条过目）：① 2.4b 的 `peak_overshoot` 至今没有真机数据（T5/T11）；② 标记与方块自 H1 标定后没有被挪动过；③ 人站在急停旁。

**第 1 步：启动训练。**

```bash
# GPU 容器内（rank 0，head；已 source b/x/configs/setup_before_ray_gpu_5090.sh，
# 且 §S3.2 的双节点集群已起好、ray status 显示 2 个 alive 节点）
bash b/x/scripts/run_cube_place_sac.sh realworld_cube_place_sac
# 试水可加：runner.max_epochs=20 algorithm.replay_buffer.min_buffer_size=2
```

按顺序应看到（缺任何一条就停下来查，不要「先跑跑看」）：

| # | 期望 | 缺了意味着 |
|---|------|-----------|
| 1 | `RLinf is running on a cluster with 2 node` | 集群没起对，回 §S3.2 |
| 2 | env worker 的日志前缀落在 **franky 容器**、actor/rollout 在 GPU 容器 | placement 写错，env 跑到 GPU 容器上去了（那里没有 franky） |
| 3 | `FrankaLibfrankaGripper connected (... cube_width=<你的实测值> ... holding=True)` | **这一条 LOG-034 首跑就没过**：YAML 钉了 0.0325、preflight 也 OK，这里却是 `cube_width=0.0460 ... holding=False width=0.0314`。原因是 YAML 的 `env_configs.env_vars` 到不了控制器 actor（§7 T17、§S3.10 第 1 条的警告框）。**这一行是唯一的生效值证据**，宽度不对就得回 franky 容器 `ray stop` → export → re-source → `ray start`，不能改 YAML 了事 |
| 4 | `authority: spring force <= 20.0N/axis ... interp<=2.0cm/s` + `collision behavior tightened: ... 40.0 ... 12.0` | 权限没生效，**立刻停**（LOG-019 的权限） |
| 5 | `motion guard armed: ...` + `motion guard confirmed: ...` + `reach: ...` | 围栏没装 |
| 6 | 第一段运动是**抬到悬停**（`cube_place go_to_rest` → `rest pose reached`） | 直冲桌外/扫地 → 急停，查 `target_ee_pose` 是否填反、是否米 |
| 7 | `env/reward` 不是恒 0 | `use_dense_reward` 没生效（§S3.0 对照表里 env 包那一行） |
| 8 | 每回合 100 步左右自动 reset，方块始终夹着 | 张爪/掉块 → §6 |
| **9** | **策略接手后的头 30 秒不出现 `WATCHDOG trip [lag]`** | **LOG-034 首跑就是死在这里**（第 10 秒，`0.0719m > 0.0500m`、`\|dq\|=2.69rad/s`）。前 8 条全绿也不代表能活过第 10 秒：那 8 条查的是启动配置，第 9 条查的是**策略的授权**。根因已实测定为**雅可比放大**（§7 T19），修法已落地（§S3.11），所以第三次开训时这一条应该能过。**多盯一条**：`step slew clamped: joint demand ...` 的出现频率——偶发说明限幅在边缘工作，持续刷屏说明策略在坏条件区打转，那要收工作区而不是放宽限幅。真跳了按 §S3.7 的 `WATCHDOG trip` 行处理，**不要放宽围栏**；若跳闸消息是 `[watchdog:dq] joint runaway` 而不是 `[lag]`，那是 ② 生效了、诊断名字对了。**自 LOG-041 起跳闸不再终止训练**（§S3.12）：紧跟着应看到 `guard trip recovered (N of 10 used)`，本回合按 truncated 结束、下一次 reset 重新逼近悬停位姿。此时要盯的变成**跳闸频率**——预算被稳定消耗说明该收工作区，而不是把预算调大 |

#### S3.6 训练中的人工介入（SpaceMouse，可选）

cube-place 的 env 包**刻意**把 `use_spacemouse` 钉成 `False`（charger 是 `True`）：没有设备时 `SpacemouseIntervention` 会去找 HID 设备。有设备再改成 `True`。开启后的语义：

- 推动手柄 → 该步的动作被**替换**为人的增量，并在之后约 0.5 s 内继续用专家动作；这些 transition **照常进 replay buffer**，所以「人把方块送到标记」就是一次纠正样本。
- `no_gripper: True` 时 wrapper 的 `gripper_enabled=False`：**推得动臂，张不开爪**。想张爪只能停训练回 REPL。
- 指标里会出现 `env/intervened_once`、`env/intervened_steps`、`env/success_no_intervened`——最后这个才是「不靠人也成功」的那部分。

#### S3.7 掉块 / 急停 / 续训

| 情况 | 处理 |
|------|------|
| 掉块（env 还连着） | **不要**在 env 连着时张爪。**GPU 容器** Ctrl+C 停训练 → 确认臂静止 → **franky 容器** REPL `open`/捡起/`close`（复现 H1 握姿）→ 引导回标记上方 → `q` → 按 §S3.4 重新 pre-flight → 带 `runner.resume_dir=...` 续训 |
| 急停 / user-stop | 按 §6 最后几行恢复：**两个容器各自** `ray stop` → Desk 清 fault → 松 user-stop → 重新 Activate FCI → **franky 容器** `diag-probe` 确认 `Idle` → 引导回起始位形 |
| `motion guard abort` / `WATCHDOG trip` | 控制器只刹车 + 锁存，env 侧轮询抛出普通异常。**LOG-034 真机首次跳闸实测：前四句齐全**（`brake ... stop=clean` → `WATCHDOG trip` → env 侧 `RuntimeError` → `refusing to command motion`），**但 `finally` 的 teardown health 不会出现**——训练路径的 driver 会 `ray.kill` 掉控制器 actor（T15）。所以别等那一行，直接按下面两步走：① **不要放宽围栏、不要立刻重跑**；② 宿主机 `docker exec ... run_cube_place_phase2.sh diag-probe` 确认 `Idle`/`has_errors=False`（`stop=clean` 的刹车通常**不留 Desk fault**），然后回 §2.4b 跟那张实测基线表逐列比 `peak_lag` / `peak\|dq\|`——差一个数量级就说明异常不在阻抗链路，而在某个没被限住的授权（当前已知：旋转，§7 T16） |
| 续训 | `save_interval: 50` → `<log_path>/<experiment_name>/checkpoints/global_step_<N>/`（本配置里 `experiment_name: cube_place_sac`，目录里还有一层 `actor/`，`resume_dir` 要指到 `global_step_<N>` 而不是 `actor`）。续训（**GPU 容器内**）：`bash b/x/scripts/run_cube_place_sac.sh realworld_cube_place_sac runner.resume_dir=<该目录>`。恢复的是 actor 权重 + optimizer + alpha + target 网络 + **replay buffer**；`global_step` 从目录名解析。embodied runner **不支持** `resume_dir: auto`，必须给完整路径 |

#### S3.8 指标（TensorBoard）

```bash
# 宿主机（或任何能读到 <REPO>/logs 的环境；不需要 GPU/franky venv）
tensorboard --logdir <REPO>/logs --port 6006     # 目录由启动脚本按时间戳生成
```

| 命名空间 | 关键键 | 读法 |
|---|---|---|
| `env/` | `reward`、`return`、`episode_len`、`success_once` | `success_once` 是「本回合是否碰到过标记」；恒 0 且 `reward` 也不涨 = 策略没学到，恒 0 但 `reward` 在涨 = 还差最后几毫米 |
| `env/` | `intervened_once`、`intervened_steps`、`success_no_intervened` | 只有开了 SpaceMouse 才有意义 |
| `train/sac/` | `critic_loss`、`actor_loss`、`alpha_loss`、`alpha` | `alpha` 一路上冲 = 熵目标太高（`target_entropy=-4` 对 6D 动作是 `-action_dim`） |
| `train/actor/` `train/critic/` | `lr`、`grad_norm`、`entropy`、`q_data` | `critic/q_data` 一路发散通常先于策略发疯，是最早的预警 |
| `train/replay_buffer/` | `num_trajectories`、`total_samples` | 真机每回合约 100 步，涨得比仿真慢两个数量级 |
| `train/replay_channel_qsize` | 队列深度 | 一直涨 = actor 消费不过来（异步的正常现象，但持续变大要留意） |
| `time/` | `step`、`actor/*`、`env/*`、`rollout/*` | 真机瓶颈几乎总在 `time/env`（10 Hz 硬上限） |

#### S3.9 阶段 3 验收门闩

| ID | 项 | 通过标准 |
|----|----|----------|
| 3-0 | 权限 | 日志 `authority:` 仍是 `<= 20.0N/axis`（范数 34.6 N）/ `<= 6.00Nm/axis`，且 `collision behavior tightened: ... 40.0 ... 12.0` |
| 3-1 | 放置 | `2 node`；env worker 与控制器在 franky 容器，actor/rollout 在 GPU 容器 |
| 3-2 | 相机 | §S3.3 全过，且训练里 `wrist_1` 非全零（黑图不算通过） |
| 3-3 | 不超程 | 全程无 `motion guard abort`；目视 z 未冲过标记上方 8 cm |
| 3-4 | 闭爪 | 全程 `gripper_open == False`，方块未掉、未被夹扁 |
| 3-5 | 学习 | `env/reward` 与 `env/success_once` 有上升趋势；目视方块多次碰到标记 |
| 3-6 | 可续训 | 至少存过一个 `global_step_<N>`，且用 `runner.resume_dir` 成功从它续跑 |

#### S3.10 已核实的坑（每条都对过代码）

1. **环境变量会被 GPU 侧悄悄覆盖。** 优先级是（`NodeProbe._configure_node_envs` 的 docstring 写得很明白）：① 各节点 `ray start` 之前的环境 → ② **head 节点上 `ray start` 之后、RLinf 初始化之前**新增/改变的环境（driver 的 `os.environ` 与 head raylet 启动环境做差，差异**广播到所有节点并覆盖节点自己的值**）→ ③ YAML `env_configs.env_vars`（在 `allocate` 时应用，**最高**）。

   两个已经埋好的具体地雷：`setup_before_ray_gpu_5090.sh` 里有 `export RLINF_SKIP_CAMERA="${RLINF_SKIP_CAMERA:-1}"`，而 `run_cube_place_sac.sh` 会 source 它——**这发生在 `ray start` 之后**，于是 `RLINF_SKIP_CAMERA=1` 作为「head 侧改动」被广播到 franky 节点，把相机关掉，策略从此看黑图；同理在 GPU 容器里随手 `export FRANKA_CUBE_WIDTH_M=0.046` 会改掉机器人侧的夹持判定窗口，而 franky 容器里那句 `export ...=0.0316` 完全无效，日志上也看不出是谁改的。

   防法：把这两个键写进 YAML 里 franky 组的 `env_configs.env_vars`（层级 ③ 压过 ②），并且**不要**在 GPU 容器 `ray start` 之后再改任何 `FRANKA_*` / `RLINF_CUBE_*` / `RLINF_SKIP_CAMERA`。

   > ⚠️ **层级 ③ 只对「在该 node group 里启动的 worker」生效——控制器 actor 不在其内**（LOG-034 实测，§7 T17）。`launch_controller` 用的是 `NodePlacementStrategy(node_ranks=[...])`，落进保留的 `node`/`cluster` 组（`env_configs: null`），而 `Cluster.allocate` 只合并**当前组**的 `env_vars`。于是同一个键会出现两个不同的生效值：
   >
   > | 谁 | 走哪个组 | 拿到什么 |
   > |---|---|---|
   > | `EnvGroup`（env worker，读 `RLINF_SKIP_CAMERA`） | `franky`（`component_placement`） | YAML 的值 ✅ |
   > | `FrankyControllerExtended`（读夹爪与 `RLINF_CUBE_*`） | `node`/`cluster` | **节点 raylet 环境 + head 侧广播** ❌ |
   >
   > 首跑实证：YAML 钉了 `FRANKA_CUBE_WIDTH_M: "0.0325"`、preflight 也 OK，控制器却回显 `cube_width=0.0460m ... holding=False width=0.0314m`。**所以对住在控制器里的量（夹爪三项、`RLINF_CUBE_*` 全套权限），唯一可靠的办法仍是 franky 容器 `ray start` 之前 export**（即 `setup_before_ray_5090.sh` 干的事）；YAML 那份只对 env worker 有效。改这些量的操作纪律不变：`ray stop` → 改 → re-source → `ray start`。判断有没有生效**只看控制器启动时那两行回显**（`FrankaLibfrankaGripper connected (... cube_width=...)` 和 `authority: ...`），不要看 YAML。
2. **`hardware: type: Franka` 的校验不碰 1337。** 集群枚举阶段它只做 `icmplib.ping`（2 次 1 s）+ 相机 SDK/serial 比对；`robot_ip` 留空则从 `ROBOT_IP` 环境变量取；`disable_validate: true` 会把 ping 和相机检查一起跳过。真正独占 1337 的是 env 初始化时起的 `FrankyControllerExtended`——所以**训练与 REPL / 烟测 / 8c 三者互斥**这条铁律在阶段 3 依然成立。
3. **训练时 reset 是带随机化的。** `enable_random_reset` 继承 PegInsertion 的默认 `True`（env 包没覆盖），于是每次 reset 的悬停 xy 会叠加 `±random_xy_range`（0.03）的抖动、yaw 叠加 `±random_rz_range`。这是**刻意**的域随机化，但意味着：阶段 2 那种「hover check 的 xy 误差 ≤0.03」不再是可重复的判据，别把训练日志里的悬停偏差当回归。想临时关掉做对照，在命令行加 `env.train.override_cfg.enable_random_reset=false`。
4. **策略的物理幅度被夹到 5 mm/步。** `CubePlaceConfig.__post_init__` 把 `action_scale[0]` 夹到 `RLINF_CUBE_STEP_SPEED / step_frequency`（默认 0.05/10 = 0.005 m/步），所以 `action=±1` 只走 5 mm、峰值 5 cm/s；启动时那句 `WARNING: action_scale[0]=0.0200 ... clamping to 0.0050` 是**正常**的。这个 5 cm/s 仍是推算值（T12），改它请改 `RLINF_CUBE_STEP_SPEED` 而不是代码，且必须在 franky 容器 `ray start` 之前改。
5. **`RLINF_EXT_MODULE` 两个容器都要有。** 值是 `franky_ext.runtime_bootstrap`（不是 `franky_ext.tasks.register`）：它先补 NO_ACCEL/CPU 的 torch 平台，再 `import franky_ext.tasks.register` 完成 gym 注册；两个 setup 脚本已经默认导出。gym id 在 GPU 容器里也能注册成功（1B 就是这么跑的），因为注册本身不 import franky 后端。
6. **`env.eval` 不要留 dummy。** 1B 的配置里 `env.eval.override_cfg` 是 `is_dummy: True` 的占位；真机配置里若把它留着而 `val_check_interval` 又不是 `-1`，验证时会去构造一个 dummy env，指标看起来「有」但和机器人无关。要么两段用同一份 override，要么把 `val_check_interval` 明确设成 `-1`。
7. **`env.close()` 那句 `'VideoPlayer' object has no attribute 'stop'`** 在训练里也会出现（来自 `FrankaEnv.close` 里的 `camera_player.stop()`），是已知无害警告，见 §6。

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
| `reset_z_lift_m` | 0.10（PegInsertion 硬编码） | **0.03** | charger 抬 10 cm 是为了拔插头；平面标记只需脱离接触 |
| **`K_t × clip`（逐轴力上限）** | 隐式 100 N | **20 N**（范数 ≤40 N） | 见下 |
| **libfranka 硬件反射** | 100 N / 25 N·m | **40 N / 12 N·m** | 唯一不靠 Python 调度的界 |
| **`step()` 速度上限** | 隐式 20 cm/s | **5 cm/s**（`action_scale[0]` 0.02→0.005） | `step` 不走 `_interpolate_move` |
| **插值速度** | 隐式 10 cm/s | **≤2 cm/s** | 见下 |

**这两行是 LOG-019 的核心，单独说明：**

`clip` 不是独立旋钮。指令力 = `刚度 × 误差截断`，所以**要恒定的是这个乘积**。旧代码让刚度来自 `compliance_param`（2000）而截断来自 franky 默认（0.05），乘出 100 N/轴；现在 `error_clips_for_stiffness` 反过来算：`clip = 力上限 ÷ K_t`，并同时满足范数上限（clip 是**逐轴**的，三轴饱和时是 `K·clip·√3`）。同理时长不是旋钮，**速度**才是：`interp_duration_s` 从位移反推时长，而不是像上游那样硬编码 `timeout=1`。

20 N/轴够不够？LOG-018 实测 1 cm/s 下滞后 4.7 mm，即只用了约 `2000 × 0.0047 = 9.4 N`，其中约 8 N 是**与速度无关的静摩擦**（franky 不做摩擦前馈）。2 cm/s 时约 11 N，对应 5.5 mm 滞后，小于 10 mm 的 clip → 不饱和，但**边际只有约 1.8 倍**。若 2.4b 出现「抬不动」或 `ceiling not achieved` 警告，用 `--force-ceiling 30` 复测，**不要直接改代码**。

注意 `K_t × clip` 只界定**弹簧项**：阻尼项（`K_t=2000` 时约 155 N·s/m）不被裁剪，只阻碍运动而不驱动超程，但人手拦臂时的峰值力不受这些数字约束（T14）。

阻抗仍原样复用 PegInsertion 的 `compliance_param`（K_t=2000），只是截断跟着它走。接触力靠 z 下沿 + 笛卡尔阻抗，**不要**一上来加刚度。真要调，先在 2.4b 的诊断脚本里调通再进 env。

---

## 6. 排障总表（按症状索引）

| 症状 | 处理 |
|------|------|
| `motion guard abort [fence] ...` / `[orient] ...` | 臂冲出围栏或腕部被甩，已按「先 `stop()`」刹住。日志里有刹车期间走过的距离，末尾会说 `arm braked and latched; ... the env-side poll will raise`。**不要放宽围栏**，回 2.4b 降 `--force-ceiling` |
| `motion guard abort [lag] ...` | 臂跟不上目标，已按「先 `set_target(测量)`」刹住。力上限太低 / 位形权限不足 / tracker 已死，看同段日志的 `mode=` 和真因 |
| `motion guard WATCHDOG trip [lag]: not tracking: \|measured-commanded\|=0.0xxx > 0.0500m`，**且训练刚开始几秒**（LOG-034） | 先看同段日志里的 `brake (...): \|dq\| X -> Y`。**`\|dq\|` 是判据**：跟 §2.4b 那张实测基线表比，正常档位是 **0.01–0.11 rad/s**；若像首跑那样是 **2.69 rad/s**，说明臂在自己飞，不是「跟不上」——这一档不要去调力上限（那是反方向）。**根因已实测查明（§7 T19）：不是某个授权没被限住，而是笛卡尔限幅与关节速度之间的比例随位形变化 14.6 倍**，所以修法在关节空间（§S3.11），任何笛卡尔数字的微调都是徒劳。若跳闸消息已经是 `[watchdog:dq] joint runaway` 而不是 `[lag]`，说明 ② 生效了，直接看它打印的 `q` 定位坏位形。**先跑 diag-probe 确认 `Idle`/`has_errors=False`（`stop=clean` 通常不留 Desk fault），再回 §2.4b 对基线，不要立刻重跑训练** |
| `motion guard tripped and the arm was braked: [...]` | 这一句是 **env 侧**抛的普通异常（控制器只刹车 + 锁存），所以 `finally` 的 `env.close()` / `controller health at teardown` 都会正常跑。方括号里就是控制器锁存的原文（含 `[fence]` / `[lag]` / `[orient]` 和刹车前后的 TCP）。Desk 清 fault、复核起始位形，再看本表其它行 |
| `refusing to command motion: the motion guard is tripped (...)` | 跳闸之后**又**有人下发运动（多半是同一次 reset 里后续的路点）。臂保持刹住、tracker 不会被重建，这是刻意的。**在训练路径上这一句现在应该很少见**：自 LOG-041 起 env 会先调 `recover_from_guard_trip()` 显式解闩锁再继续（§S3.12），所以看到它说明恢复被**拒绝**了（臂没停稳、或 Desk 故障清不掉）——那就照旧重启进程，并先跑 `diag-probe` 查清为什么。烟测/阶段 2 脚本不走恢复路径，那里看到它仍是「重启进程」 |
| `controller health: {... 'guard_check_error': ...}` | `motion_health()` 内部某项读取失败（它保证不抛，改为把错误放进这个字段）。健康信息本身不可全信了，但 `guard_tripped` / `watchdog_alive` 之外的判断要靠日志 |
| `refusing commanded pose ... outside the motion guard fence` | 指令位姿本身出界，臂**没动**。几何填错或臂不在该在的位置 |
| `refusing to rotate 0.xxxrad under cartesian impedance` | 目标姿态离当前太远。**优先怀疑 `target_ee_pose` 里填的是四元数而不是欧拉角**（会让目标姿态落在单位姿态附近，而盒子和姿态围栏都会跟着重新居中，于是没有任何围栏会响） |
| `refusing to run on hardware without a motion guard` | `ee_pose_limit` 全零 —— 配置没给 `target_ee_pose` / `clip_*`。这是刻意的**拒绝构造**，不是可以忽略的 warn |
| `grasp did not capture the cube: measured width=...` | **不一定是空爪。** 判定只看 `measured width` 是否落在 `FRANKA_CUBE_WIDTH_M ± FRANKA_HOLD_TOL_M` 窗口里，跟指间有没有物理接触无关（LOG-024：明明夹住了方块，但默认窗口 `0.046±0.012`=[0.034,0.058] 不含实测的 0.0325，照样报错）。**先看错误里报出来的 `measured width` 数值**：若接近你预期的方块厚度，就是默认值/上次标定值离谱，直接用这个数重新 `export FRANKA_CUBE_WIDTH_M=<该值>` 后重启 REPL（会把整个进程带走，见下一行和 §3.4）；只有当 `measured width` 明显异常（接近 0 或接近全开）时才真的是空爪，见 `gripper is not holding anything` |
| REPL 报错后直接回到 shell 提示符、`cmd>` 消失，且日志里能看到 `Exception occurred while running ...` / 进程整体退出 | 这不是 REPL 自己的 `try/except` 失效——是**所有** Ray Worker 方法的通用行为：任何未捕获异常都会经 `WorkerGroupFuncResult` → 后台线程 `SIGUSR1` → `ray.kill` 全部 actor + `exit(-1)`，跟异常是不是 motion guard 无关（LOG-024，§3.4「这条规则不是围栏专用的」）。**处理方式是重启整个命令**（**franky 容器内** `ray stop` 若卡住，再重新 `python b/x/scripts/test_franky_controller_ext.py` / 对应烟测脚本），不要指望同一个 `cmd>` 还能继续 |
| `gripper ... did not finish in 6.0s` | 手爪无响应。已尝试 `stop()`。**不要重试**：那期间臂在阻抗下带电。检查手爪供电/连接 |
| `action_scale[0]=0.0200 ... clamping to 0.0050` | 正常信息，不是错误：上游 `action_scale` 是 20 cm/s，被夹到 5 cm/s |
| **成排的 `step slew clamped: ...`，且请求距离逐条变大** | LOG-022 的回归签名。插值期间**不该**出现这条（`_move_action` 在 `_in_interpolate` 作用域里跳过 slew clamp）。出现就说明跑的是旧代码。在 `step()` 路径上偶发一条是正常兜底 |
| `rest pose NOT reached after 3 attempts: err=...` | env 侧的 0.01 m 绝对收敛没达到。先按上一行查 clamp 洪流；确认没有的话是跟随滞后，回 2.4b 抬 `--force-ceiling`。注意烟测脚本的门更宽（z 0.025 m），所以这条 warning 后面**仍可能** `reset-only PASS` |
| `gripper: holding=True` 但 `width` 距判定窗边沿只剩几毫米 | 现在能过，但方块一滑或指垫一压就掉出窗外 → `holding=False` → `close()` 会在回合中途重抓。用实测值：`--cube-width <实测>`（或 `FRANKA_CUBE_WIDTH_M`）。窗宽是 `FRANKA_HOLD_TOL_M`（默认 ±0.012） |
| `compliance: translational ceiling not achieved ...` | 刚度太低导致 clip 被下界夹住，实际力上限不足 → 臂可能根本不动。抬 `K_t` 或抬 `--force-ceiling` |
| `collision behavior tightened: ...` | 正常信息：libfranka 硬件反射已从 100 N 降到 40 N |
| `could not tighten collision behavior ...` | **要当心**：硬件反射仍在 100 N，只剩 Python 护栏。可以继续但要更谨慎，并记进 LOG |
| **臂突然大幅移动 / 冲过指令值** | **拍急停（或 user-stop）**。然后：**每个在跑 ray 的容器各自** `ray stop`（阶段 3 是两个容器）→ **宿主机** `ss -tn ... :1337` 应为空 → Desk 清 fault → 松开 user-stop → 重新 Activate FCI → **franky 容器** `diag-probe` 确认 `Idle` + `has_errors=False` → 引导回标记上方 3–5 cm。**不要直接重跑 `reset`**，先回 2.4b 用 `--force-ceiling` 把 `peak_overshoot` 调到 ≤0.02。根因分析见 §1.5 / LOG-019 |
| `refusing to interpolate 0.xxxm under cartesian impedance` | 请求的插值位移超过 0.35 m。几何填错了，或臂被留在了不该在的地方。用引导键挪回标记附近，**不要**提高这个上限——大位移在 franky 上就不该走阻抗（`DualFrankaEnv` 用阻塞 `reset_joint`） |
| `refusing to reset from this pose (N problem(s))` | 起始位形硬门。按提示引导到标记上方 3–5 cm，`connect` 复核。`--allow-start-outside-box` 只在明确知道为什么时用 |
| `gripper is not holding anything` | 闭爪任务空爪起跑会对空气 `grasp`。用 REPL 重夹（`open` → 放入 → `close`），复现 H1 握姿 |
| `authority: ... 100.0N/axis`（或明显不是 20 N/轴） | `motion_limits` 没生效——检查是否跑的是旧代码，或 `RLINF_CUBE_FORCE_CEILING_N` 被设成了大值。**立刻停**，这正是 LOG-019 的权限 |
| **`reset` 打印 OK 但 `dz=0.0000`** | 先 **franky 容器内** `python b/x/scripts/diag_franky_motion.py --probe`。`UserStopped` → 松开手持设备 user-stop（这是 LOG-011…016 全部 `dz=0` 的唯一原因）。`Idle` 才继续查阻抗 |
| `ControlException: ... command not possible in the current mode ("User stopped")` | 同上。注意 `has_errors=False` 不代表没问题，要看 `robot_mode` |
| `cartesian impedance tracking stopped: <真因>` | v2 护栏。读真因：`"User stopped"` → 模式；`cartesian_reflex` / `joint_position_limits_violation` → 位形或碰撞，Desk 清 fault 后换起始姿态；`communication_constraints_violation` → 1 kHz 时序（容器 / RT 调度） |
| `robot_mode=Guiding` | 引导键还被按着，松开 |
| `robot_mode=Reflex` | Desk 清 fault，再 Activate FCI |
| 臂朝桌外 / 扫地 | 急停 → Desk 清 fault → **不要**直接重跑，先查 `target_ee_pose` 是否填反、单位是否米 |
| `Couldn't connect` / FCI 被抢 | 停掉所有 python；**宿主机** `ss -tn ... :1337` 应为空；REPL 必须 `q`；必要时**每个在跑 ray 的容器各自** `ray stop` |
| `NotImplementedError ... original Franka Hand` | 跑了官方 toolkit。**franky 容器内** `ray stop` 后改用 `python b/x/scripts/test_franky_controller_ext.py` |
| `libfranka gripper: Command failed`（reset 时） | 已夹住又发了一次 `grasp`。当前 `FrankaLibfrankaGripper` 会打印 `skip grasp`；若仍报错，方块可能已掉，先用 REPL 轻力 `close` |
| `close` 把方块夹扁 | 现默认 20 N（上限 40 N）。若还在死夹：敲 `open` 或 `stop`，或拍急停 |
| 掉块 | **不要**在 env 还连着时 `open`。Ctrl+C 停（阶段 3 在 **GPU 容器**的训练进程上）→ **franky 容器** REPL `open` → 捡起放入 → `close`（尽量复现 H1 握姿）→ 引导到标记上方 → `q`。标记被带跑则重做 2.5 |
| `'VideoPlayer' object has no attribute 'stop'`（`env.close()`） | 已知无害警告，烟测里被吞掉，不影响门闩 |
| `/dev/shm has only 67108864 bytes` | Ray 警告，非失败原因。`docker_run_franky_5090.sh` 现已带 `--shm-size=10g`，但**当前运行中的容器是旧参数起的**，重建后才没有这条警告 |
| `obs ... not within the observation space` / `dtype float64` | gymnasium passive checker 的警告，上游行为，忽略 |
| 脚本报 uncalibrated | 先做 2.5，**禁止**把全零当 target |
| 想「换个运动接口绕过去」 | 先读 `DualFrankaEnv._go_to_rest`：RLinf 自己的 franky env 用**阻塞 `reset_joint`** 做 reset，阻抗只用于 `step` 增量。不要凭感觉发明接口 |

---

## 7. 待办与已知缺口

| ID | 状态 | 缺口 | 说明 |
|----|------|------|------|
| T1 | **关闭** | reset 运动原语要不要换 | LOG-018：阻抗 hold + ramp 都可用，不需要换成阻塞 `CartesianMotion`；而且后者在近奇异位形 + 残余速度下会 reflex。reset 继续走阻抗路点 |
| **T2** | **已做**（LOG-027），**未在真机验证** | 阶段 3 真机训练 YAML / 启动脚本 / 相机配置 / `--with-camera` 旗标四件已按 §S3.1 创建，但从未在双容器集群或真机上跑过 | 四个文件：`b/x/configs/realworld_cube_place_sac.yaml`、`b/x/scripts/run_cube_place_sac.sh`、`b/x/configs/realworld_cube_place_camera.yaml`、`step_cube_place_robot.py --with-camera/--camera-serials/--camera-type/--camera-json`。开训前必须按文件顶部三处 `EDIT ME` 注释填入实测值（GPU 容器 `which python`、`FRANKA_CUBE_WIDTH_M`、相机 serial）。现有 `realworld_cube_place_dummy_sac*.yaml` 仍只是 dummy（`defaults:` 带 `env/realworld_cube_place`，`is_dummy` / `safe_smoke_hold` / `use_dense_reward` 从 env 包继承）。**`safe_smoke_hold` 的数据类默认值是 `False`**，只有 env 包把它钉成 `True`——新 YAML 走的正是这个 env 包，所以 `FrankaEnv.__init__` 不会自己做 `_interpolate_move(reset_pose)`；`reset()` 仍会移动到悬停。另新增宿主机一键训练前检查脚本 `b/x/scripts/preflight_cube_place_sac.sh`（LOG-031，§S3.5 第 0 步） |
| T3 | **完成** | `run_cube_place_phase2.sh` 没有 `diag` 子命令 | 已加 `diag-probe` / `diag` / `diag-replay`，且所有子命令都会把多余参数透传给 python |
| T4 | **完成** | franky 容器 shm 只有 64 MB | `docker_run_franky_5090.sh` 已加 `--shm-size`（默认 10g，`RLINF_FRANKA_SHM_SIZE` 可改）+ `set -euo pipefail` + REPO 存在性检查。**当前正在跑的容器仍是 64 MB**，下次重建才生效 |
| ~~T5~~ | **关闭**（LOG-034） | 阻抗在 x≈0.70 伸展位形下的实际权限 | 2.4b 已在真实位形（起点接触点上方 2.1 cm、夹着方块）跑完六档：`peak_overshoot` 全 **0.0000**、最差 `peak_lag` 9.2 mm、最差 `peak\|dq\|` 0.112 rad/s、`peak\|F_ext\|` 爬到 10.7 N 仍在 20 N/轴内。**权限既不过大也不不足**；实测表见 §2.4b，当基线对照表用 |
| T6 | 未做（**工具已备**，LOG-027） | `wrist_1` 相机在 cube-place 链路上未验证 | 烟测全程 `RLINF_SKIP_CAMERA=1`，而 skip 时拿到的是**全零 stub 帧**（不是缺键、不报错），所以「训练在跑、策略看黑图」不会自己暴露。`step_cube_place_robot.py` 现在有 `--with-camera` 旗标（reset 后打印每路 `frames` 的 shape/dtype/min/max/mean，全零帧会标 `ALL-ZERO`），验收命令与通过标准见 **§S3.3**，它是阶段 3 的第一步 |
| T7 | **完成** | 诊断脚本测不到超调峰值 | `diag_franky_motion.py` 已重写：20 ms 连续采样、ramp 后 settle 观测窗、`peak_overshoot` / `peak_lag` / `peak\|F_ext\|` / `peak\|dq\|` 全部记录，四道 abort 门 |
| **T8** | 未做 | 路点间 `recover_from_errors()` 是否有害 | LOG-013 的假说，LOG-014 的「证伪」因在 `UserStopped` 下进行而**无效**。已加 `clear_error_per_waypoint` 旋钮（默认 True = 上游）与 `diag --test-recover-loop` 对照测试。**用测量结论，不要凭感觉改默认值** |
| T9 | **完成** | 阶段 1B 在本轮改动后未重跑 | 已重跑两次（改 env/控制器之后、改 `action_scale` 之后），均 exit 0 且 `sac/*` 指标齐全 |
| T10 | 未做 | 零空间刚度 5 Nm/rad 是否过软 | 事故里 z+36 cm 伴随 x−6 cm 的圆弧说明肘部几乎自由。现已启用 franky 的**软关节限位斥力**（上游从不传那两个限位向量，所以该功能一直关着），但零空间刚度本身未动：`RLINF_CART_K_NS` 可调，**先测再调**（2.4b 的 `peak\|dq\|` 会给线索） |
| **T11** | 未做，**假说空间已缩小**（LOG-034） | **26 cm 超调的机理仍未确定** | 「全程力饱和」早已被证伪：滞后由摩擦主导（实测 9.4 N 里约 8 N 与速度无关），10 cm/s 时滞后仅约 11.7 mm，远未吃满 50 mm 的 clip。所以 100 N 是**可用权限**而非**实际输出**，护栏是防御纵深而非根因。**现在又排除一条**：`--test-waypoints`（10 Hz 零阶保持阶梯）在真实位形上跑出 `overshoot=0.0000`、`peak\|dq\|=0.102`，所以「10 Hz ZOH + `K_t=2000` + 软零空间在这个位形下本身就不稳」**在纯平移、单调、慢速条件下不成立**。剩下的候选都在 2.4b 覆盖不到的地方：旋转（T16）、方向反转、横向 |
| **T12** | 未做，**平移侧已有基线** | `step()` 速度上限 5 cm/s 是推算值 | 由「指令力上限下终端速度约 13 cm/s」反推的保守值，非实测。LOG-034 给了 1–3 cm/s 的滞后基线（4.3→5.7 mm，增长很慢），但 5 cm/s 本身仍未直接测过。阶段 3 应据实测重定，并注意它同时改变了 RL 动作空间的语义（`action_scale[0]` 0.02 → 0.005） |
| **T13** | 未做 | 精简审计的低严重度发现丢失 | 合并 agent 结果在通知里被截断（43 KB 输出文件只是进度 JSON，journal 的 value 全为 `None`）。四条最高严重度已全文取得并落地；余下需重跑该 workflow 才能恢复 |
| **T15** | **部分验证**（LOG-034，真机首跳闸） | **围栏跳闸后的收尾路径** | 五环里**前四环真机确认**：`brake (lag, freeze-then-stop) stop=clean` → `WATCHDOG trip [lag]`（只锁存不抛）→ env 侧 `_raise_if_guard_tripped` 抛出 `RuntimeError` → `refusing to command motion`。**第五环没出现**：`finally` 停 tracker 与 teardown health 打印都没跑到，因为训练路径的 driver（`Cluster.signal_handler`）收到 SIGUSR1 后 `list_actors` + 逐个 `ray.kill`，把控制器 actor 直接杀了；而且它遍历的是过期列表，**自己抛了** `ValueError: Failed to look up actor 'FrankyControllerExtended-0-0:0'`。本次无安全后果（tracker 已在 `_abort_motion` 拆掉、臂已刹住），但**取证仍缺**。教训：在烟测脚本（driver 有自己的 `try/finally`）里验证过的收尾链，**推不出训练路径也成立** |
| **T14** | 未做 | 阻尼项不受任何软件上限约束 | franky 的阻尼由刚度推导（`K_t=2000` 时约 155 N·s/m）且**不被裁剪**。因 `desired_twist≡0` 它只阻碍运动、不会驱动超程，但人手拦臂时的峰值力不受这些数字约束。真要降只能降 `K_t`——而 `K_t=2000` 来自 PegInsertion 的 `compliance_param`，改它要先有数据 |
| **T16** | **真机已验：有效但不充分（LOG-036）** | **`step()` 路径的旋转授权没有任何上限** | 代码修复见 LOG-035（`max_action_scale_rot()` + `CubePlaceConfig.__post_init__` + `_clamp_step_slew` 同时 clamp 姿态）。**LOG-036 真机重训证实限幅确实逐周期生效**（日志反复出现 `step slew clamped: rot ...` / `xyz ...`，两个分量都被钳在预算内），跳闸推后到第 ~205 秒（LOG-034 是第 10 秒）、`\|dq\|` 峰值降到 2.03rad/s（LOG-034 是 2.69rad/s）——**但仍然跳闸**。根因下修：跳闸时 `q5≈0.085rad`（Panda 腕部近奇异位形），笛卡尔空间限幅无法保证雅可比病态区间的关节速度有界——**限的是错误的空间**。见新增 **T19**（关节速度需要独立上限）与 **T20**（基线要覆盖事故位形，不能只测悬停位形） |
| **T17** | **代码有二次 bug，已修，未再验证（LOG-036）** | **YAML `env_configs.env_vars` 到不了控制器 actor** | LOG-035 的修复本身有两层错误：① `kwargs.get("worker_info")`——`worker_info` 是 `FrankaEnv.__init__` 的**位置参数**，从不按关键字传，恒为 `None`；② 即使拿到也是 `WorkerInfo`（无 `node_group_label` 字段），跟真正带这个字段的 `Placement` 是两个类型，`getattr(..., None)` 静默退回旧行为，**没有任何报错**——这正是教训 43 想防的洞，这次是在补洞代码里自己又开了一个同类洞（教训 46）。LOG-036 真机重训证实：`raw_env FRANKA_CUBE_WIDTH_M=0.046` 仍是错的。**已改用真值源**：`FrankySingleFrankaEnvMixin.__init__` 现在直接读 `os.environ.get("NODE_GROUP_LABEL")`——这正是 `Worker._init_node_group`（`worker.py:1500`）自己读取 `self._node_group` 的同一个环境变量，`FrankaEnv` 就是在这个 env worker 进程内部构造的。**鉴于 T16/T19 的跳闸问题未解决，本次未再开训验证这一行日志** |
| **T18** | 新增（LOG-035），**优先级下降（LOG-036）** | **`STEP_ROT_SPEED_RAD_S_DEFAULT=0.3rad/s` 是推算值，不是实测值** | LOG-036 的旋转阶梯已把悬停位形测满到 `0.15rad/s`（四档全过，`peak\|dq\|` 最差 0.379rad/s），但**悬停位形本身不是事故位形**（`q5≈0.18` vs 事故的 `q5≈0.085`），所以这个实测结果现在只能说明「远离奇异点时这个上限是安全的」，不能说明「训练路径上也安全」。在 T19/T20 有结论之前，继续加大这个数字**没有意义**——真正的瓶颈在关节空间，不在这个笛卡尔速度值本身 |
| **T19** | **关闭（LOG-039 真机验证通过）** | **关节速度没有独立于笛卡尔限幅的软件上限** | **修法已落地**：命令侧 `joint_demand_scale` + `_clamp_joint_demand`（10Hz，降速不跳闸）、看门狗 `dq` 判据（50Hz，排在 `lag` 前）、以及插值路径的时长拉长 `_stretch_interp_for_joint_demand`（实现时才量出来的漏洞：事故位形上连 2cm/s 插值速度都超预算 1.73 倍，而那条路径故意跳过位置限幅）。默认值、env 旋钮、实测锚与验收脚本见 §S3.11。**LOG-039 从事故位形做了 `reset` + `box` 活体验证，两轮全过、未跳闸**：三段插值被拉长（均为 j4），`step()` 路径打出 `rot 0.0734→0.0520rad（笛卡尔限幅）; joint demand j4 21.20x budget -> step x0.047（关节限幅）`——两道界叠加、把一个通过了全部笛卡尔检查的危险指令当场缩掉。**下面这一整段是机理，保留备查** |
| T19 机理 | 实测记录（LOG-037），**保留备查** | 同上 | `_clamp_step_slew`（T16）把末端位移钳在 5cm/s、转角钳在 0.3rad/s，这两个数字本身完全按设计工作，但事故量是**关节速度**，两者的比例由雅可比条件数决定，与位形有关。**LOG-037 用 `diag_franky_jacobian.py` 量到了硬数据**：事故位形 `sigma_min(J)=0.0062`、可操作度 0.0061；悬停位形 `sigma_min=0.1115`、可操作度 0.0883（**相差 18 倍 / 14.6 倍**）。同一个合规指令（每轴 5cm/s + 0.3rad/s）在事故位形要求 **`\|dq\|=11.77rad/s`**（逐关节最大 9.64，是 Panda 自身 2.075rad/s 关节限速的 4.6 倍），在悬停位形只要求 **0.81rad/s**。实测 2.03rad/s 只是这个需求的一小部分——因为它是力矩控制、跟不上就变成滞后，于是 `lag` 长到 0.0512m 触发围栏。**两条已被实测排除的假说**：① 不是「`q5≈0` 腕部奇异」（Panda 腕部奇异在 `q6≈0`，实测 `q6=1.4967`；且平移块 `cond=4.28`、旋转块 `cond=1.41` 单独看都很好，退化发生在**平移与旋转耦合**的那一个混合方向上，几何上是**接近满臂展**，`q4` 距上限仅 0.3535rad）；② 不是关节限位斥力、也不是零空间自运动（实测无任何关节进入 0.1rad 斥力带）。**故 `target_ee_pose` 的 `roll≈−π` 与本问题无关，原「换姿态」候选修法作废。** 据此设计的修法见 §S3.11 |
| **T20** | 新增（LOG-036），**前置条件已就绪（LOG-038）** | **现有基线（2.4b + 旋转阶梯）全部测在悬停位形，覆盖不到事故位形** | 2.4b（LOG-034）与旋转阶梯（LOG-036）都跑在悬停附近，干净得不像话，但训练实际会把臂带到事故位形一带。**基线证明了"别处没问题"，不能反推"这里也没问题"**（教训 48）。原先「设计这个实验本身就是在重复触发同一个风险」的顾虑，**已被 T19 的①②解除**——现在坏位形上的指令会被自动降速而不是跳闸，所以可以安全地在事故位形附近做复现实验了。目前 `JOINT_VEL_DEMAND_FRACTION=0.5` 与 `GUARD_MAX_DQ=1.2` 各有实测锚，但锚点只有两个位形；下一步用 `diag_franky_jacobian.py` 扫安全盒内 `sigma_min` 的分布来复核 |
| **T21**（LOG-040 发现，LOG-041 落地） | **已实现，静态与桩件验收通过；真机未验** | **跳闸后 `RuntimeError` 会杀死整条 Ray 训练任务（`ActorGroup`+`RolloutGroup`），而不是被 env 一层捕获后 reset 继续** | 第三次开训第 2 个 rollout epoch 中 `[watchdog:dq]` 按设计清洁刹停（`braked=True stop=clean`，机器人 `Idle has_errors=False`），但异常顺着 `_move_action → env.step` 一路穿透，Ray 判定 actor 异常退出，`ray.kill` 掉整条训练进程。**修法（§S3.12）**：控制器新增显式的 `recover_from_guard_trip()`（先确认臂已停稳、再清 Desk 故障、最后才解闩锁，围栏保持武装）；env 侧新增 `MotionGuardTripped` 专用异常，`step()` 把跳闸变成 truncated episode（reward 0，不编造惩罚），`reset()` 最多重试 2 次。**有预算上限**（默认 10 次/env，`RLINF_CUBE_GUARD_RECOVERY_BUDGET=0` 可退回旧行为），用完就让异常终止训练——无上限的自动恢复就是"安全机制自己退场" |
| **T22**（LOG-040 发现，LOG-041 落地） | **已实现，静态与实测雅可比验收通过；真机未验** | **肘部关节位形在纯阻抗控制下会悄悄漂移到病态，且没有任何自动检测** | 末端位姿完全正常（安全盒内部，不靠边）时，关节解仍能病态到 `sigma_min=0.0009`（比 LOG-037 事故位形还差 7 倍）——纯笛卡尔阻抗跟踪对关节冗余方向不加约束，多次跳闸+复位积累下来会往坏方向漂。本次靠人手引导恢复（回到 `sigma_min=0.11` 附近，对照 `hover_log036`）。**修法（§S3.12）**：`reset()` 开头与每次跳闸恢复后各做一次 `sigma_min` 自检（复用 `motion_limits.jacobian_conditioning`，一次 6×7 SVD，用控制器随状态返回的雅可比，不动臂、不加往返），低于 `sigma_min_warn()=0.04` 就打星号横幅、给出要瞄准的良态关节角。**只预警不拒绝**：那一刻的臂并不危险，能修的是操作员不是软件。`joint_reset` 那条路仍然没启用（`joint_reset_cycle=20000` 事实上永不触发，默认目标 `[0,0,0,-1.9,0,2,0]` 继承自 charger、从未在这套工装上验证过安全性） |

风险与缓解（沿用 v1 §10，加 LOG-019/020/021）：未标定就开训 → 2.5 门闩 + `calibrated` 检查（`write_cube_place_pose.py` 现在会**拒写**不像真机读数的位姿并留 `.bak`）；空爪/张爪后读 pose → 2.5 语义 + `connect` 的 `holding` 检查（按**测量宽度**判定，不看 `is_grasped`）；抄 BinRelocation 张爪 reset → Env 走 PegInsertion 闭爪抬升；误设 `no_gripper: False` → env yaml 已钉 `True` + 1A-2 检查 6D；砸桌 → `clip_z_range_low` 小 + 围栏 −z 余量仅 1 cm + 标定只贴住不死压；**超程 → 40 N 硬件反射 + 20 N/轴软件上限 + 2 cm/s（reset）与 5 cm/s（step）限速 + 控制器进程内围栏 + 50 Hz 看门狗 + 按失效类型分向的刹车；起始位形不对 → Ray 启动前硬门（含姿态与夹爪）；配置退化 → 盒子不可用时真机上拒绝构造；改了 `b/x/` 忘了验 → 1A 现在断言力上限（逐轴与范数）、两条限速、`插值速度 ≤ 步进速度`、位移与转角上限、起始位形门、有效盒子与打印一致、`is_dummy`/`robot_ip` 一致、配置字段不漂移、**以及围栏锁存后「拒绝下发而不是抛异常」的契约**（共 18 组）；围栏跳闸把整个 driver 打掉、收尾跑不完 → 控制器里的围栏路径一律不抛，只刹车 + 锁存，由 env 侧轮询转成普通异常**。

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
| **阶段 3 的 driver**（异步入口） | [`examples/embodiment/train_async.py`](../../../examples/embodiment/train_async.py)、[`examples/embodiment/run_realworld_async.sh`](../../../examples/embodiment/run_realworld_async.sh) |
| **异步 embodied 训练循环 / 指标命名空间** | [`rlinf/runners/async_embodied_runner.py`](../../../rlinf/runners/async_embodied_runner.py)、[`rlinf/runners/embodied_runner.py`](../../../rlinf/runners/embodied_runner.py)（`resume_dir` 不支持 `auto`） |
| **SAC actor / replay buffer / checkpoint** | [`rlinf/workers/actor/fsdp_sac_policy_worker.py`](../../../rlinf/workers/actor/fsdp_sac_policy_worker.py) |
| **双节点 `node_groups` / `env_configs` / `python_interpreter_path` 的 schema 与示例** | [`rlinf/scheduler/cluster/config.py`](../../../rlinf/scheduler/cluster/config.py)、环境变量转发在 [`rlinf/scheduler/cluster/node.py`](../../../rlinf/scheduler/cluster/node.py) |
| **`hardware: type: Franka` 的校验内容** | [`rlinf/scheduler/hardware/robots/franka.py`](../../../rlinf/scheduler/hardware/robots/franka.py)（只 ping + 相机 SDK，不碰 1337） |
| franky tracker 实现（容器内） | `/opt/venv/franky-0.19.0/lib/python3.11/site-packages/franky/tracker.py` |
| 本机扩展 | [`b/x/franky_ext/`](../../../b/x/franky_ext/)、[`b/x/scripts/`](../../../b/x/scripts/) |
| 集群 / 训练前检查脚本（宿主机） | [`b/x/scripts/verify_ray_cluster.sh`](../../../b/x/scripts/verify_ray_cluster.sh)（§S3.2 验收，LOG-029）、[`b/x/scripts/preflight_cube_place_sac.sh`](../../../b/x/scripts/preflight_cube_place_sac.sh)（§S3.5 第 0 步，LOG-031） |
| **旋转授权阶梯诊断（franky 容器，LOG-035，T16/T18）** | [`b/x/scripts/diag_franky_motion.py`](../../../b/x/scripts/diag_franky_motion.py)（`--test-rotation` / `--test-rotation-waypoints`）、[`b/x/scripts/run_2_4b_rotation_ladder.sh`](../../../b/x/scripts/run_2_4b_rotation_ladder.sh)（全自动分档、失败即停） |
| **雅可比条件数诊断（franky 容器，LOG-037，T19/T20）** | [`b/x/scripts/diag_franky_jacobian.py`](../../../b/x/scripts/diag_franky_jacobian.py)——**只读、不动臂**，可评估任意 `q`（`franky.Model.zero_jacobian` 有接受 `q` 的重载），所以不必把臂开过去就能量某个位形的条件数与关节速度需求 |
| **关节需求限幅验收（franky 容器，LOG-038，T19）** | [`b/x/scripts/test_t19_joint_demand.py`](../../../b/x/scripts/test_t19_joint_demand.py)——不带参数跑数学与接线检查（无硬件）；`--robot` 追加两个实测位形上的真雅可比检查（只读连接，不下发运动）。见 §S3.11 |
| **跳闸恢复与肘部自检验收（franky 容器，LOG-041，T21/T22）** | [`b/x/scripts/test_t21_t22_recovery.py`](../../../b/x/scripts/test_t21_t22_recovery.py)——不带参数跑 40 项数学、桩件行为与接线检查（无硬件）；`--robot` 追加四个已记录位形上的真雅可比复核（只读，不动臂）。见 §S3.12 |
| 标定与急停（人） | [`charger_sac_async.md`](charger_sac_async.md) §6.2.1、§15.5 |

---

**下一个动作**（LOG-026 之后：阶段 2.6 / 2.7 / 2.8 都已 PASS，臂应仍在 `0.7194, 0.0250, 0.3134` 附近悬停、夹着方块）：**阶段 3 的第一步是相机验收（§S3.3）**，因为它不动臂、不占 1337、失败代价最小，而且它挡住的是「训练跑起来但策略看黑图」这种只会表现为「学不动」的静默故障。

```bash
# franky 容器内（单节点即可，验完 ray stop；已 source setup_before_ray_5090.sh）
source b/x/configs/setup_before_ray_5090.sh
python b/x/scripts/step8_detect_cameras.py --write-yaml
export RLINF_SKIP_CAMERA=0 FRANKA_ROBOT_IP=172.16.0.2
ray start --head --port=6379 --disable-usage-stats
python b/x/scripts/step8_test_env_camera.py --save-jpeg --require-live
ray stop
```

**期望：** serial 非占位；`obs['frames']['wrist_1']` 是 uint8 128×128×3 且 `max>0`；零动作步之间帧有变化。

**上面这一段是阶段 3 的历史入口，现在已经全部走过了**：§S3.1 四个文件已建（LOG-027）、集群已起并过 14 项验收（LOG-029）、相机验收已过、preflight 已具自动修复能力并真机全 PASS（LOG-031/032/033）、**训练 YAML 已真机首跑**（LOG-034）、**2.4b 六档已补做完成且全过**（LOG-034，T5 关闭）、**T16/T17 的第一版代码修复已落地并过 1A（LOG-035）**、**旋转阶梯与第二次真机重训已完成（LOG-036）**、**根因已用只读雅可比诊断量清（LOG-037）**、**T19 的两道界已实现并通过静态验收（LOG-038，§S3.11）**、**从事故位形做的 `reset` + `box` 活体验证已通过（LOG-039）**。

**LOG-036 的结论：T16 不充分，问题根因下修为关节空间，训练暂停。** 旋转阶梯四档在悬停位形全过，重启训练后 T16 的限幅确认逐周期生效（`step slew clamped: rot/xyz ...`），跳闸推后到第 ~205 秒（LOG-034 是第 10 秒）、`|dq|` 峰值降到 2.03rad/s（原 2.69rad/s），**但仍然跳闸**——跳闸时 `q5≈0.085rad`，是 Panda 腕部近奇异位形。笛卡尔空间限幅（T16 的整个思路）无法保证雅可比病态区间的关节速度有界，即**卡住的是错误的空间**。同时发现 LOG-035 对 T17 的修复代码本身有 bug（`worker_info` 既不是关键字参数也不带 `node_group_label` 字段），已改用 `os.environ.get("NODE_GROUP_LABEL")` 这个真值源，但尚未再开训验证。

**LOG-038 的结论：T19 的①②已实现，静态与实测雅可比验证全过；另外补掉了一个实现过程中量出来的真实漏洞。** 命令侧关节需求限幅（`joint_demand_scale` + `_clamp_joint_demand`）与看门狗 `|dq|` 判据都已落地并有测试锚（见下方 §S3.11）。补掉的漏洞是：**插值路径（reset 走的那条）原本完全不设防**——`_move_action` 在插值期间故意跳过 slew 限幅（LOG-020 的正当理由：状态陈旧会把目标钉死），而实测在事故位形上**连 2cm/s 的插值速度本身就超预算 1.73 倍**，三轴平移+旋转组合是 4.21 倍。修法换了机制而不是换阈值：位置限幅不行，**时长限幅行**（关节需求与时长成反比），所以 `_stretch_interp_for_joint_demand` 在 `interp_duration_s` 之后再把时长乘 `1/scale`，臂仍然到得了目标、只是走得慢。

**LOG-039 的结论：活体验证通过，第三次开训的阻塞项已清。** 从事故位形起跑 `reset` → `box`，两轮共动臂约 40 秒，**全程未跳闸**。三段插值被拉长（均为 j4），`box` 的 `step()` 路径打出了这一行——**两道界叠加的完整证据**：

```
step slew clamped: rot 0.0734rad -> 0.0520rad (0.30rad/s cap);
                   joint demand j4 21.20x budget (0.50 of 2.075rad/s) -> step x0.047
```

T16 的笛卡尔限幅先把姿态增量钳到合规，然后关节限幅发现这个**已经合规**的步子仍要求关节 4 的 21.2 倍预算，缩到 4.7%。这正是 LOG-036 的失效形态被当场抓住；旧代码会把它发下去。注意 21.2 倍**高于**设计时按"三轴同时饱和"算出的 9.29 倍——真实指令还要同时修正姿态残差，**合成的最坏情况不一定是真实的最坏情况**。

> ⚠️ **两个环境陷阱，都在 LOG-039 里踩过一次**（与限幅无关，但会让你以为是限幅的问题）：
> 1. **同一主机上两个 Ray 节点，工作进程池按 `nproc` 各算一份**——两个 `--network host` 容器 × 64 CPU = 123 个 `ray::IDLE`，空转吃掉 66 GB / 93 GB，控制器 actor 一起来就被 OOM killer 杀掉，而报错指向那个 actor 而不是真正的原因。**两边 `ray start` 都要加 `--num-cpus=16 --object-store-memory=4000000000`**（加了之后空转 17 GB，验收 14 项仍全 OK）。
> 2. **§2.x 的阶段 2 脚本与 §S3.2 的两节点集群目前互斥**：`step_cube_place_robot.py:119` 把 `FrankaConfig(node_rank=0)` 写死，靠"自己起的单节点 Ray 里 rank 0 就是 franky 容器"这个隐含前提工作。两节点集群里 franky 是 **rank 1**，控制器会落到 GPU 节点并报 `ModuleNotFoundError: No module named 'franky'`（那个进程占的 19.63 GB 是 CUDA torch，不是控制器）。**跑阶段 2 前先 `ray stop`**；根治要加一个 `--controller-node-rank`。

**当前状态：第三次开训已跑通训练循环（reward 有输出、无回归性跳闸）；那次按设计的 `[watchdog:dq]` 跳闸暴露的 T21/T22 已按 §S3.12 实现，40 项无硬件验收 + 实测雅可比复核全通过（LOG-041），但两者都还没在真机训练里被真正触发验证过。集群本身仍存活，无需重起。**

**下一个动作（按依赖排序）：**

1. **重开训练，让 T21/T22 在真机上被触发一次**（这是唯一还没做的验证）。要看到的两条新日志：reset 前的 `joint configuration OK before reset: sigma_min=...`；以及万一再跳闸时的 `guard trip recovered (1 of 10 used)` 之后训练**继续**而不是整条任务死掉。跳闸后仍然值得人工跑一次 `diag_franky_jacobian.py --live`——自动恢复解决的是"训练别死"，不是"位形别漂"。
2. **T20 的基线实验**：用 `b/x/scripts/diag_franky_jacobian.py`（只读、不动臂）扫出安全盒范围内 `sigma_min` 的分布，据数据回头复核 `JOINT_VEL_DEMAND_FRACTION_DEFAULT=0.5` 与 `GUARD_MAX_DQ_RAD_S_DEFAULT=1.2` 这两个数（现在它们各有实测锚，但锚点只有两个位形）。**注意 franky 没有暴露 IK**，所以「配置期检查盒子八个角」这条路走不通（无法从笛卡尔角点反解 `q`），只能靠在线检查，或用已经安全的插值路径慢慢走到角点、逐点实测记录。
3. **复核安全盒本身**：事故点 `x=0.7788`，而盒子 x 上界是 `0.726857+0.05=0.7769`——臂当时已经在盒外 1.9mm（围栏还有 0.05m 余量所以没拦）。也就是说**盒子的 +x 边缘本身就落在 `sigma_min≈0.006` 的坏区里**。第 1 条做完之后训练会在这里一直被限幅减速（实测缩到 10.8%，即 0.54mm/周期），**能跑但很慢**，所以应考虑把 `clip_x_range` 收小、或把 `target_ee_pose` 整体往内移——这是「别把工作区定在坏条件区」，与限幅是两件互补的事，**而且现在它从安全问题降级成了效率问题**。LOG-039 已经看到这个代价：从事故位形做 `reset` 会留下约 20mm 的残差（`rest pose NOT reached`），因为那里的阻抗弹簧驱不动一个被缩到 4.7% 的步子。
4. 开训后头五分钟除 §S3.5 那张表外，**多盯两条**：`step slew clamped: joint demand ...` 的频率——偶发说明限幅在边缘工作，持续刷屏说明策略在坏条件区打转，那就回第 3 条收工作区而不是放宽限幅；以及 `guard trip recovered (N of 10 used)` 的计数——**预算被稳定消耗本身就是信号**，说明该收工作区了，而不是把 `RLINF_CUBE_GUARD_RECOVERY_BUDGET` 调大。

#### S3.11 T19 的两道界：改了什么、怎么验、旋钮在哪

| | ① 命令侧关节需求限幅 | ② 看门狗 `\|dq\|` 判据 |
|---|---|---|
| 限的量 | 关节速度**需求** `dq_req = J⁺·(Δpose/Δt)` | 关节速度**实测** `\|dq\|` |
| 频率 / 位置 | 10Hz，`_clamp_step_slew` 末尾（`_clamp_joint_demand`）+ `_interpolate_move` 时长（`_stretch_interp_for_joint_demand`） | 50Hz，`_evaluate_guard`（排在 `lag` **之前**） |
| 行为 | **降速**，回合继续 | 刹车 + 锁存 + 抛异常 |
| 默认值 | `JOINT_VEL_DEMAND_FRACTION_DEFAULT = 0.5`（占 `JOINT_VEL_LIMITS` 的比例） | `GUARD_MAX_DQ_RAD_S_DEFAULT = 1.2` rad/s |
| env 覆盖 | `RLINF_CUBE_DQ_DEMAND_FRAC` | `RLINF_CUBE_GUARD_MAX_DQ` |
| 实测锚 | 悬停位形满速需求 = 限速的 0.26（**不受影响**）；事故位形 9.29 倍预算（缩到 **10.8%**） | 已知正常：旋转阶梯最差 0.379、悬停满速需求 0.81；已知事故：2.03 / 2.69；Panda j1–j4 限速 2.075 |

**为什么要两道而不是一道**：① 限的是**需求**，因为臂是力矩控制的——一条不可能的指令不会表现为关节转得快，而是表现为位姿误差涨到 `lag` 门限（这正是 LOG-036 的形状）。等 `|dq|` 测得出来时指令早已被接受，所以只有②是不够的。反过来，① 只在**下令时的雅可比**上评估，路径中途条件数会变，所以只有①也不够。

**`dq` 必须排在 `lag` 之前判**：关节失控是**间接**表现为 lag 的（跟不上→误差涨→lag 触发），**两次真实跳闸都因此被归错档**，LOG-036 还为一个关节空间问题跑了一整条笛卡尔旋转阶梯。现在跳闸消息会直接说 `[watchdog:dq] joint runaway: |dq|=... worst j4=...`，并带上 `q`。刹车顺序沿用 `lag` 的 `freeze-then-stop`（先撤掉追不上的目标），这一点有测试断言看着。

**验收脚本**：`b/x/scripts/test_t19_joint_demand.py`（franky 容器内）。不带参数只跑数学与接线检查（无硬件）；`--robot` 追加在 LOG-037 两个实测位形上的真雅可比检查（**只读连接，不下发任何运动**）——这一半才能抓住「雅可比取错帧」或「矩阵转置了」这类合成测试永远发现不了的错。同样的断言也进了常设闸门 `step_cube_place_dummy.py`（1A）。

```bash
# franky 容器内，已 source b/x/configs/setup_before_ray_5090.sh
python b/x/scripts/test_t19_joint_demand.py            # 无硬件
python b/x/scripts/test_t19_joint_demand.py --robot    # 只读连机器人，不动臂
```

#### S3.12 T21/T22：跳闸不再终止训练，肘部位形有了自检

LOG-040 的第三次开训里，一次 `[watchdog:dq]` 同时暴露了两件事：**安全层做对了**（清洁刹停，机器人 `Idle`、无故障），**但训练层做错了**（异常穿透 Ray actor，`ActorGroup`+`RolloutGroup` 全被 `ray.kill`）；而臂之所以会跑到那种位形，是因为**没有任何东西在看关节配置**。这两条各自的修法如下。

| | T21 跳闸后继续训练 | T22 肘部位形自检 |
|---|---|---|
| 回答的问题 | 跳闸之后，**训练**该不该继续（机器人安不安全是另一个问题，答案一直是"安全"） | 末端位姿合法时，**关节**配置是不是也还好 |
| 触发点 | `step()` / `reset()` 捕获 `MotionGuardTripped` | `reset()` 开头；以及每次跳闸恢复之后 |
| 代价 | 一次控制器 RPC（仅在跳闸时） | 一次 6×7 SVD，用状态里已有的雅可比，**不动臂、不加往返** |
| 行为 | 恢复成功 → 本回合按 `truncated` 结束，训练继续；失败或预算用尽 → 异常照旧终止训练 | **只预警，从不拒绝**：打星号横幅 + 要瞄准的良态关节角 |
| 默认值 | `GUARD_RECOVERY_BUDGET_DEFAULT = 10` 次/env | `SIGMA_MIN_WARN_DEFAULT = 0.04` |
| env 覆盖 | `RLINF_CUBE_GUARD_RECOVERY_BUDGET`（**设 0 即退回 LOG-040 之前的行为**：任何跳闸都终止训练） | `RLINF_CUBE_SIGMA_MIN_WARN` |
| 实测锚 | LOG-040 的那次跳闸：`\|dq\|=1.2942 > 1.2000`，`braked=True stop=clean`，事后 `Idle has_errors=False \|dq\|=0.0016` | 良态 0.1115（悬停）/ 0.1141（手引导恢复后）；坏：0.0062（LOG-036 事故位形）/ 0.0009（LOG-040 漂移位形，`reset` 都做不了） |

**T21 的三个设计约束，每一个都是"不这样做会更糟"：**

1. **恢复必须是一个显式命名的方法**（`FrankyControllerExtended.recover_from_guard_trip`），不能做成 `set_motion_guard` 的副作用——尽管后者本来就会顺手清掉闩锁。凡是能解除安全闩锁的代码都应该是 grep 得出来的。
2. **顺序不能反**：先确认臂真的停稳（`|dq| <= 0.02`，因为 `_abort_motion` 是先刹车后锁存，臂还在动就意味着刹车没生效），再 `recover_from_errors()` 清故障，**最后**才解闩锁。反过来就是在失控之上重新授权运动。围栏全程保持武装，tracker 与看门狗在下一条运动指令时由 `_ensure_cart_tracking_motion` 一起重建——所以不存在"能动但没人看着"的窗口。
3. **必须有预算**。"恢复了接着跑"的失败模式比崩溃更坏：一条每几步就跳闸的臂会被反复怼在同一个坏位形上，围栏尽职地一次次刹车，日志一路滚过去。用完 10 次就让异常终止训练，并在日志里指明这通常是**工作区**问题（回 §7 T19/T20，考虑收 `clip_x_range`），而不是把预算调大。
4. **捕获的必须是 `MotionGuardTripped` 而不是 `RuntimeError`**。几行之外的 `_interpolate_move` / `_stretch_interp_for_joint_demand` refusal 也是 `RuntimeError`，但它们**必须保持致命**——那是"臂在一个笛卡尔阻抗开不出去的位形里"，重试只会再被拒一次。有测试专门断言这两个 `except` 子句里没有裸 `RuntimeError`。

**reward 为什么给 0 而不是给惩罚**：跳闸时那一步**没有按指令执行完**（臂在半路被刹住了），拿它停下的位姿去打分等于给一个没做完的动作记账。同样地也**不编造惩罚**——在错误路径里改奖励函数，是奖励函数开始名不副实的经典方式。跳闸在 `info["motion_guard_trip"]` 里，要惩罚也该是显式的设计决定。用 `truncated` 而非 `terminated`，因为回合是被装置打断的、不是被任务完成的，SAC 的 bootstrap 也据此才对。

**T22 为什么只预警**：那一刻的臂没有在做任何危险的事——它可能就停在安全盒正中间。能修的人是操作员（用使动装置把肘部引回去），不是软件；`joint_reset` 那条自动路仍然不能用（默认关节目标继承自 charger 任务，从未在这套工装上验证过安全性，见 §7 T22）。警告会直接给出要瞄准的关节角与验证命令：

```
************ ILL-CONDITIONED JOINT CONFIGURATION before reset ************
sigma_min=0.0009 < 0.0400 (cond=2115.7, manipulability=0.000001)
q=[0.0726, 1.0085, -0.0485, -0.4606, 0.0656, 1.353, 0.9832]
The TCP pose may be perfectly legal -- this is about the ELBOW, ...
MANUAL ACTION: guide the arm back with the enabling device, aiming the elbow at
the known-good q=[0.23, 0.55, -0.29, -1.30, 0.18, 1.80, 1.05] (sigma_min~0.11);
joints 2 and 4 dominate. Verify with 'diag_franky_jacobian.py --live'.
```

`sigma_min` 的计算只有一份（`motion_limits.jacobian_conditioning`），在线自检和离线诊断脚本共用——操作员照着警告去跑 `diag_franky_jacobian.py --live` 时，读到的必须是同一个数。诊断脚本现在也会直接打印 env 会给出的判定（`VERDICT: ok / ILL-CONDITIONED`）。

**验收脚本**：`b/x/scripts/test_t21_t22_recovery.py`。不带参数跑 40 项数学、桩件行为与接线检查（无硬件，含"跳闸变 truncated""非跳闸异常仍然穿透""预算 0 等价于旧行为""恢复顺序不能反"）；`--robot` 追加四个已记录位形上的真雅可比复核（**只读，不动臂**）。常设闸门同样进了 `step_cube_place_dummy.py`（1A）。

```bash
# franky 容器内，已 source b/x/configs/setup_before_ray_5090.sh
python b/x/scripts/test_t21_t22_recovery.py            # 无硬件，40 项
python b/x/scripts/test_t21_t22_recovery.py --robot    # 只读连机器人，不动臂
python b/x/scripts/diag_franky_jacobian.py --live      # 手引导后核实位形
```
