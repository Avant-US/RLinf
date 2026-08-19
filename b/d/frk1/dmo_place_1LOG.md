# cube place 落地日志

方案：[`dmo_place_1.md`](dmo_place_1.md)  
代码只改 `b/x/`，不改 `rlinf/`。按时间顺序记录操作、命令、错误与修复。

---

## 2026-08-18 — 阶段 1 开始

### LOG-001 | 阶段 1 | 细化方案 + 基线探测

**操作：** 在 `dmo_place_1.md` §7 / §9 写明阶段 1A（gym.make）与 1B（GPU dummy SAC）命令与门闩。宿主机探测 Docker / GPU / 权重 / 现有容器。

**命令：**
```bash
docker ps -a --format '{{.Names}} {{.Image}}'
docker inspect rlinf --format '{{range .Mounts}}{{.Source}} -> {{.Destination}}{{end}}'
nvidia-smi -L
ls /home/nvidia/ckpts/RLinf-ResNet10-pretrained/resnet10_pretrained.pt
```

**结果：**
| 项 | 值 |
|----|-----|
| GPU | NVIDIA GeForce RTX 5090 D |
| 权重 | `/home/nvidia/ckpts/RLinf-ResNet10-pretrained/resnet10_pretrained.pt` 存在 |
| 镜像 | `rlinf/rlinf:agentic-rlinf0.4-franka`、`rlinf/rlinf:agentic-rlinf0.4-maniskill_libero` |
| 容器 `rlinf` | 挂载 **`/home/nvidia/cxy_ws/RLinf`**，不是本仓库。**禁止** `docker exec rlinf` 测本任务 |

**文件：** 更新 `b/d/frk1/dmo_place_1.md` §7、§9.1；新建本日志。

---

### LOG-002 | 阶段 1 | 编码（无真机）

**操作：** 按 §7 增加闭爪触达 env、Gym 注册、Hydra env 包、dummy YAML、1A/1B 脚本。

**新增/修改文件与原因：**

| 文件 | 增删改 | 原因 |
|------|--------|------|
| `b/x/franky_ext/tasks/cube_place.py` | 新增 | 独立 Gym 任务：§5 几何；`go_to_rest` 闭爪 −1 再抬升 |
| `b/x/franky_ext/tasks/register.py` | 改 | 注册 `FrankyCubePlaceEnv-v1`，不改 `rlinf/` |
| `b/x/configs/env/realworld_cube_place.yaml` | 新增 | Hydra env 包；不设 `no_gripper: False` |
| `b/x/configs/realworld_cube_place_dummy_sac.yaml` | 新增 | CPU dummy 骨架 |
| `b/x/configs/realworld_cube_place_dummy_sac_gpu.yaml` | 新增 | 本机 1B 主配置：placement `0-0`，`action_dim: 6` |
| `b/x/scripts/step_cube_place_dummy.py` | 新增 | 1A 门闩：6D、clip、闭爪源码、dummy step |
| `b/x/scripts/run_cube_place_dummy_sac_gpu.sh` | 新增 | 对齐 7b；**不**调 `step7_install_deps.sh` |

下一步：1A `docker run` franky 镜像跑 `step_cube_place_dummy.py`。

---

### LOG-003 | 阶段 1A | gym.make dummy | PASS

**操作：** 本仓库 bind-mount 进官方 franky 镜像（不用已有 `rlinf` 容器），`source setup_before_ray_5090.sh` 后跑 `step_cube_place_dummy.py`。

**命令：**
```bash
docker run --rm --privileged --network host \
  -v /home/nvidia/bt/s/RLinf:/workspace/RLinf \
  -w /workspace/RLinf \
  rlinf/rlinf:agentic-rlinf0.4-franka \
  bash -lc 'source b/x/configs/setup_before_ray_5090.sh && python b/x/scripts/step_cube_place_dummy.py'
```

**结果：** exit 0，约 7s。
- `gym_id=FrankyCubePlaceEnv-v1`
- wrapper：`GripperCloseEnv` → `FrankyCubePlaceEnv`（6D 闭爪）
- `action_space=Box(-1,1,(6,),float32)`
- clips `xy=0.05 z_low=0.005 z_high=0.08 rand_xy=0.03`
- `go_to_rest` 使用 `-1.0` 闭爪
- dummy `step` 无 FCI

**警告（非失败）：** gymnasium PassiveEnvChecker 对 obs dtype float64 / 越界的 WARN，与现有 Step4 dummy 同类，官方 Franka dummy 亦如此。

**门闩：** 1A-1 … 1A-5 全部满足。

下一步：1B GPU dummy SAC。


---

### LOG-004 | 阶段 1B | GPU dummy SAC | PASS

**操作：** 用 `maniskill_libero` CUDA 镜像（`--gpus all`），`source setup_before_ray_gpu_5090.sh`（`switch_env openvla`，**不**跑 `step7_install_deps.sh`），执行 `run_cube_place_dummy_sac_gpu.sh`。

**命令：**
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

**结果：** docker exit 0，约 72s。
- `CUDA OK: torch=2.11.0+cu128 device=NVIDIA GeForce RTX 5090 D`
- 启动前打印 `gym FrankyCubePlaceEnv-v1`
- Ray head `10.229.18.21:6379`
- 日志目录：`logs/20260818-062629-realworld_cube_place_dummy_sac_gpu/`
- TensorBoard `config.yaml`：`init_params.id: FrankyCubePlaceEnv-v1`（train/eval），`actor.model.action_dim: 6`
- Metric Table Global Step 1/2 与 2/2：`sac/actor_loss`、`sac/critic_loss`、`sac/alpha` 均有值；`episode_len=100`；无 shape / FSDP 报错
- env `reward=0.0`、`success_once=0.0`：dummy TCP 全零，目标 `[0.5,0,0.1,…]`，预期碰不到，不阻塞阶段 1

**警告（非失败）：** Hydra `_self_` / version_base；TF cuDNN factory already registered；Python interpreter path `python` vs `python3` 提示。与 7b dummy 同类。

**未出现的错误：** 无 Traceback；无 `PegInsertionEnv-v1` 误用；无 7D / `no_gripper: False`。

**门闩：** 1B-1 … 1B-4 全部满足。

---

### LOG-005 | 阶段 1 | 验收结论

| 门闩 | 结果 |
|------|------|
| 1A-1 Gym ID | PASS |
| 1A-2 action (6,) | PASS（含 GripperCloseEnv） |
| 1A-3 clip 0.05 / 0.005 / 0.08 | PASS |
| 1A-4 go_to_rest 闭爪 −1 | PASS |
| 1A-5 dummy reset+step | PASS |
| 1B-1 CUDA | PASS |
| 1B-2 train/sac + exit 0 | PASS |
| 1B-3 无维数错误 | PASS |
| 1B-4 FrankyCubePlaceEnv-v1 | PASS |

**阶段 1 总评：PASS。** 本阶段未连真机、未占 FCI。下一步阶段 2（标定 H0–H1 + 安全盒短跑）。

---

## 2026-08-18 — 阶段 2 开始

### LOG-006 | 阶段 2 | 操作手册与脚本落盘（尚未动臂验收）

**操作：** 把 `dmo_place_1.md` §9 阶段 2 写成操作手册（Desk → 容器 → H1 写 pose → `connect` / `reset` / `box`）。生成标定 YAML 与真机短跑脚本；约定**先不执行**会动臂的 `reset`/`box`。

**新增文件：**

| 文件 | 作用 |
|------|------|
| `b/x/configs/cube_place_target_ee_pose.yaml` | H1 六元组；默认 `calibrated: false` |
| `b/x/scripts/write_cube_place_pose.py` | 把 `getpos_euler` 写入上表 |
| `b/x/scripts/step_cube_place_robot.py` | 真机 `connect` / `reset` / `box` |
| `b/x/scripts/run_cube_place_phase2.sh` | 容器内子命令封装 |

**结果：** 手册与脚本已就绪。当时手册 §2.5 仍写官方 `python -m toolkits.realworld_check.test_franky_controller`，随后在真机执行中暴露为 LOG-007。

---

### LOG-007 | 阶段 2.5 | 官方 `test_franky_controller` + 原装 Hand | FAIL

**操作：** 按当时手册 §2.5，在 franky 容器内（已 `source setup_before_ray_5090.sh`）标定开合爪。

**命令：**
```bash
export FRANKA_ROBOT_IP=172.16.0.2
export FRANKA_GRIPPER_TYPE=franka
python -m toolkits.realworld_check.test_franky_controller
```

**现象：** 输入 `close` 前后，Ray 拉起本地 cluster，`FrankyController` 创建任务失败，达不到可用的 `cmd>` 标定流程。

**关键报错（节选，约 2026-08-18 07:03 UTC / 15:03 CST）：**
```
Could not connect to an existing Ray cluster. Initializing a new cluster ...
WARNING: The object store is using /tmp/ray instead of /dev/shm
  because /dev/shm has only 67108864 bytes available.
FrankyController pid=80345 ... Using communication devices ... eno2
NotImplementedError: FrankyController: the libfranka backend for the original
Franka Hand is not yet supported. Use gripper_type='robotiq' for now.
```
栈：`franky_controller.py` `__init__` → `_build_gripper`（约 115 / 142 行）。Actor 名 `FrankyController-0-0`（**upstream**，不是 Extended）。

**原因：**
1. 官方 toolkit 启动 `rlinf/.../franky_controller.py` 的 `FrankyController`。`gripper_type=franka` 时 **故意** `NotImplementedError`，只声明支持 Robotiq。本机是原装 Franka Hand，`FRANKA_GRIPPER_TYPE=franka` 设对了，反而踩中该分支。
2. `/dev/shm` 64MB 是 franky 容器默认 shm，**只是 Ray 警告，不是这次失败原因**。
3. 失败过程在容器内留下**本地 Ray**；`__init__` 在建夹爪前已 `franky.Robot(ip)`，需确认 1337 已释放后再重试。

**修复（不改 `rlinf/`）：**
- 新增 `b/x/scripts/test_franky_controller_ext.py`：交互 REPL，启动 `FrankyControllerExtended` + `FrankaLibfrankaGripper`。命令与官方 REPL 对齐（`open` / `close` / `getpos_euler` / `q`），**不**自动 `home`。
- `run_cube_place_phase2.sh` 增加 `calibrate`；禁止再调官方 toolkit。
- 更新 `dmo_place_1.md` §4.2 / §8.3 / §9 阶段 2：本机标定入口改为 Extended。明确不要用 `step3_test_controller.py` 标定（会自动 home 并张爪）。

**重试步骤（执行侧）：**
```bash
ray stop
ss -tn state established '( dport = :1337 or sport = :1337 )'
python b/x/scripts/test_franky_controller_ext.py
# 或: bash b/x/scripts/run_cube_place_phase2.sh calibrate
```
应看到 `FrankyControllerExtended REPL` 与 `Connected to Franka at 172.16.0.2`，然后才是 `cmd>`。

**门闩：** 2.5 官方入口 **FAIL**；改走 Extended 后由后续标定继续（见 LOG-008 夹爪力）。

---

### LOG-008 | 阶段 2.5 | `close` 以 130 N 持续力控夹方块 | FAIL

**操作：** 已能进 Extended REPL 后，在 `cmd>` 输入 `close` 夹小方块。

**现象：** 夹爪一直用力夹，方块几乎被夹碎。操作员指出：夹紧即可，不应持续大力；夹到人会很不安全。

**原因：**
- `FrankaLibfrankaGripper.close()` 调用 `franky.Gripper.grasp(width=0.01, force=130, …)`。libfranka 的 `grasp` **在接触后仍维持设定力**，不是「合到位就卸力」。
- 130 N 从 ROS `FrankaGripper` 抄来，接近 Franka Hand 上限，对小方块过猛。
- 上游 `FrankyController.close_gripper()` 还传 `speed=1.0`，闭合过快。

**修复（`b/x/`）：**
| 文件 | 改动 |
|------|------|
| `b/x/franky_ext/franka_libfranka_gripper.py` | 默认抓取力 **20 N**，硬上限 **40 N**；闭合速度封顶 **0.08 m/s**；可用 `FRANKA_GRASP_FORCE`；增加 `stop()` |
| `b/x/franky_ext/controller_extended.py` | 覆盖 `close_gripper`（慢速轻力）；新增 `stop_gripper` / `close_gripper_force` |
| `b/x/scripts/test_franky_controller_ext.py` | `close [N]`、`stop`；帮助里写明轻力抓取 |
| `dmo_place_1.md` §4.2 / §9 阶段 2.5 / 2.10 | 记录 20 N 默认与死夹恢复 |

说明：夹住后仍会维持这 **20 N**（API 没有「完全卸力还握着」）；这是握持力，不应再把方块夹扁。打滑可 `export FRANKA_GRASP_FORCE=30`，不要回到 130 N。

**执行注意：** 正在跑的 Ray actor **不会**自动加载新代码。若当时还在死夹：`open` 或 `stop` 或急停 → `q` → `ray stop` → 重新 `python b/x/scripts/test_franky_controller_ext.py`。新进程日志应有 `force=20.0N`。

**门闩：** 旧 130 N `close` **FAIL**（安全）。轻力修复已落盘；是否夹稳、是否完成 `getpos_euler` 写入 YAML，待下一轮真机确认。

---

**阶段 2 截至 LOG-008：** 手册/脚本已有；官方标定入口与 130 N 死夹两处执行故障已定位并改 `b/x/`。**尚未**验收 2.6 `connect`、2.7 `reset`、2.8 `box`（会动臂）。

---

### LOG-009 | 阶段 2.6 | `write-pose` + `connect` 只读几何 | PASS

**操作：** 人完成 H1 `getpos_euler` 后写入 YAML，再 `source setup_before_ray_5090.sh` 跑 `connect`（不动臂、不 `gym.make`）。

**命令：**
```bash
python b/x/scripts/write_cube_place_pose.py \
  0.7062065 0.03620906 0.23192134 -3.11614319 0.02628124 0.17913087
bash b/x/scripts/run_cube_place_phase2.sh connect
```

**YAML：** `calibrated: true`，`target_ee_pose` 与上列六元组一致。

**connect 输出（核对）：**

| 项 | 值 | 判定 |
|----|----|------|
| target H1 | `[0.7062, 0.0362, 0.2319, -3.1161, 0.0263, 0.1791]` | 与 YAML 一致 |
| hover z | `0.3119` = `0.2319+0.08` | 符合 |
| 盒子 xy | `x∈[0.6562,0.7562]` `y∈[-0.0138,0.0862]` 半宽 0.05 | 符合 |
| 盒子 z | `[0.2269, 0.3119]` = 接触±(0.005 / 0.08) | 符合 |
| probed − target xyz | `[-0.0036, 0, +0.0389]` | xy 约 4 mm；当前比接触高约 **3.9 cm**（引导后「几厘米」上方，正常） |
| 结束语 | `connect-only OK (no env created)` | 未创建 env |

**说明：** 手册写「悬停时 z 大约 +0.05～0.10 m」指的是 **2.7 reset 之后**应到 +8 cm。`connect` 不运动，当前 +3.9 cm 表示臂已在标记上方附近，可以进 2.7。`x≈0.71 m` 在 Panda 工作半径内。

**门闩：** 2.5-1 写文件、2.6-1 只读几何 **PASS**。下一步：人在急停旁跑 `bash b/x/scripts/run_cube_place_phase2.sh reset`（会动臂，抬到接触 + 8 cm）。

---

### LOG-010 | 阶段 2.7 | `reset` 在已夹持时再次 `grasp` | FAIL

**操作：** LOG-009 之后，方块仍夹着、臂在标记上方约 4 cm，执行 `bash b/x/scripts/run_cube_place_phase2.sh reset`。

**已正常的部分：**
- H1 / 盒子打印与 `connect` 相同
- 创建 `FrankyCubePlaceEnv-v1`
- `safe_smoke_hold: skip __init__ _interpolate_move`
- `FrankyControllerExtended` 连上 `172.16.0.2`
- 夹爪 `grasp_force=20.0N`（LOG-008 修复生效）
- wrapper：`Quat2EulerWrapper -> GripperCloseEnv -> FrankyCubePlaceEnv`

**非失败：** `/dev/shm` 64MB 警告；gymnasium `env.config` deprecated WARN。

**失败点：** `reset()` → `go_to_rest` → `_end_effector_action([-1])` → `close_gripper` → `grasp(width=0.01, force=20N)`：
```
franky._franky.CommandException: libfranka gripper: Command failed!
Exiting main process due to a failure upon worker execution.
```
臂在报错前**没有**完成抬到 8 cm 悬停。

**原因：** H1 已经 `close` 夹住方块。libfranka 对**已在 grasp 保持中**的夹爪再发 `grasp` 会失败。软件侧 `__init__` 把 `is_open` 默认成 True，env 以为还张着，于是又关一次。

**修复：** `FrankaLibfrankaGripper`：按 `is_grasped` / 当前宽度判断已夹持则 **skip grasp**；`is_open` 跟硬件；`Command failed` 但手指仍握着则继续。不改 `rlinf/`。

**重试（会动臂，须急停旁；先清失败留下的 Ray）：**
```bash
ray stop
ss -tn state established '( dport = :1337 or sport = :1337 )'
# 方块仍应在爪中；不要张爪
bash b/x/scripts/run_cube_place_phase2.sh reset
```
期望：`holding=True` 或 `already holding ... skip grasp`，然后抬到 z≈0.312 m，`reset-only PASS`。

---

### LOG-011 | 阶段 2.8 | `box`：夹爪过了，臂没到悬停 | FAIL（先不改代码）

**操作：** `bash b/x/scripts/run_cube_place_phase2.sh box`（内含 reset + 零动作 + 下探）。时间约 2026-08-18 08:05 UTC。

**日志说明（按时间）：**

| 段落 | 含义 |
|------|------|
| H1 / hover / 盒子与 `connect` 相同 | 标定仍有效；当前仍在接触点上方约 3.9 cm |
| 新建本地 Ray、`/dev/shm` 64MB | 与 LOG-007 同类**警告**，不是这次退出原因 |
| `FrankyControllerExtended` + `grasp_force=20.0N` | 连臂成功 |
| `safe_smoke_hold: skip __init__ _interpolate_move` | **只**跳过构造时插值；`reset()` 仍应动臂 |
| `gripper grasp: … force=20.0N` | 这次 `grasp` **没有** Command failed（与 LOG-010 不同） |
| Cartesian impedance tracker started | 笛卡尔阻抗已起来，准备跟 TCP 目标 |
| `wrapper … GripperCloseEnv` | 6D 闭爪栈正确 |
| gymnasium dtype/obs 越界 WARN | dummy/真机 Franka 同类，非退出原因 |
| `reset OK` + `after reset xyz=[0.7025, 0.0362, 0.2708]` | **与脚本开头 probed 相同**，臂基本没动 |
| `gripper_position≈0.0315` | 开口约 3.1 cm，方块仍在爪中 |
| `\|xy-target\|=0.0037m` | xy 已对准标记（门闩 3 cm 内，过） |
| `\|z-hover\|=0.0412 > 0.025` | 期望 z=`0.2319+0.08=0.3119`，实际 `0.2708`，差 **4.1 cm** |
| `VideoPlayer has no attribute stop` | `env.close` 的相机 stub，可忽略 |
| `RuntimeError: arm did not go to mark+0.08m` | 烟测**故意停**，避免没悬停就做 box 步进 |

**结论：** 不是 FCI、不是 20 N、不是 shm。失败点是 **`reset()` 之后 TCP 仍停在 reset 前的高度**，没有到接触点上方 8 cm。`box` 的零动作/下探根本没开始。

**和 LOG-010 的差别：** 本次 `grasp` 成功了；挡在后面的是 **高度门闩**。

**日志还不能单独钉死的原因（需下次看运动/插值）：** `FrankaEnv.go_to_rest` 对 xyz 容差 2 cm、最多插值 3 次后会放弃；`CubePlace.go_to_rest` 会先把**当前** z 再加 8 cm（约 0.35 m），高于盒子上沿 0.312 m，和真正的 hover 目标不一致。阻抗跟踪刚启动时插值也可能跟不住。本次 TCP 与 probed 完全一致，更像指令未落实，而不是走到半路。

---

### LOG-012 | 阶段 2.7 | `reset`：skip grasp 成功，臂仍未悬停 | FAIL

**操作：** `bash b/x/scripts/run_cube_place_phase2.sh reset`（约 08:34 UTC）。skip-grasp 修复已加载。

**已正常：**
- `holding=True width=0.0464m`，**没有**再 `grasp` / Command failed（LOG-010 已修）
- `safe_smoke_hold` 只 skip init 一次
- wrapper 6D 闭爪；xy 差 2 mm；方块仍在（width≈4.6 cm）

**失败：** `after reset xyz` 与 probed 相同 `[0.7051, 0.0345, 0.2626]`；`|z-hover|=0.0493>0.025`。期望悬停 z=0.3119。笛卡尔阻抗有启动，但 TCP 高度没变。

**原因：** `go_to_rest` 把**当前** z 再加 8 cm（0.263+0.08≈0.343），高于盒子上沿 0.312 m；真正 hover 是 H1+8 cm。Peg Step 6 是从接触点抬 5 cm 刚好到盒顶，所以能看到 ~38 mm 抬升；这里目标超出盒子，阻抗跟不住，门闩判定没到悬停。

**修复：** `cube_place.go_to_rest` 改为插值到 `_reset_pose`（H1+8 cm），timeout 3 s；烟测 settle 后再读 live TCP。不改 `rlinf/`。

**重试：**
```bash
ray stop
bash b/x/scripts/run_cube_place_phase2.sh reset
```
期望日志含 `cube_place go_to_rest: tcp_xyz=... -> hover_xyz=[0.7062, 0.0362, 0.3119]`，目视上抬，`reset-only PASS`。

---

### LOG-013 | 阶段 2.7 | 插值 9 s 但 dz=0 | FAIL

**操作：** 已改 hover 目标后重跑 `reset`（约 08:43 UTC）。

**新证据：**
```
cube_place go_to_rest: tcp_xyz=[0.7051, 0.0345, 0.2626] -> hover_xyz=[0.7062, 0.0362, 0.3119]
Cartesian impedance tracker started
cube_place go_to_rest done: tcp_xyz=[0.7051, 0.0345, 0.2626] dz=0.0000
```
目标对了（H1+8 cm，在盒子内），插值跑了约 9 s，阻抗跟踪有启动，**TCP 完全没动**。LOG-012「抬出盒子」不是根因。

**原因：** `FrankaEnv._interpolate_move` 每个路点调用 `_move_action` → `_clear_error()` → `robot.recover_from_errors()`。libfranka 的 recover 会**打断**正在跑的 `CartesianImpedanceTracker`，但 Python 里 `_cart_tracker` 仍非 None，之后 `set_target` 等于空操作。

**修复（`b/x/`）：**
- `FrankyControllerExtended.clear_errors`：笛卡尔跟踪进行中不 recover
- mixin `_move_action`：路点之间不再 recover

**重试：**
```bash
ray stop
bash b/x/scripts/run_cube_place_phase2.sh reset
```
期望 `dz` 明显为正（约 3–8 cm），live z 接近 0.312 m，`reset-only PASS`。

---

### LOG-014 | 阶段 2.7 | 去掉 recover 后仍 dz=0 | FAIL → 改 CartesianMotion

**操作：** 用户重跑 `bash b/x/scripts/run_cube_place_phase2.sh reset`（约 08:48 UTC），此时 LOG-013 的 skip-recover 已加载。

**日志（要点）：**
```
holding=True width=0.0464m
cube_place go_to_rest: tcp_xyz=[0.7051, 0.0345, 0.2626] -> hover_xyz=[0.7062, 0.0362, 0.3119]
Cartesian impedance tracker started (K_t=2000 K_r=150.0 tc=0.089)
cube_place go_to_rest done: tcp_xyz=[0.7051, 0.0345, 0.2626] dz=-0.0000
after settle live xyz=[0.7051, 0.0345, 0.2626]
hover check: |xy-target|=0.0020m |z-hover|=0.0493m
RuntimeError: after reset, |z-hover|=0.0493 > tol 0.0250
```

**结论：** skip grasp、hover 目标、盒子都对。插值跑了约 9 s，阻抗 tracker 有启动，**TCP 高度完全不变**。LOG-013 的 recover 假说被这次重跑证伪（去掉 recover 仍 dz=0）。Peg Step 6 同类 `go_to_rest` 曾抬过 ~38 mm，但那是「当前位姿 +z」且当时 mixin 仍走父类 `_move_action`；本任务不能再把 reset 押在 10 Hz `set_target` 上。

**修复（只改 `b/x/`）：**
- `FrankyControllerExtended.move_tcp_cartesian_motion`：停掉 tracker 后 `robot.move(CartesianMotion(Affine, Absolute))`，与 `home`/`reset_joint` 同一类阻塞运动
- mixin `_interpolate_move`（非 init、非 dummy）：走上述 CartesianMotion，不再用阻抗路点
- `cube_place.go_to_rest`：先平移到 hover xyz，姿态用当前 live 四元数（避免边抬边拧）
- 阻抗 tracker 构造后若有 `__enter__` 则调用（给后续 `env.step` 用）；reset 本身不再依赖它

**重试（会动臂，须急停旁；先清失败留下的 Ray）：**
```bash
ray stop
ss -tn state established '( dport = :1337 or sport = :1337 )'
bash b/x/scripts/run_cube_place_phase2.sh reset
```
期望日志含 `CartesianMotion Absolute` / `CartesianMotion done` 且 `dz` 约 +0.05 m，live z≈0.312 m，`reset-only PASS`。运动比 10 Hz 插值更干脆，相对动力学因子仍是 0.2。

---

### LOG-015 | 对照 charger 原代码 | 撤回 CartesianMotion，改回 PegInsertion 两段 reset

**操作：** 用户要求对照 `realworld_charger_sac_cnn_async.yaml` 与 RLinf 原实现，而不是另发明运动接口。

**原版结论（YAML 本身没有 `impedance_mode` 开关）：**

| 项 | 充电器 / PegInsertion | 我们之前错在哪 |
|----|----------------------|----------------|
| 开阻抗 | ROS `FrankaController.__init__` 里 `start_impedance()`（`roslaunch impedance.launch`）。YAML 只给 `compliance_param`（K_t=2000 等）。`reset()` 里 `reconfigure_compliance_params`。`move_arm` 往 `/cartesian_impedance_controller/equilibrium_pose` 发平衡点 | franky 用 `CartesianImpedanceTracker` 对应；`reconfigure` 会停掉 tracker，必须紧接着 `_move_action(当前)` 才能再启动 |
| 抬高 | **有。** `PegInsertionEnv.go_to_rest`：闭爪 → `_move_action(当前)` → **当前 TCP 的 z += 0.10**（相对**此刻末端**，不是相对 H1）→ `FrankaEnv.go_to_rest` 再到 `reset_ee_pose` | 改成一次性插值到绝对 hover；还跳过了 seed |
| 悬停点 | `PegInsertionConfig.__post_init__`：`reset_ee_pose = target + [0,0,clip_z_high,0,0,0]`。charger YAML `clip_z_range_high: 0.05` → 相对**标定接触/插入点**上方 5 cm | 把「相对当前 +10 cm」和「相对 target + clip_z」混成一件事 |
| 插值 | `_interpolate_move` 10 Hz 路点 `move_arm`；**不**按 `ee_pose_limit` 裁剪。拔插头时可以先高出盒子再落回悬停 | 误判「高出盒子所以阻抗不动」；又改成 `CartesianMotion` |

**修正（只改 `b/x/`）：**
- 撤回 mixin 的 `CartesianMotion` 插值；reset 仍走原版 10 Hz 阻抗路点
- `cube_place.go_to_rest` 按 `PegInsertionEnv` 抄：seed 当前 → 相对当前 +10 cm → `FrankaEnv.go_to_rest`（target+8 cm）
- smoke 不再把 `reset_z_lift_m` 设成 `clip_z_high`（那是两件不同的量）
- `reconfigure_compliance_params` 读入 charger 的 `translational_clip_*`（默认 3 mm / 10 mm z）

**重试：**
```bash
ray stop
bash b/x/scripts/run_cube_place_phase2.sh reset
```
期望：`current ... +z=0.100`，目视先上抬再落到标记上方约 8 cm，`reset-only PASS`。

---

### LOG-016 | 读 franky 0.19 源码 + 只读探测 | 定位到「异步 torque motion 静默死亡」

**操作：** 用户重跑 `reset`（约 00:09 UTC）。序列已是 PegInsertion 两段，日志确认 `current [0.692, 0.0368, 0.2686] +z=0.100 -> [0.692, 0.0368, 0.3686]`，7 s 后 **`dz=-0.0000`**，`|z-hover|=0.0433 > 0.025` FAIL。

#### 为什么 RLinf 的充电器能动，我们不能：**两套控制器不是一回事**

| | 充电器（官方，能跑） | 我们（franky） |
|---|---|---|
| 后端 | `FrankaController`（ROS/franka_ros） | `FrankyController`（libfranka 0.19 + franky） |
| 阻抗怎么开 | `__init__` 里 `start_impedance()` → `roslaunch impedance.launch`，**常驻 1 kHz** `cartesian_impedance_controller` | `CartesianImpedanceTracker(...)`，构造器内 `robot.move(motion, asynchronous=True)` |
| `move_arm` 干什么 | 往 `/cartesian_impedance_controller/equilibrium_pose` **发 topic**；控制器一直在跟 | `set_target()` 写 `CartesianReferenceHandle` |
| 控制器挂了会怎样 | ROS 节点还在，topic 继续被消费 | **异步线程存下异常并退出；`set_target` 变成写死句柄，静默无效** |
| reset 怎么走 | `_interpolate_move` 10 Hz 路点（阻抗跟） | 见下：官方 franky env **不用**阻抗做 reset |

`franky/tracker.py`（容器内 `/opt/venv/franky-0.19.0/.../franky/tracker.py`）关键三行：

```python
motion = CartesianImpedanceTrackingMotion(reference_handle=self._reference_handle, **kwargs)
self._robot.move(motion, asynchronous=True)   # 构造即启动，无需 __enter__
...
@property
def is_running(self): return self._robot.is_in_control
```

`stop()` 里 `join_motion()` 会**重抛**杀死控制线程的真正异常。也就是说：`dz=0.0000` 完全符合「异步 torque motion 早就死了，但我们从来没检查过 `is_running`，也从来没 `join_motion` 过」。

#### RLinf 自己的 franky env 怎么做 reset

`DualFrankaEnv._go_to_rest`（`rlinf/envs/realworld/franka/dual_franka_env.py`）是仓库里**唯一**用 franky 后端的 env reset：

```python
self._left_ctrl.reset_joint(self.config.joint_reset_qpos[0])   # 阻塞 robot.move(JointMotion)
```

它**不做**笛卡尔插值 reset，`move_tcp_pose`（阻抗）只用于 `step()` 的小增量；`_clear_errors()` 每个 reset 只调一次，**不是每个路点**。所以「大位移用阻塞 `robot.move`，小增量用阻抗」才是 RLinf 对 franky 的既有设计。

#### 只读探测（未动臂）

```
b/x/scripts/diag_franky_motion.py --probe
probe: has_errors=False is_in_control=False signal=None
probe tcp xyz=[0.7001, 0.0347, 0.2510]
probe q=[-0.0008, 0.5436, 0.0494, -1.4950, 0.0240, 2.0335, 0.6828]
```

机器人现在**没有** latched fault，所以不是「残留错误挡住」。注意 z 从失败时的 0.2686 掉到 0.2510（约 −1.8 cm）：进程退出时若还在 torque 模式，控制连接断开会短暂失控下沉，反过来说明那条 motion 至少一度在控。

#### 代码审计：我自己引入的 4 处偏离（全部撤回）

peg Step 6（`franka_3LOG` LOG-022）在**完全上游**的行为下抬升过 38 mm。之后我加的东西都没有证据支持：

| 改动 | 依据 | 处理 |
|------|------|------|
| `clear_errors` 跟踪中不 recover（LOG-013） | 假设 recover 掐死 tracker；LOG-014 去掉后仍 dz=0，**已证伪** | 撤回，回到上游 |
| mixin `_move_action` 不 `_clear_error()` | 同上 | 撤回，回到上游 |
| tracker 构造后调 `__enter__`（LOG-014） | 照抄 franky 示例的 `with`；读源码后确认 `__enter__` 只 `return self`，**构造器已经 `move`** | 删除 |
| `translational_clip_*` 映射成 franky `translational_error_clip`（LOG-015） | charger 那些键是 **ROS** `cartesian_impedance_controller` 的，语义不同；3 mm 会把 xy 权限砍到 6 N | 撤回，用 franky 默认 0.05 m |

#### 真正的修复：让失败大声报错

`FrankyControllerExtended.move_tcp_pose` 覆写：`super()` 之后检查 `self._cart_tracker.is_running`；若已死则 `stop()`（`join_motion` 重抛真因）、清 tracker、`log_error` + `raise RuntimeError`。tracker 启动日志也加 `is_running=`。这样下一次要么动，要么打印出 libfranka 的真实原因（`cartesian_reflex` / `joint_position_limits_violation` / `communication_constraints_violation` / `power_limit_violation` …），不再是静默 `dz=0`。

#### 新增诊断脚本（绕开 Ray 与 FrankaEnv）

`b/x/scripts/diag_franky_motion.py`：`--probe`（只读）、`--test-hold`（目标=当前位姿，只测 tracker 存活）、`--test-impedance`（ramp z）、`--test-cartesian-motion`（阻塞 `robot.move`）。后三个需 `--yes-move`。

**回归：** Phase 1A dummy 仍 PASS（`gym_id=FrankyCubePlaceEnv-v1`、6D、闭爪 reset）。

**下一步（人在急停旁；先 `ray stop`）：**
```bash
ray stop
source /workspace/RLinf/b/x/configs/setup_before_ray_5090.sh
python b/x/scripts/diag_franky_motion.py --test-hold --test-impedance --test-cartesian-motion --yes-move --dz 0.03
```
判读：
- `hold` 就死 → 1 kHz torque 链路问题（RT 调度 / 容器 / FCI），SAC 的 `step` 也不可能work，必须先解决
- `hold` 活但 `impedance` 不动 → 刚度/clip/位形权限不足，调 `RLINF_CART_*`
- `impedance` 不动而 `CartesianMotion` 动 → 按 `DualFrankaEnv` 的分工：reset 用阻塞 motion，`step` 用阻抗

---

### LOG-017 | 根因 | **user-stop 被按住**：`RobotMode.UserStopped`，代码无关

> 本条之后手册已重写为 [`dmo_place_2.md`](dmo_place_2.md)（v1 仅留作设计记录）。


**操作：** 用户按 LOG-016 跑三项诊断（约 00:30 UTC）。

**结果（三项全 FAIL，且都是同一条错误）：**

```
=== impedance hold (no displacement) for 3.0s ===
tracker created: is_running=True
  !! died at t=0.1s live=[0.7001, 0.0347, 0.2510]
tracker.stop() surfaced: ControlException: libfranka: Move command rejected:
    command not possible in the current mode ("User stopped")!
=== impedance tracker: ramp z by +0.030 m ===
after create: has_errors=False is_in_control=True signal=ControlSignalType.Torques
  step 1/30 ... is_running=False        # 100 ms 内就死
=== blocking CartesianMotion: z +0.030 m ===
move() raised: ControlException: ... ("User stopped")!
```

**只读复核（`robot.state.robot_mode` 不需要动臂）：**

```
robot_mode      : RobotMode.UserStopped
has_errors      : False
current_errors  : []
cmd_success_rate: 0.0
```

**根因：** 手持设备（enabling device）的 **user-stop 被按下**。此状态下 libfranka 拒绝**一切** `robot.move`，而
- `robot.state` / `O_T_EE` 读取正常 → 探测出来的 TCP 全都是对的
- `franka::Gripper` 命令正常 → `holding=True width=0.0463m`、闭爪逻辑全部按预期走
- `robot.move(..., asynchronous=True)` **立刻返回**，`is_in_control` 有约 100 ms 为 True、`signal=Torques` → tracker 打印 `is_running=True` 后当场死掉

所以 `reset` 一路 `reset OK`、几何全对、`dz=0.0000`。**LOG-011 起所有 `dz=0` 都是这一个原因**，与 `recover_from_errors`、`CartesianMotion` vs 阻抗、`translational_error_clip`、两段 reset 顺序**全都无关**。`cmd_success_rate: 0.0` 是最直接的旁证。

也解释了 peg Step 6（`franka_3LOG` LOG-022）为什么能抬 38 mm：那次 user-stop 是松开的。以及 0.2686 → 0.2510 那 1.8 cm：切入 user-stop 时的受控停止下沉。

**教训（我的）：** 「人在急停旁」被执行成了「按住 user-stop」，而我从 LOG-011 到 LOG-016 一直在几何和运动接口里找原因，没有先读 `robot_mode`。franky 上 `dz=0` 的第一步永远是查模式，不是改代码。

**修复（只改 `b/x/`）：**
- `tcp_probe.py`：新增 `probe_robot_state`（子进程读 pose + `robot_mode` + `has_errors`）、`describe_robot_mode`、`require_motion_ready`；`UserStopped` / `Guiding` / `Reflex` 各给可执行提示
- `step_cube_place_robot.py`：打印 `robot_mode`；`reset` / `box` 在 `gym.make` **之前** `require_motion_ready`（省掉 30 s Ray 启动后才发现）；`connect` 只提示不拦
- `controller_extended._ensure_cart_tracking_motion`：建 tracker 前检查模式，非 `Idle` 直接抛
- `controller_extended.move_tcp_pose`：tracker 死亡信息带上 `mode=` 和模式解释
- `diag_franky_motion.py`：`--probe` 打印 `robot_mode` + 提示；非 `Idle` 时**跳过**运动测试（`--force` 可强来）
- 手册 §2.1 加规则 1b、§2.3 检查单加只读模式确认

**验证：**
```
python b/x/scripts/diag_franky_motion.py --probe
probe robot_mode=RobotMode.UserStopped
probe WARNING: robot_mode=RobotMode.UserStopped: the user-stop button ... Pull the user-stop button up ...
skipping motion tests (pass --force to try anyway)   # exit=1
```
Phase 1A dummy 仍 PASS。

**下一步（松开 user-stop 后）：**
```bash
python b/x/scripts/diag_franky_motion.py --probe          # 必须 RobotMode.Idle
python b/x/scripts/diag_franky_motion.py --test-hold --test-impedance --yes-move --dz 0.03
ray stop && bash b/x/scripts/run_cube_place_phase2.sh reset
```
`--test-hold` OK 且 `--test-impedance` dz≈+0.03 → 阻抗链路可用，`reset` 应直接过（`dz≈+0.10` 后落到 `target+0.08`）。若 hold 活而 impedance 仍不动，再谈刚度/位形，那时才是真的调参。

