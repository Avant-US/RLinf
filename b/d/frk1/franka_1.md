# 单臂 Franky 改造设计方案（固件 5.10.0）

> **目标环境：** Franka Emika Panda · 固件 **5.10.0** · libfranka **0.19.0** · Franka Hand（原生夹爪）  
> **目标能力：** 在 RLinf 上跑通单臂真机全流程——数据采集、Pi0 SFT 部署、RLPD、HG-DAgger、RLT（RL Token）、RTC  
> **关联 Issue：** [RLinf#1477](https://github.com/RLinf/RLinf/issues/1477)

---

## 目录

1. [背景与结论](#1-背景与结论)
2. [固件 5.10.0 版本锁定](#2-固件-5100-版本锁定)
3. [现状与差距分析](#3-现状与差距分析)
4. [总体架构设计](#4-总体架构设计)
5. [Phase 0：控制器与 Franka Hand（第 1–2 周）](#5-phase-0控制器与-franka-hand第-12-周)
6. [Phase 1：单臂 FrankyEnv（第 2–4 周）](#6-phase-1单臂-frankyenv第-24-周)
7. [Phase 2：各工作流打通（第 4–7 周）](#7-phase-2各工作流打通第-47-周)
8. [Phase 3：测试、文档与 CI（第 7–8 周）](#8-phase-3测试文档与-ci第-78-周)
9. [测试与验收方案](#9-测试与验收方案)
10. [风险、依赖与里程碑](#10-风险依赖与里程碑)
11. [附录](#11-附录)

---

## 1. 背景与结论

### 1.1 问题

RLinf 当前单臂真机栈走 **ROS + `serl_franka_controllers`**，文档要求固件 **`< 5.9.0`**。你的机器人固件为 **5.10.0**，无法降级到兼容版本，因此：

- 无法使用 `bash requirements/install.sh embodied --env franka`
- 无法使用 `switch_env franka-0.15.0` 等 ROS 虚拟环境
- 官方 Spacemouse / Pi0 SFT / RLPD / HG-DAgger / RLT 单臂示例均不可用

### 1.2 唯一可行路径

**`franka-franky` 栈**：`FrankyController` + libfranka（通过 `franky-control` wheel 内置）→ 直连机器人，不经过 ROS。

该路径已在 RLinf **双臂**场景验证（PR [#1139](https://github.com/RLinf/RLinf/pull/1139)），但**尚未接入单臂** `FrankaEnv-v1` 及上述工作流。

### 1.3 改造目标（交付定义）

| 编号 | 交付物 | 说明 |
|------|--------|------|
| D1 | `FrankyController` API 对齐 | 补齐 `move_arm`、`move_gripper`、`reconfigure_compliance_params` |
| D2 | Franka Hand libfranka 驱动 | 替换当前 `NotImplementedError` |
| D3 | `FrankyEnv-v1` + task env | 单臂 env 注册，支持 `controller_backend: franky` |
| D4 | 6 套 franky 变体 YAML | collect / SFT eval / RLPD / DAgger / RLT / RTC |
| D5 | 文档 + 验收 checklist | 固件 5.10.0 基准实机通过 L0–L3 |

---

## 2. 固件 5.10.0 版本锁定

依据 [Franka libfranka 兼容矩阵](https://frankarobotics.github.io/docs/doc/libfranka/docs/compatibility_matrix.html)：

| 项目 | 5.10.0 取值 |
|------|-------------|
| Robot System Version | **5.10.0**（属于 `>= 5.9.0` 区间） |
| 最低 libfranka | **>= 0.18.0** |
| Robot / Gripper Server | **10 / 3** |
| **RLinf 选用** | **libfranka 0.19.0** |
| **明确禁用** | **0.18.0**（阻抗控制 bug，见 [#1012](https://github.com/RLinf/RLinf/issues/1012)） |
| RLinf franky wheel 可选 | 0.15.0（**不适用于 5.10.0**）、**0.19.0**（唯一选择） |
| ROS / serl 路径 | **不可用** |

### 2.1 控制节点标准安装（第一步就做）

在**直连 Franka 的控制节点**上执行：

```bash
# 1. 克隆 RLinf（若尚未克隆）
git clone https://github.com/RLinf/RLinf.git
cd RLinf

# 2. 设置固件对应的 libfranka 版本（5.10.0 固定为 0.19.0）
export LIBFRANKA_VERSION=0.19.0
export REPO_PATH=$(pwd)

# 3. 安装 franky 环境（不装 ROS / serl_franka_controllers）
bash requirements/install.sh embodied --env franka-franky
source .venv/bin/activate

# 4. 验证 franky 已安装
python -c "import franky; print('franky OK')"
```

Docker 镜像内等价操作：

```bash
source switch_env franky-0.19.0
# 禁止：source switch_env franka-0.15.0  # 5.10.0 下会连接失败
```

### 2.2 实时内核与权限（第二步）

`FrankyController` 要求 PREEMPT_RT 与实时调度权限。在控制节点执行：

```bash
# 检查是否为 RT 内核
uname -r | grep rt

# 若未安装 RT 内核，按 Franka 官方文档安装：
# https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html

# 配置 rtprio 与 memlock（/etc/security/limits.d/99-rlinf-franka.conf）
# 内容示例：
#   <你的用户名>  -  rtprio  99
#   <你的用户名>  -  memlock unlimited

# 重新登录后验证
ulimit -r    # 应 >= 80
ulimit -l    # 应为 unlimited
```

每次启动 Ray 前，在控制节点执行网络与 CPU 调优（参考 `dual_franka.rst`）：

```bash
# 替换 <FRANKA_NIC> 为连接机器人的网卡名
sudo bash -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > "$g"
done'
sudo sysctl -w kernel.sched_rt_runtime_us=-1
sudo ethtool -C <FRANKA_NIC> rx-usecs 0 tx-usecs 0 2>/dev/null || true
```

### 2.3 Desk 侧上线前检查

1. 浏览器打开 `http://<ROBOT_IP>/desk`
2. **Settings → Dashboard**，确认 Control 版本为 **5.10.0**
3. 确认 FCI 已激活，无 safety violation
4. 在控制节点 ping 机器人：`ping -c 3 <ROBOT_IP>`

---

## 3. 现状与差距分析

### 3.1 两条控制栈对比

```
┌─────────────────────────────────────────────────────────────────┐
│  工作流 Runner（collect / SFT / RLPD / DAgger / RLT / RTC）      │
└────────────────────────────┬────────────────────────────────────┘
                             │ env.step(action)
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│  Env 层：FrankaEnv / PegInsertionEnv / FrankaBinRelocationEnv   │
│  + Wrapper：Spacemouse / GELLO / Pico / RelativeFrame / RLT键盘   │
└────────────────────────────┬────────────────────────────────────┘
                             │ controller.move_arm / get_state / ...
              ┌──────────────┴──────────────┐
              ▼                             ▼
   FrankaController (ROS)          FrankyController (libfranka)
   serl_franka_controllers         franky-control wheel
   固件 < 5.9.0 ✅                 固件 >= 5.9.0 ✅（5.10.0）
   Franka Hand via ROS topic ✅     Franka Hand ❌ 未实现
```

### 3.2 控制器 API 差距（必须补齐）

| API | `FrankaController`（ROS） | `FrankyController`（当前） | 单臂 env 是否依赖 |
|-----|---------------------------|----------------------------|-------------------|
| `move_arm(pose7)` | ✅ ROS equilibrium pose | ❌ 仅有 `move_tcp_pose` | **是** — `FrankaEnv.step` |
| `move_gripper(0-255)` | ✅ | ❌ 仅有 open/close | **是** — 连续夹爪控制 |
| `reconfigure_compliance_params` | ✅ dynamic_reconfigure | ❌ | **是** — reset 时切换 |
| `command_end_effector` | ✅ 灵巧手 | ❌ | 否（Phase 4） |
| `get_state()` | ✅ | ✅ | 是 |
| `reset_joint()` | ✅ | ✅ | 是 |
| Franka Hand | ✅ ROS `/franka_gripper/*` | ❌ `NotImplementedError` | **是**（你的硬件） |

### 3.3 现有可复用组件

| 组件 | 路径 | 复用方式 |
|------|------|----------|
| `FrankyController` 骨架 | `rlinf/envs/realworld/franka/franky_controller.py` | 扩展 API + Franka Hand |
| 双臂 env 参考 | `rlinf/envs/realworld/franka/dual_franka_env.py` | 参考 `_setup_hardware`、step pacing |
| 单臂 env 业务逻辑 | `rlinf/envs/realworld/franka/franka_env.py` | 复制 delta action、reward、相机、reset |
| Wrapper 栈 | `rlinf/envs/realworld/common/wrappers/apply.py` | `apply_single_arm_wrappers` 无需改 ROS |
| 安装脚本 | `requirements/install.sh` → `install_franka_franky_env` | 已默认 0.19.0 |
| Smoke test | `toolkits/realworld_check/test_franky_controller.py` | 扩展 Franka Hand 命令 |

---

## 4. 总体架构设计

### 4.1 设计原则

1. **最小侵入**：不重写 Runner / 算法层；只在 Env + Controller 层插入 franky 分支。
2. **API 对齐**：`FrankyController` 对外接口与 `FrankaController` 保持一致，使 `FrankaEnv` 逻辑可复用。
3. **配置驱动**：YAML 中 `controller_backend: franky` 切换栈；5.10.0 用户**强制**使用该值。
4. **向后兼容**：保留现有 ROS 路径与 YAML，供固件 `< 5.9.0` 用户继续使用。

### 4.2 目标代码结构

```
rlinf/envs/realworld/
├── franka/
│   ├── franka_controller.py      # ROS 路径（legacy，5.10.0 不用）
│   ├── franky_controller.py      # libfranka 路径（扩展 API + Franka Hand）
│   ├── franka_env.py             # ROS 单臂 env（legacy）
│   ├── franky_env.py             # 【新建】libfranka 单臂 env
│   ├── dual_franka_env.py        # 双臂 franky（已有）
│   └── tasks/
│       ├── __init__.py           # 注册 FrankyEnv-v1
│       ├── peg_insertion_env.py  # 增加 controller_backend 分支
│       └── ...
├── common/
│   └── gripper/
│       ├── franka_gripper.py           # ROS Franka Hand（legacy）
│       └── franka_libfranka_gripper.py # 【新建】libfranka Franka Hand
└── scheduler/hardware/robots/
    └── franka.py                 # FrankaConfig 增加 controller_backend
```

### 4.3 配置层结构

每个工作流保留原 YAML，**新增 `_franky` 后缀变体**，头部统一注释：

```yaml
# Target: Franka firmware 5.10.0, libfranka 0.19.0, Franka Hand
# Install: export LIBFRANKA_VERSION=0.19.0 && bash requirements/install.sh embodied --env franka-franky
# Do NOT use: switch_env franka-0.15.0, serl_franka_controllers
```

---

## 5. Phase 0：控制器与 Franka Hand（第 1–2 周）

> **Phase 0 完成标准：** 在固件 5.10.0 实机上，`test_franky_controller.py` 能 home / TCP nudge / Franka Hand 开闭。

### 5.1 任务 A：实现 Franka Hand libfranka 驱动

#### 步骤 A1 — 调研 franky 夹爪 API

在已激活的 venv 中：

```bash
python - <<'PY'
import franky
# 列出 Robot 上与 gripper 相关的属性和方法
robot_cls = franky.Robot
print([x for x in dir(robot_cls) if "grip" in x.lower()])
PY
```

> **说明：** [Brunch-Life/franky](https://github.com/Brunch-Life/franky) fork 自 [TimSchneider42/franky](https://github.com/TimSchneider42/franky)，RLinf 使用其 `enhance-impedance-control` 分支（`install.sh` 安装的 `franky-control` wheel 亦来自该 fork，而非 upstream 原版）。

同时查阅 Brunch-Life/franky 仓库 README 与 examples，确认是否暴露：

- `robot.gripper` 或独立 `Gripper(robot_ip)` 对象
- `move(width)` / `grasp()` / `get_state()` 等接口

#### 步骤 A2 — 新建 gripper 模块

**新建文件：** `rlinf/envs/realworld/common/gripper/franka_libfranka_gripper.py`

实现 `BaseGripper` 接口，对齐 ROS 版 `franka_gripper.py` 的语义：

```python
class FrankaLibfrankaGripper(BaseGripper):
    """Franka Hand via libfranka / franky (no ROS)."""

    def open(self, speed: float = 1.0) -> None: ...
    def close(self, speed: float = 1.0) -> None: ...
    def move(self, width: float, speed: float = 0.1) -> None: ...
    @property
    def position(self) -> float: ...      # 0=closed, 1=open，与 FrankaRobotState 对齐
    def is_open(self) -> bool: ...
    def is_ready(self) -> bool: ...
    def cleanup(self) -> None: ...
```

**宽度映射参考（与 ROS 版一致）：**

- 全开：约 `0.08 m`
- 全闭：`0.0 m`
- `move_gripper(0-255)`：`width = (255 - pos) / 255 * MAX_WIDTH`

#### 步骤 A3 — 修改 `FrankyController._build_gripper`

**文件：** `rlinf/envs/realworld/franka/franky_controller.py`

将当前：

```python
if gt == "franka":
    raise NotImplementedError(...)
```

改为：

```python
if gt == "franka":
    from rlinf.envs.realworld.common.gripper.franka_libfranka_gripper import (
        FrankaLibfrankaGripper,
    )
    return FrankaLibfrankaGripper(robot=self._robot)
```

#### 步骤 A4 — 实机验证 Franka Hand

```bash
export FRANKA_ROBOT_IP=<你的机器人IP>
export FRANKA_GRIPPER_TYPE=franka   # 注意：不是 robotiq

python -m toolkits.realworld_check.test_franky_controller
# 依次测试：home → open → close → getpos
```

**验收：** 夹爪开合正常，`get_state().gripper_open` 与物理状态一致。

---

### 5.2 任务 B：补齐 `FrankyController` API

#### 步骤 B1 — 添加 `move_arm` 别名

在 `franky_controller.py` 中添加：

```python
def move_arm(self, position: np.ndarray) -> None:
    """Alias for move_tcp_pose; matches FrankaController signature."""
    self.move_tcp_pose(np.asarray(position, dtype=np.float64))
```

#### 步骤 B2 — 添加 `move_gripper`

```python
def move_gripper(self, position: int, speed: float = 0.3) -> None:
    """Map 0-255 to gripper width; matches FrankaController."""
    assert 0 <= position <= 255
    self._gripper.move(width=self._pos_to_width(position), speed=speed)
```

#### 步骤 B3 — 添加 `reconfigure_compliance_params`

将 ROS 版 compliance dict 映射到 Cartesian tracker 参数或环境变量：

| ROS compliance key | franky 映射 |
|--------------------|-------------|
| `trans_stiffness` | `RLINF_CART_K_T` 或 tracker 重建 |
| `rot_stiffness` | `RLINF_CART_K_R` |
| `trans_damping` | tracker damping（若 API 支持） |

实现策略（推荐）：

1. 收到 `reconfigure_compliance_params(params)` 时，停止当前 cart tracker；
2. 用新刚度/阻尼重建 `CartesianImpedanceTracker`；
3. 与 `FrankaEnv.reset()` 中 precision/compliance 切换语义对齐。

#### 步骤 B4 — 修复 smoke test

**文件：** `toolkits/realworld_check/test_franky_controller.py`

- 删除或替换不存在的 `grip` / `impedance` 命令（当前会报错）；
- 增加 `open` / `close` / `getpos` / `getpos_euler` 对 Franka Hand 的测试；
- 默认 `FRANKA_GRIPPER_TYPE=franka`。

---

### 5.3 任务 C：固件检查工具（可选但建议）

**新建：** `toolkits/realworld_check/check_franka_firmware.py`

逻辑：

```
输入：用户提供的固件版本字符串（或从 Desk API 读取）
输出：
  - firmware 5.10.0 → LIBFRANKA_VERSION=0.19.0, backend=franky, wheel URL
  - firmware < 5.9.0  → 可选 ros (0.15.0) 或 franky
  - firmware >= 5.9.0 → 强制 franky, 禁止 0.15.0/0.18.0
```

用法：

```bash
python -m toolkits.realworld_check.check_franka_firmware --firmware 5.10.0
```

---

## 6. Phase 1：单臂 FrankyEnv（第 2–4 周）

> **Phase 1 完成标准：** `FrankyEnv-v1` dummy unit test 通过；实机 10 Hz step 返回正确 obs。

### 6.1 任务 D：新建 `FrankyEnv`

#### 步骤 D1 — 创建 `franky_env.py`

**新建文件：** `rlinf/envs/realworld/franka/franky_env.py`

**做法（推荐，改动最小）：**

1. 以 `franka_env.py` 为模板复制；
2. 类名改为 `FrankyEnv`，config 类名 `FrankyRobotConfig`（或直接复用 `FrankaRobotConfig`）；
3. 仅修改 `_setup_hardware()`：

```python
def _setup_hardware(self):
    from .franky_controller import FrankyController
    # ... 与 FrankaEnv 相同的 hardware_info 解析 ...
    self._controller = FrankyController.launch_controller(
        robot_ip=self.config.robot_ip,
        env_idx=self.env_idx,
        node_rank=controller_node_rank,
        worker_rank=self.env_worker_rank,
        gripper_type="franka",  # Franka Hand
        gripper_connection=None,
    )
```

4. 确认 `step()` / `reset()` / `_execute_action()` 中所有 controller 调用均使用已对齐的 API（`move_arm`、`move_gripper`、`reconfigure_compliance_params`）。

**不要改动的部分（直接从 FrankaEnv 保留）：**

- delta action 合成逻辑
- `ee_pose_limit_min/max` 安全盒
- 相机初始化与 `_open_cameras()`
- reward 计算（pose threshold / dense reward）
- `use_reward_model` + `EmbodiedRewardWorker`（与 controller 无关）

#### 步骤 D2 — 注册 Gym 环境

**文件：** `rlinf/envs/realworld/franka/tasks/__init__.py`

添加：

```python
def create_franky_env(...):
    from rlinf.envs.realworld.franka.franky_env import FrankyEnv
    env = FrankyEnv(...)
    return apply_single_arm_wrappers(env, env_cfg)

register(
    id="FrankyEnv-v1",
    entry_point="rlinf.envs.realworld.franka.tasks:create_franky_env",
)
```

#### 步骤 D3 — 扩展 task env（PegInsertion 等）

以 `PegInsertionEnv` 为例，两种实现方式（二选一）：

**方式 1（推荐）：** 在 `FrankaEnv` 基类增加 controller 工厂

```python
def _launch_controller(self, ...):
    backend = getattr(self.config, "controller_backend", "ros")
    if backend == "franky":
        from .franky_controller import FrankyController
        return FrankyController.launch_controller(...)
    from .franka_controller import FrankaController
    return FrankaController.launch_controller(...)
```

**方式 2：** 为每个 task 建 `PegInsertionFrankyEnv(FrankaEnv)` 子类，仅 override `_setup_hardware`。

#### 步骤 D4 — 扩展硬件配置

**文件：** `rlinf/scheduler/hardware/robots/franka.py`

在 `FrankaConfig` 中增加字段：

```python
controller_backend: str = "ros"       # "ros" | "franky"
libfranka_version: str = "0.19.0"     # 仅 franky 有效
controller_node_rank: Optional[int] = None
```

校验逻辑：若检测到用户环境变量或配置声明 `firmware >= 5.9.0`，强制 `controller_backend=franky`。

#### 步骤 D5 — 编写 dummy unit test

**新建：** `tests/unit_tests/test_franky_env_dummy.py`

```python
def test_franky_env_obs_space():
    env = create_franky_env(cfg_with_is_dummy=True)
    obs, _ = env.reset()
    assert "images" in obs or "wrist_0_rgb" in obs  # 按实际 key 断言
    action = env.action_space.sample()
    obs, reward, term, trunc, info = env.step(action)
```

运行：

```bash
pytest tests/unit_tests/test_franky_env_dummy.py -v
```

---

## 7. Phase 2：各工作流打通（第 4–7 周）

> **Phase 2 完成标准：** 6 个工作流各至少完成 1 次端到端 smoke（实机或 dummy→实机）。

### 7.0 通用前置步骤（每个工作流都做）

#### 步骤 0.1 — 启动 Ray 集群

在**控制节点**（直连 Franka 的机器）：

```bash
export RLINF_NODE_RANK=0          # 单节点时 rank=0
export ROBOT_IP=<你的机器人IP>
source .venv/bin/activate
# 若用 Docker：source switch_env franky-0.19.0

ray start --head --port=6379 --node-ip-address=<控制节点IP>
```

在 **GPU 节点**（若双节点布局）：

```bash
export RLINF_NODE_RANK=1
ray start --address=<控制节点IP>:6379
```

验证：

```bash
python -m ray_utils.check_ray
```

#### 步骤 0.2 — 公共 env 配置片段

所有 franky YAML 的 `cluster.node_groups` 中增加：

```yaml
hardware:
  type: Franka
  configs:
    - robot_ip: <ROBOT_IP>
      node_rank: 0
      controller_node_rank: 0
      controller_backend: franky
      libfranka_version: "0.19.0"
      gripper_type: franka
      disable_validate: false
```

env 层增加：

```yaml
override_cfg:
  controller_backend: franky
  end_effector_type: franka_gripper
  is_dummy: false
```

---

### 7.1 工作流 1：数据采集（collect_data）

#### 参考配置

- `examples/embodiment/config/realworld_collect_data.yaml`

#### 步骤（逐步执行）

**步骤 1.1 — 复制并修改 YAML**

```bash
cp examples/embodiment/config/realworld_collect_data.yaml \
   examples/embodiment/config/realworld_collect_data_franky.yaml
```

修改要点：

| 字段 | 原值（ROS） | franky 变体 |
|------|-------------|-------------|
| env eval `init_params.id` | `PegInsertionEnv-v1` | 同左（task env 内部走 franky backend） |
| `override_cfg.controller_backend` | （无） | `franky` |
| `use_spacemouse` | `True` | `True`（wrapper 不依赖 ROS） |
| hardware `gripper_type` | 可能为 robotiq | **`franka`** |

**步骤 1.2 — 确认 Spacemouse**

```bash
python -m rlinf.envs.realworld.common.spacemouse.spacemouse_expert
# 移动 Spacemouse，终端应输出 delta
```

**步骤 1.3 — 采集**

```bash
export EMBODIED_PATH=examples/embodiment
export MUJOCO_GL=egl
bash examples/embodiment/collect_data.sh realworld_collect_data_franky
```

**步骤 1.4 — 检查输出**

```bash
ls ${runner.logger.log_path}/collected_data/
# 应有 episode 文件；检查 obs 含图像、action 维度为 7
```

**验收：** ≥10 个成功 episode；pickle/LeRobot 字段与 ROS 路径一致。

---

### 7.2 工作流 2：Pi0 SFT 部署

#### 参考配置

- SFT 训练：`examples/sft/config/realworld_sft_openpi.yaml`
- 部署评测：`evaluations/realworld/realworld_eval.yaml`
- Env 模板：`examples/embodiment/config/env/realworld_franka_sft_env.yaml`

#### 步骤（逐步执行）

**步骤 2.1 — SFT 训练（与 controller 无关，可在 GPU 节点完成）**

```bash
# 使用已采集的数据
bash examples/sft/run_sft.sh realworld_sft_openpi
```

**步骤 2.2 — 计算 norm stats（若尚未做）**

按 `franka_pi0_sft_deploy.rst` 文档，在 OpenPI checkpoint 目录生成 `norm_stats.json`。

**步骤 2.3 — 复制 eval 配置**

```bash
cp evaluations/realworld/realworld_eval.yaml \
   evaluations/realworld/realworld_eval_franky.yaml
```

修改要点：

```yaml
defaults:
  - env/realworld_franka_sft_env@env.eval   # 见步骤 2.4 修改 env 模板

env:
  eval:
    init_params:
      id: "FrankyEnv-v1"                   # 或 FrankaEnv-v1 + controller_backend: franky
    override_cfg:
      controller_backend: franky
      task_description: "your task"
      target_ee_pose: [0.5, 0.0, 0.1, -3.14, 0.0, 0.0]
      action_scale: [1.0, 1.0, 1.0]
    use_spacemouse: false

rollout:
  model:
    model_path: <你的 SFT checkpoint 路径>
```

**步骤 2.4 — 修改 env 模板（或新建 franky 专用模板）**

```bash
cp examples/embodiment/config/env/realworld_franka_sft_env.yaml \
   examples/embodiment/config/env/realworld_franka_sft_env_franky.yaml
```

将 `init_params.id` 改为 `FrankyEnv-v1`，并增加 `controller_backend: franky`。

**步骤 2.5 — 运行部署**

```bash
bash evaluations/run_eval.sh realworld_eval_franky
```

**验收：** 策略控制机械臂运动 ≥100 step 无 exception；任务成功率可记录。

---

### 7.3 工作流 3：RLPD（异步 SAC + demo buffer）

#### 参考配置

- `examples/embodiment/config/realworld_peginsertion_rlpd_cnn_async.yaml`

#### 步骤（逐步执行）

**步骤 3.1 — 复制 YAML**

```bash
cp examples/embodiment/config/realworld_peginsertion_rlpd_cnn_async.yaml \
   examples/embodiment/config/realworld_peginsertion_rlpd_franky_async.yaml
```

**步骤 3.2 — 修改关键字段**

```yaml
algorithm:
  demo_buffer:
    load_path: <离线 demo 路径>    # 需先用 franky collect_data 采集

env:
  train:
    override_cfg:
      controller_backend: franky
  eval:
    override_cfg:
      controller_backend: franky
    use_spacemouse: true           # 在线干预采集

cluster:
  node_groups:
    - label: franka
      hardware:
        configs:
          - controller_backend: franky
            gripper_type: franka
```

**步骤 3.3 — 启动训练**

```bash
export EMBODIED_PATH=examples/embodiment
python examples/embodiment/train_async.py \
  --config-name realworld_peginsertion_rlpd_franky_async
```

**验收：** ≥100 env steps；actor loss 无 NaN；demo buffer 正常加载。

---

### 7.4 工作流 4：HG-DAgger

#### 参考配置

- `examples/embodiment/config/realworld_pnp_dagger_openpi.yaml`

#### 步骤（逐步执行）

**步骤 4.1 — 复制 YAML**

```bash
cp examples/embodiment/config/realworld_pnp_dagger_openpi.yaml \
   examples/embodiment/config/realworld_pnp_dagger_openpi_franky.yaml
```

**步骤 4.2 — 修改**

```yaml
algorithm:
  dagger:
    only_save_expert: true

env:
  train:
    use_spacemouse: true
    override_cfg:
      controller_backend: franky
  eval:
    use_spacemouse: true
    override_cfg:
      controller_backend: franky
```

**步骤 4.3 — 启动**

```bash
python examples/embodiment/train_embodied_agent.py \
  --config-name realworld_pnp_dagger_openpi_franky
```

**验收：** 干预步写入 buffer；`only_save_expert` 数据可用于后续训练。

---

### 7.5 工作流 5：RLT / RL Token（Stage 2）

#### 参考配置

- Stage 1 SFT：`examples/sft/config/realworld_rlt_stage1_sft_openpi_pi05.yaml`
- Stage 2 RL：`examples/embodiment/config/realworld_rlt_stage2_ac_mlp.yaml`

#### 背景

RLT 在 **rollout 层**加载冻结的 Stage1 模型（π₀.₅ + `RLTTokenTransformer`）提取 `z_rl` token；**env/controller 接口不变**。

#### 步骤（逐步执行）

**步骤 5.1 — 完成 Stage 1 SFT**

按 `rlt.rst` 文档训练 Stage1，得到 checkpoint。

**步骤 5.2 — 复制 Stage 2 配置**

```bash
cp examples/embodiment/config/realworld_rlt_stage2_ac_mlp.yaml \
   examples/embodiment/config/realworld_rlt_stage2_ac_mlp_franky.yaml
```

**步骤 5.3 — 修改 env 部分**

```yaml
env:
  train:
    keyboard_reward_wrapper: rlt_policy_switch
    use_spacemouse: true
    override_cfg:
      controller_backend: franky
  eval:
    keyboard_reward_wrapper: rlt_policy_switch
    override_cfg:
      controller_backend: franky

rollout:
  rlt_feature_model:
    model_path: <Stage1 checkpoint>
```

**步骤 5.4 — 启动 Stage 2**

```bash
python examples/embodiment/train_embodied_agent.py \
  --config-name realworld_rlt_stage2_ac_mlp_franky
```

**验收：**

- `rlt_feature_model` 正常加载；
- `algorithm.loss_type: rlt_ac` 训练 loss 有限；
- 键盘切换策略/参考动作有效。

---

### 7.6 工作流 6：RTC（Real-Time Chunking）

#### 参考配置

- `evaluations/realworld/realworld_pnp_eval_pi05_sft_RTC.yaml`

#### 步骤（逐步执行）

**步骤 6.1 — 复制 YAML**

```bash
cp evaluations/realworld/realworld_pnp_eval_pi05_sft_RTC.yaml \
   evaluations/realworld/realworld_pnp_eval_pi05_sft_RTC_franky.yaml
```

**步骤 6.2 — 修改**

```yaml
runner:
  rtc:
    enabled: true
    inject_delay_ms: <按文档>
    min_exec_horizon: <按文档>

rollout:
  model:
    openpi:
      rtc_enabled: ${runner.rtc.enabled}

env:
  eval:
    override_cfg:
      controller_backend: franky
```

**步骤 6.3 — 运行**

```bash
bash evaluations/run_eval.sh realworld_pnp_eval_pi05_sft_RTC_franky
```

**验收：** chunk 重叠执行；`time/rollout` 延迟指标正常；无 control timeout。

---

## 8. Phase 3：测试、文档与 CI（第 7–8 周）

### 8.1 单元测试清单

| 测试文件 | 命令 | 通过标准 |
|----------|------|----------|
| `test_franky_env_dummy.py` | `pytest tests/unit_tests/test_franky_env_dummy.py` | obs/action space 正确 |
| 既有 scheduler 测试 | `pytest tests/unit_tests/ -k franka` | 无回归 |

### 8.2 实机 smoke 脚本顺序

在固件 **5.10.0** 实机上，按顺序执行：

```bash
# 1. 控制器 smoke
export FRANKA_ROBOT_IP=<IP>
export FRANKA_GRIPPER_TYPE=franka
python -m toolkits.realworld_check.test_franky_controller

# 2. 相机 smoke
python -m toolkits.realworld_check.test_franka_camera

# 3. Spacemouse smoke（若用 Spacemouse 采集）
python -m rlinf.envs.realworld.common.spacemouse.spacemouse_expert

# 4. Dummy env
pytest tests/unit_tests/test_franky_env_dummy.py -v

# 5. 单步实机 env（写临时脚本或用 collect_data 跑 1 episode）
bash examples/embodiment/collect_data.sh realworld_collect_data_franky
```

### 8.3 文档更新

| 文档 | 修改内容 |
|------|----------|
| `docs/source-en/rst_source/examples/embodied/franka.rst` | 增加「Firmware >= 5.9.0 / 5.10.0 单臂 franky 路径」章节 |
| `docs/source-zh/rst_source/examples/embodied/franka.rst` | 中文版同步 |
| `docs/source-en/rst_source/examples/embodied/franka_pi0_sft_deploy.rst` | franky 变体命令 |
| `AGENTS.md` | 注明单臂 franky 与固件要求 |

### 8.4 负向测试（必做）

验证版本锁定文档正确：

```bash
# 应连接失败或报版本不兼容
export LIBFRANKA_VERSION=0.15.0
bash requirements/install.sh embodied --env franka-franky --venv /tmp/franky-wrong
source /tmp/franky-wrong/bin/activate
python -m toolkits.realworld_check.test_franky_controller  # 预期失败
```

---

## 9. 测试与验收方案

### 9.1 L0：安装与连通（必须通过）

- [ ] Desk 显示 Control **5.10.0**，Robot/Gripper Server **10/3**
- [ ] `export LIBFRANKA_VERSION=0.19.0 && bash requirements/install.sh embodied --env franka-franky`
- [ ] PREEMPT_RT 内核 + `ulimit -r >= 80` + `ulimit -l unlimited`
- [ ] `python -c "import franky; print(franky)"` 成功
- [ ] `test_franky_controller`：home / nudge / **Franka Hand open-close**
- [ ] 负向：`LIBFRANKA_VERSION=0.15.0` 连接失败

### 9.2 L1：单臂 Env 闭环（必须通过）

- [ ] `FrankyEnv-v1` dummy unit test 通过
- [ ] 实机 `is_dummy=False`：10 Hz step，obs 含 TCP + 相机 + gripper
- [ ] reset：关节复位 + compliance 切换无 exception
- [ ] `get_tcp_pose()` 与 Desk 显示大致一致（误差 < 1 cm / 5°）

### 9.3 L2：工作流验收

| 工作流 | 命令 | 通过标准 |
|--------|------|----------|
| collect_data | `collect_data.sh realworld_collect_data_franky` | ≥10 成功 episode |
| Pi0 SFT deploy | `run_eval.sh realworld_eval_franky` | ≥100 step 无 exception |
| RLPD | `train_async.py --config-name realworld_peginsertion_rlpd_franky_async` | ≥100 env steps，loss 有限 |
| HG-DAgger | `train_embodied_agent.py --config-name realworld_pnp_dagger_openpi_franky` | 干预数据可写入 |
| RLT Stage2 | `train_embodied_agent.py --config-name realworld_rlt_stage2_ac_mlp_franky` | `rlt_ac` loss 有限，键盘切换有效 |
| RTC | `run_eval.sh realworld_pnp_eval_pi05_sft_RTC_franky` | chunk 重叠正常 |

### 9.4 L3：稳定性回归（必须通过）

- [ ] **主环境：5.10.0 + libfranka 0.19.0 + Franka Hand**
- [ ] 连续 30 min Spacemouse 或 policy rollout：
  - 无 `power_limit_violation`
  - 无「臂不动但夹爪动」（#1012 类问题）
  - 无 libfranka UDP timeout
- [ ] 记录 cyclictest max latency（建议 < 150 µs，参考 PR #1139）

---

## 10. 风险、依赖与里程碑

### 10.1 风险

| 风险 | 影响 | 缓解 |
|------|------|------|
| 5.10.0 无 ROS 退路 | 改造是必选项 | 优先 Phase 0，不做 ros 适配 |
| Franka Hand franky API 不明确 | Phase 0 阻塞 | 调研 Brunch-Life/franky；必要时 fork wheel |
| libfranka 0.18 阻抗 bug | 臂不动 | **固定 0.19.0**，禁止 0.18 |
| wheel 仅 0.15/0.19 | 未来固件可能需更新 wheel | 跟踪 Brunch-Life releases |
| DexHand 不支持 | 灵巧手任务不可用 | 文档标注 Phase 4 |
| 单进程 libfranka 独占 | smoke test 与 env 不能同时连 | 文档强调「一次只有一个 client」 |

### 10.2 外部依赖

- [Brunch-Life/franky](https://github.com/Brunch-Life/franky) wheels（libfranka 0.19.0）
- [Franka 兼容矩阵](https://frankarobotics.github.io/docs/compatibility.html)
- RLinf PR [#1139](https://github.com/RLinf/RLinf/pull/1139)（FrankyController 双臂参考实现）

### 10.3 里程碑时间表

| 阶段 | 时间 | 交付 |
|------|------|------|
| Phase 0 | 第 1–2 周 | Franka Hand + API parity + smoke test |
| Phase 1 | 第 2–4 周 | `FrankyEnv-v1` + unit test + 实机 step |
| Phase 2 | 第 4–7 周 | 6 个工作流 franky YAML + 端到端 smoke |
| Phase 3 | 第 7–8 周 | 文档 + L3 稳定性 + 可选 PR  upstream |

---

## 11. 附录

### 11.1 新建/修改文件清单

| 操作 | 路径 |
|------|------|
| **新建** | `rlinf/envs/realworld/common/gripper/franka_libfranka_gripper.py` |
| **新建** | `rlinf/envs/realworld/franka/franky_env.py` |
| **新建** | `toolkits/realworld_check/check_franka_firmware.py` |
| **新建** | `tests/unit_tests/test_franky_env_dummy.py` |
| **新建** | `examples/embodiment/config/realworld_collect_data_franky.yaml` |
| **新建** | `examples/embodiment/config/realworld_peginsertion_rlpd_franky_async.yaml` |
| **新建** | `examples/embodiment/config/realworld_pnp_dagger_openpi_franky.yaml` |
| **新建** | `examples/embodiment/config/realworld_rlt_stage2_ac_mlp_franky.yaml` |
| **新建** | `examples/embodiment/config/env/realworld_franka_sft_env_franky.yaml` |
| **新建** | `evaluations/realworld/realworld_eval_franky.yaml` |
| **新建** | `evaluations/realworld/realworld_pnp_eval_pi05_sft_RTC_franky.yaml` |
| **修改** | `rlinf/envs/realworld/franka/franky_controller.py` |
| **修改** | `rlinf/envs/realworld/franka/tasks/__init__.py` |
| **修改** | `rlinf/scheduler/hardware/robots/franka.py` |
| **修改** | `toolkits/realworld_check/test_franky_controller.py` |
| **修改** | `docs/source-en/rst_source/examples/embodied/franka.rst`（及中文） |

### 11.2 控制器 API 对齐对照表

| `FrankaController` | `FrankyController`（改造后） |
|--------------------|------------------------------|
| `move_arm(pose7)` | `move_arm` → `move_tcp_pose` |
| `move_gripper(pos, speed)` | `move_gripper` → gripper `move(width)` |
| `reconfigure_compliance_params(dict)` | 重建 CartesianImpedanceTracker |
| `open_gripper()` | `open_gripper()` |
| `close_gripper()` | `close_gripper()` |
| `get_state()` | `get_state()` |
| `reset_joint(qpos)` | `reset_joint(qpos)` |
| `clear_errors()` | `clear_errors()` |
| `command_end_effector(action)` | Phase 4（DexHand） |

### 11.3 工作流配置映射总表

| 工作流 | 原配置 | franky 变体 | Env ID | 关键 override |
|--------|--------|-------------|--------|---------------|
| collect_data | `realworld_collect_data.yaml` | `realworld_collect_data_franky.yaml` | PegInsertion | `controller_backend: franky` |
| Pi0 SFT deploy | `realworld_eval.yaml` | `realworld_eval_franky.yaml` | FrankyEnv-v1 | `use_spacemouse: false` |
| RLPD | `realworld_peginsertion_rlpd_cnn_async.yaml` | `realworld_peginsertion_rlpd_franky_async.yaml` | PegInsertion | demo_buffer path |
| HG-DAgger | `realworld_pnp_dagger_openpi.yaml` | `realworld_pnp_dagger_openpi_franky.yaml` | BinRelocation | `only_save_expert: true` |
| RLT Stage2 | `realworld_rlt_stage2_ac_mlp.yaml` | `realworld_rlt_stage2_ac_mlp_franky.yaml` | PegInsertion | `rlt_feature_model` |
| RTC | `realworld_pnp_eval_pi05_sft_RTC.yaml` | `realworld_pnp_eval_pi05_sft_RTC_franky.yaml` | BinRelocation | `runner.rtc.enabled: true` |

### 11.4 常见问题

**Q: 能否在 5.10.0 上继续用 ROS 路径？**  
A: 不能。`serl_franka_controllers` + libfranka 0.15 仅支持固件 `< 5.9.0`。

**Q: 能否用 libfranka 0.18.0？**  
A: 不推荐。满足最低兼容但有阻抗 bug（[#1012](https://github.com/RLinf/RLinf/issues/1012)）。请用 **0.19.0**。

**Q: 双臂 franky 代码能否直接用于单臂？**  
A: `FrankyController` 本身是单臂设计；缺的是 `FrankyEnv` 和工作流 YAML。参考 `dual_franka_env.py` 的 `_setup_hardware`，但动作语义应复用 `franka_env.py` 的 delta TCP 逻辑。

**Q: 运行 test_franky_controller 时报 UDP timeout？**  
A: 检查：机器人 FCI 是否激活、防火墙、网线直连、是否有其他 libfranka client 占用连接。

---

# 其它说明

## Franky是什么
**`franky`** 是一个 **Python 库**，用来在 Python 里直接控制 Franka 机械臂，底层走 **libfranka（Franka 官方 C++ 接口）**，**不经过 ROS**。

### 在 RLinf 里的角色

文档第 85 行的：

```bash
python -c "import franky; print('franky OK')"
```

是在检查：`franka-franky` 环境是否装成功，能否 `import franky`。

| 概念 | 说明 |
|------|------|
| **包名（pip）** | `franky-control`（wheel 里 import 名是 `franky`） |
| **来源** | [Brunch-Life/franky](https://github.com/Brunch-Life/franky) fork，按 libfranka 版本发预编译 wheel |
| **安装方式** | `bash requirements/install.sh embodied --env franka-franky` |
| **默认 libfranka** | **0.19.0**（对应你固件 5.10.0） |
| **代码里谁用它** | `FrankyController`（`rlinf/envs/realworld/franka/franky_controller.py`） |

安装脚本里的说明：

```2368:2382:requirements/install.sh
install_franka_franky_env() {
    # Prebuilt franky-control wheel (libfranka bundled), published per
    # libfranka version by the Brunch-Life/franky fork.  LIBFRANKA_VERSION
    # must match your Franka firmware ...
    local LIBFRANKA_VERSION="${LIBFRANKA_VERSION:-0.19.0}"
    ...
    uv pip install --reinstall-package franky-control --no-deps "$FRANKY_WHEEL"
```

### 和 ROS 路径的区别

```
单臂 ROS 路径（旧，固件 < 5.9.0）:
  Python → FrankaController → ROS Noetic → serl_franka_controllers → libfranka → 机器人

franky 路径（新，固件 >= 5.9.0，含 5.10.0）:
  Python → FrankyController → import franky → libfranka（wheel 内置）→ 机器人
```

`FrankyController` 里典型用法：

```python
import franky
robot = franky.Robot(robot_ip)           # 连接机械臂
robot.move(franky.JointMotion(...))      # 关节运动
# 还有 CartesianImpedanceTracker 等阻抗控制接口
```

### 名字容易混淆的地方

- **`franky`**：Python 库（`import franky`）
- **`franka-franky`**：RLinf 的安装 target（`--env franka-franky`），会装 franky + 相关依赖
- **`FrankyController`**：RLinf 里基于 franky 的 Ray Worker 控制器类
- **`franka`**（另一个 env）：装的是 ROS + `serl_franka_controllers`，**不是** franky

### 对你（5.10.0）的意义

固件 5.10.0 不能用 ROS/`serl` 那条路，**必须**用 franky 这条 libfranka 直连路径。第 85 行就是在确认这条链路的 Python 层是否就绪；通过后还要用 `test_franky_controller.py` 在实机上验证能否连上机器人。

## Franka 的 docker 镜像

这里说的 Docker 镜像，指的是 RLinf 官方提供的 **Franka 真机专用镜像**：

### 镜像名称

```bash
rlinf/rlinf:agentic-rlinf0.4-franka
```

国内可用镜像加速：

```bash
docker.1ms.run/rlinf/rlinf:agentic-rlinf0.4-franka
```

文档里的启动示例（[`docs/source-zh/rst_source/examples/embodied/franka.rst`](docs/source-zh/rst_source/examples/embodied/franka.rst)）：

```bash
docker run -it --rm \
   --privileged \
   --network host \
   --name rlinf \
   -v .:/workspace/RLinf \
   rlinf/rlinf:agentic-rlinf0.4-franka
```

对应 Dockerfile 构建目标：`embodied-franka-image`（[`docker/Dockerfile`](docker/Dockerfile) 第 349 行起）。

---

### 镜像里有什么

该镜像基于 **Ubuntu 20.04**，预装了多套 Python 虚拟环境：

| 类型 | venv 名称 | 用途 |
|------|-----------|------|
| ROS 路径 | `franka-0.10.0` … `franka-0.19.0` | ROS + `serl_franka_controllers`，固件 **< 5.9.0** |
| **franky 路径** | **`franky-0.15.0`、`franky-0.19.0`** | libfranka + franky，固件 **≥ 5.9.0** |
| 其他 | `franka-dexhand` | 灵巧手相关依赖 |

切换环境用镜像内置脚本：

```bash
source switch_env <env_name>
```

---

### 对你（固件 5.10.0）应选哪个

文档第 88 行的意思是：在容器里不要用默认环境，而要执行：

```bash
source switch_env franky-0.19.0
```

原因：

- 镜像默认 `.bashrc` 激活的是 **`franka-0.15.0`**（ROS 路径，面向旧固件）
- 5.10.0 必须走 **franky + libfranka 0.19.0**
- **不要**用 `source switch_env franka-0.15.0` 或任意 `franka-*` ROS 环境

---

### 和宿主机安装的关系

| 方式 | 命令 |
|------|------|
| 宿主机安装 | `bash requirements/install.sh embodied --env franka-franky` → 激活 `.venv` |
| Docker 内 | 进容器后 `source switch_env franky-0.19.0` |

两者装的是同一类东西（`franky-control` wheel），只是路径不同。

---

**注意：** 这不是 `frankasim` 仿真镜像（`agentic-rlinf0.4-frankasim`），而是连真机 Franka 的控制节点镜像。真机场景通常加 `--privileged --network host`，以便访问机器人和相机设备。