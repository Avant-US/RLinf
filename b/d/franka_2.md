# 单臂 Franka franky 改造方案（固件 5.10.0）—— 修订版

> **目标环境：** Franka Emika Panda · 固件 **5.10.0** · libfranka **0.19.0** · Franka Hand（原生夹爪）
> **目标能力：** 在 RLinf 上跑通单臂真机全流程——数据采集、Pi0 SFT 部署、RLPD、HG-DAgger、RLT、RTC
> **关联 Issue：** [RLinf#1477](https://github.com/RLinf/RLinf/issues/1477)
> **修订说明：** 本方案基于对代码库的逐行分析重写，修正了旧方案（`franka_1.md`）中的架构错误和事实偏差

---

## 目录

1. [旧方案问题总结](#1-旧方案问题总结)
2. [固件 5.10.0 版本锁定与环境准备](#2-固件-5100-版本锁定与环境准备)
3. [现状分析（基于代码实查）](#3-现状分析基于代码实查)
4. [总体架构设计](#4-总体架构设计)
5. [Phase 0：Franka Hand 驱动（第 1 周）](#5-phase-0franka-hand-驱动第-1-周)
6. [Phase 1：FrankyController API 补齐（第 1–2 周）](#6-phase-1frankycontroller-api-补齐第-12-周)
7. [Phase 2：控制器工厂接入 FrankaEnv（第 2–3 周）](#7-phase-2控制器工厂接入-frankaenv第-23-周)
8. [Phase 3：Smoke Test 修复与扩展（第 3 周）](#8-phase-3smoke-test-修复与扩展第-3-周)
9. [Phase 4：各工作流 YAML 与端到端验证（第 3–5 周）](#9-phase-4各工作流-yaml-与端到端验证第-35-周)
10. [Phase 5：CI / Docker / 文档（第 5–6 周）](#10-phase-5ci--docker--文档第-56-周)
11. [测试与验收方案](#11-测试与验收方案)
12. [风险与里程碑](#12-风险与里程碑)
13. [附录](#13-附录)

---

## 1. 旧方案问题总结

以下是对 `franka_1.md` 逐条审查后发现的主要问题。

### 1.1 架构级错误：提议复制整个 FrankaEnv

旧方案（6.1 节）主要推荐"复制 `franka_env.py` 为 `franky_env.py`，类名改为 `FrankyEnv`"。这是最大的架构失误：

- `franka_env.py` 共 **933 行**，包含 delta action 合成、安全盒裁剪、相机管理、reward 计算、reward model、灵巧手支持等业务逻辑——全部与 controller 无关。
- 复制后，每个 task env（`PegInsertionEnv`、`FrankaBinRelocationEnv`、`BottleEnv`、`DexpnpEnv`）都需要同步创建 franky 版本子类。
- 需要注册新的 Gymnasium ID（`FrankyEnv-v1`、`FrankyPegInsertionEnv-v1` 等），破坏现有 YAML 配置兼容性。
- 后续 FrankaEnv 的任何 bug fix 或功能增加都要在两处同步维护。

**正确做法：** 在 `FrankaEnv._setup_hardware()` 中加入控制器工厂分支。一个 `controller_backend` 配置字段，零代码复制，所有现有 task env 和 Gymnasium ID 自动获得 franky 支持。

### 1.2 事实性错误

| 旧方案说法 | 实际代码 | 影响 |
|------------|----------|------|
| `move_gripper(0-255)` 标注"是——连续夹爪控制"，暗示 env 依赖 | `FrankaEnv` 从未调用 `move_gripper`；`_binary_gripper_action`（第 807 行）仅用 `open_gripper()` / `close_gripper()`；灵巧手走 `command_end_effector` | `move_gripper` 仅用于 smoke test CLI，不影响 env 运行 |
| Franka Hand 最大宽度 0.08m | ROS 版 `FrankaGripper.open()` 发送 `width=0.09`（`franka_gripper.py:68`） | 映射公式需用 0.09 而非 0.08 |
| `launch_controller` 签名可直接对齐 | `FrankaEnv._setup_hardware` 传 `end_effector_type`、`end_effector_config`（第 268–269 行），`FrankyController.launch_controller` 不接受这些参数 | 必须在工厂层做参数适配 |

### 1.3 关键遗漏

| 遗漏项 | 说明 |
|--------|------|
| `start_impedance` / `stop_impedance` | `FrankaController` 用这两个方法管理 ROS 进程生命周期；franky 用 tracker 的 `_ensure_*` / `_stop_*` 模式，语义不同但对 env 透明——旧方案未分析 |
| `reconfigure_compliance_params` 阻尼映射 | franky 的 `CartesianImpedanceTracker` 无显式 damping 参数，用 `gains_time_constant` 和 `max_delta_tau` 替代——旧方案只列了"映射表"但映射关系错误 |
| `create_gripper` 工厂更新 | 现有 `create_gripper(gripper_type="franka")` 要求传入 `ros` 参数（`gripper/__init__.py:51-58`），无法用于 franky 栈——旧方案未提及 |
| dtype 转换 | `FrankaEnv._move_action` 传 `position.astype(np.float32)`（第 874 行），但 `FrankyController.move_tcp_pose` assert `float64`——alias 必须做 dtype 转换 |
| dummy e2e 测试 | 现有 `tests/e2e_tests/embodied/realworld_dummy_sac_cnn.yaml` 只覆盖 ROS 路径——franky 路径无 CI 覆盖 |
| Docker / CI 更新 | Docker 已有 franky venv 构建（`Dockerfile:349`），但无单臂 franky e2e CI job |
| `FrankaRobotState.gripper_position` 字段语义 | Robotiq 返回 0-255 int；Franka Hand 用 meter 制——需统一 |

### 1.4 阶段划分过粗

旧方案 Phase 0-3 跨度 2-3 周/phase，不利于逐日跟踪。本方案拆分为 6 个更细粒度的 Phase（每个 ≤ 1 周）。

---

## 2. 固件 5.10.0 版本锁定与环境准备

### 2.1 兼容矩阵

| 项目 | 5.10.0 取值 |
|------|-------------|
| Robot System Version | **5.10.0**（`>= 5.9.0` 区间） |
| 最低 libfranka | **>= 0.18.0** |
| Robot / Gripper Server | **10 / 3** |
| **RLinf 选用** | **libfranka 0.19.0**（唯一推荐） |
| **明确禁用** | **0.18.0**（阻抗控制 bug，[#1012](https://github.com/RLinf/RLinf/issues/1012)） |
| ROS / serl 路径 | **不可用**（`serl_franka_controllers` + libfranka 0.15 仅支持 `< 5.9.0`） |

### 2.2 控制节点安装

```bash
# 1. 克隆 RLinf
git clone https://github.com/RLinf/RLinf.git && cd RLinf

# 2. 设置固件对应版本
export LIBFRANKA_VERSION=0.19.0
export REPO_PATH=$(pwd)

# 3. 安装 franky 环境
# 底层调用 install_franka_franky_env()（install.sh:2368），
# 下载 franky-control wheel 并安装 lerobot
bash requirements/install.sh embodied --env franka-franky
source .venv/bin/activate

# 4. 验证
python -c "import franky; print('franky OK')"
```

Docker 等价操作：

```bash
source switch_env franky-0.19.0
# 禁止：source switch_env franka-0.15.0  # 5.10.0 下会连接失败
```

### 2.3 实时内核与权限

`FrankyController._apply_rt_hardening()`（`franky_controller.py:154-178`）会在启动时自动执行 mlockall、SCHED_FIFO、CPU 亲和性设置，但需要系统级前置条件：

```bash
# 检查 RT 内核
uname -r | grep rt

# 配置 /etc/security/limits.d/99-rlinf-franka.conf
#   <user>  -  rtprio  99
#   <user>  -  memlock unlimited

# 重新登录后验证
ulimit -r    # 应 >= 80
ulimit -l    # 应为 unlimited
```

每次启动 Ray 前，执行 CPU / 网卡调优：

```bash
# <FRANKA_NIC> = 连接机器人的网卡名
sudo bash -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > "$g"
done'
sudo sysctl -w kernel.sched_rt_runtime_us=-1
sudo ethtool -C <FRANKA_NIC> rx-usecs 0 tx-usecs 0 2>/dev/null || true
```

### 2.4 Desk 侧检查

1. 浏览器 `http://<ROBOT_IP>/desk` → Settings → Dashboard：Control 版本 **5.10.0**
2. FCI 已激活，无 safety violation
3. 控制节点 `ping -c 3 <ROBOT_IP>`

---

## 3. 现状分析（基于代码实查）

### 3.1 两条控制栈对比

```
┌────────────────────────────────────────────────────────────────────────┐
│  工作流 Runner（collect / SFT / RLPD / DAgger / RLT / RTC）             │
└───────────────────────────────┬────────────────────────────────────────┘
                                │ env.step(action)
                                ▼
┌────────────────────────────────────────────────────────────────────────┐
│  FrankaEnv (franka_env.py:142)                                         │
│  └─ _setup_hardware() → 硬编码 FrankaController                        │
│  子类：PegInsertionEnv / FrankaBinRelocationEnv / BottleEnv / DexpnpEnv  │
│  Wrappers：Spacemouse / GELLO / Pico / RelativeFrame / Quat2Euler       │
└───────────────────────────────┬────────────────────────────────────────┘
                                │ _controller.move_arm / get_state / ...
                 ┌──────────────┴─────────────────┐
                 ▼                                ▼
  FrankaController (ROS)              FrankyController (libfranka)
  franka_controller.py                franky_controller.py
  serl_franka_controllers             franky-control wheel
  固件 < 5.9.0 ✅                      固件 >= 5.9.0 ✅ (5.10.0)
  Franka Hand via ROS ✅               Franka Hand ❌ NotImplementedError
  单臂 ✅                              仅双臂 DualFrankaEnv ✅
```

**问题核心：** `FrankaEnv._setup_hardware()` 在第 226 行硬编码 `from .franka_controller import FrankaController`，所有单臂 env 只能走 ROS 路径。

### 3.2 FrankyController 方法清单与缺口（逐行核实）

**文件：** `rlinf/envs/realworld/franka/franky_controller.py`（397 行）

**继承关系：** `FrankyController(Worker)` — 与 `FrankaController(Worker)` 无共享基类，API 对齐仅靠约定。

| 方法 | FrankyController | FrankaController | FrankaEnv 是否调用 | 说明 |
|------|:---:|:---:|:---:|------|
| `launch_controller(robot_ip, ...)` | ✅ 参数不同 | ✅ | 是（`_setup_hardware`） | 签名需适配 |
| `is_robot_up()` | ✅ | ✅ | 是（`__init__:205`） | |
| `get_state()` → `FrankaRobotState` | ✅ | ✅ | 是（多处） | |
| `clear_errors()` | ✅ | ✅ | 是（`_clear_error:805`） | |
| `reset_joint(qpos)` | ✅ | ✅ | 是（`go_to_rest:506`） | |
| `open_gripper()` | ✅ | ✅ | 是（`_binary_gripper_action:819`） | |
| `close_gripper()` | ✅ | ✅ | 是（`_binary_gripper_action:813`） | |
| `move_tcp_pose(pose7)` | ✅ | — | 否（env 用 `move_arm`） | |
| `move_joints(q7)` | ✅ | — | 否（仅 DualFrankaEnv 用） | |
| **`move_arm(pose7)`** | **❌ 缺失** | ✅ | **是（`_move_action:874`）** | **必须补齐** |
| **`reconfigure_compliance_params(dict)`** | **❌ 缺失** | ✅ | **是（`reset:482`）** | **必须补齐** |
| **`move_gripper(pos, speed)`** | **❌ 缺失** | ✅ | 否（仅 smoke test 用） | 建议补齐 |
| `command_end_effector(action)` | ❌ 缺失 | ✅ | 是（灵巧手，`_end_effector_action:851`） | Phase 后期，初始可 stub |
| `reset_end_effector(state)` | ❌ 缺失 | ✅ | 是（灵巧手，`go_to_rest:535`） | Phase 后期，初始可 stub |
| `start_impedance()` | — | ✅ | 否（ROS 内部生命周期） | 无需实现 |
| `stop_impedance()` | — | ✅ | 否（ROS 内部生命周期） | 无需实现 |
| `cleanup()` | ✅ | — | — | |

### 3.3 launch_controller 签名差异

```python
# FrankaController（ROS）— franka_controller.py
FrankaController.launch_controller(
    robot_ip, env_idx=0, node_rank=0, worker_rank=0,
    ros_pkg="serl_franka_controllers",
    end_effector_type="franka_gripper",    # ← FrankyController 无此参数
    end_effector_config=None,              # ← FrankyController 无此参数
    gripper_type=None,
    gripper_connection=None,
)

# FrankyController（libfranka）— franky_controller.py
FrankyController.launch_controller(
    robot_ip, env_idx=0, node_rank=0, worker_rank=0,
    gripper_type="robotiq",                # ← 默认值不同
    gripper_connection=None,
)
```

**FrankaEnv._setup_hardware 实际调用（第 263-271 行）：**

```python
self._controller = FrankaController.launch_controller(
    robot_ip=self.config.robot_ip,
    env_idx=self.env_idx,
    node_rank=controller_node_rank,
    worker_rank=self.env_worker_rank,
    end_effector_type=self.config.end_effector_type,
    end_effector_config=self.config.end_effector_config,
    gripper_connection=self.config.gripper_connection,
)
```

注意：此调用没有传 `gripper_type`（ROS 版在内部从 `end_effector_type` 推断）。工厂分支必须处理这个差异。

### 3.4 Gripper 子系统

**基类：** `BaseGripper`（`common/gripper/base_gripper.py`）

```python
class BaseGripper(ABC):
    def open(self, speed=0.3) -> None: ...
    def close(self, speed=0.3, force=130.0) -> None: ...
    def move(self, position: float, speed=0.3) -> None: ...
    @property position -> float: ...
    @property is_open -> bool: ...
    def is_ready(self) -> bool: ...
    def cleanup(self) -> None: ...  # 有默认空实现
```

**ROS 版 FrankaGripper 关键参数（`franka_gripper.py`）：**

- `open()`: `width=0.09`，`speed` 透传
- `close()`: `width=0.01`，`force=130.0`
- `move(position)`: `width = position / (255 * 10)` — 即 position=255 → width=0.1（偏大，但这是现有行为）
- `position` 属性: `np.sum(msg.position)` — 返回两指位置之和（meter 制浮点数）
- `is_open`: 布尔标志（open/close 时设置，非基于宽度判断）

**工厂函数（`gripper/__init__.py`）：**

```python
def create_gripper(gripper_type, ros=None, port=None, **kwargs):
    if gt == "robotiq": return RobotiqGripper(port=port)
    if gt == "franka":
        assert ros is not None  # ← 要求 ROSController，franky 栈无法满足
        return FrankaGripper(ros=ros)
```

**FrankyController._build_gripper（第 133-152 行）当前实现：**

```python
if gt == "franka":
    raise NotImplementedError(...)  # ← 唯一的 NotImplementedError
if gt == "robotiq":
    return create_gripper(gripper_type="robotiq", port=gripper_connection)
```

### 3.5 FrankaConfig（调度器硬件配置）

**文件：** `rlinf/scheduler/hardware/robots/franka.py:209-261`

现有字段：`robot_ip`, `camera_serials`, `camera_type`, `gripper_type`（默认 `"franka"`）, `gripper_connection`, `controller_node_rank`, `disable_validate`

**缺失字段：** `controller_backend`（本方案需新增）

### 3.6 现有可复用资产

| 组件 | 路径 | 复用方式 |
|------|------|----------|
| FrankyController 骨架 | `franky_controller.py`（397 行） | 扩展 3 个方法 + Franka Hand |
| FrankaEnv 全部业务逻辑 | `franka_env.py`（933 行） | 仅改 `_setup_hardware` 一个方法 |
| 所有 task env 子类 | `tasks/*.py` | **零改动** |
| Wrapper 栈 | `common/wrappers/apply.py` | **零改动**（不依赖 controller） |
| Gymnasium 注册 | `tasks/__init__.py` | **零改动** |
| 安装脚本 | `install.sh` → `install_franka_franky_env` | 已支持 0.19.0 |
| Docker | `Dockerfile:349` → franky-0.19.0 venv | 已有构建 |
| Smoke test | `test_franky_controller.py` | 修复 2 个坏命令 |
| Dummy e2e | `tests/e2e_tests/embodied/realworld_dummy_sac_cnn.yaml` | 克隆 + `controller_backend: franky` |

---

## 4. 总体架构设计

### 4.1 设计原则

1. **控制器工厂，不复制 env**：在 `FrankaEnv._setup_hardware()` 中根据 `controller_backend` 选择控制器，消除代码复制。
2. **API 对齐最小集**：只补齐 FrankaEnv 实际调用的方法（`move_arm`、`reconfigure_compliance_params`），不做冗余对齐。
3. **配置驱动**：YAML 中 `controller_backend: franky` 切换栈；所有现有 Gymnasium ID 保持不变。
4. **向后兼容**：`controller_backend` 默认 `"ros"`，现有 ROS 用户无需改配置。

### 4.2 改动范围总览

```
修改 5 个文件 + 新建 1 个文件（对比旧方案的修改 6 + 新建 11）

修改:
├── rlinf/envs/realworld/franka/franky_controller.py   # +3 方法 + Franka Hand
├── rlinf/envs/realworld/franka/franka_env.py          # _setup_hardware 工厂分支
├── rlinf/scheduler/hardware/robots/franka.py          # +controller_backend 字段
├── toolkits/realworld_check/test_franky_controller.py # 修复坏命令
└── docs/source-en/rst_source/examples/embodied/franka.rst  # +franky 章节

新建:
└── rlinf/envs/realworld/common/gripper/franka_libfranka_gripper.py
```

YAML 配置：每个工作流新增 `_franky` 后缀变体（6 个文件），但不涉及代码改动。

### 4.3 架构图

```
                                  YAML: controller_backend: franky
                                            │
┌───────────────────────────────────────────┼────────────────────────────┐
│  FrankaEnv._setup_hardware()              ▼                            │
│  ┌─────────────────────────────────────────────────────────────┐       │
│  │  if controller_backend == "franky":                         │       │
│  │      FrankyController.launch_controller(                    │       │
│  │          robot_ip, env_idx, node_rank, worker_rank,         │       │
│  │          gripper_type, gripper_connection)                   │       │
│  │  else:                                                      │       │
│  │      FrankaController.launch_controller(                    │       │
│  │          robot_ip, env_idx, node_rank, worker_rank,         │       │
│  │          end_effector_type, end_effector_config,             │       │
│  │          gripper_connection)                                 │       │
│  └─────────────────────────────────────────────────────────────┘       │
│  此后所有调用统一走 self._controller.move_arm / get_state / ...         │
│  （FrankyController 已补齐这些方法）                                    │
└───────────────────────────────────────────────────────────────────────┘
```

---

## 5. Phase 0：Franka Hand 驱动（第 1 周）

> **完成标准：** `FrankyController(gripper_type="franka")` 构造成功；open/close/position 正常。

### 5.1 调研 franky 夹爪 API

先确认 `franky` wheel 暴露了什么夹爪接口：

```bash
source .venv/bin/activate
python - <<'PY'
import franky
# 检查 Robot 类上与 gripper 相关的属性
print([x for x in dir(franky.Robot) if "grip" in x.lower()])
# 检查是否有独立 Gripper 类
print([x for x in dir(franky) if "grip" in x.lower() or "Grip" in x])
# 查看 franky.Gripper（如果存在）的方法
if hasattr(franky, 'Gripper'):
    print(dir(franky.Gripper))
PY
```

根据 Brunch-Life/franky 仓库的 C++ 源码和 pybind11 绑定，预期可用的 API 为：

```python
gripper = franky.Gripper(robot_ip)
gripper.move(width=0.05, speed=0.1)          # 移动到指定宽度
gripper.grasp(width=0.01, speed=0.1, force=50, epsilon_inner=0.005, epsilon_outer=0.005)
gripper.open(speed=0.1)                       # 最大宽度
gripper.width                                 # 当前宽度 (m)
gripper.max_width                             # 最大宽度 (m), 通常 0.08
gripper.is_grasped                            # 是否夹持中
```

> **注意：** 以上需实机验证。如果 franky wheel 未暴露 `Gripper` 类，则回退方案是直接使用 `libfranka` C API 通过 ctypes 或编写简单的 pybind 扩展。

### 5.2 新建 FrankaLibfrankaGripper

**新建文件：** `rlinf/envs/realworld/common/gripper/franka_libfranka_gripper.py`

```python
"""Franka parallel-jaw gripper via libfranka / franky (no ROS)."""

import numpy as np

from rlinf.utils.logging import get_logger

from .base_gripper import BaseGripper

# Franka Hand 参数（与 ROS 版 FrankaGripper 对齐）
_MAX_WIDTH_M = 0.09       # open() 时的目标宽度
_CLOSE_WIDTH_M = 0.01     # close() 时的目标宽度
_DEFAULT_GRASP_FORCE = 130.0  # N
_OPEN_THRESHOLD_M = 0.06  # 宽度 > 此值视为 "open"


class FrankaLibfrankaGripper(BaseGripper):
    """Franka Emika parallel-jaw gripper via libfranka (no ROS dependency).

    Args:
        robot_ip: Robot IP address for constructing franky.Gripper.
    """

    def __init__(self, robot_ip: str):
        import franky

        self._logger = get_logger()
        self._gripper = franky.Gripper(robot_ip)
        self._is_open_flag: bool = True
        self._logger.info(
            f"FrankaLibfrankaGripper connected (max_width={self._gripper.max_width:.3f}m)"
        )

    def open(self, speed: float = 0.3) -> None:
        self._gripper.move(_MAX_WIDTH_M, speed)
        self._is_open_flag = True

    def close(self, speed: float = 0.3, force: float = _DEFAULT_GRASP_FORCE) -> None:
        self._gripper.grasp(
            width=_CLOSE_WIDTH_M,
            speed=speed,
            force=force,
            epsilon_inner=0.05,
            epsilon_outer=0.05,
        )
        self._is_open_flag = False

    def move(self, position: float, speed: float = 0.3) -> None:
        # 与 ROS 版 FrankaGripper.move 语义一致：position 0-255 → width
        width = float(position / (255 * 10))
        width = max(0.0, min(width, _MAX_WIDTH_M))
        self._gripper.move(width, speed)

    @property
    def position(self) -> float:
        # 返回当前宽度（meter），与 ROS 版 FrankaGripper.position 语义一致
        return float(self._gripper.width)

    @property
    def is_open(self) -> bool:
        return self._is_open_flag

    def is_ready(self) -> bool:
        try:
            _ = self._gripper.width
            return True
        except Exception:
            return False

    def cleanup(self) -> None:
        pass  # franky.Gripper 无需显式释放
```

**关键设计决策：**

- `position` 属性返回 meter 制宽度（与 ROS 版 `FrankaGripper.position` 一致，即 `np.sum(msg.position)`）。
- `is_open` 用标志而非宽度判断（与 ROS 版一致）。
- `move(position)` 采用 ROS 版相同的换算公式 `width = position / (255 * 10)`。
- **不修改 `create_gripper` 工厂**——该工厂仅被 ROS 路径使用。`FrankyController._build_gripper` 直接实例化 gripper，不经过工厂。

### 5.3 修改 FrankyController._build_gripper

**文件：** `rlinf/envs/realworld/franka/franky_controller.py`，第 133-152 行

**当前代码：**

```python
def _build_gripper(self, gripper_type, gripper_connection, robot_ip):
    gt = (gripper_type or "robotiq").lower()
    if gt == "franka":
        raise NotImplementedError(
            "FrankyController: the libfranka backend for the original "
            "Franka Hand is not yet supported. Use gripper_type='robotiq' "
            "for now."
        )
    if gt == "robotiq":
        return create_gripper(gripper_type="robotiq", port=gripper_connection)
    raise ValueError(...)
```

**改为：**

```python
def _build_gripper(self, gripper_type, gripper_connection, robot_ip):
    gt = (gripper_type or "robotiq").lower()
    if gt == "franka":
        from rlinf.envs.realworld.common.gripper.franka_libfranka_gripper import (
            FrankaLibfrankaGripper,
        )
        return FrankaLibfrankaGripper(robot_ip=robot_ip)
    if gt == "robotiq":
        return create_gripper(gripper_type="robotiq", port=gripper_connection)
    raise ValueError(
        f"FrankyController: unsupported gripper_type={gripper_type!r}. "
        f"Supported: 'franka', 'robotiq'."
    )
```

### 5.4 实机验证

```bash
export FRANKA_ROBOT_IP=<IP>
export FRANKA_GRIPPER_TYPE=franka

python - <<'PY'
import os, time
from rlinf.envs.realworld.common.gripper.franka_libfranka_gripper import (
    FrankaLibfrankaGripper,
)

grip = FrankaLibfrankaGripper(robot_ip=os.environ["FRANKA_ROBOT_IP"])
print(f"ready={grip.is_ready()}, pos={grip.position:.4f}, open={grip.is_open}")

grip.open(speed=0.5)
time.sleep(2)
print(f"after open: pos={grip.position:.4f}, open={grip.is_open}")

grip.close(speed=0.5)
time.sleep(2)
print(f"after close: pos={grip.position:.4f}, open={grip.is_open}")
PY
```

**验收：** 夹爪开合正常；`position` 值在 open 时 ~0.08-0.09，close 时 ~0.0-0.01。

---

## 6. Phase 1：FrankyController API 补齐（第 1–2 周）

> **完成标准：** `FrankyController` 具备 `move_arm`、`reconfigure_compliance_params`、`move_gripper`，smoke test 全部通过。

### 6.1 添加 `move_arm`

**文件：** `rlinf/envs/realworld/franka/franky_controller.py`

在 `move_tcp_pose` 方法（第 317 行）之后添加：

```python
def move_arm(self, position: np.ndarray) -> None:
    """Move TCP to target pose; matches FrankaController.move_arm signature.

    FrankaEnv._move_action passes float32 (franka_env.py:874), but
    move_tcp_pose requires float64 — handle the conversion here.
    """
    self.move_tcp_pose(np.asarray(position, dtype=np.float64))
```

**为什么需要 dtype 转换：** `FrankaEnv._move_action`（第 874 行）调用 `self._controller.move_arm(position.astype(np.float32))`，而 `move_tcp_pose`（第 321 行）assert `pose.shape == (7,)` 且内部运算全部 float64。不做转换会导致精度损失或 assert 失败。

### 6.2 添加 `reconfigure_compliance_params`

这是最复杂的补齐项。`FrankaEnv.reset()`（第 482 行）调用：

```python
self._controller.reconfigure_compliance_params(self.config.compliance_param).wait()
```

`compliance_param` 来自 YAML（例如 `realworld_franka_sft_env.yaml`）：

```yaml
compliance_param:
  translational_stiffness: 2000
  translational_damping: 89
  rotational_stiffness: 150
  rotational_damping: 7
  Ki: 0
```

franky 的 `CartesianImpedanceTracker` 构造参数（`franky_controller.py:286-296`）：

```python
self._cart_tracker = franky.CartesianImpedanceTracker(
    self._robot,
    translational_stiffness=_CART_TRANS_STIFFNESS,  # 默认 500 N/m
    rotational_stiffness=_CART_ROT_STIFFNESS,       # 默认 40 Nm/rad
    nullspace_target=nullspace_target,
    nullspace_stiffness=_CART_NULLSPACE_STIFFNESS,
    translational_error_clip=trans_clip,
    rotational_error_clip=rot_clip,
    max_delta_tau=_CART_MAX_DELTA_TAU,
    gains_time_constant=_CART_GAINS_TC,
)
```

**映射策略：**

| ROS compliance key | franky CartesianImpedanceTracker 参数 | 映射方式 |
|--------------------|--------------------------------------|----------|
| `translational_stiffness` | `translational_stiffness` | 直接传递 |
| `rotational_stiffness` | `rotational_stiffness` | 直接传递 |
| `translational_damping` | 无直接对应 | 通过 `gains_time_constant` 近似：`tc = 2 * damping / stiffness` |
| `rotational_damping` | 无直接对应 | 同上 |
| `Ki` | 无直接对应 | franky 的 CartesianImpedanceTracker 无积分项，忽略并 log warning |

**实现（在 `franky_controller.py` 中添加）：**

```python
def reconfigure_compliance_params(self, params: dict[str, float]) -> None:
    """Reconfigure Cartesian impedance parameters.

    Stops the current Cartesian tracker (if running) and stores the new
    parameters so the next _ensure_cart_tracking_motion() picks them up.
    """
    trans_k = float(params.get("translational_stiffness", _CART_TRANS_STIFFNESS))
    rot_k = float(params.get("rotational_stiffness", _CART_ROT_STIFFNESS))
    trans_d = params.get("translational_damping")
    rot_d = params.get("rotational_damping")
    ki = params.get("Ki", 0)

    if ki and ki > 0:
        self._logger.warning(
            f"reconfigure_compliance_params: Ki={ki} ignored — "
            f"CartesianImpedanceTracker has no integral term"
        )

    # Approximate gains_time_constant from damping ratio
    # For a second-order system: zeta = d / (2*sqrt(k*m)), with m≈1 for normalized
    # gains_time_constant controls the exponential ramp to the new gains
    if trans_d is not None and trans_k > 0:
        tc = max(0.01, 2.0 * float(trans_d) / trans_k)
    else:
        tc = _CART_GAINS_TC

    self._compliance_trans_k = trans_k
    self._compliance_rot_k = rot_k
    self._compliance_tc = tc

    # Force tracker rebuild on next move_arm / move_tcp_pose call
    self._stop_cart_tracking_motion()
```

同时修改 `_ensure_cart_tracking_motion`，使用实例变量而非模块常量：

```python
def _ensure_cart_tracking_motion(self) -> None:
    if self._cart_tracker is not None:
        return
    self._stop_tracking_motion()
    self._safe_join()
    self._robot.recover_from_errors()
    nullspace_target = np.asarray(self._robot.state.q, dtype=np.float64).copy()
    trans_clip = np.full(3, _CART_TRANS_ERROR_CLIP_M, dtype=np.float64)
    rot_clip = np.full(3, _CART_ROT_ERROR_CLIP_RAD, dtype=np.float64)

    # 使用实例变量（reconfigure_compliance_params 设置的值）
    trans_k = getattr(self, "_compliance_trans_k", _CART_TRANS_STIFFNESS)
    rot_k = getattr(self, "_compliance_rot_k", _CART_ROT_STIFFNESS)
    tc = getattr(self, "_compliance_tc", _CART_GAINS_TC)

    self._cart_tracker = self._franky.CartesianImpedanceTracker(
        self._robot,
        translational_stiffness=trans_k,
        rotational_stiffness=rot_k,
        nullspace_target=nullspace_target,
        nullspace_stiffness=_CART_NULLSPACE_STIFFNESS,
        translational_error_clip=trans_clip,
        rotational_error_clip=rot_clip,
        max_delta_tau=_CART_MAX_DELTA_TAU,
        gains_time_constant=tc,
    )
    self._logger.info(
        f"Cartesian impedance tracker started "
        f"(K_t={trans_k:.0f} N/m, K_r={rot_k:.1f} Nm/rad, tc={tc:.3f}s)"
    )
```

**线程安全考虑：** `reconfigure_compliance_params` 在 `FrankaEnv.reset()` 中被调用，而 `_ensure_cart_tracking_motion` 在 `move_tcp_pose` → `step()` 中被调用。由于 `FrankyController` 是 Ray Actor（单线程执行），不存在并发问题。

### 6.3 添加 `move_gripper`

虽然 FrankaEnv 不直接调用此方法，但 smoke test 的 `grip <0-255>` 命令依赖它：

```python
def move_gripper(self, position: int, speed: float = 0.3) -> None:
    """Move gripper to absolute position (0=closed, 255=open).

    Matches FrankaController.move_gripper signature. Only used by
    the interactive smoke test — FrankaEnv uses open/close_gripper.
    """
    assert 0 <= position <= 255
    self._gripper.move(position=float(position), speed=speed)
```

### 6.4 添加灵巧手 stub

`FrankaEnv._end_effector_action` 和 `go_to_rest` 会在灵巧手模式下调用 `command_end_effector` / `reset_end_effector`。5.10.0 单臂方案暂不支持灵巧手，但必须有 stub 防止 import 时 AttributeError：

```python
def command_end_effector(self, action: np.ndarray) -> bool:
    raise NotImplementedError(
        "FrankyController does not support dexterous hands. "
        "Use end_effector_type='franka_gripper' or 'robotiq_gripper'."
    )

def reset_end_effector(self, target_state) -> None:
    raise NotImplementedError(
        "FrankyController does not support dexterous hands. "
        "Use end_effector_type='franka_gripper' or 'robotiq_gripper'."
    )
```

---

## 7. Phase 2：控制器工厂接入 FrankaEnv（第 2–3 周）

> **完成标准：** `FrankaEnv(override_cfg={"controller_backend": "franky", ...})` 构造成功；dummy unit test 通过。

### 7.1 添加 `controller_backend` 到 FrankaRobotConfig

**文件：** `rlinf/envs/realworld/franka/franka_env.py`，`FrankaRobotConfig` 类（第 46 行起）

在 `gripper_connection` 字段之后添加：

```python
@dataclass
class FrankaRobotConfig:
    # ... 现有字段 ...
    gripper_connection: Optional[str] = None

    # -- Controller backend -----------------------------------------------
    controller_backend: str = "ros"  # "ros" | "franky"

    enable_camera_player: bool = True
    # ... 后续字段不变 ...
```

### 7.2 添加 `controller_backend` 到 FrankaConfig（调度器）

**文件：** `rlinf/scheduler/hardware/robots/franka.py`，`FrankaConfig` 类（第 209 行起）

在 `controller_node_rank` 字段之后添加：

```python
@dataclass
class FrankaConfig(HardwareConfig):
    # ... 现有字段 ...
    controller_node_rank: Optional[int] = None

    controller_backend: str = "ros"
    """Controller backend: ``"ros"`` (serl_franka_controllers) or ``"franky"``
    (libfranka via franky-control). Firmware >= 5.9.0 must use ``"franky"``."""

    disable_validate: bool = False
    # ... 后续不变 ...
```

### 7.3 修改 FrankaEnv._setup_hardware — 核心改动

**文件：** `rlinf/envs/realworld/franka/franka_env.py`，第 225-271 行

**当前代码（精简）：**

```python
def _setup_hardware(self):
    from .franka_controller import FrankaController

    assert isinstance(self.hardware_info, FrankaHWInfo)
    # ... 从 hardware_info 填充 config 字段 ...

    controller_node_rank = getattr(
        self.hardware_info.config, "controller_node_rank", None
    )
    if controller_node_rank is None:
        controller_node_rank = self.node_rank
    self._controller = FrankaController.launch_controller(
        robot_ip=self.config.robot_ip,
        env_idx=self.env_idx,
        node_rank=controller_node_rank,
        worker_rank=self.env_worker_rank,
        end_effector_type=self.config.end_effector_type,
        end_effector_config=self.config.end_effector_config,
        gripper_connection=self.config.gripper_connection,
    )
```

**改为：**

```python
def _setup_hardware(self):
    assert self.env_idx >= 0, "env_idx must be set for FrankaEnv."
    assert isinstance(self.hardware_info, FrankaHWInfo), (
        f"hardware_info must be FrankaHWInfo, but got {type(self.hardware_info)}."
    )

    # --- 从 hardware_info 填充 config 字段（不变） ---
    if self.config.robot_ip is None:
        self.config.robot_ip = self.hardware_info.config.robot_ip
    if self.config.camera_serials is None:
        self.config.camera_serials = self.hardware_info.config.camera_serials
    if self.config.camera_type is None:
        self.config.camera_type = getattr(
            self.hardware_info.config, "camera_type", "realsense"
        )
    if self.config.gripper_type is None:
        self.config.gripper_type = getattr(
            self.hardware_info.config, "gripper_type", "franka"
        )
    if self.config.gripper_connection is None:
        self.config.gripper_connection = getattr(
            self.hardware_info.config, "gripper_connection", None
        )
    self.config.end_effector_type = normalize_end_effector_type(
        self.config.end_effector_type,
        self.config.gripper_type,
    ).value

    controller_node_rank = getattr(
        self.hardware_info.config, "controller_node_rank", None
    )
    if controller_node_rank is None:
        controller_node_rank = self.node_rank

    # --- 从 hardware_info 继承 controller_backend（若 env config 未设置） ---
    hw_backend = getattr(self.hardware_info.config, "controller_backend", "ros")
    if self.config.controller_backend == "ros" and hw_backend == "franky":
        self.config.controller_backend = hw_backend

    # --- 控制器工厂 ---
    self._controller = self._launch_controller(controller_node_rank)

def _launch_controller(self, controller_node_rank: int):
    """Create the arm controller based on controller_backend."""
    if self.config.controller_backend == "franky":
        from .franky_controller import FrankyController

        return FrankyController.launch_controller(
            robot_ip=self.config.robot_ip,
            env_idx=self.env_idx,
            node_rank=controller_node_rank,
            worker_rank=self.env_worker_rank,
            gripper_type=self.config.gripper_type or "franka",
            gripper_connection=self.config.gripper_connection,
        )

    from .franka_controller import FrankaController

    return FrankaController.launch_controller(
        robot_ip=self.config.robot_ip,
        env_idx=self.env_idx,
        node_rank=controller_node_rank,
        worker_rank=self.env_worker_rank,
        end_effector_type=self.config.end_effector_type,
        end_effector_config=self.config.end_effector_config,
        gripper_connection=self.config.gripper_connection,
    )
```

**要点说明：**

1. `_launch_controller` 提取为独立方法，便于 task env 子类覆写（虽然正常情况下不需要）。
2. franky 路径传 `gripper_type`（FrankyController 需要），ROS 路径传 `end_effector_type`（FrankaController 需要）——两者签名不同，在工厂层适配。
3. `controller_backend` 优先级：env 的 `override_cfg` > hardware_info 的 `FrankaConfig` > 默认 `"ros"`。

### 7.4 验证：所有 task env 零改动可用

由于所有 task env（`PegInsertionEnv`、`FrankaBinRelocationEnv`、`BottleEnv`、`DexpnpEnv`）继承自 `FrankaEnv`，它们的 `_setup_hardware()` 来自父类，**无需任何改动**即可使用 `controller_backend: franky`。

唯一前提是 YAML 中设置 `override_cfg.controller_backend: franky`（或在 hardware config 中设置）。

Gymnasium 注册也无需改动——`PegInsertionEnv-v1` 等 ID 继续使用，仅配置不同。

### 7.5 Dummy Unit Test

**新建文件：** `tests/unit_tests/test_franky_env_dummy.py`

```python
"""Verify FrankaEnv with controller_backend=franky in dummy mode."""
import gymnasium as gym
import numpy as np
import pytest

# Trigger gymnasium registrations
import rlinf.envs.realworld.franka.tasks  # noqa: F401


@pytest.fixture
def dummy_franky_env():
    """Create a FrankaEnv-v1 in dummy mode with franky backend."""
    env = gym.make(
        "FrankaEnv-v1",
        override_cfg={
            "is_dummy": True,
            "controller_backend": "franky",
            "camera_serials": ["000000000000"],
        },
        worker_info=None,
        hardware_info=None,
        env_idx=0,
        env_cfg={},
    )
    yield env
    env.close()


def test_obs_space(dummy_franky_env):
    obs, info = dummy_franky_env.reset()
    assert "state" in obs
    assert "frames" in obs
    assert "tcp_pose" in obs["state"]


def test_step(dummy_franky_env):
    dummy_franky_env.reset()
    action = dummy_franky_env.action_space.sample()
    obs, reward, terminated, truncated, info = dummy_franky_env.step(action)
    assert obs is not None
    assert isinstance(reward, float)
```

运行：

```bash
pytest tests/unit_tests/test_franky_env_dummy.py -v
```

> **注意：** dummy 模式下不创建 controller（`franka_env.py:188` 的 `if not self.config.is_dummy` 跳过 `_setup_hardware`），所以此测试验证的是配置字段不会破坏 env 初始化，而非 franky 控制器本身。

---

## 8. Phase 3：Smoke Test 修复与扩展（第 3 周）

> **完成标准：** `test_franky_controller.py` 所有命令均可执行，含 Franka Hand 开闭与连续位置控制。

### 8.1 修复现有坏命令

**文件：** `toolkits/realworld_check/test_franky_controller.py`

当前 `grip <0-255>` 和 `impedance <7 ints>` 命令调用不存在的方法，会触发 `AttributeError`。

**修复 `grip` 命令：**

将 `controller.move_gripper(pos)` 调用改为使用 Phase 1 中添加的 `move_gripper` 方法（无需额外改动，添加方法后自然修复）。

**修复 `impedance` 命令：**

将 `controller.reconfigure_compliance_params({"Kq": Kq})` 改为使用新的 compliance 参数格式：

```python
# 替换旧的 impedance 命令处理
elif cmd == "impedance":
    # 接受 translational_stiffness rotational_stiffness 两个参数
    if len(parts) >= 3:
        trans_k = float(parts[1])
        rot_k = float(parts[2])
    else:
        trans_k = 500.0
        rot_k = 40.0
    controller.reconfigure_compliance_params({
        "translational_stiffness": trans_k,
        "rotational_stiffness": rot_k,
    }).wait()
    print(f"Compliance: K_t={trans_k}, K_r={rot_k}")
```

### 8.2 添加 Franka Hand 专用命令

在命令循环中添加：

```python
elif cmd == "getpos":
    state = controller.get_state().wait()[0]
    print(f"TCP: {state.tcp_pose}")
    print(f"Gripper: pos={state.gripper_position:.4f}, open={state.gripper_open}")

# 确认现有 open/close 命令已覆盖
```

### 8.3 实机全命令验证

```bash
export FRANKA_ROBOT_IP=<IP>
export FRANKA_GRIPPER_TYPE=franka

python -m toolkits.realworld_check.test_franky_controller

# 依次执行：
#   home        → 关节回原点
#   getpos      → 打印 TCP 和夹爪状态
#   nudge 0 0.1 → 微动关节 0
#   open        → 夹爪全开
#   close       → 夹爪夹紧
#   grip 128    → 夹爪半开
#   impedance 2000 150  → 切换刚度
#   nudge 2 0.05        → 验证新刚度下运动
#   shutdown    → 退出
```

---

## 9. Phase 4：各工作流 YAML 与端到端验证（第 3–5 周）

> **完成标准：** 6 个工作流各至少完成 1 次端到端 smoke（实机或 dummy→实机）。

### 9.0 通用前置

#### Ray 集群启动

```bash
# 控制节点
export RLINF_NODE_RANK=0
export ROBOT_IP=<机器人IP>
source .venv/bin/activate
ray start --head --port=6379 --node-ip-address=<控制节点IP>

# GPU 节点（若双节点）
export RLINF_NODE_RANK=1
ray start --address=<控制节点IP>:6379

# 验证
python -m ray_utils.check_ray
```

#### 公共 hardware 配置片段

所有 franky YAML 的 hardware 配置使用：

```yaml
cluster:
  node_groups:
    - label: franka
      hardware:
        type: Franka
        configs:
          - robot_ip: ${oc.env:ROBOT_IP}
            node_rank: 0
            controller_node_rank: 0
            controller_backend: franky
            gripper_type: franka
```

env 层增加：

```yaml
override_cfg:
  controller_backend: franky
  end_effector_type: franka_gripper
  is_dummy: false
```

### 9.1 工作流 1：数据采集（collect_data）

**原配置：** `examples/embodiment/config/realworld_collect_data.yaml`

**创建 franky 变体：**

```bash
cp examples/embodiment/config/realworld_collect_data.yaml \
   examples/embodiment/config/realworld_collect_data_franky.yaml
```

**修改要点：**

```yaml
# 头部注释
# Target: Franka firmware 5.10.0, libfranka 0.19.0, Franka Hand
# Install: export LIBFRANKA_VERSION=0.19.0 && bash requirements/install.sh embodied --env franka-franky

defaults:
  - env/realworld_peg_insertion@env.eval  # 保持不变

env:
  eval:
    override_cfg:
      controller_backend: franky   # ← 新增
    use_spacemouse: true            # wrapper 不依赖 ROS

cluster:
  node_groups:
    - label: franka
      hardware:
        type: Franka
        configs:
          - robot_ip: ${oc.env:ROBOT_IP}
            node_rank: 0
            controller_backend: franky   # ← 新增
            gripper_type: franka         # ← 改为 franka（非 robotiq）
```

**运行：**

```bash
export EMBODIED_PATH=examples/embodiment
export MUJOCO_GL=egl
bash examples/embodiment/collect_data.sh realworld_collect_data_franky
```

**验收：** >= 10 个成功 episode；检查 `collected_data/` 中 obs 含图像、action 维度为 7。

### 9.2 工作流 2：Pi0 SFT 部署

**原配置：** `evaluations/realworld/realworld_eval.yaml`

**创建 franky 变体：**

```bash
cp evaluations/realworld/realworld_eval.yaml \
   evaluations/realworld/realworld_eval_franky.yaml
```

**修改要点：**

```yaml
env:
  eval:
    init_params:
      id: "FrankaEnv-v1"              # 保持不变，无需新 ID
    override_cfg:
      controller_backend: franky       # ← 新增
      task_description: "your task"
      target_ee_pose: [0.5, 0.0, 0.1, -3.14, 0.0, 0.0]
    use_spacemouse: false

cluster:
  node_groups:
    - label: franka
      hardware:
        type: Franka
        configs:
          - robot_ip: ${oc.env:ROBOT_IP}
            node_rank: 0
            controller_backend: franky
            gripper_type: franka

rollout:
  model:
    model_path: <SFT checkpoint 路径>
```

**运行：**

```bash
bash evaluations/run_eval.sh realworld_eval_franky
```

**验收：** 策略控制机械臂 >= 100 step 无 exception。

### 9.3 工作流 3：RLPD

**原配置：** `examples/embodiment/config/realworld_peginsertion_rlpd_cnn_async.yaml`

**创建 franky 变体：**

```bash
cp examples/embodiment/config/realworld_peginsertion_rlpd_cnn_async.yaml \
   examples/embodiment/config/realworld_peginsertion_rlpd_franky_async.yaml
```

**修改要点：**

```yaml
algorithm:
  demo_buffer:
    load_path: <用 franky collect_data 采集的 demo 路径>

env:
  train:
    override_cfg:
      controller_backend: franky
  eval:
    override_cfg:
      controller_backend: franky
    use_spacemouse: true

cluster:
  node_groups:
    - label: franka
      hardware:
        type: Franka
        configs:
          - robot_ip: ${oc.env:ROBOT_IP}
            node_rank: 0
            controller_backend: franky
            gripper_type: franka
```

**运行：**

```bash
python examples/embodiment/train_async.py \
  --config-name realworld_peginsertion_rlpd_franky_async
```

**验收：** >= 100 env steps；actor loss 无 NaN；demo buffer 正常加载。

### 9.4 工作流 4：HG-DAgger

**原配置：** `examples/embodiment/config/realworld_pnp_dagger_openpi.yaml`

```bash
cp examples/embodiment/config/realworld_pnp_dagger_openpi.yaml \
   examples/embodiment/config/realworld_pnp_dagger_openpi_franky.yaml
```

**修改要点：** train/eval 的 `override_cfg` 均加 `controller_backend: franky`，hardware 加 `controller_backend: franky` 和 `gripper_type: franka`。

**运行：**

```bash
python examples/embodiment/train_embodied_agent.py \
  --config-name realworld_pnp_dagger_openpi_franky
```

**验收：** 干预步写入 buffer；`only_save_expert` 数据可用。

### 9.5 工作流 5：RLT Stage 2

**原配置：** `examples/embodiment/config/realworld_rlt_stage2_ac_mlp.yaml`

```bash
cp examples/embodiment/config/realworld_rlt_stage2_ac_mlp.yaml \
   examples/embodiment/config/realworld_rlt_stage2_ac_mlp_franky.yaml
```

**修改要点：** 与上述相同模式——env 层加 `controller_backend: franky`，hardware 层加 `controller_backend: franky`。RLT 特有配置（`rlt_feature_model`、`loss_type: rlt_ac`）不受 controller backend 影响。

**运行：**

```bash
python examples/embodiment/train_embodied_agent.py \
  --config-name realworld_rlt_stage2_ac_mlp_franky
```

**验收：** `rlt_feature_model` 正常加载；`rlt_ac` loss 有限；键盘切换有效。

### 9.6 工作流 6：RTC

**原配置：** `evaluations/realworld/realworld_pnp_eval_pi05_sft_RTC.yaml`

```bash
cp evaluations/realworld/realworld_pnp_eval_pi05_sft_RTC.yaml \
   evaluations/realworld/realworld_pnp_eval_pi05_sft_RTC_franky.yaml
```

**修改要点：** 同上模式加 `controller_backend: franky`。RTC 特有配置（`runner.rtc.*`）不受影响。

**运行：**

```bash
bash evaluations/run_eval.sh realworld_pnp_eval_pi05_sft_RTC_franky
```

**验收：** chunk 重叠执行正常；`time/rollout` 延迟正常；无 control timeout。

---

## 10. Phase 5：CI / Docker / 文档（第 5–6 周）

### 10.1 新增 Dummy E2E 测试

**克隆现有测试并修改：**

```bash
cp tests/e2e_tests/embodied/realworld_dummy_sac_cnn.yaml \
   tests/e2e_tests/embodied/realworld_dummy_franky_sac_cnn.yaml
```

**修改要点：**

```yaml
env:
  train:
    override_cfg:
      controller_backend: franky
      is_dummy: true
  eval:
    override_cfg:
      controller_backend: franky
      is_dummy: true
```

**在 CI 中注册：** `.github/workflows/embodied-e2e-tests.yml` 中添加此测试到现有 franka 测试 job。

### 10.2 Docker 更新

Docker 已有 franky-0.19.0 venv（`Dockerfile:349`），无需新增 build stage。但可在 `switch_env` 脚本中增加别名：

```bash
# 已有：source switch_env franky-0.19.0
# 建议增加：source switch_env franky  → 默认 0.19.0
```

### 10.3 文档更新

**主文档（英文）：** `docs/source-en/rst_source/examples/embodied/franka.rst`

在现有内容基础上增加章节：

```rst
Firmware >= 5.9.0: franky Backend (No ROS)
==========================================

If your Franka robot runs firmware **5.9.0 or later** (e.g., 5.10.0), the
ROS-based ``serl_franka_controllers`` path is not available. Use the
**franky** backend instead, which communicates with the robot directly
via libfranka.

Installation
------------

.. code-block:: bash

   export LIBFRANKA_VERSION=0.19.0
   bash requirements/install.sh embodied --env franka-franky
   source .venv/bin/activate

Configuration
-------------

Add ``controller_backend: franky`` to your YAML:

.. code-block:: yaml

   env:
     train:
       override_cfg:
         controller_backend: franky

   cluster:
     node_groups:
       - label: franka
         hardware:
           type: Franka
           configs:
             - robot_ip: ${oc.env:ROBOT_IP}
               controller_backend: franky
               gripper_type: franka  # or robotiq
```

**中文文档同步：** `docs/source-zh/rst_source/examples/embodied/franka.rst`

---

## 11. 测试与验收方案

### 11.1 L0：安装与连通

- [ ] Desk 显示 Control **5.10.0**，Robot/Gripper Server **10/3**
- [ ] `export LIBFRANKA_VERSION=0.19.0 && bash requirements/install.sh embodied --env franka-franky`
- [ ] `python -c "import franky; print(franky)"` 成功
- [ ] PREEMPT_RT + `ulimit -r >= 80` + `ulimit -l unlimited`
- [ ] `test_franky_controller`：`home` / `nudge` / `open` / `close` / `grip 128` / `impedance 2000 150` 全部通过
- [ ] 负向：`LIBFRANKA_VERSION=0.15.0` 安装后连接失败

### 11.2 L1：单臂 Env 闭环

- [ ] `pytest tests/unit_tests/test_franky_env_dummy.py -v` 通过
- [ ] 实机 `is_dummy=False` + `controller_backend=franky`：10 Hz step，obs 含 TCP + 相机 + gripper
- [ ] `reset()`：`reconfigure_compliance_params` 无 exception，关节复位正常
- [ ] `get_tcp_pose()` 与 Desk 显示一致（误差 < 1cm / 5°）
- [ ] 原 ROS 路径（`controller_backend=ros`）无回归：`pytest tests/unit_tests/ -k franka` 通过

### 11.3 L2：工作流验收

| 工作流 | 命令 | 通过标准 |
|--------|------|----------|
| collect_data | `collect_data.sh realworld_collect_data_franky` | >= 10 成功 episode |
| Pi0 SFT deploy | `run_eval.sh realworld_eval_franky` | >= 100 step 无 exception |
| RLPD | `train_async.py --config-name realworld_peginsertion_rlpd_franky_async` | >= 100 env steps，loss 有限 |
| HG-DAgger | `train_embodied_agent.py --config-name realworld_pnp_dagger_openpi_franky` | 干预数据可写入 |
| RLT Stage2 | `train_embodied_agent.py --config-name realworld_rlt_stage2_ac_mlp_franky` | `rlt_ac` loss 有限 |
| RTC | `run_eval.sh realworld_pnp_eval_pi05_sft_RTC_franky` | chunk 重叠正常 |

### 11.4 L3：稳定性回归

- [ ] 连续 30 min Spacemouse rollout：无 `power_limit_violation`、无 UDP timeout
- [ ] `cyclictest` max latency < 150 us
- [ ] 无"臂不动但夹爪动"（#1012 类问题）

### 11.5 L4：CI 回归

- [ ] `realworld_dummy_franky_sac_cnn.yaml` e2e 测试在 CI 通过
- [ ] Docker build `embodied-franka` 成功（含 franky-0.19.0 venv）

---

## 12. 风险与里程碑

### 12.1 风险

| 风险 | 影响 | 缓解 |
|------|------|------|
| franky wheel 未暴露 `Gripper` 类 | Phase 0 阻塞 | 调研 pybind 绑定；回退方案：ctypes 调用 libfranka C API |
| `CartesianImpedanceTracker` 阻尼映射不精确 | compliance/precision 切换行为与 ROS 版略有差异 | 实测对比两种 tracker 的阶跃响应，调整 `gains_time_constant` |
| libfranka 0.18 阻抗 bug | 臂不动 | **固定 0.19.0**，代码中 assert |
| 单进程 libfranka 独占连接 | smoke test 与 env 不能同时运行 | 文档强调"一次只有一个 client" |
| DexHand 在 franky 栈不可用 | 灵巧手任务不可用 | stub 方法 + 清晰错误消息 + 文档标注 |

### 12.2 里程碑

| 阶段 | 时间 | 交付 |
|------|------|------|
| Phase 0 | 第 1 周 | `FrankaLibfrankaGripper` + `_build_gripper` 修复 |
| Phase 1 | 第 1–2 周 | `move_arm` / `reconfigure_compliance_params` / `move_gripper` |
| Phase 2 | 第 2–3 周 | `_setup_hardware` 工厂 + config 字段 + dummy test |
| Phase 3 | 第 3 周 | Smoke test 全命令通过 |
| Phase 4 | 第 3–5 周 | 6 个工作流 YAML + 端到端验证 |
| Phase 5 | 第 5–6 周 | CI e2e + Docker + 文档 |

---

## 13. 附录

### 13.1 新建/修改文件清单

| 操作 | 路径 | 改动量 |
|------|------|--------|
| **新建** | `rlinf/envs/realworld/common/gripper/franka_libfranka_gripper.py` | ~70 行 |
| **新建** | `tests/unit_tests/test_franky_env_dummy.py` | ~40 行 |
| **新建** | `tests/e2e_tests/embodied/realworld_dummy_franky_sac_cnn.yaml` | ~30 行 |
| **新建** | 6 个 `_franky` 后缀 YAML 配置 | 各 ~10 行差异 |
| **修改** | `rlinf/envs/realworld/franka/franky_controller.py` | +~60 行（4 个方法） |
| **修改** | `rlinf/envs/realworld/franka/franka_env.py` | ~30 行（`_setup_hardware` 重构 + config 字段） |
| **修改** | `rlinf/scheduler/hardware/robots/franka.py` | +3 行（config 字段） |
| **修改** | `toolkits/realworld_check/test_franky_controller.py` | ~15 行（修复 2 个命令） |
| **修改** | `docs/.../franka.rst`（英文 + 中文） | +~50 行/文件 |

### 13.2 与旧方案对比

| 维度 | 旧方案 (franka_1.md) | 本方案 |
|------|---------------------|--------|
| 核心架构 | 复制 `franka_env.py` 为 `franky_env.py` | 控制器工厂，`_setup_hardware` 内分支 |
| 代码复制 | ~933 行 env + 每个 task env | 0 行 |
| 新 Gymnasium ID | 需要 `FrankyEnv-v1` 等 | 不需要，复用现有 ID |
| Task env 改动 | 每个需 franky 子类或 controller_backend 分支 | **零改动** |
| Wrapper 改动 | 无 | 无 |
| 新建文件数 | 11 个 | 3 个（1 个 gripper + 2 个 test） |
| 修改文件数 | 6 个 | 4 个 |
| Phase 粒度 | 4 phase / 2-3 周每 phase | 6 phase / ≤ 1 周每 phase |
| `launch_controller` 签名差异 | 未处理 | 工厂层做参数适配 |
| `move_arm` dtype 问题 | 未提及 | 显式 float32→float64 转换 |
| `reconfigure_compliance_params` 阻尼 | 映射表错误 | 基于 `gains_time_constant` 的近似映射 |
| CI 覆盖 | 未提及 | dummy e2e test + CI job |
| `gripper_position` 语义 | 未讨论 | 明确 meter 制，与 ROS 版一致 |

### 13.3 控制器 API 对齐总表（改造后）

| 方法 | FrankaController (ROS) | FrankyController (改造后) | 实现方式 |
|------|------------------------|--------------------------|----------|
| `move_arm(pose7)` | ROS equilibrium topic | → `move_tcp_pose`（dtype 转换） | 新增 alias |
| `move_gripper(pos, speed)` | ROS move/goal | → `self._gripper.move(pos, speed)` | 新增方法 |
| `reconfigure_compliance_params(dict)` | dynamic_reconfigure | 停止 tracker → 存参数 → 下次 ensure 重建 | 新增方法 |
| `command_end_effector(action)` | ROS + EndEffector 系统 | `raise NotImplementedError` | 新增 stub |
| `reset_end_effector(state)` | ROS + EndEffector 系统 | `raise NotImplementedError` | 新增 stub |
| `open_gripper()` | ✅ 已有 | ✅ 已有 | — |
| `close_gripper()` | ✅ 已有 | ✅ 已有 | — |
| `get_state()` | ✅ 已有 | ✅ 已有 | — |
| `reset_joint(qpos)` | ✅ 已有 | ✅ 已有 | — |
| `clear_errors()` | ✅ 已有 | ✅ 已有 | — |
| `is_robot_up()` | ✅ 已有 | ✅ 已有 | — |
| `cleanup()` | — | ✅ 已有 | — |
| `start_impedance()` | ROS 进程管理 | 无需实现（tracker 自动管理） | — |
| `stop_impedance()` | ROS 进程管理 | 无需实现（tracker 自动管理） | — |

### 13.4 工作流配置映射总表

| 工作流 | 原配置 | franky 变体 | Env ID | 核心 override |
|--------|--------|-------------|--------|---------------|
| collect_data | `realworld_collect_data.yaml` | `realworld_collect_data_franky.yaml` | PegInsertionEnv-v1 | `controller_backend: franky` |
| Pi0 SFT | `realworld_eval.yaml` | `realworld_eval_franky.yaml` | FrankaEnv-v1 | `controller_backend: franky` |
| RLPD | `realworld_peginsertion_rlpd_cnn_async.yaml` | `realworld_peginsertion_rlpd_franky_async.yaml` | PegInsertionEnv-v1 | `controller_backend: franky` |
| HG-DAgger | `realworld_pnp_dagger_openpi.yaml` | `realworld_pnp_dagger_openpi_franky.yaml` | FrankaBinRelocationEnv-v1 | `controller_backend: franky` |
| RLT Stage2 | `realworld_rlt_stage2_ac_mlp.yaml` | `realworld_rlt_stage2_ac_mlp_franky.yaml` | PegInsertionEnv-v1 | `controller_backend: franky` |
| RTC | `realworld_pnp_eval_pi05_sft_RTC.yaml` | `realworld_pnp_eval_pi05_sft_RTC_franky.yaml` | FrankaBinRelocationEnv-v1 | `controller_backend: franky` |

### 13.5 常见问题

**Q: 为什么不创建新的 `FrankyEnv-v1` Gymnasium ID？**
A: FrankaEnv 与 FrankyController 的差异仅在 controller 创建逻辑（`_setup_hardware` 的 10 行代码）。复制整个 env 类意味着 933 行重复代码和 4 个 task env 子类的平行维护负担。通过 `controller_backend` 配置字段，所有现有 ID 自动支持两种后端。

**Q: 能否在 5.10.0 上继续用 ROS 路径？**
A: 不能。`serl_franka_controllers` + libfranka 0.15 仅支持固件 `< 5.9.0`。

**Q: 能否用 libfranka 0.18.0？**
A: 不推荐。有阻抗 bug（[#1012](https://github.com/RLinf/RLinf/issues/1012)）。请用 **0.19.0**。

**Q: `reconfigure_compliance_params` 的阻尼映射精确吗？**
A: 非精确映射。ROS 版的 damping 直接控制 PD 控制器阻尼系数，franky 版通过 `gains_time_constant` 控制增益平滑过渡时间。实测建议：先用默认 `tc=0.1`，如果响应过冲则增大 tc，过慢则减小。

**Q: 灵巧手任务能用 franky 后端吗？**
A: 当前不能。`command_end_effector` / `reset_end_effector` 为 stub，会抛出 `NotImplementedError`。使用 `end_effector_type=franka_gripper` 或 `robotiq_gripper`。灵巧手支持可作为后续 Phase 扩展。

**Q: 双臂 env 受影响吗？**
A: 不受影响。`DualFrankaEnv` 直接 import `FrankyController`（不经过 `_setup_hardware` 工厂），其调用链（`move_joints`/`move_tcp_pose`）不涉及本次新增的方法。

---

*文档版本：2026-08-12 · 目标固件 5.10.0 · 基于 RLinf 代码库逐行分析*
