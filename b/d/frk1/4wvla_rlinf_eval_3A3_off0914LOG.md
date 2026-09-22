# 4DWVLA 纯 VLA 评估 — 离线测试执行日志

> **日期**: 2026-09-14
> **基于文档**: `4wvla_rlinf_eval_3A3.md` v3A3.4
> **范围**: §14.1 不需要连接真机的测试 (离线) — T1, T2, T3, T4, T10
> **执行环境**: 宿主机 (非 Docker 容器)

---

## 0. 执行前环境检查

### 0.1 宿主机环境

| 项目 | 值 |
|:---|:---|
| OS | Linux 5.15.0-1032-realtime |
| Python | 3.10.12 (系统 Python) |
| numpy | 2.2.6 |
| gymnasium | 1.3.0 (本次安装) |
| torch | 未安装 (系统 Python) |
| franky-control | 未安装 (系统 Python) |
| evdev | 未安装 (系统 Python) |

### 0.2 可执行的离线测试

| 测试 | 宿主机可执行? | 原因 |
|:---|:---:|:---|
| T1 (Transform) | 否 | 需要 GPU 容器 + 4dwvla venv (torch, lerobot) |
| T2 (IPC) | **是** | 仅需 Python 3.10+ numpy |
| T3 (安全逻辑 + Gym) | **是** | 仅需 Python 3.10+ numpy gymnasium |
| T4 (模型加载) | 否 | 需要 GPU 容器 + 4dwvla venv + 检查点 |
| T10 (Keyboard Wrapper) | **是** | 仅需 Python 3.10+ gymnasium |

**结论**: 宿主机上可执行 T2, T3, T10. T1 和 T4 需要 GPU 容器环境, 将在后续部署 Docker 容器后执行.

---

## 1. 代码生成

### 1.1 创建目录结构

```bash
mkdir -p /home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/{configs,tests}
touch /home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/__init__.py
touch /home/nvidia/bt/s/RLmm/b/x/4dwvla_ext/tests/__init__.py
```

### 1.2 从 eval_3A3.md 提取的代码文件

通过 4 个并行 Agent 从 eval_3A3.md 中提取代码:
- Agent 1: `franky_controller_direct.py` (§6.3) — 978-1335 行
- Agent 2: `franky_joint_env.py` (§6.4), `keyboard_vla_eval.py` (§6.5), `franka_vla_client.py` (§6.6)
- Agent 3: 4 个测试文件 (T1, T2, T3, T10)
- Agent 4: `vla_inference_server.py` (§5.2), Docker 脚本 (§7), `extreme_pose_explorer.py` (§10)

全部 4 个 Agent 执行完毕后, 验证文件列表:

```bash
$ find /home/nvidia/bt/s/RLmm/b/x/4dwvla_ext -type f | sort
```

| 文件 | 来源章节 | 说明 |
|:---|:---|:---|
| `__init__.py` | — | 空, 标记为 Python 包 |
| `franky_controller_direct.py` | §6.3 | 安全控制器 (复制 FrankyControllerExtended 算法) |
| `franky_joint_env.py` | §6.4 | gym.Env + 8 级安全 |
| `keyboard_vla_eval.py` | §6.5 | gym.Wrapper + KeyboardListener |
| `franka_vla_client.py` | §6.6 | 评估主脚本 |
| `vla_inference_server.py` | §5.2 | GPU 推理服务 |
| `extreme_pose_explorer.py` | §10 | 极限位姿探测 |
| `configs/docker_run_4dwvla_gpu.sh` | §7.1 | GPU 容器启动脚本 |
| `configs/docker_run_4dwvla_franky.sh` | §7.2 | Franky 容器启动脚本 |
| `configs/setup_4dwvla_venv.sh` | §5.1 | 4dwvla venv 初始化脚本 |
| `tests/__init__.py` | — | 空, 标记为 Python 包 |
| `tests/test_transforms_offline.py` | T1 | Transform 管线测试 |
| `tests/test_ipc_offline.py` | T2 | IPC 通信测试 |
| `tests/test_safety_offline.py` | T3 | 安全逻辑 + Gym 环境测试 |
| `tests/test_keyboard_wrapper_offline.py` | T10 | KeyboardVLAEvalWrapper 逻辑测试 |

共 15 个文件.

### 1.3 franky_ext.motion_limits 导入验证

```bash
$ python3 -c "from franky_ext.motion_limits import guard_margin_m; print('OK')"
OK
```

`franky_ext` 位于 `/home/nvidia/bt/s/RLmm/b/x/franky_ext/`, `4dwvla_ext` 中的 `franky_controller_direct.py` 通过 `RLINF_EXT_PATH` 环境变量或默认路径 `/workspace/RLinf/b/x` 导入.

### 1.4 rlinf 导入限制

```bash
$ python3 -c "import sys; sys.path.insert(0, '/home/nvidia/bt/s/RLmm'); from rlinf import *"
# → ModuleNotFoundError: No module named 'torch'
```

**根因**: `rlinf/__init__.py` → `from .utils.omega_resolver import omegaconf_register` → `import torch`. 宿主机系统 Python 未安装 torch.

**影响**: `keyboard_vla_eval.py` 在宿主机上无法直接导入 (它 import `KeyboardListener` from `rlinf`). T10 需要 mock rlinf 导入链.

---

## 2. 离线测试执行

### 2.1 T2: IPC 通信测试

**执行**: `python3 tests/test_ipc_offline.py`

```
=== T2.1: IPC Round-trip ===
  [PASS] status ok
  [PASS] actions shape
  [PASS] latency < 100ms

=== T2.2: Message Format Validation ===
  [PASS] images keys
  [PASS] image shape
  [PASS] image dtype
  [PASS] arm length
  [PASS] gripper length
  [PASS] task is str

=== T2.3: Graceful Shutdown ===
  [PASS] shutdown no hang

=== Results: 10 passed, 0 failed ===
```

**结果**: ✅ 10/10 全部通过, 首次运行即通过.

---

### 2.2 T3: 安全逻辑与 Gym 环境测试

#### 2.2.1 首次运行 — 失败

```bash
$ python3 tests/test_safety_offline.py
```

```
ModuleNotFoundError: No module named 'franky_ext'
```

**根因分析**: `test_safety_offline.py` 的 `sys.path.insert(0, ...)` 只添加了 `4dwvla_ext/` 目录, 但 `franky_controller_direct.py` 通过 `from franky_ext.motion_limits import ...` 导入, 需要 `b/x/` (即 `4dwvla_ext` 的父目录) 也在 `sys.path` 中.

`franky_controller_direct.py` 第 27 行:
```python
sys.path.insert(0, os.environ.get("RLINF_EXT_PATH", "/workspace/RLinf/b/x"))
```
默认路径 `/workspace/RLinf/b/x` 是 Docker 容器路径, 宿主机上不存在. 宿主机的实际路径是 `/home/nvidia/bt/s/RLmm/b/x`.

**修复**: 在 `test_safety_offline.py` 中添加 `4dwvla_ext` 的父目录到 `sys.path`:

```python
# 修改前:
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# 修改后:
_ext_dir = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _ext_dir)
sys.path.insert(0, str(Path(_ext_dir).parent))  # b/x — contains franky_ext
```

#### 2.2.2 修复后重新运行 — 通过

```
=== T3.1: L1 Hard Joint Limits ===
  [PASS] safe action no clip
  [PASS] safe action no warnings
  [PASS] extreme clipped to upper
  [PASS] extreme has L1 warning
  [PASS] extreme_low clipped to lower

=== T3.2: L2 Training Range ===
  [PASS] beyond train has L2 warning
  [PASS] beyond train clipped

=== T3.3: L3 Velocity Limit ===
  [PASS] velocity limited
  [PASS] velocity has L3 warning

=== T3.4: FrankyJointEnv Dummy Mode ===
  [PASS] action_space shape
  [PASS] obs_space has state
  [PASS] obs state shape
  [PASS] reset obs has state
  [PASS] reset obs state shape
  [PASS] reset info is dict
  [PASS] step obs has state
  [PASS] step reward is float
  [PASS] step terminated bool
  [PASS] step truncated bool
  [PASS] step info has warnings
  [PASS] step info has step
  [PASS] multi step ok
  [PASS] dummy frames global
  [PASS] dummy frames wrist
  [PASS] go_to_rest dummy ok
  [PASS] close ok

=== T3.5: Action Space Bounds ===
  [PASS] low arm = JOINT_LIMITS_LOWER
  [PASS] high arm = JOINT_LIMITS_UPPER
  [PASS] low gripper = 0
  [PASS] high gripper = 1

=== T3.6: HOME Joints Validity ===
  [PASS] HOME within hard limits
  [PASS] HOME within training range
  [PASS] HOME has 7 joints

=== T3.7: MotionGuardTripped Exception ===
  [PASS] is RuntimeError
  [PASS] message correct
  [PASS] catchable as RuntimeError

=== Results: 36 passed, 0 failed ===
```

**结果**: ✅ 36/36 全部通过.

---

### 2.3 T10: KeyboardVLAEvalWrapper 离线测试

#### 2.3.1 首次运行 — 失败 (Error 1: franky_ext)

```bash
$ python3 tests/test_keyboard_wrapper_offline.py
```

```
ModuleNotFoundError: No module named 'franky_ext'
```

**根因**: 同 T3, 缺少 `b/x/` 在 sys.path. 修复同 T3.

#### 2.3.2 第二次运行 — 失败 (Error 2: rlinf)

```bash
$ python3 tests/test_keyboard_wrapper_offline.py
```

```
ModuleNotFoundError: No module named 'rlinf'
```

**根因分析**: `keyboard_vla_eval.py` 第 28-29 行在模块级别导入 `KeyboardListener`:
```python
sys.path.insert(0, "/workspace/RLinf")
from rlinf.envs.realworld.common.keyboard.keyboard_listener import KeyboardListener
```

宿主机上 `rlinf` 无法导入 (其 `__init__.py` 依赖 torch), 原测试代码中的 mock 设置:
```python
with patch.dict("sys.modules", {}):
    pass
import keyboard_vla_eval as kvmod
```
这是一个空操作 — `with` 块退出后 `sys.modules` 恢复原状, `import` 仍然失败.

**修复方案**: 完全重写测试的 mock 策略. 在导入 `keyboard_vla_eval` 之前, 预先在 `sys.modules` 中注入 rlinf 整个导入链的 mock, 使 `KeyboardListener` 指向 `MockKeyboardListener`:

```python
import types
from unittest.mock import MagicMock

_mock_kl_module = types.ModuleType("keyboard_listener")
_mock_kl_module.KeyboardListener = MockKeyboardListener

for _mod_name in [
    "rlinf",
    "rlinf.envs",
    "rlinf.envs.realworld",
    "rlinf.envs.realworld.common",
    "rlinf.envs.realworld.common.keyboard",
]:
    sys.modules.setdefault(_mod_name, MagicMock())
sys.modules["rlinf.envs.realworld.common.keyboard.keyboard_listener"] = _mock_kl_module

import keyboard_vla_eval as kvmod
```

同时简化 `make_wrapped_env()`:
```python
def make_wrapped_env():
    env = FrankyJointEnv(is_dummy=True)
    wrapped = kvmod.KeyboardVLAEvalWrapper(env)
    return wrapped, wrapped.listener  # listener 是 MockKeyboardListener 实例
```

**关键设计**: `KeyboardVLAEvalWrapper.__init__()` 调用 `self.listener = KeyboardListener()`, 由于 `KeyboardListener` 已被替换为 `MockKeyboardListener`, 因此每次 `make_wrapped_env()` 都会创建全新的 `MockKeyboardListener` 实例, 测试间完全隔离.

#### 2.3.3 修复后重新运行 — 通过

```
=== T10.1: Abort Key ('r') ===
  [PASS] normal step not truncated
  [PASS] r key truncated
  [PASS] r key abort_reset in info
  [PASS] abort latches

=== T10.2: Success/Failure Keys ('c'/'b') ===
  [PASS] c key terminated
  [PASS] c key reward=1
  [PASS] c key result=success
  [PASS] b key terminated
  [PASS] b key reward=0
  [PASS] b key result=failure

=== T10.3: Home Key ('h') ===
  [PASS] h key not terminated
  [PASS] h key not truncated
  [PASS] h key episode continues

=== T10.4: Idle Before Start ===
  [PASS] idle not terminated
  [PASS] idle not truncated
  [PASS] idle eval_phase=pre

=== T10.5: Debounce ===
  [PASS] first c -> terminated
  [PASS] debounced c -> not terminated

=== Results: 18 passed, 0 failed ===
```

**结果**: ✅ 18/18 全部通过.

---

## 3. 文件修改汇总

### 3.1 修改的文件

| 文件 | 修改内容 | 原因 |
|:---|:---|:---|
| `tests/test_safety_offline.py` (L14) | 添加 `sys.path.insert(0, str(Path(_ext_dir).parent))` | `franky_ext` 在 `b/x/` 中, 测试需要将该目录加入 sys.path |
| `tests/test_keyboard_wrapper_offline.py` | 完全重写 mock 策略 | 原 mock 无效 (空 `with` 块); rlinf 在宿主机不可导入 |

### 3.2 未修改的文件 (从 eval_3A3.md 原样提取)

`franky_controller_direct.py`, `franky_joint_env.py`, `keyboard_vla_eval.py`, `franka_vla_client.py`, `vla_inference_server.py`, `extreme_pose_explorer.py`, `configs/*.sh`, `tests/test_ipc_offline.py`, `tests/test_transforms_offline.py`

---

## 4. 离线测试结果汇总

| ID | 测试名称 | 子测试数 | 通过 | 失败 | 首次通过? | 修复次数 |
|:---:|:---|:---:|:---:|:---:|:---:|:---:|
| T2 | IPC 通信 | 10 | 10 | 0 | ✅ 是 | 0 |
| T3 | 安全逻辑 + Gym | 36 | 36 | 0 | ❌ 否 | 1 |
| T10 | Keyboard Wrapper | 18 | 18 | 0 | ❌ 否 | 2 |
| **合计** | | **64** | **64** | **0** | | |

### 4.1 Error 汇总

| # | 错误 | 测试 | 根因 | 修复 |
|:---:|:---|:---|:---|:---|
| E1 | `ModuleNotFoundError: No module named 'franky_ext'` | T3, T10 | 测试 sys.path 只含 `4dwvla_ext/`, 缺少 `b/x/` (franky_ext 所在目录). `franky_controller_direct.py` 的 `RLINF_EXT_PATH` 默认值指向 Docker 容器路径 | 在测试文件中添加 `sys.path.insert(0, str(Path(_ext_dir).parent))` |
| E2 | `ModuleNotFoundError: No module named 'rlinf'` | T10 | `keyboard_vla_eval.py` 模块级 `from rlinf...import KeyboardListener`; rlinf→torch 依赖链在宿主机不可用. 原测试 mock 无效 (空 `with patch.dict` 块) | 重写测试: 用 `types.ModuleType` + `sys.modules` 预注入完整 rlinf mock 链 |

### 4.2 未执行的离线测试

| ID | 测试名称 | 原因 | 预计执行环境 |
|:---:|:---|:---|:---|
| T1 | Transform 管线一致性 | 需要 torch + lerobot + 4dwvla venv | GPU 容器 |
| T4 | 模型加载 | 需要 torch + 检查点 + GPU | GPU 容器 |

