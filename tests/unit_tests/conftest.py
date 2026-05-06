# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pytest collection-time stubs for minimal environments.

The lightweight Galaxea R1 Pro tests (``test_galaxea_r1_pro_safety.py``,
``test_galaxea_r1_pro_action_schema.py``, ``test_galaxea_r1_pro_safety_l3a.py``)
only need ``r1_pro_safety`` / ``r1_pro_action_schema`` / ``r1_pro_robot_state``,
which are pure-Python and have **no** ROS / Ray / heavy-DL dependencies.

But importing any of them currently runs through
``rlinf/__init__.py`` -> ``rlinf/envs/realworld/__init__.py`` which side-imports
Franka / Turtle2 envs and ``rlinf.scheduler.collective`` (the latter touches
``torch.distributed.Work``, which is not available on JetPack-shipped torch
forks).

To let real-robot bring-up engineers run these unit tests on the Orin
**without** installing ray/gymnasium-with-extras and **without** patching
JetPack torch, we stub the heavy submodules here.  The stubs only kick in
if the module is not already importable, so on a full GPU-server install
this conftest is a no-op.

NOTE: This file is intentionally kept minimal -- if a test legitimately
needs one of the stubbed modules (e.g. tests that exercise Ray RPC),
remove the corresponding stub there.

**适用于最小环境的Pytest用例收集阶段桩模块**

轻量版Galaxea R1 Pro测试（`test_galaxea_r1_pro_safety.py`、`test_galaxea_r1_pro_action_schema.py`、`test_galaxea_r1_pro_safety_l3a.py`）仅依赖`r1_pro_safety`/`r1_pro_action_schema`/`r1_pro_robot_state`这几个模块，它们都是纯Python实现， **完全不依赖** ROS、Ray和大型深度学习库。

但目前导入其中任意一个模块，都会走`rlinf/__init__.py` → `rlinf/envs/realworld/__init__.py`这条引入链路，这个过程会附带导入Franka、Turtle2环境，以及`rlinf.scheduler.collective`模块——后者会调用`torch.distributed.Work`，但JetPack预装的torch分支版本并没有这个类。

为了让实机调试部署工程师能够在Orin平台上运行这些单元测试，既不需要安装ray、gymnasium扩展包，也不需要修改JetPack自带的torch，我们在这里对这些重依赖子模块做了桩处理：桩模块仅在原模块无法正常导入时才会生效，因此在完整的GPU服务器安装环境中，这个conftest不会产生任何影响。

注：本文件刻意保持极简设计，如果某个测试确实需要用到某一个被桩替换的模块（比如测试Ray RPC功能的用例），只需要在对应测试中移除对应的桩即可。
"""

from __future__ import annotations

import sys
import types


def _ensure_stub(name: str, attrs: dict | None = None) -> types.ModuleType:
    """Insert a lightweight placeholder package into ``sys.modules``.

    Safe to call multiple times: if the module is already loaded (real
    install present), we leave it alone and just back-fill missing
    attributes from ``attrs``.
    """
    if name in sys.modules:
        mod = sys.modules[name]
    else:
        mod = types.ModuleType(name)
        # Mark as a package so that ``from name.sub import ...`` doesn't
        # try to load missing files; submodules are stubbed explicitly.
        mod.__path__ = []
        sys.modules[name] = mod
    if attrs:
        for k, v in attrs.items():
            if not hasattr(mod, k):
                setattr(mod, k, v)
    return mod


class _StubAttr:
    """No-op placeholder for objects that the tests don't touch."""

    pass


class _RealWorldEnvStub:
    @staticmethod
    def realworld_setup() -> None:
        return None


def _try_import(name: str) -> bool:
    try:
        __import__(name)
        return True
    except Exception:
        return False


# ── 1. rlinf.scheduler ────────────────────────────────────────────
# Import path:  rlinf.envs.realworld.galaxear.r1_pro_env
#                 -> from rlinf.scheduler import GalaxeaR1ProHWInfo, WorkerInfo
#                 -> rlinf.scheduler.channel.channel
#                 -> rlinf.scheduler.collective.async_work
#                 -> AttributeError on torch.distributed.Work (JetPack torch).
if not _try_import("rlinf.scheduler"):
    # NOTE: Do NOT stub the parent ``rlinf`` package -- the real one must
    # load so that ``rlinf.envs.realworld.galaxear`` resolves through the
    # filesystem.  Only inject the leaf module that fails to import.
    _ensure_stub("rlinf.scheduler", {
        "Cluster": _StubAttr,
        "NodePlacementStrategy": _StubAttr,
        "Worker": _StubAttr,
        "WorkerInfo": _StubAttr,
        "GalaxeaR1ProHWInfo": _StubAttr,
    })


# ── 2. rlinf.envs.realworld submodules (franka / xsquare / realworld_env) ──
# Import path:  rlinf.envs.realworld.__init__
#                 -> from .franka import FrankaEnv, ...    (needs ray/gym)
#                 -> from .xsquare import Turtle2Env, ...  (needs ray/gym)
#                 -> from .realworld_env import RealWorldEnv (needs psutil/filelock)
# We only stub the ones that fail to import; the .galaxear subpackage is
# always loaded for real because that's what the tests target.
#
# Importantly, we let Python load ``rlinf``, ``rlinf.envs`` and
# ``rlinf.envs.realworld`` from their real on-disk paths -- otherwise
# ``rlinf.envs.realworld.galaxear`` cannot be imported.

if not _try_import("rlinf.envs.realworld.franka"):
    _franka_tasks = types.ModuleType("rlinf.envs.realworld.franka.tasks")
    _ensure_stub("rlinf.envs.realworld.franka", {
        "FrankaEnv": _StubAttr,
        "FrankaRobotConfig": _StubAttr,
        "FrankaRobotState": _StubAttr,
        "tasks": _franka_tasks,
    })
    sys.modules["rlinf.envs.realworld.franka.tasks"] = _franka_tasks

if not _try_import("rlinf.envs.realworld.xsquare"):
    _xsq_tasks = types.ModuleType("rlinf.envs.realworld.xsquare.tasks")
    _ensure_stub("rlinf.envs.realworld.xsquare", {
        "Turtle2Env": _StubAttr,
        "Turtle2RobotConfig": _StubAttr,
        "Turtle2RobotState": _StubAttr,
        "tasks": _xsq_tasks,
    })
    sys.modules["rlinf.envs.realworld.xsquare.tasks"] = _xsq_tasks

if not _try_import("rlinf.envs.realworld.realworld_env"):
    _ensure_stub("rlinf.envs.realworld.realworld_env", {
        "RealWorldEnv": _RealWorldEnvStub,
    })
