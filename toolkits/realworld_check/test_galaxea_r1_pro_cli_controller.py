#!/usr/bin/env python3
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

"""Joint + gripper interactive CLI for Galaxea R1 Pro.

Per design doc ``bt/docs/rwRL/test_galaxea_r1_pro_cli_controller.md``.

The tool replaces the Pi0.5 policy at the input end of the RLinf
real-robot pipeline with a human at the keyboard.  Everything else --
:class:`SafetySupervisor` (L1-L5), :class:`ActionDispatcher` (joint /
ee), :class:`GripperMixer` ([-1, 1] -> physical pct), the ROS 2 publish
itself -- runs unchanged.

Run with ``--help`` for the argument surface, or ``--dummy`` for a
ROS-free / Ray-free smoke test that's safe to run anywhere.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np

# Make this script self-bootstrapping when invoked directly:
# ``python toolkits/realworld_check/test_galaxea_r1_pro_cli_controller.py``
# from the repo root.  In CI / pytest the path is already on sys.path.
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


def _bootstrap_minimal_env_stubs() -> None:
    """Mirror tests/unit_tests/conftest.py for live (non-pytest) runs.

    On JetPack-shipped torch the ``rlinf.scheduler.collective`` import
    chain explodes (no ``torch.distributed.Work``).  The CLI doesn't
    actually need scheduler / channel / Worker -- it talks to ROS via
    the backend abstraction, not Ray RPC -- so we stub the parts that
    only matter at training time.  The stubs only kick in when the
    real module fails to import; on a full GPU-server install with
    a normal torch, this whole function is a no-op.
    """
    import types

    def _try(name: str) -> bool:
        try:
            __import__(name)
            return True
        except Exception:
            return False

    def _stub(name: str, attrs: dict | None = None) -> types.ModuleType:
        if name in sys.modules:
            mod = sys.modules[name]
        else:
            mod = types.ModuleType(name)
            mod.__path__ = []
            sys.modules[name] = mod
        if attrs:
            for k, v in attrs.items():
                if not hasattr(mod, k):
                    setattr(mod, k, v)
        return mod

    class _StubAttr:
        pass

    class _RealWorldEnvStub:
        @staticmethod
        def realworld_setup() -> None:
            return None

    if not _try("rlinf.scheduler"):
        _stub(
            "rlinf.scheduler",
            {
                "Cluster": _StubAttr,
                "NodePlacementStrategy": _StubAttr,
                "Worker": _StubAttr,
                "WorkerInfo": _StubAttr,
                "GalaxeaR1ProHWInfo": _StubAttr,
            },
        )
    if not _try("rlinf.envs.realworld.franka"):
        _ft = types.ModuleType("rlinf.envs.realworld.franka.tasks")
        _stub(
            "rlinf.envs.realworld.franka",
            {
                "FrankaEnv": _StubAttr,
                "FrankaRobotConfig": _StubAttr,
                "FrankaRobotState": _StubAttr,
                "tasks": _ft,
            },
        )
        sys.modules["rlinf.envs.realworld.franka.tasks"] = _ft
    if not _try("rlinf.envs.realworld.xsquare"):
        _xt = types.ModuleType("rlinf.envs.realworld.xsquare.tasks")
        _stub(
            "rlinf.envs.realworld.xsquare",
            {
                "Turtle2Env": _StubAttr,
                "Turtle2RobotConfig": _StubAttr,
                "Turtle2RobotState": _StubAttr,
                "tasks": _xt,
            },
        )
        sys.modules["rlinf.envs.realworld.xsquare.tasks"] = _xt
    if not _try("rlinf.envs.realworld.realworld_env"):
        _stub(
            "rlinf.envs.realworld.realworld_env",
            {
                "RealWorldEnv": _RealWorldEnvStub,
            },
        )


_bootstrap_minimal_env_stubs()


from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import (  # noqa: E402
    ActionDispatcher,
    EePoseDispatcher,
    JointDeltaDispatcher,
    JointStateDispatcher,
    build_action_dispatcher,
)
from rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer import (  # noqa: E402
    GripperMixer,
)
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (  # noqa: E402
    GalaxeaR1ProRobotState,
)
from rlinf.envs.realworld.galaxear.r1_pro_safety import (  # noqa: E402
    GalaxeaR1ProSafetySupervisor,
    SafetyConfig,
    SafetyInfo,
    build_safety_config,
)
from toolkits.realworld_check._galaxea_backends import (  # noqa: E402
    ControllerBackend,
    build_backend,
)
from toolkits.realworld_check._galaxea_cli_input import (  # noqa: E402
    CliInputError,
    InputParser,
    ParsedCommand,
)
from toolkits.realworld_check._galaxea_cli_state import (  # noqa: E402
    StateMonitor,
    TopologySpec,
)
logger = logging.getLogger("r1_pro_cli")
logger.setLevel(logging.DEBUG)


# ────────────────────────── CliConfig ───────────────────────────


@dataclass
class CliConfig:
    """Resolved CLI runtime configuration.

    Built from argparse + defaults.  Passed to :class:`CliContext`
    which threads it through every component.  Mutable at runtime via
    the ``set`` REPL command (small set of allowed keys, see
    :meth:`CliContext.handle_set_command`).
    """

    # Backend
    backend_kind: str = "rclpy"
    dummy: bool = False
    controller_node_rank: int = 0
    ros_domain_id: int = 41
    ros_localhost_only: bool = False
    galaxea_install_path: str = "~/galaxea/install"

    # Mode
    use_joint_mode: bool = True
    use_right_arm: bool = True
    use_left_arm: bool = False
    use_torso: bool = False
    use_chassis: bool = False
    no_gripper: bool = False
    mobiman_launch_mode: str = "joint"

    # Joint sub-mode (per r1pro6op47.md §3.6)
    # When True (and use_joint_mode=True), the CLI builds a
    # JointDeltaDispatcher: action[:7] is interpreted as a per-step
    # joint delta in [-1, 1] -> rad.  ActionComposer hides this from
    # the operator -- typing `j r 0.5 ...` still means "absolute
    # target rad", composer back-computes the delta.  See §3.6.
    joint_delta_mode: bool = False
    joint_delta_scale_right: list[float] = field(
        default_factory=lambda: [0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20],
    )
    joint_delta_scale_left: Optional[list[float]] = None

    # Gripper business range
    gripper_min_pct: float = 0.0
    gripper_max_pct: float = 90.0

    # Joint mode bounds (default to URDF-derived SafetyConfig values)
    arm_q_min_right: list[float] = field(
        default_factory=lambda: list(
            SafetyConfig().right_arm_q_min,
        )
    )
    arm_q_max_right: list[float] = field(
        default_factory=lambda: list(
            SafetyConfig().right_arm_q_max,
        )
    )
    arm_q_min_left: list[float] = field(
        default_factory=lambda: list(
            SafetyConfig().left_arm_q_min,
        )
    )
    arm_q_max_left: list[float] = field(
        default_factory=lambda: list(
            SafetyConfig().left_arm_q_max,
        )
    )
    arm_qvel_max: list[float] = field(
        default_factory=lambda: [
            3.0,
            3.0,
            3.0,
            3.0,
            5.0,
            5.0,
            5.0,
        ]
    )
    home_q_right: list[float] = field(
        default_factory=lambda: [
            0.0,
            0.3,
            0.0,
            -1.5,
            0.0,
            1.8,
            0.0,
        ]
    )
    home_q_left: Optional[list[float]] = None

    # EE mode bounds (used when use_joint_mode=False)
    ee_min_right: list[float] = field(
        default_factory=lambda: [0.20, -0.40, 0.10],
    )
    ee_max_right: list[float] = field(
        default_factory=lambda: [0.70, 0.40, 0.80],
    )
    ee_min_left: list[float] = field(
        default_factory=lambda: [0.20, -0.40, 0.10],
    )
    ee_max_left: list[float] = field(
        default_factory=lambda: [0.70, 0.40, 0.80],
    )
    home_pose_right: Optional[list[float]] = None
    home_pose_left: Optional[list[float]] = None

    # Safety
    operator_heartbeat_timeout_ms: float = 30000.0
    bms_low_battery_threshold_pct: float = 20.0
    feedback_stale_threshold_ms: float = 3000.0
    estop_swd_value_down: bool = False  # default off; rely on ``brake``

    # CLI runtime
    strict_topo: bool = False
    strict_heartbeat: bool = False
    strict_runtime: bool = False
    heartbeat_interval_ms: int = 2000
    log_file: Optional[str] = None
    # ROS 2 DDS discovery is asynchronous; the first
    # ``get_publisher_count`` call after a node is created frequently
    # returns 0 for topics that are in fact being published.  The CLI
    # therefore polls the topo check up to this many seconds before
    # declaring it failed.  Default 3.0s is conservative for a healthy
    # local DDS; slow / cross-host setups can raise it.
    topo_discovery_timeout_s: float = 3.0


# ────────────────────────── CliContext ──────────────────────────


@dataclass
class CliContext:
    """Mutable runtime context shared by ReplLoop / ActionComposer."""

    cfg: CliConfig
    backend: ControllerBackend
    safety: GalaxeaR1ProSafetySupervisor
    dispatcher: ActionDispatcher
    mixer: GripperMixer
    monitor: StateMonitor
    last_safety_info: Optional[SafetyInfo] = None
    running: bool = True


# ─────────────────────── ActionComposer ─────────────────────────


class ActionComposer:
    """Translate :class:`ParsedCommand` into a normalised action vector.

    Stateful: keeps the per-side most-recent normalised vector so
    successive commands "stack" (e.g. set joints once, then change
    only the gripper, then change just one joint).  This matches the
    ergonomics of bring-up where the operator wants to tweak one
    dimension at a time, NOT replay a full 8-D vector each command.
    """

    def __init__(self, ctx: CliContext):
        self.ctx = ctx
        self._reset_latest()

    def _reset_latest(self) -> None:
        d = self._per_arm_dim()
        self._latest: dict[str, np.ndarray] = {
            "right": np.zeros(d, dtype=np.float32),
            "left": np.zeros(d, dtype=np.float32),
        }

    def _per_arm_dim(self) -> int:
        return 7 if self.ctx.cfg.no_gripper else 8

    def _q_bounds(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.ctx.cfg
        if side == "right":
            return (
                np.asarray(cfg.arm_q_min_right, dtype=np.float32)[:7],
                np.asarray(cfg.arm_q_max_right, dtype=np.float32)[:7],
            )
        return (
            np.asarray(cfg.arm_q_min_left, dtype=np.float32)[:7],
            np.asarray(cfg.arm_q_max_left, dtype=np.float32)[:7],
        )

    def _q_rad_to_norm(self, side: str, q_rad: np.ndarray) -> np.ndarray:
        """Inverse mapping rad -> norm for the **abs** dispatcher.

        The delta dispatcher uses a different inverse via
        :meth:`_q_rad_target_to_delta_norm`; see r1pro6op47.md §3.6.5
        for the reverse-calc rationale.
        """
        q_min, q_max = self._q_bounds(side)
        # Inverse of JointStateDispatcher._unnormalize_arm.
        return 2.0 * (q_rad - q_min) / np.maximum(q_max - q_min, 1e-9) - 1.0

    def _delta_scale(self, side: str) -> np.ndarray:
        cfg = self.ctx.cfg
        scale = (
            cfg.joint_delta_scale_right
            if side == "right"
            else (
                cfg.joint_delta_scale_left
                if cfg.joint_delta_scale_left is not None
                else cfg.joint_delta_scale_right
            )
        )
        return np.asarray(scale, dtype=np.float32).reshape(-1)[:7]

    def _q_rad_target_to_delta_norm(
        self,
        side: str,
        q_target_rad: np.ndarray,
        q_current_rad: np.ndarray,
    ) -> np.ndarray:
        """Reverse calc for **delta sub-mode**, per design doc §3.6 / CLI:

            delta_norm = clip((q_target - q_current) / joint_delta_scale, -1, 1)

        Operator types ``j r 0.5 ...`` (absolute target rad); composer
        translates to the per-step delta_norm the model would have
        emitted.  If the target is more than one step away, the clip
        bounds it and the operator hits Enter again on the next step.
        """
        scale = self._delta_scale(side)
        delta_rad = q_target_rad - q_current_rad
        return np.clip(
            delta_rad / np.maximum(scale, 1e-9),
            -1.0,
            1.0,
        ).astype(np.float32)

    def _ee_bounds(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        cfg = self.ctx.cfg
        if side == "right":
            return (
                np.asarray(cfg.ee_min_right, dtype=np.float32)[:3],
                np.asarray(cfg.ee_max_right, dtype=np.float32)[:3],
            )
        return (
            np.asarray(cfg.ee_min_left, dtype=np.float32)[:3],
            np.asarray(cfg.ee_max_left, dtype=np.float32)[:3],
        )

    def _xyz_m_to_norm(self, side: str, xyz: np.ndarray) -> np.ndarray:
        lo, hi = self._ee_bounds(side)
        return 2.0 * (xyz - lo) / np.maximum(hi - lo, 1e-9) - 1.0

    # ── Core API ──────────────────────────────────────────────

    def reset_after_mode_switch(self) -> None:
        self._reset_latest()

    def compose(self, cmd: ParsedCommand) -> np.ndarray:
        """Apply *cmd* into the per-side latest vector and return the
        full assembled action (single-arm or dual-arm layout)."""
        side = cmd.side or "right"
        if cmd.verb == "joint":
            self._compose_joint_abs(side, cmd.payload)
        elif cmd.verb == "jn":
            self._compose_joint_norm(side, cmd.payload)
        elif cmd.verb == "jd":
            self._compose_joint_delta(side, cmd.payload)
        elif cmd.verb == "gripper":
            self._compose_gripper_pct(side, cmd.payload)
        elif cmd.verb == "gn":
            self._compose_gripper_norm(side, cmd.payload)
        elif cmd.verb == "pose":
            self._compose_pose_abs(side, cmd.payload)
        elif cmd.verb == "pn":
            self._compose_pose_norm(side, cmd.payload)
        else:
            raise CliInputError(
                cmd.raw,
                f"compose() called with non-action verb '{cmd.verb}'",
            )
        return self._assemble()

    def _assemble(self) -> np.ndarray:
        cfg = self.ctx.cfg
        parts = []
        if cfg.use_right_arm:
            parts.append(self._latest["right"])
        if cfg.use_left_arm:
            parts.append(self._latest["left"])
        if not parts:
            return np.zeros(1, dtype=np.float32)
        return np.concatenate(parts).astype(np.float32)

    # ── Joint-mode composers ──────────────────────────────────

    def _read_cur_q(self, side: str) -> np.ndarray:
        """Read the latest feedback qpos for *side* (zeros if unset)."""
        state = self.ctx.backend.get_state()
        return np.asarray(
            state.get_arm_qpos(side),
            dtype=np.float32,
        ).reshape(-1)[:7]

    def _compose_joint_abs(self, side: str, payload: dict) -> None:
        """``j r ...`` semantics in CLI = "absolute joint target in rad".

        For abs sub-mode this stores ``_q_rad_to_norm(q_rad)``.
        For delta sub-mode the operator's intent is still "go to this
        absolute target", so we back-compute the per-step delta_norm
        the model would have emitted (per r1pro6op47.md §3.6, CLI
        compatibility section).  If the target is more than one step
        away, the per-axis clip in :meth:`_q_rad_target_to_delta_norm`
        bounds the delta to ``[-1, 1]`` and the operator just types
        again.
        """
        if not self.ctx.cfg.use_joint_mode:
            raise CliInputError(
                "",
                "joint command requires mode=joint; switch with 'mode joint'",
            )
        delta_mode = bool(self.ctx.cfg.joint_delta_mode)
        logger.info(f">>>compose_joint_abs cmd payload before compose: {side} : {payload}")
        if payload["mode"] == "abs":
            q_rad = np.asarray(payload["q_rad"], dtype=np.float32)
            if delta_mode:
                cur_q = self._read_cur_q(side)
                self._latest[side][:7] = self._q_rad_target_to_delta_norm(
                    side,
                    q_rad,
                    cur_q,
                )
            else:
                self._latest[side][:7] = self._q_rad_to_norm(side, q_rad)
        elif payload["mode"] == "single_abs":
            i = payload["idx"]
            q = float(payload["q_rad"])
            if delta_mode:
                cur_q = self._read_cur_q(side)
                # Build a target vector that keeps all other joints at
                # their current value; only the touched joint moves.
                q_target = cur_q.copy()
                q_target[i] = q
                self._latest[side][:7] = self._q_rad_target_to_delta_norm(
                    side,
                    q_target,
                    cur_q,
                )
            else:
                # Build a target vector that keeps all other joints at
                # their current value; only the touched joint moves.
                # Then normalise the full 7-vector through the same
                # inverse mapping the abs branch above uses, so the
                # downstream JointStateDispatcher receives a consistent
                # `_latest[side][:7]` regardless of single- vs multi-axis
                # input.  ``_q_rad_to_norm`` returns a (7,) ndarray; do
                # NOT wrap in ``float()`` (that raises on length>1).
                cur_q = self._read_cur_q(side)
                q_target = cur_q.copy()
                q_target[i] = q
                self._latest[side][:7] = self._q_rad_to_norm(side, q_target)

    def _compose_joint_norm(self, side: str, payload: dict) -> None:
        """``jn r ...`` is the raw normalised vector the model emits.

        In abs sub-mode it's interpreted as "absolute target norm";
        in delta sub-mode it's interpreted as "delta_norm" -- exactly
        what JointDeltaDispatcher consumes.  Either way the value
        flows through unchanged.
        """
        if not self.ctx.cfg.use_joint_mode:
            raise CliInputError("", "jn requires mode=joint")
        if payload["mode"] == "norm":
            self._latest[side][:7] = np.asarray(
                payload["q_norm"],
                dtype=np.float32,
            )
        else:  # single_norm
            self._latest[side][payload["idx"]] = float(payload["q_norm"])

    def _compose_joint_delta(self, side: str, payload: dict) -> None:
        """``jd r idx delta_rad`` always means "increment joint idx by
        delta_rad starting from CURRENT feedback qpos".

        In abs sub-mode this stores ``_q_rad_to_norm(q_current + delta)``.
        In delta sub-mode the per-axis answer is just
        ``delta_norm = clip(delta_rad / scale, -1, 1)`` without
        touching other axes.
        """
        if not self.ctx.cfg.use_joint_mode:
            raise CliInputError("", "jd requires mode=joint")
        idx = payload["idx"]
        delta_rad = float(payload["delta_rad"])
        if self.ctx.cfg.joint_delta_mode:
            scale = float(self._delta_scale(side)[idx])
            n = float(np.clip(delta_rad / max(scale, 1e-9), -1.0, 1.0))
            # Other joints stay at "no movement" -> delta_norm = 0.
            self._latest[side][:7] = 0.0
            self._latest[side][idx] = n
        else:
            cur_q = self._read_cur_q(side)
            new_q = cur_q.copy()
            new_q[idx] += delta_rad
            self._latest[side][:7] = self._q_rad_to_norm(side, new_q)

    # ── EE-mode composers ─────────────────────────────────────

    def _compose_pose_abs(self, side: str, payload: dict) -> None:
        if self.ctx.cfg.use_joint_mode:
            raise CliInputError("", "pose requires mode=ee")
        pose = np.asarray(payload["pose"], dtype=np.float32)
        xyz_norm = self._xyz_m_to_norm(side, pose[:3])
        self._latest[side][:3] = xyz_norm
        # Quat passed through verbatim (the dispatcher will L2-normalise).
        self._latest[side][3:7] = pose[3:7]

    def _compose_pose_norm(self, side: str, payload: dict) -> None:
        if self.ctx.cfg.use_joint_mode:
            raise CliInputError("", "pn requires mode=ee")
        self._latest[side][:7] = np.asarray(
            payload["pose_norm"],
            dtype=np.float32,
        )

    # ── Gripper composers ─────────────────────────────────────

    def _compose_gripper_pct(self, side: str, payload: dict) -> None:
        if self.ctx.cfg.no_gripper:
            raise CliInputError("", "gripper command but no_gripper=True")
        if payload["mode"] == "open":
            pct = self.ctx.cfg.gripper_max_pct
        elif payload["mode"] == "close":
            pct = self.ctx.cfg.gripper_min_pct
        else:
            pct = float(payload["pct"])
        # pct -> norm via mixer's inverse map.
        self._latest[side][7] = self.ctx.mixer.pct_to_obs11(pct)

    def _compose_gripper_norm(self, side: str, payload: dict) -> None:
        if self.ctx.cfg.no_gripper:
            raise CliInputError("", "gn command but no_gripper=True")
        self._latest[side][7] = float(np.clip(payload["value"], -1.0, 1.0))


# ─────────────────────── REPL main loop ─────────────────────────


# Set of action verbs that go through the full safety + dispatch
# pipeline.  Distinguished from "meta" verbs that only read state.
_ACTION_VERBS = frozenset(
    {
        "joint",
        "jn",
        "jd",
        "gripper",
        "gn",
        "pose",
        "pn",
    }
)


class ReplLoop:
    """The interactive REPL.

    Owns the ``CliContext``, the parser, the composer, the monitor,
    and a daemon heartbeat thread.  ``run()`` is a blocking call;
    returns the exit code (0 on clean shutdown, 2 on strict-topo
    failure, 1 on uncaught error).

    Tests can drive the REPL programmatically by calling
    :meth:`handle_line` directly -- no stdin needed.
    """

    def __init__(
        self,
        ctx: CliContext,
        *,
        input_fn: Optional[Callable[[str], str]] = None,
        output_fn: Optional[Callable[[str], None]] = None,
    ):
        self.ctx = ctx
        self.parser = InputParser()
        self.composer = ActionComposer(ctx)
        self._input_fn = input_fn or input
        self._output_fn = output_fn or (lambda s: print(s, flush=True))
        self._heartbeat_thread: Optional[threading.Thread] = None

    # ── Output helpers ─────────────────────────────────────────

    def emit(self, msg: str) -> None:
        self._output_fn(msg)

    def emit_info(self, msg: str) -> None:
        self.emit(StateMonitor.info(msg))

    def emit_warn(self, msg: str) -> None:
        self.emit(StateMonitor.warn(msg))

    def emit_error(self, msg: str) -> None:
        self.emit(StateMonitor.error(msg))

    # ── Main entry ────────────────────────────────────────────

    def run(self) -> int:
        if self.ctx.cfg.use_joint_mode:
            mode_label = (
                "joint_delta" if self.ctx.cfg.joint_delta_mode else "joint"
            )
        else:
            mode_label = "ee"
        self.emit_info(
            f"Backend: {self.ctx.backend.kind}   "
            f"ROS_DOMAIN_ID={self.ctx.cfg.ros_domain_id}   "
            f"mode={mode_label}"
        )
        self.ctx.safety.heartbeat()
        # Run topo check up front.  In strict mode any WARN exits.
        topo_ok = self._run_topo_check(strict=self.ctx.cfg.strict_topo)
        if not topo_ok:
            # Cleanly shut down the backend so rclpy's C++ destructors
            # don't emit ``terminate called without an active exception``
            # when Python interpreter exits.
            try:
                self._shutdown()
            except Exception:  # noqa: BLE001
                pass
            return 2

        if not self.ctx.cfg.strict_heartbeat:
            self._start_heartbeat_thread()

        self.emit_info("REPL ready.  Type '?' for help.")
        try:
            while self.ctx.running:
                try:
                    line = self._input_fn(self._prompt())
                except EOFError:
                    self.emit_info("EOF -- shutting down.")
                    break
                except KeyboardInterrupt:
                    self.emit_warn("Ctrl+C -- applying brake then exiting.")
                    try:
                        self.ctx.backend.apply_brake(True)
                    except Exception:  # noqa: BLE001
                        pass
                    break
                self.handle_line(line)
        finally:
            self._shutdown()
        return 0

    # ── Per-line entry (used by tests) ─────────────────────────

    def handle_line(self, line: str) -> None:
        try:
            cmd = self.parser.parse(line)
        except CliInputError as e:
            self.emit_error(f"parse error: {e.hint}")
            return
        self._dispatch_command(cmd)

    def _dispatch_command(self, cmd: ParsedCommand) -> None:
        verb = cmd.verb
        if verb == "noop":
            return
        if verb == "quit":
            self.ctx.running = False
            return
        if verb == "help":
            self.emit(self.parser.help())
            return
        if verb == "state":
            self._handle_state(cmd)
            return
        if verb == "safety":
            self._handle_safety_query()
            return
        if verb == "topo":
            self._run_topo_check(strict=False)
            return
        if verb == "mode":
            self._handle_mode_switch(cmd)
            return
        if verb == "brake":
            self._handle_brake(cmd)
            return
        if verb == "set":
            self._handle_set(cmd)
            return
        if verb == "home":
            self._handle_home(cmd)
            return
        if verb in _ACTION_VERBS:
            self._handle_action(cmd)
            return
        self.emit_warn(f"unhandled verb '{verb}'")

    def _handle_home(self, cmd: ParsedCommand) -> None:
        """Reset arms to home via dispatcher.reset_to_safe_pose.

        We deliberately bypass the action / safety pipeline here --
        ``reset_to_safe_pose`` itself goes through the dispatcher's
        own bounds-aware reset choreography (designed exactly for
        this case in r1pro6op47.md §3.2.4 / §3.3.5).  Heartbeat is
        still poked so L5 stays calm.
        """
        self.ctx.safety.heartbeat()
        try:
            self.ctx.dispatcher.reset_to_safe_pose(self.ctx.backend.get_state())
            self.emit_info(f"home reset issued (side={cmd.side or 'all'}).")
        except Exception as e:  # noqa: BLE001
            self.emit_error(f"home failed: {e}")
            if self.ctx.cfg.strict_runtime:
                self.ctx.running = False
            return
        # Reset composer so subsequent jd starts from feedback (which
        # the dispatcher just rewrote to home_q) rather than the
        # previous latest-norm.
        self.composer.reset_after_mode_switch()
        # Echo the latest TX (there may be more than one if both arm
        # and gripper were home'd; just show the last for brevity).
        tx = getattr(self.ctx.backend, "tx_log", [])
        if tx:
            self.emit(
                self.ctx.monitor.tx_line(
                    tx[-1],
                    dry=self.ctx.cfg.dummy,
                )
            )

    # ── Action pipeline (compose -> safety -> dispatch) ────────

    def _handle_action(self, cmd: ParsedCommand) -> None:
        # 1) Heartbeat -- per design doc §4.6.3 (per-action hook).
        self.ctx.safety.heartbeat()
        # 2) Compose the normalised action vector.
        try:
            action = self.composer.compose(cmd)
        except CliInputError as e:
            self.emit_error(f"compose error: {e.hint}")
            return
        # 3) Get latest state and run safety.
        state = self.ctx.backend.get_state()
        info = self._validate_safety(action, state)
        self.ctx.last_safety_info = info
        self.emit(self.ctx.monitor.safety_overlay(info))
        # 4) Decide what to dispatch.
        if info.emergency_stop or info.safe_stop:
            self.emit_warn("emergency/safe stop -- applying brake.")
            try:
                self.ctx.backend.apply_brake(True)
            except Exception:  # noqa: BLE001
                logger.exception("apply_brake failed")
            return
        if info.soft_hold:
            self.emit_warn("soft_hold -- skipping dispatch this step.")
            return
        # 5) Only send the components the user actually asked for.
        # Verb-class -> components mapping (per design doc §4.5):
        #   joint / jn / jd  -> just send_arm_joints (no gripper TX)
        #   pose  / pn       -> just send_arm_pose
        #   gripper / gn     -> just send_gripper
        # This avoids the "I touched J0 and the gripper jumped to mid-pct"
        # UX trap that a naive ``dispatcher.dispatch()`` produces.
        side = cmd.side or "right"
        try:
            self._dispatch_components(cmd.verb, side, info.safe_action, state)
        except Exception as e:  # noqa: BLE001
            self.emit_error(f"dispatch failed: {e}")
            if self.ctx.cfg.strict_runtime:
                self.ctx.running = False
            return
        # Echo the latest TX.
        tx = getattr(self.ctx.backend, "tx_log", [])
        if tx:
            self.emit(
                self.ctx.monitor.tx_line(
                    tx[-1],
                    dry=self.ctx.cfg.dummy,
                )
            )

    def _dispatch_components(
        self,
        verb: str,
        side: str,
        safe_action: np.ndarray,
        state: GalaxeaR1ProRobotState,
    ) -> None:
        """Translate the post-safety action into one (or more) backend
        send_* calls based on which verb was issued.

        We borrow the dispatcher's un-normalisation helpers so the
        translation is byte-identical to the training-time path; we
        just bypass dispatcher.dispatch() (which would send EVERY
        component every time, regardless of which one the user
        intended to change).
        """
        per_arm_dim = 7 if self.ctx.cfg.no_gripper else 8
        # Locate the per-side slice in the assembled action vector.
        idx = 0
        if self.ctx.cfg.use_right_arm and side != "right":
            idx += per_arm_dim
        # If the slice for the requested side wasn't built (side not
        # enabled), fall back to the first slice present.
        if (side == "left" and not self.ctx.cfg.use_left_arm) or (
            side == "right" and not self.ctx.cfg.use_right_arm
        ):
            self.emit_warn(f"side='{side}' is not enabled in this CLI session")
            return
        slc = safe_action[idx : idx + per_arm_dim]

        disp = self.ctx.dispatcher
        if verb in ('joint', 'jn', 'jd'):
            if not isinstance(disp, JointStateDispatcher):
                # JointDeltaDispatcher is a subclass of JointStateDispatcher,
                # so this isinstance covers both joint sub-modes; the only
                # case that hits this branch is ee mode.
                self.emit_error(
                    "joint command but current mode is ee; type 'mode joint'"
                )
                return
            q7_norm = slc[:7]
            # Per r1pro6op47.md §3.6: in delta sub-mode the dispatcher
            # un-normalises by `q_target = clip(q_current + a*scale, ...)`,
            # which needs the current feedback qpos.  Keep abs sub-mode's
            # state-free path unchanged so existing tests still pass.
            if isinstance(disp, JointDeltaDispatcher):
                cur_q = np.asarray(
                    state.get_arm_qpos(side),
                    dtype=np.float32,
                ).reshape(-1)[:7]
                q_target = disp._compute_q_target(side, cur_q, q7_norm)
            else:
                logger.info(f">>>_dispatch_components()/before disp._unnormalize_arm: {q7_norm}")
                q_target = disp._unnormalize_arm(side, q7_norm)
            logger.info(f">>>_dispatch_components()/('joint', 'jn', 'jd')/q_target: {q_target}")
            self.ctx.backend.send_arm_joints(
                side,
                q_target.tolist(),
                np.asarray(self.ctx.cfg.arm_qvel_max, dtype=np.float32).tolist(),
            )
        elif verb in ('pose', 'pn'):
            if not isinstance(disp, EePoseDispatcher):
                self.emit_error(
                    "pose command but current mode is joint; type 'mode ee'"
                )
                return
            xyz = disp._unnormalize_xyz(side, slc[:3])
            quat = EePoseDispatcher._normalize_quat(slc[3:7])
            pose7 = np.concatenate([xyz, quat]).astype(np.float32)
            logger.info(f">>>_dispatch_components()/('pose', 'pn')/pose7: {pose7}")
            self.ctx.backend.send_arm_pose(side, pose7.tolist())
        elif verb in ('gripper', 'gn'):
            if self.ctx.cfg.no_gripper:
                self.emit_error("gripper command but no_gripper=True")
                return
            # Gripper slot is at index 7 of the per-arm slice.
            if slc.size < 8:
                self.emit_error("gripper slot missing from action layout")
                return
            pct = self.ctx.mixer.action11_to_pct(float(slc[7]))
            logger.info(f">>>_dispatch_components()/('gripper', 'gn')/pose7: {pct}")
            self.ctx.backend.send_gripper(side, pct)
        else:
            self.emit_warn(f"_dispatch_components: unhandled verb '{verb}'")

    def _validate_safety(
        self,
        action: np.ndarray,
        state: GalaxeaR1ProRobotState,
    ) -> SafetyInfo:
        """Lightweight safety pipeline aligned with
        ``GalaxeaR1ProEnv._safety_validate_dispatcher`` (r1pro6op47.md
        §6.6).

        Joint-mode and ee+quat-mode actions don't fit the legacy L2/L3
        ``xyz/rpy`` schema, so we run only L1 (clip+NaN), L4 (per-step
        cap), and L5 (BMS / heartbeat / status / estop).  Position
        bounds are enforced by the dispatcher itself when un-normalising.
        """
        a = np.asarray(action, dtype=np.float32).reshape(-1)
        logger.info(f">>>action befor val: {a} ,\n >>>State: {state} \n\n")
        info = SafetyInfo(raw_action=a.copy(), safe_action=a.copy())
        # L1
        if not np.all(np.isfinite(info.safe_action)):
            info.safe_action = np.zeros_like(info.safe_action)
            info.emergency_stop = True
            info.reason.append("L1:non_finite_action")
        clipped = np.clip(info.safe_action, -1.0, 1.0)
        if not np.array_equal(clipped, info.safe_action):
            info.clipped = True
            info.reason.append("L1:clipped_to_unit_box")
        info.safe_action = clipped
        # L5
        sup_cfg = self.ctx.safety.cfg
        bms_pct = float(state.bms.get("capital_pct", 100.0))
        if bms_pct < sup_cfg.bms_low_battery_threshold_pct:
            info.safe_stop = True
            info.reason.append(f"L5:bms_low {bms_pct:.1f}pct")
        for src, age in (state.feedback_age_ms or {}).items():
            if age > sup_cfg.feedback_stale_threshold_ms:
                info.soft_hold = True
                info.reason.append(f"L5:stale {src} {age:.0f}ms")
        cs = state.controller_signal or {}
        if sup_cfg.estop_swd_value_down and int(cs.get("swd", 0)):
            info.emergency_stop = True
            info.reason.append("L5:SWD_DOWN")
        for side, errs in (state.status_errors or {}).items():
            if errs:
                info.soft_hold = True
                info.reason.append(f"L5:status_errors_{side}={list(errs)[:3]}")
        # Operator heartbeat
        now_ms = time.monotonic() * 1000.0
        hb_age = now_ms - self.ctx.safety._operator_heartbeat_ms  # type: ignore[attr-defined]
        if hb_age > sup_cfg.operator_heartbeat_timeout_ms:
            info.soft_hold = True
            info.reason.append(f"L5:operator_hb_age={hb_age:.0f}ms")
        if info.emergency_stop or info.safe_stop:
            info.safe_action = np.zeros_like(info.safe_action)
        info.metrics = {
            "safety/clip_ratio": float(info.clipped),
            "safety/soft_hold": float(info.soft_hold),
            "safety/safe_stop": float(info.safe_stop),
            "safety/emergency_stop": float(info.emergency_stop),
            "safety/reason_count": float(len(info.reason)),
            "hw/bms_capital_pct": bms_pct,
        }
        logger.info(f">>>info.safe_action after val: {info.safe_action}")
        return info

    # ── Meta handlers ─────────────────────────────────────────

    def _handle_state(self, cmd: ParsedCommand) -> None:
        state = self.ctx.backend.get_state()
        self.emit(self.ctx.monitor.pretty_state(state, side=cmd.side))

    def _handle_safety_query(self) -> None:
        if self.ctx.last_safety_info is None:
            self.emit_info("no SafetyInfo recorded yet (run an action first).")
            return
        self.emit(self.ctx.monitor.safety_overlay(self.ctx.last_safety_info))

    def _handle_mode_switch(self, cmd: ParsedCommand) -> None:
        new_mode = cmd.payload["mode"]
        target_joint = new_mode == "joint"
        if target_joint == self.ctx.cfg.use_joint_mode:
            self.emit_info(f"already in mode={new_mode}.")
            return
        self.ctx.cfg.use_joint_mode = target_joint
        self.ctx.dispatcher = self._build_dispatcher()
        self.composer.reset_after_mode_switch()
        self.emit_info(f"switched to mode={new_mode}.")

    def _handle_brake(self, cmd: ParsedCommand) -> None:
        on = bool(cmd.payload["on"])
        try:
            self.ctx.backend.apply_brake(on)
            self.emit_info(f"brake {'on' if on else 'off'}.")
        except Exception as e:  # noqa: BLE001
            self.emit_error(f"brake call failed: {e}")
        # Echo the TX.
        tx = getattr(self.ctx.backend, "tx_log", [])
        if tx:
            self.emit(
                self.ctx.monitor.tx_line(
                    tx[-1],
                    dry=self.ctx.cfg.dummy,
                )
            )

    def _handle_set(self, cmd: ParsedCommand) -> None:
        k, v = cmd.payload["k"], cmd.payload["v"]
        # Whitelist of mutable runtime keys.  Other keys are silently
        # rejected to avoid the operator changing things that won't
        # propagate (e.g. backend kind needs a restart).
        ALLOWED = {
            "operator_heartbeat_timeout_ms",
            "heartbeat_interval_ms",
            "strict_runtime",
            "strict_heartbeat",
            "log_file",
            "gripper_min_pct",
            "gripper_max_pct",
        }
        if k not in ALLOWED:
            self.emit_warn(
                f"set: '{k}' is not a runtime-mutable key.  Allowed: {sorted(ALLOWED)}",
            )
            return
        try:
            setattr(self.ctx.cfg, k, v)
            # Reflect in the live components if applicable.
            if k == "operator_heartbeat_timeout_ms":
                self.ctx.safety.cfg.operator_heartbeat_timeout_ms = float(v)
            elif k in ("gripper_min_pct", "gripper_max_pct"):
                self.ctx.mixer = GripperMixer(
                    self.ctx.cfg.gripper_min_pct,
                    self.ctx.cfg.gripper_max_pct,
                )
                self.ctx.monitor = StateMonitor(self.ctx.mixer)
            self.emit_info(f"set {k}={v}.")
        except Exception as e:  # noqa: BLE001
            self.emit_error(f"set failed: {e}")

    # ── Topo check ────────────────────────────────────────────

    def _topology_spec(self) -> TopologySpec:
        cfg = self.ctx.cfg
        pubs: list[str] = []
        subs: list[str] = []
        opt: list[str] = []
        if cfg.use_right_arm:
            if cfg.use_joint_mode:
                pubs.append("/motion_target/target_joint_state_arm_right")
            else:
                pubs.append("/motion_target/target_pose_arm_right")
            if not cfg.no_gripper:
                pubs.append("/motion_target/target_position_gripper_right")
            subs.extend(
                [
                    "/hdas/feedback_arm_right",
                    "/motion_control/pose_ee_arm_right",
                ]
            )
            if not cfg.no_gripper:
                subs.append("/hdas/feedback_gripper_right")
        if cfg.use_left_arm:
            if cfg.use_joint_mode:
                pubs.append("/motion_target/target_joint_state_arm_left")
            else:
                pubs.append("/motion_target/target_pose_arm_left")
            if not cfg.no_gripper:
                pubs.append("/motion_target/target_position_gripper_left")
            subs.extend(
                [
                    "/hdas/feedback_arm_left",
                    "/motion_control/pose_ee_arm_left",
                ]
            )
            if not cfg.no_gripper:
                subs.append("/hdas/feedback_gripper_left")
        opt.extend(["/hdas/bms", "/controller"])
        return TopologySpec(publishers=pubs, subscribers=subs, optional=opt)

    def _run_topo_check(self, *, strict: bool) -> bool:
        """Run the topology health check, retrying while ROS 2 DDS
        discovery is still in progress.

        ROS 2 DDS discovery is asynchronous: when an rclpy node creates a
        new subscriber, it can take a fraction of a second (sometimes
        up to a couple of seconds on busy networks / first run after
        node restart) for the local participant to discover existing
        publishers from other nodes.  The first ``get_publisher_count``
        call right after ``rclpy.init`` therefore frequently returns
        ``0`` for topics that are in fact being published.

        We poll up to ``cfg.topo_discovery_timeout_s`` seconds, waiting
        for all required topics to have non-zero counts.  Only the
        FINAL formatted report is emitted (intermediate fail-and-wait
        cycles are silent) so the user just sees one clean output.
        Dummy backends return ``1`` for every topic and break out on
        the first try with no delay.
        """
        spec = self._topology_spec()
        timeout_s = float(
            getattr(self.ctx.cfg, "topo_discovery_timeout_s", 3.0)
        )
        # Always do at least one attempt; never poll faster than ~10 Hz.
        delay_s = 0.25
        max_attempts = max(1, int(round(timeout_s / delay_s)) + 1)

        report = self.ctx.monitor.topology_report(self.ctx.backend, spec)
        ok = "WARN" not in report
        attempts = 1
        elapsed_s = 0.0
        while not ok and attempts < max_attempts:
            time.sleep(delay_s)
            elapsed_s += delay_s
            report = self.ctx.monitor.topology_report(self.ctx.backend, spec)
            ok = "WARN" not in report
            attempts += 1
        self.emit(report)
        if attempts > 1:
            self.emit_info(
                f"topo check: {attempts} attempts over {elapsed_s:.2f}s "
                f"(DDS discovery)"
            )
        if not ok and strict:
            self.emit_error(
                f"strict_topo: missing required topics after {attempts} "
                f"attempts ({elapsed_s:.2f}s); aborting startup.  "
                f"If your DDS is slow, raise "
                f"--topo-discovery-timeout-s (currently {timeout_s:.1f}s); "
                f"or temporarily drop --strict-topo to inspect the REPL."
            )
            return False
        return True

    # ── Heartbeat thread ─────────────────────────────────────

    def _start_heartbeat_thread(self) -> None:
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="r1_pro_cli_heartbeat",
            daemon=True,
        )
        self._heartbeat_thread.start()

    def _heartbeat_loop(self) -> None:
        interval = max(self.ctx.cfg.heartbeat_interval_ms / 1000.0, 0.05)
        while self.ctx.running:
            try:
                self.ctx.safety.heartbeat()
            except Exception:  # noqa: BLE001
                logger.exception("heartbeat tick crashed")
            time.sleep(interval)

    # ── Dispatcher build (also used on mode switch) ─────────

    def _build_dispatcher(self) -> ActionDispatcher:
        cfg = self.ctx.cfg
        return build_action_dispatcher(
            use_joint_mode=cfg.use_joint_mode,
            use_right_arm=cfg.use_right_arm,
            use_left_arm=cfg.use_left_arm,
            no_gripper=cfg.no_gripper,
            gripper_mixer=self.ctx.mixer,
            controller=self.ctx.backend,
            q_min_right=np.asarray(cfg.arm_q_min_right, dtype=np.float32),
            q_max_right=np.asarray(cfg.arm_q_max_right, dtype=np.float32),
            q_min_left=np.asarray(cfg.arm_q_min_left, dtype=np.float32),
            q_max_left=np.asarray(cfg.arm_q_max_left, dtype=np.float32),
            q_vel_max=np.asarray(cfg.arm_qvel_max, dtype=np.float32),
            home_q_right=np.asarray(cfg.home_q_right, dtype=np.float32),
            home_q_left=(
                np.asarray(cfg.home_q_left, dtype=np.float32)
                if cfg.home_q_left is not None
                else None
            ),
            joint_delta_mode=bool(cfg.joint_delta_mode),
            joint_delta_scale_right=np.asarray(
                cfg.joint_delta_scale_right,
                dtype=np.float32,
            ),
            joint_delta_scale_left=(
                np.asarray(cfg.joint_delta_scale_left, dtype=np.float32)
                if cfg.joint_delta_scale_left is not None
                else None
            ),
            ee_min_right=np.asarray(cfg.ee_min_right, dtype=np.float32),
            ee_max_right=np.asarray(cfg.ee_max_right, dtype=np.float32),
            ee_min_left=np.asarray(cfg.ee_min_left, dtype=np.float32),
            ee_max_left=np.asarray(cfg.ee_max_left, dtype=np.float32),
            home_pose_right=(
                np.asarray(cfg.home_pose_right, dtype=np.float32)
                if cfg.home_pose_right is not None
                else None
            ),
            home_pose_left=(
                np.asarray(cfg.home_pose_left, dtype=np.float32)
                if cfg.home_pose_left is not None
                else None
            ),
        )

    # ── Prompt / shutdown ───────────────────────────────────

    def _prompt(self) -> str:
        cfg = self.ctx.cfg
        if cfg.use_joint_mode:
            mode = "joint_delta" if cfg.joint_delta_mode else "joint"
        else:
            mode = "ee"
        side = (
            "dual"
            if (cfg.use_right_arm and cfg.use_left_arm)
            else ("right" if cfg.use_right_arm else "left")
        )
        return f"r1pro [{mode}][{side}] ({time.strftime('%H:%M:%S')}) > "

    def _shutdown(self) -> None:
        self.ctx.running = False
        try:
            self.ctx.backend.shutdown()
        except Exception:  # noqa: BLE001
            logger.exception("backend shutdown failed")
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=2.0)
        self.emit_info("backend shut down.")


# ─────────────────── Builders / main ────────────────────────────


def build_context(cfg: CliConfig) -> CliContext:
    """Construct the full :class:`CliContext` from *cfg*.

    Splits out for easy unit testing -- tests call ``build_context``
    with a custom ``CliConfig`` then poke at ``ctx.backend.tx_log``
    after running ``ReplLoop.handle_line``.
    """
    backend = build_backend(
        kind=cfg.backend_kind,
        is_dummy=cfg.dummy,
        ros_domain_id=cfg.ros_domain_id,
        ros_localhost_only=cfg.ros_localhost_only,
        galaxea_install_path=cfg.galaxea_install_path,
        use_right_arm=cfg.use_right_arm,
        use_left_arm=cfg.use_left_arm,
        no_gripper=cfg.no_gripper,
        use_torso=cfg.use_torso,
        use_chassis=cfg.use_chassis,
        mobiman_launch_mode=cfg.mobiman_launch_mode,
        controller_node_rank=cfg.controller_node_rank,
    )
    safety_cfg_dict = {
        "operator_heartbeat_timeout_ms": cfg.operator_heartbeat_timeout_ms,
        "bms_low_battery_threshold_pct": cfg.bms_low_battery_threshold_pct,
        "feedback_stale_threshold_ms": cfg.feedback_stale_threshold_ms,
        "estop_swd_value_down": cfg.estop_swd_value_down,
        "right_arm_q_min": cfg.arm_q_min_right,
        "right_arm_q_max": cfg.arm_q_max_right,
        "left_arm_q_min": cfg.arm_q_min_left,
        "left_arm_q_max": cfg.arm_q_max_left,
        "arm_qvel_max": cfg.arm_qvel_max,
    }
    safety = GalaxeaR1ProSafetySupervisor(build_safety_config(safety_cfg_dict))
    mixer = GripperMixer(cfg.gripper_min_pct, cfg.gripper_max_pct)
    monitor = StateMonitor(mixer)
    # Build dispatcher last so it can borrow the backend.
    dispatcher = build_action_dispatcher(
        use_joint_mode=cfg.use_joint_mode,
        use_right_arm=cfg.use_right_arm,
        use_left_arm=cfg.use_left_arm,
        no_gripper=cfg.no_gripper,
        gripper_mixer=mixer,
        controller=backend,
        q_min_right=np.asarray(cfg.arm_q_min_right, dtype=np.float32),
        q_max_right=np.asarray(cfg.arm_q_max_right, dtype=np.float32),
        q_min_left=np.asarray(cfg.arm_q_min_left, dtype=np.float32),
        q_max_left=np.asarray(cfg.arm_q_max_left, dtype=np.float32),
        q_vel_max=np.asarray(cfg.arm_qvel_max, dtype=np.float32),
        home_q_right=np.asarray(cfg.home_q_right, dtype=np.float32),
        joint_delta_mode=bool(cfg.joint_delta_mode),
        joint_delta_scale_right=np.asarray(
            cfg.joint_delta_scale_right,
            dtype=np.float32,
        ),
        joint_delta_scale_left=(
            np.asarray(cfg.joint_delta_scale_left, dtype=np.float32)
            if cfg.joint_delta_scale_left is not None
            else None
        ),
        ee_min_right=np.asarray(cfg.ee_min_right, dtype=np.float32),
        ee_max_right=np.asarray(cfg.ee_max_right, dtype=np.float32),
        ee_min_left=np.asarray(cfg.ee_min_left, dtype=np.float32),
        ee_max_left=np.asarray(cfg.ee_max_left, dtype=np.float32),
    )
    return CliContext(
        cfg=cfg,
        backend=backend,
        safety=safety,
        dispatcher=dispatcher,
        mixer=mixer,
        monitor=monitor,
    )


def build_argparser() -> argparse.ArgumentParser:
    """argparse for the launch flags.  All runtime mutation is
    handled by the ``set`` REPL command, not here."""
    p = argparse.ArgumentParser(
        prog="test_galaxea_r1_pro_cli_controller",
        description=(
            "Joint+gripper CLI for Galaxea R1 Pro.  Replaces the Pi0.5 "
            "policy at the input end of the RLinf real-robot pipeline "
            "with a human typing commands."
        ),
    )
    p.add_argument("--backend", default="rclpy", choices=["ray", "rclpy", "dummy"])
    p.add_argument(
        "--dummy", action="store_true", help="Force dummy mode (no ROS, no Ray)."
    )
    # Mode flags
    p.add_argument(
        "--use-joint-mode", dest="use_joint_mode", action="store_true", default=True
    )
    p.add_argument("--use-ee-mode", dest="use_joint_mode", action="store_false")
    p.add_argument(
        "--use-right-arm", dest="use_right_arm", action="store_true", default=True
    )
    p.add_argument(
        "--use-left-arm", dest="use_left_arm", action="store_true", default=False
    )
    p.add_argument(
        "--no-gripper", dest="no_gripper", action="store_true", default=False
    )
    # Joint sub-mode (per r1pro6op47.md §3.6).  Off by default to keep
    # CLI semantics identical to existing absolute joint mode.
    p.add_argument(
        "--joint-delta-mode",
        dest="joint_delta_mode",
        action="store_true",
        default=False,
        help=(
            "Enable joint delta sub-mode (model action[:7] is per-step "
            "joint increment, not absolute).  Operator's `j r ...` still "
            "means absolute target rad; composer back-computes the delta. "
            "See bt/docs/rwRL/r1pro6op47.md §3.6."
        ),
    )
    p.add_argument(
        "--joint-delta-scale",
        type=float,
        nargs=7,
        default=None,
        metavar=("S0", "S1", "S2", "S3", "S4", "S5", "S6"),
        help=(
            "Per-joint rad/step cap for delta sub-mode (length 7).  "
            "Default: [0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20]."
        ),
    )
    # Gripper business range
    p.add_argument("--gripper-min-pct", type=float, default=0.0)
    p.add_argument("--gripper-max-pct", type=float, default=90.0)
    # ROS env
    p.add_argument(
        "--ros-domain-id", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", "41"))
    )
    p.add_argument("--ros-localhost-only", action="store_true", default=False)
    p.add_argument(
        "--galaxea-install-path",
        default=os.environ.get("GALAXEA_INSTALL", "~/galaxea/install"),
    )
    p.add_argument("--controller-node-rank", type=int, default=0)
    p.add_argument("--mobiman-launch-mode", default="joint")
    # Strictness
    p.add_argument("--strict-topo", action="store_true", default=False)
    p.add_argument(
        "--strict-heartbeat",
        action="store_true",
        default=False,
        help="Use the training-time 1.5s heartbeat timeout "
        "and disable the background heartbeat thread.",
    )
    p.add_argument(
        "--strict-runtime",
        action="store_true",
        default=False,
        help="Exit on any backend send_* exception.",
    )
    p.add_argument("--operator-heartbeat-timeout-ms", type=float, default=30000.0)
    p.add_argument("--heartbeat-interval-ms", type=int, default=200)
    p.add_argument(
        "--topo-discovery-timeout-s",
        type=float,
        default=3.0,
        help=(
            "Wait up to N seconds for ROS 2 DDS discovery to settle "
            "before declaring the topology check failed.  Default 3.0. "
            "Useful when cross-host DDS or fresh node restart causes "
            "publishers to take >1s to be discovered.  Set to 0 to fail "
            "on the first attempt (matches old behaviour)."
        ),
    )
    # Logging
    p.add_argument("--log-file", default=None)
    p.add_argument(
        "--script",
        default=None,
        help="Read commands from PATH instead of stdin (one "
        "command per line; comments OK).  Useful for "
        "fixture replay and CI smoke tests.",
    )
    return p


def cfg_from_args(args: argparse.Namespace) -> CliConfig:
    cfg = CliConfig()
    cfg.backend_kind = args.backend
    cfg.dummy = bool(args.dummy)
    cfg.use_joint_mode = bool(args.use_joint_mode)
    cfg.use_right_arm = bool(args.use_right_arm)
    cfg.use_left_arm = bool(args.use_left_arm)
    cfg.no_gripper = bool(args.no_gripper)
    cfg.joint_delta_mode = bool(args.joint_delta_mode)
    if args.joint_delta_scale is not None:
        cfg.joint_delta_scale_right = list(args.joint_delta_scale)
    cfg.gripper_min_pct = float(args.gripper_min_pct)
    cfg.gripper_max_pct = float(args.gripper_max_pct)
    cfg.ros_domain_id = int(args.ros_domain_id)
    cfg.ros_localhost_only = bool(args.ros_localhost_only)
    cfg.galaxea_install_path = args.galaxea_install_path
    cfg.controller_node_rank = int(args.controller_node_rank)
    cfg.mobiman_launch_mode = args.mobiman_launch_mode
    cfg.strict_topo = bool(args.strict_topo)
    cfg.strict_heartbeat = bool(args.strict_heartbeat)
    cfg.strict_runtime = bool(args.strict_runtime)
    cfg.operator_heartbeat_timeout_ms = float(args.operator_heartbeat_timeout_ms)
    cfg.heartbeat_interval_ms = int(args.heartbeat_interval_ms)
    cfg.topo_discovery_timeout_s = float(args.topo_discovery_timeout_s)
    cfg.log_file = args.log_file
    if args.strict_heartbeat:
        cfg.operator_heartbeat_timeout_ms = 1500.0
    return cfg


def _make_script_input(path: str) -> Callable[[str], str]:
    """Return an input function that yields lines from *path* one at a
    time, then raises EOFError to terminate the REPL.  Comment lines
    are forwarded as-is (parser strips them)."""
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    it = iter(lines)

    def _read(prompt: str) -> str:
        try:
            return next(it)
        except StopIteration as exc:
            raise EOFError() from exc

    return _read


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argparser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    cfg = cfg_from_args(args)
    ctx = build_context(cfg)
    input_fn: Optional[Callable[[str], str]] = None
    if args.script:
        input_fn = _make_script_input(args.script)
    repl = ReplLoop(ctx, input_fn=input_fn)
    return repl.run()


if __name__ == "__main__":
    sys.exit(main())
