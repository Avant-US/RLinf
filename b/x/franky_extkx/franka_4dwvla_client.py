#!/usr/bin/env python3
"""4DWVLA evaluation client for Franka, built on franky_ext safety infrastructure.

Connects to the 4DWVLA inference server (vla_inference_server.py) via
multiprocessing.connection, reads observations from the robot and cameras,
sends them for inference, and executes the returned joint-space actions
with motion-guard / watchdog safety from franky_ext.

Safety hierarchy (matching FrankyJointEnv from 4dwvla_ext):
  L1: Hard joint-limit clipping (per step)
  L2: Training-range + margin clipping (per step)
  L3: Per-step joint velocity limiting
  L4: TCP fence watchdog (50 Hz, motion_limits constants)
  L5: Joint velocity norm limit (50 Hz watchdog)
  L6: Collision behavior tightening (init)
  L7: libfranka hardware reflex (1 kHz, firmware)
  L8: E-Stop (hardware)

State history (修复 A from grperr_1.md): every arm pose executed between
two inference calls is shipped to the server so observation.his_len
advances per control step, not per inference.

Usage (on the robot host):
    source /opt/venv/franky-0.19.0/bin/activate
    python /workspace/RLinf/b/x/franky_extkx/franka_4dwvla_client.py \\
        --robot-ip 172.16.0.2 \\
        --task "plug into socket" \\
        --server-host <gpu_host> \\
        --use-realsense
"""
from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading
import time
from collections import deque
from multiprocessing.connection import Client
from pathlib import Path
from typing import Optional

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
_EXT_DIR = _THIS_DIR.parent
for _p in (str(_THIS_DIR), str(_EXT_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from franky_ext.motion_limits import (
    cartesian_collision_thresholds,
    guard_floor_margin_m,
    guard_margin_m,
    guard_max_dq_rad_s,
    guard_recovery_budget,
    reach_radius_m,
    PANDA_MAX_REACH_M,
    REACH_WARN_FRACTION,
)
from franky_ext.franka_libfranka_gripper import FrankaLibfrankaGripper
from franky_ext.dsplug.home_pose import load_home_joints

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    force=True,
)
logger = logging.getLogger("4dwvla-client")

# ── IPC ──────────────────────────────────────────────────────────────────────

AUTHKEY = b"4dwvla-eval"
DEFAULT_SERVER_PORT = 5555

# ── Joint limits (FR3v2.1) ──────────────────────────────────────────────────

JOINT_LIMITS_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_LIMITS_UPPER = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
JOINT_VEL_LIMITS = np.array([2.075, 2.075, 2.075, 2.075, 2.51, 2.51, 2.51])

# ── Training data range ─────────────────────────────────────────────────────

TRAIN_ARM_MIN = np.array([-0.4842, -0.1030, -0.2025, -2.2044, -0.2041, 1.5702, 0.4843])
TRAIN_ARM_MAX = np.array([0.0452, 0.3120, 0.4789, -1.5347, 0.0806, 2.4536, 0.9807])
TRAIN_TCP_MIN = np.array([0.534, -0.140, 0.178])
TRAIN_TCP_MAX = np.array([0.602, 0.053, 0.517])

SAFETY_MARGIN_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.0])
ACTION_LIMIT_LOWER = np.maximum(TRAIN_ARM_MIN - SAFETY_MARGIN_RAD, JOINT_LIMITS_LOWER)
ACTION_LIMIT_UPPER = np.minimum(TRAIN_ARM_MAX + SAFETY_MARGIN_RAD, JOINT_LIMITS_UPPER)
MAX_JOINT_STEP_RAD = 0.15
TRAIN_EDGE_WARN_RAD = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03])

HOME_JOINTS = load_home_joints()

# ── Gripper ──────────────────────────────────────────────────────────────────

GRIPPER_CLOSE_THRESHOLD = float(os.environ.get("VLA_GRIPPER_CLOSE_THRESHOLD", "0.5"))
GRIPPER_CLOSE_IF_ABOVE = os.environ.get(
    "VLA_GRIPPER_CLOSE_IF_ABOVE", "1"
).strip().lower() not in ("0", "false", "no", "off")

# ── Watchdog ─────────────────────────────────────────────────────────────────

_WATCHDOG_PERIOD_S = 0.02
_HZ_LOG_INTERVAL = 50
_HZ_WARN_RATIO = 0.5

# ── Plug-eval gripper env defaults (same as FrankyControllerDirect) ──────────

_PLUG_WIDTH_M = "0.010"
_PLUG_HOLD_TOL_M = "0.008"
_PLUG_GRASP_FORCE_N = "20"
_PLACEHOLDER_CUBE_WIDTH_M = "0.046"


def _ensure_plug_gripper_env() -> None:
    os.environ.setdefault("FRANKA_GRASP_FORCE", _PLUG_GRASP_FORCE_N)
    current = os.environ.get("FRANKA_CUBE_WIDTH_M")
    if current is None or current.strip() in ("", _PLACEHOLDER_CUBE_WIDTH_M):
        os.environ["FRANKA_CUBE_WIDTH_M"] = _PLUG_WIDTH_M
    os.environ.setdefault("FRANKA_HOLD_TOL_M", _PLUG_HOLD_TOL_M)


# ── Array formatting ────────────────────────────────────────────────────────

def _fmt(value, precision: int = 6) -> str:
    return np.array2string(
        np.asarray(value), precision=precision, suppress_small=False,
        separator=", ", max_line_width=100_000,
    )


def _meta(value) -> dict:
    a = np.asarray(value)
    r: dict = {"shape": list(a.shape), "dtype": str(a.dtype)}
    if a.size and np.issubdtype(a.dtype, np.number):
        f = np.isfinite(a)
        r["finite"] = bool(f.all())
        if f.any():
            fv = a[f]
            r.update(min=float(fv.min()), max=float(fv.max()), mean=float(fv.mean()))
    return r


# ── Logging configuration ───────────────────────────────────────────────────

def _configure_logging(log_dir: str | None) -> Path:
    from datetime import datetime

    d = Path(log_dir or os.environ.get("VLA_LOG_DIR", str(_THIS_DIR / "logs")))
    d.mkdir(parents=True, exist_ok=True)
    now = datetime.now()
    path = d / f"4dwvla_client_{now.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}.log"
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    fh = logging.FileHandler(path, encoding="utf-8")
    fh.setFormatter(fmt)
    logging.getLogger().addHandler(fh)
    logger.info("Log file: %s", path)
    return path


# ═══════════════════════════════════════════════════════════════════════════
# Joint-space safety (L1-L3, from FrankyJointEnv)
# ═══════════════════════════════════════════════════════════════════════════

def check_action_safety(
    action_arm: np.ndarray, current_joints: np.ndarray, step_idx: int
) -> tuple[np.ndarray, list[str]]:
    warnings: list[str] = []
    clipped = action_arm.copy()

    # L1: Hard joint limits
    below = clipped < JOINT_LIMITS_LOWER
    above = clipped > JOINT_LIMITS_UPPER
    if np.any(below) or np.any(above):
        viol = []
        for i in range(7):
            if below[i]:
                viol.append(f"q{i+1}={clipped[i]:.4f}<{JOINT_LIMITS_LOWER[i]:.4f}")
            elif above[i]:
                viol.append(f"q{i+1}={clipped[i]:.4f}>{JOINT_LIMITS_UPPER[i]:.4f}")
        warnings.append(f"[step {step_idx}] HARD LIMIT: {', '.join(viol)}")
        clipped = np.clip(clipped, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

    # L2: Training range + margin
    below_t = clipped < ACTION_LIMIT_LOWER
    above_t = clipped > ACTION_LIMIT_UPPER
    if np.any(below_t) or np.any(above_t):
        viol = []
        for i in range(7):
            if below_t[i]:
                viol.append(f"q{i+1}={clipped[i]:.4f}<{ACTION_LIMIT_LOWER[i]:.4f}")
            elif above_t[i]:
                viol.append(f"q{i+1}={clipped[i]:.4f}>{ACTION_LIMIT_UPPER[i]:.4f}")
        warnings.append(f"[step {step_idx}] OUT-OF-TRAIN: {', '.join(viol)}")
        clipped = np.clip(clipped, ACTION_LIMIT_LOWER, ACTION_LIMIT_UPPER)

    # L2b: training-range edge warning (q7 specifically)
    for i in range(7):
        if TRAIN_EDGE_WARN_RAD[i] <= 0.0:
            continue
        low_gap = clipped[i] - TRAIN_ARM_MIN[i]
        high_gap = TRAIN_ARM_MAX[i] - clipped[i]
        if 0.0 <= low_gap < TRAIN_EDGE_WARN_RAD[i]:
            warnings.append(
                f"[step {step_idx}] TRAIN-EDGE: q{i+1}={clipped[i]:.4f} "
                f"is {low_gap:.4f} above min"
            )
        elif 0.0 <= high_gap < TRAIN_EDGE_WARN_RAD[i]:
            warnings.append(
                f"[step {step_idx}] TRAIN-EDGE: q{i+1}={clipped[i]:.4f} "
                f"is {high_gap:.4f} below max"
            )

    # L3: Velocity limit
    delta = clipped - current_joints
    if np.any(np.abs(delta) > MAX_JOINT_STEP_RAD):
        viol = [
            f"q{i+1}: {abs(delta[i]):.4f}"
            for i in range(7)
            if abs(delta[i]) > MAX_JOINT_STEP_RAD
        ]
        warnings.append(f"[step {step_idx}] VEL LIMIT: {', '.join(viol)}")
        scale = min(1.0, MAX_JOINT_STEP_RAD / float(np.abs(delta).max()))
        clipped = current_joints + delta * scale

    return clipped, warnings


# ═══════════════════════════════════════════════════════════════════════════
# State history buffer (修复 A: keypoint history per control step)
# ═══════════════════════════════════════════════════════════════════════════

class ExecutedStateBuffer:
    """Buffers arm joint angles executed between inference calls."""

    def __init__(self, max_len: int = 512) -> None:
        self._buf: deque[list[float]] = deque(maxlen=max_len)

    def record(self, arm_q7) -> None:
        self._buf.append([float(v) for v in np.asarray(arm_q7).reshape(-1)[:7]])

    def drain(self) -> list[list[float]]:
        out = list(self._buf)
        self._buf.clear()
        return out

    def clear(self) -> None:
        self._buf.clear()


# ═══════════════════════════════════════════════════════════════════════════
# Franka controller (joint-space, franky direct, franky_ext safety)
# ═══════════════════════════════════════════════════════════════════════════

class FrankaController:
    """Joint-space Franka controller with motion guard and watchdog.

    Uses franky directly (no Ray) and reuses safety constants / thresholds
    from franky_ext.motion_limits.
    """

    def __init__(self, robot_ip: str, gripper_type: str = "franka"):
        import franky

        self._robot = franky.Robot(robot_ip)
        self._robot.recover_from_errors()
        self._robot.relative_dynamics_factor = 0.2
        self._franky = franky

        _ensure_plug_gripper_env()
        if gripper_type == "franka":
            self._gripper = FrankaLibfrankaGripper(robot_ip=robot_ip)
        else:
            self._gripper = None

        self._guard_min_xyz: Optional[np.ndarray] = None
        self._guard_max_xyz: Optional[np.ndarray] = None
        self._guard_max_dq = guard_max_dq_rad_s()
        self._guard_enabled = False
        self._guard_trip_reason: Optional[str] = None
        self._guard_trip_lock = threading.Lock()
        self._guard_recoveries_used = 0
        self._guard_recovery_budget = guard_recovery_budget()

        self._watchdog: Optional[threading.Thread] = None
        self._watchdog_stop = threading.Event()

        self._tighten_collision_behavior()
        logger.info(
            "FrankaController: connected to %s, guard_max_dq=%.2frad/s, "
            "recovery_budget=%d",
            robot_ip, self._guard_max_dq, self._guard_recovery_budget,
        )

    def _tighten_collision_behavior(self) -> None:
        thresholds = cartesian_collision_thresholds()
        torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        try:
            self._robot.set_collision_behavior(
                lower_torque_threshold=torque,
                upper_torque_threshold=torque,
                lower_force_threshold=thresholds,
                upper_force_threshold=thresholds,
            )
            logger.info("Collision behavior tightened: %s", thresholds)
        except Exception as e:
            logger.warning("Could not tighten collision behavior: %s", e)

    # ── Motion guard ────────────────────────────────────────────────────

    def set_motion_guard(
        self,
        limit_min_xyz: np.ndarray,
        limit_max_xyz: np.ndarray,
        *,
        margin: Optional[float] = None,
        floor_margin: Optional[float] = None,
    ) -> None:
        m = guard_margin_m() if margin is None else float(margin)
        fm = guard_floor_margin_m() if floor_margin is None else float(floor_margin)
        self._guard_min_xyz = np.array(limit_min_xyz, dtype=np.float64) - m
        self._guard_min_xyz[2] = limit_min_xyz[2] - fm
        self._guard_max_xyz = np.array(limit_max_xyz, dtype=np.float64) + m
        self._guard_enabled = True
        logger.info(
            "Motion guard set: min=%s, max=%s (margin=%.3f, floor=%.3f)",
            np.round(self._guard_min_xyz, 4).tolist(),
            np.round(self._guard_max_xyz, 4).tolist(),
            m, fm,
        )
        if self._watchdog is None or not self._watchdog.is_alive():
            self._start_watchdog()

    def guard_tripped(self) -> Optional[str]:
        with self._guard_trip_lock:
            return self._guard_trip_reason

    def _evaluate_guard(self) -> Optional[tuple[str, str]]:
        if not self._guard_enabled:
            return None
        try:
            state = self._robot.state
            tcp = np.array(state.O_T_EE.translation)
        except Exception:
            return None

        if self._guard_min_xyz is not None and self._guard_max_xyz is not None:
            below = tcp < self._guard_min_xyz
            above = tcp > self._guard_max_xyz
            if np.any(below) or np.any(above):
                axes = "XYZ"
                viol = []
                for i in range(3):
                    if below[i]:
                        viol.append(f"{axes[i]}={tcp[i]:.4f}<{self._guard_min_xyz[i]:.4f}")
                    elif above[i]:
                        viol.append(f"{axes[i]}={tcp[i]:.4f}>{self._guard_max_xyz[i]:.4f}")
                return ("fence", f"TCP outside fence: {', '.join(viol)}")

        try:
            dq = np.array(state.dq[:7])
            dq_norm = float(np.linalg.norm(dq))
            if dq_norm > self._guard_max_dq:
                return ("dq", f"|dq|={dq_norm:.3f} > {self._guard_max_dq:.3f} rad/s")
        except Exception:
            pass

        r = reach_radius_m(tcp)
        if r > PANDA_MAX_REACH_M * REACH_WARN_FRACTION:
            logger.warning(
                "NEAR-SINGULAR: reach=%.3fm (%.0f%% of max)",
                r, r / PANDA_MAX_REACH_M * 100,
            )
        return None

    def _abort_motion(self, kind: str, reason: str) -> None:
        with self._guard_trip_lock:
            if self._guard_trip_reason is not None:
                return
            self._guard_trip_reason = f"{kind}: {reason}"
        logger.error("MOTION GUARD TRIP [%s]: %s", kind, reason)
        self._brake(kind)

    def _brake(self, kind: str) -> None:
        try:
            if kind in ("fence", "orient"):
                self._robot.stop()
            else:
                try:
                    q = list(self._robot.state.q[:7])
                    motion = self._franky.JointWaypointMotion(
                        [self._franky.JointWaypoint(q)]
                    )
                    saved = self._robot.relative_dynamics_factor
                    self._robot.relative_dynamics_factor = 0.05
                    self._robot.move(motion)
                    self._robot.relative_dynamics_factor = saved
                except Exception:
                    self._robot.stop()
        except Exception as e:
            logger.error("Brake failed: %s", e)

    def _start_watchdog(self) -> None:
        self._watchdog_stop.clear()
        self._watchdog = threading.Thread(
            target=self._watchdog_loop,
            args=(self._watchdog_stop,),
            daemon=True,
            name="motion-guard-watchdog",
        )
        self._watchdog.start()
        logger.info("Watchdog started (%.0f Hz)", 1.0 / _WATCHDOG_PERIOD_S)

    def _stop_watchdog(self) -> None:
        if self._watchdog is not None:
            self._watchdog_stop.set()
            self._watchdog.join(timeout=2.0)
            self._watchdog = None

    def _watchdog_loop(self, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            try:
                violation = self._evaluate_guard()
                if violation is not None:
                    self._abort_motion(*violation)
            except Exception as e:
                logger.error("Watchdog error: %s", e)
            stop_event.wait(_WATCHDOG_PERIOD_S)

    def recover_from_guard_trip(self) -> dict:
        with self._guard_trip_lock:
            was = self._guard_trip_reason
            if was is None:
                return {"recovered": True, "was_tripped": False}

        if self._guard_recoveries_used >= self._guard_recovery_budget:
            return {"recovered": False, "was_tripped": True, "budget_exhausted": True}

        try:
            self._robot.recover_from_errors()
            time.sleep(0.5)
            violation = self._evaluate_guard()
            if violation is not None:
                return {"recovered": False, "was_tripped": True, "reason": str(violation)}
            with self._guard_trip_lock:
                self._guard_trip_reason = None
            self._guard_recoveries_used += 1
            logger.info(
                "Guard recovery %d/%d",
                self._guard_recoveries_used, self._guard_recovery_budget,
            )
            return {"recovered": True, "was_tripped": True, "previous_reason": was}
        except Exception as e:
            return {"recovered": False, "was_tripped": True, "error": str(e)}

    # ── Robot control ───────────────────────────────────────────────────

    def get_state(self) -> dict:
        state = self._robot.state
        q = np.array(state.q[:7], dtype=np.float64)
        dq = np.array(state.dq[:7], dtype=np.float64)
        tcp = np.array(state.O_T_EE.translation)
        gw = self.gripper_width()
        return {
            "arm_joint_position": q,
            "arm_joint_velocity": dq,
            "tcp_position": tcp,
            "gripper_width": gw,
        }

    def move_joints(self, joint_positions: np.ndarray) -> None:
        tripped = self.guard_tripped()
        if tripped is not None:
            raise RuntimeError(f"Motion guard tripped: {tripped}")
        clipped = np.clip(joint_positions, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
        motion = self._franky.JointWaypointMotion(
            [self._franky.JointWaypoint(clipped.tolist())]
        )
        self._robot.move(motion)

    def reset_joint(self, positions: list[float]) -> None:
        motion = self._franky.JointWaypointMotion(
            [self._franky.JointWaypoint(positions)]
        )
        saved = self._robot.relative_dynamics_factor
        self._robot.relative_dynamics_factor = 0.1
        self._robot.move(motion)
        self._robot.relative_dynamics_factor = saved

    def open_gripper(self) -> None:
        if self._gripper is not None:
            self._gripper.open(speed=0.05)

    def close_gripper(self) -> None:
        if self._gripper is None:
            return
        try:
            self._gripper.close(speed=0.05)
        except RuntimeError as exc:
            if "did not finish" in str(exc):
                raise
            logger.warning("close_gripper: %s", exc)
        except Exception as exc:
            logger.warning("close_gripper failed: %s", exc)

    def gripper_width(self) -> Optional[float]:
        if self._gripper is None:
            return None
        try:
            return float(self._gripper.position)
        except Exception:
            inner = getattr(self._gripper, "_gripper", None)
            if inner is None:
                return None
            return float(inner.width)

    def gripper_is_open(self) -> bool:
        if self._gripper is None:
            return True
        return bool(getattr(self._gripper, "is_open", True))

    def gripper_holding(self) -> bool:
        if self._gripper is None:
            return False
        fn = getattr(self._gripper, "_hardware_holding", None)
        return bool(fn()) if callable(fn) else False

    def stop(self) -> None:
        try:
            self._robot.stop()
        except Exception as e:
            logger.error("stop failed: %s", e)

    def cleanup(self) -> None:
        self._stop_watchdog()
        try:
            q = list(self._robot.state.q[:7])
            saved = self._robot.relative_dynamics_factor
            self._robot.relative_dynamics_factor = 0.05
            self._robot.move(
                self._franky.JointWaypointMotion(
                    [self._franky.JointWaypoint(q)]
                )
            )
            self._robot.relative_dynamics_factor = saved
        except Exception:
            pass
        if self._gripper is not None:
            cleanup_fn = getattr(self._gripper, "cleanup", None)
            if callable(cleanup_fn):
                cleanup_fn()
        logger.info("Controller cleanup complete")


# ═══════════════════════════════════════════════════════════════════════════
# Camera capture
# ═══════════════════════════════════════════════════════════════════════════

class CameraCapture:
    """RealSense stereo camera capture (global + wrist)."""

    def __init__(self, serials: dict[str, str | None]):
        import pyrealsense2 as rs

        self._pipes: dict[str, "rs.pipeline"] = {}
        for name in ("global", "wrist"):
            pipe = rs.pipeline()
            cfg = rs.config()
            serial = serials.get(name)
            if serial:
                cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            pipe.start(cfg)
            self._pipes[name] = pipe
        logger.info(
            "Cameras initialized: global=%s wrist=%s",
            serials.get("global", "auto"), serials.get("wrist", "auto"),
        )

    def get_frames(self) -> dict[str, np.ndarray]:
        frames = {}
        for name, pipe in self._pipes.items():
            fs = pipe.wait_for_frames(timeout_ms=1000)
            color = fs.get_color_frame()
            if not color:
                raise RuntimeError(f"No frame from {name} camera")
            frames[name] = np.asarray(color.get_data(), dtype=np.uint8)
        return frames

    def close(self) -> None:
        for pipe in self._pipes.values():
            try:
                pipe.stop()
            except Exception:
                pass


# ═══════════════════════════════════════════════════════════════════════════
# VLA evaluation controller
# ═══════════════════════════════════════════════════════════════════════════

class VLAEvalController:
    """Orchestrates 4DWVLA inference and Franka execution.

    Connects to vla_inference_server.py over TCP, sends observations,
    receives and queues action chunks, executes them with safety checks.
    """

    def __init__(
        self,
        controller: FrankaController,
        camera: CameraCapture | None,
        server_address: tuple[str, int],
        task: str,
        *,
        n_exec: int = 10,
        max_steps: int = 300,
        control_hz: float = 10.0,
        dry_run: bool = False,
    ):
        self._ctrl = controller
        self._camera = camera
        self._server_address = server_address
        self._task = task
        self._n_exec = n_exec
        self._max_steps = max_steps
        self._control_hz = control_hz
        self._dry_run = dry_run

        self._conn = None
        self._action_queue: deque = deque()
        self._state_history = ExecutedStateBuffer()
        self._abort = False
        self._step_count = 0
        self._inference_count = 0
        self._total_warnings = 0
        self._last_gripper_want_close: Optional[bool] = None

        self._last_step_ts: Optional[float] = None
        self._step_interval_sum = 0.0
        self._step_interval_count = 0

        signal.signal(signal.SIGINT, lambda s, f: setattr(self, "_abort", True))

    # ── Server connection ───────────────────────────────────────────────

    def connect(self) -> None:
        logger.info("Connecting to %s:%d...", *self._server_address)
        self._conn = Client(self._server_address, authkey=AUTHKEY)
        logger.info("Connected to inference server")

    def disconnect(self) -> None:
        if self._conn:
            try:
                self._conn.send({"command": "shutdown"})
            except Exception:
                pass
            self._conn.close()
            self._conn = None

    def _notify_server_reset(self) -> None:
        """Tell server to clear policy/keypoint state (修复 E)."""
        if self._conn is None:
            return
        try:
            self._conn.send({"command": "reset"})
            resp = self._conn.recv()
            if resp.get("status") != "ok":
                logger.warning("Server reset returned: %s", resp.get("status"))
        except Exception as exc:
            logger.warning("Server reset failed: %s", exc)

    # ── Observations ────────────────────────────────────────────────────

    def _get_observation(self) -> dict:
        if self._dry_run:
            return {"state": np.concatenate([HOME_JOINTS, [0.078]])}
        state = self._ctrl.get_state()
        q = state["arm_joint_position"]
        gw = state["gripper_width"]
        if gw is None:
            raise RuntimeError("Gripper width unavailable")
        return {"state": np.concatenate([q, [float(gw)]])}

    def _get_camera_frames(self) -> dict[str, np.ndarray]:
        if self._camera is not None:
            return self._camera.get_frames()
        return {
            "global": np.zeros((480, 640, 3), dtype=np.uint8),
            "wrist": np.zeros((480, 640, 3), dtype=np.uint8),
        }

    # ── Inference ───────────────────────────────────────────────────────

    def _request_inference(self, images: dict, state: np.ndarray) -> list:
        state_history = self._state_history.drain()
        logger.info(
            "[inference %d] request: state=%s state_history_len=%d "
            "image_meta=%s task=%r",
            self._inference_count,
            _fmt(state),
            len(state_history),
            {n: _meta(img) for n, img in images.items()},
            self._task,
        )
        self._conn.send({
            "images": images,
            "state": {
                "arm": state[:7].tolist(),
                "gripper": [float(state[7])],
            },
            "state_history": state_history,
            "task": self._task,
            "protocol": 2,
        })
        resp = self._conn.recv()
        if resp["status"] != "ok":
            raise RuntimeError(f"Server error: {resp['status']}")
        actions = np.asarray(resp["actions"], dtype=np.float64)
        if actions.ndim != 2 or actions.shape[1] != 8:
            raise ValueError(f"Expected actions [N, 8], got {actions.shape}")
        if not np.isfinite(actions).all():
            raise ValueError("Actions contain NaN or infinite values")

        plan_delta = np.diff(np.vstack([state, actions]), axis=0)
        logger.info(
            "[inference %d] response: shape=%s actions=%s "
            "delta_from_previous=%s",
            self._inference_count,
            list(actions.shape),
            _fmt(actions),
            _fmt(plan_delta),
        )
        self._inference_count += 1
        return actions.tolist()

    # ── Gripper control ─────────────────────────────────────────────────

    def _execute_gripper(self, action_grip: float) -> None:
        if self._dry_run or self._ctrl is None:
            return
        want_close = (
            float(action_grip) >= GRIPPER_CLOSE_THRESHOLD
            if GRIPPER_CLOSE_IF_ABOVE
            else float(action_grip) < GRIPPER_CLOSE_THRESHOLD
        )
        if want_close != self._last_gripper_want_close:
            logger.info(
                "[step %d] gripper action=%.4f -> %s (threshold %s %.2f)",
                self._step_count,
                action_grip,
                "close" if want_close else "open",
                ">=" if GRIPPER_CLOSE_IF_ABOVE else "<",
                GRIPPER_CLOSE_THRESHOLD,
            )
            self._last_gripper_want_close = want_close
        if want_close:
            if self._ctrl.gripper_is_open():
                self._ctrl.close_gripper()
        elif not self._ctrl.gripper_is_open():
            self._ctrl.open_gripper()

    # ── Control-rate monitoring (修复 B) ────────────────────────────────

    def _track_control_interval(self) -> None:
        now = time.monotonic()
        if self._last_step_ts is not None:
            dt = now - self._last_step_ts
            if dt > 0:
                self._step_interval_sum += dt
                self._step_interval_count += 1
        self._last_step_ts = now

        if (
            self._step_interval_count
            and self._step_interval_count % _HZ_LOG_INTERVAL == 0
        ):
            achieved = self._step_interval_count / self._step_interval_sum
            ratio = achieved / self._control_hz if self._control_hz else 1.0
            fn = logger.warning if ratio < _HZ_WARN_RATIO else logger.info
            fn(
                "[step %d] control rate: %.2f Hz (requested %.1f Hz, ratio=%.2f)",
                self._step_count, achieved, self._control_hz, ratio,
            )

    # ── Reset ───────────────────────────────────────────────────────────

    def _reset_episode(self) -> dict:
        if not self._dry_run and self._ctrl is not None:
            tripped = self._ctrl.guard_tripped()
            if tripped:
                result = self._ctrl.recover_from_guard_trip()
                if not result["recovered"]:
                    raise RuntimeError(f"Cannot recover from guard trip: {tripped}")
            self._ctrl.open_gripper()
            time.sleep(0.3)
            self._ctrl.reset_joint(HOME_JOINTS.tolist())
            time.sleep(0.5)
            self._ctrl.open_gripper()
            time.sleep(0.3)

        self._step_count = 0
        self._action_queue.clear()
        self._state_history.clear()
        self._last_gripper_want_close = None
        self._last_step_ts = None
        self._notify_server_reset()
        return self._get_observation()

    # ── Main loop ───────────────────────────────────────────────────────

    def run(self) -> None:
        logger.info(
            "Starting: task=%r max_steps=%d n_exec=%d control_hz=%.1f dry_run=%s",
            self._task, self._max_steps, self._n_exec,
            self._control_hz, self._dry_run,
        )
        obs = self._reset_episode()

        while self._step_count < self._max_steps and not self._abort:
            if not self._action_queue:
                images = self._get_camera_frames()
                state = obs["state"]
                logger.info(
                    "[step %d] Inference (q1=%.3f grip=%.4f)",
                    self._step_count, state[0], state[7],
                )
                actions = self._request_inference(images, state)
                self._action_queue.extend(actions)
                logger.info("  Received %d actions", len(actions))

            action = self._action_queue.popleft()
            action_arr = np.array(action, dtype=np.float64)
            state_before = np.asarray(obs["state"], dtype=np.float64)
            action_delta = action_arr - state_before
            logger.info(
                "[step %d] execute: action=%s state_before=%s delta=%s",
                self._step_count,
                _fmt(action_arr),
                _fmt(state_before),
                _fmt(action_delta),
            )

            # Record arm state for keypoint history (修复 A)
            self._state_history.record(state_before[:7])

            if not self._dry_run:
                # L1-L3 safety
                current_q = self._ctrl.get_state()["arm_joint_position"]
                action_arm, warnings = check_action_safety(
                    action_arr[:7], current_q, self._step_count
                )
                for w in warnings:
                    logger.warning(w)
                self._total_warnings += len(warnings)

                # L4-L5 guard check
                tripped = self._ctrl.guard_tripped()
                if tripped is not None:
                    logger.error("MOTION GUARD TRIP: %s", tripped)
                    self._action_queue.clear()
                    input("[operator] Reset scene, then press Enter...")
                    obs = self._reset_episode()
                    continue

                # Execute arm
                try:
                    self._ctrl.move_joints(action_arm)
                except RuntimeError as e:
                    if "guard" in str(e).lower():
                        logger.error("Guard trip during move: %s", e)
                        self._action_queue.clear()
                        input("[operator] Reset scene, then press Enter...")
                        obs = self._reset_episode()
                        continue
                    raise

                # Gripper
                action_grip = float(action_arr[7]) if len(action_arr) > 7 else 0.5
                self._execute_gripper(action_grip)

                time.sleep(1.0 / self._control_hz)

                # Post-step guard check
                tripped = self._ctrl.guard_tripped()
                if tripped is not None:
                    logger.error("Guard trip post-step: %s", tripped)
                    self._action_queue.clear()
                    input("[operator] Reset scene, then press Enter...")
                    obs = self._reset_episode()
                    continue

                obs = self._get_observation()
                state_after = np.asarray(obs["state"], dtype=np.float64)
                logger.info(
                    "[step %d] result: state_after=%s realized_delta=%s",
                    self._step_count,
                    _fmt(state_after),
                    _fmt(state_after - state_before),
                )
            else:
                logger.info("[step %d] DRY RUN: skipping execution", self._step_count)

            self._track_control_interval()
            self._step_count += 1

        achieved_hz = (
            self._step_interval_count / self._step_interval_sum
            if self._step_interval_count
            else float("nan")
        )
        logger.info(
            "Done: %d steps, %d warnings, abort=%s, achieved_hz=%.2f (requested %.1f)",
            self._step_count, self._total_warnings, self._abort,
            achieved_hz, self._control_hz,
        )


# ═══════════════════════════════════════════════════════════════════════════
# Keyboard-gated wrapper
# ═══════════════════════════════════════════════════════════════════════════

def _try_import_keyboard():
    try:
        sys.path.insert(0, "/workspace/RLinf")
        from rlinf.envs.realworld.common.keyboard.keyboard_listener import (
            KeyboardListener,
        )
        return KeyboardListener
    except ImportError:
        return None


def run_with_keyboard(
    ctrl_obj: VLAEvalController,
    controller: FrankaController,
) -> None:
    """Keyboard-gated evaluation loop: 'a' to start, 'r' abort, 'c' success, 'b' failure."""
    KBListener = _try_import_keyboard()
    if KBListener is None:
        logger.warning(
            "KeyboardListener not available (no evdev or RLinf); "
            "running without keyboard gating"
        )
        ctrl_obj.run()
        return

    listener = KBListener()
    logger.info(
        "Keyboard controls: 'a'=start, 'r'=abort, 'b'=failure, "
        "'c'=success, 'h'=HOME"
    )

    while not ctrl_obj._abort:
        logger.info("Arrange scene, press 'a' to start (Ctrl-C to quit)...")
        listener.pop_pressed_keys()

        # Wait for 'a'
        started = False
        while not started and not ctrl_obj._abort:
            time.sleep(0.05)
            for key in listener.pop_pressed_keys():
                if key == "a":
                    started = True
                    logger.info("'a' pressed -- starting episode.")
                    break

        if not started:
            break

        obs = ctrl_obj._reset_episode()
        aborted = False

        while ctrl_obj._step_count < ctrl_obj._max_steps and not ctrl_obj._abort:
            # Check keyboard
            for key in listener.pop_pressed_keys():
                if key == "r":
                    logger.warning(">>> ABORT: 'r' key <<<")
                    controller.stop()
                    aborted = True
                    break
                elif key == "c":
                    logger.info("'c' pressed -- SUCCESS")
                    aborted = True
                    break
                elif key == "b":
                    logger.info("'b' pressed -- FAILURE")
                    aborted = True
                    break
                elif key == "h":
                    logger.info(">>> HOME: 'h' key <<<")
                    controller.stop()
                    controller.open_gripper()
                    time.sleep(0.3)
                    controller.reset_joint(HOME_JOINTS.tolist())
                    time.sleep(0.5)

            if aborted:
                break

            # Execute one step
            if not ctrl_obj._action_queue:
                images = ctrl_obj._get_camera_frames()
                state = obs["state"]
                logger.info(
                    "[step %d] Inference (q1=%.3f grip=%.4f)",
                    ctrl_obj._step_count, state[0], state[7],
                )
                actions = ctrl_obj._request_inference(images, state)
                ctrl_obj._action_queue.extend(actions)

            action = ctrl_obj._action_queue.popleft()
            action_arr = np.array(action, dtype=np.float64)
            state_before = np.asarray(obs["state"], dtype=np.float64)

            ctrl_obj._state_history.record(state_before[:7])

            if not ctrl_obj._dry_run:
                current_q = controller.get_state()["arm_joint_position"]
                action_arm, warnings = check_action_safety(
                    action_arr[:7], current_q, ctrl_obj._step_count
                )
                for w in warnings:
                    logger.warning(w)

                tripped = controller.guard_tripped()
                if tripped is not None:
                    logger.error("Guard trip: %s", tripped)
                    break

                try:
                    controller.move_joints(action_arm)
                except RuntimeError as e:
                    if "guard" in str(e).lower():
                        logger.error("Guard trip during move: %s", e)
                        break
                    raise

                grip = float(action_arr[7]) if len(action_arr) > 7 else 0.5
                ctrl_obj._execute_gripper(grip)

                time.sleep(1.0 / ctrl_obj._control_hz)

                tripped = controller.guard_tripped()
                if tripped is not None:
                    logger.error("Guard trip post-step: %s", tripped)
                    break

                obs = ctrl_obj._get_observation()
                state_after = np.asarray(obs["state"], dtype=np.float64)
                logger.info(
                    "[step %d] state_after=%s delta=%s",
                    ctrl_obj._step_count,
                    _fmt(state_after),
                    _fmt(state_after - state_before),
                )
            else:
                logger.info("[step %d] DRY RUN", ctrl_obj._step_count)

            ctrl_obj._track_control_interval()
            ctrl_obj._step_count += 1

        logger.info(
            "Episode done: %d steps, aborted=%s",
            ctrl_obj._step_count, aborted,
        )


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description="4DWVLA evaluation client (franky_ext safety infrastructure)"
    )
    p.add_argument("--robot-ip", default="172.16.0.2")
    p.add_argument("--server-host", default="localhost")
    p.add_argument("--server-port", type=int, default=DEFAULT_SERVER_PORT)
    p.add_argument("--task", required=True)
    p.add_argument("--n-exec", type=int, default=10)
    p.add_argument("--control-hz", type=float, default=10.0)
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--use-realsense", action="store_true")
    p.add_argument(
        "--global-camera-serial",
        default=os.environ.get("RS_GLOBAL_SERIAL"),
    )
    p.add_argument(
        "--wrist-camera-serial",
        default=os.environ.get("RS_WRIST_SERIAL"),
    )
    p.add_argument("--log-dir", default=None)
    p.add_argument("--dry-run", action="store_true")
    p.add_argument(
        "--no-keyboard", action="store_true",
        help="Run without keyboard gating (use Ctrl-C to stop)",
    )
    args = p.parse_args()

    _configure_logging(args.log_dir)

    if args.use_realsense:
        logger.info(
            "Camera serials: global=%s wrist=%s",
            args.global_camera_serial, args.wrist_camera_serial,
        )
        if not args.global_camera_serial or not args.wrist_camera_serial:
            logger.warning(
                "A camera serial is missing; RealSense SDK will auto-assign "
                "devices, risking global/wrist swap."
            )

    controller = None
    camera = None
    ctrl = None
    try:
        if not args.dry_run:
            controller = FrankaController(args.robot_ip)
            controller.set_motion_guard(TRAIN_TCP_MIN, TRAIN_TCP_MAX)
        else:
            controller = None

        if args.use_realsense:
            camera = CameraCapture({
                "global": args.global_camera_serial,
                "wrist": args.wrist_camera_serial,
            })

        ctrl = VLAEvalController(
            controller=controller,
            camera=camera,
            server_address=(args.server_host, args.server_port),
            task=args.task,
            n_exec=args.n_exec,
            max_steps=args.max_steps,
            control_hz=args.control_hz,
            dry_run=args.dry_run,
        )
        ctrl.connect()

        if args.no_keyboard or args.dry_run:
            ctrl.run()
        else:
            run_with_keyboard(ctrl, controller)

    except KeyboardInterrupt:
        logger.info("Interrupted")
    except Exception as exc:
        logger.error("Fatal: %s: %s", type(exc).__name__, exc)
        raise
    finally:
        if ctrl is not None:
            try:
                ctrl.disconnect()
            except Exception as exc:
                logger.warning("disconnect failed: %s", exc)
        if camera is not None:
            try:
                camera.close()
            except Exception as exc:
                logger.warning("camera close failed: %s", exc)
        if controller is not None:
            try:
                controller.cleanup()
            except Exception as exc:
                logger.warning("controller cleanup failed: %s", exc)


if __name__ == "__main__":
    main()
