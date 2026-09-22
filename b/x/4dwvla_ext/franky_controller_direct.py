#!/usr/bin/env python3
"""Standalone safe Franka controller -- no Ray dependency.

Replicates the safety mechanisms of FrankyControllerExtended
(motion guard, watchdog, collision tightening, trip recovery)
using the same constants from franky_ext.motion_limits,
but without inheriting from Worker (which requires Ray).

Usage:
    controller = FrankyControllerDirect("172.16.0.2")
    controller.set_motion_guard(tcp_min, tcp_max)
    controller.move_joints(target_q)
    controller.cleanup()
"""
from __future__ import annotations

import logging
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

# franky_ext is Ray-free; FrankaLibfrankaGripper still imports rlinf.BaseGripper.
_EXT_PATH = os.environ.get("RLINF_EXT_PATH", "/workspace/RLinf/b/x")
_REPO_PATH = os.environ.get("REPO_PATH", str(Path(_EXT_PATH).resolve().parent))
for _p in (_EXT_PATH, _REPO_PATH):
    if _p and _p not in sys.path:
        sys.path.insert(0, _p)

# Cube-place placeholder. A ~15 mm charger body is outside 0.046 ± 0.012.
_PLACEHOLDER_CUBE_WIDTH_M = "0.046"
_PLUG_WIDTH_M = "0.010"
_PLUG_HOLD_TOL_M = "0.008"
_PLUG_GRASP_FORCE_N = "20"

from franky_ext.motion_limits import (
    GUARD_MARGIN_M_DEFAULT,
    GUARD_FLOOR_MARGIN_M_DEFAULT,
    GUARD_MAX_LAG_M_DEFAULT,
    GUARD_MAX_DQ_RAD_S_DEFAULT,
    GUARD_RECOVERY_BUDGET_DEFAULT,
    PANDA_MAX_REACH_M,
    PANDA_SHOULDER_Z_M,
    REACH_WARN_FRACTION,
    guard_margin_m,
    guard_floor_margin_m,
    guard_max_lag_m,
    guard_max_dq_rad_s,
    guard_recovery_budget,
    cartesian_collision_thresholds,
    reach_radius_m,
)

logger = logging.getLogger(__name__)


def _ensure_plug_gripper_env() -> None:
    """Pin the InternVLA plug-eval grasp window unless the operator already set one.

    RLiKx ``setup_franky.sh`` uses ``FRANKA_CUBE_WIDTH_M=0.015``. Hold tolerance
    is 10 mm so an empty hand at ~0 m is *not* counted as holding (0.015 ± 0.015
    would include empty-closed).

    修复 D (b/d/frk1/grperr_1.md R4/§4 修复 D): this window has NOT been
    verified against the real plug. Demonstration data closes to <1mm width
    87% of the time (mean 2.6mm); if the real robot does the same,
    ``FrankaLibfrankaGripper.close()``/``gripper_holding()`` will silently
    treat every real grasp as a miss. Override ``FRANKA_CUBE_WIDTH_M`` /
    ``FRANKA_HOLD_TOL_M`` (e.g. via
    ``configs/franka_plug_eval.env``) once the plug has been measured with
    calipers -- do not just trust these defaults.
    """
    os.environ.setdefault("FRANKA_GRASP_FORCE", _PLUG_GRASP_FORCE_N)
    current_w = os.environ.get("FRANKA_CUBE_WIDTH_M")
    if current_w is None or current_w.strip() in ("", _PLACEHOLDER_CUBE_WIDTH_M):
        os.environ["FRANKA_CUBE_WIDTH_M"] = _PLUG_WIDTH_M
        logger.info(
            "plug eval: FRANKA_CUBE_WIDTH_M=%s (replaced unset/placeholder 0.046)",
            _PLUG_WIDTH_M,
        )
    os.environ.setdefault("FRANKA_HOLD_TOL_M", _PLUG_HOLD_TOL_M)
    logger.info(
        "plug eval grasp window: FRANKA_CUBE_WIDTH_M=%s +/- FRANKA_HOLD_TOL_M=%s "
        "(UNVERIFIED against the real plug -- see grperr_1.md R4 before "
        "trusting close()/gripper_holding() results; override via "
        "configs/franka_plug_eval.env)",
        os.environ["FRANKA_CUBE_WIDTH_M"], os.environ["FRANKA_HOLD_TOL_M"],
    )


# FR3v2.1 joint limits (from rlinf.envs.realworld.franka.franky_controller)
JOINT_LIMITS_LOWER = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_LIMITS_UPPER = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])
JOINT_VEL_LIMITS   = np.array([ 2.075,   2.075,   2.075,   2.075,   2.51,    2.51,    2.51])

# Watchdog and braking constants (from controller_extended.py)
_WATCHDOG_PERIOD_S = 0.02
_BRAKE_DWELL_S = 0.25
_BRAKE_SETTLED_RAD_S = 0.02


class FrankyControllerDirect:
    """Safe Franka controller with motion guard and watchdog.

    Replicates FrankyControllerExtended safety mechanisms:
    - Collision behavior tightening (_tighten_collision_behavior)
    - Motion guard: TCP fence with margin (set_motion_guard)
    - Watchdog thread: 50Hz continuous monitoring (_watchdog_loop)
    - Joint velocity norm limiting
    - Trip detection and recovery
    """

    def __init__(self, robot_ip: str, gripper_type: str = "franka"):
        import franky

        self._robot = franky.Robot(robot_ip)
        self._robot.recover_from_errors()
        self._robot.relative_dynamics_factor = 0.2

        if gripper_type == "franka":
            _ensure_plug_gripper_env()
            from franky_ext.franka_libfranka_gripper import FrankaLibfrankaGripper

            self._gripper = FrankaLibfrankaGripper(robot_ip)
        else:
            self._gripper = None

        self._prev_target_q = None
        self._prev_target_ts = 0.0

        # Motion guard state
        self._guard_min_xyz: Optional[np.ndarray] = None
        self._guard_max_xyz: Optional[np.ndarray] = None
        self._guard_max_lag = guard_max_lag_m()
        self._guard_max_dq = guard_max_dq_rad_s()
        self._guard_enabled = False
        self._guard_trip_reason: Optional[str] = None
        self._guard_trip_lock = threading.Lock()
        self._guard_recoveries_used = 0
        self._guard_recovery_budget = guard_recovery_budget()

        # Watchdog
        self._watchdog: Optional[threading.Thread] = None
        self._watchdog_stop = threading.Event()

        # Tighten collision behavior (same as FrankyControllerExtended)
        self._tighten_collision_behavior()

        logger.info(
            "FrankyControllerDirect: connected to %s, guard_margin=%.3fm, "
            "guard_max_dq=%.2frad/s, recovery_budget=%d",
            robot_ip, guard_margin_m(), self._guard_max_dq,
            self._guard_recovery_budget,
        )

    # -- Collision behavior (from FrankyControllerExtended) ---------------

    def _tighten_collision_behavior(self):
        """Replicates FrankyControllerExtended._tighten_collision_behavior()."""
        thresholds = cartesian_collision_thresholds()
        torque_lower = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        torque_upper = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        try:
            self._robot.set_collision_behavior(
                lower_torque_threshold=torque_lower,
                upper_torque_threshold=torque_upper,
                lower_force_threshold=thresholds,
                upper_force_threshold=thresholds,
            )
            logger.info("Collision behavior tightened")
        except Exception as e:
            logger.warning("Could not tighten collision behavior: %s", e)

    # -- Motion guard -----------------------------------------------------

    def set_motion_guard(
        self,
        limit_min_xyz: np.ndarray,
        limit_max_xyz: np.ndarray,
        *,
        margin: Optional[float] = None,
        floor_margin: Optional[float] = None,
    ):
        """Install TCP position fence.
        Replicates FrankyControllerExtended.set_motion_guard().
        """
        if margin is None:
            margin = guard_margin_m()
        if floor_margin is None:
            floor_margin = guard_floor_margin_m()

        self._guard_min_xyz = np.array(limit_min_xyz, dtype=np.float64) - margin
        self._guard_min_xyz[2] = limit_min_xyz[2] - floor_margin
        self._guard_max_xyz = np.array(limit_max_xyz, dtype=np.float64) + margin
        self._guard_enabled = True

        logger.info(
            "Motion guard set: min=%s, max=%s (margin=%.3f, floor=%.3f)",
            np.round(self._guard_min_xyz, 4).tolist(),
            np.round(self._guard_max_xyz, 4).tolist(),
            margin, floor_margin,
        )

        if self._watchdog is None or not self._watchdog.is_alive():
            self._start_watchdog()

    def clear_motion_guard(self):
        self._guard_enabled = False
        self._stop_watchdog()

    def guard_tripped(self) -> Optional[str]:
        with self._guard_trip_lock:
            return self._guard_trip_reason

    # -- Watchdog (from FrankyControllerExtended) -------------------------

    def _start_watchdog(self):
        self._watchdog_stop.clear()
        self._watchdog = threading.Thread(
            target=self._watchdog_loop,
            args=(self._watchdog_stop,),
            daemon=True,
            name="motion-guard-watchdog",
        )
        self._watchdog.start()
        logger.info("Watchdog started (%.0f Hz)", 1.0 / _WATCHDOG_PERIOD_S)

    def _stop_watchdog(self):
        if self._watchdog is not None:
            self._watchdog_stop.set()
            self._watchdog.join(timeout=2.0)
            self._watchdog = None

    def _watchdog_loop(self, stop_event: threading.Event):
        """50Hz motion guard check.
        Replicates FrankyControllerExtended._watchdog_loop().
        """
        while not stop_event.is_set():
            try:
                violation = self._evaluate_guard()
                if violation is not None:
                    kind, desc = violation
                    self._abort_motion(kind, desc)
            except Exception as e:
                logger.error("Watchdog error: %s", e)
            stop_event.wait(_WATCHDOG_PERIOD_S)

    def _evaluate_guard(self) -> Optional[tuple[str, str]]:
        """Check TCP position against fence + joint velocity.
        Replicates FrankyControllerExtended._evaluate_guard().
        """
        if not self._guard_enabled:
            return None

        try:
            state = self._robot.state
            tcp_xyz = np.array(state.O_T_EE.translation)
        except Exception:
            return None

        # Fence check
        if self._guard_min_xyz is not None and self._guard_max_xyz is not None:
            below = tcp_xyz < self._guard_min_xyz
            above = tcp_xyz > self._guard_max_xyz
            if np.any(below) or np.any(above):
                axis = ["X", "Y", "Z"]
                viol = []
                for i in range(3):
                    if below[i]:
                        viol.append(f"{axis[i]}={tcp_xyz[i]:.4f}<{self._guard_min_xyz[i]:.4f}")
                    elif above[i]:
                        viol.append(f"{axis[i]}={tcp_xyz[i]:.4f}>{self._guard_max_xyz[i]:.4f}")
                return ("fence", f"TCP outside fence: {', '.join(viol)}")

        # Joint velocity check
        try:
            dq = np.array(state.dq[:7])
            dq_norm = np.linalg.norm(dq)
            if dq_norm > self._guard_max_dq:
                return ("dq", f"|dq|={dq_norm:.3f} > {self._guard_max_dq:.3f} rad/s")
        except Exception:
            pass

        # Reach check
        r = reach_radius_m(tcp_xyz)
        if r > PANDA_MAX_REACH_M * REACH_WARN_FRACTION:
            logger.warning("NEAR-SINGULAR: reach=%.3fm (%.0f%% of max)", r, r / PANDA_MAX_REACH_M * 100)

        return None

    def _abort_motion(self, kind: str, reason: str):
        with self._guard_trip_lock:
            if self._guard_trip_reason is not None:
                return
            self._guard_trip_reason = f"{kind}: {reason}"
        logger.error("MOTION GUARD TRIP [%s]: %s", kind, reason)
        self._brake(kind)

    def _brake(self, kind: str):
        """Replicates FrankyControllerExtended._brake()."""
        try:
            if kind in ("fence", "orient"):
                self._robot.stop()
            else:
                try:
                    import franky
                    current_q = list(self._robot.state.q[:7])
                    motion = franky.JointWaypointMotion([franky.JointWaypoint(current_q)])
                    saved = self._robot.relative_dynamics_factor
                    self._robot.relative_dynamics_factor = 0.05
                    self._robot.move(motion)
                    self._robot.relative_dynamics_factor = saved
                except Exception:
                    self._robot.stop()
        except Exception as e:
            logger.error("Brake failed: %s", e)

    def recover_from_guard_trip(self) -> dict:
        """Replicates FrankyControllerExtended.recover_from_guard_trip()."""
        with self._guard_trip_lock:
            was_tripped = self._guard_trip_reason
            if was_tripped is None:
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
            logger.info("Guard recovery %d/%d", self._guard_recoveries_used, self._guard_recovery_budget)
            return {"recovered": True, "was_tripped": True, "previous_reason": was_tripped}
        except Exception as e:
            return {"recovered": False, "was_tripped": True, "error": str(e)}

    # -- Robot control API ------------------------------------------------

    def get_state(self):
        state = self._robot.state
        q = np.array(state.q[:7], dtype=np.float64)
        dq = np.array(state.dq[:7], dtype=np.float64)
        tcp_xyz = np.array(state.O_T_EE.translation)
        gw = self.gripper_width()
        return {"arm_joint_position": q, "arm_joint_velocity": dq,
                "tcp_position": tcp_xyz, "gripper_width": gw}

    def move_joints(self, joint_positions: np.ndarray):
        """Replicates FrankyController.move_joints() with guard check."""
        tripped = self.guard_tripped()
        if tripped is not None:
            raise RuntimeError(f"Motion guard tripped: {tripped}")
        clipped = np.clip(joint_positions, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
        import franky
        motion = franky.JointWaypointMotion([franky.JointWaypoint(clipped.tolist())])
        self._robot.move(motion)

    def reset_joint(self, reset_pos: list[float]):
        """Replicates FrankyController.reset_joint()."""
        import franky
        motion = franky.JointWaypointMotion([franky.JointWaypoint(reset_pos)])
        saved = self._robot.relative_dynamics_factor
        self._robot.relative_dynamics_factor = 0.1
        self._robot.move(motion)
        self._robot.relative_dynamics_factor = saved

    def open_gripper(self):
        if self._gripper is None:
            return
        self._gripper.open(speed=0.05)

    def close_gripper(self, force: float = 20.0):
        """Calibrated-width grasp via FrankaLibfrankaGripper.

        Skip-if-holding, 6 s timeout, and 20 N hold force live in that class.
        A miss (empty hand, wrong object size) is logged rather than aborting
        the VLA episode — the policy often commands close before contact.
        A hung hand (timeout) still raises.
        """
        if self._gripper is None:
            return
        try:
            self._gripper.close(speed=0.05, force=force)
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

    def gripper_move_width_m(self, width_m: float, speed: float = 0.05) -> None:
        if self._gripper is None:
            return
        fn = getattr(self._gripper, "move_width_m", None)
        if not callable(fn):
            raise RuntimeError("gripper does not support move_width_m()")
        fn(width_m, speed)

    def gripper_max_width(self) -> Optional[float]:
        """Width the hand reports as fully open, in libfranka's own units.

        This can differ from the caliper-measured finger gap when the hand's
        homing offset is stale; callers must clamp commands to this value, not
        to the physical measurement (see grperr_1.2.md Q1).
        """
        if self._gripper is None:
            return None
        try:
            value = getattr(self._gripper, "max_width", None)
            return None if value is None else float(value)
        except Exception:
            return None

    def gripper_is_open(self) -> bool:
        if self._gripper is None:
            return True
        return bool(getattr(self._gripper, "is_open", True))

    def gripper_holding(self) -> bool:
        if self._gripper is None:
            return False
        fn = getattr(self._gripper, "_hardware_holding", None)
        if callable(fn):
            return bool(fn())
        return False

    def stop(self):
        try:
            self._robot.stop()
        except Exception as e:
            logger.error("stop failed: %s", e)

    def recover_from_errors(self):
        self._robot.recover_from_errors()

    def freeze_at_current(self) -> bool:
        try:
            import franky
            q = list(self._robot.state.q[:7])
            saved = self._robot.relative_dynamics_factor
            self._robot.relative_dynamics_factor = 0.05
            self._robot.move(franky.JointWaypointMotion([franky.JointWaypoint(q)]))
            self._robot.relative_dynamics_factor = saved
            return True
        except Exception:
            return False

    def cleanup(self):
        self._stop_watchdog()
        self.freeze_at_current()
        gripper_cleanup = getattr(self._gripper, "cleanup", None)
        if callable(gripper_cleanup):
            gripper_cleanup()
        logger.info("FrankyControllerDirect cleanup complete")
