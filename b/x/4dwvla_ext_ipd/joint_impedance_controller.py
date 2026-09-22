"""Joint-impedance backend for the 4DWVLA Franka eval client.

``FrankyControllerDirect.move_joints`` sends one blocking
``JointWaypointMotion`` and waits until that motion settles. This subclass
keeps the gripper, collision limits, TCP fence, and watchdog, and replaces
only the arm command with ``franky.JointImpedanceTracker.set_target``.

The tracker holds the latest joint target at libfranka's 1 kHz torque loop.
``move_joints`` therefore returns immediately. Homing and guard braking still
use a blocking joint waypoint, after the tracker has been stopped: the two
motion generators must not own the FCI session at the same time.
"""
from __future__ import annotations

import logging
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from franky_controller_direct import (
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    JOINT_VEL_LIMITS,
    FrankyControllerDirect,
)
from impedance_gains import joint_velocity_feedforward, load_joint_gains

logger = logging.getLogger(__name__)


class JointImpedanceController(FrankyControllerDirect):
    """Track absolute joint targets with a joint impedance controller."""

    def __init__(
        self,
        robot_ip: str,
        gripper_type: str = "franka",
        stiffness: np.ndarray | None = None,
        damping: np.ndarray | None = None,
        compensate_coriolis: bool | None = None,
    ):
        super().__init__(robot_ip, gripper_type=gripper_type)
        env_k, env_d, env_coriolis = load_joint_gains()
        self._stiffness = np.asarray(
            env_k if stiffness is None else stiffness, dtype=np.float64
        )
        self._damping = np.asarray(
            env_d if damping is None else damping, dtype=np.float64
        )
        self._compensate_coriolis = (
            env_coriolis if compensate_coriolis is None else bool(compensate_coriolis)
        )
        self._tracker = None
        self._prev_target_q: np.ndarray | None = None
        self._prev_target_ts: float | None = None
        # Gripper move/grasp block for up to a few seconds. Running them on
        # this pool lets the impedance loop keep updating the arm target.
        self._gripper_pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="ipd-gripper"
        )
        self._gripper_future = None
        logger.info(
            "Joint impedance armed (tracker starts on the first move_joints): "
            "K=%s D=%s coriolis=%s",
            np.round(self._stiffness, 3).tolist(),
            np.round(self._damping, 3).tolist(),
            self._compensate_coriolis,
        )

    def _build_tracker(self):
        import franky

        return franky.JointImpedanceTracker(
            self._robot,
            stiffness=self._stiffness,
            damping=self._damping,
            compensate_coriolis=self._compensate_coriolis,
        )

    def _ensure_tracker(self) -> None:
        if self._tracker is not None:
            return
        try:
            self._robot.recover_from_errors()
        except Exception as exc:
            logger.warning("recover_from_errors before impedance start: %s", exc)
        self._tracker = self._build_tracker()
        logger.info("Joint impedance tracker started")

    def _stop_tracker(self) -> None:
        tracker = self._tracker
        self._tracker = None
        self._prev_target_q = None
        self._prev_target_ts = None
        if tracker is None:
            return
        try:
            tracker.stop()
        except Exception as exc:
            logger.warning("joint impedance tracker.stop: %s", exc)
        try:
            self._robot.join_motion()
        except Exception:
            pass
        try:
            self._robot.recover_from_errors()
        except Exception as exc:
            logger.warning("recover_from_errors after impedance stop: %s", exc)

    def move_joints(self, joint_positions: np.ndarray):
        """Update the impedance equilibrium. Does not block on arrival."""
        tripped = self.guard_tripped()
        if tripped is not None:
            raise RuntimeError(f"Motion guard tripped: {tripped}")
        q = np.clip(
            np.asarray(joint_positions, dtype=np.float64),
            JOINT_LIMITS_LOWER,
            JOINT_LIMITS_UPPER,
        )
        now = time.perf_counter()
        dq = joint_velocity_feedforward(
            q, self._prev_target_q, self._prev_target_ts, now, JOINT_VEL_LIMITS
        )
        self._ensure_tracker()
        if dq is None:
            self._tracker.set_target(q)
        else:
            self._tracker.set_target(q, dq=dq)
        self._prev_target_q = q
        self._prev_target_ts = now

    def _gripper_busy(self) -> bool:
        future = self._gripper_future
        return future is not None and not future.done()

    def _submit_gripper(self, label: str, fn) -> bool:
        """Start a gripper command and return without waiting for the hand."""
        if self._gripper_busy():
            logger.info("gripper %s skipped; previous command still running", label)
            return False
        self._gripper_future = self._gripper_pool.submit(fn)
        return True

    def gripper_move_width_m(self, width_m: float, speed: float = 0.05) -> None:
        self._submit_gripper(
            "move_width",
            lambda: super(JointImpedanceController, self).gripper_move_width_m(
                width_m, speed
            ),
        )

    def close_gripper(self, force: float = 20.0) -> None:
        self._submit_gripper(
            "grasp",
            lambda: super(JointImpedanceController, self).close_gripper(force),
        )

    def reset_joint(self, reset_pos: list[float]):
        """Blocking HOME move. The impedance tracker must not be running."""
        self._stop_tracker()
        super().reset_joint(reset_pos)

    def _brake(self, kind: str):
        self._stop_tracker()
        super()._brake(kind)

    def stop(self):
        self._stop_tracker()
        super().stop()

    def freeze_at_current(self) -> bool:
        self._stop_tracker()
        return super().freeze_at_current()

    def cleanup(self):
        self._stop_tracker()
        pool = getattr(self, "_gripper_pool", None)
        if pool is not None:
            pool.shutdown(wait=False, cancel_futures=True)
        super().cleanup()
