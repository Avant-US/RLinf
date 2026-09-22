"""4DWVLA joint env whose arm commands are impedance targets.

Camera, gripper semantics, and the L1-L8 safety checks stay in
``FrankyJointEnv``. Only ``_make_controller`` is replaced, so ``step()``
still clips the action and then calls ``move_joints``; that call now updates
``JointImpedanceTracker`` instead of waiting for ``Robot.move``.
"""
from __future__ import annotations

import logging

import numpy as np

from franky_joint_env import TRAIN_TCP_MAX, TRAIN_TCP_MIN, FrankyJointEnv
from joint_impedance_controller import JointImpedanceController

logger = logging.getLogger(__name__)

# Equilibrium step cap. The shared env allows 0.15 rad because a waypoint
# move waits until that step finishes. An impedance tick must not jump the
# spring target that far.
IPD_MAX_JOINT_STEP_RAD = 0.05


def limit_equilibrium_step(
    action_arm: np.ndarray,
    current_q: np.ndarray,
    max_step_rad: float = IPD_MAX_JOINT_STEP_RAD,
) -> tuple[np.ndarray, bool]:
    """Scale a joint target so no joint moves more than ``max_step_rad``.

    Returns the limited target and whether a scale was applied. The shared
    safety check still runs afterwards; this only makes the impedance cap
    stricter than that 0.15 rad limit.
    """
    target = np.asarray(action_arm, dtype=np.float64).copy()
    current = np.asarray(current_q, dtype=np.float64)
    delta = target - current
    peak = float(np.max(np.abs(delta))) if delta.size else 0.0
    if peak <= float(max_step_rad) or peak <= 0.0:
        return target, False
    return current + delta * (float(max_step_rad) / peak), True


class ImpedanceJointEnv(FrankyJointEnv):
    """Same observation and action contract as ``FrankyJointEnv``."""

    def __init__(
        self,
        robot_ip="172.16.0.2",
        control_hz=10.0,
        is_dummy=False,
        use_realsense=False,
        camera_serials=None,
        stiffness: np.ndarray | None = None,
        damping: np.ndarray | None = None,
    ):
        self._ipd_stiffness = None if stiffness is None else np.asarray(stiffness)
        self._ipd_damping = None if damping is None else np.asarray(damping)
        super().__init__(
            robot_ip=robot_ip,
            control_hz=control_hz,
            is_dummy=is_dummy,
            use_realsense=use_realsense,
            camera_serials=camera_serials,
        )
        if self._controller is not None:
            logger.info(
                "arm backend=joint_impedance tcp_fence=%s..%s control_hz=%.1f",
                np.round(TRAIN_TCP_MIN, 3).tolist(),
                np.round(TRAIN_TCP_MAX, 3).tolist(),
                control_hz,
            )

    def _make_controller(self, robot_ip: str):
        return JointImpedanceController(
            robot_ip,
            stiffness=self._ipd_stiffness,
            damping=self._ipd_damping,
        )

    def step(self, action):
        action = np.array(action, dtype=np.float64, copy=True)
        if (
            not self._is_dummy
            and self._controller is not None
            and action.shape[0] >= 7
        ):
            current_q = self._controller.get_state()["arm_joint_position"]
            limited, scaled = limit_equilibrium_step(action[:7], current_q)
            if scaled:
                logger.info(
                    "impedance step cap: equilibrium scaled to %.3f rad",
                    IPD_MAX_JOINT_STEP_RAD,
                )
            action[:7] = limited
        return super().step(action)
