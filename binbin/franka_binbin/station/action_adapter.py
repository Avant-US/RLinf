"""
Policy action → robot command adapter.

PI0.5 DROID outputs (N, 8) action chunks:
- dims 0..6: joint velocity in [-1, 1] (normalized, NOT rad/s)
- dim 7:     gripper position (0=closed, 1=open), binarized at 0.5

DROID's RobotIKSolver converts velocity → delta via:
    joint_delta = clip(velocity, -1, 1) * max_joint_delta

where max_joint_delta = 0.2 rad (see droid/robot_ik/robot_ik_solver.py).
The robot_server speaks joint positions, so we apply the same transform:
    q_next = q_prev + clip(v, -1, 1) * max_joint_delta * speed_scale
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Sequence, Tuple

import numpy as np


DEFAULT_Q_LO = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
DEFAULT_Q_HI = (+2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973)

DROID_Q_LO = (-0.828, -0.840, -0.843, -2.773, -1.843, +1.172, -2.047)
DROID_Q_HI = (+0.900, +1.385, +0.692, -0.454, +1.732, +3.467, +2.198)

DROID_CONTROL_HZ = 15.0
DROID_MAX_JOINT_DELTA = 0.2


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def parse_actions(actions: Any) -> np.ndarray:
    """Parse actions into a (N, 8) float32 numpy array.

    Accepts numpy arrays or nested lists.
    """
    if isinstance(actions, np.ndarray):
        if actions.ndim == 2 and actions.shape[1] == 8:
            return actions.astype(np.float32)
        raise ValueError(f"Unexpected actions shape: {actions.shape} (expected (N, 8))")

    if isinstance(actions, list):
        arr = np.array(actions, dtype=np.float32)
        if arr.ndim == 2 and arr.shape[1] == 8:
            return arr
        raise ValueError(f"Unexpected actions shape from list: {arr.shape}")

    raise TypeError(f"Unsupported actions type: {type(actions)}")


@dataclass
class ActionAdapterConfig:
    control_hz: float = DROID_CONTROL_HZ
    max_joint_delta: float = DROID_MAX_JOINT_DELTA
    speed_scale: float = 1.0
    max_joint_step: float = 0.20
    max_chunk_drift: float = 0.80

    q_lo: Tuple[float, ...] = DEFAULT_Q_LO
    q_hi: Tuple[float, ...] = DEFAULT_Q_HI
    enforce_droid_distribution: bool = True

    gripper_mode: str = "position"   # position | ignore
    gripper_invert: bool = False
    gripper_lo: float = 0.0
    gripper_hi: float = 0.08
    gripper_max_width: float = 0.08
    gripper_threshold: float = 0.5


class ActionAdapter:
    def __init__(self, cfg: ActionAdapterConfig) -> None:
        self._cfg = cfg
        if len(cfg.q_lo) != 7 or len(cfg.q_hi) != 7:
            raise ValueError("q_lo/q_hi must have 7 elements")

    def _clamp_joints(self, q: np.ndarray) -> np.ndarray:
        for i in range(7):
            lo = float(self._cfg.q_lo[i])
            hi = float(self._cfg.q_hi[i])
            if self._cfg.enforce_droid_distribution:
                lo = max(lo, float(DROID_Q_LO[i]))
                hi = min(hi, float(DROID_Q_HI[i]))
            q[i] = _clamp(float(q[i]), lo, hi)
        return q

    def _gripper_target(self, grip_raw: float) -> float:
        if self._cfg.gripper_mode == "ignore":
            return -1.0  # sentinel: caller should keep current
        binarized = 1.0 if grip_raw > self._cfg.gripper_threshold else 0.0
        if self._cfg.gripper_invert:
            binarized = 1.0 - binarized
        return _clamp(
            binarized * self._cfg.gripper_max_width,
            self._cfg.gripper_lo,
            self._cfg.gripper_hi,
        )

    def make_waypoints(
        self,
        *,
        actions: Any,
        current_q: Tuple[float, ...],
        current_gripper: float,
    ) -> List[Tuple[Tuple[float, ...], float]]:
        """Convert an (N, 8) action chunk into (q_des, g_des) waypoints.

        Each row: 7 normalized joint velocities in [-1, 1] + 1 gripper position.
        Matches DROID: delta = clip(vel, -1, 1) * max_joint_delta * speed_scale.
        """
        rows = parse_actions(actions)
        base_delta = float(self._cfg.max_joint_delta)
        scale = max(0.0, float(self._cfg.speed_scale))
        max_step = float(self._cfg.max_joint_step)

        waypoints: List[Tuple[Tuple[float, ...], float]] = []
        q_prev = np.array(current_q, dtype=np.float64)
        q_base = q_prev.copy()

        for row in rows:
            vel = np.clip(row[:7].astype(np.float64), -1.0, 1.0)
            delta = vel * base_delta * scale
            for j in range(7):
                delta[j] = _clamp(float(delta[j]), -max_step, max_step)

            q_des = q_prev + delta

            if float(self._cfg.max_chunk_drift) > 0.0:
                drift = float(self._cfg.max_chunk_drift)
                q_des = np.clip(q_des, q_base - drift, q_base + drift)

            q_des = self._clamp_joints(q_des)

            grip_raw = float(row[7])
            g_des = self._gripper_target(grip_raw)
            if g_des < 0.0:
                g_des = current_gripper

            waypoints.append((tuple(float(x) for x in q_des), float(g_des)))
            q_prev = q_des.copy()

        return waypoints
