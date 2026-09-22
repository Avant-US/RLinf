"""Joint-impedance gains and velocity feed-forward.

Defaults are the joint impedance values already used by
``rlinf.envs.realworld.franka.franky_controller.FrankyController``.
Override them per robot or experiment; do not hard-code a second copy
in the controller.
"""
from __future__ import annotations

import os

import numpy as np

# q1-q4 match rlinf FrankyController. q5 and q6 are doubled and q7 is
# quadrupled so the soft wrist can track an absolute target inside one
# impedance tick. Damping scales with sqrt(stiffness ratio) to keep the
# damping ratio of those three joints.
DEFAULT_JOINT_STIFFNESS = np.array(
    [103.75, 265.734, 227.273, 221.445, 27.0, 25.6, 20.5],
    dtype=np.float64,
)
DEFAULT_JOINT_DAMPING = np.array(
    [16.7, 40.263, 25.0, 12.862, 2.12, 2.83, 2.66],
    dtype=np.float64,
)
DQ_MIN_DT_S = 1e-3


def parse_gain_vector(raw: str | None, default: np.ndarray, name: str) -> np.ndarray:
    """Parse a comma-separated 7-vector, or return ``default`` when unset."""
    if raw is None or raw.strip() == "":
        return np.array(default, dtype=np.float64, copy=True)
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    if len(parts) != 7:
        raise ValueError(f"{name} must have 7 comma-separated values, got {len(parts)}")
    values = np.array([float(p) for p in parts], dtype=np.float64)
    if not np.isfinite(values).all() or np.any(values < 0.0):
        raise ValueError(f"{name} must be finite and non-negative, got {values.tolist()}")
    return values


def load_joint_gains() -> tuple[np.ndarray, np.ndarray, bool]:
    """Read stiffness, damping, and Coriolis compensation from the environment."""
    stiffness = parse_gain_vector(
        os.environ.get("VLA_IPD_STIFFNESS"),
        DEFAULT_JOINT_STIFFNESS,
        "VLA_IPD_STIFFNESS",
    )
    damping = parse_gain_vector(
        os.environ.get("VLA_IPD_DAMPING"),
        DEFAULT_JOINT_DAMPING,
        "VLA_IPD_DAMPING",
    )
    raw = os.environ.get("VLA_IPD_COMPENSATE_CORIOLIS", "1").strip().lower()
    compensate = raw not in ("0", "false", "no", "off")
    return stiffness, damping, compensate


def joint_velocity_feedforward(
    target_q: np.ndarray,
    prev_q: np.ndarray | None,
    prev_ts: float | None,
    now: float,
    vel_limits: np.ndarray,
) -> np.ndarray | None:
    """Finite-difference target velocity, clipped to Franka joint limits.

    The first target has no previous sample, so feed-forward is omitted.
    ``FrankyController.move_joints`` does the same: without ``dq``, a 10 Hz
    impedance update lags and overshoots.
    """
    if prev_q is None or prev_ts is None:
        return None
    dt = max(float(now) - float(prev_ts), DQ_MIN_DT_S)
    dq = (np.asarray(target_q, dtype=np.float64) - np.asarray(prev_q, dtype=np.float64)) / dt
    return np.clip(dq, -vel_limits, vel_limits)
