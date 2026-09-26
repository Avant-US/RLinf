"""Shared loader for the Franka plug-task HOME pose."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

JOINT_LIMITS_LOWER = np.array(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    dtype=np.float64,
)
JOINT_LIMITS_UPPER = np.array(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    dtype=np.float64,
)


def default_home_path() -> Path:
    """Return the repository-local plug-task HOME file."""
    return Path(__file__).with_name("home_pose.json")


def load_home_pose(
    path: str | Path | None = None,
) -> tuple[dict[str, Any], np.ndarray]:
    """Load and validate the seven FR3v2.1 HOME joint positions."""
    home_path = Path(path) if path is not None else default_home_path()
    metadata = json.loads(home_path.read_text(encoding="utf-8"))
    joints = np.asarray(metadata.get("joint_position_rad"), dtype=np.float64)
    if joints.shape != (7,) or not np.all(np.isfinite(joints)):
        raise ValueError(
            f"{home_path}: joint_position_rad must contain seven finite values"
        )
    if np.any(joints < JOINT_LIMITS_LOWER) or np.any(joints > JOINT_LIMITS_UPPER):
        raise ValueError(
            f"{home_path}: HOME is outside FR3v2.1 joint limits; "
            f"q={joints.tolist()}"
        )
    return metadata, joints


def load_home_joints(path: str | Path | None = None) -> np.ndarray:
    """Return a copy of the validated HOME joint vector."""
    _, joints = load_home_pose(path)
    return joints.copy()
