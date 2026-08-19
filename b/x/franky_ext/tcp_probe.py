"""TCP pose probe helpers (subprocess franky, releases FCI before env connect)."""

from __future__ import annotations

import json
import os
import subprocess
import sys

import numpy as np

_PROBE_SCRIPT = r"""
import json, os, sys
import franky
import numpy as np
from scipy.spatial.transform import Rotation as R

ip = os.environ["FRANKA_ROBOT_IP"]
robot = franky.Robot(ip)
state = robot.state
affine = state.O_T_EE
xyz = np.asarray(affine.translation, dtype=np.float64)
quat = np.asarray(affine.quaternion, dtype=np.float64)
euler = R.from_quat(quat).as_euler("xyz")
print(json.dumps({
    "pose": np.concatenate([xyz, euler]).tolist(),
    "robot_mode": str(state.robot_mode),
    "has_errors": bool(robot.has_errors),
}))
"""

# Motion-ready mode. Guiding/UserStopped/Reflex all reject robot.move().
_READY_MODE = "RobotMode.Idle"

MODE_HINT = {
    "RobotMode.UserStopped": (
        "the user-stop button on the Franka enabling device is pressed. "
        "libfranka rejects every motion with 'Move command rejected: command "
        "not possible in the current mode (\"User stopped\")', while state "
        "reads and gripper commands keep working -- so reset looks like it ran "
        "but the arm never moves (dz=0.0000). Pull the user-stop button up, "
        "confirm Desk shows joints unlocked and FCI activated, then retry."
    ),
    "RobotMode.Guiding": (
        "the enabling device is held in guiding mode; release it so FCI can "
        "command motion."
    ),
    "RobotMode.Reflex": (
        "a reflex (collision / limit) is latched; recover in Desk or call "
        "recover_from_errors(), then retry."
    ),
}


def probe_robot_state(robot_ip: str, *, python: str | None = None) -> dict:
    """Read TCP pose, ``robot_mode`` and error flag in a child process.

    Runs in a subprocess so the FCI control connection is released before the
    env's own controller connects.
    """
    env = {**os.environ, "FRANKA_ROBOT_IP": robot_ip}
    proc = subprocess.run(
        [python or sys.executable, "-c", _PROBE_SCRIPT],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"TCP probe failed (exit {proc.returncode}):\n{proc.stderr.strip()}"
        )
    line = proc.stdout.strip().splitlines()[-1]
    info = json.loads(line)
    pose = info["pose"]
    if len(pose) != 6:
        raise ValueError(f"expected 6-D pose, got {pose!r}")
    info["pose"] = [float(x) for x in pose]
    return info


def probe_tcp_pose_euler(robot_ip: str, *, python: str | None = None) -> list[float]:
    """Read current TCP ``[x,y,z, roll,pitch,yaw]`` in a child process."""
    return probe_robot_state(robot_ip, python=python)["pose"]


def describe_robot_mode(mode: str) -> str:
    """Human-readable reason why ``mode`` cannot execute motions."""
    hint = MODE_HINT.get(mode, "FCI only executes motions in RobotMode.Idle.")
    return f"robot_mode={mode}: {hint}"


def require_motion_ready(mode: str) -> None:
    """Raise before commanding the arm if ``mode`` rejects motion."""
    if mode != _READY_MODE:
        raise RuntimeError(describe_robot_mode(mode))


def ee_pose_limits_from_probe(
    rest_pose: list[float] | np.ndarray,
    *,
    xyz_margin: float,
    rpy_margin: float,
) -> tuple[list[float], list[float]]:
    """Build 6-D ``ee_pose_limit_{min,max}`` centered on the probed TCP pose."""
    xyz = np.asarray(rest_pose[:3], dtype=np.float64)
    rpy = np.asarray(rest_pose[3:6], dtype=np.float64)
    xyz_m = np.full(3, xyz_margin, dtype=np.float64)
    rpy_m = np.full(3, rpy_margin, dtype=np.float64)
    lim_min = np.concatenate([xyz - xyz_m, rpy - rpy_m]).tolist()
    lim_max = np.concatenate([xyz + xyz_m, rpy + rpy_m]).tolist()
    return lim_min, lim_max


def peg_target_and_reset_from_probe(
    probed_pose: list[float] | np.ndarray,
    *,
    z_offset_m: float,
) -> tuple[list[float], list[float]]:
    """Peg task poses: ``target`` = probed TCP, ``reset`` = target + z hover."""
    target = [float(x) for x in probed_pose[:6]]
    reset = target.copy()
    reset[2] += float(z_offset_m)
    return target, reset
