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
affine = robot.state.O_T_EE
xyz = np.asarray(affine.translation, dtype=np.float64)
quat = np.asarray(affine.quaternion, dtype=np.float64)
euler = R.from_quat(quat).as_euler("xyz")
print(json.dumps(np.concatenate([xyz, euler]).tolist()))
"""


def probe_tcp_pose_euler(robot_ip: str, *, python: str | None = None) -> list[float]:
    """Read current TCP ``[x,y,z, roll,pitch,yaw]`` in a child process."""
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
    pose = json.loads(line)
    if len(pose) != 6:
        raise ValueError(f"expected 6-D pose, got {pose!r}")
    return [float(x) for x in pose]


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
