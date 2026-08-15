#!/usr/bin/env python3
"""Step 5: safe real-robot smoke for FrankyFrankaEnv-v1 (no camera).

Default mode holds the arm at its current pose:
  - probe TCP in a subprocess (releases FCI before env connects)
  - ``safe_smoke_hold=True`` skips init/reset interpolate moves
  - world-frame observations (``use_relative_frame=False``)
  - 3 zero-action steps, gripper action forced to 0
  - optional ``--micro-nudge``: multi-step TCP nudge until到位 (default 5 mm on +x, then return)
  - ``--nudge-settle-s`` (default 0.1 s): wait after each nudge step so impedance can converge
  - ``ee_pose_limit_{min,max}`` auto-centered on probed TCP (so nudge is not clipped to origin)

Usage (container, after Desk FCI + unlock):
  source b/x/configs/setup_before_ray_5090.sh
  ray start --head --port=6379
  python b/x/scripts/step5_test_env_robot.py --connect-only   # pose only, no env
  python b/x/scripts/step5_test_env_robot.py                  # safe smoke
  python b/x/scripts/step5_test_env_robot.py --micro-nudge    # + zero steps + 5mm nudge + return
  ray stop
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

import gymnasium as gym
import numpy as np
import ray
from scipy.spatial.transform import Rotation as R

REPO = os.environ.get("REPO_PATH", os.path.abspath(os.path.join(__file__, "../../..")))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

import franky_ext.tasks.register  # noqa: F401

from rlinf.scheduler import FrankaHWInfo
from rlinf.scheduler.hardware.robots.franka import FrankaConfig, FrankaRobot

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


def probe_tcp_pose_euler(robot_ip: str) -> list[float]:
    """Read current TCP [x,y,z, roll,pitch,yaw] in a child process (releases FCI)."""
    env = {**os.environ, "FRANKA_ROBOT_IP": robot_ip}
    proc = subprocess.run(
        [sys.executable, "-c", _PROBE_SCRIPT],
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


def _ee_pose_limits_from_probe(
    rest_pose: list[float],
    *,
    xyz_margin: float,
    rpy_margin: float,
) -> tuple[list[float], list[float]]:
    """Build a 6-D safety box [xyz, euler_xyz] centered on the probed TCP pose."""
    xyz = np.asarray(rest_pose[:3], dtype=np.float64)
    rpy = np.asarray(rest_pose[3:6], dtype=np.float64)
    xyz_m = np.full(3, xyz_margin, dtype=np.float64)
    rpy_m = np.full(3, rpy_margin, dtype=np.float64)
    lim_min = np.concatenate([xyz - xyz_m, rpy - rpy_m]).tolist()
    lim_max = np.concatenate([xyz + xyz_m, rpy + rpy_m]).tolist()
    return lim_min, lim_max


def _build_hardware(robot_ip: str) -> FrankaHWInfo:
    franka_config = FrankaConfig(
        node_rank=0,
        robot_ip=robot_ip,
        camera_serials=["000000000000"],
        gripper_type="franka",
        disable_validate=True,
    )
    return FrankaHWInfo(
        type=FrankaRobot.HW_TYPE,
        model=FrankaRobot.HW_TYPE,
        config=franka_config,
    )


def _safe_env_cfg() -> dict:
    return {
        "use_spacemouse": False,
        "use_relative_frame": False,
        "no_gripper": True,
    }


def _make_env(
    robot_ip: str,
    rest_pose: list[float],
    *,
    safe_hold: bool,
    xyz_margin: float,
    rpy_margin: float,
) -> gym.Env:
    lim_min, lim_max = _ee_pose_limits_from_probe(
        rest_pose, xyz_margin=xyz_margin, rpy_margin=rpy_margin
    )
    return gym.make(
        "FrankyFrankaEnv-v1",
        override_cfg={
            "is_dummy": False,
            "robot_ip": robot_ip,
            "camera_serials": ["000000000000"],
            "gripper_type": "franka",
            "target_ee_pose": rest_pose,
            "reset_ee_pose": rest_pose,
            "ee_pose_limit_min": lim_min,
            "ee_pose_limit_max": lim_max,
            "enable_camera_player": False,
            "enable_random_reset": False,
            "safe_smoke_hold": safe_hold,
        },
        worker_info=None,
        hardware_info=_build_hardware(robot_ip),
        env_idx=0,
        env_cfg=_safe_env_cfg(),
    )


def _print_tcp(label: str, obs: dict) -> None:
    tcp = obs["state"]["tcp_pose"]
    print(f"{label}: xyz={tcp[:3]} euler={tcp[3:6]}")


def _close_env(env: gym.Env) -> None:
    try:
        env.close()
    except AttributeError as exc:
        print(f"env.close() warning (ignored for smoke): {exc}")


def _zero_action(env: gym.Env) -> np.ndarray:
    return np.zeros(env.action_space.shape, dtype=np.float32)


def _refresh_tcp_obs(env: gym.Env, obs: dict) -> dict:
    """Re-read TCP from controller after settle wait (quat -> euler for obs)."""
    tcp = np.asarray(env.get_wrapper_attr("get_tcp_pose")(), dtype=np.float64)
    refreshed = {**obs, "state": {**obs["state"]}}
    refreshed["state"]["tcp_pose"] = np.concatenate(
        [tcp[:3], R.from_quat(tcp[3:].copy()).as_euler("xyz")]
    ).astype(np.float32)
    return refreshed


def _step_axis_toward(
    env: gym.Env,
    obs: dict,
    *,
    axis: int,
    goal_value: float,
    max_step: float,
    tolerance: float,
    max_steps: int,
    settle_s: float,
    label: str,
) -> tuple[dict, int]:
    """Repeated env.step until tcp[axis] is within tolerance of goal_value."""
    axis_names = ("x", "y", "z")
    steps = 0
    while steps < max_steps:
        current = float(obs["state"]["tcp_pose"][axis])
        remaining = goal_value - current
        if abs(remaining) <= tolerance:
            print(
                f"{label}: converged in {steps} step(s), "
                f"err={remaining * 1000:.2f} mm on {axis_names[axis]}"
            )
            break
        step_val = float(np.clip(remaining, -max_step, max_step))
        action = _zero_action(env)
        action[axis] = np.float32(step_val)
        obs, _, _, _, _ = env.step(action)
        if settle_s > 0:
            time.sleep(settle_s)
            obs = _refresh_tcp_obs(env, obs)
        steps += 1
        if steps == 1 or steps % 5 == 0:
            _print_tcp(f"{label} step {steps}", obs)
    else:
        current = float(obs["state"]["tcp_pose"][axis])
        print(
            f"WARNING: {label} did not converge in {max_steps} steps, "
            f"err={(goal_value - current) * 1000:.2f} mm on {axis_names[axis]}"
        )
    return obs, steps


def _run_micro_nudge(
    env: gym.Env,
    obs: dict,
    *,
    axis: int,
    delta: float,
    max_step: float,
    tolerance: float,
    max_steps: int,
    settle_s: float,
) -> dict:
    axis_names = ("x", "y", "z")
    start_xyz = obs["state"]["tcp_pose"][:3].copy()
    goal_out = float(start_xyz[axis] + delta)
    settle_note = f", settle={settle_s * 1000:.0f} ms/step" if settle_s > 0 else ""
    print(
        f"micro-nudge: move +{delta * 1000:.1f} mm on base-frame {axis_names[axis]} "
        f"(multi-step, per-step max={max_step * 1000:.1f} mm, "
        f"tol={tolerance * 1000:.1f} mm{settle_note})"
    )
    obs, n_out = _step_axis_toward(
        env,
        obs,
        axis=axis,
        goal_value=goal_out,
        max_step=max_step,
        tolerance=tolerance,
        max_steps=max_steps,
        settle_s=settle_s,
        label="nudge-out",
    )
    _print_tcp("after nudge", obs)
    moved_mm = float((obs["state"]["tcp_pose"][axis] - start_xyz[axis]) * 1000.0)
    print(
        f"micro-nudge measured +delta on {axis_names[axis]}: {moved_mm:.2f} mm "
        f"({n_out} steps)"
    )

    goal_back = float(start_xyz[axis])
    print(f"micro-nudge: return to start on {axis_names[axis]}")
    obs, n_back = _step_axis_toward(
        env,
        obs,
        axis=axis,
        goal_value=goal_back,
        max_step=max_step,
        tolerance=tolerance,
        max_steps=max_steps,
        settle_s=settle_s,
        label="nudge-return",
    )
    _print_tcp("after return", obs)

    end_xyz = obs["state"]["tcp_pose"][:3]
    drift_mm = float(np.linalg.norm(end_xyz - start_xyz) * 1000.0)
    print(
        f"micro-nudge return used {n_back} steps; drift vs pre-nudge: {drift_mm:.2f} mm"
    )
    if abs(moved_mm) < delta * 1000.0 * 0.7:
        print(
            "WARNING: nudge displacement < 70% of requested; "
            "increase --nudge-max-steps or --nudge-step-size"
        )
    return obs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Step 5 safe FrankyFrankaEnv smoke")
    parser.add_argument(
        "--connect-only",
        action="store_true",
        help="Only probe and print current TCP pose; do not create env",
    )
    parser.add_argument(
        "--num-steps",
        type=int,
        default=3,
        help="Number of zero-action steps (default: 3)",
    )
    parser.add_argument(
        "--unsafe-full-reset",
        action="store_true",
        help="Disable safe_smoke_hold and move to probed pose on init/reset",
    )
    parser.add_argument(
        "--micro-nudge",
        action="store_true",
        help="After zero steps, multi-step nudge by --nudge-delta then return",
    )
    parser.add_argument(
        "--nudge-delta",
        type=float,
        default=0.005,
        help="Total TCP displacement goal in metres (default: 0.005 = 5 mm; max 0.02)",
    )
    parser.add_argument(
        "--nudge-axis",
        type=int,
        choices=(0, 1, 2),
        default=0,
        help="Action axis for translation delta: 0=x, 1=y, 2=z (default: 0)",
    )
    parser.add_argument(
        "--safety-margin",
        type=float,
        default=0.05,
        help="Half-width (m) of auto ee_pose_limit box around probed TCP xyz (default: 0.05)",
    )
    parser.add_argument(
        "--rpy-margin",
        type=float,
        default=0.35,
        help="Half-width (rad) of auto ee_pose_limit box around probed euler (default: 0.35)",
    )
    parser.add_argument(
        "--nudge-step-size",
        type=float,
        default=0.005,
        help="Max |action[axis]| per step during nudge (default: 0.005 = 5 mm)",
    )
    parser.add_argument(
        "--nudge-tolerance",
        type=float,
        default=0.001,
        help="Stop when |goal - tcp[axis]| below this (default: 0.001 = 1 mm)",
    )
    parser.add_argument(
        "--nudge-max-steps",
        type=int,
        default=40,
        help="Max env.step calls per nudge leg (out / return)",
    )
    parser.add_argument(
        "--nudge-settle-s",
        type=float,
        default=0.1,
        help="Seconds to wait after each nudge step for impedance to settle (default: 0.1)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    os.environ["RLINF_SKIP_CAMERA"] = "1"
    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    safe_hold = not args.unsafe_full_reset

    print(f"Step5: robot={robot_ip} safe_hold={safe_hold} micro_nudge={args.micro_nudge}")
    rest_pose = probe_tcp_pose_euler(robot_ip)
    print(
        "probed rest_pose (xyz m, euler xyz rad):",
        [f"{v:.4f}" for v in rest_pose],
    )

    if args.connect_only:
        print("connect-only OK (no env created)")
        return 0

    xyz_margin = max(args.safety_margin, args.nudge_delta * 4.0)
    lim_min, lim_max = _ee_pose_limits_from_probe(
        rest_pose, xyz_margin=xyz_margin, rpy_margin=args.rpy_margin
    )
    print(
        f"auto ee_pose_limit xyz margin={xyz_margin:.3f}m "
        f"rpy margin={args.rpy_margin:.3f}rad"
    )
    print("ee_pose_limit_min:", [f"{v:.4f}" for v in lim_min])
    print("ee_pose_limit_max:", [f"{v:.4f}" for v in lim_max])

    if args.micro_nudge:
        if args.nudge_delta <= 0 or args.nudge_delta > 0.02:
            raise ValueError("--nudge-delta must be in (0, 0.02] metres")
        if args.nudge_step_size <= 0 or args.nudge_step_size > 0.02:
            raise ValueError("--nudge-step-size must be in (0, 0.02] metres")
        if args.nudge_settle_s < 0 or args.nudge_settle_s > 2.0:
            raise ValueError("--nudge-settle-s must be in [0, 2.0] seconds")

    # Subprocess probe should have released FCI; brief pause before Ray env.
    time.sleep(1.0)

    if not ray.is_initialized():
        ray.init(log_to_driver=False, logging_level="ERROR")

    print("creating FrankyFrankaEnv-v1 ...")
    env = _make_env(
        robot_ip,
        rest_pose,
        safe_hold=safe_hold,
        xyz_margin=xyz_margin,
        rpy_margin=args.rpy_margin,
    )

    obs, _ = env.reset()
    print("reset OK")
    _print_tcp("after init/reset", obs)

    zero = _zero_action(env)
    for i in range(args.num_steps):
        obs, reward, term, trunc, info = env.step(zero)
        _print_tcp(f"zero step {i + 1}/{args.num_steps}", obs)
        time.sleep(0.1)

    if args.micro_nudge:
        obs = _run_micro_nudge(
            env,
            obs,
            axis=args.nudge_axis,
            delta=args.nudge_delta,
            max_step=args.nudge_step_size,
            tolerance=args.nudge_tolerance,
            max_steps=args.nudge_max_steps,
            settle_s=args.nudge_settle_s,
        )

    _close_env(env)
    print("Step5 PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
