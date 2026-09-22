#!/usr/bin/env python3
"""Step 6: safe real-robot smoke for FrankyPegInsertionEnv-v1 (no camera).

Sub-steps:
  6a  ``--connect-only`` — probe TCP (same as Step 5a), print target/reset poses
  6b  ``--no-random-reset`` — reset + zero steps + reward (no xy/rz random)
  6c  default — same as 6b but ``enable_random_reset=True`` (±5 cm xy, ±30° rz)

Safety defaults (vs upstream PegInsertion):
  - ``target_ee_pose`` = probed TCP; ``reset_ee_pose`` = target + 5 cm z (via config)
  - ``reset_z_lift_m`` = 5 cm pre-lift in ``go_to_rest`` (was 10 cm upstream)
  - ``safety_box_half_width_m`` = 5 cm for ``ee_pose_limit`` and random xy reset
  - ``safe_smoke_hold=True`` skips ``__init__`` interpolate (arm holds at connect)

Usage (container, after Desk FCI + unlock):
  source b/x/configs/setup_before_ray_5090.sh
  export FRANKA_ROBOT_IP=172.16.0.2
  export RLINF_SKIP_CAMERA=1
  ray start --head --port=6379
  python b/x/scripts/step6_test_peg_env_robot.py --connect-only   # 6a
  python b/x/scripts/step6_test_peg_env_robot.py --no-random-reset  # 6b
  python b/x/scripts/step6_test_peg_env_robot.py   # 6c
  ray stop
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import gymnasium as gym
import numpy as np
import ray

REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

import franky_ext.tasks.register  # noqa: F401

from franky_ext.tcp_probe import (
    ee_pose_limits_from_probe,
    peg_target_and_reset_from_probe,
    probe_tcp_pose_euler,
)
from rlinf.scheduler import FrankaHWInfo
from rlinf.scheduler.hardware.robots.franka import FrankaConfig, FrankaRobot


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


def _build_override_cfg(
    robot_ip: str,
    probed_pose: list[float],
    *,
    safe_hold: bool,
    z_offset_m: float,
    safety_half_width_m: float,
    enable_random_reset: bool,
    rpy_margin: float,
) -> dict[str, object]:
    target_pose, reset_pose = peg_target_and_reset_from_probe(
        probed_pose, z_offset_m=z_offset_m
    )
    lim_min, lim_max = ee_pose_limits_from_probe(
        target_pose,
        xyz_margin=safety_half_width_m,
        rpy_margin=rpy_margin,
    )
    return {
        "is_dummy": False,
        "robot_ip": robot_ip,
        "camera_serials": ["000000000000"],
        "gripper_type": "franka",
        "target_ee_pose": target_pose,
        "reset_ee_pose": reset_pose,
        "ee_pose_limit_min": lim_min,
        "ee_pose_limit_max": lim_max,
        "enable_camera_player": False,
        "enable_random_reset": enable_random_reset,
        "safe_smoke_hold": safe_hold,
        "reset_z_lift_m": z_offset_m,
        "safety_box_half_width_m": safety_half_width_m,
        "clip_x_range": safety_half_width_m,
        "clip_y_range": safety_half_width_m,
        "clip_z_range_high": z_offset_m,
        "random_xy_range": safety_half_width_m,
        "add_gripper_penalty": False,
    }


def _print_pose_block(
    probed_pose: list[float],
    target_pose: list[float],
    reset_pose: list[float],
) -> None:
    print("probed tcp (xyz m, euler xyz rad):", [f"{v:.4f}" for v in probed_pose])
    print("target_ee_pose:", [f"{v:.4f}" for v in target_pose])
    print("reset_ee_pose (target + z):", [f"{v:.4f}" for v in reset_pose])


def _print_tcp(label: str, obs: dict) -> None:
    tcp = obs["state"]["tcp_pose"]
    print(f"{label}: xyz={tcp[:3]} euler={tcp[3:6]}")


def _close_env(env: gym.Env) -> None:
    try:
        env.close()
    except AttributeError as exc:
        print(f"env.close() warning (ignored for smoke): {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 6 safe FrankyPegInsertionEnv smoke"
    )
    parser.add_argument(
        "--connect-only",
        action="store_true",
        help="6a: probe TCP and print target/reset; do not create env",
    )
    parser.add_argument(
        "--num-steps",
        type=int,
        default=3,
        help="Number of zero-action steps after reset (default: 3)",
    )
    parser.add_argument(
        "--unsafe-full-reset",
        action="store_true",
        help="Disable safe_smoke_hold (init may interpolate to reset pose)",
    )
    parser.add_argument(
        "--no-random-reset",
        action="store_true",
        help="Disable random xy/rz perturbation on go_to_rest",
    )
    parser.add_argument(
        "--reset-z-offset",
        type=float,
        default=0.05,
        help="Hover height above target for reset_ee_pose and go_to_rest lift (m)",
    )
    parser.add_argument(
        "--safety-half-width",
        type=float,
        default=0.05,
        help="Half-width (m) of xyz safety box and random xy reset (default: 0.05)",
    )
    parser.add_argument(
        "--rpy-margin",
        type=float,
        default=0.35,
        help="Half-width (rad) of ee_pose_limit euler box (default: 0.35)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    os.environ["RLINF_SKIP_CAMERA"] = "1"
    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    safe_hold = not args.unsafe_full_reset

    if args.reset_z_offset <= 0 or args.reset_z_offset > 0.15:
        raise ValueError("--reset-z-offset must be in (0, 0.15] metres")
    if args.safety_half_width <= 0 or args.safety_half_width > 0.15:
        raise ValueError("--safety-half-width must be in (0, 0.15] metres")

    print(
        f"Step6: robot={robot_ip} safe_hold={safe_hold} "
        f"z_offset={args.reset_z_offset:.3f}m safety_half={args.safety_half_width:.3f}m"
    )

    probed_pose = probe_tcp_pose_euler(robot_ip)
    target_pose, reset_pose = peg_target_and_reset_from_probe(
        probed_pose, z_offset_m=args.reset_z_offset
    )
    _print_pose_block(probed_pose, target_pose, reset_pose)

    if args.connect_only:
        print("Step6a connect-only OK (no env created)")
        return 0

    lim_min, lim_max = ee_pose_limits_from_probe(
        target_pose,
        xyz_margin=args.safety_half_width,
        rpy_margin=args.rpy_margin,
    )
    print(
        f"ee_pose_limit xyz half-width={args.safety_half_width:.3f}m "
        f"rpy margin={args.rpy_margin:.3f}rad "
        f"random_reset={not args.no_random_reset}"
    )
    print("ee_pose_limit_min:", [f"{v:.4f}" for v in lim_min])
    print("ee_pose_limit_max:", [f"{v:.4f}" for v in lim_max])

    time.sleep(1.0)

    if not ray.is_initialized():
        ray.init(log_to_driver=False, logging_level="ERROR")

    override_cfg = _build_override_cfg(
        robot_ip,
        probed_pose,
        safe_hold=safe_hold,
        z_offset_m=args.reset_z_offset,
        safety_half_width_m=args.safety_half_width,
        enable_random_reset=not args.no_random_reset,
        rpy_margin=args.rpy_margin,
    )

    print("creating FrankyPegInsertionEnv-v1 ...")
    env = gym.make(
        "FrankyPegInsertionEnv-v1",
        override_cfg=override_cfg,
        worker_info=None,
        hardware_info=_build_hardware(robot_ip),
        env_idx=0,
        env_cfg=_safe_env_cfg(),
    )

    obs, _ = env.reset()
    print("reset OK")
    _print_tcp("after reset", obs)

    zero = np.zeros(env.action_space.shape, dtype=np.float32)
    for i in range(args.num_steps):
        obs, reward, term, trunc, info = env.step(zero)
        print(f"zero step {i + 1}/{args.num_steps}: reward={float(reward):.4f}")
        _print_tcp(f"  tcp", obs)
        time.sleep(0.1)

    _close_env(env)
    print("Step6 PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
