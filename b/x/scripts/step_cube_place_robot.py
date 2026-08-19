#!/usr/bin/env python3
"""Phase 2 real-robot smoke for FrankyCubePlaceEnv-v1 (no camera, no SAC).

``target_ee_pose`` comes from the H1 YAML (human ``getpos_euler``), **not**
from the current TCP. Current TCP is only printed for comparison.

Sub-steps (container, Desk FCI + unlock, cube grasped, interactive
``test_franky_controller_ext.py`` already ``q``-exited):

  ``--connect-only`` — probe TCP, print target / hover / box; no env
  ``--reset-only`` — gym.make + reset (closed-gripper lift to hover)
  default — reset + zero steps + optional ``--approach-steps`` toward mark

Safety:
  - ``safe_smoke_hold=True`` skips __init__ interpolate
  - ``enable_random_reset=False`` so the rest pose stays above the mark
  - ``reset_z_lift_m`` defaults to 0.10 (PegInsertion: lift from *current* TCP)
  - rest hover is ``target + clip_z_range_high`` (not the relative lift)
  - ``no_gripper=True`` → GripperCloseEnv (6D, cannot open)
  - ``RLINF_SKIP_CAMERA=1``
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import gymnasium as gym
import numpy as np
import ray
import yaml

REPO = os.environ.get("REPO_PATH", os.path.abspath(os.path.join(__file__, "../../..")))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

import franky_ext.tasks.register  # noqa: F401

from franky_ext.tcp_probe import (
    describe_robot_mode,
    ee_pose_limits_from_probe,
    probe_robot_state,
    require_motion_ready,
)
from rlinf.scheduler import FrankaHWInfo
from rlinf.scheduler.hardware.robots.franka import FrankaConfig, FrankaRobot

DEFAULT_POSE_FILE = os.path.join(
    REPO, "b", "x", "configs", "cube_place_target_ee_pose.yaml"
)


def _wrapper_names(env: gym.Env) -> list[str]:
    names = [type(env).__name__]
    inner = env
    while hasattr(inner, "env"):
        inner = inner.env
        names.append(type(inner).__name__)
    return names


def load_target_pose(path: str, *, require_calibrated: bool) -> list[float]:
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    pose = data.get("target_ee_pose")
    calibrated = bool(data.get("calibrated", False))
    if not isinstance(pose, (list, tuple)) or len(pose) != 6:
        raise ValueError(f"{path}: target_ee_pose must be 6 floats, got {pose!r}")
    pose_f = [float(v) for v in pose]
    if require_calibrated and not calibrated:
        raise RuntimeError(
            f"{path}: calibrated is false. Finish H1 and run "
            "write_cube_place_pose.py (or set calibrated: true)."
        )
    if require_calibrated and all(abs(v) < 1e-8 for v in pose_f):
        raise RuntimeError(f"{path}: target_ee_pose is all zeros; not a valid H1 pose")
    return pose_f


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


def _hover_and_box(
    target: list[float],
    *,
    z_high: float,
    z_low: float,
    clip_xy: float,
    clip_rz: float,
) -> tuple[list[float], list[float], list[float]]:
    reset = list(target)
    reset[2] = float(target[2]) + float(z_high)
    lim_min, lim_max = ee_pose_limits_from_probe(
        target,
        xyz_margin=clip_xy,
        rpy_margin=clip_rz,
    )
    lim_min[2] = float(target[2]) - float(z_low)
    lim_max[2] = float(target[2]) + float(z_high)
    return reset, lim_min, lim_max


def _build_override_cfg(
    robot_ip: str,
    target: list[float],
    *,
    safe_hold: bool,
    z_high: float,
    z_low: float,
    clip_xy: float,
    clip_rz: float,
) -> dict[str, object]:
    reset, lim_min, lim_max = _hover_and_box(
        target, z_high=z_high, z_low=z_low, clip_xy=clip_xy, clip_rz=clip_rz
    )
    return {
        "is_dummy": False,
        "robot_ip": robot_ip,
        "camera_serials": ["000000000000"],
        "gripper_type": "franka",
        "target_ee_pose": target,
        "reset_ee_pose": reset,
        "ee_pose_limit_min": lim_min,
        "ee_pose_limit_max": lim_max,
        "enable_camera_player": False,
        "enable_random_reset": False,
        "safe_smoke_hold": safe_hold,
        "clip_x_range": clip_xy,
        "clip_y_range": clip_xy,
        "clip_z_range_low": z_low,
        "clip_z_range_high": z_high,
        "clip_rz_range": clip_rz,
        "random_xy_range": 0.0,
        "add_gripper_penalty": False,
    }


def _fmt(pose: list[float] | np.ndarray) -> str:
    return "[" + ", ".join(f"{float(v):.4f}" for v in pose) + "]"


def _tcp_from_obs(obs: dict) -> np.ndarray:
    return np.asarray(obs["state"]["tcp_pose"][:6], dtype=np.float64)


def _gripper_open_from_obs(obs: dict) -> bool | None:
    state = obs.get("state") or {}
    if "gripper_open" in state:
        return bool(np.asarray(state["gripper_open"]).reshape(-1)[0])
    return None


def _print_tcp(label: str, obs: dict) -> None:
    tcp = _tcp_from_obs(obs)
    grip = obs["state"].get("gripper_position")
    opened = _gripper_open_from_obs(obs)
    extra = ""
    if grip is not None:
        extra += f" gripper_position={np.asarray(grip).tolist()}"
    if opened is not None:
        extra += f" gripper_open={opened}"
    print(f"{label}: xyz={_fmt(tcp[:3])} euler={_fmt(tcp[3:6])}{extra}")


def _close_env(env: gym.Env) -> None:
    try:
        env.close()
    except AttributeError as exc:
        print(f"env.close() warning (ignored for smoke): {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Phase 2 FrankyCubePlaceEnv-v1 real-robot smoke"
    )
    parser.add_argument(
        "--pose-file",
        default=DEFAULT_POSE_FILE,
        help="H1 YAML written by write_cube_place_pose.py",
    )
    parser.add_argument(
        "--connect-only",
        action="store_true",
        help="2c: probe TCP and print geometry; do not create env",
    )
    parser.add_argument(
        "--reset-only",
        action="store_true",
        help="2d: gym.make + reset, then close (no step)",
    )
    parser.add_argument(
        "--num-steps",
        type=int,
        default=3,
        help="2e: zero-action steps after reset (default: 3)",
    )
    parser.add_argument(
        "--approach-steps",
        type=int,
        default=0,
        help="2e: extra negative-z steps toward the mark (default: 0)",
    )
    parser.add_argument(
        "--unsafe-full-reset",
        action="store_true",
        help="Disable safe_smoke_hold (init may interpolate)",
    )
    parser.add_argument(
        "--reset-z-high",
        type=float,
        default=0.08,
        help="Hover height above H1 contact TCP (m); default CubePlace 0.08",
    )
    parser.add_argument(
        "--clip-z-low",
        type=float,
        default=0.005,
        help="Allowed drop below H1 contact TCP (m); default 0.005",
    )
    parser.add_argument(
        "--clip-xy",
        type=float,
        default=0.05,
        help="xy half-width of clip / safety box (m); default 0.05",
    )
    parser.add_argument(
        "--clip-rz",
        type=float,
        default=0.35,
        help="yaw half-width (rad); default 0.35",
    )
    parser.add_argument(
        "--xy-tol",
        type=float,
        default=0.03,
        help="2d/2e: max |xy - target_xy| after reset (m)",
    )
    parser.add_argument(
        "--z-tol",
        type=float,
        default=0.025,
        help="2d/2e: max |z - (target_z + hover)| after reset (m)",
    )
    return parser.parse_args()


def _print_geometry(
    probed: list[float],
    target: list[float],
    reset: list[float],
    lim_min: list[float],
    lim_max: list[float],
) -> None:
    delta = np.asarray(probed[:3]) - np.asarray(target[:3])
    print("probed tcp (current):", _fmt(probed))
    print("target_ee_pose (H1): ", _fmt(target))
    print("reset_ee_pose hover:", _fmt(reset))
    print("probed - target xyz (m):", _fmt(delta))
    print("ee_pose_limit_min:", _fmt(lim_min))
    print("ee_pose_limit_max:", _fmt(lim_max))


def main() -> int:
    args = parse_args()
    os.environ["RLINF_SKIP_CAMERA"] = "1"
    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    safe_hold = not args.unsafe_full_reset

    if args.reset_z_high <= 0 or args.reset_z_high > 0.15:
        raise ValueError("--reset-z-high must be in (0, 0.15] metres")
    if args.clip_z_low < 0 or args.clip_z_low > 0.03:
        raise ValueError("--clip-z-low must be in [0, 0.03] metres")
    if args.clip_xy <= 0 or args.clip_xy > 0.15:
        raise ValueError("--clip-xy must be in (0, 0.15] metres")

    print(
        f"cube-place phase2: robot={robot_ip} safe_hold={safe_hold} "
        f"z_high={args.reset_z_high:.3f}m clip_xy={args.clip_xy:.3f}m "
        f"pose_file={args.pose_file}"
    )

    pose_ready = True
    try:
        target = load_target_pose(args.pose_file, require_calibrated=True)
    except (OSError, ValueError, RuntimeError) as exc:
        pose_ready = False
        if not args.connect_only:
            raise
        print(f"connect warning: {exc}")
        print("connect will still probe TCP; reset/box stay blocked until H1 is written.")
        try:
            target = load_target_pose(args.pose_file, require_calibrated=False)
        except (OSError, ValueError):
            target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    reset, lim_min, lim_max = _hover_and_box(
        target,
        z_high=args.reset_z_high,
        z_low=args.clip_z_low,
        clip_xy=args.clip_xy,
        clip_rz=args.clip_rz,
    )

    robot_state = probe_robot_state(robot_ip)
    probed = robot_state["pose"]
    mode = robot_state["robot_mode"]
    _print_geometry(probed, target, reset, lim_min, lim_max)
    print(f"robot_mode: {mode} has_errors: {robot_state['has_errors']}")

    if args.connect_only:
        if not pose_ready:
            print("connect-only incomplete: H1 pose not calibrated")
            return 1
        if mode != "RobotMode.Idle":
            print(f"connect-only note: {describe_robot_mode(mode)}")
        print("connect-only OK (no env created)")
        return 0

    # reset/box command the arm: fail here rather than after ~30 s of Ray
    # startup followed by a silent dz=0.0000.
    require_motion_ready(mode)

    time.sleep(1.0)
    if not ray.is_initialized():
        ray.init(log_to_driver=False, logging_level="ERROR")

    override_cfg = _build_override_cfg(
        robot_ip,
        target,
        safe_hold=safe_hold,
        z_high=args.reset_z_high,
        z_low=args.clip_z_low,
        clip_xy=args.clip_xy,
        clip_rz=args.clip_rz,
    )

    print("creating FrankyCubePlaceEnv-v1 ...")
    env = gym.make(
        "FrankyCubePlaceEnv-v1",
        override_cfg=override_cfg,
        worker_info=None,
        hardware_info=_build_hardware(robot_ip),
        env_idx=0,
        env_cfg=_safe_env_cfg(),
    )
    names = _wrapper_names(env)
    print("wrapper stack:", " -> ".join(names))
    if "GripperCloseEnv" not in names:
        _close_env(env)
        raise RuntimeError("expected GripperCloseEnv in wrapper stack (6D closed)")
    if int(np.prod(env.action_space.shape)) != 6:
        _close_env(env)
        raise RuntimeError(f"expected action_dim 6, got {env.action_space.shape}")

    obs, _ = env.reset()
    print("reset OK")
    time.sleep(0.5)
    inner = env.unwrapped
    inner._franka_state = inner._controller.get_state().wait()[0]
    fresh_xyz = np.asarray(inner._franka_state.tcp_pose[:3], dtype=np.float64)
    _print_tcp("after reset (wrapper obs)", obs)
    print(f"after settle live xyz={_fmt(fresh_xyz)}")

    target_np = np.asarray(target, dtype=np.float64)
    xy_err = float(np.linalg.norm(fresh_xyz[:2] - target_np[:2]))
    z_err = abs(float(fresh_xyz[2] - (target_np[2] + args.reset_z_high)))
    print(f"hover check: |xy-target|={xy_err:.4f}m |z-hover|={z_err:.4f}m")
    if xy_err > args.xy_tol:
        _close_env(env)
        raise RuntimeError(
            f"after reset, |xy-target|={xy_err:.4f} > tol {args.xy_tol:.4f}; "
            "H1 pose or clip box may be wrong — abort before stepping"
        )
    if z_err > args.z_tol:
        _close_env(env)
        raise RuntimeError(
            f"after reset, |z-hover|={z_err:.4f} > tol {args.z_tol:.4f}; "
            f"arm did not go to mark+{args.reset_z_high}m"
        )

    state = getattr(inner, "_franka_state", None)
    if state is None:
        _close_env(env)
        raise RuntimeError("no _franka_state after reset; cannot confirm gripper")
    opened = bool(state.gripper_open)
    print(f"gripper_open after reset: {opened}")
    if opened:
        _close_env(env)
        raise RuntimeError("gripper opened during reset; V1 forbids open")

    if args.reset_only:
        _close_env(env)
        print("reset-only PASS")
        return 0

    zero = np.zeros(env.action_space.shape, dtype=np.float32)
    for i in range(args.num_steps):
        obs, reward, term, trunc, info = env.step(zero)
        print(f"zero step {i + 1}/{args.num_steps}: reward={float(reward):.4f}")
        _print_tcp("  tcp", obs)
        if inner._franka_state.gripper_open:
            _close_env(env)
            raise RuntimeError("gripper opened on a zero step")
        time.sleep(0.1)

    if args.approach_steps > 0:
        down = np.zeros(env.action_space.shape, dtype=np.float32)
        down[2] = -0.4
        z_before = float(_tcp_from_obs(obs)[2])
        for i in range(args.approach_steps):
            obs, reward, term, trunc, info = env.step(down)
            tcp = _tcp_from_obs(obs)
            print(
                f"approach step {i + 1}/{args.approach_steps}: "
                f"reward={float(reward):.4f} z={tcp[2]:.4f}"
            )
            _print_tcp("  tcp", obs)
            z_floor = float(target_np[2] - args.clip_z_low - 0.01)
            if float(tcp[2]) < z_floor:
                _close_env(env)
                raise RuntimeError(
                    f"z={tcp[2]:.4f} dropped below box floor {z_floor:.4f}"
                )
            if inner._franka_state.gripper_open:
                _close_env(env)
                raise RuntimeError("gripper opened on an approach step")
            time.sleep(0.15)
        z_after = float(_tcp_from_obs(obs)[2])
        print(f"approach dz={z_after - z_before:.4f}m (expect slightly negative or ~0 if clipped)")

    _close_env(env)
    print("box-steps PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
