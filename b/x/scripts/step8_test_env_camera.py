#!/usr/bin/env python3
"""Step 8c: real-robot env smoke with cameras enabled (arm holds pose).

Opens FrankyFrankaEnv-v1 with RLINF_SKIP_CAMERA=0 and serials from Step 8a.
Checks obs['frames'] shape/dtype, rejects all-zero stub frames, optional JPEG.

Usage (franky container, after Desk FCI + 8a):
  source b/x/configs/setup_before_ray_5090.sh
  export FRANKA_ROBOT_IP=172.16.0.2
  python b/x/scripts/step8_detect_cameras.py
  ray start --head --port=6379
  python b/x/scripts/step8_test_env_camera.py
  python b/x/scripts/step8_test_env_camera.py --save-jpeg
  ray stop
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import gymnasium as gym
import numpy as np
import ray

REPO = os.environ.get(
    "REPO_PATH",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")),
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))
sys.path.insert(0, os.path.dirname(__file__))

import franky_ext.tasks.register  # noqa: F401

from franky_ext.tcp_probe import ee_pose_limits_from_probe, probe_tcp_pose_euler
from rlinf.scheduler import FrankaHWInfo
from rlinf.scheduler.hardware.robots.franka import FrankaConfig, FrankaRobot
from step8_checks import check, is_placeholder_serial, result

DEFAULT_JSON = os.path.join(REPO, "b", "x", "configs", "camera_detected.json")


def _load_detected(path: str) -> tuple[str, list[str]]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    camera_type = str(data.get("camera_type") or "realsense")
    serials = [str(s) for s in data.get("camera_serials") or []]
    return camera_type, serials


def _validate_serials(serials: list[str]) -> list[str]:
    cleaned = [s.strip() for s in serials if str(s).strip()]
    if not cleaned:
        raise ValueError("no camera serials; run step8_detect_cameras.py first")
    bad = [s for s in cleaned if is_placeholder_serial(s)]
    if bad:
        raise ValueError(f"placeholder camera serials not allowed: {bad}")
    return cleaned


def _build_hardware(
    robot_ip: str, serials: list[str], camera_type: str
) -> FrankaHWInfo:
    franka_config = FrankaConfig(
        node_rank=0,
        robot_ip=robot_ip,
        camera_serials=list(serials),
        camera_type=camera_type,
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
    serials: list[str],
    camera_type: str,
    *,
    safe_hold: bool,
    xyz_margin: float,
    rpy_margin: float,
) -> gym.Env:
    lim_min, lim_max = ee_pose_limits_from_probe(
        rest_pose, xyz_margin=xyz_margin, rpy_margin=rpy_margin
    )
    camera_names = {serial: f"wrist_{i}" for i, serial in enumerate(serials, start=1)}
    return gym.make(
        "FrankyFrankaEnv-v1",
        override_cfg={
            "is_dummy": False,
            "robot_ip": robot_ip,
            "camera_serials": list(serials),
            "camera_type": camera_type,
            "camera_names": camera_names,
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
        hardware_info=_build_hardware(robot_ip, serials, camera_type),
        env_idx=0,
        env_cfg=_safe_env_cfg(),
    )


def _frames_from_obs(obs: dict) -> dict[str, np.ndarray]:
    frames = obs.get("frames")
    if not isinstance(frames, dict) or not frames:
        raise AssertionError(
            f"obs['frames'] missing or empty; keys={list(obs.keys())}. "
            "If images are zeros, check RLINF_SKIP_CAMERA and camera_serials."
        )
    return frames


def _check_frame(name: str, frame: np.ndarray, expected_shape: tuple[int, ...]) -> None:
    if frame.dtype != np.uint8:
        raise AssertionError(f"{name}: dtype={frame.dtype}, expected uint8")
    if tuple(frame.shape) != tuple(expected_shape):
        raise AssertionError(
            f"{name}: shape={frame.shape}, expected {expected_shape}"
        )
    if int(frame.max()) == 0:
        raise AssertionError(
            f"{name}: all-zero frame (skip_camera stub or failed capture)"
        )


def _save_jpeg(path: Path, frame: np.ndarray) -> None:
    import cv2

    path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(path), bgr):
        raise RuntimeError(f"failed to write JPEG {path}")


def _close_env(env: gym.Env) -> None:
    try:
        env.close()
    except AttributeError as exc:
        print(f"env.close() warning (ignored for smoke): {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 8c: FrankyFrankaEnv-v1 camera interaction"
    )
    parser.add_argument(
        "--from-json",
        default=DEFAULT_JSON,
        help=f"Step 8a JSON (default: {DEFAULT_JSON})",
    )
    parser.add_argument(
        "--serials",
        nargs="+",
        default=None,
        help="override camera serials (skips JSON serials)",
    )
    parser.add_argument(
        "--camera-type",
        default=None,
        help="override camera_type (realsense / zed / lumos)",
    )
    parser.add_argument("--num-steps", type=int, default=3)
    parser.add_argument(
        "--unsafe-full-reset",
        action="store_true",
        help="disable safe_smoke_hold (arm may interpolate)",
    )
    parser.add_argument("--safety-margin", type=float, default=0.05)
    parser.add_argument("--rpy-margin", type=float, default=0.35)
    parser.add_argument(
        "--save-jpeg",
        action="store_true",
        help="write RGB frames under --save-dir",
    )
    parser.add_argument(
        "--save-dir",
        default=os.path.join(REPO, "b", "x", "logs", "step8_camera"),
        help="JPEG output directory",
    )
    parser.add_argument(
        "--require-live",
        action="store_true",
        help="FAIL if frames do not change across zero-steps",
    )
    return parser.parse_args()


def _run(args: argparse.Namespace) -> int:
    os.environ["RLINF_SKIP_CAMERA"] = "0"
    skip = os.environ.get("RLINF_SKIP_CAMERA", "")
    if not check("skip_camera_is_0", skip in ("0", "false", "no"), skip):
        return result("8c", False, reason="RLINF_SKIP_CAMERA must be 0")

    if args.serials:
        serials = _validate_serials(args.serials)
        camera_type = args.camera_type or "realsense"
        check("serials_from_cli", True, str(serials))
    else:
        json_ok = os.path.isfile(args.from_json)
        check("json_exists", json_ok, args.from_json)
        if not json_ok:
            return result("8c", False, reason="run 8a first or pass --serials")
        json_type, json_serials = _load_detected(args.from_json)
        serials = _validate_serials(json_serials)
        camera_type = args.camera_type or json_type or "realsense"

    ph = [s for s in serials if is_placeholder_serial(s)]
    if not check("serials_not_placeholder", not ph, str(serials)):
        return result("8c", False, camera_serials=serials)

    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    safe_hold = not args.unsafe_full_reset
    check("safe_smoke_hold", safe_hold, "use --unsafe-full-reset to interpolate")
    print(
        f"Step8c: robot={robot_ip} camera_type={camera_type} "
        f"serials={serials} skip_camera={skip} safe_hold={safe_hold}"
    )

    rest_pose = probe_tcp_pose_euler(robot_ip)
    print("probed rest_pose:", [f"{v:.4f}" for v in rest_pose])
    check("tcp_probe", len(rest_pose) == 6, str(rest_pose))
    time.sleep(1.0)

    if not ray.is_initialized():
        ray.init(log_to_driver=False, logging_level="ERROR")

    env = _make_env(
        robot_ip,
        rest_pose,
        serials,
        camera_type,
        safe_hold=safe_hold,
        xyz_margin=args.safety_margin,
        rpy_margin=args.rpy_margin,
    )
    obs, _ = env.reset()
    check("env_reset", True)

    frame_space = env.observation_space["frames"]
    inner_spaces = getattr(frame_space, "spaces", frame_space)
    frames = _frames_from_obs(obs)
    keys = list(frames.keys())
    if not check("obs_frames_nonempty", bool(keys), str(keys)):
        _close_env(env)
        return result("8c", False, reason="empty frames (skip_camera stub?)")
    if not check("wrist_1_present", "wrist_1" in frames, str(keys)):
        _close_env(env)
        return result("8c", False, frame_keys=keys)

    prev = {}
    frame_stats = {}
    for name, frame in frames.items():
        expected = inner_spaces[name].shape
        try:
            _check_frame(name, frame, expected)
            shape_ok, nonzero = True, True
        except AssertionError as exc:
            shape_ok, nonzero = False, False
            check(f"frame_{name}", False, str(exc))
            _close_env(env)
            return result("8c", False, reason=str(exc))
        stats = (
            f"shape={tuple(frame.shape)} dtype={frame.dtype} "
            f"min={int(frame.min())} max={int(frame.max())} "
            f"mean={float(frame.mean()):.2f}"
        )
        check(f"frame_{name}_uint8_128", shape_ok, stats)
        check(f"frame_{name}_nonzero", nonzero, stats)
        print("reset frame:", name, stats)
        frame_stats[name] = stats
        prev[name] = frame.copy()
        if args.save_jpeg:
            jpeg_path = Path(args.save_dir) / f"{name}_reset.jpg"
            _save_jpeg(jpeg_path, frame)
            check(f"jpeg_{name}", jpeg_path.is_file(), str(jpeg_path))

    zero = np.zeros(env.action_space.shape, dtype=np.float32)
    saw_change = False
    for i in range(args.num_steps):
        obs, reward, term, trunc, info = env.step(zero)
        frames = _frames_from_obs(obs)
        for name, frame in frames.items():
            _check_frame(name, frame, inner_spaces[name].shape)
            delta = float(np.abs(frame.astype(np.int16) - prev[name]).mean())
            print(
                f"zero step {i + 1}/{args.num_steps} {name}: "
                f"mean={float(frame.mean()):.2f} abs_delta={delta:.3f}"
            )
            if delta > 0.0:
                saw_change = True
            prev[name] = frame.copy()
            if args.save_jpeg:
                _save_jpeg(
                    Path(args.save_dir) / f"{name}_step{i + 1}.jpg",
                    frame,
                )
        time.sleep(0.1)

    _close_env(env)
    if saw_change:
        check("live_frames_changed", True, "abs_delta>0 across steps")
    elif args.require_live:
        check("live_frames_changed", False, "static; --require-live set")
        return result("8c", False, reason="static frames", frame_stats=frame_stats)
    else:
        print("CHECK live_frames_changed SKIP  static scene OK (use --require-live to fail)")

    return result(
        "8c",
        True,
        camera_type=camera_type,
        camera_serials=serials,
        frame_keys=keys,
        live_frames=saw_change,
        safe_hold=safe_hold,
    )


def main() -> int:
    args = parse_args()
    try:
        return _run(args)
    except Exception as exc:
        check("uncaught", False, f"{type(exc).__name__}: {exc}")
        return result("8c", False, reason=str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
