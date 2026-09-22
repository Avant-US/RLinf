#!/usr/bin/env python3
"""Phase 2 real-robot smoke for FrankyCubePlaceEnv-v1 (no camera, no SAC).

``target_ee_pose`` comes from the H1 YAML (human ``getpos_euler``), **not**
from the current TCP. Current TCP is only printed for comparison.

Sub-steps (container, Desk FCI + unlock, cube grasped, interactive
``test_franky_controller_ext.py`` already ``q``-exited):

  ``--connect-only`` — probe TCP, print target / hover / box; no env
  ``--reset-only`` — gym.make + reset (closed-gripper lift to hover)
  default — reset + zero steps + optional ``--approach-steps`` toward mark

Safety (see dmo_place_2LOG.md LOG-019 for why each of these exists):
  - a **hard start-pose gate** before Ray starts: the TCP must be above the
    contact point, inside the xy box, and the gripper must be holding the cube
  - ``safe_smoke_hold=True`` skips __init__ interpolate
  - ``enable_random_reset=False`` so the rest pose stays above the mark
  - ``reset_z_lift_m`` defaults to 0.03 (enough to break contact with a flat
    mark; PegInsertion's 0.10 exists to pull a plug out of a socket)
  - interpolated moves are speed-capped, and the controller fences the measured
    TCP against ``ee_pose_limit`` + margin, aborting on overshoot or loss of
    tracking
  - rest hover is ``target + clip_z_range_high`` (not the relative lift)
  - ``no_gripper=True`` → GripperCloseEnv (6D, cannot open)
  - ``RLINF_SKIP_CAMERA=1`` by default (``--with-camera`` flips this to ``0``
    and wires real camera serials -- see dmo_place_2.md S3.3's "cube-place
    link self-check")
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
import yaml

# Four ".." because __file__ is a file, so the first one only strips the filename:
# b/x/scripts/step_cube_place_robot.py -> b/x/scripts -> b/x -> b -> repo root.
# Three resolved to ``<repo>/b``, which put the default pose file at
# ``<repo>/b/b/x/configs/...`` -- masked only because every documented entry point
# exports REPO_PATH first (LOG-023 finding 4).
REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))
sys.path.insert(0, os.path.dirname(__file__))

import franky_ext.tasks.register  # noqa: F401

from franky_ext.motion_limits import (
    clip_shortfall,
    describe_authority,
    reach_report,
    worst_reach_corner,
)
from franky_ext.tcp_probe import (
    check_start_pose,
    describe_robot_mode,
    effective_ee_pose_limits,
    probe_robot_state,
    require_motion_ready,
)
from rlinf.scheduler import FrankaHWInfo
from rlinf.scheduler.hardware.robots.franka import FrankaConfig, FrankaRobot
from step8_checks import is_placeholder_serial

DEFAULT_POSE_FILE = os.path.join(
    REPO, "b", "x", "configs", "cube_place_target_ee_pose.yaml"
)
DEFAULT_CAMERA_JSON = os.path.join(REPO, "b", "x", "configs", "camera_detected.json")
#: Placeholder used when --with-camera is not given; RLINF_SKIP_CAMERA=1
#: means nothing ever tries to open a device with this serial.
_NO_CAMERA_SERIAL = "000000000000"


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


def _build_hardware(
    robot_ip: str,
    *,
    camera_serials: list[str] | None = None,
    camera_type: str = "realsense",
) -> FrankaHWInfo:
    franka_config = FrankaConfig(
        node_rank=0,
        robot_ip=robot_ip,
        camera_serials=list(camera_serials) if camera_serials else [_NO_CAMERA_SERIAL],
        camera_type=camera_type,
        gripper_type="franka",
        disable_validate=True,
    )
    return FrankaHWInfo(
        type=FrankaRobot.HW_TYPE,
        model=FrankaRobot.HW_TYPE,
        config=franka_config,
    )


def _load_camera_serials(path: str) -> tuple[str, list[str]]:
    """Read Step 8a's ``camera_detected.json`` (camera_type, [serials])."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    camera_type = str(data.get("camera_type") or "realsense")
    serials = [str(s) for s in data.get("camera_serials") or []]
    return camera_type, serials


def _resolve_camera_serials(args: argparse.Namespace) -> tuple[str, list[str]]:
    """Resolve (camera_type, serials) for --with-camera from CLI or Step 8a JSON."""
    if args.camera_serials:
        serials = [s.strip() for s in args.camera_serials if str(s).strip()]
        camera_type = args.camera_type or "realsense"
    else:
        if not os.path.isfile(args.camera_json):
            raise RuntimeError(
                f"--with-camera given but {args.camera_json} does not exist; run "
                "step8_detect_cameras.py first, or pass --camera-serials."
            )
        json_type, serials = _load_camera_serials(args.camera_json)
        camera_type = args.camera_type or json_type
    if not serials:
        raise RuntimeError("--with-camera given but no camera serials resolved")
    bad = [s for s in serials if is_placeholder_serial(s)]
    if bad:
        raise RuntimeError(f"placeholder camera serials not allowed: {bad}")
    return camera_type, serials


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
    """Hover pose and the box the env will really enforce.

    Uses ``effective_ee_pose_limits`` rather than a generic probe-centred box:
    ``PegInsertionConfig.__post_init__`` overwrites ``ee_pose_limit_*`` from
    ``target_ee_pose`` + the clip ranges, and its roll/pitch half-width is a
    hardcoded 0.01 rad, not ``clip_rz``. Printing the generic box told the operator
    an orientation window 35x wider than the enforced one -- a reassuring number
    presented as pre-flight verification.
    """
    reset = list(target)
    reset[2] = float(target[2]) + float(z_high)
    lim_min, lim_max = effective_ee_pose_limits(
        target,
        clip_xy=clip_xy,
        z_low=z_low,
        z_high=z_high,
        clip_rz=clip_rz,
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
    reset_z_lift: float,
    waypoint_clear_error: bool,
    camera_serials: list[str] | None = None,
    camera_type: str = "realsense",
) -> dict[str, object]:
    reset, lim_min, lim_max = _hover_and_box(
        target, z_high=z_high, z_low=z_low, clip_xy=clip_xy, clip_rz=clip_rz
    )
    serials = list(camera_serials) if camera_serials else [_NO_CAMERA_SERIAL]
    camera_cfg: dict[str, object] = {
        "camera_serials": serials,
        "camera_type": camera_type,
    }
    if camera_serials:
        # Default naming (_build_camera_infos) is already "wrist_{i}" for a
        # single camera, but naming it explicitly matches the YAML wiring in
        # dmo_place_2.md S3.1 and keeps this script's obs['frames'] key
        # ("wrist_1") independent of enumeration order.
        camera_cfg["camera_names"] = {
            serial: f"wrist_{i}" for i, serial in enumerate(serials, start=1)
        }
    return {
        "is_dummy": False,
        "robot_ip": robot_ip,
        "gripper_type": "franka",
        **camera_cfg,
        "target_ee_pose": target,
        "reset_ee_pose": reset,
        "ee_pose_limit_min": lim_min,
        "ee_pose_limit_max": lim_max,
        "enable_camera_player": False,
        "enable_random_reset": False,
        "safe_smoke_hold": safe_hold,
        # See env/realworld_cube_place.yaml: defaults to False upstream, and there
        # is no other reward code in this task -- without it every printed reward
        # is 0.0000, indistinguishable from a broken path (round-2 audit finding 8).
        "use_dense_reward": True,
        "clip_x_range": clip_xy,
        "clip_y_range": clip_xy,
        "clip_z_range_low": z_low,
        "clip_z_range_high": z_high,
        "clip_rz_range": clip_rz,
        "random_xy_range": 0.0,
        "add_gripper_penalty": False,
        "reset_z_lift_m": reset_z_lift,
        "clear_error_per_waypoint": waypoint_clear_error,
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


def _print_frame_stats(obs: dict) -> None:
    """--with-camera: print shape/dtype/min/max/mean per camera frame.

    Deliberately not an assertion (unlike step8_test_env_camera.py's
    ``_check_frame``): a max()==0 frame here is diagnostic evidence for the
    operator ("camera stubbed or capture failed"), not a reason to blow up a
    script that also just moved the arm and is mid-teardown-sensitive flow.
    """
    frames = obs.get("frames")
    if not isinstance(frames, dict) or not frames:
        print(
            "camera check: obs['frames'] missing or empty "
            f"(keys={list(obs.keys())}); RLINF_SKIP_CAMERA or camera_serials wrong?"
        )
        return
    for name, frame in frames.items():
        arr = np.asarray(frame)
        flag = "" if int(arr.max()) > 0 else "  ALL-ZERO (stub or capture failure)"
        print(
            f"camera frame {name}: shape={tuple(arr.shape)} dtype={arr.dtype} "
            f"min={int(arr.min())} max={int(arr.max())} mean={float(arr.mean()):.2f}"
            f"{flag}"
        )


def _close_env(env: gym.Env) -> None:
    """Best-effort teardown. Must never mask the original failure.

    Also reports the controller's final health, because that is the one moment
    where "was the fence still armed / did the watchdog trip" is cheap to record
    and expensive to reconstruct later.

    Catches broadly on purpose: this runs from a ``finally``, and a secondary
    exception here (the known ``VideoPlayer.stop`` AttributeError, a Ray actor
    already gone, a dead FCI connection) would otherwise replace the real reason
    the run aborted -- which is exactly the kind of lost evidence that cost
    LOG-011..017 seven rounds.

    ``FrankySingleFrankaEnvMixin.close`` is what actually stops the impedance
    tracker; without it a clean ``reset-only PASS`` would leave the tracker
    commanding torque until the process exited.
    """
    try:
        inner = env.unwrapped
        controller = getattr(inner, "_controller", None)
        if controller is not None and not inner.config.is_dummy:
            print("controller health at teardown:", controller.motion_health().wait()[0])
    except Exception as exc:  # noqa: BLE001 - diagnostic only
        print(f"motion_health at teardown unavailable: {type(exc).__name__}: {exc}")
    try:
        env.close()
    except Exception as exc:  # noqa: BLE001 - see docstring
        print(f"env.close() warning (ignored): {type(exc).__name__}: {exc}")


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
    # --- motion authority (LOG-019). All of these end up as env vars read by
    # franky_ext.motion_limits, including inside the controller's Ray actor.
    parser.add_argument(
        "--reset-z-lift",
        type=float,
        default=0.03,
        help="go_to_rest lift above the CURRENT TCP (m); CubePlace default 0.03",
    )
    parser.add_argument(
        "--interp-speed",
        type=float,
        default=None,
        help="speed cap for interpolated reset moves (m/s); default 0.02",
    )
    parser.add_argument(
        "--force-ceiling",
        type=float,
        default=None,
        help="commanded cartesian PER-AXIS force ceiling (N); default 20, and "
        "also bounded by the norm ceiling / sqrt(3) = 23.1N. error_clip is "
        "derived as ceiling/stiffness",
    )
    parser.add_argument(
        "--force-norm-ceiling",
        type=float,
        default=None,
        help="worst-case (3-axis) commanded force ceiling (N); default 40. If "
        "--force-ceiling / sqrt(3) exceeds this, THIS is the one that actually "
        "binds -- raise it too if --force-ceiling alone is not taking effect",
    )
    parser.add_argument(
        "--torque-norm-ceiling",
        type=float,
        default=None,
        help="worst-case (3-axis) commanded torque ceiling (N.m); default 12 = the "
        "j5-j7 joint limit. clip_shortfall names this flag when the norm ceiling "
        "caps a per-axis torque request",
    )
    parser.add_argument(
        "--guard-margin",
        type=float,
        default=None,
        help="how far the MEASURED TCP may leave ee_pose_limit before abort (m); "
        "default 0.05",
    )
    parser.add_argument(
        "--guard-max-lag",
        type=float,
        default=None,
        help="abort when |measured - commanded| exceeds this (m); default 0.05. "
        "The most likely source of a nuisance abort, so it gets a flag",
    )
    parser.add_argument(
        "--cube-width",
        type=float,
        default=None,
        help="calibrated grasped-cube width (m); default 0.046. The gripper's "
        "'am I holding the cube' window is centred on this",
    )
    parser.add_argument(
        "--min-start-clearance",
        type=float,
        default=0.005,
        help="required start height above the H1 contact point (m); "
        "0 means 'just not below contact'",
    )
    parser.add_argument(
        "--allow-start-outside-box",
        action="store_true",
        help="downgrade the start-pose gate to a warning. Only with a hand on the "
        "e-stop and a reason; this gate exists because LOG-019 started below the "
        "contact point",
    )
    parser.add_argument(
        "--no-waypoint-clear-error",
        action="store_true",
        help="skip clear_errors() between interpolation waypoints (LOG-019 R5 "
        "experiment; default keeps upstream behaviour)",
    )
    parser.add_argument(
        "--skip-gripper-check",
        action="store_true",
        help="allow reset/box with an empty gripper (normally refused: a "
        "closed-gripper task would grasp air)",
    )
    parser.add_argument(
        "--with-camera",
        action="store_true",
        help="set RLINF_SKIP_CAMERA=0 and wire real camera serials into the "
        "override, so this becomes the cube-place link's own camera "
        "self-check (dmo_place_2.md S3.3), instead of Step 8c's "
        "FrankyFrankaEnv-v1. Prints obs['frames'] shape/dtype/min/max/mean "
        "after reset",
    )
    parser.add_argument(
        "--camera-serials",
        nargs="+",
        default=None,
        help="--with-camera: override camera serials (skips --camera-json)",
    )
    parser.add_argument(
        "--camera-type",
        default=None,
        help="--with-camera: override camera_type (default: from --camera-json, "
        "else realsense)",
    )
    parser.add_argument(
        "--camera-json",
        default=DEFAULT_CAMERA_JSON,
        help=f"--with-camera: Step 8a JSON (default: {DEFAULT_CAMERA_JSON})",
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
    # The controller prints this too, but only from set_motion_guard -- i.e. after
    # gym.make, with the arm about to move. `connect` exists to catch bad geometry
    # before that, and near-singular corners are exactly the kind of bad geometry a
    # human should see while there is still time to move the mark (LOG-023).
    _, corner = worst_reach_corner(lim_min[:3], lim_max[:3])
    print(f"reach: worst box corner {_fmt(corner)} {reach_report(corner)}")


def main() -> int:
    args = parse_args()
    camera_type = "realsense"
    camera_serials: list[str] | None = None
    if args.with_camera:
        camera_type, camera_serials = _resolve_camera_serials(args)
        os.environ["RLINF_SKIP_CAMERA"] = "0"
    else:
        os.environ["RLINF_SKIP_CAMERA"] = "1"
    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    safe_hold = not args.unsafe_full_reset

    if args.reset_z_high <= 0 or args.reset_z_high > 0.15:
        raise ValueError("--reset-z-high must be in (0, 0.15] metres")
    if args.clip_z_low < 0 or args.clip_z_low > 0.03:
        raise ValueError("--clip-z-low must be in [0, 0.03] metres")
    if args.clip_xy <= 0 or args.clip_xy > 0.15:
        raise ValueError("--clip-xy must be in (0, 0.15] metres")
    if args.reset_z_lift <= 0 or args.reset_z_lift > 0.12:
        raise ValueError("--reset-z-lift must be in (0, 0.12] metres")
    # --clip-rz was the one geometry argument with no range check. A negative value
    # inverts the rpy half-widths in PegInsertionConfig.__post_init__, and
    # np.clip(x, lower, upper) with lower > upper silently returns `upper`, so every
    # commanded orientation would be pinned to a constant wrong offset.
    if not 0.0 < args.clip_rz <= 0.6:
        raise ValueError("--clip-rz must be in (0, 0.6] radians")

    # Export the authority knobs before the cluster is built. A Ray worker process
    # is forked from the raylet, so it does NOT inherit the driver's environment --
    # but RLinf forwards it explicitly: ``NodeGroup`` diffs the driver's os.environ
    # against the env captured at ``ray start`` and passes the difference as the
    # actor's ``runtime_env["env_vars"]`` (``scheduler/cluster/node.py``), so these
    # do reach the controller actor without restarting Ray. motion_limits reads (and
    # clamps) them at call time, and the tracker start log echoes the resulting
    # products, so "did it take effect?" is answerable from the log, not by trust.
    if args.interp_speed is not None:
        os.environ["RLINF_CUBE_INTERP_SPEED"] = repr(float(args.interp_speed))
    if args.force_ceiling is not None:
        os.environ["RLINF_CUBE_FORCE_CEILING_N"] = repr(float(args.force_ceiling))
    if args.force_norm_ceiling is not None:
        os.environ["RLINF_CUBE_FORCE_NORM_CEILING_N"] = repr(float(args.force_norm_ceiling))
    if args.torque_norm_ceiling is not None:
        os.environ["RLINF_CUBE_TORQUE_NORM_CEILING_NM"] = repr(
            float(args.torque_norm_ceiling)
        )
    if args.guard_margin is not None:
        os.environ["RLINF_CUBE_GUARD_MARGIN"] = repr(float(args.guard_margin))
    if args.guard_max_lag is not None:
        os.environ["RLINF_CUBE_GUARD_MAX_LAG"] = repr(float(args.guard_max_lag))
    if args.cube_width is not None:
        os.environ["FRANKA_CUBE_WIDTH_M"] = repr(float(args.cube_width))

    print(
        f"cube-place phase2: robot={robot_ip} safe_hold={safe_hold} "
        f"z_high={args.reset_z_high:.3f}m clip_xy={args.clip_xy:.3f}m "
        f"z_lift={args.reset_z_lift:.3f}m pose_file={args.pose_file} "
        f"with_camera={args.with_camera}"
        + (f" camera_serials={camera_serials}" if camera_serials else "")
    )
    print(f"authority: {describe_authority(2000.0, 150.0)}  (K from compliance_param)")
    for msg in clip_shortfall(2000.0, 150.0):
        print(f"WARNING: {msg}")

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
    print(
        f"gripper: holding={robot_state.get('gripper_holding')} "
        f"width={robot_state.get('gripper_width')} "
        f"cmd_success_rate={robot_state.get('cmd_success_rate')} "
        f"|dq|={robot_state.get('joint_vel_norm')}"
    )
    if robot_state.get("gripper_error"):
        print(f"gripper probe warning: {robot_state['gripper_error']}")
    raw = robot_state.get("is_grasped_raw")
    decided = robot_state.get("gripper_holding")
    if raw is not None and raw != decided:
        print(
            f"gripper note: libfranka is_grasped={raw} disagrees with the "
            f"width-based holding={decided}; width-based is what gates on"
        )

    start_problems = check_start_pose(
        probed,
        target,
        clip_xy=args.clip_xy,
        z_low=args.clip_z_low,
        z_high=args.reset_z_high,
        min_clearance_m=args.min_start_clearance,
        clip_rz=args.clip_rz,
    )

    if args.connect_only:
        if not pose_ready:
            print("connect-only incomplete: H1 pose not calibrated")
            return 1
        if mode != "RobotMode.Idle":
            print(f"connect-only note: {describe_robot_mode(mode)}")
        for problem in start_problems:
            print(f"connect-only start-pose note: {problem}")
        print("connect-only OK (no env created)")
        return 0

    # reset/box command the arm. Everything below fails BEFORE the ~30 s of Ray
    # startup, so a bad setup costs seconds instead of a robot session.

    # 1. Mode. LOG-017: UserStopped reads state and drives the gripper normally
    #    but rejects every motion, so reset looks like it ran with dz=0.0000.
    require_motion_ready(mode)

    # 2. Start pose. LOG-019 began 6.6 mm below the contact point with the cube
    #    pressed into the mark, and nothing checked it.
    if start_problems:
        for problem in start_problems:
            print(f"START-POSE PROBLEM: {problem}")
        if not args.allow_start_outside_box:
            raise RuntimeError(
                "refusing to reset from this pose ("
                + str(len(start_problems))
                + " problem(s) above). Guide the arm with the enabling device to a "
                "few cm above the mark, re-run `connect` to confirm, then retry. "
                "Override with --allow-start-outside-box only with a hand on the "
                "e-stop."
            )
        print("--allow-start-outside-box given: continuing despite the above")

    # 3. Cube present. A closed-gripper task with an empty hand would grasp air,
    #    and the hover/reward geometry is calibrated for a cube of known height.
    holding = robot_state.get("gripper_holding")
    if holding is False and not args.skip_gripper_check:
        raise RuntimeError(
            f"gripper is not holding anything (width={robot_state.get('gripper_width')}). "
            "This task keeps the gripper closed on a cube the whole time. Re-grip "
            "with `python b/x/scripts/test_franky_controller_ext.py` (open -> insert "
            "-> close), or pass --skip-gripper-check if you really mean to run empty."
        )
    if holding is None:
        print("gripper state unknown; cannot verify the cube is held")

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
        reset_z_lift=args.reset_z_lift,
        waypoint_clear_error=not args.no_waypoint_clear_error,
        camera_serials=camera_serials,
        camera_type=camera_type,
    )

    print("creating FrankyCubePlaceEnv-v1 ...")
    # gym.make itself launches the controller actor and (unless safe_smoke_hold)
    # performs a real interpolate, so it can fail with the actor already alive.
    # Nothing would clean that up, so say so loudly rather than exiting silently.
    try:
        env = gym.make(
            "FrankyCubePlaceEnv-v1",
            override_cfg=override_cfg,
            worker_info=None,
            hardware_info=_build_hardware(
                robot_ip, camera_serials=camera_serials, camera_type=camera_type
            ),
            env_idx=0,
            env_cfg=_safe_env_cfg(),
        )
    except Exception as exc:
        print(
            f"gym.make FAILED ({type(exc).__name__}: {exc}). If a controller actor "
            "was already launched it is not cleaned up by this path: run `ray stop` "
            "and check `ss -tn ... :1337` is empty before retrying."
        )
        raise
    # Single teardown path. Previously every ``raise`` had to remember its own
    # ``_close_env`` and the happy paths had their own; the one case that was
    # missed -- ``env.reset()`` itself raising, which is exactly what LOG-019 did
    # -- left the Ray actor and its 1 kHz torque motion to be cleaned up by
    # process death.
    try:
        return _run_env(env, args, target)
    finally:
        _close_env(env)


def _run_env(env: gym.Env, args: argparse.Namespace, target: list[float]) -> int:
    """Wrapper checks, reset, hover gate, then the optional box steps."""
    names = _wrapper_names(env)
    print("wrapper stack:", " -> ".join(names))
    if "GripperCloseEnv" not in names:
        raise RuntimeError("expected GripperCloseEnv in wrapper stack (6D closed)")
    if int(np.prod(env.action_space.shape)) != 6:
        raise RuntimeError(f"expected action_dim 6, got {env.action_space.shape}")

    # The box the env ACTUALLY enforces, read back from the constructed config
    # rather than from the candidate we passed in -- PegInsertionConfig.__post_init__
    # recomputes both limits and its roll/pitch half-width is a hardcoded 0.01 rad.
    cfg = env.unwrapped.config
    print("effective ee_pose_limit_min:", _fmt(np.asarray(cfg.ee_pose_limit_min)))
    print("effective ee_pose_limit_max:", _fmt(np.asarray(cfg.ee_pose_limit_max)))
    print(
        "effective reset_ee_pose:",
        _fmt(np.asarray(cfg.reset_ee_pose)),
        f" reset_z_lift_m={getattr(cfg, 'reset_z_lift_m', None)}",
    )

    obs, _ = env.reset()
    print("reset OK")
    if args.with_camera:
        _print_frame_stats(obs)
    time.sleep(0.5)
    inner = env.unwrapped
    inner._franka_state = inner._controller.get_state().wait()[0]
    fresh_xyz = np.asarray(inner._franka_state.tcp_pose[:3], dtype=np.float64)
    _print_tcp("after reset (wrapper obs)", obs)
    print(f"after settle live xyz={_fmt(fresh_xyz)}")
    try:
        print("controller health:", inner._controller.motion_health().wait()[0])
    except Exception as exc:  # noqa: BLE001 - diagnostic only
        print(f"motion_health unavailable: {type(exc).__name__}: {exc}")

    target_np = np.asarray(target, dtype=np.float64)
    xy_err = float(np.linalg.norm(fresh_xyz[:2] - target_np[:2]))
    z_err = abs(float(fresh_xyz[2] - (target_np[2] + args.reset_z_high)))
    print(f"hover check: |xy-target|={xy_err:.4f}m |z-hover|={z_err:.4f}m")
    if xy_err > args.xy_tol:
        raise RuntimeError(
            f"after reset, |xy-target|={xy_err:.4f} > tol {args.xy_tol:.4f}; "
            "H1 pose or clip box may be wrong — abort before stepping"
        )
    if z_err > args.z_tol:
        raise RuntimeError(
            f"after reset, |z-hover|={z_err:.4f} > tol {args.z_tol:.4f}; "
            f"arm did not go to mark+{args.reset_z_high}m"
        )

    state = getattr(inner, "_franka_state", None)
    if state is None:
        raise RuntimeError("no _franka_state after reset; cannot confirm gripper")
    opened = bool(state.gripper_open)
    print(f"gripper_open after reset: {opened}")
    if opened:
        raise RuntimeError("gripper opened during reset; V1 forbids open")

    if args.reset_only:
        print("reset-only PASS")
        return 0

    zero = np.zeros(env.action_space.shape, dtype=np.float32)
    for i in range(args.num_steps):
        obs, reward, term, trunc, info = env.step(zero)
        print(f"zero step {i + 1}/{args.num_steps}: reward={float(reward):.4f}")
        _print_tcp("  tcp", obs)
        if inner._franka_state.gripper_open:
            raise RuntimeError("gripper opened on a zero step")
        time.sleep(0.1)

    if args.approach_steps > 0:
        down = np.zeros(env.action_space.shape, dtype=np.float32)
        # Full scale, because action_scale[0] is now clamped to the step-speed cap
        # (see CubePlaceConfig.__post_init__). The old -0.4 was tuned against the
        # unclamped 0.02 m/step, so it advertised a magnitude 4x the cap this file
        # documents; at the clamped scale it would barely move.
        down[2] = -1.0
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
                raise RuntimeError(
                    f"z={tcp[2]:.4f} dropped below box floor {z_floor:.4f}"
                )
            if inner._franka_state.gripper_open:
                raise RuntimeError("gripper opened on an approach step")
            time.sleep(0.15)
        z_after = float(_tcp_from_obs(obs)[2])
        print(f"approach dz={z_after - z_before:.4f}m (expect slightly negative or ~0 if clipped)")

    print("box-steps PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
