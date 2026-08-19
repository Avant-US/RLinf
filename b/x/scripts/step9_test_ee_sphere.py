#!/usr/bin/env python3
"""Step 9: random EE motion inside a 5 cm ball around the current TCP.

Current TCP is the origin. xyz wanders in a sphere of radius 0.05 m for ~10 s
(100 steps at 10 Hz). Orientation is locked; gripper does not move. Five
wrist_1 JPEGs are written under b/x/logs/step9_camera/.

Safety:
  - safe_smoke_hold (no interpolate to a far rest pose)
  - per-step L2 |Δxyz| <= 5 mm
  - script-level projection into the 5 cm ball (env only clips an AABB)
  - abort if ||p - origin|| > 5.8 cm, then return to origin

Usage (franky container, after Desk FCI + Step 8a):
  source b/x/configs/setup_before_ray_5090.sh
  python b/x/scripts/step9_test_ee_sphere.py --math-only
  export FRANKA_ROBOT_IP=172.16.0.2
  export RLINF_SKIP_CAMERA=0
  ray start --head --port=6379
  python b/x/scripts/step9_test_ee_sphere.py
  ray stop
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = os.environ.get(
    "REPO_PATH",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")),
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))
sys.path.insert(0, os.path.dirname(__file__))

from step8_checks import check, is_placeholder_serial, result

DEFAULT_JSON = os.path.join(REPO, "b", "x", "configs", "camera_detected.json")
DEFAULT_SAVE_DIR = os.path.join(REPO, "b", "x", "logs", "step9_camera")

SPHERE_RADIUS_M = 0.05
SPHERE_FAIL_RADIUS_M = 0.058
MAX_STEP_M = 0.005
MOTION_DURATION_S = 10.0
STEP_FREQUENCY_HZ = 10.0
N_MOTION_STEPS = int(MOTION_DURATION_S * STEP_FREQUENCY_HZ)
MOTION_TIMEOUT_S = 12.0
WAYPOINT_PERIOD_STEPS = 20
RETURN_TOL_M = 0.008
RETURN_MAX_STEPS = 20
RETURN_STEP_M = 0.05
# Impedance SSE is ~10 mm (K_t=500 N/m). env.step always sets the next
# Cartesian target from *measured* TCP, so commanding exactly the remaining
# error snaps the setpoint back to origin and the sag never closes. Command
# 2x remaining (mirror through origin) and hold that setpoint while sleeping.
RETURN_GAIN = 2.0
RETURN_SETTLE_S = 0.4
RPY_MAX_DELTA_RAD = 0.15
# AABB must strictly contain sphere + impedance overshoot, else return
# commands clip to the box face while measured TCP stays ~5 cm out.
EE_LIMIT_MARGIN_M = 0.08
DEFAULT_SEED = 9
PHOTO_STEPS = (0, 25, 50, 75, 100)
PHOTO_NAMES = {
    0: "wrist_1_t00s.jpg",
    25: "wrist_1_t02s.jpg",
    50: "wrist_1_t05s.jpg",
    75: "wrist_1_t07s.jpg",
    100: "wrist_1_t10s.jpg",
}


def sample_in_ball(
    rng: np.random.Generator, radius: float, origin: np.ndarray | None = None
) -> np.ndarray:
    """Uniform sample inside a ball of *radius* (optionally around *origin*)."""
    vec = rng.normal(size=3)
    norm = float(np.linalg.norm(vec))
    if norm < 1e-12:
        vec = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        norm = 1.0
    direction = vec / norm
    u = float(rng.random())
    r = radius * (u ** (1.0 / 3.0))
    point = direction * r
    if origin is not None:
        point = np.asarray(origin, dtype=np.float64) + point
    return point.astype(np.float64)


def project_to_ball(
    point: np.ndarray, origin: np.ndarray, radius: float
) -> np.ndarray:
    """Project *point* onto the closed ball around *origin*."""
    origin = np.asarray(origin, dtype=np.float64)
    point = np.asarray(point, dtype=np.float64)
    delta = point - origin
    norm = float(np.linalg.norm(delta))
    if norm <= radius or norm < 1e-12:
        return point.copy()
    return origin + delta * (radius / norm)


def clipped_delta(
    current: np.ndarray,
    target: np.ndarray,
    origin: np.ndarray,
    radius: float,
    max_step: float,
) -> np.ndarray:
    """L2-capped xyz delta; target is projected into the ball first.

    If *current* is already outside the ball (impedance overshoot), do not
    snap onto the sphere in one step — that produced >max_step chords and
    left return-to-origin clipped against the env AABB face.
    """
    current = np.asarray(current, dtype=np.float64)
    origin = np.asarray(origin, dtype=np.float64)
    target = project_to_ball(target, origin, radius)
    delta = target - current
    dist = float(np.linalg.norm(delta))
    if dist > max_step and dist > 1e-12:
        delta = delta * (max_step / dist)
    nxt = current + delta
    off = nxt - origin
    nrm = float(np.linalg.norm(off))
    if nrm > radius and nrm > 1e-12:
        nxt = origin + off * (radius / nrm)
        delta = nxt - current
        dist = float(np.linalg.norm(delta))
        if dist > max_step and dist > 1e-12:
            delta = delta * (max_step / dist)
    return delta.astype(np.float64)


def wrap_rpy_delta(current_rpy: np.ndarray, origin_rpy: np.ndarray) -> np.ndarray:
    """Smallest-angle RPY difference, each axis in (-pi, pi]."""
    d = np.asarray(current_rpy, dtype=np.float64) - np.asarray(
        origin_rpy, dtype=np.float64
    )
    return (d + np.pi) % (2.0 * np.pi) - np.pi


def _run_math_tests() -> int:
    rng = np.random.default_rng(DEFAULT_SEED)
    origin = np.array([0.4, 0.0, 0.35], dtype=np.float64)
    samples = np.stack(
        [sample_in_ball(rng, SPHERE_RADIUS_M, origin) for _ in range(2000)]
    )
    radii = np.linalg.norm(samples - origin, axis=1)
    in_ball = bool(np.all(radii <= SPHERE_RADIUS_M + 1e-12))
    mean_r = float(radii.mean())
    sample_ok = in_ball and 0.5 * SPHERE_RADIUS_M < mean_r < 0.9 * SPHERE_RADIUS_M
    check(
        "math_sample_in_ball",
        sample_ok,
        f"max={float(radii.max()):.4f} mean={mean_r:.4f} R={SPHERE_RADIUS_M}",
    )

    outside = origin + np.array([0.1, 0.0, 0.0])
    projected = project_to_ball(outside, origin, SPHERE_RADIUS_M)
    proj_ok = abs(float(np.linalg.norm(projected - origin)) - SPHERE_RADIUS_M) < 1e-9
    inside = origin + np.array([0.01, 0.0, 0.0])
    proj_inside = project_to_ball(inside, origin, SPHERE_RADIUS_M)
    proj_ok = proj_ok and bool(np.allclose(proj_inside, inside))
    check(
        "math_project_to_ball",
        proj_ok,
        f"proj={projected.tolist()}",
    )

    current = origin.copy()
    target = origin + np.array([0.04, 0.0, 0.0])
    delta = clipped_delta(current, target, origin, SPHERE_RADIUS_M, MAX_STEP_M)
    step_ok = abs(float(np.linalg.norm(delta)) - MAX_STEP_M) < 1e-9
    far = origin + np.array([0.2, 0.2, 0.2])
    edge = origin + np.array([SPHERE_RADIUS_M, 0.0, 0.0])
    out_delta = clipped_delta(edge, far, origin, SPHERE_RADIUS_M, MAX_STEP_M)
    nxt = edge + out_delta
    stay = float(np.linalg.norm(nxt - origin)) <= SPHERE_RADIUS_M + 1e-9
    clip_ok = (
        step_ok
        and stay
        and float(np.linalg.norm(out_delta)) <= MAX_STEP_M + 1e-9
    )
    check(
        "math_clipped_delta",
        clip_ok,
        f"first={delta.tolist()} edge_delta={out_delta.tolist()}",
    )

    rpy_a = np.array([np.pi - 0.1, 0.0, 0.0])
    rpy_b = np.array([-np.pi + 0.1, 0.0, 0.0])
    rpy_ok = bool(
        np.allclose(wrap_rpy_delta(rpy_a, rpy_b), np.array([-0.2, 0.0, 0.0]), atol=1e-9)
    )
    check("math_rpy_wrap", rpy_ok)

    names_ok = set(PHOTO_NAMES) == set(PHOTO_STEPS) and len(PHOTO_STEPS) == 5
    check("math_photo_slots", names_ok, str(PHOTO_NAMES))

    all_ok = sample_ok and proj_ok and clip_ok and rpy_ok and names_ok
    return result("9math", all_ok)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Step 9: 5 cm EE sphere wander + 5 wrist_1 photos"
    )
    parser.add_argument(
        "--math-only",
        action="store_true",
        help="run sphere-geometry unit checks only (no robot, no camera)",
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
    parser.add_argument(
        "--unsafe-full-reset",
        action="store_true",
        help="rejected: Step 9 requires safe_smoke_hold",
    )
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--radius", type=float, default=SPHERE_RADIUS_M)
    parser.add_argument("--max-step", type=float, default=MAX_STEP_M)
    parser.add_argument(
        "--save-dir",
        default=DEFAULT_SAVE_DIR,
        help="JPEG output directory",
    )
    parser.add_argument("--safety-margin", type=float, default=EE_LIMIT_MARGIN_M)
    parser.add_argument("--rpy-margin", type=float, default=RPY_MAX_DELTA_RAD)
    return parser.parse_args()


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


def _tcp_xyz(obs: dict) -> np.ndarray:
    return np.asarray(obs["state"]["tcp_pose"][:3], dtype=np.float64)


def _tcp_rpy(obs: dict) -> np.ndarray:
    """Return xyz-euler RPY from obs.

    ``Quat2EulerWrapper`` (always applied by ``apply_single_arm_wrappers``)
    converts the env's 7-D xyz+quat into 6-D xyz+euler. Raw unwrapped obs
    may still be 7-D; both shapes are accepted.
    """
    pose = np.asarray(obs["state"]["tcp_pose"], dtype=np.float64).reshape(-1)
    if pose.size >= 7:
        return _rpy_from_quat(pose[3:7])
    if pose.size >= 6:
        return pose[3:6]
    raise AssertionError(f"tcp_pose dim={pose.shape} too small for RPY")


def _frames_from_obs(obs: dict) -> dict[str, np.ndarray]:
    frames = obs.get("frames")
    if not isinstance(frames, dict) or not frames:
        raise AssertionError(
            f"obs['frames'] missing or empty; keys={list(obs.keys())}"
        )
    return frames


def _save_jpeg(path: Path, frame: np.ndarray) -> None:
    import cv2

    path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(path), bgr):
        raise RuntimeError(f"failed to write JPEG {path}")


def _clear_step9_jpegs(save_dir: Path) -> None:
    save_dir.mkdir(parents=True, exist_ok=True)
    for path in save_dir.glob("*.jpg"):
        path.unlink()


def _zero_action(env) -> np.ndarray:
    return np.zeros(env.action_space.shape, dtype=np.float32)


def _close_env(env) -> None:
    try:
        env.close()
    except AttributeError as exc:
        print(f"env.close() warning (ignored): {exc}")


def _rpy_from_quat(quat: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation as R

    return R.from_quat(quat).as_euler("xyz")


def _run_robot(args: argparse.Namespace) -> int:
    import gymnasium as gym
    import ray

    import franky_ext.tasks.register  # noqa: F401
    from franky_ext.tcp_probe import ee_pose_limits_from_probe, probe_tcp_pose_euler
    from rlinf.scheduler import FrankaHWInfo
    from rlinf.scheduler.hardware.robots.franka import FrankaConfig, FrankaRobot

    radius = float(args.radius)
    max_step = float(args.max_step)
    fail_radius = radius + (SPHERE_FAIL_RADIUS_M - SPHERE_RADIUS_M)
    if radius <= 0 or radius > 0.05 + 1e-9:
        return result(
            "9", False, reason=f"radius must be in (0, 0.05], got {radius}"
        )
    if max_step <= 0 or max_step > 0.02:
        return result(
            "9", False, reason=f"max-step must be in (0, 0.02], got {max_step}"
        )

    os.environ["RLINF_SKIP_CAMERA"] = "0"
    skip = os.environ.get("RLINF_SKIP_CAMERA", "")
    if not check("skip_camera_is_0", skip in ("0", "false", "no"), skip):
        return result("9", False, reason="RLINF_SKIP_CAMERA must be 0")

    if args.unsafe_full_reset:
        check("safe_smoke_hold", False, "--unsafe-full-reset is forbidden in Step 9")
        return result("9", False, reason="unsafe-full-reset rejected")
    check("safe_smoke_hold", True, "hold current TCP; no interpolate")

    if args.serials:
        serials = _validate_serials(args.serials)
        camera_type = args.camera_type or "realsense"
        check("serials_from_cli", True, str(serials))
    else:
        json_ok = os.path.isfile(args.from_json)
        check("json_exists", json_ok, args.from_json)
        if not json_ok:
            return result("9", False, reason="run Step 8a first or pass --serials")
        json_type, json_serials = _load_detected(args.from_json)
        serials = _validate_serials(json_serials)
        camera_type = args.camera_type or json_type or "realsense"

    ph = [s for s in serials if is_placeholder_serial(s)]
    if not check("serials_not_placeholder", not ph, str(serials)):
        return result("9", False, camera_serials=serials)

    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    print(
        f"Step9: robot={robot_ip} camera_type={camera_type} serials={serials} "
        f"radius={radius} max_step={max_step} seed={args.seed}"
    )

    rest_pose = probe_tcp_pose_euler(robot_ip)
    origin_xyz = np.asarray(rest_pose[:3], dtype=np.float64)
    origin_rpy = np.asarray(rest_pose[3:6], dtype=np.float64)
    print("probed origin:", [f"{v:.4f}" for v in rest_pose])
    check("tcp_probe", len(rest_pose) == 6, str(rest_pose))
    time.sleep(1.0)

    lim_min, lim_max = ee_pose_limits_from_probe(
        rest_pose, xyz_margin=args.safety_margin, rpy_margin=args.rpy_margin
    )
    camera_names = {serial: f"wrist_{i}" for i, serial in enumerate(serials, start=1)}
    franka_config = FrankaConfig(
        node_rank=0,
        robot_ip=robot_ip,
        camera_serials=list(serials),
        camera_type=camera_type,
        gripper_type="franka",
        disable_validate=True,
    )
    hardware = FrankaHWInfo(
        type=FrankaRobot.HW_TYPE,
        model=FrankaRobot.HW_TYPE,
        config=franka_config,
    )

    if not ray.is_initialized():
        ray.init(log_to_driver=False, logging_level="ERROR")

    env = None
    save_dir = Path(args.save_dir)
    saved: list[str] = []
    radii_log: list[float] = []
    rpy_log: list[float] = []
    sphere_ok = True
    orient_ok = True
    photos_nonzero = True
    returned_ok = False
    duration = 0.0
    n_done = 0
    max_radius_seen = 0.0
    max_rpy_seen = 0.0

    def _record(obs: dict) -> tuple[np.ndarray, np.ndarray]:
        nonlocal max_radius_seen, max_rpy_seen, sphere_ok, orient_ok
        xyz = _tcp_xyz(obs)
        rpy = _tcp_rpy(obs)
        rad = float(np.linalg.norm(xyz - origin_xyz))
        rpy_delta = float(np.max(np.abs(wrap_rpy_delta(rpy, origin_rpy))))
        radii_log.append(rad)
        rpy_log.append(rpy_delta)
        max_radius_seen = max(max_radius_seen, rad)
        max_rpy_seen = max(max_rpy_seen, rpy_delta)
        if rad > fail_radius:
            sphere_ok = False
        if rpy_delta > RPY_MAX_DELTA_RAD:
            orient_ok = False
        return xyz, rpy

    def _maybe_photo(obs: dict, step_idx: int) -> None:
        nonlocal photos_nonzero
        if step_idx not in PHOTO_NAMES:
            return
        frames = _frames_from_obs(obs)
        frame = frames["wrist_1"]
        if int(frame.max()) == 0:
            photos_nonzero = False
        path = save_dir / PHOTO_NAMES[step_idx]
        _save_jpeg(path, frame)
        saved.append(str(path))
        print(f"saved {path} step={step_idx} mean={float(frame.mean()):.2f}")

    def _refresh_tcp(obs: dict) -> dict:
        """Re-read TCP after settle wait without sending a zero action.

        A zero ``env.step`` would set the Cartesian target to the current
        measured pose and wipe an overshoot setpoint.
        """
        tcp = np.asarray(env.get_wrapper_attr("get_tcp_pose")(), dtype=np.float64)
        pose = np.concatenate([tcp[:3], _rpy_from_quat(tcp[3:7])])
        refreshed = {**obs, "state": {**obs["state"]}}
        refreshed["state"]["tcp_pose"] = pose.astype(np.float32)
        return refreshed

    def _return_home(obs: dict) -> dict:
        """Walk TCP back to the probed origin.

        Do not use the 5 mm wander cap or sphere projection. Command up to
        5 cm * 2x remaining (mirror through origin) so Cartesian impedance
        sag of ~10 mm lands the *measured* TCP on the origin. After each
        ``step``, sleep without a zero action so the tracker keeps that
        setpoint.
        """
        for i in range(RETURN_MAX_STEPS):
            xyz = _tcp_xyz(obs)
            err_vec = origin_xyz - xyz
            err = float(np.linalg.norm(err_vec))
            if err <= RETURN_TOL_M:
                print(f"return_to_origin: arrived in {i} step(s)")
                return obs
            cmd_len = min(RETURN_STEP_M, err * RETURN_GAIN)
            delta = err_vec * (cmd_len / err)
            action = _zero_action(env)
            action[:3] = delta.astype(np.float32)
            obs, _, _, _, _ = env.step(action)
            time.sleep(RETURN_SETTLE_S)
            try:
                obs = _refresh_tcp(obs)
            except Exception as exc:
                print(f"return refresh warning (ignored): {exc}")
            _record(obs)
            dmm = (origin_xyz - _tcp_xyz(obs)) * 1000.0
            print(
                f"return step {i + 1}/{RETURN_MAX_STEPS} "
                f"r={radii_log[-1]*1000:.1f} mm "
                f"|cmd|={float(np.linalg.norm(delta))*1000:.2f} mm "
                f"dxyz=[{dmm[0]:.1f},{dmm[1]:.1f},{dmm[2]:.1f}] mm"
            )
        print(
            "return_to_origin: did not converge, "
            f"err={float(np.linalg.norm(_tcp_xyz(obs) - origin_xyz)) * 1000:.2f} mm"
        )
        return obs

    try:
        env = gym.make(
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
                "safe_smoke_hold": True,
                "action_scale": [1.0, 1.0, 1.0],
                "step_frequency": STEP_FREQUENCY_HZ,
                "max_num_steps": 400,
            },
            worker_info=None,
            hardware_info=hardware,
            env_idx=0,
            env_cfg={
                "use_spacemouse": False,
                "use_relative_frame": False,
                "no_gripper": True,
            },
        )
        obs, _ = env.reset()
        check("env_reset", True)
        frames = _frames_from_obs(obs)
        keys = list(frames.keys())
        if not check("obs_frames_nonempty", bool(keys), str(keys)):
            return result("9", False, reason="empty frames")
        if not check("wrist_1_present", "wrist_1" in frames, str(keys)):
            return result("9", False, frame_keys=keys)

        frame0 = frames["wrist_1"]
        shape_ok = tuple(frame0.shape) == (128, 128, 3) and frame0.dtype == np.uint8
        check(
            "frame_wrist_1_uint8_128",
            shape_ok,
            f"shape={tuple(frame0.shape)} dtype={frame0.dtype}",
        )
        if not shape_ok:
            return result("9", False, reason="unexpected wrist_1 frame")

        _clear_step9_jpegs(save_dir)
        pose0 = np.asarray(obs["state"]["tcp_pose"]).reshape(-1)
        print(f"obs tcp_pose dim={pose0.size} values={pose0.tolist()}")
        xyz, _ = _record(obs)
        print(
            f"origin xyz={xyz.tolist()} radius_now={radii_log[-1]*1000:.2f} mm"
        )
        _maybe_photo(obs, 0)

        rng = np.random.default_rng(args.seed)
        waypoint = origin_xyz.copy()
        t0 = time.monotonic()
        timed_out = False
        for i in range(N_MOTION_STEPS):
            elapsed = time.monotonic() - t0
            if elapsed > MOTION_TIMEOUT_S:
                timed_out = True
                print(f"motion timeout at step {i} after {elapsed:.2f}s")
                break
            if i % WAYPOINT_PERIOD_STEPS == 0:
                waypoint = sample_in_ball(rng, radius, origin_xyz)
                print(
                    f"waypoint {i // WAYPOINT_PERIOD_STEPS + 1}: "
                    f"{waypoint.tolist()} "
                    f"|Δ|={(np.linalg.norm(waypoint - origin_xyz)*1000):.1f} mm"
                )
            xyz = _tcp_xyz(obs)
            delta = clipped_delta(xyz, waypoint, origin_xyz, radius, max_step)
            action = _zero_action(env)
            action[:3] = delta.astype(np.float32)
            obs, _, _, _, _ = env.step(action)
            n_done += 1
            _record(obs)
            print(
                f"step {i + 1}/{N_MOTION_STEPS} "
                f"r={radii_log[-1]*1000:.1f} mm "
                f"rpyΔ={rpy_log[-1]:.3f} rad "
                f"|cmd|={float(np.linalg.norm(delta))*1000:.2f} mm"
            )
            if (i + 1) in PHOTO_NAMES:
                _maybe_photo(obs, i + 1)
            if not sphere_ok or not orient_ok:
                print("safety abort: sphere or orientation limit exceeded")
                break
        duration = time.monotonic() - t0

        obs = _return_home(obs)
        returned_ok = float(np.linalg.norm(_tcp_xyz(obs) - origin_xyz)) <= RETURN_TOL_M
    finally:
        if env is not None:
            _close_env(env)

    jpeg_files = sorted(save_dir.glob("*.jpg"))
    n_jpeg = len(jpeg_files)
    nonempty = all(p.stat().st_size > 0 for p in jpeg_files) if jpeg_files else False
    expected_names = {PHOTO_NAMES[i] for i in PHOTO_STEPS}
    have_names = {p.name for p in jpeg_files}

    dur_ok = (not timed_out) and (9.0 <= duration <= 12.0) and n_done == N_MOTION_STEPS
    check(
        "motion_duration_s",
        dur_ok,
        f"{duration:.2f}s steps={n_done}/{N_MOTION_STEPS} timeout={timed_out}",
    )
    check(
        "tcp_inside_sphere",
        sphere_ok and max_radius_seen <= fail_radius,
        f"max_r={max_radius_seen*1000:.1f} mm fail={fail_radius*1000:.1f} mm",
    )
    check(
        "orientation_locked",
        orient_ok and max_rpy_seen <= RPY_MAX_DELTA_RAD,
        f"max_rpyΔ={max_rpy_seen:.3f} rad limit={RPY_MAX_DELTA_RAD}",
    )
    check(
        "photos_count_5",
        n_jpeg == 5 and have_names == expected_names and nonempty,
        f"n={n_jpeg} files={[p.name for p in jpeg_files]}",
    )
    check("frame_wrist_1_nonzero", photos_nonzero, f"saved={len(saved)}")
    check(
        "return_to_origin",
        returned_ok,
        f"final_r={radii_log[-1]*1000:.1f} mm" if radii_log else "no tcp",
    )

    ok = (
        dur_ok
        and sphere_ok
        and orient_ok
        and n_jpeg == 5
        and have_names == expected_names
        and nonempty
        and photos_nonzero
        and returned_ok
    )
    return result(
        "9",
        ok,
        camera_type=camera_type,
        camera_serials=serials,
        origin=rest_pose,
        duration_s=round(duration, 3),
        motion_steps=n_done,
        max_radius_m=round(max_radius_seen, 5),
        max_rpy_delta_rad=round(max_rpy_seen, 4),
        photos=saved,
        returned=returned_ok,
    )


def main() -> int:
    args = parse_args()
    try:
        if args.math_only:
            return _run_math_tests()
        return _run_robot(args)
    except Exception as exc:
        check("uncaught", False, f"{type(exc).__name__}: {exc}")
        step = "9math" if args.math_only else "9"
        return result(step, False, reason=str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
