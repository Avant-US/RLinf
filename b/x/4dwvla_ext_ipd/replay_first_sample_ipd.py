#!/usr/bin/env python3
"""Replay demonstration actions with franky joint impedance (standalone).

Reads ``plug_into_socket_lrb_4D_8sml``. The dataset's **first indexed row** is
``episode_index=0``, ``frame_index=0``. By default this script replays **all
596 actions of episode 0** at the dataset fps (30 Hz), which is the trajectory
that contains that first sample. Use ``--frames 1`` to send only that single
8D action once.

No evaluation stack, no joint/TCP/gripper safety clipping: targets are taken
from the parquet as recorded.

Gripper width is ``0.08 * (1 - action.gripper)``, matching training affine
``a = 1 - w/0.08``.

Before impedance replay, the arm makes a **slow blocking move** to
``franky_ext/dsplug/home_pose.json`` (open gripper, ramped joint waypoints,
``relative_dynamics_factor`` default 0.05). Impedance starts only after HOME.

Example (Franka container, FCI not held by another process)::

    python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --robot-ip 172.16.0.2
    python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --robot-ip 172.16.0.2 --home-dynamics 0.03

    # Only the first frame's action (one impedance tick):
    python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --robot-ip 172.16.0.2 --frames 1

    # Replay the 3rd demonstration (1-based), i.e. LeRobot episode_index=2:
    python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --robot-ip 172.16.0.2 --episode 3

    # List demonstrations in the dataset:
    python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --list-episodes

    # Dry-run: print the extracted sample, do not connect to the robot:
    python b/x/4dwvla_ext_ipd/replay_first_sample_ipd.py --dry-run --episode 2
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np
import pandas as pd

DEFAULT_DATASET = Path("/home/nvidia/bt/dt/plug_into_socket_lrb_4D_8sml")
W0_M = 0.08

# Impedance gains (N.m/rad, N.m.s/rad). Tunable here only; not a safety layer.
STIFFNESS = np.array(
    [103.75, 265.734, 227.273, 221.445, 27.0, 25.6, 20.5], dtype=np.float64
)
DAMPING = np.array(
    [16.7, 40.263, 25.0, 12.862, 2.12, 2.83, 2.66], dtype=np.float64
)
GRIPPER_MOVE_SPEED_M_S = 0.1
DEFAULT_HOME_POSE = (
    Path(__file__).resolve().parents[1] / "franky_ext" / "dsplug" / "home_pose.json"
)
# Blocking HOME uses libfranka ``relative_dynamics_factor`` (0..1). Lower = slower.
DEFAULT_HOME_DYNAMICS = 0.05
HOME_GRIPPER_OPEN_SPEED = 0.05
# Max joint change per HOME waypoint when ramping (rad); smaller = gentler motion.
HOME_RAMP_STEP_RAD = 0.02


def _scalar_cell(value) -> float:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.size != 1:
        raise ValueError(f"expected scalar cell, got shape {arr.shape}")
    return float(arr[0])


def _arm_cell(value) -> np.ndarray:
    return np.asarray(value, dtype=np.float64).reshape(7)


def _parquet_path(dataset_root: Path) -> Path:
    return dataset_root / "data" / "chunk-000" / "file-000.parquet"


def load_dataset_table(dataset_root: Path) -> pd.DataFrame:
    parquet = _parquet_path(dataset_root)
    if not parquet.is_file():
        raise FileNotFoundError(parquet)
    return pd.read_parquet(parquet)


def episode_catalog(dataset_root: Path) -> list[tuple[int, int]]:
    """Return ``[(episode_index, num_frames), ...]`` sorted by episode_index."""
    table = load_dataset_table(dataset_root)
    rows: list[tuple[int, int]] = []
    for ep_idx, group in table.groupby("episode_index", sort=True):
        rows.append((int(ep_idx), int(len(group))))
    return rows


def print_episode_catalog(dataset_root: Path) -> None:
    catalog = episode_catalog(dataset_root)
    print(f"Dataset: {dataset_root}", flush=True)
    print(f"{'#':>4}  {'episode_index':>14}  {'frames':>8}  {'duration@30Hz':>14}", flush=True)
    fps = 30.0
    info_path = dataset_root / "meta" / "info.json"
    if info_path.is_file():
        fps = float(json.loads(info_path.read_text())["fps"])
    for ordinal, (ep_idx, n_frames) in enumerate(catalog, start=1):
        duration = n_frames / fps
        print(
            f"{ordinal:4d}  {ep_idx:14d}  {n_frames:8d}  {duration:11.2f}s",
            flush=True,
        )
    print(
        f"\nUse --episode N with N in 1..{len(catalog)} "
        f"(or --episode-index for LeRobot 0-based id).",
        flush=True,
    )


def resolve_episode_index(
    *,
    episode: int | None,
    episode_index: int | None,
    dataset_root: Path,
) -> int:
    """Map CLI selection to LeRobot ``episode_index`` (0-based).

    ``--episode`` is 1-based (第 1 条示教 = ``episode_index`` 0).
    ``--episode-index`` overrides ``--episode`` when both are given.
    """
    if episode is not None and episode_index is not None:
        raise ValueError("Use only one of --episode or --episode-index, not both")

    catalog = episode_catalog(dataset_root)
    if not catalog:
        raise RuntimeError(f"no episodes in {dataset_root}")

    valid_indices = {ep for ep, _ in catalog}
    max_ordinal = len(catalog)

    if episode_index is not None:
        if episode_index not in valid_indices:
            raise ValueError(
                f"episode_index={episode_index} not in dataset; "
                f"available: {sorted(valid_indices)}"
            )
        return int(episode_index)

    ordinal = 1 if episode is None else int(episode)
    if ordinal < 1 or ordinal > max_ordinal:
        raise ValueError(
            f"--episode must be 1..{max_ordinal}, got {ordinal}"
        )
    return catalog[ordinal - 1][0]


def load_episode_actions(
    dataset_root: Path,
    episode_index: int = 0,
    max_frames: int | None = None,
) -> tuple[np.ndarray, np.ndarray, float, dict]:
    """Load ``action.arm`` and gripper command widths for one episode.

    Returns:
        arm: ``[N, 7]`` joint targets (rad).
        width_m: ``[N]`` finger opening from ``action.gripper`` (m).
        fps: dataset frame rate.
        meta: first-row metadata for logging.
    """
    info_path = dataset_root / "meta" / "info.json"
    fps = 30.0
    if info_path.is_file():
        fps = float(json.loads(info_path.read_text())["fps"])

    table = load_dataset_table(dataset_root)
    episode = (
        table[table["episode_index"] == episode_index]
        .sort_values("frame_index")
        .reset_index(drop=True)
    )
    if episode.empty:
        available = sorted(int(x) for x in table["episode_index"].unique())
        raise RuntimeError(
            f"episode_index={episode_index} not found under {dataset_root}; "
            f"available episode_index values: {available}"
        )

    if max_frames is not None:
        episode = episode.iloc[: max(0, int(max_frames))]

    arm_rows = [_arm_cell(row) for row in episode["action.arm"]]
    arm = np.stack(arm_rows, axis=0)
    grip_a = np.array([_scalar_cell(row) for row in episode["action.gripper"]], dtype=np.float64)
    width_m = W0_M * (1.0 - grip_a)

    first = episode.iloc[0]
    meta = {
        "episode_index": int(first["episode_index"]),
        "frame_index": int(first["frame_index"]),
        "task_index": int(first["task_index"]) if "task_index" in first else None,
        "num_frames": len(episode),
    }
    return arm, width_m, fps, meta


def load_home_joints(home_pose_path: Path) -> np.ndarray:
    """Read seven joint angles from ``home_pose.json``."""
    meta = json.loads(home_pose_path.read_text(encoding="utf-8"))
    joints = np.asarray(meta["joint_position_rad"], dtype=np.float64).reshape(7)
    if not np.all(np.isfinite(joints)):
        raise ValueError(f"{home_pose_path}: joint_position_rad must be finite")
    return joints


def _set_relative_dynamics(robot, factor: float) -> None:
    """Set motion speed factor (franky 0.19 uses ``RelativeDynamicsFactor``)."""
    import franky  # pyright: ignore[reportMissingImports]

    rdf_cls = getattr(franky, "RelativeDynamicsFactor", None)
    if rdf_cls is not None:
        try:
            robot.relative_dynamics_factor = rdf_cls(float(factor))
            return
        except (TypeError, ValueError):
            pass
    robot.relative_dynamics_factor = float(factor)


def _home_waypoints(current_q: np.ndarray, home_q: np.ndarray) -> list[np.ndarray]:
    """Linear ramp from ``current_q`` to ``home_q`` in small joint steps."""
    delta = home_q - current_q
    peak = float(np.max(np.abs(delta)))
    if peak <= HOME_RAMP_STEP_RAD:
        return [home_q.copy()]
    n = int(np.ceil(peak / HOME_RAMP_STEP_RAD))
    return [current_q + delta * (k / n) for k in range(1, n + 1)]


def go_home_slow(
    robot,
    gripper,
    home_q: np.ndarray,
    *,
    dynamics: float,
    open_gripper: bool = True,
) -> None:
    """Blocking, slow move to HOME before impedance replay starts."""
    import franky  # pyright: ignore[reportMissingImports]

    if open_gripper:
        try:
            gripper.open(HOME_GRIPPER_OPEN_SPEED)
        except TypeError:
            gripper.open()
        time.sleep(0.4)

    state = robot.state
    current_q = np.asarray(state.q[:7], dtype=np.float64)
    waypoints = _home_waypoints(current_q, home_q)
    print(
        f"Moving to HOME: {len(waypoints)} waypoint(s), "
        f"relative_dynamics_factor={dynamics:.3f}",
        flush=True,
    )

    # franky 0.19 returns RelativeDynamicsFactor, not a float — store as-is.
    saved_dynamics = robot.relative_dynamics_factor
    _set_relative_dynamics(robot, dynamics)
    try:
        for index, q in enumerate(waypoints):
            motion = franky.JointWaypointMotion([franky.JointWaypoint(q.tolist())])
            robot.move(motion)
            if index == 0 or index + 1 == len(waypoints):
                print(
                    f"  HOME segment {index + 1}/{len(waypoints)} q1={q[0]:+.3f}",
                    flush=True,
                )
    finally:
        robot.relative_dynamics_factor = saved_dynamics

    if open_gripper:
        try:
            gripper.open(HOME_GRIPPER_OPEN_SPEED)
        except TypeError:
            gripper.open()
        time.sleep(0.3)
    print("HOME reached.", flush=True)


class _AsyncGripperWidth:
    """Issue ``Gripper.move`` on a side thread so the arm loop keeps its rate."""

    def __init__(self, gripper) -> None:
        self._gripper = gripper
        self._lock = threading.Lock()
        self._target: float | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="ipd-gripper", daemon=True)
        self._thread.start()

    def command(self, width_m: float) -> None:
        with self._lock:
            self._target = float(width_m)

    def _loop(self) -> None:
        last_sent: float | None = None
        while not self._stop.is_set():
            with self._lock:
                target = self._target
            if target is None:
                time.sleep(0.002)
                continue
            if last_sent is not None and abs(target - last_sent) < 1e-5:
                time.sleep(0.002)
                continue
            try:
                self._gripper.move(max(0.0, target), GRIPPER_MOVE_SPEED_M_S)
            except Exception as exc:
                print(f"[gripper] move to {target:.4f} m failed: {exc}", flush=True)
            last_sent = target

    def shutdown(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)


def replay_on_robot(
    robot_ip: str,
    arm: np.ndarray,
    width_m: np.ndarray,
    hz: float,
    *,
    home_q: np.ndarray | None,
    home_dynamics: float,
    skip_home: bool,
) -> None:
    import franky  # pyright: ignore[reportMissingImports]

    period = 1.0 / hz
    print(
        f"Connecting to {robot_ip}: {len(arm)} commands, {len(arm) / hz:.2f} s at {hz:.1f} Hz",
        flush=True,
    )

    robot = franky.Robot(robot_ip)
    robot.recover_from_errors()
    gripper = franky.Gripper(robot_ip)

    if not skip_home:
        if home_q is None:
            raise ValueError("HOME requested but home_q is None")
        go_home_slow(robot, gripper, home_q, dynamics=home_dynamics)

    grip = _AsyncGripperWidth(gripper)
    tracker = franky.JointImpedanceTracker(
        robot,
        stiffness=STIFFNESS,
        damping=DAMPING,
        compensate_coriolis=True,
    )
    print("JointImpedanceTracker running", flush=True)

    prev_q: np.ndarray | None = None
    tick = time.perf_counter()
    try:
        for i, q_cmd in enumerate(arm):
            grip.command(width_m[i])
            if prev_q is None:
                tracker.set_target(q_cmd)
            else:
                tracker.set_target(q_cmd, dq=(q_cmd - prev_q) * hz)
            prev_q = q_cmd

            tick += period
            sleep_s = tick - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)

            if i == 0 or (i + 1) % max(1, int(hz)) == 0:
                print(
                    f"  frame {i + 1}/{len(arm)} q1={q_cmd[0]:+.3f} w={width_m[i]:.4f} m",
                    flush=True,
                )
    finally:
        grip.shutdown()
        try:
            tracker.stop()
        except Exception as exc:
            print(f"tracker.stop: {exc}", flush=True)
        try:
            robot.join_motion()
        except Exception:
            pass
        try:
            robot.recover_from_errors()
        except Exception:
            pass
    print("Done.", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="172.16.0.2")
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument(
        "--episode",
        type=int,
        default=1,
        metavar="N",
        help=(
            "Which demonstration to replay, 1-based (第 N 条示教). "
            "Default 1 = first episode (LeRobot episode_index 0)."
        ),
    )
    parser.add_argument(
        "--episode-index",
        type=int,
        default=None,
        metavar="IDX",
        help=(
            "LeRobot episode_index (0-based). Overrides --episode when set, "
            "e.g. 2 for the third demonstration."
        ),
    )
    parser.add_argument(
        "--list-episodes",
        action="store_true",
        help="Print demonstrations in the dataset and exit.",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=None,
        help="Cap replay length; default is full episode. Use 1 for only the first frame.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=None,
        help="Command rate; default is dataset fps from meta/info.json.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print extracted actions and exit without connecting to the robot.",
    )
    parser.add_argument(
        "--home-pose",
        type=Path,
        default=DEFAULT_HOME_POSE,
        help=f"JSON with joint_position_rad (default: {DEFAULT_HOME_POSE}).",
    )
    parser.add_argument(
        "--skip-home",
        action="store_true",
        help="Do not move to HOME before replay (impedance starts from current pose).",
    )
    parser.add_argument(
        "--home-dynamics",
        type=float,
        default=DEFAULT_HOME_DYNAMICS,
        help=(
            "libfranka relative_dynamics_factor for the blocking HOME move "
            f"(default {DEFAULT_HOME_DYNAMICS}, smaller is slower)."
        ),
    )
    args = parser.parse_args()

    if args.list_episodes:
        print_episode_catalog(args.dataset_root)
        return

    episode_index = resolve_episode_index(
        episode=None if args.episode_index is not None else args.episode,
        episode_index=args.episode_index,
        dataset_root=args.dataset_root,
    )
    catalog = episode_catalog(args.dataset_root)
    ordinal = next(
        i for i, (ep, _) in enumerate(catalog, start=1) if ep == episode_index
    )

    arm, width_m, fps, meta = load_episode_actions(
        args.dataset_root,
        episode_index=episode_index,
        max_frames=args.frames,
    )
    hz = float(fps if args.hz is None else args.hz)

    print(
        f"Replay selection: episode #{ordinal} (LeRobot episode_index={episode_index}) "
        f"frame={meta['frame_index']} replay_frames={meta['num_frames']} fps={fps}",
        flush=True,
    )
    print(f"  arm[0]   = {np.array2string(arm[0], precision=4, separator=', ')}", flush=True)
    print(f"  width[0] = {width_m[0]:.4f} m (from action.gripper)", flush=True)

    home_q: np.ndarray | None = None
    if not args.skip_home:
        if not args.home_pose.is_file():
            raise FileNotFoundError(
                f"HOME file not found: {args.home_pose} "
                "(use --skip-home or --home-pose)"
            )
        home_q = load_home_joints(args.home_pose)
        print(
            f"  HOME q   = {np.array2string(home_q, precision=4, separator=', ')}",
            flush=True,
        )
        print(f"  HOME move dynamics={args.home_dynamics}", flush=True)

    if args.dry_run:
        return

    replay_on_robot(
        args.robot_ip,
        arm,
        width_m,
        hz,
        home_q=home_q,
        home_dynamics=float(args.home_dynamics),
        skip_home=args.skip_home,
    )


if __name__ == "__main__":
    main()
