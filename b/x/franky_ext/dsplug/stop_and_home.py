#!/usr/bin/env python3
"""Stop the current Franka command and move to the plug-task reference home.

This is a standalone ``franky`` client.  It deliberately does not attach to
Ray, because the process that owns the FCI connection must be stopped first.
The default mode is read-only.  ``--stop-only`` sends a software stop, while
``--execute`` sends a software stop and, after explicit confirmation, commands
the absolute joint home stored in ``home_pose.json``.
``--capture-home`` only reads a manually positioned, settled robot; adding
``--write-home`` persists that reading after a second confirmation.

The home is the global mean of ``observation.state.arm`` from
``b/d/frk1/plug/abs_stats.json``.  It is an operational dataset-reference
home, not a semantic label proving that every demonstration started there.

Examples (inside the Franky container)::

    python b/x/franky_ext/dsplug/stop_and_home.py --robot-ip 172.16.0.2
    python b/x/franky_ext/dsplug/stop_and_home.py \
        --robot-ip 172.16.0.2 --stop-only
    python b/x/franky_ext/dsplug/stop_and_home.py \
        --robot-ip 172.16.0.2 --execute

Before connecting, stop any RLinf/Ray process that owns FCI port 1337.  A
second client cannot take over an existing FCI session.  ``robot.stop()`` is a
software controlled stop; it is not a hardware E-stop and cannot clear an
unreset Desk fault or a pressed enabling/user-stop device.
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

# Allow this file to run as a path from either the host checkout or the
# /workspace/RLinf bind mount.  ``franky_ext`` is an out-of-tree extension.
_BX_ROOT = Path(__file__).resolve().parents[2]
if str(_BX_ROOT) not in sys.path:
    sys.path.insert(0, str(_BX_ROOT))

from franky_ext.dsplug.home_pose import (  # noqa: E402
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    default_home_path,
    load_home_pose,
)
from franky_ext.motion_limits import cartesian_collision_thresholds  # noqa: E402

LOGGER = logging.getLogger("dsplug.stop_and_home")
_READY_MODE = "RobotMode.Idle"
_SETTLED_DQ_RAD_S = 0.02
_DEFAULT_SETTLE_TIMEOUT_S = 2.0
_DEFAULT_JOINT_TOLERANCE_RAD = 0.01

_TORQUE_THRESHOLDS = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]


def _default_home_file() -> Path:
    return default_home_path()


def _array_or_none(value: Any, size: int) -> np.ndarray | None:
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError):
        return None
    return array[:size] if array.size >= size else None


def read_snapshot(robot: Any) -> dict[str, Any]:
    """Read a small, version-tolerant robot state snapshot."""
    state = robot.state
    q = _array_or_none(getattr(robot, "current_joint_positions", None), 7)
    if q is None:
        q = _array_or_none(getattr(state, "q", None), 7)
    dq = _array_or_none(getattr(robot, "current_joint_velocities", None), 7)
    if dq is None:
        dq = _array_or_none(getattr(state, "dq", None), 7)
    if q is None or dq is None:
        raise RuntimeError("franky did not expose seven joint positions and velocities")

    snapshot: dict[str, Any] = {
        "mode": str(state.robot_mode),
        "has_errors": bool(robot.has_errors),
        "q": q,
        "dq": dq,
        "dq_norm": float(np.linalg.norm(dq)),
    }
    try:
        affine = state.O_T_EE
        snapshot["tcp_xyz"] = _array_or_none(affine.translation, 3)
        snapshot["tcp_quat_xyzw"] = _array_or_none(affine.quaternion, 4)
    except Exception:  # noqa: BLE001 - diagnostics must not hide joint state
        snapshot["tcp_xyz"] = None
        snapshot["tcp_quat_xyzw"] = None
    return snapshot


def print_snapshot(label: str, snapshot: dict[str, Any]) -> None:
    """Print a human-readable state line."""
    q = np.round(snapshot["q"], 6).tolist()
    tcp = snapshot["tcp_xyz"]
    tcp_text = "n/a" if tcp is None else str(np.round(tcp, 6).tolist())
    print(
        f"{label}: mode={snapshot['mode']} has_errors={snapshot['has_errors']} "
        f"|dq|={snapshot['dq_norm']:.4f} rad/s q={q} tcp_xyz={tcp_text}"
    )


def stop_and_wait(robot: Any, timeout_s: float) -> dict[str, Any]:
    """Issue a software stop and wait until joint speed is settled."""
    try:
        robot.stop()
    except Exception as exc:  # noqa: BLE001 - state is the decisive evidence
        LOGGER.warning("robot.stop() returned %s: %s", type(exc).__name__, exc)

    deadline = time.monotonic() + timeout_s
    snapshot = read_snapshot(robot)
    while snapshot["dq_norm"] > _SETTLED_DQ_RAD_S and time.monotonic() < deadline:
        time.sleep(0.01)
        snapshot = read_snapshot(robot)
    if snapshot["dq_norm"] > _SETTLED_DQ_RAD_S:
        raise RuntimeError(
            f"arm did not settle after robot.stop(): |dq|={snapshot['dq_norm']:.4f} "
            f"> {_SETTLED_DQ_RAD_S:.4f} rad/s"
        )
    return snapshot


def tighten_collision_behavior(robot: Any, *, allow_default: bool) -> None:
    """Apply the same conservative reflex thresholds as the direct controller."""
    force_thresholds = cartesian_collision_thresholds()
    try:
        robot.set_collision_behavior(
            lower_torque_threshold=_TORQUE_THRESHOLDS,
            upper_torque_threshold=_TORQUE_THRESHOLDS,
            lower_force_threshold=force_thresholds,
            upper_force_threshold=force_thresholds,
        )
    except Exception as exc:  # noqa: BLE001 - fail closed before motion
        message = (
            "could not apply conservative collision thresholds; refusing home "
            f"motion ({type(exc).__name__}: {exc})"
        )
        if not allow_default:
            raise RuntimeError(message) from exc
        LOGGER.warning("%s; --allow-default-collision-behavior was supplied", message)


def confirm_home(current_q: np.ndarray, target_q: np.ndarray, yes: bool) -> None:
    """Require an unambiguous operator acknowledgement for the joint sweep."""
    max_delta = float(np.max(np.abs(target_q - current_q)))
    print(
        f"planned absolute joint move: max_delta={max_delta:.4f} rad "
        f"target={np.round(target_q, 6).tolist()}"
    )
    if yes:
        return
    answer = input("The arm is stopped. Type HOME to command this target: ").strip()
    if answer != "HOME":
        raise RuntimeError("operator did not confirm HOME; arm remains stopped")


def capture_home_metadata(
    path: Path,
    metadata: dict[str, Any],
    snapshot: dict[str, Any],
    *,
    write: bool,
) -> None:
    """Print or explicitly persist a manually positioned home."""
    captured = dict(metadata)
    captured["method"] = "manual_live_robot_capture"
    captured["source"] = "franky.Robot.state"
    captured["source_feature"] = "franky.Robot.state.q"
    captured["captured_at_utc"] = datetime.now(timezone.utc).isoformat()
    captured["joint_position_rad"] = snapshot["q"].tolist()
    # The original field was computed by offline URDF FK for the statistical
    # home.  It would be stale after a manual capture, so replace it with the
    # live FCI measurement under an explicit name.
    captured.pop("tcp_pose_from_urdf", None)
    captured["interpretation"] = (
        "Manually captured operational home. The robot was physically positioned "
        "by the operator, released to RobotMode.Idle, and then read through FCI."
    )
    if snapshot["tcp_xyz"] is not None and snapshot["tcp_quat_xyzw"] is not None:
        captured["tcp_pose_from_robot"] = {
            "position_m": snapshot["tcp_xyz"].tolist(),
            "quaternion_xyzw": snapshot["tcp_quat_xyzw"].tolist(),
        }

    print(json.dumps(captured, indent=2))
    if not write:
        print("home file unchanged; add --write-home after reviewing this capture")
        return

    answer = input("Type WRITE_HOME to replace the home file: ").strip()
    if answer != "WRITE_HOME":
        raise RuntimeError("operator did not confirm WRITE_HOME; file unchanged")
    backup = path.with_name(path.name + ".bak")
    backup.write_text(path.read_text())
    path.write_text(json.dumps(captured, indent=2) + "\n")
    print(f"wrote {path}; previous value backed up to {backup}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="172.16.0.2")
    parser.add_argument("--home-file", type=Path, default=_default_home_file())
    parser.add_argument(
        "--stop-only",
        action="store_true",
        help="stop the current command and verify settling, without moving home",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="stop, confirm, and move to the dataset-reference home",
    )
    parser.add_argument(
        "--capture-home",
        action="store_true",
        help="read a manually positioned, settled robot pose without moving it",
    )
    parser.add_argument(
        "--write-home",
        action="store_true",
        help="with --capture-home, persist the captured pose after WRITE_HOME confirmation",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the interactive HOME confirmation (use only with an operator present)",
    )
    parser.add_argument(
        "--open-gripper",
        action="store_true",
        help="explicitly open to 0.08 m before the arm move; may drop a held plug",
    )
    parser.add_argument(
        "--recover-errors",
        action="store_true",
        help=(
            "explicitly call recover_from_errors() after stopping; Desk faults "
            "may need manual recovery"
        ),
    )
    parser.add_argument(
        "--allow-default-collision-behavior",
        action="store_true",
        help="allow motion if conservative collision-threshold setup fails",
    )
    parser.add_argument("--dynamics-factor", type=float, default=0.1)
    parser.add_argument("--settle-timeout", type=float, default=_DEFAULT_SETTLE_TIMEOUT_S)
    parser.add_argument("--joint-tolerance", type=float, default=_DEFAULT_JOINT_TOLERANCE_RAD)
    parser.add_argument(
        "--print-home",
        action="store_true",
        help="print the persisted joint/TCP home and exit without connecting",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.stop_only and args.execute:
        raise SystemExit("--stop-only and --execute are mutually exclusive")
    if args.capture_home and (args.stop_only or args.execute):
        raise SystemExit("--capture-home cannot be combined with --stop-only/--execute")
    if args.write_home and not args.capture_home:
        raise SystemExit("--write-home requires --capture-home")
    if args.open_gripper and not args.execute:
        raise SystemExit("--open-gripper requires --execute")
    if args.recover_errors and not args.execute:
        raise SystemExit("--recover-errors requires --execute")
    if not 0.05 <= args.dynamics_factor <= 0.3:
        raise SystemExit("--dynamics-factor must be in [0.05, 0.3]")
    if args.settle_timeout <= 0 or args.joint_tolerance <= 0:
        raise SystemExit("--settle-timeout and --joint-tolerance must be positive")

    home_meta, home_q = load_home_pose(args.home_file)
    if args.print_home:
        print(json.dumps(home_meta, indent=2))
        return 0
    if not args.stop_only and not args.execute:
        print("read-only mode; use --stop-only or --execute to send a command")

    try:
        import franky  # pyright: ignore[reportMissingImports]
    except ImportError as exc:
        raise SystemExit(
            "franky is required for robot access; run this in the Franky container"
        ) from exc

    robot = None
    motion_active = False

    def emergency_stop(_signum: int, _frame: Any) -> None:
        if robot is not None:
            try:
                robot.stop()
            except Exception as exc:  # noqa: BLE001 - best effort on interruption
                LOGGER.error("robot.stop() after signal failed: %s", exc)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, emergency_stop)
    signal.signal(signal.SIGTERM, emergency_stop)

    try:
        robot = franky.Robot(args.robot_ip)
        snapshot = read_snapshot(robot)
        print_snapshot("connected", snapshot)

        if args.capture_home:
            if snapshot["mode"] != _READY_MODE:
                raise RuntimeError(
                    f"refusing capture in {snapshot['mode']}; release hand-guiding "
                    f"and wait for {_READY_MODE}"
                )
            if snapshot["has_errors"]:
                raise RuntimeError("refusing capture while the robot reports active errors")
            if snapshot["dq_norm"] > _SETTLED_DQ_RAD_S:
                raise RuntimeError(
                    f"refusing capture while moving: |dq|={snapshot['dq_norm']:.4f} "
                    f"> {_SETTLED_DQ_RAD_S:.4f} rad/s"
                )
            if np.any(snapshot["q"] < JOINT_LIMITS_LOWER) or np.any(
                snapshot["q"] > JOINT_LIMITS_UPPER
            ):
                raise RuntimeError("refusing capture: live joint state is outside FR3v2.1 limits")
            capture_home_metadata(
                args.home_file,
                home_meta,
                snapshot,
                write=args.write_home,
            )
            return 0

        if not (args.stop_only or args.execute):
            return 0

        snapshot = stop_and_wait(robot, args.settle_timeout)
        print_snapshot("stopped", snapshot)

        if args.stop_only:
            print("RESULT DSPLUG_STOP PASS")
            return 0

        if args.recover_errors and snapshot["has_errors"]:
            print("recovering errors only because --recover-errors was supplied")
            robot.recover_from_errors()
            time.sleep(0.2)
            snapshot = read_snapshot(robot)
            print_snapshot("after-recover", snapshot)

        if snapshot["mode"] != _READY_MODE:
            raise RuntimeError(
                f"refusing home motion in {snapshot['mode']}; expected {_READY_MODE}. "
                "Clear UserStopped/Reflex/Desk faults and retry."
            )

        tighten_collision_behavior(
            robot, allow_default=args.allow_default_collision_behavior
        )
        confirm_home(snapshot["q"], home_q, args.yes)

        if args.open_gripper:
            print("WARNING: opening the gripper may drop the plug")
            gripper = franky.Gripper(args.robot_ip)
            gripper.move(width=0.08, speed=0.05)

        robot.relative_dynamics_factor = args.dynamics_factor
        motion = franky.JointWaypointMotion(
            [franky.JointWaypoint(home_q.tolist())]
        )
        motion_active = True
        robot.move(motion)
        motion_active = False

        final = stop_and_wait(robot, args.settle_timeout)
        print_snapshot("home-result", final)
        error = float(np.max(np.abs(final["q"] - home_q)))
        if error > args.joint_tolerance:
            raise RuntimeError(
                f"home verification failed: max joint error={error:.6f} rad "
                f"> tolerance={args.joint_tolerance:.6f}"
            )
        print(
            f"home verified: max joint error={error:.6f} rad, "
            f"gripper={'opened' if args.open_gripper else 'unchanged'}"
        )
        print("RESULT DSPLUG_HOME PASS")
        return 0
    except KeyboardInterrupt:
        print("interrupted; software stop was requested")
        return 130
    finally:
        if robot is not None and motion_active:
            try:
                robot.stop()
            except Exception as exc:  # noqa: BLE001 - teardown must not mask failure
                LOGGER.error("final robot.stop() failed: %s", exc)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    raise SystemExit(main())
