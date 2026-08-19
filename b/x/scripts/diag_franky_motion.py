#!/usr/bin/env python3
"""Which franky motion primitive actually moves this arm? (no Ray, no env)

``FrankaEnv._interpolate_move`` waypoints go through
``CartesianImpedanceTracker.set_target``. In franky 0.19 that tracker starts an
**asynchronous** ``robot.move(motion, asynchronous=True)`` in its constructor;
if that control thread dies, ``set_target`` silently writes to a dead reference
handle and the arm never moves (our cube-place ``dz=0.0000``). The stored
exception only surfaces on ``join_motion`` / the next ``move``.

This script talks to ``franky`` directly so Ray, the Worker layer and FrankaEnv
are out of the picture.

    --probe                 read-only: errors / is_in_control / TCP / joints
    --test-hold             tracker at the *current* pose, watch is_running
                            (no commanded displacement; safest liveness test)
    --test-impedance        tracker + set_target ramp (needs --yes-move)
    --test-cartesian-motion blocking robot.move(CartesianMotion) (needs --yes-move)

Run inside the franky container after
``source b/x/configs/setup_before_ray_5090.sh``, with nothing else holding FCI.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from franky_ext.tcp_probe import describe_robot_mode  # noqa: E402

DEFAULT_IP = "172.16.0.2"
# Matches PegInsertionConfig.compliance_param / franky_controller defaults.
K_TRANS = 2000.0
K_ROT = 150.0
ERR_CLIP_M = 0.05
ERR_CLIP_RAD = 0.3
MAX_DELTA_TAU = 0.3
GAINS_TC = 0.089
DYNAMICS_FACTOR = 0.2


def _fmt(v) -> str:
    return "[" + ", ".join(f"{float(x):.4f}" for x in np.asarray(v).reshape(-1)) + "]"


def _report(robot, label: str) -> None:
    print(
        f"{label}: has_errors={robot.has_errors} "
        f"is_in_control={robot.is_in_control} "
        f"signal={robot.current_control_signal_type}"
    )


def _tcp(robot) -> np.ndarray:
    return np.asarray(robot.current_pose.end_effector_pose.translation, dtype=np.float64)


def _quat(robot) -> np.ndarray:
    return np.asarray(robot.current_pose.end_effector_pose.quaternion, dtype=np.float64)


def probe(robot) -> str:
    """Print state and return ``robot_mode``. Motion only runs in Idle."""
    _report(robot, "probe")
    mode = str(robot.state.robot_mode)
    print(f"probe robot_mode={mode}")
    print(f"probe tcp xyz={_fmt(_tcp(robot))} quat={_fmt(_quat(robot))}")
    print(f"probe q={_fmt(robot.current_joint_positions)}")
    if mode != "RobotMode.Idle":
        print(f"probe WARNING: {describe_robot_mode(mode)}")
    return mode


def _make_tracker(robot, franky):
    nullspace_target = np.asarray(robot.current_joint_positions, dtype=np.float64).copy()
    return franky.CartesianImpedanceTracker(
        robot,
        translational_stiffness=K_TRANS,
        rotational_stiffness=K_ROT,
        nullspace_target=nullspace_target,
        nullspace_stiffness=5.0,
        translational_error_clip=np.full(3, ERR_CLIP_M),
        rotational_error_clip=np.full(3, ERR_CLIP_RAD),
        max_delta_tau=MAX_DELTA_TAU,
        gains_time_constant=GAINS_TC,
    )


def test_hold(robot, franky, seconds: float) -> bool:
    """Target = current pose. Does the async torque motion stay alive at all?"""
    print(f"\n=== impedance hold (no displacement) for {seconds:.1f}s ===")
    robot.recover_from_errors()
    start_xyz = _tcp(robot)
    quat = _quat(robot)
    tracker = _make_tracker(robot, franky)
    print(f"tracker created: is_running={tracker.is_running}")

    alive = True
    steps = max(1, int(seconds * 10))
    for i in range(steps):
        tracker.set_target(franky.Affine(start_xyz, quat))
        time.sleep(0.1)
        live = _tcp(robot)
        if not tracker.is_running:
            print(f"  !! died at t={0.1 * (i + 1):.1f}s live={_fmt(live)}")
            alive = False
            break
        if i % 5 == 0:
            print(
                f"  t={0.1 * (i + 1):.1f}s live={_fmt(live)} "
                f"dz={live[2] - start_xyz[2]:+.4f} is_running=True"
            )

    try:
        tracker.stop()
        print("tracker.stop() clean")
    except Exception as exc:
        print(f"tracker.stop() surfaced: {type(exc).__name__}: {exc}")
        alive = False
    _report(robot, "after hold")
    print(f"hold: tracker_alive={alive} sag={_tcp(robot)[2] - start_xyz[2]:+.4f} m")
    return alive


def test_impedance(robot, franky, dz: float, seconds: float) -> bool:
    """Create the tracker exactly like FrankyController, ramp z, watch is_running."""
    print(f"\n=== impedance tracker: ramp z by {dz:+.3f} m over {seconds:.1f}s ===")
    robot.recover_from_errors()
    start_xyz = _tcp(robot)
    quat = _quat(robot)
    tracker = _make_tracker(robot, franky)
    print(f"tracker created: is_running={tracker.is_running}")
    _report(robot, "after create")

    alive = True
    steps = max(1, int(seconds * 10))
    for i in range(steps):
        frac = (i + 1) / steps
        target = start_xyz + np.array([0.0, 0.0, dz * frac])
        tracker.set_target(franky.Affine(target, quat))
        time.sleep(0.1)
        live = _tcp(robot)
        running = tracker.is_running
        print(
            f"  step {i + 1:2d}/{steps} target_z={target[2]:.4f} "
            f"live={_fmt(live)} dz={live[2] - start_xyz[2]:+.4f} is_running={running}"
        )
        if not running:
            print("  !! tracker died: async control thread is gone")
            alive = False
            break

    try:
        tracker.stop()
        print("tracker.stop() clean")
    except Exception as exc:
        # stop() -> join_motion() re-raises whatever killed the control thread.
        print(f"tracker.stop() surfaced: {type(exc).__name__}: {exc}")
        alive = False
    _report(robot, "after stop")
    total = _tcp(robot)[2] - start_xyz[2]
    print(f"impedance total dz={total:+.4f} m (tracker_alive={alive})")
    return abs(total) > 0.005


def test_cartesian_motion(robot, franky, dz: float) -> bool:
    """Blocking robot.move(CartesianMotion) -- what DualFrankaEnv reset relies on."""
    print(f"\n=== blocking CartesianMotion: z {dz:+.3f} m ===")
    robot.recover_from_errors()
    start_xyz = _tcp(robot)
    quat = _quat(robot)
    target = start_xyz + np.array([0.0, 0.0, dz])
    motion = franky.CartesianMotion(
        franky.Affine(target, quat),
        reference_type=franky.ReferenceType.Absolute,
    )
    t0 = time.perf_counter()
    try:
        robot.move(motion)
        print(f"move() returned in {time.perf_counter() - t0:.2f}s")
    except Exception as exc:
        print(f"move() raised: {type(exc).__name__}: {exc}")
    _report(robot, "after move")
    total = _tcp(robot)[2] - start_xyz[2]
    print(f"CartesianMotion total dz={total:+.4f} m (target {dz:+.4f})")
    return abs(total) > 0.005


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default=DEFAULT_IP)
    parser.add_argument("--probe", action="store_true", help="read-only, no motion")
    parser.add_argument("--test-hold", action="store_true")
    parser.add_argument("--test-impedance", action="store_true")
    parser.add_argument("--test-cartesian-motion", action="store_true")
    parser.add_argument(
        "--yes-move",
        action="store_true",
        help="required for any test that commands the arm",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="run motion tests even when robot_mode is not Idle",
    )
    parser.add_argument("--dz", type=float, default=0.03, help="lift height (m)")
    parser.add_argument("--seconds", type=float, default=3.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    wants_motion = args.test_hold or args.test_impedance or args.test_cartesian_motion
    if wants_motion and not args.yes_move:
        raise SystemExit("refusing to move without --yes-move (stand by the e-stop)")
    if not wants_motion:
        args.probe = True

    import franky

    robot = franky.Robot(args.robot_ip)
    robot.relative_dynamics_factor = DYNAMICS_FACTOR
    print(f"connected to {args.robot_ip} (relative_dynamics_factor={DYNAMICS_FACTOR})")
    mode = probe(robot)

    if args.probe and not wants_motion:
        return 0
    if mode != "RobotMode.Idle" and not args.force:
        print("\nskipping motion tests (pass --force to try anyway)")
        return 1

    results = {}
    if args.test_hold:
        results["impedance hold alive"] = test_hold(robot, franky, args.seconds)
    if args.test_impedance:
        results["impedance set_target"] = test_impedance(
            robot, franky, args.dz, args.seconds
        )
    if args.test_cartesian_motion:
        results["blocking CartesianMotion"] = test_cartesian_motion(
            robot, franky, args.dz
        )

    print("\n=== summary ===")
    for name, ok in results.items():
        print(f"{name}: {'OK' if ok else 'FAILED'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
