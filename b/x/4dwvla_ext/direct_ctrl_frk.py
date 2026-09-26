#!/usr/bin/env python3
"""Move the Franka EE by a Cartesian offset while keeping orientation.

Usage (inside rlinf-4dwvla-franky container, franky-0.19.0 venv):

    source /opt/venv/franky-0.19.0/bin/activate
    python3 direct_ctrl_frk.py [--dx 0.005] [--dy 0] [--dz 0] [--yes-move]

Franka base frame: +X = forward, +Y = left, +Z = up.
Requires --yes-move to actually command the arm.
"""
from __future__ import annotations

import argparse
import time

import numpy as np

DEFAULT_IP = "172.16.0.2"
DYNAMICS_FACTOR = 0.2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-ip", default=DEFAULT_IP)
    parser.add_argument("--dx", type=float, default=0.0, help="forward displacement in meters (+X)")
    parser.add_argument("--dy", type=float, default=0.0, help="left displacement in meters (+Y)")
    parser.add_argument("--dz", type=float, default=0.0, help="up displacement in meters (+Z)")
    parser.add_argument("--yes-move", action="store_true", help="required to command the arm")
    args = parser.parse_args()

    import franky

    robot = franky.Robot(args.robot_ip)
    robot.recover_from_errors()
    robot.relative_dynamics_factor = DYNAMICS_FACTOR
    print(f"Connected to {args.robot_ip} (dynamics_factor={DYNAMICS_FACTOR})")

    # Read current state
    pose = robot.current_pose  # RobotPose
    ee = pose.end_effector_pose  # Affine
    xyz = np.asarray(ee.translation, dtype=np.float64)
    quat = np.asarray(ee.quaternion, dtype=np.float64)

    print(f"Current EE position : [{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}]")
    print(f"Current EE quaternion: [{quat[0]:.5f}, {quat[1]:.5f}, {quat[2]:.5f}, {quat[3]:.5f}]")
    print(f"Current joint pos    : {np.array2string(np.asarray(robot.current_joint_positions), precision=4)}")

    target_xyz = xyz.copy()
    delta = np.array([args.dx, args.dy, args.dz])
    target_xyz += delta
    print(f"\nTarget EE position : [{target_xyz[0]:.5f}, {target_xyz[1]:.5f}, {target_xyz[2]:.5f}]")
    print(f"Target EE quaternion: same (orientation preserved)")
    print(f"Displacement       : dx={delta[0]*1000:+.1f} dy={delta[1]*1000:+.1f} dz={delta[2]*1000:+.1f} mm")

    if not args.yes_move:
        print("\nDry run — pass --yes-move to execute the motion.")
        return 0

    # Wait for the arm to be still
    dq = np.asarray(robot.current_joint_velocities, dtype=np.float64)
    dq_norm = float(np.linalg.norm(dq))
    if dq_norm > 0.02:
        print(f"Arm is moving (|dq|={dq_norm:.4f} rad/s), waiting 1s...")
        time.sleep(1.0)

    # Build and execute a blocking CartesianMotion
    target_affine = franky.Affine(target_xyz, quat)
    target_pose = franky.RobotPose(target_affine)
    target_state = franky.CartesianState(target_pose)
    motion = franky.CartesianMotion(
        target_state,
        reference_type=franky.ReferenceType.Absolute,
        relative_dynamics_factor=franky.RelativeDynamicsFactor(DYNAMICS_FACTOR),
    )

    print("\nExecuting CartesianMotion...")
    t0 = time.perf_counter()
    try:
        robot.move(motion)
        elapsed = time.perf_counter() - t0
        print(f"Motion completed in {elapsed:.2f}s")
    except Exception as exc:
        elapsed = time.perf_counter() - t0
        print(f"Motion raised after {elapsed:.2f}s: {type(exc).__name__}: {exc}")
        robot.recover_from_errors()

    time.sleep(0.3)

    # Report final state
    final_pose = robot.current_pose.end_effector_pose
    final_xyz = np.asarray(final_pose.translation, dtype=np.float64)
    final_quat = np.asarray(final_pose.quaternion, dtype=np.float64)
    actual_d = final_xyz - xyz
    print(f"\nFinal EE position  : [{final_xyz[0]:.5f}, {final_xyz[1]:.5f}, {final_xyz[2]:.5f}]")
    print(f"Final EE quaternion: [{final_quat[0]:.5f}, {final_quat[1]:.5f}, {final_quat[2]:.5f}, {final_quat[3]:.5f}]")
    print(f"Actual displacement: dx={actual_d[0]*1000:+.2f} dy={actual_d[1]*1000:+.2f} dz={actual_d[2]*1000:+.2f} mm")

    # Check orientation drift
    quat_dot = abs(float(np.dot(quat, final_quat)))
    ang_drift_rad = 2.0 * np.arccos(min(1.0, quat_dot))
    print(f"Orientation drift  : {np.degrees(ang_drift_rad):.3f} deg")

    error = actual_d - delta
    if np.linalg.norm(error) > 0.001:
        print(f"\nWARNING: displacement error {error[0]*1000:+.2f} {error[1]*1000:+.2f} {error[2]*1000:+.2f} mm")
    if ang_drift_rad > np.radians(1.0):
        print(f"\nWARNING: orientation drifted {np.degrees(ang_drift_rad):.2f} deg")

    print("\nDone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
