#!/usr/bin/env python3
import time
import math
import numpy as np
from station.robot_client import RobotClient, RobotConnectionParams
from station.robot_protocol import CmdMode

# Center of the DROID training distribution so the model starts in a
# familiar state. Center ≈ (DROID_Q_LO + DROID_Q_HI) / 2.
START_POSE_Q = [0.036, 0.273, -0.076, -1.614, -0.055, 2.320, 0.076]
TARGET_GRIPPER = 0.08  # Fully open (meters)
ARM_TOLERANCE_RAD = 0.02

# DROID training distribution bounds (same limits as station/action_adapter.py).
DROID_Q_LO = (-0.828, -0.840, -0.843, -2.773, -1.843, +1.172, -2.047)
DROID_Q_HI = (+0.900, +1.385, +0.692, -0.454, +1.732, +3.467, +2.198)


def _clamp_start_pose(q):
    out = []
    clipped = False
    for i, x in enumerate(q):
        lo = float(DROID_Q_LO[i])
        hi = float(DROID_Q_HI[i])
        y = max(lo, min(hi, float(x)))
        if abs(y - float(x)) > 1e-9:
            clipped = True
        out.append(y)
    if clipped:
        print("[reset] WARNING: start pose was out-of-distribution; clamped to DROID bounds.")
    return out

def main():
    print("[reset] Connecting to robot_server at 127.0.0.1:15555 ...")
    client = RobotClient(RobotConnectionParams(host="127.0.0.1", command_port=15555))
    try:
        try:
            client.connect()
        except Exception as e:
            print(f"[reset] Error connecting: {e}")
            print("[reset] Make sure ./run_server.sh is running!")
            return

        # Wait for state
        print("[reset] Waiting for robot state...")
        for _ in range(50):
            if client.get_latest_state() is not None:
                break
            time.sleep(0.1)

        state = client.get_latest_state()
        if state is None:
            print("[reset] No robot state received. Is the robot connected?")
            return

        # Check for errors (robot_server protocol: 0=connecting/idle, 1=running, 2=error, 3=mock)
        print(f"[reset] Robot Mode: {state.robot_mode} (0=idle/connecting, 1=running, 2=error, 3=mock)")

        if state.error_code != 0 or state.robot_mode == 2:
            print(f"\n[reset] !!! ROBOT ERROR DETECTED (Code {state.error_code}) !!!")
            print("[reset] The robot is locked (likely due to collision).")
            print("[reset] ACTION REQUIRED: clear the robot error (Desk UI Unlock, or run ./robot/build/recover_reflex 172.16.0.2).")
            print("[reset] Waiting for you to clear the error...")

            while state.error_code != 0 or state.robot_mode == 2:
                time.sleep(1.0)
                state = client.get_latest_state()
                if state is None:
                    print("[reset] Connection lost while waiting.")
                    return
                if state.error_code == 0 and state.robot_mode != 2:
                    print("[reset] Error cleared! Resuming...")
                    break

        current_q = np.array(state.q)
        target_q_list = _clamp_start_pose(START_POSE_Q)
        target_q = np.array(target_q_list)

        print(f"[reset] Current Q: {[round(x, 4) for x in current_q]}")
        print(f"[reset] Target (Start Pose): {[round(x, 4) for x in target_q_list]}")

        # Calculate duration based on max joint distance
        max_diff = float(np.max(np.abs(target_q - current_q)))
        arm_needs_move = max_diff >= float(ARM_TOLERANCE_RAD)
        hz = 50

        if arm_needs_move:
            # Safety: limit reset speed to ~0.4 rad/s
            speed_limit = 0.4
            duration = max(2.0, max_diff / speed_limit)
            steps = int(duration * hz)
            print(f"[reset] Moving to start pose in {duration:.1f}s ({steps} steps)...")

            for i in range(steps):
                # Cosine interpolation for smooth start/stop
                progress = (i + 1) / steps
                alpha = (1 - math.cos(progress * math.pi)) / 2
                q_cmd = current_q + alpha * (target_q - current_q)

                client.send_arm_command(
                    q=tuple(float(x) for x in q_cmd),
                    gripper=TARGET_GRIPPER,
                    mode=CmdMode.JOINT_POSITION,
                    validity_ms=500,
                    enable_arm=True,
                    enable_gripper=True,
                )
                time.sleep(1.0 / hz)
        else:
            # Important: still open/hold gripper even if arm is already near target.
            print("[reset] Arm already near target pose; enforcing open gripper hold.")

        # Hold final pose to stabilize and ensure gripper is fully open.
        print("[reset] Holding final pose (arm + gripper open)...")
        for _ in range(50):
            client.send_arm_command(
                q=tuple(float(x) for x in target_q_list),
                gripper=TARGET_GRIPPER,
                mode=CmdMode.JOINT_POSITION,
                validity_ms=500,
                enable_arm=True,
                enable_gripper=True,
            )
            time.sleep(1.0 / hz)

        state = client.get_latest_state()
        if state is not None:
            final_q = np.array(state.q)
            final_max_err = float(np.max(np.abs(final_q - target_q)))
            print(f"[reset] Final max |q-target| = {final_max_err:.4f} rad, gripper={float(state.gripper):.4f} m")

        print("[reset] Done.")
    finally:
        client.close()

if __name__ == "__main__":
    main()
