#!/usr/bin/env bash
#
# Auto-recover Franka Research 3 from error state (red light)
# and move to safe home pose.
#
# Usage:  ./auto_recover.sh
#
# Recovery sequence:
#   1. Stop robot_server (must release the FCI connection)
#   2. Try recover_reflex (libfranka automaticErrorRecovery)
#   3. If that fails, prompt user to clear via Desk UI / EAD toggle
#   4. Restart robot_server
#   5. Move arm slowly to safe home pose with gripper open
#
set -euo pipefail
cd "$(dirname "$0")"

FRANKA_IP="172.16.0.2"
RECOVER_BIN="$PWD/robot/build/recover_reflex"
ROBOT_HOST="127.0.0.1"
ROBOT_PORT="15555"
RECOVER_TIMEOUT=8

# ── Preflight checks ──────────────────────────────────────────────

if [[ ! -x "$RECOVER_BIN" ]]; then
    echo "[recover] ERROR: $RECOVER_BIN not found or not executable."
    echo "[recover] Build it:  cd robot && cmake --build build -j\$(nproc)"
    exit 1
fi

if ! ping -c 1 -W 2 "$FRANKA_IP" >/dev/null 2>&1; then
    echo "[recover] ERROR: cannot reach $FRANKA_IP — check FCI cable"
    exit 1
fi
echo "[recover] FCI reachable at $FRANKA_IP"

# ── Step 1: Stop robot_server (release FCI) ───────────────────────

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Step 1: Stopping robot_server (release FCI connection)"
echo "════════════════════════════════════════════════════════════"

OLD_PIDS=$(pgrep -f robot_server || true)
if [[ -n "$OLD_PIDS" ]]; then
    echo "[recover] Killing robot_server (pid $OLD_PIDS) ..."
    kill $OLD_PIDS 2>/dev/null || true
    sleep 1
    kill -9 $OLD_PIDS 2>/dev/null || true
    sleep 1
    echo "[recover] robot_server stopped."
else
    echo "[recover] robot_server was not running."
fi

# ── Step 2: Try automaticErrorRecovery via recover_reflex ─────────

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Step 2: Clearing robot error (recover_reflex)"
echo "════════════════════════════════════════════════════════════"

recovered=false
for attempt in 1 2; do
    echo "[recover] Attempt $attempt/2 ..."
    set +e
    timeout "$RECOVER_TIMEOUT" "$RECOVER_BIN" "$FRANKA_IP"
    rc=$?
    set -e
    if [[ $rc -eq 0 ]]; then
        echo "[recover] Error cleared via automaticErrorRecovery!"
        recovered=true
        break
    else
        echo "[recover] recover_reflex failed (exit $rc, attempt $attempt)."
        sleep 2
    fi
done

# ── Step 3: If auto-recovery failed, guide manual Desk clear ─────

if [[ "$recovered" != "true" ]]; then
    echo ""
    echo "════════════════════════════════════════════════════════════"
    echo "  automaticErrorRecovery could not clear this error."
    echo "  This usually means a safety lockout (joint limit, etc.)"
    echo ""
    echo "  Please do ONE of the following:"
    echo ""
    echo "    A) Desk UI:"
    echo "       1. Run:  ./switch_franka.sh desk"
    echo "       2. Open https://192.168.0.1 in a browser"
    echo "       3. Click the lock icon -> Unlock joints"
    echo "       4. Click the lock icon -> Lock joints"
    echo "       5. Run:  ./switch_franka.sh fci"
    echo ""
    echo "    B) Toggle the enabling device (black button on base):"
    echo "       Release it, wait 3s, press it again."
    echo ""
    echo "    C) Power-cycle the robot (last resort)."
    echo "════════════════════════════════════════════════════════════"
    echo ""
    echo "[recover] Waiting for you to clear the error ..."
    echo "[recover] (Press Enter here after the light turns white/yellow)"
    read -r

    echo "[recover] Verifying recovery via recover_reflex ..."
    set +e
    timeout "$RECOVER_TIMEOUT" "$RECOVER_BIN" "$FRANKA_IP"
    rc=$?
    set -e
    if [[ $rc -ne 0 ]]; then
        echo "[recover] Still in error (exit $rc). Trying once more in 3s ..."
        sleep 3
        set +e
        timeout "$RECOVER_TIMEOUT" "$RECOVER_BIN" "$FRANKA_IP"
        rc=$?
        set -e
    fi
    if [[ $rc -ne 0 ]]; then
        echo "[recover] FAILED: robot is still in error. Cannot proceed."
        exit 1
    fi
    echo "[recover] Error cleared!"
fi

# ── Step 4: Restart robot_server ──────────────────────────────────

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Step 4: Restarting robot_server"
echo "════════════════════════════════════════════════════════════"

echo "[recover] Starting robot_server in background ..."
nohup bash ./run_server.sh > /tmp/robot_server_recover.log 2>&1 &
SERVER_PID=$!
echo "[recover] robot_server started (pid $SERVER_PID)"
echo "[recover] Waiting 6s for robot_server to initialize ..."
sleep 6

if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "[recover] ERROR: robot_server exited. Check /tmp/robot_server_recover.log"
    tail -20 /tmp/robot_server_recover.log 2>/dev/null || true
    exit 1
fi
echo "[recover] robot_server is running."

# ── Step 5: Move to safe home pose ────────────────────────────────

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Step 5: Moving to safe home pose"
echo "════════════════════════════════════════════════════════════"

python3 - "$ROBOT_HOST" "$ROBOT_PORT" <<'PYEOF'
import sys
import time
import math
import numpy as np

from station.robot_client import RobotClient, RobotConnectionParams
from station.robot_protocol import CmdMode

ROBOT_HOST = sys.argv[1]
ROBOT_PORT = int(sys.argv[2])

SAFE_POSE_Q = [0.0, -0.628, 0.0, -2.513, 0.0, 1.885, 0.785]
GRIPPER_OPEN = 0.08
TOLERANCE_RAD = 0.02
SPEED_LIMIT_RAD_S = 0.3
HZ = 50

def main():
    print(f"[recover] Connecting to robot_server at {ROBOT_HOST}:{ROBOT_PORT} ...")
    client = RobotClient(RobotConnectionParams(host=ROBOT_HOST, command_port=ROBOT_PORT))

    try:
        client.connect()
    except Exception as e:
        print(f"[recover] Connection failed: {e}")
        sys.exit(1)

    print("[recover] Waiting for robot state...")
    state = None
    for _ in range(100):
        state = client.get_latest_state()
        if state is not None:
            break
        time.sleep(0.1)

    if state is None:
        print("[recover] No state received — giving up.")
        client.close()
        sys.exit(1)

    print(f"[recover] Robot mode={state.robot_mode} error_code={state.error_code}")

    if state.error_code != 0 or state.robot_mode == 2:
        print("[recover] Robot still in error state. Waiting up to 20s for robot_server to auto-recover...")
        deadline = time.time() + 20
        while time.time() < deadline:
            time.sleep(1.0)
            state = client.get_latest_state()
            if state is None:
                continue
            if state.error_code == 0 and state.robot_mode != 2:
                print("[recover] Error cleared by robot_server!")
                break
        else:
            print("[recover] Robot still in error. Cannot proceed.")
            client.close()
            sys.exit(1)

    current_q = np.array(state.q)
    target_q = np.array(SAFE_POSE_Q)

    print(f"[recover] Current Q: {[round(x, 3) for x in current_q]}")
    print(f"[recover] Target  Q: {[round(x, 3) for x in target_q]}")

    max_diff = float(np.max(np.abs(target_q - current_q)))
    print(f"[recover] Max joint distance: {max_diff:.4f} rad")

    if max_diff < TOLERANCE_RAD:
        print("[recover] Already at safe pose. Opening gripper...")
    else:
        duration = max(3.0, max_diff / SPEED_LIMIT_RAD_S)
        steps = int(duration * HZ)
        print(f"[recover] Moving to safe pose in {duration:.1f}s ({steps} steps) ...")

        for i in range(steps):
            alpha = (1 - math.cos((i + 1) / steps * math.pi)) / 2
            q_cmd = current_q + alpha * (target_q - current_q)
            client.send_arm_command(
                q=tuple(float(x) for x in q_cmd),
                gripper=GRIPPER_OPEN,
                mode=CmdMode.JOINT_POSITION,
                validity_ms=500,
                enable_arm=True,
                enable_gripper=True,
            )
            time.sleep(1.0 / HZ)
            if (i + 1) % (HZ * 2) == 0:
                pct = (i + 1) / steps * 100
                print(f"[recover]   {pct:.0f}%")

    print("[recover] Holding final pose (1s) ...")
    for _ in range(HZ):
        client.send_arm_command(
            q=tuple(float(x) for x in target_q),
            gripper=GRIPPER_OPEN,
            mode=CmdMode.JOINT_POSITION,
            validity_ms=500,
            enable_arm=True,
            enable_gripper=True,
        )
        time.sleep(1.0 / HZ)

    final_state = client.get_latest_state()
    if final_state is not None:
        final_q = np.array(final_state.q)
        err = float(np.max(np.abs(final_q - target_q)))
        print(f"[recover] Final max |q - target| = {err:.4f} rad")
        print(f"[recover] Gripper: {final_state.gripper:.4f} m")

    print("[recover] Recovery complete. Robot is at safe home pose.")
    client.close()

if __name__ == "__main__":
    main()
PYEOF

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Recovery finished — robot_server is running on :$ROBOT_PORT"
echo "════════════════════════════════════════════════════════════"
