#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

SERVER_BIN="$PWD/robot/build/robot_server"
FRANKA_IP="172.16.0.2"
BIND_ADDR="0.0.0.0:15555"
USB_IFACE="enxc8a36265251b"
LOCAL_IP="172.16.0.1/24"
SERVER_WATCHDOG_MS="1500"
SERVER_MAX_JOINT_STEP="0.05"
SERVER_MAX_JOINT_DELTA="0.10"

# ── Ensure FCI network is up ───────────────────────────────────────
if ! ip -4 addr show dev "$USB_IFACE" 2>/dev/null | grep -q "172.16.0.1"; then
    echo "[server] assigning $LOCAL_IP to $USB_IFACE"
    sudo ip addr add "$LOCAL_IP" dev "$USB_IFACE" 2>/dev/null || true
    sudo ip link set "$USB_IFACE" up
    sleep 1
fi
if ! ping -c 1 -W 2 "$FRANKA_IP" >/dev/null 2>&1; then
    echo "[server] ERROR: cannot reach $FRANKA_IP — check cable and FCI activation"
    exit 1
fi
echo "[server] FCI reachable at $FRANKA_IP"

# ── Kill any previous robot_server ──────────────────────────────────
OLD_PIDS=$(pgrep -f robot_server || true)
if [[ -n "$OLD_PIDS" ]]; then
    echo "[server] killing previous robot_server (pid $OLD_PIDS)"
    kill $OLD_PIDS 2>/dev/null || true
    sleep 1
    kill -9 $OLD_PIDS 2>/dev/null || true
    sleep 0.5
fi

# ── Start robot server (foreground) ────────────────────────────────
echo "[server] starting robot_server on $BIND_ADDR ..."
echo "[server] cfg: watchdog_ms=$SERVER_WATCHDOG_MS max_joint_step=$SERVER_MAX_JOINT_STEP max_joint_delta=$SERVER_MAX_JOINT_DELTA"
exec taskset -c 3 chrt -f 80 "$SERVER_BIN" \
    --franka-ip "$FRANKA_IP" \
    --bind "$BIND_ADDR" \
    --watchdog-ms "$SERVER_WATCHDOG_MS" \
    --max-joint-step "$SERVER_MAX_JOINT_STEP" \
    --max-joint-delta "$SERVER_MAX_JOINT_DELTA" \
    --realtime-ignore
