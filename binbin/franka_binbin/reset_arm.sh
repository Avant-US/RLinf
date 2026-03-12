#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

FRANKA_IP="172.16.0.2"
FCI_LOCAL="172.16.0.1"

find_franka_iface() {
    # 1) Interface that already has the FCI local IP
    local iface
    iface=$(ip -4 -o addr show | awk -v ip="$FCI_LOCAL" '$4 ~ ip {print $2; exit}')
    [ -n "$iface" ] && echo "$iface" && return 0

    # 2) USB adapter (enx*)
    for p in /sys/class/net/enx*; do
        [ -d "$p" ] && basename "$p" && return 0
    done

    # 3) Built-in Ethernet (en*), excluding loopback and wireless
    for p in /sys/class/net/en*; do
        [ -d "$p" ] || continue
        local name; name=$(basename "$p")
        [[ "$name" == enx* ]] && continue   # already checked
        echo "$name" && return 0
    done
    return 1
}

FRANKA_IFACE=$(find_franka_iface) || {
    echo "[reset] ERROR: No ethernet adapter found for Franka."
    echo "        Check your cable and try again."
    exit 1
}

echo "[reset] Franka arm reset to home position  (iface: $FRANKA_IFACE)"

# ── Ensure FCI network ─────────────────────────────────────────────
if ! ip -4 addr show dev "$FRANKA_IFACE" 2>/dev/null | grep -q "$FCI_LOCAL"; then
    echo "[reset] assigning $FCI_LOCAL/24 to $FRANKA_IFACE"
    sudo ip addr add "$FCI_LOCAL/24" dev "$FRANKA_IFACE" 2>/dev/null || true
    sudo ip link set "$FRANKA_IFACE" up
    sleep 1
fi

# ── Real-time tuning (critical for 1 kHz FCI loop) ───────────────
echo "[reset] applying real-time tuning …"

# CPU governor → performance (powersave causes huge latency at 1 kHz)
GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)
if [ "$GOV" != "performance" ]; then
    for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        sudo bash -c "echo performance > $g" 2>/dev/null
    done
    echo "        CPU governor: powersave → performance"
fi

# Stop thermald — it can throttle CPU during the control loop
if systemctl is-active --quiet thermald 2>/dev/null; then
    sudo systemctl stop thermald
    echo "        stopped thermald"
fi

# Stop irqbalance — it migrates NIC IRQs and causes latency spikes
if systemctl is-active --quiet irqbalance 2>/dev/null; then
    sudo systemctl stop irqbalance
    echo "        stopped irqbalance"
fi

# Disable RT throttling so the control loop is never preempted
RT_FILE=/proc/sys/kernel/sched_rt_runtime_us
if [ "$(cat "$RT_FILE")" != "-1" ]; then
    sudo bash -c "echo -1 > $RT_FILE"
    echo "        disabled RT throttling (sched_rt_runtime_us=-1)"
fi

# Minimise NIC interrupt coalescing for lowest latency
if command -v ethtool &>/dev/null; then
    sudo ethtool -C "$FRANKA_IFACE" rx-usecs 0 rx-frames 1 2>/dev/null || true
    echo "        set NIC coalescing to minimum"
fi

# ── Make sure robot_server is NOT running (it holds the FCI lock) ──
if pgrep -f robot_server >/dev/null 2>&1; then
    echo "[reset] ERROR: robot_server is running and holds the FCI connection."
    echo "        Stop it first (kill the process), then re-run this script."
    exit 1
fi

# ── Move to home directly via libfranka ────────────────────────────
echo "[reset] keep hand near E-STOP"
./robot/build/move_to_home --ip "$FRANKA_IP" --speed 0.3
