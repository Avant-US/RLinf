#!/bin/bash
# Fix real-time scheduling issues for Franka 1 kHz control loop.
# Run with: sudo bash fix_rt.sh

set -e

echo "=== 1. Setting CPU governor to 'performance' on all cores ==="
for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > "$gov"
done
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

echo "=== 2. Disabling interrupt coalescing on FCI interface (enx00e04c68100b) ==="
# rx-usecs=0 means deliver packets immediately (critical for 1 kHz UDP)
ethtool -C enx00e04c68100b rx-usecs 0 tx-usecs 0 2>/dev/null || echo "(skip: adapter may not support it)"
ethtool -c enx00e04c68100b 2>/dev/null | grep -E "rx-usecs|tx-usecs" || true

echo "=== 3. Verifying RT kernel ==="
uname -r

echo "=== 4. Verifying realtime group and limits ==="
id | grep -o "realtime" || echo "WARNING: user not in realtime group"
ulimit -r

echo "=== Done. Now start robot_server with: ==="
echo "  taskset -c 3 chrt -f 80 ./build/robot_server --franka-ip 172.16.0.2 --bind 0.0.0.0:15555 --watchdog-ms 500 --max-joint-step 0.02 --filter-tau-s 0.08"
