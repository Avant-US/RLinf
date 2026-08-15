#!/bin/bash
# Host-side RT / NIC tuning for Franka robot link (eno1).
set -euo pipefail
NIC="${FRANKA_NIC:-eno1}"
echo "=== tune_eno1: CPU governor -> performance ==="
if [ -w /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ] 2>/dev/null; then
  for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance >"$g"; done
else
  sudo bash -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do echo performance > "$g"; done'
fi
echo "=== tune_eno1: kernel.sched_rt_runtime_us=-1 ==="
if sysctl kernel.sched_rt_runtime_us 2>/dev/null | grep -q '\-1'; then echo "already -1"; else sudo sysctl -w kernel.sched_rt_runtime_us=-1; fi
echo "=== tune_eno1: ethtool coalesce ${NIC} ==="
if command -v ethtool >/dev/null 2>&1; then
  sudo ethtool -C "${NIC}" rx-usecs 0 tx-usecs 0 2>/dev/null || echo "warn: ethtool -C ${NIC} failed"
else echo "warn: ethtool not installed"; fi
echo "=== tune_eno1 done ==="
