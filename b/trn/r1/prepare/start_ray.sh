#!/usr/bin/env bash
# R1 Pro FastWAM SFT — 启动 Ray (8 GPU)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/env.sh"

# 关键: raylet 会继承启动 shell 的 nofile 软限制, 默认 1024 会被 fd 用满导致
# "raylet is dead / Too many open files". 必须在 ray start 之前抬高.
ulimit -n 1048576 || ulimit -n "$(ulimit -Hn)"
echo "[ray] nofile 软限制: $(ulimit -Sn) (硬限制: $(ulimit -Hn))"

echo "[ray] 停止现有 Ray..."
ray stop --force 2>/dev/null || true
pkill -9 -f "ray/core/src/ray" 2>/dev/null || true
sleep 2

echo "[ray] 启动 Ray head (port=6399)..."
export CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7
# 单机绑回 127.0.0.1, 避免 GCP 内网 IP 的 node-manager 端口被防火墙拦截
ray start --head --port=6399 --num-gpus=8 \
  --node-ip-address=127.0.0.1 --dashboard-host=127.0.0.1
sleep 5

echo "[ray] 状态:"
ray status | head -10
RAYLET_PID="$(pgrep -f 'raylet/raylet' | head -1 || true)"
if [ -n "${RAYLET_PID}" ]; then
  echo "[ray] raylet fd 限制: $(grep -i 'open files' /proc/${RAYLET_PID}/limits)"
fi
echo
echo "[ray] Ray 已就绪。"
