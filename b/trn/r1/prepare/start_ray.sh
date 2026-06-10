#!/usr/bin/env bash
# R1 Pro FastWAM SFT — 启动 Ray (8 GPU)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/env.sh"

echo "[ray] 停止现有 Ray..."
ray stop 2>/dev/null || true
sleep 2

echo "[ray] 启动 Ray head (port=6399)..."
export CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7
ray start --head --port=6399 --num-gpus=8
sleep 5

echo "[ray] 状态:"
ray status | head -10
echo
echo "[ray] Ray 已就绪。"
