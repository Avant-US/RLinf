#!/bin/bash
# RLT Stage 1 training launcher (run inside rlinf-4dwvla-gpu container)
# Usage: bash b/x/4dwvla_ext/rlt/launch_rlt_stage1.sh [config_path]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${1:-${SCRIPT_DIR}/configs/rlt_stage1_franka_plug.yaml}"
VENV_DIR="${VENV_DIR:-/opt/venv/4dwvla}"

source "${VENV_DIR}/bin/activate"

export PYTHONPATH="/workspace/RLinf:${PYTHONPATH:-}"
export DATA_DIR="${DATA_DIR:-/home/nvidia/bt/dt}"
export CKPT_DIR="${CKPT_DIR:-/home/nvidia/ckpts}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"

echo "=== RLT Stage 1 Training ==="
echo "Config: ${CONFIG}"
echo "Python: $(python --version)"
echo "GPU: $(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader 2>/dev/null || echo 'N/A')"
echo "==========================="

python "${SCRIPT_DIR}/train_4dwvla_rlt_stage1.py" --config "${CONFIG}"
