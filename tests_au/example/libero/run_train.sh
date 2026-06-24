#!/bin/bash
set -euo pipefail

# ============================================================================
# RLinf Pi0.5 LIBERO Training Script
# ============================================================================
# Replicates the openpi JAX pi0.5 training on LIBERO with matching configs.
#
# Hardware:  8x NVIDIA H200 (143GB HBM3e each)
# Model:    pi0.5 base (PaliGemma 2B + Action Expert 300M)
# Dataset:  physical-intelligence/libero (LeRobot format)
# Training: Flow Matching loss, FSDP 8-way, EMA 0.999
# ============================================================================

VENV="/mnt/r/VENV/openpi_venv"
OPENPI_ROOT="/home/physical/SRC/Robot/openpi05"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Environment Variables ---
export XLA_PYTHON_CLIENT_MEM_FRACTION=0.9
export CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7
export XLA_FLAGS="--xla_gpu_enable_latency_hiding_scheduler=true"
export JAX_COMPILATION_CACHE_DIR="${HOME}/.cache/jax_compilation_cache"

echo "============================================================"
echo " RLinf Pi0.5 LIBERO Fine-Tuning"
echo "============================================================"
echo " GPUs:            8x H200"
echo " Global Batch:    128 (16 per GPU)"
echo " Steps:           1000"
echo " Checkpoint:      every 200 steps"
echo " FSDP:            8-way model sharding"
echo " EMA:             0.999"
echo " LR:              5e-5 → 5e-6 cosine decay"
echo " Optimizer:       AdamW (b1=0.9, b2=0.95, grad_clip=1.0)"
echo "============================================================"
echo ""

cd "${OPENPI_ROOT}"

"${VENV}/bin/python" "${SCRIPT_DIR}/train_pi05_libero_rlinf.py" pi05_libero \
    --exp-name=rlinf_pi05_libero_8gpu_1k \
    --batch-size=128 \
    --num-train-steps=1000 \
    --save-interval=200 \
    --log-interval=50 \
    --fsdp-devices=8 \
    --ema-decay=0.999 \
    --lr-schedule.warmup-steps=100 \
    --lr-schedule.peak-lr=5e-5 \
    --lr-schedule.decay-steps=1000 \
    --lr-schedule.decay-lr=5e-6 \
    --overwrite

echo ""
echo "============================================================"
echo " Training Complete!"
echo "============================================================"
