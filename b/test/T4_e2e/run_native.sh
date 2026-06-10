#!/usr/bin/env bash
# T4: Run FastWAM native training for 50 steps (4 GPUs)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RESULTS_DIR="${T4_RESULTS_BASE:-/mnt/r/tmp/fw_test}/T4/native"
mkdir -p "${RESULTS_DIR}"

export FASTWAM_ROOT="${FASTWAM_ROOT:-/home/Luogang/SRC/Robot/FastWAM}"
export FASTWAM_PATH="${FASTWAM_ROOT}/src"
export DIFFSYNTH_MODEL_BASE_PATH="${DIFFSYNTH_MODEL_BASE_PATH:-/mnt/r/CKPT/VLA/FW}"
export DIFFSYNTH_SKIP_DOWNLOAD=true
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-4,5,6,7}"
export PYTHONPATH="${FASTWAM_PATH}:${PYTHONPATH:-}"

cd "${FASTWAM_ROOT}"

echo "[T4-native] Starting FastWAM native training (50 steps, 4 GPUs)..."
bash scripts/train_zero1.sh 4 \
  task=r1_pro_chassis_uncond_3cam_384_1e-4 \
  batch_size=1 \
  gradient_accumulation_steps=1 \
  max_steps=50 \
  save_every=999 \
  log_every=1 \
  seed=42 \
  wandb.enabled=false \
  "output_dir=${RESULTS_DIR}/run" \
  2>&1 | tee "${RESULTS_DIR}/train.log"

echo "[T4-native] Done. Logs at ${RESULTS_DIR}/train.log"
