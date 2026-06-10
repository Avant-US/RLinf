#!/usr/bin/env bash
# T5: RLinf 4-GPU training (20 steps)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}/../../.."
RESULTS_DIR="${T5_RESULTS_BASE:-/mnt/r/tmp/fw_test}/T5/4gpu"
mkdir -p "${RESULTS_DIR}"

export FASTWAM_ROOT="${FASTWAM_ROOT:-/home/Luogang/SRC/Robot/FastWAM}"
export FASTWAM_PATH="${FASTWAM_ROOT}/src"
export DIFFSYNTH_MODEL_BASE_PATH="${DIFFSYNTH_MODEL_BASE_PATH:-/mnt/r/CKPT/VLA/FW}"
export DIFFSYNTH_SKIP_DOWNLOAD=true
export R1PRO_DATA="${R1PRO_DATA:-/mnt/r/share/zwy/datasets/r1_pro_data_v2}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-4,5,6,7}"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.8}"
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
export RAY_ADDRESS="${RAY_ADDRESS:-127.0.0.1:6399}"

cd "${REPO_ROOT}"

ray status 2>/dev/null || { echo "ERROR: Ray not running"; exit 1; }

export EMBODIED_PATH="${REPO_ROOT}/examples/sft"
export PYTHONPATH="${REPO_ROOT}:${FASTWAM_PATH}:${PYTHONPATH:-}"

echo "[T5-4gpu] Starting RLinf 4-GPU training (20 steps)..."
python "${REPO_ROOT}/examples/sft/train_vla_sft.py" \
  --config-path "${REPO_ROOT}/examples/sft/config/" \
  --config-name r1_pro_sft_fastwam \
  runner.max_steps=20 \
  runner.save_interval=999 \
  runner.log_interval=1 \
  actor.micro_batch_size=1 \
  actor.global_batch_size=4 \
  actor.seed=42 \
  actor.optim.total_training_steps=20 \
  cluster.component_placement.actor=4-7 \
  "runner.logger.log_path=${RESULTS_DIR}" \
  2>&1 | tee "${RESULTS_DIR}/train.log"

echo "[T5-4gpu] Done."
