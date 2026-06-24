#!/usr/bin/env bash
# Train RLinf openpi_au pi0.5 on LIBERO across 8 GPUs (global batch 128, 1000 steps,
# checkpoint every 200). Matches the openpi JAX reference (openpi05/b/tst/libero).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"

PY="${PY:-/mnt/r/VENV/openpi_venv/bin/python}"
OPENPI_SRC="${OPENPI_SRC:-/home/physical/SRC/Robot/openpi05/src}"

# Hydra group defaults (model/pi0_5_au, training_backend/fsdp) resolve via this dir.
export EMBODIED_PATH="${REPO}/examples/sft"
export PYTHONPATH="${REPO}:${OPENPI_SRC}:${PYTHONPATH:-}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0,1,2,3,4,5,6,7}"

# SFT trains on an offline dataset; EGL only needed if any env import touches GL.
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-egl}"

# Use the local LeRobot cache only -> avoids HF 429 when 8 ranks load data at once.
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_DATASETS_OFFLINE="${HF_DATASETS_OFFLINE:-1}"

# Example model dir (weights symlink + recomputed norm stats) and log dir.
export RLINF_LIBERO_MODEL="${RLINF_LIBERO_MODEL:-$HERE/_ckpt/pi05_base_pt}"
export RLINF_LIBERO_LOG="${RLINF_LIBERO_LOG:-$HERE/_out}"

# --- Preflight: norm stats present ---
NORM="${RLINF_LIBERO_MODEL}/physical-intelligence/libero/norm_stats.json"
if [ ! -f "$NORM" ]; then
    echo "ERROR: norm stats not found at $NORM"
    echo "       Run: bash tests_au/example/libero/run_norm_stats.sh"
    exit 1
fi
if [ ! -e "${RLINF_LIBERO_MODEL}/model.safetensors" ]; then
    echo "ERROR: model weights not found at ${RLINF_LIBERO_MODEL}/model.safetensors"
    echo "       Convert pi05_base via rlinf/utils/ckpt_convertor/convert_openpi_jax_to_python.py"
    echo "       then symlink model.safetensors + config.json into ${RLINF_LIBERO_MODEL}/"
    exit 1
fi

# --- Preflight: GPUs free (warn only) ---
if command -v nvidia-smi >/dev/null 2>&1; then
    USED=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | paste -sd+ | bc 2>/dev/null || echo 0)
    echo "[run_train] total GPU memory used across visible devices: ${USED} MiB"
fi

cd "$REPO"
echo "[run_train] 8-GPU pi0.5 LIBERO SFT: batch 128, 1000 steps, save every 200, EMA 0.999"
echo "[run_train] model_path=$RLINF_LIBERO_MODEL  log=$RLINF_LIBERO_LOG"

"$PY" examples/sft/train_vla_sft_au.py \
    --config-path "$HERE" \
    --config-name libero_sft_pi05_au_8gpu

echo "[run_train] done. checkpoints under $RLINF_LIBERO_LOG/pi05_libero_8gpu_1k/checkpoints/"
