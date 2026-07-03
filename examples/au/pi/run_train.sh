#!/usr/bin/env bash
# Train RLinf openpi_au pi0.5 on the pushdoor (r1_pro) tst1 dataset across 8 GPUs
# (global batch 128, 50-step smoke test, checkpoint at 25/50). Same optimizer/
# precision/EMA recipe as the LIBERO reference (tests_au/example/libero). Base
# weights default to the vanilla pi0.5 base checkpoint on /mnt/r; see BASE_CKPT
# below to point at the converted pi05_r1pro_chassis_alig_newnorm weights instead.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"

PY="${PY:-/mnt/r/VENV/openpi_venv/bin/python}"
OPENPI_SRC="${OPENPI_SRC:-/home/physical/SRC/Robot/openpi05/src}"

# Hydra group defaults (model/pi0_5_au, training_backend/fsdp) resolve via this dir.
export EMBODIED_PATH="${REPO}/examples/sft"
export PYTHONPATH="${REPO}:${OPENPI_SRC}:${PYTHONPATH:-}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0,1,2,3,4,5,6,7}"

# This box may be shared with other jobs, one of which can run its own long-lived
# Ray cluster. RLinf's Cluster calls ray.init(address="auto"); per Ray's rules an
# "auto" address is silently REPLACED by RAY_ADDRESS when set, and address="local"
# means "always start a fresh local Ray instance" -- so this makes our smoke test
# spin up its own throwaway Ray cluster instead of piggybacking on another job's,
# with zero changes to RLinf's core scheduler code.
export RAY_ADDRESS="${RAY_ADDRESS:-local}"

# SFT trains on an offline dataset; EGL only needed if any env import touches GL.
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-egl}"

# Use the local LeRobot cache only -> avoids HF 429 when 8 ranks load data at once.
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_DATASETS_OFFLINE="${HF_DATASETS_OFFLINE:-1}"
# PyTorch-only pipeline: tell HF transformers not to import TensorFlow/Flax. Some
# venvs ship a TF whose protobuf is too old (ImportError: cannot import name
# 'runtime_version' from google.protobuf), which otherwise crashes any transformers
# import via image_transforms.py `import tensorflow`.
export USE_TF="${USE_TF:-0}"
export USE_FLAX="${USE_FLAX:-0}"

# Example model dir (weights symlink + recomputed norm stats), dataset path, log dir.
export RLINF_PUSHDOOR_MODEL="${RLINF_PUSHDOOR_MODEL:-$HERE/_ckpt/pi05_pushdoor_tst1}"
export RLINF_PUSHDOOR_DATA="${RLINF_PUSHDOOR_DATA:-/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data_tst1}"
export RLINF_PUSHDOOR_LOG="${RLINF_PUSHDOOR_LOG:-$HERE/_out}"
# Base pi0.5 weights to stage (symlink) into the model dir. Swap for the converted
# r1_pro checkpoint dir if you have one (must contain model.safetensors + config.json).
BASE_CKPT="${BASE_CKPT:-/mnt/r/CKPT/VLA/pi05_base_pt_fp32}"

# --- Stage base weights: symlink model.safetensors + config.json into MODEL_DIR ---
# get_model() loads weights from {model_path}/model.safetensors and quantile norm
# stats from {model_path}/rlinf/pushdoor_open0622/norm_stats.json, so both must live
# under the same dir. Symlink (not copy) the 14GB base weights to save disk.
mkdir -p "${RLINF_PUSHDOOR_MODEL}"
for f in model.safetensors config.json; do
    if [ ! -e "${RLINF_PUSHDOOR_MODEL}/${f}" ]; then
        if [ -e "${BASE_CKPT}/${f}" ]; then
            ln -s "${BASE_CKPT}/${f}" "${RLINF_PUSHDOOR_MODEL}/${f}"
            echo "[run_train] symlinked ${f} <- ${BASE_CKPT}/${f}"
        else
            echo "ERROR: base checkpoint missing ${BASE_CKPT}/${f}"
            echo "       set BASE_CKPT to a dir with model.safetensors + config.json"
            exit 1
        fi
    fi
done

# --- Preflight: norm stats present ---
NORM="${RLINF_PUSHDOOR_MODEL}/rlinf/pushdoor_open0622/norm_stats.json"
if [ ! -f "$NORM" ]; then
    echo "ERROR: norm stats not found at $NORM"
    echo "       Run first: bash examples/au/pi/run_norm_stats.sh"
    exit 1
fi
if [ ! -f "${RLINF_PUSHDOOR_DATA}/meta/episodes.jsonl" ]; then
    echo "ERROR: pushdoor LeRobot dataset not found at ${RLINF_PUSHDOOR_DATA}"
    exit 1
fi

# --- Preflight: GPU memory (warn only) ---
if command -v nvidia-smi >/dev/null 2>&1; then
    USED=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | paste -sd+ | bc 2>/dev/null || echo 0)
    echo "[run_train] total GPU memory used across visible devices: ${USED} MiB"
fi

cd "$REPO"
echo "[run_train] 8-GPU pi0.5 pushdoor(tst1) SFT smoke test: batch 128, 50 steps, save every 25, EMA 0.999"
echo "[run_train] model_path=$RLINF_PUSHDOOR_MODEL  data=$RLINF_PUSHDOOR_DATA  log=$RLINF_PUSHDOOR_LOG"

"$PY" examples/sft/train_vla_sft_au.py \
    --config-path "$HERE" \
    --config-name pushdoor_sft_pi05_au

echo "[run_train] done. checkpoints under $RLINF_PUSHDOOR_LOG/pi05_pushdoor_tst1_smoke50/checkpoints/"
