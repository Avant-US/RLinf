#!/usr/bin/env bash
# Train RLinf openpi_au pi0.5 on the pushdoor (r1_pro) dataset across 8 GPUs
# (global batch 128, 30-step smoke test, checkpoint every 15). Same optimizer/
# precision/EMA recipe as the LIBERO reference (tests_au/example/libero). Base
# weights default to the converted pi05_r1pro_chassis_alig_newnorm checkpoint
# (see convert_r1pro_ckpt.py) rather than vanilla pi05_base_pt.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"

PY="${PY:-/mnt/r/VENV/openpi_venv/bin/python}"
OPENPI_SRC="${OPENPI_SRC:-/home/physical/SRC/Robot/openpi05/src}"

# Hydra group defaults (model/pi0_5_au, training_backend/fsdp) resolve via this dir.
export EMBODIED_PATH="${REPO}/examples/sft"
export PYTHONPATH="${REPO}:${OPENPI_SRC}:${PYTHONPATH:-}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0,1,2,3,4,5,6,7}"

# This box is shared with other unrelated jobs, one of which already runs its own
# long-lived Ray cluster (discoverable at /tmp/ray/ray_current_cluster). RLinf's
# Cluster._init_and_launch_managers() calls ray.init(address="auto", ...); per Ray's
# own resolution rules, an "auto" address is silently REPLACED by the RAY_ADDRESS env
# var when set, and address="local" means "always start a fresh local Ray instance,
# even if another one is already running" -- so this line makes our smoke test spin
# up its own throwaway Ray cluster instead of piggybacking on (and disrupting) the
# other job's, with zero changes to RLinf's core scheduler code.
export RAY_ADDRESS="${RAY_ADDRESS:-local}"

# SFT trains on an offline dataset; EGL only needed if any env import touches GL.
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-egl}"

# Use the local LeRobot cache only -> avoids HF 429 when 8 ranks load data at once.
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_DATASETS_OFFLINE="${HF_DATASETS_OFFLINE:-1}"

# Example model dir (weights symlink + recomputed norm stats), dataset path, and log dir.
export RLINF_PUSHDOOR_MODEL="${RLINF_PUSHDOOR_MODEL:-$HERE/_ckpt/pi05_r1pro_pt}"
export RLINF_PUSHDOOR_DATA="${RLINF_PUSHDOOR_DATA:-/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data}"
export RLINF_PUSHDOOR_LOG="${RLINF_PUSHDOOR_LOG:-$HERE/_out}"

# --- Preflight: norm stats present ---
NORM="${RLINF_PUSHDOOR_MODEL}/rlinf/pushdoor_open0622/norm_stats.json"
if [ ! -f "$NORM" ]; then
    echo "ERROR: norm stats not found at $NORM"
    echo "       Run: bash tests_au/example/pushdoor/run_norm_stats.sh"
    exit 1
fi
if [ ! -e "${RLINF_PUSHDOOR_MODEL}/model.safetensors" ]; then
    echo "ERROR: model weights not found at ${RLINF_PUSHDOOR_MODEL}/model.safetensors"
    echo "       symlink model.safetensors + config.json (e.g. from /mnt/r/CKPT/VLA/pi05_base_pt_fp32/)"
    echo "       into ${RLINF_PUSHDOOR_MODEL}/"
    exit 1
fi
if [ ! -f "${RLINF_PUSHDOOR_DATA}/meta/episodes.jsonl" ]; then
    echo "ERROR: pushdoor LeRobot dataset not found at ${RLINF_PUSHDOOR_DATA}"
    exit 1
fi

# --- Preflight: GPUs free (warn only) ---
if command -v nvidia-smi >/dev/null 2>&1; then
    USED=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | paste -sd+ | bc 2>/dev/null || echo 0)
    echo "[run_train] total GPU memory used across visible devices: ${USED} MiB"
fi

cd "$REPO"
echo "[run_train] 8-GPU pi0.5 pushdoor SFT smoke test: batch 128, 30 steps, save every 15, EMA 0.999"
echo "[run_train] model_path=$RLINF_PUSHDOOR_MODEL  data=$RLINF_PUSHDOOR_DATA  log=$RLINF_PUSHDOOR_LOG"

"$PY" examples/sft/train_vla_sft_au.py \
    --config-path "$HERE" \
    --config-name pushdoor_sft_pi05_au_8gpu

echo "[run_train] done. checkpoints under $RLINF_PUSHDOOR_LOG/pi05_pushdoor_8gpu_smoke30/checkpoints/"
