#!/usr/bin/env bash
# Compute pushdoor (tst1) normalization stats inside Docker container.
# Reuses examples/au/pi/compute_norm_stats_au.py (no copy needed).
# Writes {MODEL_DIR}/rlinf/pushdoor_open0622/norm_stats.json.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"

IMAGE="${IMAGE:-rlinf-aupi-dev:260702}"
RLINF_DIR="${RLINF_DIR:-$REPO}"
AUPI_DIR="${AUPI_DIR:-/home/physical/SRC/Robot/aupi05}"
MODEL_DIR="${RLINF_PUSHDOOR_MODEL:-$HERE/_ckpt/pi05_pushdoor_tst1}"
DATA_DIR="${RLINF_PUSHDOOR_DATA:-/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data_tst1}"
MAX_FRAMES="${MAX_FRAMES:-1024}"
NUM_WORKERS="${NUM_WORKERS:-16}"
BATCH_SIZE="${BATCH_SIZE:-64}"

# Stage base weights if needed (config.json is required by compute_norm_stats)
BASE_CKPT="${BASE_CKPT:-/mnt/r/CKPT/VLA/pi05_base_pt_fp32}"
mkdir -p "${MODEL_DIR}"
for f in model.safetensors config.json; do
    if [ ! -e "${MODEL_DIR}/${f}" ]; then
        if [ -e "${BASE_CKPT}/${f}" ]; then
            ln -s "${BASE_CKPT}/${f}" "${MODEL_DIR}/${f}"
            echo "[norm_stats] symlinked ${f} <- ${BASE_CKPT}/${f}"
        fi
    fi
done

C_RLINF="/workspace/RLinf"
C_AUPI="/workspace/aupi05"
C_MODEL="/workspace/model"
C_DATA="/workspace/data"

echo "[norm_stats] image=${IMAGE}"
echo "[norm_stats] model_dir=${MODEL_DIR}  data=${DATA_DIR}  max_frames=${MAX_FRAMES}"

docker run --rm \
    --gpus '"device=0"' \
    --network=host \
    --shm-size=16g \
    -v "${RLINF_DIR}:${C_RLINF}" \
    -v "${AUPI_DIR}:${C_AUPI}" \
    -v "${MODEL_DIR}:${C_MODEL}" \
    -v "${DATA_DIR}:${C_DATA}:ro" \
    -v /mnt/r:/mnt/r:ro \
    -e CUDA_VISIBLE_DEVICES=0 \
    -e PYTHONUNBUFFERED=1 \
    -e HF_HUB_OFFLINE=1 \
    -e HF_DATASETS_OFFLINE=1 \
    -e USE_TF=0 \
    -e USE_FLAX=0 \
    "${IMAGE}" \
    bash -lc "
      source /venv/rlinf/bin/activate
      export PYTHONPATH=${C_RLINF}:\${PYTHONPATH:-}
      cd ${C_RLINF}
      python -u examples/au/pi/compute_norm_stats_au.py \
        --config_name pi05_pushdoor \
        --repo_id ${C_DATA} \
        --output_dir ${C_MODEL} \
        --max_frames ${MAX_FRAMES} \
        --batch_size ${BATCH_SIZE} \
        --num_workers ${NUM_WORKERS}
    "

echo "[norm_stats] done -> ${MODEL_DIR}/rlinf/pushdoor_open0622/norm_stats.json"
