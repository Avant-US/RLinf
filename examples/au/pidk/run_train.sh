#!/usr/bin/env bash
# =============================================================================
# Host-side launcher for Docker-containerized pi0.5 pushdoor SFT.
#
# Single-node (default):
#   bash examples/au/pidk/run_train.sh
#
# Multi-node (run on each physical node, specifying RANK and HEAD_HOST):
#   # Node 0 (head):
#   RANK=0 NUM_NODES=2 bash examples/au/pidk/run_train.sh
#   # Node 1 (worker):
#   RANK=1 NUM_NODES=2 HEAD_HOST=<node0-ip> bash examples/au/pidk/run_train.sh
#
# Docker image: rlinf-aupi-dev:260702 (build with b/gcp/demo4/Dockerfile.aupi_dev)
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"

# ---- Docker image ----
IMAGE="${IMAGE:-rlinf-aupi-dev:260702}"

# ---- Host paths for bind mounts ----
RLINF_DIR="${RLINF_DIR:-$REPO}"
AUPI_DIR="${AUPI_DIR:-/home/physical/SRC/Robot/aupi05}"

# Model / data / output (host paths)
RLINF_PUSHDOOR_MODEL="${RLINF_PUSHDOOR_MODEL:-$HERE/_ckpt/pi05_pushdoor_tst1}"
RLINF_PUSHDOOR_DATA="${RLINF_PUSHDOOR_DATA:-/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data_tst1}"
RLINF_PUSHDOOR_LOG="${RLINF_PUSHDOOR_LOG:-$HERE/_out}"
BASE_CKPT="${BASE_CKPT:-/mnt/r/CKPT/VLA/pi05_base_pt_fp32}"

# ---- Multi-node params (defaults = single-node) ----
RANK="${RANK:-0}"
NUM_NODES="${NUM_NODES:-1}"
HEAD_HOST="${HEAD_HOST:-localhost}"
RAY_PORT="${RAY_PORT:-6379}"

# ---- Stage base weights: symlink model.safetensors + config.json ----
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

# ---- Preflight: norm stats ----
NORM="${RLINF_PUSHDOOR_MODEL}/rlinf/pushdoor_open0622/norm_stats.json"
if [ ! -f "$NORM" ]; then
    echo "ERROR: norm stats not found at $NORM"
    echo "       Run first: bash examples/au/pidk/run_norm_stats.sh"
    exit 1
fi

# ---- Preflight: dataset ----
if [ ! -f "${RLINF_PUSHDOOR_DATA}/meta/episodes.jsonl" ]; then
    echo "ERROR: pushdoor LeRobot dataset not found at ${RLINF_PUSHDOOR_DATA}"
    exit 1
fi

mkdir -p "${RLINF_PUSHDOOR_LOG}"

echo "[run_train] image=${IMAGE}  RANK=${RANK}  NUM_NODES=${NUM_NODES}  HEAD_HOST=${HEAD_HOST}"
echo "[run_train] model=${RLINF_PUSHDOOR_MODEL}  data=${RLINF_PUSHDOOR_DATA}  log=${RLINF_PUSHDOOR_LOG}"

# Container paths (must match Dockerfile.aupi_dev's editable-install paths)
C_RLINF="/workspace/RLinf"
C_AUPI="/workspace/aupi05"
C_MODEL="/workspace/model"
C_DATA="/workspace/data"
C_OUTPUT="/workspace/output"

docker run --rm \
    --gpus all \
    --network=host \
    --shm-size=64g \
    --ulimit memlock=-1 \
    --ulimit stack=67108864 \
    -v "${RLINF_DIR}:${C_RLINF}" \
    -v "${AUPI_DIR}:${C_AUPI}" \
    -v "${RLINF_PUSHDOOR_MODEL}:${C_MODEL}" \
    -v "${RLINF_PUSHDOOR_DATA}:${C_DATA}:ro" \
    -v "${RLINF_PUSHDOOR_LOG}:${C_OUTPUT}" \
    -v /mnt/r:/mnt/r:ro \
    -e RANK="${RANK}" \
    -e NUM_NODES="${NUM_NODES}" \
    -e HEAD_HOST="${HEAD_HOST}" \
    -e RAY_PORT="${RAY_PORT}" \
    -e RLINF_PUSHDOOR_MODEL="${C_MODEL}" \
    -e RLINF_PUSHDOOR_DATA="${C_DATA}" \
    -e RLINF_PUSHDOOR_LOG="${C_OUTPUT}" \
    "${IMAGE}" \
    bash -l "${C_RLINF}/examples/au/pidk/bootstrap.sh"
