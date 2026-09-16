#!/bin/bash
# Launch RLT Stage 1 training in Docker container
# Usage: bash b/x/4dwvla_ext/rlt/docker_run_rlt_stage1.sh [config_path]
set -euo pipefail

RLINF_REPO="${RLINF_REPO:-/home/nvidia/bt/s/RLmm}"
WVLA_REPO="${WVLA_REPO:-/home/nvidia/bt/s/4WVLA}"
CKPT_DIR="${CKPT_DIR:-/home/nvidia/bt/ckp}"
DATA_DIR="${DATA_DIR:-/home/nvidia/bt/dt}"
HF_CACHE="${HF_CACHE:-$HOME/.cache/huggingface}"
RLINF_GPU_IMAGE="${RLINF_GPU_IMAGE:-rlinf/rlinf:agentic-rlinf0.4-maniskill_libero}"
CONTAINER_NAME="${CONTAINER_NAME:-rlinf-4dwvla-rlt-stage1}"
CONFIG_PATH="${1:-b/x/4dwvla_ext/rlt/configs/rlt_stage1_franka_plug.yaml}"

echo "=== Launching RLT Stage 1 Training Container ==="
echo "Image: ${RLINF_GPU_IMAGE}"
echo "Config: ${CONFIG_PATH}"

docker stop "${CONTAINER_NAME}" 2>/dev/null || true

docker run -it \
  --gpus all --privileged --network host --shm-size=20g \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e HF_HOME=/home/nvidia/.cache/huggingface \
  -e DATA_DIR=/home/nvidia/data \
  -e CKPT_DIR=/home/nvidia/ckpts \
  -v "${RLINF_REPO}":/workspace/RLinf \
  -v "${WVLA_REPO}":/workspace/4WVLA:ro \
  -v "${CKPT_DIR}":/home/nvidia/ckpts:ro \
  -v "${DATA_DIR}":/home/nvidia/data:ro \
  -v "${HF_CACHE}":/home/nvidia/.cache/huggingface \
  --name "${CONTAINER_NAME}" \
  "${RLINF_GPU_IMAGE}" \
  bash -c "bash /workspace/RLinf/b/x/4dwvla_ext/rlt/launch_rlt_stage1.sh /workspace/RLinf/${CONFIG_PATH}"

echo ""
echo "Container '${CONTAINER_NAME}' is stopped but NOT removed."
echo "To export as image: docker commit ${CONTAINER_NAME} ${RLINF_GPU_IMAGE}-rlt-stage1"
echo "To restart:         docker start -ai ${CONTAINER_NAME}"
echo "To remove manually: docker rm ${CONTAINER_NAME}"
