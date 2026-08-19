#!/bin/bash
# Step 7b: GPU training container (CUDA). Do NOT use docker_run_franky_5090.sh here.
set -euo pipefail
REPO="${REPO:-/home/nvidia/bt/s/RLinf}"
IMAGE="${RLINF_GPU_IMAGE:-rlinf/rlinf:agentic-rlinf0.4-maniskill_libero}"
NAME="${RLINF_GPU_CONTAINER:-rlinf-gpu-5090}"
CKPT="${RLINF_RESNET10_PATH:-/home/nvidia/ckpts/RLinf-ResNet10-pretrained}"

exec docker run -it --rm --gpus all \
  --privileged \
  --network host \
  --shm-size=20g \
  --name "${NAME}" \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e RLINF_RESNET10_PATH="${CKPT}" \
  -e RLINF_SKIP_CAMERA=1 \
  -v "${REPO}:/workspace/RLinf" \
  -v /home/nvidia/ckpts:/home/nvidia/ckpts:ro \
  -w /workspace/RLinf \
  "${IMAGE}" bash
