#!/bin/bash
# Start the GPU container for 4DWVLA inference.
# Extends docker_run_gpu_5090.sh with 4WVLA code mount and checkpoint mount.
set -euo pipefail

RLINF_REPO="${RLINF_REPO:-/home/nvidia/bt/s/RLmm}"
WVLA_REPO="${WVLA_REPO:-/home/nvidia/bt/s/4WVLA}"
CKPT_DIR="${CKPT_DIR:-/home/nvidia/bt/ckp}"
HF_CACHE="${HF_CACHE:-${HOME}/.cache/huggingface}"
IMAGE="${RLINF_GPU_IMAGE:-rlinf/rlinf:agentic-rlinf0.4-maniskill_libero}"
NAME="${CONTAINER_NAME:-rlinf-4dwvla-gpu}"

for d in "${RLINF_REPO}" "${WVLA_REPO}" "${CKPT_DIR}"; do
    if [[ ! -d "${d}" ]]; then
        echo "ERROR: directory not found: ${d}" >&2
        exit 1
    fi
done

exec docker run -it --rm --gpus all \
    --privileged \
    --network host \
    --shm-size=20g \
    --name "${NAME}" \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e HF_HOME=/home/nvidia/.cache/huggingface \
    -v "${RLINF_REPO}:/workspace/RLinf" \
    -v "${WVLA_REPO}:/workspace/4WVLA:ro" \
    -v "${CKPT_DIR}:/home/nvidia/ckpts:ro" \
    -v "${HF_CACHE}:/home/nvidia/.cache/huggingface" \
    -w /workspace/RLinf \
    "${IMAGE}" bash
