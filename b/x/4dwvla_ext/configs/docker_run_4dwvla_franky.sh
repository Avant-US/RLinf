#!/bin/bash
# Start the Franky container for robot control during 4DWVLA evaluation.
# Based on docker_run_franky_5090.sh.
set -euo pipefail

RLINF_REPO="${RLINF_REPO:-/home/nvidia/bt/s/RLmm}"
IMAGE="${RLINF_FRANKA_IMAGE:-rlinf/rlinf:agentic-rlinf0.4-franka}"
NAME="${CONTAINER_NAME:-rlinf-4dwvla-franky}"
SHM="${SHM_SIZE:-10g}"

if [[ ! -d "${RLINF_REPO}" ]]; then
    echo "ERROR: RLINF_REPO not found: ${RLINF_REPO}" >&2
    exit 1
fi

# Check for FCI conflicts
ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
if command -v ss >/dev/null 2>&1; then
    if ss -tn state established "( dport = :1337 or sport = :1337 )" 2>/dev/null \
        | grep -q "${ROBOT_IP}"; then
        echo "ERROR: something already holds FCI on ${ROBOT_IP}:1337." >&2
        echo "       Stop it first, then re-run." >&2
        exit 1
    fi
fi

exec docker run -it --rm --privileged --network host \
    --name "${NAME}" \
    --shm-size="${SHM}" \
    -v "${RLINF_REPO}:/workspace/RLinf" \
    -w /workspace/RLinf \
    "${IMAGE}" bash
