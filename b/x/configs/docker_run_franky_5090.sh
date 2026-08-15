#!/bin/bash
REPO="${REPO:-/home/nvidia/bt/s/RLinf}"
IMAGE="${RLINF_FRANKA_IMAGE:-rlinf/rlinf:agentic-rlinf0.4-franka}"
NAME="${RLINF_FRANKA_CONTAINER:-rlinf-franky-5090}"
exec docker run -it --rm --privileged --network host --name "${NAME}" \
  -v "${REPO}:/workspace/RLinf" -w /workspace/RLinf "${IMAGE}" bash
