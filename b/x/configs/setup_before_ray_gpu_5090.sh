#!/bin/bash
# Step 7b GPU container env. Do NOT source setup_before_ray_5090.sh (that switches franky + CPU torch).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REPO_PATH="${REPO_PATH:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
export EMBODIED_PATH="${EMBODIED_PATH:-${REPO_PATH}/examples/embodiment}"
export PYTHONPATH="${REPO_PATH}:${REPO_PATH}/b/x:${PYTHONPATH:-}"
export PYTHONSTARTUP="${REPO_PATH}/b/x/franky_ext/ray_register_startup.py"
export RLINF_EXT_MODULE="${RLINF_EXT_MODULE:-franky_ext.runtime_bootstrap}"
export RLINF_NODE_RANK="${RLINF_NODE_RANK:-0}"
export RLINF_COMM_NET_DEVICES="${RLINF_COMM_NET_DEVICES:-eno2}"
export RLINF_SKIP_CAMERA="${RLINF_SKIP_CAMERA:-1}"
export RLINF_RESNET10_PATH="${RLINF_RESNET10_PATH:-/home/nvidia/ckpts/RLinf-ResNet10-pretrained}"

if command -v switch_env >/dev/null 2>&1; then
  if [ -d /opt/venv/openvla ]; then
    # shellcheck source=/dev/null
    source switch_env openvla
  elif [ -d /opt/venv/openvla-oft ]; then
    # shellcheck source=/dev/null
    source switch_env openvla-oft
  elif [ -d /opt/venv/openpi ]; then
    # shellcheck source=/dev/null
    source switch_env openpi
  fi
fi

cd "${REPO_PATH}"
echo "setup_before_ray_gpu_5090: python=$(which python) REPO_PATH=${REPO_PATH}"
