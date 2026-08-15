#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export REPO_PATH="${REPO_PATH:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
export EMBODIED_PATH="${EMBODIED_PATH:-${REPO_PATH}/examples/embodiment}"
export PYTHONPATH="${REPO_PATH}:${REPO_PATH}/b/x:${PYTHONPATH:-}"
export PYTHONSTARTUP="${REPO_PATH}/b/x/franky_ext/ray_register_startup.py"
export RLINF_EXT_MODULE="${RLINF_EXT_MODULE:-franky_ext.runtime_bootstrap}"
export RLINF_NODE_RANK="${RLINF_NODE_RANK:-0}"
export RLINF_COMM_NET_DEVICES="${RLINF_COMM_NET_DEVICES:-eno2}"
export FRANKA_ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
export FRANKA_NIC="${FRANKA_NIC:-eno1}"
export FRANKA_GRIPPER_TYPE="${FRANKA_GRIPPER_TYPE:-franka}"
if ! command -v switch_env >/dev/null 2>&1; then
  echo "ERROR: switch_env not found. Run inside franky Docker container." >&2
  return 1 2>/dev/null || exit 1
fi
source switch_env franky-0.19.0
cd "${REPO_PATH}"
echo "setup_before_ray_5090: python=$(which python) REPO_PATH=${REPO_PATH}"
