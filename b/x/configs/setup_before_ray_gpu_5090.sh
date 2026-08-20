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

# Motion authority. Exported here too because this script is sourced before
# `ray start --head` on the training path, and a Ray actor inherits the raylet's
# environment, not the driver's -- so exporting these after `ray start` is silently
# ignored. Kept identical to setup_before_ray_5090.sh; see the comment there.
# Changing any of them requires: ray stop -> re-source -> ray start.
export RLINF_CUBE_FORCE_CEILING_N="${RLINF_CUBE_FORCE_CEILING_N:-20}"
export RLINF_CUBE_FORCE_NORM_CEILING_N="${RLINF_CUBE_FORCE_NORM_CEILING_N:-40}"
export RLINF_CUBE_TORQUE_CEILING_NM="${RLINF_CUBE_TORQUE_CEILING_NM:-6}"
export RLINF_CUBE_TORQUE_NORM_CEILING_NM="${RLINF_CUBE_TORQUE_NORM_CEILING_NM:-12}"
export RLINF_CUBE_INTERP_SPEED="${RLINF_CUBE_INTERP_SPEED:-0.02}"
export RLINF_CUBE_INTERP_SPEED_RAD="${RLINF_CUBE_INTERP_SPEED_RAD:-0.15}"
# The POLICY-path cap -- round-2 audit finding 15: missing here (and in the CPU
# script) meant an operator following the manual had no way to set the one knob
# that bounds the phase-3 policy for a training run.
export RLINF_CUBE_STEP_SPEED="${RLINF_CUBE_STEP_SPEED:-0.05}"
export RLINF_CUBE_GUARD_MARGIN="${RLINF_CUBE_GUARD_MARGIN:-0.05}"
export RLINF_CUBE_GUARD_FLOOR_MARGIN="${RLINF_CUBE_GUARD_FLOOR_MARGIN:-0.01}"
export RLINF_CUBE_GUARD_MAX_LAG="${RLINF_CUBE_GUARD_MAX_LAG:-0.05}"
export RLINF_CUBE_GUARD_ORIENT_SLACK="${RLINF_CUBE_GUARD_ORIENT_SLACK:-0.20}"
export RLINF_CUBE_Z_CEILING="${RLINF_CUBE_Z_CEILING:-}"
export FRANKA_GRASP_FORCE="${FRANKA_GRASP_FORCE:-20}"
export FRANKA_CUBE_WIDTH_M="${FRANKA_CUBE_WIDTH_M:-0.046}"
export FRANKA_HOLD_TOL_M="${FRANKA_HOLD_TOL_M:-0.012}"

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
echo "setup_before_ray_gpu_5090: authority force=${RLINF_CUBE_FORCE_CEILING_N}N/axis" \
     "(norm<=${RLINF_CUBE_FORCE_NORM_CEILING_N}N)" \
     "torque=${RLINF_CUBE_TORQUE_CEILING_NM}Nm/axis" \
     "interp=${RLINF_CUBE_INTERP_SPEED}m/s step=${RLINF_CUBE_STEP_SPEED}m/s" \
     "guard=${RLINF_CUBE_GUARD_MARGIN}m/floor ${RLINF_CUBE_GUARD_FLOOR_MARGIN}m/lag ${RLINF_CUBE_GUARD_MAX_LAG}m"
echo "setup_before_ray_gpu_5090: gripper force=${FRANKA_GRASP_FORCE}N cube_width=${FRANKA_CUBE_WIDTH_M}m +/-${FRANKA_HOLD_TOL_M}m"
