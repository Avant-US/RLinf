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

# --- Motion authority (b/x/franky_ext/motion_limits.py) -----------------------
# These MUST be exported before `ray start`, for the same reason as
# RLINF_COMM_NET_DEVICES: a Ray actor inherits the environment of the raylet, not
# of the driver. On the phase-2 smoke path the script's own os.environ writes reach
# the actor because ray.init() starts the cluster as a child, but on the training
# path (`ray start --head` first) they do not -- so an operator exporting
# RLINF_CUBE_GUARD_MARGIN in their shell after `ray start` would be silently
# ignored while believing the fence had been tightened.
#
# Changing any of these requires: ray stop -> re-source this script -> ray start.
# Every value is clamped to a hard range inside motion_limits, so a typo cannot
# recreate the LOG-019 authority; the tracker start log echoes the resulting
# products so "did it take effect" is answerable from the log.
export RLINF_CUBE_FORCE_CEILING_N="${RLINF_CUBE_FORCE_CEILING_N:-20}"
export RLINF_CUBE_FORCE_NORM_CEILING_N="${RLINF_CUBE_FORCE_NORM_CEILING_N:-40}"
export RLINF_CUBE_TORQUE_CEILING_NM="${RLINF_CUBE_TORQUE_CEILING_NM:-6}"
export RLINF_CUBE_TORQUE_NORM_CEILING_NM="${RLINF_CUBE_TORQUE_NORM_CEILING_NM:-12}"
export RLINF_CUBE_INTERP_SPEED="${RLINF_CUBE_INTERP_SPEED:-0.02}"
export RLINF_CUBE_INTERP_SPEED_RAD="${RLINF_CUBE_INTERP_SPEED_RAD:-0.15}"
# The POLICY-path cap (step() does not go through _interpolate_move, so this is a
# separate knob -- round-2 audit finding 15 caught it missing from both setup
# scripts, i.e. an operator following the manual had no way to set it for a
# phase-3 run, and on the training path a shell export after `ray start` is
# silently ignored, which is exactly the failure this whole block exists to avoid).
export RLINF_CUBE_STEP_SPEED="${RLINF_CUBE_STEP_SPEED:-0.05}"
export RLINF_CUBE_GUARD_MARGIN="${RLINF_CUBE_GUARD_MARGIN:-0.05}"
export RLINF_CUBE_GUARD_FLOOR_MARGIN="${RLINF_CUBE_GUARD_FLOOR_MARGIN:-0.01}"
export RLINF_CUBE_GUARD_MAX_LAG="${RLINF_CUBE_GUARD_MAX_LAG:-0.05}"
export RLINF_CUBE_GUARD_ORIENT_SLACK="${RLINF_CUBE_GUARD_ORIENT_SLACK:-0.20}"
# Absolute +z fence ceiling, box-relative (box_top .. box_top+0.40); see
# franky_single_franka_env.ABS_Z_CEILING_ABOVE_BOX_M for the default offset.
export RLINF_CUBE_Z_CEILING="${RLINF_CUBE_Z_CEILING:-}"
# Gripper: hold force, the calibrated cube width the "am I holding it" window is
# centred on, and that window's half-width. Re-measure and update the width when
# the cube changes.
export FRANKA_GRASP_FORCE="${FRANKA_GRASP_FORCE:-20}"
export FRANKA_CUBE_WIDTH_M="${FRANKA_CUBE_WIDTH_M:-0.046}"
export FRANKA_HOLD_TOL_M="${FRANKA_HOLD_TOL_M:-0.012}"
if ! command -v switch_env >/dev/null 2>&1; then
  echo "ERROR: switch_env not found. Run inside franky Docker container." >&2
  return 1 2>/dev/null || exit 1
fi
source switch_env franky-0.19.0
cd "${REPO_PATH}"
echo "setup_before_ray_5090: python=$(which python) REPO_PATH=${REPO_PATH}"
echo "setup_before_ray_5090: authority force=${RLINF_CUBE_FORCE_CEILING_N}N/axis" \
     "(norm<=${RLINF_CUBE_FORCE_NORM_CEILING_N}N)" \
     "torque=${RLINF_CUBE_TORQUE_CEILING_NM}Nm/axis" \
     "interp=${RLINF_CUBE_INTERP_SPEED}m/s step=${RLINF_CUBE_STEP_SPEED}m/s" \
     "guard=${RLINF_CUBE_GUARD_MARGIN}m/floor ${RLINF_CUBE_GUARD_FLOOR_MARGIN}m/lag ${RLINF_CUBE_GUARD_MAX_LAG}m"
echo "setup_before_ray_5090: gripper force=${FRANKA_GRASP_FORCE}N cube_width=${FRANKA_CUBE_WIDTH_M}m +/-${FRANKA_HOLD_TOL_M}m"
