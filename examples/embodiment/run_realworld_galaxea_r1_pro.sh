#!/bin/bash
# Async training entry for Galaxea R1 Pro real-world RL.
# Run only on the head node (GPU server, RLINF_NODE_RANK=0).

export EMBODIED_PATH="$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd )"
export REPO_PATH=$(dirname $(dirname "${EMBODIED_PATH}"))
export SRC_FILE="${EMBODIED_PATH}/train_async.py"

if [ -z "${1:-}" ]; then
    CONFIG_NAME="realworld_galaxea_r1_pro_right_arm_rlpd_cnn_async"
else
    CONFIG_NAME="$1"
fi

export ROBOT_PLATFORM="galaxea_r1_pro"
export MUJOCO_GL="${MUJOCO_GL:-egl}"

LOG_DIR="${REPO_PATH}/logs/$(date +'%Y%m%d-%H%M%S')-${CONFIG_NAME}"
MEGA_LOG_FILE="${LOG_DIR}/run_galaxea_r1_pro.log"
mkdir -p "${LOG_DIR}"
echo "Using Python at $(which python)"
CMD="python ${SRC_FILE} --config-path ${EMBODIED_PATH}/config/ --config-name ${CONFIG_NAME} runner.logger.log_path=${LOG_DIR}"
echo "${CMD}" | tee "${MEGA_LOG_FILE}"
${CMD} 2>&1 | tee -a "${MEGA_LOG_FILE}"
