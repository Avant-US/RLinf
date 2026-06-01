#!/usr/bin/env bash
set -euo pipefail

export TF_ENABLE_ONEDNN_OPTS=0

export EMBODIED_PATH="$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd )"
export REPO_PATH=$(dirname $(dirname "$EMBODIED_PATH"))
export SRC_FILE="${EMBODIED_PATH}/train_vla_sft.py"

export BTLOG_ROOT=${BTLOG_ROOT:-"/mnt/r/CKPT/VLA/FW/RUN"}
export FASTWAM_ROOT=${FASTWAM_ROOT:-"/home/Luogang/SRC/Robot/FastWAM"}
export FASTWAM_PATH=${FASTWAM_PATH:-"${FASTWAM_ROOT}/src"}
export DIFFSYNTH_MODEL_BASE_PATH=${DIFFSYNTH_MODEL_BASE_PATH:-"/mnt/r/share/fastwam_checkpoints"}
export CUDA_HOME=${CUDA_HOME:-"/usr/local/cuda-12.8"}

export PYTHONPATH=${REPO_PATH}:${FASTWAM_PATH}:${PYTHONPATH:-}

if [ -z "${1:-}" ]; then
    CONFIG_NAME="libero_sft_fastwam"
else
    CONFIG_NAME=$1
fi

echo "Using Python at $(which python)"
echo "FastWAM root: ${FASTWAM_ROOT}"
echo "Config: ${CONFIG_NAME}"

LOG_DIR="${BTLOG_ROOT}/logs/$(date +'%Y%m%d-%H:%M:%S')-${CONFIG_NAME}"
MEGA_LOG_FILE="${LOG_DIR}/run_fastwam_sft.log"
mkdir -p "${LOG_DIR}"

shift || true
CMD="python ${SRC_FILE} --config-path ${EMBODIED_PATH}/config/ --config-name ${CONFIG_NAME} runner.logger.log_path=${LOG_DIR}"
echo "CMD: ${CMD} $*" | tee ${MEGA_LOG_FILE}
${CMD} "$@" 2>&1 | tee -a ${MEGA_LOG_FILE}
