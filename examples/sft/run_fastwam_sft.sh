#!/usr/bin/env bash
set -euo pipefail

export EMBODIED_PATH="$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd )"
export REPO_PATH=$(dirname $(dirname "$EMBODIED_PATH"))
export SRC_FILE="${EMBODIED_PATH}/train_vla_sft.py"

export FASTWAM_ROOT=${FASTWAM_ROOT:-"/home/luogang/S/Rb/FastWAM"}
export FASTWAM_PATH=${FASTWAM_PATH:-"${FASTWAM_ROOT}/src"}
export DIFFSYNTH_MODEL_BASE_PATH=${DIFFSYNTH_MODEL_BASE_PATH:-"/mnt/localssd/share/fastwam_checkpoints"}
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

LOG_DIR="${REPO_PATH}/logs/$(date +'%Y%m%d-%H:%M:%S')-${CONFIG_NAME}"
MEGA_LOG_FILE="${LOG_DIR}/run_fastwam_sft.log"
mkdir -p "${LOG_DIR}"

shift || true
CMD="python ${SRC_FILE} --config-path ${EMBODIED_PATH}/config/ --config-name ${CONFIG_NAME} runner.logger.log_path=${LOG_DIR}"
echo "CMD: ${CMD} $*" | tee ${MEGA_LOG_FILE}
${CMD} "$@" 2>&1 | tee -a ${MEGA_LOG_FILE}
