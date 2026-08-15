#!/bin/bash
# Step 7: dummy SAC async smoke — mirrors examples/embodiment/run_realworld_async.sh
set -euo pipefail

RUN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${RUN_SCRIPT_DIR}/../configs/setup_before_ray_5090.sh"
bash "${RUN_SCRIPT_DIR}/step7_install_deps.sh"

export EMBODIED_PATH="${REPO_PATH}/examples/embodiment"
export CONFIG_PATH="${REPO_PATH}/b/x/configs"
CONFIG_NAME="${1:-realworld_franky_dummy_sac}"

MODEL_DIR="${RLINF_RESNET10_PATH:-/home/nvidia/ckpts/RLinf-ResNet10-pretrained}"
export RLINF_RESNET10_PATH="${MODEL_DIR}"

python -c "import franky_ext.runtime_bootstrap"  # noqa: CPU shim + Gym register

if [[ ! -f "${MODEL_DIR}/resnet10_pretrained.pt" ]]; then
  echo "ERROR: missing ${MODEL_DIR}/resnet10_pretrained.pt" >&2
  echo "Download once (container or host):" >&2
  echo "  mkdir -p ${MODEL_DIR}" >&2
  echo "  hf download RLinf/RLinf-ResNet10-pretrained --local-dir ${MODEL_DIR}" >&2
  echo "Or: git clone https://huggingface.co/RLinf/RLinf-ResNet10-pretrained ${MODEL_DIR}" >&2
  exit 1
fi

if ! ray status >/dev/null 2>&1; then
  ray stop --force 2>/dev/null || true
  ray start --head --port=6379
fi

LOG_DIR="${REPO_PATH}/logs/$(date +'%Y%m%d-%H%M%S')-${CONFIG_NAME}"
MEGA_LOG_FILE="${LOG_DIR}/run_embodiment.log"
mkdir -p "${LOG_DIR}"

SRC_FILE="${EMBODIED_PATH}/train_async.py"
CMD="python ${SRC_FILE} --config-path ${CONFIG_PATH} --config-name ${CONFIG_NAME} runner.logger.log_path=${LOG_DIR}"
echo "${CMD}" | tee "${MEGA_LOG_FILE}"
${CMD} 2>&1 | tee -a "${MEGA_LOG_FILE}"
