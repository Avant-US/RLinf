#!/bin/bash
# Step 7b: dummy SAC on GPU. Do NOT call step7_install_deps.sh (that force-installs CPU torch).
set -euo pipefail

RUN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${RUN_SCRIPT_DIR}/../configs/setup_before_ray_gpu_5090.sh"

export EMBODIED_PATH="${REPO_PATH}/examples/embodiment"
export CONFIG_PATH="${REPO_PATH}/b/x/configs"
CONFIG_NAME="${1:-realworld_franky_dummy_sac_gpu}"

MODEL_DIR="${RLINF_RESNET10_PATH:-/home/nvidia/ckpts/RLinf-ResNet10-pretrained}"
export RLINF_RESNET10_PATH="${MODEL_DIR}"

if [[ ! -f "${MODEL_DIR}/resnet10_pretrained.pt" ]]; then
  echo "ERROR: missing ${MODEL_DIR}/resnet10_pretrained.pt" >&2
  exit 1
fi

if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi >/dev/null 2>&1; then
  echo "ERROR: nvidia-smi failed inside this container. Use --gpus all and a CUDA image." >&2
  exit 1
fi

python - <<'PY'
import torch
assert torch.cuda.is_available(), (
    f"torch.cuda.is_available() is False (torch={torch.__version__}). "
    "Do not use franky venv / step7_install_deps.sh CPU wheels."
)
print(f"CUDA OK: torch={torch.__version__} device={torch.cuda.get_device_name(0)}")
PY

python -c "import franky_ext.tasks.register; print('Gym Franky* registered')"

if ray status >/dev/null 2>&1; then
  echo "WARNING: an existing Ray cluster is up (often leftover from franky 7a). Stopping it."
  ray stop --force || true
  sleep 2
fi
ray start --head --port=6379 --disable-usage-stats

LOG_DIR="${REPO_PATH}/logs/$(date +'%Y%m%d-%H%M%S')-${CONFIG_NAME}"
MEGA_LOG_FILE="${LOG_DIR}/run_embodiment.log"
mkdir -p "${LOG_DIR}"

SRC_FILE="${EMBODIED_PATH}/train_async.py"
CMD="python ${SRC_FILE} --config-path ${CONFIG_PATH} --config-name ${CONFIG_NAME} runner.logger.log_path=${LOG_DIR}"
echo "${CMD}" | tee "${MEGA_LOG_FILE}"
${CMD} 2>&1 | tee -a "${MEGA_LOG_FILE}"
