#!/bin/bash
# Phase 3: real-robot online SAC for FrankyCubePlaceEnv-v1.
#
# Run this ONLY in the GPU container (rank 0, Ray head). Unlike
# run_cube_place_dummy_sac_gpu.sh, this script does NOT `ray start`: the
# two-node cluster (GPU container = head/rank0, franky container = rank1)
# must already be up, started manually in the order documented in
# dmo_place_2.md S3.2. This script only asserts that cluster exists and has
# both nodes alive before launching the driver.
#
# See dmo_place_2.md "阶段 3" (S3.0-S3.10) for the full walkthrough: camera
# acceptance (S3.3), pre-flight (S3.4), what to watch in the first five
# minutes (S3.5), and known pitfalls (S3.10).
set -euo pipefail

RUN_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${RUN_SCRIPT_DIR}/../configs/setup_before_ray_gpu_5090.sh"

# setup_before_ray_gpu_5090.sh defaults RLINF_SKIP_CAMERA to 1, and this
# source happens AFTER the franky-side `ray start` (dmo_place_2.md S3.2's
# sequencing). Per S3.10 point 1, the YAML's env_configs.env_vars for the
# franky node group wins regardless (it has the highest precedence), so this
# export is only a second, belt-and-suspenders layer -- do not rely on it
# alone; the authoritative switch lives in the YAML.
export RLINF_SKIP_CAMERA=0

export EMBODIED_PATH="${REPO_PATH}/examples/embodiment"
export CONFIG_PATH="${REPO_PATH}/b/x/configs"
CONFIG_NAME="${1:-realworld_cube_place_sac}"
shift || true

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

python -c "import franky_ext.tasks.register; import gymnasium as gym; print('gym', gym.spec('FrankyCubePlaceEnv-v1').id)"

# The cluster must already be a two-node cluster: rank0 = this container
# (head), rank1 = franky container. See dmo_place_2.md S3.2 for the startup
# order -- this script refuses to start it itself because franky_ext's
# controller and camera must come up under the franky container's own
# environment (RLINF_NODE_RANK=1, franky venv), which this container cannot
# provide.
if ! ray status >/dev/null 2>&1; then
  echo "ERROR: no Ray cluster reachable. Start both nodes first (dmo_place_2.md S3.2)." >&2
  exit 1
fi

NODES="$(python - <<'PY'
import ray

ray.init(address="auto", logging_level="ERROR")
print(sum(1 for n in ray.nodes() if n["Alive"]))
PY
)"
if [[ "${NODES}" != "2" ]]; then
  echo "ERROR: expected 2 alive Ray nodes, got ${NODES}. Start the franky-container node too (S3.2)." >&2
  exit 1
fi

# Same reasoning as run_cube_place_dummy_sac_gpu.sh: an ALIVE
# FrankyControllerExtended actor may be mid-motion under a 1 kHz torque
# controller. Killing the process group out from under it (e.g. by letting
# this script proceed and later Ctrl+C-ing train_async.py badly) hands the
# arm to libfranka's comms-timeout stop instead of the guided deceleration
# freeze_at_current() provides. Refuse rather than warn-and-proceed.
if ray list actors --filter "class_name=FrankyControllerExtended" \
  --filter "state=ALIVE" 2>/dev/null | grep -q ALIVE; then
  echo "ERROR: a live FrankyControllerExtended actor already holds the robot." >&2
  echo "       It may be mid-motion from a prior run. Confirm the arm is safe," >&2
  echo "       stop that process cleanly, then re-run." >&2
  exit 1
fi

LOG_DIR="${REPO_PATH}/logs/$(date +'%Y%m%d-%H%M%S')-${CONFIG_NAME}"
MEGA_LOG_FILE="${LOG_DIR}/run_embodiment.log"
mkdir -p "${LOG_DIR}"

SRC_FILE="${EMBODIED_PATH}/train_async.py"
CMD="python ${SRC_FILE} --config-path ${CONFIG_PATH} --config-name ${CONFIG_NAME} runner.logger.log_path=${LOG_DIR}"
if [[ $# -gt 0 ]]; then
  CMD="${CMD} $*"
fi
echo "${CMD}" | tee "${MEGA_LOG_FILE}"
${CMD} 2>&1 | tee -a "${MEGA_LOG_FILE}"
