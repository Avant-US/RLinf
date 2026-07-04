#!/usr/bin/env bash
# =============================================================================
# Container entrypoint for Docker-based pi0.5 (openpi_au) pushdoor SFT.
#
# Supports single-node (default) and multi-node distributed training via
# RANK/NUM_NODES/HEAD_HOST env vars. No GCP/Vertex dependency — designed for
# on-prem Docker clusters where each physical node runs one container.
#
# Adapted from b/gcp/demo3/pushdoor_bootstrap.sh: same Ray cluster formation
# and keep-alive logic, stripped of CLUSTER_SPEC parsing / GCS FUSE staging /
# gcloud storage rsync. Source code and model weights are bind-mounted.
#
# Env vars (all passed by the host-side run_train.sh via docker run -e):
#   RANK        Node rank (0 = head, default 0)
#   NUM_NODES   Total node count (default 1)
#   HEAD_HOST   IP/hostname of rank-0 node (default localhost)
#   RAY_PORT    Ray head port (default 6379)
#   RLINF_PUSHDOOR_MODEL  Model dir in container (default /workspace/model)
#   RLINF_PUSHDOOR_DATA   Dataset dir in container (default /workspace/data)
#   RLINF_PUSHDOOR_LOG    Output dir in container (default /workspace/output)
# =============================================================================
set -euo pipefail
export PYTHONUNBUFFERED=1

# ----------------------------- 0. params & defaults --------------------------
RANK="${RANK:-0}"
NUM_NODES="${NUM_NODES:-1}"
HEAD_HOST="${HEAD_HOST:-localhost}"
RAY_PORT="${RAY_PORT:-6379}"
REPO="/workspace/RLinf"

log() { echo "[bootstrap][$(date +%H:%M:%S)] $*"; }

# ----------------------------- 1. activate venv ------------------------------
set +u
source /venv/rlinf/bin/activate
set -u
log "python = $(which python), $(python --version 2>&1)"

# ------------------- 2. export RLinf node rank (before ray start) ------------
export RLINF_NODE_RANK="${RANK}"

# ----------------------------- 3. env vars -----------------------------------
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export HF_DATASETS_OFFLINE="${HF_DATASETS_OFFLINE:-1}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-egl}"
export USE_TF="${USE_TF:-0}"
export USE_FLAX="${USE_FLAX:-0}"
export PYTHONPATH="${REPO}:${PYTHONPATH:-}"
export EMBODIED_PATH="${REPO}/examples/sft"

GPUS_PER_NODE=8
export RLINF_NUM_NODES="${NUM_NODES}"
export RLINF_GLOBAL_BATCH=$(( NUM_NODES * GPUS_PER_NODE ))

# ----------------------------- 4. paths --------------------------------------
MODEL_DIR="${RLINF_PUSHDOOR_MODEL:-/workspace/model}"
DATA_DIR="${RLINF_PUSHDOOR_DATA:-/workspace/data}"
LOG_DIR="${RLINF_PUSHDOOR_LOG:-/workspace/output}"
export RLINF_PUSHDOOR_MODEL="${MODEL_DIR}"
export RLINF_PUSHDOOR_DATA="${DATA_DIR}"
export RLINF_PUSHDOOR_LOG="${LOG_DIR}"

# ----------------------------- 5. health checks ------------------------------
nvidia-smi -L || { log "nvidia-smi failed"; exit 1; }
python -c "import rlinf, openpi; print('imports OK')" \
  || { log "import failed -- is source bind-mounted at /workspace/RLinf and /workspace/aupi05?"; exit 1; }
[ -f "${MODEL_DIR}/model.safetensors" ] \
  || { log "missing weights: ${MODEL_DIR}/model.safetensors"; exit 1; }
NORM="${MODEL_DIR}/rlinf/pushdoor_open0622/norm_stats.json"
[ -f "${NORM}" ] || { log "missing norm stats: ${NORM}"; exit 1; }
[ -f "${DATA_DIR}/meta/episodes.jsonl" ] \
  || { log "missing dataset: ${DATA_DIR}/meta/episodes.jsonl"; exit 1; }
mkdir -p "${LOG_DIR}"

CONFIG_DIR="${REPO}/examples/au/pidk"
CONFIG_NAME="pushdoor_sft_pi05_au_dk"

log "RANK=${RANK}  NUM_NODES=${NUM_NODES}  HEAD_HOST=${HEAD_HOST}  RAY_PORT=${RAY_PORT}"
log "MODEL=${MODEL_DIR}  DATA=${DATA_DIR}  LOG=${LOG_DIR}"
log "GLOBAL_BATCH=${RLINF_GLOBAL_BATCH}  GPUS_PER_NODE=${GPUS_PER_NODE}"

# =============================== 6. head / worker ============================
if [ "${RANK}" -eq 0 ]; then
  # ---------------------------- HEAD (rank 0) --------------------------------
  log "Starting Ray HEAD on port ${RAY_PORT}"
  ray start --head --port="${RAY_PORT}" --dashboard-host=0.0.0.0 --disable-usage-stats

  cd "${REPO}"
  log "Launching pi0.5 SFT (blocks until all ${NUM_NODES} nodes join Ray)"
  python examples/sft/train_vla_sft_au.py \
    --config-path "${CONFIG_DIR}" \
    --config-name "${CONFIG_NAME}" \
    cluster.num_nodes="${NUM_NODES}" \
    actor.global_batch_size="${RLINF_GLOBAL_BATCH}"

  log "Training finished; stopping Ray."
  ray stop || true
  log "DONE (rank 0). Checkpoints at ${LOG_DIR}"

else
  # --------------------------- WORKER (rank > 0) -----------------------------
  log "Waiting for Ray head ${HEAD_HOST}:${RAY_PORT} to become reachable..."
  check_port() {
    python3 -c "import socket; s=socket.socket(); s.settimeout(2); s.connect(('$1',int($2)))" >/dev/null 2>&1
  }
  for i in $(seq 1 600); do
    if check_port "${HEAD_HOST}" "${RAY_PORT}"; then
      log "Head reachable after ${i}s"
      break
    fi
    sleep 1
    if [ "${i}" -eq 600 ]; then log "ERROR: head not reachable in 600s"; exit 1; fi
  done

  log "Joining Ray cluster at ${HEAD_HOST}:${RAY_PORT}"
  ray start --address="${HEAD_HOST}:${RAY_PORT}" --disable-usage-stats

  log "Worker joined; keeping alive until head exits."
  miss=0
  while true; do
    if check_port "${HEAD_HOST}" "${RAY_PORT}"; then
      miss=0
    else
      miss=$((miss + 1))
      if [ "${miss}" -ge 12 ]; then log "Head gone; worker exiting."; break; fi
    fi
    sleep 5
  done
  ray stop || true
  log "DONE (rank ${RANK})."
fi
