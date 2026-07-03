#!/usr/bin/env bash
# =============================================================================
# Vertex AI multi-node container entrypoint for RLinf pi0.5 (openpi_au) LIBERO SFT.
#
# Every worker-pool replica runs this same script. It lives on GCS; the container
# command is:  bash /gcs/<BUCKET>/rlinf/demo3/bootstrap.sh
# (/gcs is Vertex AI's Cloud Storage FUSE auto-mount; single-region bucket only.)
#
# Differences vs the demo2 (Qwen VLM) bootstrap:
#   * The custom image already has openpi + jax + flax + lerobot + transformers_replace
#     patch installed in the `reason` venv, so we do NOT ship openpi source.
#   * We point HF_LEROBOT_HOME at the staged LIBERO LeRobot dataset and force HF offline;
#     the openpi_au worker further restricts to the contiguous local-episode prefix.
#   * The pi0.5 model dir must contain model.safetensors + config.json +
#     physical-intelligence/libero/norm_stats.json (staged in manual Step 2).
#
# Env injected by vertex_pi05_au_3node.yaml (containerSpec.env):
#   GCS_BUCKET   e.g. physical-ai-data-eu (no gs:// prefix)
#   EXP_NAME     experiment name for the output dir, e.g. pi05_libero_au_3node
#   RAY_PORT     Ray head port, default 6379
#   VENV_NAME    venv name in the image, default reason
#
# This same script works unmodified for any workerPoolSpecs[1].replicaCount (the
# CLUSTER_SPEC parsing below derives NODE_RANK/NUM_NODES generically); the default
# topology here is 1 primary + 2 workers = 3 nodes / 24 GPUs (all reservation nodes).
# =============================================================================
set -euo pipefail
export PYTHONUNBUFFERED=1

# ----------------------------- 0. params & defaults --------------------------
GCS_BUCKET="${GCS_BUCKET:?must set GCS_BUCKET}"
EXP_NAME="${EXP_NAME:-pi05_libero_au_3node}"
RAY_PORT="${RAY_PORT:-6379}"
VENV_NAME="${VENV_NAME:-reason}"

GCS_ROOT="/gcs/${GCS_BUCKET}/rlinf"          # FUSE staging area
CODE_TAR="${GCS_ROOT}/code/RLinf.tar.gz"
LOCAL_REPO="/workspace/RLinf"                # unpack code locally (FUSE is poor for imports)
MODEL_DIR="${GCS_ROOT}/models/pi05_base_pt"  # model.safetensors + config.json + norm_stats
LEROBOT_HOME="${GCS_ROOT}/data/lerobot"      # HF_LEROBOT_HOME; dataset under {repo_id}/
REPO_ID="physical-intelligence/libero"
OUTPUT_DIR="${GCS_ROOT}/runs/${EXP_NAME}"    # checkpoints/tensorboard land on GCS

log() { echo "[bootstrap][$(date +%H:%M:%S)] $*"; }

# ----------------------------- 1. activate venv ------------------------------
# venv activate scripts touch unbound $PYTHONPATH; disable `set -u` around it.
set +u
source "/opt/venv/${VENV_NAME}/bin/activate"
set -u
log "python = $(which python), $(python --version 2>&1)"

# ----------------------- 2. parse CLUSTER_SPEC -> ranks ----------------------
log "CLUSTER_SPEC=${CLUSTER_SPEC:-<empty>}"
read -r NODE_RANK NUM_NODES HEAD_HOST < <(python - <<'PY'
import json, os
spec = json.loads(os.environ.get("CLUSTER_SPEC") or "{}")
cluster = spec.get("cluster", {})
task = spec.get("task", {"type": "workerpool0", "index": 0})
pool0 = cluster.get("workerpool0", ["localhost:2222"])
pool1 = cluster.get("workerpool1", [])
n0, n1 = len(pool0), len(pool1)
ttype, tindex = task.get("type", "workerpool0"), int(task.get("index", 0))
rank = 0 if ttype == "workerpool0" else n0 + tindex
head_host = pool0[0].rsplit(":", 1)[0]
print(rank, n0 + n1, head_host)
PY
)
log "NODE_RANK=${NODE_RANK}  NUM_NODES=${NUM_NODES}  HEAD_HOST=${HEAD_HOST}  RAY_PORT=${RAY_PORT}"

# RLinf identifies nodes via this var; must be exported BEFORE `ray start`.
export RLINF_NODE_RANK="${NODE_RANK}"

# ----------------- 3. offline HF + LeRobot home (avoid 429 at N ranks) --------
export HF_HUB_OFFLINE=1
export HF_DATASETS_OFFLINE=1
export HF_LEROBOT_HOME="${LEROBOT_HOME}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-egl}"

# --------------------- 4. wait for GCS FUSE & stage code ---------------------
log "Waiting for GCS FUSE mount to become ready..."
for i in $(seq 1 30); do
  if [ -f "${CODE_TAR}" ]; then log "GCS FUSE ready; code tarball found."; break; fi
  sleep 1
  if [ "${i}" -eq 30 ]; then
    log "ERROR: code tarball not found at ${CODE_TAR} after 30s."; ls -la /gcs || true; exit 1
  fi
done

log "Staging code from ${CODE_TAR} -> ${LOCAL_REPO}"
mkdir -p "${LOCAL_REPO}"
tar -xzf "${CODE_TAR}" -C "${LOCAL_REPO}" --strip-components=1
export REPO_PATH="${LOCAL_REPO}"
# openpi is installed in the image; only RLinf source needs to be on PYTHONPATH.
export PYTHONPATH="${LOCAL_REPO}:${PYTHONPATH:-}"
# Hydra group defaults (model/pi0_5_au, training_backend/fsdp) resolve via this dir.
export EMBODIED_PATH="${LOCAL_REPO}/examples/sft"
export RLINF_LIBERO_MODEL="${MODEL_DIR}"
export RLINF_LIBERO_LOG="${OUTPUT_DIR}"
cd "${LOCAL_REPO}"

# --------------------------- 5. health checks -------------------------------
nvidia-smi -L || { log "nvidia-smi failed"; exit 1; }
python -c "import openpi, jax, lerobot; print('openpi runtime OK')" \
  || { log "openpi runtime import failed (image build issue)"; exit 1; }
ls "${MODEL_DIR}/model.safetensors" >/dev/null 2>&1 \
  || { log "missing weights: ${MODEL_DIR}/model.safetensors"; exit 1; }
NORM="${MODEL_DIR}/${REPO_ID}/norm_stats.json"
[ -f "${NORM}" ] || { log "missing norm stats: ${NORM}"; exit 1; }
ls "${LEROBOT_HOME}/${REPO_ID}/data" >/dev/null 2>&1 \
  || { log "missing LeRobot data: ${LEROBOT_HOME}/${REPO_ID}/data"; exit 1; }
mkdir -p "${OUTPUT_DIR}"

CONFIG_DIR="${LOCAL_REPO}/b/gcp/demo3"
CONFIG_NAME="libero_sft_pi05_au_multinode"

# =============================== 6. head / worker ============================
if [ "${NODE_RANK}" -eq 0 ]; then
  # ---------------------------- HEAD (rank 0) ------------------------------
  log "Starting Ray HEAD on port ${RAY_PORT}"
  ray start --head --port="${RAY_PORT}" --dashboard-host=0.0.0.0 --disable-usage-stats

  log "Launching RLinf pi0.5 SFT (blocks until all ${NUM_NODES} nodes join Ray)"
  python examples/sft/train_vla_sft_au.py \
    --config-path "${CONFIG_DIR}" \
    --config-name "${CONFIG_NAME}" \
    runner.logger.log_path="${OUTPUT_DIR}" \
    actor.model.model_path="${MODEL_DIR}" \
    data.train_data_paths="${REPO_ID}"

  log "Training finished; stopping Ray head."
  ray stop || true
  log "DONE (rank 0). Outputs at gs://${GCS_BUCKET}/rlinf/runs/${EXP_NAME}"

else
  # --------------------------- WORKER (rank > 0) ---------------------------
  log "Waiting for Ray head ${HEAD_HOST}:${RAY_PORT} to become reachable..."
  check_port() {
    python3 -c "import socket; s = socket.socket(); s.settimeout(2); s.connect(('$1', int($2)))" >/dev/null 2>&1
  }
  for i in $(seq 1 600); do
    if check_port "${HEAD_HOST}" "${RAY_PORT}"; then log "Head reachable after ${i}s"; break; fi
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
  log "DONE (rank ${NODE_RANK})."
fi
