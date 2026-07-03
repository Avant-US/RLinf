#!/usr/bin/env bash
# =============================================================================
# Vertex AI multi-node container entrypoint for RLinf pi0.5 (openpi_au) pushdoor
# (r1_pro) SFT smoke test.
#
# Every worker-pool replica runs this same script. It lives on GCS; the container
# command is:  bash /gcs/<BUCKET>/rlinf/demo3/pushdoor_bootstrap.sh
# (/gcs is Vertex AI's Cloud Storage FUSE auto-mount; single-region bucket only.)
#
# Differences vs the LIBERO bootstrap.sh:
#   * No HF_LEROBOT_HOME/repo_id indirection: resolve_lerobot_dataset_root()
#     (rlinf/data/lerobot_paths.py) accepts the raw pushdoor dataset root under the
#     GCS FUSE mount directly (it has meta/info.json right there), so
#     data.train_data_paths is set straight to that path.
#   * Base weights are the converted pi05_r1pro_chassis_alig_newnorm checkpoint
#     (tests_au/example/pushdoor/convert_r1pro_ckpt.py), staged under
#     rlinf/models/pi05_pushdoor_r1pro_pt/ instead of rlinf/models/pi05_base_pt/.
#   * norm_stats.json asset id is rlinf/pushdoor_open0622 (the dataset's LeRobot task
#     tag), not physical-intelligence/libero.
#   * Model weights are staged to LOCAL disk before training starts (step 4b below),
#     instead of loading straight off the GCS FUSE mount like LIBERO does. Observed
#     on the first real 3-node run: `get_model()`
#     (rlinf/models/embodiment/openpi_au/__init__.py) calls
#     `safetensors.torch.load_file()` on the ~15GB model.safetensors independently on
#     EVERY one of the 24 ranks (no rank-0-broadcast; this happens before FSDP
#     sharding), and safetensors' internal mmap-style access issues many small/random
#     reads rather than one bulk sequential read. GCS FUSE serves that access pattern
#     far slower than sequential I/O, and 24 concurrent readers made it worse still:
#     the job sat completely silent (zero log lines, zero errors, state still
#     JOB_STATE_RUNNING) for 40+ minutes at this exact step (see gcp_rlinf_pi05_au.md
#     pushdoor section). A single `gcloud storage rsync` per NODE (not per rank,
#     since this script runs once per worker-pool replica) directly from GCS
#     (bypassing FUSE) up front turns 24 slow independent FUSE mmap loads into 3 fast
#     bulk downloads followed by 24 fast local-disk loads.
#
# Env injected by pushdoor_vertex_3node.yaml (containerSpec.env):
#   GCS_BUCKET   e.g. physical-ai-data-eu (no gs:// prefix)
#   EXP_NAME     experiment name for the output dir, e.g. pi05_pushdoor_au_3node
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
EXP_NAME="${EXP_NAME:-pi05_pushdoor_au_3node}"
RAY_PORT="${RAY_PORT:-6379}"
VENV_NAME="${VENV_NAME:-reason}"

GCS_ROOT="/gcs/${GCS_BUCKET}/rlinf"                         # FUSE staging area
CODE_TAR="${GCS_ROOT}/code/RLinf.tar.gz"
LOCAL_REPO="/workspace/RLinf"                               # unpack code locally (FUSE is poor for imports)
MODEL_DIR="${GCS_ROOT}/models/pi05_pushdoor_r1pro_pt"       # model.safetensors + config.json + norm_stats (FUSE)
MODEL_GCS_URI="gs://${GCS_BUCKET}/rlinf/models/pi05_pushdoor_r1pro_pt"  # same location, direct gs:// URI
LOCAL_MODEL_DIR="/workspace/model_local/pi05_pushdoor_r1pro_pt"  # local-disk copy used for actual training (see step 4b)
DATASET_ROOT="/gcs/${GCS_BUCKET}/DATA/SKILL/pushdoor/0622_lerobot_data"
ASSET_ID="rlinf/pushdoor_open0622"                          # matches LeRobotPushdoorDataConfig's asset_id
OUTPUT_DIR="${GCS_ROOT}/runs/${EXP_NAME}"                   # checkpoints/tensorboard land on GCS

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

# ----------------- 3. offline HF (avoid 429 at N ranks); no LEROBOT_HOME -----
export HF_HUB_OFFLINE=1
export HF_DATASETS_OFFLINE=1
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
export RLINF_PUSHDOOR_DATA="${DATASET_ROOT}"
export RLINF_PUSHDOOR_LOG="${OUTPUT_DIR}"
cd "${LOCAL_REPO}"

# ------------------ 4b. stage model weights to LOCAL disk --------------------
# See the header comment for why: avoids 24-way concurrent slow FUSE mmap reads of
# the ~15GB checkpoint. `gcloud storage rsync` downloads directly from GCS (not
# through FUSE) with parallel/composite transfers, once per node.
log "Staging model weights ${MODEL_GCS_URI} -> ${LOCAL_MODEL_DIR} (bypasses FUSE)"
mkdir -p "${LOCAL_MODEL_DIR}"
time gcloud storage rsync -r "${MODEL_GCS_URI}" "${LOCAL_MODEL_DIR}" \
  || { log "ERROR: failed to stage model weights locally"; exit 1; }
log "Model staged locally: $(du -sh "${LOCAL_MODEL_DIR}" 2>/dev/null | cut -f1)"
MODEL_DIR="${LOCAL_MODEL_DIR}"
export RLINF_PUSHDOOR_MODEL="${MODEL_DIR}"

# --------------------------- 5. health checks -------------------------------
nvidia-smi -L || { log "nvidia-smi failed"; exit 1; }
python -c "import openpi, jax, lerobot; print('openpi runtime OK')" \
  || { log "openpi runtime import failed (image build issue)"; exit 1; }
ls "${MODEL_DIR}/model.safetensors" >/dev/null 2>&1 \
  || { log "missing weights (local copy): ${MODEL_DIR}/model.safetensors"; exit 1; }
NORM="${MODEL_DIR}/${ASSET_ID}/norm_stats.json"
[ -f "${NORM}" ] || { log "missing norm stats (local copy): ${NORM}"; exit 1; }
ls "${DATASET_ROOT}/meta/episodes.jsonl" >/dev/null 2>&1 \
  || { log "missing LeRobot data: ${DATASET_ROOT}/meta/episodes.jsonl"; exit 1; }
mkdir -p "${OUTPUT_DIR}"

CONFIG_DIR="${LOCAL_REPO}/b/gcp/demo3"
CONFIG_NAME="pushdoor_sft_pi05_au_multinode"

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
    data.train_data_paths="${DATASET_ROOT}"

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
