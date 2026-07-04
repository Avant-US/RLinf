#!/usr/bin/env bash
# =============================================================================
# Vertex AI 多机容器入口（demo4 / dev 镜像 rlinf-aupi-dev）：
#   RLinf pi0.5 (openpi_au) pushdoor SFT on 0622_lerobot_data_tst1.
#
# 每个 worker-pool 副本都跑本脚本。容器命令为：
#   bash /gcs/<BUCKET>/rlinf/demo4/pushdoor_bootstrap.sh
#
# dev 镜像与 demo3 的关键差异：
#   A. venv 在 /venv/rlinf（不是 /opt/venv/reason）
#   B. RLinf + openpi 都是 editable 安装 -> 必须解包源码到 /workspace/{RLinf,aupi05}
#   C. transformers_replace 补丁已内置，bootstrap 幂等 cp 作为安全网
#
# Env（由 pushdoor_vertex_3node.yaml 注入）：
#   GCS_BUCKET  例 physical-ai-data-eu
#   EXP_NAME    实验名
#   RAY_PORT    默认 6379
#   VENV_PATH   默认 /venv/rlinf
# =============================================================================
set -euo pipefail
export PYTHONUNBUFFERED=1

# ─────────────────── 0. 参数与默认值 ───────────────────
GCS_BUCKET="${GCS_BUCKET:?must set GCS_BUCKET}"
EXP_NAME="${EXP_NAME:-pi05_pushdoor_au_tst1_3node}"
RAY_PORT="${RAY_PORT:-6379}"
VENV_PATH="${VENV_PATH:-/venv/rlinf}"

GCS_ROOT="/gcs/${GCS_BUCKET}/rlinf"
CODE_TAR="${GCS_ROOT}/code/RLinf.tar.gz"
AUPI_TAR="${GCS_ROOT}/code/aupi05.tar.gz"
LOCAL_REPO="/workspace/RLinf"
LOCAL_AUPI="/workspace/aupi05"

MODEL_FUSE_DIR="/gcs/${GCS_BUCKET}/rlinf/models/pi05_pushdoor_r1pro_pt"
LOCAL_MODEL_DIR="/workspace/model_local/pi05_pushdoor_r1pro_pt"
DATASET_ROOT="/gcs/${GCS_BUCKET}/DATA/SKILL/pushdoor/0622_lerobot_data_tst1"
ASSET_ID="rlinf/pushdoor_open0622"
OUTPUT_DIR="${GCS_ROOT}/runs/${EXP_NAME}"

log() { echo "[bootstrap][$(date +%H:%M:%S)] $*"; }

# ─────────────────── 1. 激活 venv ───────────────────
set +u
source "${VENV_PATH}/bin/activate"
set -u
log "python = $(which python), $(python --version 2>&1)"

# ─────────────────── 2. 解析 CLUSTER_SPEC ───────────────────
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
export RLINF_NODE_RANK="${NODE_RANK}"

# ─────────────────── 3. 环境变量 ───────────────────
export HF_HUB_OFFLINE=1
export HF_DATASETS_OFFLINE=1
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-egl}"
export USE_TF=0
export USE_FLAX=0

# ─────────────────── 4. 等待 FUSE + 解包源码 ───────────────────
log "Waiting for GCS FUSE mount..."
for i in $(seq 1 60); do
  if [ -f "${CODE_TAR}" ] && [ -f "${AUPI_TAR}" ]; then log "FUSE ready; tarballs found."; break; fi
  sleep 1
  if [ "${i}" -eq 60 ]; then
    log "ERROR: tarballs not found after 60s"; ls -la "${GCS_ROOT}/code" || true; exit 1
  fi
done

log "Staging RLinf -> ${LOCAL_REPO}"
mkdir -p "${LOCAL_REPO}"
tar -xzf "${CODE_TAR}" -C "${LOCAL_REPO}" --strip-components=1

log "Staging aupi05 -> ${LOCAL_AUPI}"
mkdir -p "${LOCAL_AUPI}"
tar -xzf "${AUPI_TAR}" -C "${LOCAL_AUPI}" --strip-components=1

export REPO_PATH="${LOCAL_REPO}"
export PYTHONPATH="${LOCAL_REPO}:${LOCAL_AUPI}/src:${PYTHONPATH:-}"
export EMBODIED_PATH="${LOCAL_REPO}/examples/sft"
export RLINF_PUSHDOOR_DATA="${DATASET_ROOT}"
export RLINF_PUSHDOOR_LOG="${OUTPUT_DIR}"
cd "${LOCAL_REPO}"

# ─────────────────── 4b. transformers_replace 补丁（幂等安全网）───────────────────
TFM_DIR="$(python -c 'import os,transformers;print(os.path.dirname(transformers.__file__))')"
log "Applying transformers_replace patch -> ${TFM_DIR}"
cp -r "${LOCAL_AUPI}/src/openpi/models_pytorch/transformers_replace/"* "${TFM_DIR}/"
python -c "import transformers; from transformers.models.siglip import check; \
assert check.check_whether_transformers_replace_is_installed_correctly(), 'patch failed'; \
print('transformers_replace OK, transformers', transformers.__version__)" \
  || { log "ERROR: transformers_replace patch verification failed"; exit 1; }

# ─────────────────── 4c. 模型权重 stage 到本地盘 ───────────────────
log "Staging model weights -> ${LOCAL_MODEL_DIR} (cp from FUSE, bypasses concurrent mmap)"
mkdir -p "${LOCAL_MODEL_DIR}"
time cp -r "${MODEL_FUSE_DIR}/." "${LOCAL_MODEL_DIR}/" \
  || { log "ERROR: failed to stage model weights from ${MODEL_FUSE_DIR}"; exit 1; }
MODEL_DIR="${LOCAL_MODEL_DIR}"
export RLINF_PUSHDOOR_MODEL="${MODEL_DIR}"
log "Model staged: $(du -sh "${MODEL_DIR}" 2>/dev/null | cut -f1)"

# ─────────────────── 5. 健康检查 ───────────────────
nvidia-smi -L || { log "nvidia-smi failed"; exit 1; }
python -c "import openpi, jax, lerobot; import openpi.models_pytorch.pi0_pytorch; print('openpi OK')" \
  || { log "openpi runtime import failed"; exit 1; }
[ -f "${MODEL_DIR}/model.safetensors" ] \
  || { log "missing: ${MODEL_DIR}/model.safetensors"; exit 1; }
[ -f "${MODEL_DIR}/${ASSET_ID}/norm_stats.json" ] \
  || { log "missing: ${MODEL_DIR}/${ASSET_ID}/norm_stats.json"; exit 1; }
[ -f "${DATASET_ROOT}/meta/episodes.jsonl" ] \
  || { log "missing: ${DATASET_ROOT}/meta/episodes.jsonl"; exit 1; }
mkdir -p "${OUTPUT_DIR}"

CONFIG_DIR="${LOCAL_REPO}/b/gcp/demo4"
CONFIG_NAME="pushdoor_sft_pi05_au_multinode"

# ═══════════════════ 6. head / worker ═══════════════════
if [ "${NODE_RANK}" -eq 0 ]; then
  log "Starting Ray HEAD on port ${RAY_PORT}"
  ray start --head --port="${RAY_PORT}" --dashboard-host=0.0.0.0 --disable-usage-stats

  log "Launching training (waiting for ${NUM_NODES} nodes to join Ray)"
  python examples/sft/train_vla_sft_au.py \
    --config-path "${CONFIG_DIR}" \
    --config-name "${CONFIG_NAME}" \
    runner.logger.log_path="${OUTPUT_DIR}" \
    actor.model.model_path="${MODEL_DIR}" \
    data.train_data_paths="${DATASET_ROOT}"

  log "Training finished; stopping Ray."
  ray stop || true
  log "DONE (rank 0). Outputs: gs://${GCS_BUCKET}/rlinf/runs/${EXP_NAME}"

else
  log "Waiting for Ray head ${HEAD_HOST}:${RAY_PORT}..."
  check_port() {
    python -c "import socket; s = socket.socket(); s.settimeout(2); s.connect(('$1', int($2)))" >/dev/null 2>&1
  }
  for i in $(seq 1 600); do
    if check_port "${HEAD_HOST}" "${RAY_PORT}"; then log "Head reachable after ${i}s"; break; fi
    sleep 1
    if [ "${i}" -eq 600 ]; then log "ERROR: head not reachable in 600s"; exit 1; fi
  done

  log "Joining Ray cluster at ${HEAD_HOST}:${RAY_PORT}"
  ray start --address="${HEAD_HOST}:${RAY_PORT}" --disable-usage-stats

  log "Worker joined; polling head..."
  miss=0
  while true; do
    if check_port "${HEAD_HOST}" "${RAY_PORT}"; then miss=0
    else
      miss=$((miss + 1))
      if [ "${miss}" -ge 12 ]; then log "Head gone; exiting."; break; fi
    fi
    sleep 5
  done
  ray stop || true
  log "DONE (rank ${NODE_RANK})."
fi