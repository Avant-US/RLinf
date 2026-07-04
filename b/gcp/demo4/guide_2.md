# 在 Vertex AI 上用 dev 镜像 `rlinf-aupi-dev:260702` 多机训练 pi0.5 pushdoor SFT

> **目标**：用 [`Dockerfile.aupi_dev`](./Dockerfile.aupi_dev) 构建的 dev 镜像 `rlinf-aupi-dev:260702`，在 Vertex AI 的 3 台 `a3-ultragpu-8g`（24×H200，`europe-west4-a` 预留）上，对 [`openpi_au`](../../../rlinf/models/embodiment/openpi_au/) 的 pi0.5 模型做 pushdoor（R1 Pro「推门」）SFT，数据集为 `0622_lerobot_data_tst1`（LeRobot v3，12 episodes / 13136 帧）。
>
> 本手册是 [`guide_1.md`](./guide_1.md) 的修订版，修正了已知问题并对齐当前代码。

---

## 0. 概述

### 0.1 数据流

```
本地                                          GCP
─────────────────────────────────────────────────────────────────
docker buildx --load                    Artifact Registry
rlinf-aupi-dev:260702  ──tag+push──►    europe-west4-docker.pkg.dev/.../rlinf-aupi-dev:260702
                                              │
                                              ▼
tar RLinf + aupi05 ──upload──►  GCS     Vertex AI 自定义训练作业
model + norm_stats ──rsync──►   ├─ code/RLinf.tar.gz         3× a3-ultragpu-8g (24×H200)
dataset            ──rsync──►   ├─ code/aupi05.tar.gz        ┌──────────────────────────┐
                                ├─ models/pi05_.../           │ bootstrap.sh:            │
                                ├─ DATA/.../tst1/             │  解包源码→打补丁→stage权重│
                                ├─ demo4/bootstrap.sh         │  →组Ray集群→跑SFT        │
                                └─ runs/EXP_NAME/ ◄── 输出    └──────────────────────────┘
```

### 0.2 dev 镜像的核心机制：editable 安装 + 源码运行时 staging

[`Dockerfile.aupi_dev`](./Dockerfile.aupi_dev) 用 BuildKit bind-mount 把 RLinf 和 aupi05 源码在构建期以 `pip install -e` 安装进 `/venv/rlinf`。editable 安装只在 site-packages 里写 `.pth` 指针文件，指向构建期的精确路径 `/workspace/RLinf` 和 `/workspace/aupi05`。

**关键约束**：运行时**必须**把源码 stage 到这两个精确路径，否则 `import rlinf` / `import openpi` 会 `ModuleNotFoundError`。
- **本地**：`docker run -v /host/RLinf:/workspace/RLinf -v /host/aupi05:/workspace/aupi05`
- **Vertex**：bootstrap 从 GCS 解包两份 tarball 到这两个路径

### 0.3 沿用 demo3 的既有实践

| 实践 | 原因 |
|------|------|
| 模型权重 stage 到本地盘再训练 | 24 rank 并发 FUSE mmap 读 15GB safetensors 会卡 40+ 分钟 |
| `HF_HUB_OFFLINE=1` | 避免 24 rank 并发触发 HuggingFace 429 |
| 数据集直接用 FUSE 路径 | `resolve_lerobot_dataset_root()` 直接接受带 `meta/info.json` 的绝对路径 |
| FSDP `mixed_precision` 全 null | openpi 原生管 dtype（选择性 bf16）；FSDP 统一 bf16 会破坏 fp32 动作头 |
| `hyperdisk-balanced` 引导盘 | `a3-ultragpu-8g` 强制要求 |
| 单区域 GCS 桶 | FUSE 自动挂载到 `/gcs/<bucket>` 仅支持单区域桶 |

### 0.4 配套文件

| 文件 | 作用 |
|------|------|
| [`Dockerfile.aupi_dev`](./Dockerfile.aupi_dev) | dev 镜像构建文件（已存在） |
| `guide_2.md`（本文件） | 操作手册 |
| `pushdoor_sft_pi05_au_multinode.yaml` | RLinf 多机训练配置（§5 内嵌） |
| `pushdoor_bootstrap.sh` | 容器入口脚本（§6 内嵌） |
| `pushdoor_vertex_3node.yaml` | Vertex 作业定义（§7 内嵌） |
| `pushdoor_submit.sh` | 一键提交脚本（§8 内嵌） |


### 0.5 OpenPi 官方的Pi0.5资产路径

* pi0.5 base assets（含 norm_stats 等）：gs://openpi-assets/checkpoints/pi05_base/assets
* pi0 base：gs://openpi-assets/checkpoints/pi0_base/params
* pi0.5 DROID fine-tuned：gs://openpi-assets/checkpoints/pi05_droid/params
* pi0-FAST base：gs://openpi-assets/checkpoints/pi0_fast_base/params


### 0.6 OpenPi 权重转成pytorch/huggingface格式

能，aupi05 里已经有现成的转换脚本：examples/convert_jax_model_to_pytorch.py

用法：
1. 先下载 JAX checkpoint 到本地（orbax 也支持直接读 gs://）; openpi 的 download.maybe_download() 会自动缓存到 ~/.cache/openpi/
2. 转换成 PyTorch safetensors
```bash
python examples/convert_jax_model_to_pytorch.py \
  --checkpoint_dir ~/.cache/openpi/openpi-assets/checkpoints/pi05_base \
  --config_name pi05_base \
  --output_path /tmp/pi05_base_pytorch \
  --precision float32
```

转换后产出：
```
model.safetensors — PyTorch state_dict（safetensors 格式）
config.json — 模型配置
assets/ — norm_stats 等资产（如果有）
```

转换逻辑做了以下关键变换：
```
SigLIP vision encoder：JAX einsum kernel → PyTorch conv2d/linear .weight（transpose）
PaliGemma language model (Gemma 2B)：JAX einsum attention q/kv/attn_vec → PyTorch q_proj/k_proj/v_proj/o_proj（reshape + transpose）
Action expert (Gemma 300M)：同上，加上 pi0.5 特有的 AdaRMS Dense 层
Projection layers：time_mlp_in/out、action_in/out_proj（transpose）
```

---

## 1. 前置条件

```bash
gcloud auth login
gcloud auth application-default login
gcloud config set project autel-ai-physical-spat-intel

gcloud services enable \
  aiplatform.googleapis.com artifactregistry.googleapis.com \
  cloudbuild.googleapis.com storage.googleapis.com compute.googleapis.com

# 确认预留可用
gcloud compute reservations describe reservation-20260422-033135 --zone=europe-west4-a
```

---

## 2. 集中参数

```bash
# ---- 项目与地域 ----
export PROJECT_ID="autel-ai-physical-spat-intel"
export REGION="europe-west4"
export ZONE="europe-west4-a"

# ---- 镜像 ----
export LOCAL_IMAGE="rlinf-aupi-dev:260702"
export AR_IMAGE="${REGION}-docker.pkg.dev/${PROJECT_ID}/rlinf/rlinf-aupi-dev:260702"

# ---- 存储（必须单区域桶）----
export GCS_BUCKET="physical-ai-data-eu"
export GCS_ROOT="gs://${GCS_BUCKET}/rlinf"

# ---- 源码路径 ----
export RLINF_DIR="/home/physical/SRC/RL/RLinf"
export AUPI_DIR="/home/physical/SRC/Robot/aupi05"

# ---- 数据 / 模型 ----
export DATASET_LOCAL="/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data_tst1"
export DATASET_GCS="gs://${GCS_BUCKET}/DATA/SKILL/pushdoor/0622_lerobot_data_tst1"
export MODEL_LOCAL="/tmp/pi05_pushdoor_r1pro_pt"
export MODEL_GCS="${GCS_ROOT}/models/pi05_pushdoor_r1pro_pt"

# ---- 作业 ----
export EXP_NAME="pi05_pushdoor_au_tst1_3node"
```

---

## 3. 步骤一：推送 dev 镜像到 Artifact Registry

```bash
# 配置 docker 凭据（一次性）
gcloud auth configure-docker ${REGION}-docker.pkg.dev

# 确认 AR 仓库存在
gcloud artifacts repositories describe rlinf --location=${REGION} \
  || gcloud artifacts repositories create rlinf --repository-format=docker --location=${REGION}

# 打标签并推送（镜像约 20-30GB，首次 push 耗时较长）
docker tag  "${LOCAL_IMAGE}" "${AR_IMAGE}"
docker push "${AR_IMAGE}"

# 验证
gcloud artifacts docker images list "${REGION}-docker.pkg.dev/${PROJECT_ID}/rlinf" | grep aupi-dev

# 验证当前 tag 指向的新 digest
gcloud artifacts docker images describe "${AR_IMAGE}" \
  --format="value(image_summary.digest)"
```

---

## 4. 步骤二：镜像自检

构建期 [`Dockerfile.aupi_dev`](./Dockerfile.aupi_dev) 已跑过 import smoke test + `transformers_replace` 补丁校验。运行时可做更完整的验证：

```bash
docker run --rm --gpus all \
  -v ${RLINF_DIR}:/workspace/RLinf \
  -v ${AUPI_DIR}:/workspace/aupi05 \
  "${LOCAL_IMAGE}" bash -lc '
source /venv/rlinf/bin/activate
python - <<PY
import torch, transformers, openpi, jax, lerobot
print("torch", torch.__version__, "| transformers", transformers.__version__)
from transformers.models.siglip import check
assert check.check_whether_transformers_replace_is_installed_correctly(), "transformers_replace 补丁未生效"
import openpi.models_pytorch.pi0_pytorch as m
print("pi0_pytorch import OK")
import cv2
print("cv2 OK:", cv2.__version__)
print("ALL OK")
PY'
```

---

## 5. 步骤三：准备模型权重与 norm-stats

模型目录最终布局：

```
pi05_pushdoor_r1pro_pt/
├── model.safetensors              # ~15GB fp32 权重
├── config.json                    # 文档性配置
└── rlinf/pushdoor_open0622/
    └── norm_stats.json            # 归一化统计（asset_id = rlinf/pushdoor_open0622）
```

### 5.1 转换权重（若尚未有）

```bash
# 用已装 openpi + 补丁的环境
/mnt/r/VENV/openpi_venv/bin/python \
  ${RLINF_DIR}/tests_au/example/pushdoor/convert_r1pro_ckpt.py \
  --output_dir ${MODEL_LOCAL}
```

或直接用基座权重（无需转换）：

```bash
mkdir -p ${MODEL_LOCAL}
ln -sf /mnt/r/CKPT/VLA/pi05_base_pt_fp32/model.safetensors ${MODEL_LOCAL}/model.safetensors
ln -sf /mnt/r/CKPT/VLA/pi05_base_pt_fp32/config.json ${MODEL_LOCAL}/config.json
```

### 5.2 计算 norm-stats

norm_stats **必须**基于训练数据集计算。用 [`examples/au/pi/run_norm_stats.sh`](../../../examples/au/pi/run_norm_stats.sh)：

```bash
cd ${RLINF_DIR}
RLINF_PUSHDOOR_MODEL=${MODEL_LOCAL} \
RLINF_PUSHDOOR_DATA=${DATASET_LOCAL} \
MAX_FRAMES=13136 NUM_WORKERS=16 BATCH_SIZE=64 \
  bash examples/au/pi/run_norm_stats.sh
# 产出 ${MODEL_LOCAL}/rlinf/pushdoor_open0622/norm_stats.json
```

> `MAX_FRAMES=1024` 可用于快速冒烟；正式训练用 `13136`（全量帧数）。

### 5.3 同步到 GCS

```bash
gcloud storage rsync -r "${MODEL_LOCAL}" "${MODEL_GCS}"

# 校验
gcloud storage ls "${MODEL_GCS}/model.safetensors"
gcloud storage ls "${MODEL_GCS}/rlinf/pushdoor_open0622/norm_stats.json"
```

---

## 6. 步骤四：上传数据集到 GCS

保持 LeRobot v3 原始布局整体上传：

```bash
gcloud storage rsync -r "${DATASET_LOCAL}" "${DATASET_GCS}"

# 校验关键文件
gcloud storage ls "${DATASET_GCS}/meta/info.json"
gcloud storage ls "${DATASET_GCS}/data/chunk-000/" | head
gcloud storage ls "${DATASET_GCS}/videos/observation.images.head_rgb/chunk-000/" | head
```

容器内该数据集经 FUSE 挂载后路径为 `/gcs/physical-ai-data-eu/DATA/SKILL/pushdoor/0622_lerobot_data_tst1`。

---

## 7. 步骤五：多机训练配置

创建 `b/gcp/demo4/pushdoor_sft_pi05_au_multinode.yaml`：

```yaml
# RLinf pi0.5 (openpi_au) pushdoor SFT — 3 节点 Vertex 多机配置。
#
# world_size = 3 * 8 = 24 GPU；micro_batch=16 -> global_batch=384（grad_accum=1）
# 组默认 model/pi0_5_au、training_backend/fsdp 通过 EMBODIED_PATH -> examples/sft/config 解析。
defaults:
  - model/pi0_5_au@actor.model
  - training_backend/fsdp@actor.fsdp_config
  - override hydra/job_logging: stdout

hydra:
  run:
    dir: .
  output_subdir: null
  searchpath:
    - file://${oc.env:EMBODIED_PATH,examples/sft}/config/

cluster:
  num_nodes: 3
  component_placement:
    actor: all                        # FSDP actor 占满 24 张 GPU

runner:
  task_type: sft
  logger:
    log_path: "${oc.env:RLINF_PUSHDOOR_LOG,/gcs/physical-ai-data-eu/rlinf/runs/pi05_pushdoor_au_tst1}"
    project_name: rlinf_au
    experiment_name: "pi05_pushdoor_au_tst1_3node"
    logger_backends: ["tensorboard"]
  max_epochs: -1
  max_steps: 50                       # 冒烟；正式训练调大（如 1000）
  val_check_interval: -1
  save_interval: 25                   # 25/50 各存一次（含 EMA）

data:
  train_data_paths: "${oc.env:RLINF_PUSHDOOR_DATA,/gcs/physical-ai-data-eu/DATA/SKILL/pushdoor/0622_lerobot_data_tst1}"

actor:
  group_name: "ActorGroup"
  training_backend: "fsdp"
  micro_batch_size: 16                # 单卡本地 batch
  global_batch_size: 384              # 24 GPU × 16 = 384（grad_accum = 1）
  seed: 0

  model:
    precision: null
    model_path: "${oc.env:RLINF_PUSHDOOR_MODEL,/workspace/model_local/pi05_pushdoor_r1pro_pt}"
    num_action_chunks: 10
    action_dim: 23                    # 20 维 state 布局 + chassis_vel(3)
    add_value_head: False
    faithful_augmentation: true
    fp32_master_weights: false
    sft_gradient_checkpointing: true
    openpi:
      config_name: "pi05_pushdoor"
      train_expert_only: False

  optim:
    lr: 5.0e-5
    adam_beta1: 0.9
    adam_beta2: 0.95
    adam_eps: 1.0e-08
    weight_decay: 1.0e-10
    clip_grad: 1.0
    ema_decay: 0.999
    lr_scheduler: "openpi_cosine"
    lr_warmup_steps: 100
    decay_steps: 1000
    decay_lr: 5.0e-6
    total_training_steps: 50

  fsdp_config:
    strategy: "fsdp"
    sharding_strategy: "full_shard"   # 24 路全分片
    use_orig_params: False
    gradient_checkpointing: true
    gradient_checkpointing_use_reentrant: true
    mixed_precision:
      param_dtype: null
      reduce_dtype: null
      buffer_dtype: null
    grad_scaler:
      enabled: false
```

> **扩缩节点**：同步调 `cluster.num_nodes` 与 `pushdoor_vertex_3node.yaml` 的 `replicaCount`，保持 `global_batch_size % (micro_batch_size × 8 × num_nodes) == 0`。正式训练把 `max_steps` / `total_training_steps` / `save_interval` 调大。

---

## 8. 步骤六：引导脚本

创建 `b/gcp/demo4/pushdoor_bootstrap.sh`：

```bash
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

MODEL_GCS_URI="gs://${GCS_BUCKET}/rlinf/models/pi05_pushdoor_r1pro_pt"
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
export PYTHONPATH="${LOCAL_REPO}:${PYTHONPATH:-}"
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
log "Staging model weights -> ${LOCAL_MODEL_DIR} (bypasses FUSE)"
mkdir -p "${LOCAL_MODEL_DIR}"
time gcloud storage rsync -r "${MODEL_GCS_URI}" "${LOCAL_MODEL_DIR}" \
  || { log "ERROR: failed to stage model weights"; exit 1; }
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
    python3 -c "import socket; s = socket.socket(); s.settimeout(2); s.connect(('$1', int($2)))" >/dev/null 2>&1
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
```

---

## 9. 步骤七：Vertex 作业配置

创建 `b/gcp/demo4/pushdoor_vertex_3node.yaml`：

```yaml
# Vertex AI 自定义训练：pi0.5 pushdoor SFT（demo4 / dev 镜像）
# 3 台 a3-ultragpu-8g（24×H200）：1 主 + 2 worker
#
# 提交：gcloud ai custom-jobs create --region=europe-west4 \
#         --display-name=rlinf-aupi-pushdoor-tst1-3node \
#         --config=pushdoor_vertex_3node.yaml

workerPoolSpecs:
  # ─── primary / head ───
  - replicaCount: 1
    machineSpec:
      machineType: a3-ultragpu-8g
      acceleratorType: NVIDIA_H200_141GB
      acceleratorCount: 8
      reservationAffinity:
        reservationAffinityType: SPECIFIC_RESERVATION
        key: compute.googleapis.com/reservation-name
        values:
          - projects/autel-ai-physical-spat-intel/zones/europe-west4-a/reservations/reservation-20260422-033135
    diskSpec:
      bootDiskType: hyperdisk-balanced
      bootDiskSizeGb: 1000
    containerSpec:
      imageUri: europe-west4-docker.pkg.dev/autel-ai-physical-spat-intel/rlinf/rlinf-aupi-dev:260702
      command: ["/bin/bash"]
      args: ["/gcs/physical-ai-data-eu/rlinf/demo4/pushdoor_bootstrap.sh"]
      env:
        - name: GCS_BUCKET
          value: "physical-ai-data-eu"
        - name: EXP_NAME
          value: "pi05_pushdoor_au_tst1_3node"
        - name: RAY_PORT
          value: "6379"
        - name: VENV_PATH
          value: "/venv/rlinf"
        - name: PYTHONUNBUFFERED
          value: "1"

  # ─── workers ───
  - replicaCount: 2
    machineSpec:
      machineType: a3-ultragpu-8g
      acceleratorType: NVIDIA_H200_141GB
      acceleratorCount: 8
      reservationAffinity:
        reservationAffinityType: SPECIFIC_RESERVATION
        key: compute.googleapis.com/reservation-name
        values:
          - projects/autel-ai-physical-spat-intel/zones/europe-west4-a/reservations/reservation-20260422-033135
    diskSpec:
      bootDiskType: hyperdisk-balanced
      bootDiskSizeGb: 1000
    containerSpec:
      imageUri: europe-west4-docker.pkg.dev/autel-ai-physical-spat-intel/rlinf/rlinf-aupi-dev:260702
      command: ["/bin/bash"]
      args: ["/gcs/physical-ai-data-eu/rlinf/demo4/pushdoor_bootstrap.sh"]
      env:
        - name: GCS_BUCKET
          value: "physical-ai-data-eu"
        - name: EXP_NAME
          value: "pi05_pushdoor_au_tst1_3node"
        - name: RAY_PORT
          value: "6379"
        - name: VENV_PATH
          value: "/venv/rlinf"
        - name: PYTHONUNBUFFERED
          value: "1"

scheduling:
  timeout: 604800s
  restartJobOnWorkerRestart: false
```

---

## 10. 步骤八：一键提交

创建 `b/gcp/demo4/pushdoor_submit.sh`：

```bash
#!/usr/bin/env bash
# =============================================================================
# 一键提交（demo4）：打包 RLinf + aupi05 -> 上传 GCS -> 提交 Vertex 作业。
#
# 前置：
#   * dev 镜像已 push 到 AR（§3）
#   * 模型 + norm_stats 已在 GCS（§5）
#   * 数据集已在 GCS（§6）
# =============================================================================
set -euo pipefail

PROJECT_ID="${PROJECT_ID:-autel-ai-physical-spat-intel}"
REGION="${REGION:-europe-west4}"
GCS_BUCKET="${GCS_BUCKET:-physical-ai-data-eu}"
DISPLAY_NAME="${DISPLAY_NAME:-rlinf-aupi-pushdoor-tst1-3node}"
AUPI_DIR="${AUPI_DIR:-/home/physical/SRC/Robot/aupi05}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
REPO_NAME="$(basename "${REPO_ROOT}")"
AUPI_NAME="$(basename "${AUPI_DIR}")"
GCS_ROOT="gs://${GCS_BUCKET}/rlinf"
TAR="/tmp/RLinf-code-demo4.tar.gz"
AUPI_TAR="/tmp/aupi05-code-demo4.tar.gz"

gcloud config set project "${PROJECT_ID}" >/dev/null

# 1. 打包 RLinf
echo ">> Packaging RLinf from ${REPO_ROOT}"
tar -czf "${TAR}" \
  --exclude-vcs \
  --exclude-vcs-ignores \
  --exclude='*/.git' --exclude='*/.git/*' \
  --exclude='*/__pycache__' --exclude='*.pyc' \
  --exclude='*.pdf' \
  --exclude="${REPO_NAME}/b/d" \
  --exclude="${REPO_NAME}/docs" \
  --exclude="${REPO_NAME}/.venv" \
  --exclude="${REPO_NAME}/logs" \
  --exclude="${REPO_NAME}/results" \
  --exclude="${REPO_NAME}/examples/au/pi/_ckpt" \
  --exclude="${REPO_NAME}/examples/au/pi/_out" \
  --exclude="${REPO_NAME}/tests_au/example/libero/_ckpt" \
  --exclude="${REPO_NAME}/tests_au/example/libero/_out" \
  --exclude="${REPO_NAME}/tests_au/example/pushdoor/_ckpt" \
  --exclude="${REPO_NAME}/tests_au/example/pushdoor/_out" \
  -C "$(dirname "${REPO_ROOT}")" "${REPO_NAME}"
echo ">> RLinf package: $(du -h "${TAR}" | cut -f1)"

# 2. 打包 aupi05
echo ">> Packaging aupi05 from ${AUPI_DIR}"
tar -czf "${AUPI_TAR}" \
  --exclude-vcs \
  --exclude-vcs-ignores \
  --exclude='*/.git' --exclude='*/.git/*' \
  --exclude='*/__pycache__' --exclude='*.pyc' \
  --exclude="${AUPI_NAME}/.venv" \
  --exclude="${AUPI_NAME}/checkpoints" \
  --exclude="${AUPI_NAME}/assets" \
  -C "$(dirname "${AUPI_DIR}")" "${AUPI_NAME}"
echo ">> aupi05 package: $(du -h "${AUPI_TAR}" | cut -f1)"

# 3. 上传到 GCS
echo ">> Uploading to ${GCS_ROOT}"
gcloud storage cp "${SCRIPT_DIR}/pushdoor_bootstrap.sh" "${GCS_ROOT}/demo4/pushdoor_bootstrap.sh"
gcloud storage cp "${TAR}"                              "${GCS_ROOT}/code/RLinf.tar.gz"
gcloud storage cp "${AUPI_TAR}"                         "${GCS_ROOT}/code/aupi05.tar.gz"

# 4. 提交 Vertex 作业
echo ">> Submitting Vertex AI job: ${DISPLAY_NAME} in ${REGION}"
gcloud ai custom-jobs create \
  --region="${REGION}" \
  --display-name="${DISPLAY_NAME}" \
  --config="${SCRIPT_DIR}/pushdoor_vertex_3node.yaml"

echo ">> Submitted. Track: Vertex AI > Training > Custom jobs"
```

执行提交：

```bash
cd /home/physical/SRC/RL/RLinf
bash b/gcp/demo4/pushdoor_submit.sh
```

---

## 11. 步骤九：监控 / checkpoint / 续训

### 监控

```bash
# 流式日志（实时）
gcloud ai custom-jobs stream-logs <JOB_ID> --region=europe-west4

# Cloud Logging 过滤
#   resource.type="ml_job"  resource.labels.job_id="<JOB_ID>"
#   textPayload:"[bootstrap]"     # 初始化阶段
#   textPayload:"train/loss"      # 训练损失

# TensorBoard（事件经 FUSE 实时写到 GCS）
tensorboard --logdir "gs://${GCS_BUCKET}/rlinf/runs/${EXP_NAME}" --port 6006
```

### checkpoint 结构

```
gs://.../rlinf/runs/<EXP_NAME>/<EXP_NAME>/checkpoints/global_step_<N>/actor/
├── model_state_dict/    # 训练权重
├── dcp_checkpoint/      # optimizer state（可续训）
└── ema.pt               # EMA 权重（推理/评测用这个）
```

### 续训

在训练配置中指定 `runner.resume_dir`：

```bash
python examples/sft/train_vla_sft_au.py \
  --config-path ... --config-name ... \
  runner.resume_dir="/gcs/.../checkpoints/global_step_25"
```

或在 bootstrap 的训练命令行追加 `runner.resume_dir=...`。

---

## 12. 常见故障排查

| 现象 | 原因 | 处理 |
|------|------|------|
| `ModuleNotFoundError: openpi` / `No module named 'rlinf'` | 源码没解包到 editable 精确路径 | 确认 bootstrap 把两个 tar 分别解包到 `/workspace/RLinf`、`/workspace/aupi05`；确认 submit 已上传 `code/aupi05.tar.gz` |
| `transformers_replace is not installed correctly` | 补丁未打进 transformers | 确认 bootstrap 4b 成功执行；手动：`cp -r /workspace/aupi05/src/openpi/models_pytorch/transformers_replace/* $(python -c 'import os,transformers;print(os.path.dirname(transformers.__file__))')/` |
| `source: /opt/venv/reason/... 不存在` | venv 路径用了 demo3 的旧路径 | dev 镜像 venv 是 `/venv/rlinf`，确认 `VENV_PATH=/venv/rlinf` |
| `undefined symbol: ...c10::Error...`（flash_attn） | flash-attn 与 torch ABI 不匹配 | 镜像内 `pip uninstall -y flash-attn`（退回 sdpa）后重 push |
| `ImportError: libX11.so.6` | DataLoader 缺 X11 库 | `apt-get install -y libx11-6 libxext6 libxrender1 libsm6 libgl1` |
| 作业卡 40+ 分钟无输出 | 24 rank 并发 FUSE mmap 读权重 | 确认 bootstrap 4c（stage 到本地盘）成功执行 |
| `429 Too Many Requests` | 多 rank 访问 HF | 已由 `HF_HUB_OFFLINE=1` 规避；确认数据完整在 `DATASET_ROOT` |
| `mat1 and mat2 ... Float vs BFloat16` | FSDP bf16 与 fp32 动作头冲突 | 保持 `mixed_precision` 全 null + `fp32_master_weights: false` |
| `global_batch_size ... not divisible` | 批大小整除不满足 | `global_batch_size % (micro_batch × 8 × num_nodes) == 0` |
| `INVALID_ARGUMENT ... hyperdisk-balanced` | H200 机型强制盘型 | 已配置 `hyperdisk-balanced` |
| norm_stats 找不到 | `{model}/rlinf/pushdoor_open0622/norm_stats.json` 缺失 | 按 §5.2 重算并 rsync 到 GCS |
| Vertex 拉镜像失败 | 用了本地镜像名 | `imageUri` 必须是 AR 全路径，且已 `docker push` |
| Ray worker 600s 超时未连上 head | 节点间网络不通 | 检查 Vertex 日志确认所有节点都启动了；检查 CLUSTER_SPEC 是否正确注入 |

---

## 13. 首次运行校验清单

1. **镜像已推送 AR**：`gcloud artifacts docker images list .../rlinf | grep aupi-dev` 能看到 `260702`。
2. **两份源码 tar 都已上传**：`gs://.../rlinf/code/RLinf.tar.gz` 与 `gs://.../rlinf/code/aupi05.tar.gz`。
3. **bootstrap 已上传**：`gs://.../rlinf/demo4/pushdoor_bootstrap.sh`。
4. **模型 + norm_stats**：`gs://.../models/pi05_pushdoor_r1pro_pt/{model.safetensors,rlinf/pushdoor_open0622/norm_stats.json}`。
5. **数据集完整**：`${DATASET_GCS}/{data,videos,meta}` 齐全，`meta/episodes.jsonl` 有 12 条。
6. **日志关键里程碑**：
   - `NODE_RANK=0/1/2` — 三节点都启动
   - `transformers_replace OK` — 补丁生效
   - `Model staged locally` — 权重 stage 成功
   - 3 节点全部 join Ray
   - `train/loss` 开始下降
   - checkpoint 按 `save_interval` 生成 `ema.pt`
7. **批大小整除**：`384 % (16 × 24) == 0`。

> **参考基线**：单机 8 卡示例 [`examples/au/pi/`](../../../examples/au/pi/)（`run_train.sh`，batch 8 / 50 步冒烟）可作为多机结果的对照。

---

*(操作手册结束)*

---

## 附录：实际提交记录 — 错误与修复（2026-07-03）

以下是将 demo4 作业实际提交到 GCP 训练时遇到的所有错误，按时间顺序记录。

### Error 1：`gcloud: command not found`

- **Job ID**：`2008330418461343744`
- **现象**：bootstrap.sh 在 step 4c 执行 `gcloud storage rsync` 将模型权重从 GCS 复制到本地盘时报错 `gcloud: command not found`，所有 3 个节点均失败。
- **根因**：dev 镜像 `rlinf-aupi-dev:260702` 基于 `nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04`，没有预装 `gcloud` CLI。bootstrap.sh 中使用 `gcloud storage rsync -r "gs://..." ...` 来 stage 模型权重，但容器内没有 `gcloud` 命令。
- **修复**：
  - `pushdoor_bootstrap.sh`：将 `MODEL_GCS_URI="gs://..."` 改为 `MODEL_FUSE_DIR="/gcs/..."` （使用 GCS FUSE 挂载路径），将 `gcloud storage rsync -r "${MODEL_GCS_URI}" "${LOCAL_MODEL_DIR}"` 改为 `cp -r "${MODEL_FUSE_DIR}/." "${LOCAL_MODEL_DIR}/"` 。

### Error 2：`ImportError: cannot import name 'runtime_version' from 'google.protobuf'`

- **Job ID**：`7426582982653116416`
- **现象**：训练脚本在初始化 TensorBoard logger 时失败，报 `ImportError: cannot import name 'runtime_version' from 'google.protobuf'`。model loading、norm_stats、dataset loading 均正常通过，仅在创建 SummaryWriter 时崩溃。
- **根因**：镜像中的 `protobuf` 版本太旧（不含 `runtime_version` 模块，该模块在 protobuf >=5.26 引入），而 `tensorboard` 的 proto 生成代码需要该模块。multinode YAML 中配置了 `logger_backends: ["tensorboard"]`。
- **修复**：
  - `pushdoor_bootstrap.sh`：曾尝试添加 `pip install 'protobuf>=5.27,<6'`，但引发 Error 3。

### Error 3：`VersionError: Detected incompatible Protobuf Gencode/Runtime versions`

- **Job ID**：`567178537702785024`
- **现象**：将 protobuf 升级到 5.29.6 后，tensorboard 报 `VersionError: gencode 6.31.1 runtime 5.29.6`。
- **根因**：镜像中的 tensorboard 包的 proto 生成代码（gencode）是用 protobuf 6.31.1 编译的，要求运行时 protobuf >=6.31.1。安装 5.x 版本不满足。
- **修复**：
  - `pushdoor_bootstrap.sh`：将 protobuf 升级约束改为 `pip install 'protobuf>=6.31'`，但引发 Error 4。

### Error 4：`AttributeError: module 'ml_dtypes' has no attribute 'float4_e2m1fn'`

- **Job ID**：`6331786060737019904`（第一轮，Vertex 自动重试）
- **现象**：protobuf 升级到 6.x 后，tensorboard 尝试 import tensorflow 作为后端，tensorflow 内部引用 `ml_dtypes.float4_e2m1fn` 属性不存在。
- **根因**：tensorboard 的 `compat/__init__.py` 优先尝试 import tensorflow，虽然 `USE_TF=0` 环境变量可以阻止 transformers 加载 TF，但 tensorboard 自身不受 `USE_TF` 控制。镜像中安装了 tensorflow 但其 `ml_dtypes` 版本与 TF 不兼容（`float4_e2m1fn` 是 ml_dtypes >=0.5.0 新增的类型），形成级联兼容性问题。
- **修复**：
  - `pushdoor_sft_pi05_au_multinode.yaml`：将 `logger_backends: ["tensorboard"]` 改为 `logger_backends: []`，彻底跳过 tensorboard logger。
  - `pushdoor_bootstrap.sh`：移除之前添加的 protobuf 升级步骤（不再需要）。

### 最终成功

- **Job ID**：`8286348299015815168`
- **结果**：50/50 步训练完成，耗时 7 分 9 秒（8.60s/step），loss=0.123，grad_norm=0.77，EMA checkpoint 已保存。
- **输出路径**：`gs://physical-ai-data-eu/rlinf/runs/pi05_pushdoor_au_tst1_3node/`

### 修改文件汇总

| 文件 | 修改内容 |
|------|----------|
| `b/gcp/demo4/pushdoor_bootstrap.sh` | `gcloud storage rsync` → `cp -r`（FUSE 路径）；`python3` → `python`（一致性）；添加 `${LOCAL_AUPI}/src` 到 PYTHONPATH |
| `b/gcp/demo4/pushdoor_sft_pi05_au_multinode.yaml` | `logger_backends: ["tensorboard"]` → `logger_backends: []`；补末尾换行符 |
