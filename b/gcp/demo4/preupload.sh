#!/usr/bin/env bash
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
export MODEL_LOCAL="/mnt/r/CKPT/VLA/DEMO/pi05_pushdoor_tst1"
export MODEL_GCS="${GCS_ROOT}/models/pi05_pushdoor_r1pro_pt"

# ---- 作业 ----
export EXP_NAME="pi05_pushdoor_au_tst1_3node"


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

# 同步模型和norm stat到 GCS
gcloud storage rsync -r "${MODEL_LOCAL}" "${MODEL_GCS}"

# 校验
gcloud storage ls "${MODEL_GCS}/model.safetensors"
gcloud storage ls "${MODEL_GCS}/rlinf/pushdoor_open0622/norm_stats.json"

# 保持 LeRobot v3 原始布局整体上传：

gcloud storage rsync -r "${DATASET_LOCAL}" "${DATASET_GCS}"

# 校验关键文件. 容器内该数据集经 FUSE 挂载后路径为 `/gcs/physical-ai-data-eu/DATA/SKILL/pushdoor/0622_lerobot_data_tst1
gcloud storage ls "${DATASET_GCS}/meta/info.json"
gcloud storage ls "${DATASET_GCS}/data/chunk-000/" | head
gcloud storage ls "${DATASET_GCS}/videos/observation.images.head_rgb/chunk-000/" | head

