#!/usr/bin/env bash
# =============================================================================
# 一键提交：打包仓库源码 -> 上传 bootstrap.sh + 代码包到 GCS -> 提交 Vertex 作业。
# 前置：镜像已镜像到 AR；模型与数据已预存到 gs://$GCS_BUCKET/rlinf/{models,data}（见手册步骤一、二）。
# =============================================================================
set -euo pipefail

# ----------------------------- 集中参数（可改）-------------------------------
PROJECT_ID="${PROJECT_ID:-autel-ai-physical-spat-intel}"
REGION="${REGION:-europe-west4}"
GCS_BUCKET="${GCS_BUCKET:-physical-ai-data-eu}"
DISPLAY_NAME="${DISPLAY_NAME:-rlinf-sft-3node}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
REPO_NAME="$(basename "${REPO_ROOT}")"
GCS_ROOT="gs://${GCS_BUCKET}/rlinf"
TAR="/tmp/RLinf-code.tar.gz"

gcloud config set project "${PROJECT_ID}" >/dev/null

# ----------------------- 1. 打包仓库源码（排除大/无关文件）-------------------
# bootstrap.sh 解包时用 --strip-components=1，所以归档首层是仓库目录名。
echo ">> Packaging repo from ${REPO_ROOT}"
tar -czf "${TAR}" \
  --exclude='*/.git' --exclude='*/.git/*' \
  --exclude='*/__pycache__' --exclude='*.pyc' \
  --exclude='*.pdf' \
  --exclude="${REPO_NAME}/b/d" \
  --exclude="${REPO_NAME}/b/test" \
  --exclude="${REPO_NAME}/b/gcp/demo1" \
  --exclude="${REPO_NAME}/docs" \
  --exclude="${REPO_NAME}/.venv" \
  --exclude="${REPO_NAME}/logs" \
  --exclude="${REPO_NAME}/results" \
  --exclude="${REPO_NAME}/tests_au" \
  -C "$(dirname "${REPO_ROOT}")" "${REPO_NAME}"
echo ">> Package size: $(du -h "${TAR}" | cut -f1)"

# --------------------- 2. 上传 bootstrap 与代码包到 GCS ----------------------
echo ">> Uploading bootstrap.sh and code tarball to ${GCS_ROOT}"
gcloud storage cp "${SCRIPT_DIR}/bootstrap.sh" "${GCS_ROOT}/bootstrap.sh"
gcloud storage cp "${TAR}"                      "${GCS_ROOT}/code/RLinf.tar.gz"

# ------------------------------ 3. 提交作业 ----------------------------------
echo ">> Submitting Vertex AI custom job (${DISPLAY_NAME}) in ${REGION}"
gcloud ai custom-jobs create \
  --region="${REGION}" \
  --display-name="${DISPLAY_NAME}" \
  --config="${SCRIPT_DIR}/vertex_sft_3node.yaml"

echo ">> Submitted. Track it in the Vertex AI > Training > Custom jobs console."
