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