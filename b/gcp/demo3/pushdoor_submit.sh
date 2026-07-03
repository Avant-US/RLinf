#!/usr/bin/env bash
# =============================================================================
# One-click submit for RLinf pi0.5 (openpi_au) pushdoor (r1_pro) SFT smoke test on
# Vertex AI: package RLinf source -> upload bootstrap.sh + code tarball to GCS ->
# submit job. Mirrors submit.sh (LIBERO), pointed at the pushdoor bootstrap/config.
#
# Prerequisites (see gcp_rlinf_pi05_au.md's pushdoor section):
#   * Custom openpi_au image built & pushed to Artifact Registry (shared with LIBERO;
#     built once via docker build/push, see manual for the Cloud Build source-upload
#     org-policy workaround).
#   * Converted checkpoint (model.safetensors + config.json) and pushdoor
#     rlinf/pushdoor_open0622/norm_stats.json staged under
#     gs://<bucket>/rlinf/models/pi05_pushdoor_r1pro_pt/ (see convert_r1pro_ckpt.py +
#     run_norm_stats.sh, uploaded via `gcloud storage rsync`).
#   * pushdoor LeRobot dataset already present at
#     gs://<bucket>/DATA/SKILL/pushdoor/0622_lerobot_data/ (uploaded separately; this
#     script does not re-upload it).
# =============================================================================
set -euo pipefail

# ----------------------------- params (override via env) ---------------------
PROJECT_ID="${PROJECT_ID:-autel-ai-physical-spat-intel}"
REGION="${REGION:-europe-west4}"
GCS_BUCKET="${GCS_BUCKET:-physical-ai-data-eu}"
DISPLAY_NAME="${DISPLAY_NAME:-rlinf-pi05-au-pushdoor-3node}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
REPO_NAME="$(basename "${REPO_ROOT}")"
GCS_ROOT="gs://${GCS_BUCKET}/rlinf"
TAR="/tmp/RLinf-code-pi05au-pushdoor.tar.gz"

gcloud config set project "${PROJECT_ID}" >/dev/null

# ------------------- 1. package repo source (exclude big/irrelevant) ---------
# pushdoor_bootstrap.sh unpacks with --strip-components=1, so the archive's top level
# is the repo dir name. Exclude both examples' staged weights/outputs (huge) and docs.
echo ">> Packaging repo from ${REPO_ROOT}"
tar -czf "${TAR}" \
  --exclude='*/.git' --exclude='*/.git/*' \
  --exclude='*/__pycache__' --exclude='*.pyc' \
  --exclude='*.pdf' \
  --exclude="${REPO_NAME}/b/d" \
  --exclude="${REPO_NAME}/docs" \
  --exclude="${REPO_NAME}/.venv" \
  --exclude="${REPO_NAME}/logs" \
  --exclude="${REPO_NAME}/results" \
  --exclude="${REPO_NAME}/tests_au/example/libero/_ckpt" \
  --exclude="${REPO_NAME}/tests_au/example/libero/_out" \
  --exclude="${REPO_NAME}/tests_au/example/pushdoor/_ckpt" \
  --exclude="${REPO_NAME}/tests_au/example/pushdoor/_out" \
  -C "$(dirname "${REPO_ROOT}")" "${REPO_NAME}"
echo ">> Package size: $(du -h "${TAR}" | cut -f1)"

# --------------------- 2. upload bootstrap + code tarball to GCS -------------
echo ">> Uploading pushdoor_bootstrap.sh and code tarball to ${GCS_ROOT}"
gcloud storage cp "${SCRIPT_DIR}/pushdoor_bootstrap.sh" "${GCS_ROOT}/demo3/pushdoor_bootstrap.sh"
gcloud storage cp "${TAR}"                              "${GCS_ROOT}/code/RLinf.tar.gz"

# ------------------------------ 3. submit job --------------------------------
echo ">> Submitting Vertex AI custom job (${DISPLAY_NAME}) in ${REGION}"
gcloud ai custom-jobs create \
  --region="${REGION}" \
  --display-name="${DISPLAY_NAME}" \
  --config="${SCRIPT_DIR}/pushdoor_vertex_3node.yaml"

echo ">> Submitted. Track it in Vertex AI > Training > Custom jobs."
