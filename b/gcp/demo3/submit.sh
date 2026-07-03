#!/usr/bin/env bash
# =============================================================================
# One-click submit for RLinf pi0.5 (openpi_au) LIBERO SFT on Vertex AI:
#   package RLinf source -> upload bootstrap.sh + code tarball to GCS -> submit job.
#
# Prerequisites (see gcp_rlinf_pi05_au.md):
#   * Custom openpi_au image built & pushed to Artifact Registry (Step 1).
#   * Model (model.safetensors + config.json + physical-intelligence/libero/norm_stats.json)
#     and LIBERO LeRobot dataset staged to GCS (Step 2).
# =============================================================================
set -euo pipefail

# ----------------------------- params (override via env) ---------------------
PROJECT_ID="${PROJECT_ID:-autel-ai-physical-spat-intel}"
REGION="${REGION:-europe-west4}"
GCS_BUCKET="${GCS_BUCKET:-physical-ai-data-eu}"
DISPLAY_NAME="${DISPLAY_NAME:-rlinf-pi05-au-3node}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
REPO_NAME="$(basename "${REPO_ROOT}")"
GCS_ROOT="gs://${GCS_BUCKET}/rlinf"
TAR="/tmp/RLinf-code-pi05au.tar.gz"

gcloud config set project "${PROJECT_ID}" >/dev/null

# ------------------- 1. package repo source (exclude big/irrelevant) ---------
# bootstrap.sh unpacks with --strip-components=1, so the archive's top level is the
# repo dir name. Exclude the local example's staged weights/outputs (huge) and docs.
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
  -C "$(dirname "${REPO_ROOT}")" "${REPO_NAME}"
echo ">> Package size: $(du -h "${TAR}" | cut -f1)"

# --------------------- 2. upload bootstrap + code tarball to GCS -------------
echo ">> Uploading bootstrap.sh and code tarball to ${GCS_ROOT}"
gcloud storage cp "${SCRIPT_DIR}/bootstrap.sh" "${GCS_ROOT}/demo3/bootstrap.sh"
gcloud storage cp "${TAR}"                      "${GCS_ROOT}/code/RLinf.tar.gz"

# ------------------------------ 3. submit job --------------------------------
echo ">> Submitting Vertex AI custom job (${DISPLAY_NAME}) in ${REGION}"
gcloud ai custom-jobs create \
  --region="${REGION}" \
  --display-name="${DISPLAY_NAME}" \
  --config="${SCRIPT_DIR}/vertex_pi05_au_3node.yaml"

echo ">> Submitted. Track it in Vertex AI > Training > Custom jobs."
