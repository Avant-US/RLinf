#!/usr/bin/env bash
# Recompute LIBERO normalization stats for the RLinf openpi_au pi0.5 example.
# Writes {MODEL_DIR}/physical-intelligence/libero/norm_stats.json.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PY="${PY:-/mnt/r/VENV/openpi_venv/bin/python}"
OPENPI_SRC="${OPENPI_SRC:-/home/physical/SRC/Robot/openpi05/src}"
MODEL_DIR="${RLINF_LIBERO_MODEL:-$HERE/_ckpt/pi05_base_pt}"
MAX_FRAMES="${MAX_FRAMES:-10000}"

export PYTHONPATH="${REPO}:${OPENPI_SRC}:${PYTHONPATH:-}"

cd "$REPO"
echo "[run_norm_stats] python=$PY"
echo "[run_norm_stats] output_dir=$MODEL_DIR  max_frames=$MAX_FRAMES"

"$PY" tests_au/example/libero/compute_norm_stats_au.py \
    --config_name pi05_libero \
    --repo_id physical-intelligence/libero \
    --output_dir "$MODEL_DIR" \
    --max_frames "$MAX_FRAMES" \
    --batch_size 64

echo "[run_norm_stats] done -> $MODEL_DIR/physical-intelligence/libero/norm_stats.json"
