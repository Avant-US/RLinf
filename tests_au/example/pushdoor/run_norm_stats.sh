#!/usr/bin/env bash
# Compute pushdoor normalization stats for the RLinf openpi_au pi0.5 example.
# Writes {MODEL_DIR}/rlinf/pushdoor_open0622/norm_stats.json.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PY="${PY:-/mnt/r/VENV/openpi_venv/bin/python}"
OPENPI_SRC="${OPENPI_SRC:-/home/physical/SRC/Robot/openpi05/src}"
MODEL_DIR="${RLINF_PUSHDOOR_MODEL:-$HERE/_ckpt/pi05_r1pro_pt}"
REPO_ID="${RLINF_PUSHDOOR_DATA:-/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data}"
# The dataset has exactly 31 official episodes / 34084 frames total (see
# meta/episodes.jsonl). This is a "does it run" smoke test (not chasing accuracy):
# video decode is CPU-bound at ~30s/batch-of-64 with the default num_workers=2, so
# keep this small and raise num_workers to use the box's many cores instead.
MAX_FRAMES="${MAX_FRAMES:-1024}"
NUM_WORKERS="${NUM_WORKERS:-16}"
BATCH_SIZE="${BATCH_SIZE:-64}"

export PYTHONPATH="${REPO}:${OPENPI_SRC}:${PYTHONPATH:-}"
# Pure CPU/data-loading job (no model forward pass): pin to a single GPU so JAX
# (pulled in transitively via openpi.shared.normalize) doesn't preallocate its
# default ~76% memory fraction on all 8 visible devices.
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
# Force unbuffered stdout/stderr -- otherwise, when this script's output is piped
# (e.g. `| tail`) rather than attached to a tty, Python fully block-buffers
# print()/tqdm output and nothing appears until the process exits normally, which
# looks exactly like a hang if the run is long (it isn't: it's just buffering).
export PYTHONUNBUFFERED=1

cd "$REPO"
echo "[run_norm_stats] python=$PY"
echo "[run_norm_stats] repo_id=$REPO_ID"
echo "[run_norm_stats] output_dir=$MODEL_DIR  max_frames=$MAX_FRAMES  num_workers=$NUM_WORKERS"
echo "[run_norm_stats] CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES"

"$PY" -u tests_au/example/pushdoor/compute_norm_stats_au.py \
    --config_name pi05_pushdoor \
    --repo_id "$REPO_ID" \
    --output_dir "$MODEL_DIR" \
    --max_frames "$MAX_FRAMES" \
    --batch_size "$BATCH_SIZE" \
    --num_workers "$NUM_WORKERS"

echo "[run_norm_stats] done -> $MODEL_DIR/rlinf/pushdoor_open0622/norm_stats.json"
