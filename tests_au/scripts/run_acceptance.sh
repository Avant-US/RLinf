#!/usr/bin/env bash
# One-click acceptance: L1 forward numerical alignment -> L2 training-curve alignment
# -> L3 inference smoke. See rlinfpi_accept_1.md §G.
#
# Required env:
#   PY        : python interpreter (default: python)
#   JAX_CKPT  : openpi pi05_base JAX checkpoint dir (contains params/)
#   PT_CKPT   : converted openpi pi05_base PyTorch dir (contains model.safetensors)
# Optional env:
#   DATA_ROOT : LeRobot LIBERO dir (default: HF cache path)
#   NUM_SAMPLES, NUM_STEPS, PRECISION, RUN_EVAL
set -euo pipefail

PY="${PY:-python}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SUBSET="$HERE/_data/libero_subset"
OUT="$HERE/_out"
NUM_SAMPLES="${NUM_SAMPLES:-4}"
NUM_STEPS="${NUM_STEPS:-8}"
PRECISION="${PRECISION:-bf16}"
RUN_EVAL="${RUN_EVAL:-0}"
DATA_ROOT="${DATA_ROOT:-$HOME/.cache/huggingface/lerobot/physical-intelligence/libero}"

: "${JAX_CKPT:?set JAX_CKPT to pi05_base JAX dir}"
: "${PT_CKPT:?set PT_CKPT to converted PyTorch dir}"

cd "$REPO"
mkdir -p "$OUT"

echo "===================== [0/3] extract LIBERO subset ====================="
"$PY" "$HERE/extract_libero_subset.py" \
    --data_root "$DATA_ROOT" --num_samples "$NUM_SAMPLES" --out_dir "$SUBSET"

echo "===================== [1/3] L1 forward alignment ($PRECISION) ====================="
"$PY" "$HERE/forward_align.py" --side all \
    --subset_path "$SUBSET" --jax_config pi05_libero \
    --jax_ckpt "$JAX_CKPT" --pt_ckpt "$PT_CKPT" \
    --precision "$PRECISION" --num_samples "$NUM_SAMPLES" \
    --out_report "$OUT/forward_align_${PRECISION}.json"

echo "===================== [2/3] L2 training-curve alignment ====================="
"$PY" "$HERE/train_compare.py" --side all \
    --subset_path "$SUBSET" --jax_config pi05_libero \
    --jax_ckpt "$JAX_CKPT" --pt_ckpt "$PT_CKPT" \
    --num_steps "$NUM_STEPS" --batch_size 2 --warmup 3 \
    --out_report "$OUT/train_compare_report.json" \
    --out_plot "$OUT/curves.png"

echo "===================== [3/3] L3 inference smoke ====================="
"$PY" "$HERE/eval_compare.py" \
    --rlinf_ckpt "$PT_CKPT" --mode smoke \
    --subset_path "$SUBSET" --num_samples "$NUM_SAMPLES" \
    --out_report "$OUT/eval_report.json"

if [ "$RUN_EVAL" = "1" ]; then
    echo "===================== [3b] L3 full LIBERO eval ====================="
    "$PY" "$HERE/eval_compare.py" \
        --rlinf_ckpt "$PT_CKPT" --mode full \
        --suite spatial --episodes 20 --baseline 0.771 \
        --out_report "$OUT/eval_full_report.json"
fi

echo "ACCEPTANCE DONE — reports in $OUT"
