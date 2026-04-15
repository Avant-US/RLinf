#!/bin/bash
#
# Random-37 smoke test for `lerobot-train` with policy.type=str_groot.
# It will:
#   1) Randomly sample 37 episodes from HuggingFaceVLA/libero
#   2) Save sampled IDs to bt/str_groot_1/test_episodes_37.random.json
#   3) Run a short 2-step training via lerobot-train
#
# Usage:
#   ./bt/str_groot_1/test_random37_lerobot_train.sh
#
# Optional env overrides:
#   DATASET_REPO, DATASET_ROOT, SAMPLE_SIZE, SAMPLE_SEED
#   STEPS, BATCH_SIZE, NUM_WORKERS, LOG_FREQ
#   POLICY_DEVICE, STARVLA_CHECKPOINT, BASE_VLM
#

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

# Use repo-local HF caches by default to avoid permission issues.
# Override with HF_HOME_OVERRIDE if you want another cache location.
export HF_HOME="${HF_HOME_OVERRIDE:-$REPO_ROOT/.hfhome}"
export HF_DATASETS_CACHE="${HF_DATASETS_CACHE_OVERRIDE:-$HF_HOME/datasets}"
export HUGGINGFACE_HUB_CACHE="${HUGGINGFACE_HUB_CACHE_OVERRIDE:-$HF_HOME/hub}"
mkdir -p "$HF_HOME" "$HF_DATASETS_CACHE" "$HUGGINGFACE_HUB_CACHE"

DATASET_REPO="${DATASET_REPO:-HuggingFaceVLA/libero}"
DATASET_ROOT="${DATASET_ROOT:-}"
SAMPLE_SIZE="${SAMPLE_SIZE:-37}"
SAMPLE_SEED="${SAMPLE_SEED:-20260328}"
EPISODES_FILE="${EPISODES_FILE:-bt/str_groot_1/test_episodes_37.random.json}"

STEPS="${STEPS:-2}"
BATCH_SIZE="${BATCH_SIZE:-1}"
NUM_WORKERS="${NUM_WORKERS:-0}"
LOG_FREQ="${LOG_FREQ:-1}"
SAVE_FREQ="${SAVE_FREQ:-999}"

# For libero we usually skip the padded dim at index 6.
STATE_INDICES="${STATE_INDICES:-[0,1,2,3,4,5,7]}"
POLICY_DEVICE="${POLICY_DEVICE:-cuda:0}"
STARVLA_CHECKPOINT="${STARVLA_CHECKPOINT:-}"
BASE_VLM="${BASE_VLM:-}"

OUTPUT_ROOT="${OUTPUT_ROOT:-outputs/bt/str_groot_1}"
JOB_NAME="${JOB_NAME:-str_groot_libero_random37_cli}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${OUTPUT_DIR:-${OUTPUT_ROOT}/${RUN_ID}}"

if [ -x "$REPO_ROOT/lerobot-venv/bin/python" ]; then
  PYTHON_BIN="$REPO_ROOT/lerobot-venv/bin/python"
else
  PYTHON_BIN="python3"
fi

if command -v lerobot-train >/dev/null 2>&1; then
  TRAIN_CMD=("lerobot-train")
elif [ -x "$REPO_ROOT/lerobot-venv/bin/lerobot-train" ]; then
  TRAIN_CMD=("$REPO_ROOT/lerobot-venv/bin/lerobot-train")
else
  TRAIN_CMD=("$PYTHON_BIN" -m lerobot.scripts.lerobot_train)
fi

echo "==> Sampling ${SAMPLE_SIZE} episodes from ${DATASET_REPO} (seed=${SAMPLE_SEED})"
EPISODES_PAYLOAD="$(
  DATASET_REPO="$DATASET_REPO" \
  DATASET_ROOT="$DATASET_ROOT" \
  SAMPLE_SIZE="$SAMPLE_SIZE" \
  SAMPLE_SEED="$SAMPLE_SEED" \
  EPISODES_FILE="$EPISODES_FILE" \
  "$PYTHON_BIN" - <<'PY'
import json
import os
import random
from pathlib import Path

from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata

repo = os.environ["DATASET_REPO"]
root = os.environ.get("DATASET_ROOT") or None
sample_size = int(os.environ["SAMPLE_SIZE"])
sample_seed = int(os.environ["SAMPLE_SEED"])
episodes_file = Path(os.environ["EPISODES_FILE"])

meta = LeRobotDatasetMetadata(repo_id=repo, root=root)
total_episodes = int(meta.total_episodes)
if sample_size > total_episodes:
    raise ValueError(f"sample_size({sample_size}) > total_episodes({total_episodes})")

episodes = sorted(random.Random(sample_seed).sample(range(total_episodes), sample_size))

episodes_file.parent.mkdir(parents=True, exist_ok=True)
episodes_file.write_text(
    json.dumps(
        {
            "dataset_repo": repo,
            "dataset_root": root,
            "total_episodes": total_episodes,
            "sample_size": sample_size,
            "sample_seed": sample_seed,
            "episodes": episodes,
        },
        ensure_ascii=False,
        indent=2,
    ),
    encoding="utf-8",
)

print(json.dumps(episodes, separators=(",", ":")))
PY
)"

echo "==> Episodes saved to ${EPISODES_FILE}"
echo "==> Output dir: ${OUTPUT_DIR}"
echo "==> Running short lerobot-train smoke test..."

CMD=(
  "${TRAIN_CMD[@]}"
  "--policy.type=str_groot"
  "--policy.push_to_hub=false"
  "--policy.device=${POLICY_DEVICE}"
  "--policy.freeze_vlm=true"
  "--policy.tune_vlm=false"
  "--policy.tune_action_head=true"
  "--policy.state_indices=${STATE_INDICES}"
  "--policy.starvla_checkpoint=${STARVLA_CHECKPOINT}"
  "--dataset.repo_id=${DATASET_REPO}"
  "--dataset.episodes=${EPISODES_PAYLOAD}"
  "--steps=${STEPS}"
  "--batch_size=${BATCH_SIZE}"
  "--num_workers=${NUM_WORKERS}"
  "--eval_freq=0"
  "--log_freq=${LOG_FREQ}"
  "--save_checkpoint=false"
  "--save_freq=${SAVE_FREQ}"
  "--wandb.enable=false"
  "--output_dir=${OUTPUT_DIR}"
  "--job_name=${JOB_NAME}"
)

if [ -n "$DATASET_ROOT" ]; then
  CMD+=("--dataset.root=${DATASET_ROOT}")
fi
if [ -n "$BASE_VLM" ]; then
  CMD+=("--policy.base_vlm=${BASE_VLM}")
fi

printf '  %q' "${CMD[@]}"
echo
"${CMD[@]}"

echo "==> Smoke test finished successfully."
