#!/usr/bin/env bash
# Phase 1 Ablation: 无增强基线 (1500 steps)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../prepare/env.sh"

cd "${REPO_ROOT}"

echo "============================================================"
echo "  Phase 1: Ablation — augmentation=none (baseline)"
echo "============================================================"

bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  cluster.component_placement.actor=2-7 \
  runner.max_steps=1500 \
  runner.save_interval=1000 \
  runner.log_interval=100 \
  actor.micro_batch_size=2 \
  actor.global_batch_size=16 \
  actor.optim.min_lr=1e-6 \
  data.skip_padding_as_possible=true \
  runner.logger.experiment_name=r1_pro_p1_none \
  runner.logger.log_path=${BTLOG_ROOT}/phase1_none

echo "[phase1] augmentation=none 完成。"
