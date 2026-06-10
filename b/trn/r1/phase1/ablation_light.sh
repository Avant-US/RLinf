#!/usr/bin/env bash
# Phase 1 Ablation: light 增强 (1500 steps)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../prepare/env.sh"

cd "${REPO_ROOT}"

echo "============================================================"
echo "  Phase 1: Ablation — augmentation=light"
echo "============================================================"

bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  cluster.component_placement.actor=0-7 \
  runner.max_steps=1500 \
  runner.save_interval=999 \
  runner.log_interval=5 \
  actor.micro_batch_size=2 \
  actor.global_batch_size=16 \
  actor.optim.min_lr=1e-6 \
  data.skip_padding_as_possible=true \
  data.processor.augmentation_preset=light \
  runner.logger.experiment_name=r1_pro_p1_light \
  runner.logger.log_path=${BTLOG_ROOT}/phase1_light

echo "[phase1] augmentation=light 完成。"
