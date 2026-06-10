#!/usr/bin/env bash
# Phase 2: 主训练 — 187200 steps (50 epoch, gbs=16, medium 增强)
# 预估耗时: ~68h (~2.8 天)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../prepare/env.sh"

cd "${REPO_ROOT}"

AUG_PRESET="${1:-medium}"

echo "============================================================"
echo "  Phase 2: 主训练 (E=50, gbs=16, aug=${AUG_PRESET})"
echo "  预估: ~187,200 steps, ~68h"
echo "============================================================"

bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  cluster.component_placement.actor=0-7 \
  runner.max_steps=187200 \
  runner.save_interval=5000 \
  runner.log_interval=10 \
  actor.micro_batch_size=2 \
  actor.global_batch_size=16 \
  actor.optim.min_lr=1e-6 \
  data.skip_padding_as_possible=true \
  data.processor.augmentation_preset=${AUG_PRESET} \
  runner.logger.experiment_name=r1_pro_main_e50_${AUG_PRESET} \
  runner.logger.log_path=${BTLOG_ROOT}/phase2_main_${AUG_PRESET}

echo
echo "[phase2] 主训练完成。"
echo "  Checkpoints: ${BTLOG_ROOT}/phase2_main_${AUG_PRESET}/checkpoints/"
echo "  TensorBoard: tensorboard --logdir ${BTLOG_ROOT}/phase2_main_${AUG_PRESET} --port 6006"
