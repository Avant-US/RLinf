#!/usr/bin/env bash
# Phase 0 Smoke Test: gbs=8, micro=1 (50 steps)
# 验证: 训练能启动, loss 3→0.5, 无 NaN, LR step 2 达 1e-4
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../prepare/env.sh"

cd "${REPO_ROOT}"

echo "============================================================"
echo "  Phase 0: Smoke Test (gbs=8, micro=1)"
echo "============================================================"

bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  cluster.component_placement.actor=0-7 \
  runner.max_steps=50 \
  runner.save_interval=999 \
  runner.log_interval=1 \
  actor.micro_batch_size=1 \
  actor.global_batch_size=8 \
  actor.optim.min_lr=1e-6 \
  data.skip_padding_as_possible=true \
  runner.logger.experiment_name=phase0_gbs8 \
  runner.logger.log_path=${BTLOG_ROOT}/phase0_gbs8

echo
echo "[phase0] gbs=8 smoke 完成。请检查:"
echo "  1. loss 从 ~3.0 下降到 ~0.5"
echo "  2. train/learning_rate 在 step 2 达到 1e-4"
echo "  3. 无 NaN/Inf"
