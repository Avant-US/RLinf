#!/usr/bin/env bash
# Phase 0 Smoke Test: gbs=16, micro=2 (50 steps)
# 验证: micro_batch=2 不 OOM, 记录 time/step
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../prepare/env.sh"

cd "${REPO_ROOT}"

echo "============================================================"
echo "  Phase 0: Smoke Test (gbs=16, micro=2)"
echo "============================================================"

bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  cluster.component_placement.actor=0-7 \
  runner.max_steps=50 \
  runner.save_interval=999 \
  runner.log_interval=1 \
  actor.micro_batch_size=2 \
  actor.global_batch_size=16 \
  actor.optim.min_lr=1e-6 \
  data.skip_padding_as_possible=true \
  runner.logger.experiment_name=phase0_gbs16 \
  runner.logger.log_path=${BTLOG_ROOT}/phase0_gbs16

echo
echo "[phase0] gbs=16 smoke 完成。请检查:"
echo "  1. 无 OOM"
echo "  2. 记录 time/step（稳态应 ~1.3s）"
echo "  3. 如果 OOM，Phase 1/2 需回退到 micro=1, gbs=8"
