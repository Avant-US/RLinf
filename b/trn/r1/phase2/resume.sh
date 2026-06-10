#!/usr/bin/env bash
# Phase 2: 断点续训
# 用法: bash b/trn/r1/phase2/resume.sh <checkpoint_dir>
# 示例: bash b/trn/r1/phase2/resume.sh /mnt/r/tmp/fw_train/r1_pro/phase2_main_medium/checkpoints/global_step_50000
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../prepare/env.sh"

if [ -z "${1:-}" ]; then
    echo "用法: $0 <checkpoint_dir>"
    echo "示例: $0 ${BTLOG_ROOT}/phase2_main_medium/checkpoints/global_step_50000"
    exit 1
fi

RESUME_DIR="$1"
if [ ! -d "${RESUME_DIR}" ]; then
    echo "ERROR: Checkpoint 目录不存在: ${RESUME_DIR}"
    exit 1
fi

AUG_PRESET="${2:-medium}"

cd "${REPO_ROOT}"

echo "============================================================"
echo "  Phase 2: 断点续训"
echo "  Resume from: ${RESUME_DIR}"
echo "============================================================"

bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  cluster.component_placement.actor=0-7 \
  runner.max_steps=187200 \
  runner.save_interval=5000 \
  runner.log_interval=10 \
  runner.resume_dir=${RESUME_DIR} \
  actor.micro_batch_size=2 \
  actor.global_batch_size=16 \
  actor.optim.min_lr=1e-6 \
  data.skip_padding_as_possible=true \
  data.processor.augmentation_preset=${AUG_PRESET} \
  runner.logger.experiment_name=r1_pro_main_e50_${AUG_PRESET}_resumed \
  runner.logger.log_path=${BTLOG_ROOT}/phase2_main_${AUG_PRESET}

echo "[phase2] 续训完成。"
