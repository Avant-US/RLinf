#!/usr/bin/env bash
# Phase 1 Ablation: medium 增强 (1500 steps)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../prepare/env.sh"

RUNID=R1

export CUDA_VISIBLE_DEVICES=2,3,4,5,6,7

if [ ! -f "${VENV_PATH}/bin/activate" ]; then
    echo "ERROR: 虚拟环境不存在: ${VENV_PATH}"
    exit 1
fi
source "${VENV_PATH}/bin/activate"

export BTLOG_ROOT="/mnt/r/CKPT/VLA/FW/RUN/${RUNID}/"
if [ ! -d "${BTLOG_ROOT}" ]; then
    echo "创建日志目录: ${BTLOG_ROOT}"
    mkdir -p ${BTLOG_ROOT}
fi

RESUME_DIR="${BTLOG_ROOT}/phase1_medium/r1_medium/checkpoints/global_step_44000"
if [ ! -d "${RESUME_DIR}" ]; then
    echo "ERROR: Checkpoint 目录不存在: ${RESUME_DIR}"
    exit 1
fi

LOG_NAME="${BTLOG_ROOT}/csl.log"
if [ ! -f "${LOG_NAME}" ]; then
    echo "创建日志文件: ${LOG_NAME}"
    touch ${LOG_NAME}
fi

export HYDRA_FULL_ERROR=1

cd "${REPO_ROOT}"


echo "============================================================"
echo "  Phase 1: Ablation — augmentation=medium"
echo "============================================================"

nohup bash examples/sft/run_fastwam_sft.sh r1_pro_sft_fastwam \
  cluster.component_placement.actor=2-7 \
  runner.resume_dir=${RESUME_DIR} \
  runner.max_steps=187200 \
  runner.save_interval=2000 \
  runner.log_interval=500 \
  actor.micro_batch_size=16 \
  actor.global_batch_size=96 \
  actor.optim.lr=2e-5 \
  +actor.optim.min_lr=1e-6 \
  data.skip_padding_as_possible=true \
  runner.logger.experiment_name=r1_medium \
  runner.logger.log_path=${BTLOG_ROOT}/phase1_medium \
  > ${LOG_NAME} 2>&1 &

TRAIN_PID=$!
sleep 3
echo "训练已在后台启动，PID=${TRAIN_PID}，日志：${LOG_NAME} , 输出目录：${BTLOG_ROOT}"
touch ${BTLOG_ROOT}/${TRAIN_PID}.pid

echo "[phase1] augmentation=medium 启动。"
#   +data.processor.augmentation_preset=light \