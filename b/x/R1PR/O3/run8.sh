#!/usr/bin/env bash
# 从 b/trn/r1/phase1/ablation_medium.sh 复制过来的. 其实 R1PR/R1 实验是在那个文件夹启动的
# Phase 1 Ablation: medium 增强 (1500 steps)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export RUN_CFGPATH=${SCRIPT_DIR}/
source "${SCRIPT_DIR}/../env.sh"
# export R1PRO_DATA=/mnt/r/share/kaixin/data_0530/lerobot_open_merged_v21 #默认是25版R1 Pro采集的数据

BTLOG_ROOT="/mnt/r/CKPT/VLA/FW/RUN"
PRJID=R1PR
PRJ_LOGROOT="${BTLOG_ROOT}/${PRJID}"
RUNID=O3
RUN_ROOT="${PRJ_LOGROOT}/${RUNID}"
export RUN_LOGROOT=${RUN_ROOT}
# RUNLOG_ROOT="${RUN_ROOT}/logs"

export CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7
export HYDRA_FULL_ERROR=1

if [ ! -f "${VENV_PATH}/bin/activate" ]; then
    echo "ERROR: 虚拟环境不存在: ${VENV_PATH}"
    exit 1
fi
source "${VENV_PATH}/bin/activate"


if [ ! -d "${RUN_ROOT}" ]; then
    echo "创建ckpt与日志目录: ${RUN_ROOT}"
    mkdir -p ${RUN_ROOT}
fi

# RESUME_DIR="${BTLOG_ROOT}/phase1_medium/r1_medium/checkpoints/global_step_44000"
# if [ ! -d "${RESUME_DIR}" ]; then
#     echo "ERROR: Checkpoint 目录不存在: ${RESUME_DIR}"
#     exit 1
# fi

LOG_NAME="${RUN_ROOT}/csl.log"
if [ ! -f "${LOG_NAME}" ]; then
    echo "创建日志文件: ${LOG_NAME}"
    touch ${LOG_NAME}
fi



cd "${REPO_PATH}"


echo "============================================================"
echo "  Phase 1: Ablation — augmentation=medium"
echo "============================================================"

nohup bash examples/sft/run_fastwam_sft.sh r125_vid_stat_aug_stro1 \
  cluster.component_placement.actor=0-7 \
  runner.max_steps=2500 \
  runner.save_interval=500 \
  runner.log_interval=100 \
  actor.micro_batch_size=32 \
  actor.global_batch_size=256 \
  actor.optim.lr=6e-5 \
  +actor.optim.min_lr=1e-6 \
  data.skip_padding_as_possible=true \
  runner.logger.project_name=${PRJID}  \
  runner.logger.experiment_name=${RUNID} \
  runner.logger.log_path=${RUN_ROOT} \
  > ${LOG_NAME} 2>&1 &

TRAIN_PID=$!
sleep 3
echo "训练已在后台启动，PID=${TRAIN_PID}，日志：${LOG_NAME} , 输出目录：${RUN_ROOT}"
touch ${RUN_ROOT}/${TRAIN_PID}.pid

echo "[phase1] augmentation=medium 启动。"
#   +data.processor.augmentation_preset=light \
#   +runner.resume_dir=${RESUME_DIR} \