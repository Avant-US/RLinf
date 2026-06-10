#!/usr/bin/env bash
# R1 Pro FastWAM SFT — 环境变量
# 用法: source b/trn/r1/prepare/env.sh

export FASTWAM_ROOT=/home/Luogang/SRC/Robot/FastWAM
export FASTWAM_PATH=${FASTWAM_ROOT}/src
export R1PRO_DATA=/mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis
export DIFFSYNTH_MODEL_BASE_PATH=/mnt/r/CKPT/VLA/FW
export DIFFSYNTH_SKIP_DOWNLOAD=true
export CUDA_HOME=/usr/local/cuda-12.8
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
export BTLOG_ROOT="/mnt/r/CKPT/VLA/FW/RUN/"
export RAY_ADDRESS=127.0.0.1:6399
export REPO_PATH=/home/Luogang/SRC/RL/RLinf
export PYTHONPATH=${REPO_PATH}:${FASTWAM_PATH}:${PYTHONPATH:-}
export TF_ENABLE_ONEDNN_OPTS=0
export WANDB_CONSOLE=off
export VENV_PATH="/mnt/r/VENV/rlinf_venv/"
export EMBODIED_PATH=${REPO_PATH}/examples/sft

echo "[env] FASTWAM_ROOT=${FASTWAM_ROOT}"
echo "[env] R1PRO_DATA=${R1PRO_DATA}"
echo "[env] BTLOG_ROOT=${BTLOG_ROOT}"
echo "[env] DIFFSYNTH_MODEL_BASE_PATH=${DIFFSYNTH_MODEL_BASE_PATH}"
