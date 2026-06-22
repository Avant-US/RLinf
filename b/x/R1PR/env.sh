#!/usr/bin/env bash
# R1 Pro FastWAM SFT — 环境变量
# 用法: source b/trn/r1/prepare/env.sh

export FASTWAM_ROOT=/home/physical/SRC/Robot/FastWAM
export FASTWAM_PATH=${FASTWAM_ROOT}/src
# export R1PRO_DATA=/mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis
export DIFFSYNTH_MODEL_BASE_PATH=/mnt/r/CKPT/
export DIFFSYNTH_SKIP_DOWNLOAD=true
export CUDA_HOME=/usr/local/cuda-13.0
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
export BTLOG_ROOT="/mnt/r/CKPT/VLA/FW/RUN/"
export RAY_ADDRESS=127.0.0.1:6399
export REPO_PATH=/home/physical/SRC/RL/RLinf
export PYTHONPATH=${REPO_PATH}:${FASTWAM_PATH}:${PYTHONPATH:-}
export HYDRA_FULL_ERROR=1
export TF_ENABLE_ONEDNN_OPTS=0
export WANDB_CONSOLE=off
export VENV_PATH="/mnt/r/VENV/rlinf_venv/"
export EMBODIED_PATH=${REPO_PATH}/examples/sft

