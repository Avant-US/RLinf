#!/usr/bin/env bash
# Franky 控制容器环境设置 —— 在 ray start 之前 source 此脚本
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_PATH="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

export REPO_PATH
export PYTHONPATH="${REPO_PATH}:${REPO_PATH}/b/x:${PYTHONPATH:-}"
export RLINF_EXT_MODULE="franky_ext.runtime_bootstrap"
export RLINF_NODE_RANK=1
export RLINF_COMM_NET_DEVICES="${RLINF_COMM_NET_DEVICES:-rlinf-br0}"
export GLOO_SOCKET_IFNAME="${RLINF_COMM_NET_DEVICES}"

# Franky / libfranka
export FRANKA_ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
export FRANKA_GRIPPER_TYPE="${FRANKA_GRIPPER_TYPE:-franka}"

# 安全参数（与 b/x 一致）
export RLINF_CUBE_FORCE_CEILING_N="${RLINF_CUBE_FORCE_CEILING_N:-20}"
export RLINF_CUBE_INTERP_SPEED="${RLINF_CUBE_INTERP_SPEED:-0.02}"
export RLINF_CUBE_STEP_SPEED="${RLINF_CUBE_STEP_SPEED:-0.05}"
export RLINF_CUBE_GUARD_MARGIN="${RLINF_CUBE_GUARD_MARGIN:-0.05}"
export RLINF_CUBE_GUARD_FLOOR_MARGIN="${RLINF_CUBE_GUARD_FLOOR_MARGIN:-0.01}"
export RLINF_CUBE_GUARD_MAX_LAG="${RLINF_CUBE_GUARD_MAX_LAG:-0.05}"
export RLINF_CUBE_GUARD_RECOVERY_BUDGET="${RLINF_CUBE_GUARD_RECOVERY_BUDGET:-10}"

# 夹爪参数
export FRANKA_GRASP_FORCE="${FRANKA_GRASP_FORCE:-20}"
export FRANKA_CUBE_WIDTH_M="${FRANKA_CUBE_WIDTH_M:-0.015}"
export FRANKA_HOLD_TOL_M="${FRANKA_HOLD_TOL_M:-0.015}"

# 键盘设备（按现场实际修改）
export RLINF_KEYBOARD_DEVICE="${RLINF_KEYBOARD_DEVICE:-/dev/input/event2}"

# 激活 franky-0.19.0 环境（注意是 franky 不是 franka）
if command -v switch_env &>/dev/null; then
    source switch_env franky-0.19.0
else
    echo "[WARN] switch_env not found, ensure franky venv is already active"
fi

echo "=== Franky container setup ==="
echo "REPO_PATH:          ${REPO_PATH}"
echo "RLINF_EXT_MODULE:   ${RLINF_EXT_MODULE}"
echo "RLINF_NODE_RANK:    ${RLINF_NODE_RANK}"
echo "FRANKA_ROBOT_IP:    ${FRANKA_ROBOT_IP}"
echo "KEYBOARD_DEVICE:    ${RLINF_KEYBOARD_DEVICE}"
echo "PYTHONPATH includes: ${REPO_PATH}/b/x"
echo "==============================="
