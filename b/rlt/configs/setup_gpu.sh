#!/usr/bin/env bash
# GPU 容器环境设置 —— 在 ray start 之前 source 此脚本
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_PATH="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

export REPO_PATH
export PYTHONPATH="${REPO_PATH}:${PYTHONPATH:-}"
export RLINF_NODE_RANK=0
export RLINF_COMM_NET_DEVICES="${RLINF_COMM_NET_DEVICES:-eth0}"
export GLOO_SOCKET_IFNAME="${RLINF_COMM_NET_DEVICES}"
export EMBODIED_PATH="${REPO_PATH}/examples/embodiment"

# 激活 OpenPI 环境
if command -v switch_env &>/dev/null; then
    source switch_env openpi
else
    echo "[WARN] switch_env not found, ensure openpi venv is already active"
fi

echo "=== GPU container setup ==="
echo "REPO_PATH:       ${REPO_PATH}"
echo "RLINF_NODE_RANK: ${RLINF_NODE_RANK}"
echo "EMBODIED_PATH:   ${EMBODIED_PATH}"
echo "==========================="
