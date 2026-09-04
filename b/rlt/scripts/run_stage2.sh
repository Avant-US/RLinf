#!/usr/bin/env bash
# 启动 RLT Stage 2 训练（Franky 版）
# 在 GPU 容器中执行，ray start 已完成
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RLT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_PATH="$(cd "${RLT_DIR}/../.." && pwd)"

EMBODIED_PATH="${REPO_PATH}/examples/embodiment"
CONFIG_PATH="${RLT_DIR}/configs"
CONFIG_NAME="${1:-realworld_rlt_stage2_franky}"
# 同步推理：一次推理 -> 执行完整个 chunk -> 再推理。
# 异步入口会让推理和环境步进重叠，动作基于更旧的观测计算。
# 设 RLT_ASYNC=1 切回异步。
if [[ "${RLT_ASYNC:-0}" == "1" ]]; then
    SRC_FILE="${EMBODIED_PATH}/train_async.py"
    RUN_MODE="async"
else
    SRC_FILE="${EMBODIED_PATH}/train_embodied_agent.py"
    RUN_MODE="sync"
fi

export REPO_PATH
export EMBODIED_PATH
export PYTHONPATH="${REPO_PATH}:${PYTHONPATH:-}"
export RLINF_CUBE_STEP_SPEED=0.20

LOG_DIR="${RLT_DIR}/results/$(date '+%Y%m%d-%H%M%S')-${CONFIG_NAME}"
mkdir -p "${LOG_DIR}"

echo "=== RLT Stage 2 (Franky) ==="
echo "Config:   ${CONFIG_PATH}/${CONFIG_NAME}.yaml"
echo "Mode:     ${RUN_MODE} ($(basename "${SRC_FILE}"))"
echo "Log dir:  ${LOG_DIR}"
echo "============================="

python "${SRC_FILE}" \
    --config-path "${CONFIG_PATH}" \
    --config-name "${CONFIG_NAME}" \
    runner.logger.log_path="${LOG_DIR}" \
    2>&1 | tee "${LOG_DIR}/run_embodiment.log" || true
