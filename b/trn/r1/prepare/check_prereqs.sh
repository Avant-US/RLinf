#!/usr/bin/env bash
# R1 Pro FastWAM SFT — 前置条件检查
# 用法: bash b/trn/r1/prepare/check_prereqs.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/env.sh"

PASS=0
FAIL=0

check() {
    local name=$1; shift
    if "$@" >/dev/null 2>&1; then
        echo "  ✓ ${name}"
        PASS=$((PASS + 1))
    else
        echo "  ✗ ${name}"
        FAIL=$((FAIL + 1))
    fi
}

echo "============================================================"
echo "  R1 Pro SFT 前置条件检查"
echo "============================================================"
echo

echo "[1] 数据路径"
check "R1PRO_DATA 目录" test -d "${R1PRO_DATA}/r1_pro_data_convert_chassis"
check "parquet 数据" test -f "${R1PRO_DATA}/r1_pro_data_convert_chassis/data/chunk-000/episode_000000.parquet"
check "meta/info.json" test -f "${R1PRO_DATA}/r1_pro_data_convert_chassis/meta/info.json"
check "meta/episodes.jsonl" test -f "${R1PRO_DATA}/r1_pro_data_convert_chassis/meta/episodes.jsonl"

echo
echo "[2] T5 嵌入缓存"
T5_CACHE="${FASTWAM_ROOT}/data/text_embeds_cache/r1_pro_chassis"
check "T5 cache 目录" test -d "${T5_CACHE}"
T5_COUNT=$(ls "${T5_CACHE}"/*.pt 2>/dev/null | wc -l)
if [ "${T5_COUNT}" -ge 1 ]; then
    echo "  ✓ T5 cache 有 ${T5_COUNT} 个 .pt 文件"
    PASS=$((PASS + 1))
else
    echo "  ✗ T5 cache 为空！请先运行:"
    echo "    cd ${FASTWAM_ROOT} && python scripts/precompute_text_embeds.py task=r1_pro_chassis_uncond_3cam_384_1e-4"
    FAIL=$((FAIL + 1))
fi

echo
echo "[3] 模型权重"
check "DIFFSYNTH_MODEL_BASE_PATH" test -d "${DIFFSYNTH_MODEL_BASE_PATH}"
check "ActionDiT 权重" test -f "${DIFFSYNTH_MODEL_BASE_PATH}/ActionDiT_linear_interp_Wan22_alphascale_1024hdim.pt"

echo
echo "[4] GPU"
GPU_COUNT=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | wc -l)
if [ "${GPU_COUNT}" -ge 8 ]; then
    echo "  ✓ 检测到 ${GPU_COUNT} 个 GPU"
    PASS=$((PASS + 1))
else
    echo "  ✗ 仅检测到 ${GPU_COUNT} 个 GPU（需要 8）"
    FAIL=$((FAIL + 1))
fi

echo
echo "[5] Ray"
if ray status >/dev/null 2>&1; then
    echo "  ✓ Ray 集群正在运行"
    PASS=$((PASS + 1))
else
    echo "  ✗ Ray 未运行。请先执行: bash b/trn/r1/prepare/start_ray.sh"
    FAIL=$((FAIL + 1))
fi

echo
echo "[6] Python 环境"
check "torch 可导入" python -c "import torch"
check "fastwam 可导入" python -c "import fastwam"
check "rlinf 可导入" python -c "import rlinf"
check "augmentation 模块" python -c "from rlinf.data.datasets.fastwam.augmentation import AugmentationPreset"

echo
echo "============================================================"
echo "  结果: ${PASS} 通过, ${FAIL} 失败"
echo "============================================================"

if [ "${FAIL}" -gt 0 ]; then
    echo "请修复上述失败项后再开始训练。"
    exit 1
fi
echo "所有检查通过，可以开始训练。"
