#!/usr/bin/env bash
# Master script: run all T0–T5 alignment tests sequentially.
#
# Prerequisites:
#   - CUDA_VISIBLE_DEVICES=4,5,6,7 (or set in env)
#   - FASTWAM_ROOT, DIFFSYNTH_MODEL_BASE_PATH, R1PRO_DATA
#   - Ray cluster on port 6399 (auto-started for T4/T5)
#
# Usage:
#   export CUDA_VISIBLE_DEVICES=4,5,6,7
#   bash b/test/run_all.sh          # run all
#   bash b/test/run_all.sh T0 T1    # run specific tests

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

export FASTWAM_ROOT="${FASTWAM_ROOT:-/home/Luogang/SRC/Robot/FastWAM}"
export FASTWAM_PATH="${FASTWAM_ROOT}/src"
export DIFFSYNTH_MODEL_BASE_PATH="${DIFFSYNTH_MODEL_BASE_PATH:-/mnt/r/CKPT/VLA/FW}"
export DIFFSYNTH_SKIP_DOWNLOAD=true
export R1PRO_DATA="${R1PRO_DATA:-/mnt/r/share/zwy/datasets/r1_pro_data_v2}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-4,5,6,7}"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.8}"
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
export RAY_ADDRESS="${RAY_ADDRESS:-127.0.0.1:6399}"
export PYTHONPATH="${REPO_ROOT}:${FASTWAM_PATH}:${PYTHONPATH:-}"

ALL_TESTS="T0 T1 T2 T3 T4 T5"
if [ $# -gt 0 ]; then
    TESTS_TO_RUN="$*"
else
    TESTS_TO_RUN="${ALL_TESTS}"
fi

PASSED=0
FAILED=0
RESULTS=()

run_test() {
    local name=$1 cmd=$2
    echo ""
    echo "================================================================"
    echo "  Running: ${name}"
    echo "================================================================"
    if eval "${cmd}"; then
        RESULTS+=("${name}: PASS")
        PASSED=$((PASSED + 1))
    else
        RESULTS+=("${name}: FAIL")
        FAILED=$((FAILED + 1))
    fi
}

ensure_ray() {
    if ! ray status &>/dev/null; then
        echo "[run_all] Starting Ray cluster..."
        ray stop 2>/dev/null || true
        ray start --head --port=6399 --num-gpus=8
        sleep 3
    fi
}

for TEST in ${TESTS_TO_RUN}; do
    case "${TEST}" in
        T0) run_test "T0 Params"       "bash ${SCRIPT_DIR}/T0_params/run.sh" ;;
        T1) run_test "T1 Single-Step"   "bash ${SCRIPT_DIR}/T1_single_step/run.sh" ;;
        T2) run_test "T2 Multi-Step"    "bash ${SCRIPT_DIR}/T2_multi_step/run.sh" ;;
        T3) run_test "T3 Checkpoint"    "bash ${SCRIPT_DIR}/T3_checkpoint/run.sh" ;;
        T4)
            ensure_ray
            run_test "T4-native" "bash ${SCRIPT_DIR}/T4_e2e/run_native.sh"
            ensure_ray
            run_test "T4-rlinf"  "bash ${SCRIPT_DIR}/T4_e2e/run_rlinf.sh"
            run_test "T4-verify" "python ${SCRIPT_DIR}/T4_e2e/verify.py"
            ;;
        T5)
            ensure_ray
            run_test "T5-1gpu"   "bash ${SCRIPT_DIR}/T5_distributed/run_1gpu.sh"
            ensure_ray
            run_test "T5-4gpu"   "bash ${SCRIPT_DIR}/T5_distributed/run_4gpu.sh"
            run_test "T5-verify" "python ${SCRIPT_DIR}/T5_distributed/verify.py"
            ;;
        *) echo "Unknown test: ${TEST}"; exit 1 ;;
    esac
done

echo ""
echo "================================================================"
echo "  SUMMARY: ${PASSED} passed, ${FAILED} failed"
echo "================================================================"
for r in "${RESULTS[@]}"; do
    echo "  ${r}"
done
echo ""

exit ${FAILED}
