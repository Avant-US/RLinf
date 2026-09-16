#!/bin/bash
# Run all RLT Stage 1 offline tests
# Usage: bash b/x/4dwvla_ext/rlt/tests/run_all_offline.sh
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOTAL_PASS=0
TOTAL_FAIL=0
FAILED_TESTS=()

run_one() {
    local name="$1"
    local script="$2"
    echo ""
    echo "━━━ Running: ${name} ━━━"
    python "${script}"
    local rc=$?
    if [ $rc -ne 0 ]; then
        FAILED_TESTS+=("${name}")
    fi
    # Parse results from last line
    local last_line
    last_line=$(python "${script}" 2>&1 | tail -1)
    local p f
    p=$(echo "$last_line" | grep -oP '\d+ passed' | grep -oP '\d+' || echo "0")
    f=$(echo "$last_line" | grep -oP '\d+ failed' | grep -oP '\d+' || echo "0")
    TOTAL_PASS=$((TOTAL_PASS + p))
    TOTAL_FAIL=$((TOTAL_FAIL + f))
}

echo "╔══════════════════════════════════════════════════╗"
echo "║     RLT Stage 1 Offline Test Suite               ║"
echo "╚══════════════════════════════════════════════════╝"

# Run in dependency order: T1 → T7 → T2 → T3 → T4 → T5 → T6
TESTS=(
    "T-RLT1:test_rlt_module_offline.py"
    "T-RLT7:test_rlt_behavior_equiv.py"
    "T-RLT2:test_rlt_forward_offline.py"
    "T-RLT3:test_rlt_loss_offline.py"
    "T-RLT4:test_rlt_gradient_offline.py"
    "T-RLT5:test_rlt_checkpoint_offline.py"
    "T-RLT6:test_rlt_compat_offline.py"
)

for entry in "${TESTS[@]}"; do
    name="${entry%%:*}"
    script="${SCRIPT_DIR}/${entry##*:}"
    echo ""
    echo "━━━ Running: ${name} ━━━"
    python "${script}"
    rc=$?
    if [ $rc -ne 0 ]; then
        FAILED_TESTS+=("${name}")
    fi
done

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║                 SUMMARY                          ║"
echo "╚══════════════════════════════════════════════════╝"
if [ ${#FAILED_TESTS[@]} -eq 0 ]; then
    echo "=== All test groups passed ==="
else
    echo "=== Failed test groups: ${FAILED_TESTS[*]} ==="
fi
exit ${#FAILED_TESTS[@]}
