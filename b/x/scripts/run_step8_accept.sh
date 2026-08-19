#!/bin/bash
# Step 8 acceptance: 8a detect + 8b YAML check; optional 8c robot+camera.
#
# Usage (franky container):
#   source b/x/configs/setup_before_ray_5090.sh
#   bash b/x/scripts/run_step8_accept.sh              # 8a + 8b only
#   bash b/x/scripts/run_step8_accept.sh --with-robot # + 8c (Desk FCI, arm hold)
set -euo pipefail

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${REPO_PATH:-$(cd "${SCRIPTS_DIR}/../../.." && pwd)}"
export REPO_PATH="${REPO}"
cd "${REPO}"

WITH_ROBOT=0
if [ "${1:-}" = "--with-robot" ]; then
  WITH_ROBOT=1
fi

if command -v switch_env >/dev/null 2>&1; then
  # shellcheck source=/dev/null
  source "${SCRIPTS_DIR}/../configs/setup_before_ray_5090.sh"
fi

echo "=== Step 8a detect ==="
python "${SCRIPTS_DIR}/step8_detect_cameras.py" --write-yaml

echo "=== Step 8b YAML ==="
python "${SCRIPTS_DIR}/step8_check_yaml.py"

if [ "${WITH_ROBOT}" -eq 0 ]; then
  echo "RESULT Step8 accept PASS (8a+8b; skip 8c, pass --with-robot to run env)"
  exit 0
fi

echo "=== Step 8c env+camera (FCI, arm hold) ==="
export RLINF_SKIP_CAMERA=0
export FRANKA_ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
ray stop --force 2>/dev/null || true
ray start --head --port=6379 --disable-usage-stats
set +e
python "${SCRIPTS_DIR}/step8_test_env_camera.py"
rc=$?
set -e
ray stop --force 2>/dev/null || true
if [ "${rc}" -ne 0 ]; then
  echo "RESULT Step8 accept FAIL (8c exit ${rc})"
  exit "${rc}"
fi
echo "RESULT Step8 accept PASS (8a+8b+8c)"
exit 0
