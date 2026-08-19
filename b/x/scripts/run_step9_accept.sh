#!/bin/bash
# Step 9 acceptance: sphere-geometry tests; optional real-robot wander + photos.
#
# Usage (franky container):
#   source b/x/configs/setup_before_ray_5090.sh
#   bash b/x/scripts/run_step9_accept.sh              # math-only, no FCI
#   bash b/x/scripts/run_step9_accept.sh --with-robot # + 10 s sphere motion (Desk FCI)
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

if command -v python >/dev/null 2>&1; then
  PY=python
else
  PY=python3
fi

echo "=== Step 9 math ==="
"${PY}" "${SCRIPTS_DIR}/step9_test_ee_sphere.py" --math-only

if [ "${WITH_ROBOT}" -eq 0 ]; then
  echo "RESULT Step9 accept PASS (math-only; pass --with-robot for EE motion)"
  exit 0
fi

DETECTED="${REPO}/b/x/configs/camera_detected.json"
if [ ! -f "${DETECTED}" ]; then
  echo "=== Step 8a detect (missing camera_detected.json) ==="
  "${PY}" "${SCRIPTS_DIR}/step8_detect_cameras.py"
fi

echo "=== Step 9 EE sphere + photos (FCI, 5 cm ball, 10 s) ==="
export RLINF_SKIP_CAMERA=0
export FRANKA_ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
mkdir -p "${REPO}/b/x/logs"
ROBOT_LOG="${REPO}/b/x/logs/step9_robot.out"
ray stop --force 2>/dev/null || true
ray start --head --port=6379 --disable-usage-stats
set +e
"${PY}" "${SCRIPTS_DIR}/step9_test_ee_sphere.py" 2>&1 | tee "${ROBOT_LOG}"
rc=${PIPESTATUS[0]}
set -e
ray stop --force 2>/dev/null || true
echo "PYTHON_RC=${rc}"
if grep -q 'RESULT Step9 FAIL' "${ROBOT_LOG}" || [ "${rc}" -ne 0 ]; then
  echo "RESULT Step9 accept FAIL (python_rc=${rc})"
  exit 1
fi
if ! grep -q 'RESULT Step9 PASS' "${ROBOT_LOG}"; then
  echo "RESULT Step9 accept FAIL (no RESULT Step9 PASS in log)"
  exit 1
fi
echo "RESULT Step9 accept PASS (math+robot)"
exit 0
