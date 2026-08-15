#!/bin/bash
# Step 1: verify switch_env + Ray head in franky container context.
set -euo pipefail

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "${SCRIPTS_DIR}/../configs/setup_before_ray_5090.sh"

expected="/opt/venv/franky-0.19.0/bin/python"
actual="$(which python)"
if [ "${actual}" != "${expected}" ]; then
  echo "FAIL: python=${actual}, expected ${expected}" >&2
  exit 1
fi
echo "PASS: switch_env franky-0.19.0 -> ${actual}"

ray stop --force 2>/dev/null || true
ray start --head --port=6379 --disable-usage-stats
sleep 2
ray status
ray stop --force
echo "Step1 PASS"
