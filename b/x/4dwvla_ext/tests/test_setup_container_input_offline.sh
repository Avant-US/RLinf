#!/usr/bin/env bash
# Offline checks for setup_container_input.sh (no robot, no USB required).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SCRIPT="$ROOT/setup_container_input.sh"

bash -n "$SCRIPT"
test -x "$SCRIPT"

out=$(bash "$SCRIPT" --help)
echo "$out" | grep -q 'source /workspace/RLinf/b/x/4dwvla_ext/setup_container_input.sh'

set +e
bash "$SCRIPT" --nope >/tmp/setup_input_nope.out 2>&1
rc=$?
set -e
test "$rc" -eq 2
grep -q 'unknown argument' /tmp/setup_input_nope.out

# Sourcing --help must not kill the parent shell.
# shellcheck disable=SC1090
source "$SCRIPT" --help >/dev/null

echo "setup_container_input.sh offline checks passed"
