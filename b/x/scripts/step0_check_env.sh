#!/bin/bash
set -euo pipefail
REPO="${REPO:-/home/nvidia/bt/s/RLinf}"
IMAGE="${RLINF_FRANKA_IMAGE:-rlinf/rlinf:agentic-rlinf0.4-franka}"
PASS=0; FAIL=0
ok() { echo "[PASS] $*"; PASS=$((PASS+1)); }
fail() { echo "[FAIL] $*"; FAIL=$((FAIL+1)); }

echo "=== Step 0 host checks ==="
(uname -v | grep -q PREEMPT_RT || uname -r | grep -qE 'realtime|PREEMPT_RT') \
  && ok "PREEMPT_RT kernel" || fail "PREEMPT_RT kernel"
[ "$(ulimit -r)" -ge 80 ] 2>/dev/null || [ "$(ulimit -r)" = "unlimited" ] && ok "ulimit -r" || fail "ulimit -r=$(ulimit -r)"
[ "$(ulimit -l)" = "unlimited" ] && ok "ulimit -l unlimited" || fail "ulimit -l=$(ulimit -l)"
if [ "${FRANKA_NO_ROBOT:-1}" = "1" ]; then
  echo "[SKIP] ping/route (FRANKA_NO_ROBOT=1, no-robot path Step 0-1-4-7)"
else
  ping -c 1 -W 2 172.16.0.2 >/dev/null 2>&1 && ok "ping 172.16.0.2" || fail "ping 172.16.0.2"
  ip route get 172.16.0.2 2>/dev/null | grep -q 'dev eno1' && ok "route via eno1" || fail "route not eno1"
fi
[ ! -d "${REPO}/.venv" ] && ok "no host .venv" || fail "host .venv exists"

echo "=== Step 0 Docker franky checks ==="
if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  ok "docker image ${IMAGE} present"
  docker run --rm --privileged --network host \
    -v "${REPO}:/workspace/RLinf" -w /workspace/RLinf "${IMAGE}" \
    bash -lc 'ls /opt/venv/ | grep -q franky-0.19.0 && source switch_env franky-0.19.0 && python -c "import franky; print(\"franky OK\")"' \
    && ok "import franky in container" || fail "import franky in container"
else
  fail "docker image ${IMAGE} not present (run docker pull first)"
fi

echo "=== Step 0 summary: PASS=${PASS} FAIL=${FAIL} ==="
[ "${FAIL}" -eq 0 ]
