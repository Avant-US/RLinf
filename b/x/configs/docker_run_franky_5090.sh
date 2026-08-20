#!/bin/bash
# Start (and attach to) the franky control container for this 5090 machine.
#
# --privileged is required: franky/libfranka needs RT scheduling and mlockall for
# its 1 kHz torque loop, and the RealSense USB devices need direct access.
# --network host is required: libfranka talks to the arm on 172.16.0.2:1337 and
# Ray binds host interfaces.
#
# --shm-size: Ray puts its object store in /dev/shm and warns loudly when that is
# the Docker default 64 MB, falling back to /tmp. Harmless for the phase-2 smoke
# scripts, but it shows up in every log and it does cost throughput once phase-3
# training runs (dmo_place_2.md §7 T4).
#
# The container is --rm and interactive: this terminal becomes its foreground
# shell. To get a second shell, use `docker exec -it rlinf-franky-5090 bash` from
# another terminal -- do NOT run this script twice (the --name would collide).
set -euo pipefail

REPO="${REPO:-/home/nvidia/bt/s/RLinf}"
IMAGE="${RLINF_FRANKA_IMAGE:-rlinf/rlinf:agentic-rlinf0.4-franka}"
NAME="${RLINF_FRANKA_CONTAINER:-rlinf-franky-5090}"
SHM="${RLINF_FRANKA_SHM_SIZE:-10g}"

if [[ ! -d "${REPO}" ]]; then
  echo "REPO does not exist: ${REPO}" >&2
  exit 1
fi

# Only one libfranka client may hold FCI. The fixed --name stops a second
# *identical* invocation, but nothing stops someone running the phase-2 scripts
# from inside the GPU container (also --network host, also repo-mounted) while this
# one holds the arm: both would open 172.16.0.2:1337 and the second connection
# either fails opaquely or steals control.
ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
if command -v ss >/dev/null 2>&1; then
  if ss -tn state established "( dport = :1337 or sport = :1337 )" 2>/dev/null \
      | grep -q "${ROBOT_IP}"; then
    echo "ERROR: something already holds an FCI connection to ${ROBOT_IP}:1337." >&2
    echo "       Stop it first (ray stop; check docker ps; the calibration REPL" >&2
    echo "       must be exited with 'q'), then re-run this script." >&2
    exit 1
  fi
fi

exec docker run -it --rm --privileged --network host --name "${NAME}" \
  --shm-size="${SHM}" \
  -v "${REPO}:/workspace/RLinf" -w /workspace/RLinf "${IMAGE}" bash
