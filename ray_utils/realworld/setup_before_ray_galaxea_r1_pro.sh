#!/bin/bash
# Source this BEFORE `ray start` on every Galaxea R1 Pro node.
# Sets PYTHONPATH, RLINF_NODE_RANK, ROS 2 environment + Galaxea SDK,
# DDS profile, and verifies that can0 is up (Orin-only soft check).

set -euo pipefail

export CURRENT_PATH="$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd )"
export REPO_PATH=$(dirname $(dirname "$CURRENT_PATH"))
export PYTHONPATH="${REPO_PATH}:${PYTHONPATH:-}"

# ── RLinf scheduler env vars ────────────────────────────────────
# Each node MUST export a unique RLINF_NODE_RANK BEFORE running
# `ray start`.  Override on the command line, e.g.:
#   export RLINF_NODE_RANK=0  (GPU server)
#   export RLINF_NODE_RANK=1  (Orin)
export RLINF_NODE_RANK="${RLINF_NODE_RANK:--1}"
export RLINF_COMM_NET_DEVICES="${RLINF_COMM_NET_DEVICES:-eth0}"

# ── Activate project venv (if any) ──────────────────────────────
# Set R1PRO_VENV to the path of your venv before sourcing this file.
if [ -n "${R1PRO_VENV:-}" ] && [ -f "${R1PRO_VENV}/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "${R1PRO_VENV}/bin/activate"
fi

# ── ROS 2 Humble (silent if absent on GPU server) ───────────────
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

# ── Galaxea SDK install (only present on Orin in form A) ────────
export GALAXEA_INSTALL_PATH="${GALAXEA_INSTALL_PATH:-$HOME/galaxea/install}"
if [ -f "${GALAXEA_INSTALL_PATH}/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${GALAXEA_INSTALL_PATH}/setup.bash"
fi

# ── Cross-host DDS settings (must match on every node) ──────────
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
# Default 0 enables cross-host discovery (required when controller
# and EnvWorker live on different machines).
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# ── Optional FastDDS / Cyclone XML profile ──────────────────────
if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] \
    && [ -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]; then
    echo "[r1pro] FastDDS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi

# ── CAN check (idempotent; only meaningful on Orin) ─────────────
if command -v ip >/dev/null 2>&1 && ip link show can0 >/dev/null 2>&1; then
    if ! ip link show can0 | grep -q "UP"; then
        echo "[r1pro] can0 is not UP; trying \`bash ~/can.sh\` ..."
        if [ -x "${HOME}/can.sh" ]; then
            bash "${HOME}/can.sh" || true
        fi
    fi
fi

echo "[r1pro] RLINF_NODE_RANK=${RLINF_NODE_RANK}"
echo "[r1pro] RLINF_COMM_NET_DEVICES=${RLINF_COMM_NET_DEVICES}"
echo "[r1pro] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[r1pro] ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "[r1pro] GALAXEA_INSTALL_PATH=${GALAXEA_INSTALL_PATH}"
