#!/bin/bash
# Launch vla_inference_server inside the rlinf-4dwvla-gpu container.
#
# Usage:
#   1. Start the GPU container:  bash configs/docker_run_4dwvla_gpu.sh
#   2. Inside the container:     bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_gpu_server.sh
#
# grperr_1.2.md §6 recommended parameters applied.
set -euo pipefail

# ── Activate the 4dwvla venv ────────────────────────────────────────
source /opt/venv/4dwvla/bin/activate

# ── Paths (bind-mounted by docker_run_4dwvla_gpu.sh) ────────────────
CKPT_PATH="${VLA_CKPT_PATH:-/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp041680}"
SCHEMA_PATH="${VLA_SCHEMA_PATH:-/workspace/4WVLA/b/s/Frk/cfg/franka_plug.yaml}"
KPT_META="${VLA_KPT_META_PATH:-/workspace/RLinf/b/d/frk1/plug/keypoints_meta.json}"
URDF_PATH="${VLA_URDF_PATH:-/workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf}"

# ── Execution parameters ────────────────────────────────────────────
# n_exec: grperr_1.2.md §6.2 recommends 10 for Phase 2 (gripper fix validation).
#   After Phase 2 Level 2 passes, bump to 20 (align with RoboTwin infer_horizon).
#   IMPORTANT: client --n-exec must match (§9 item 6).
N_EXEC="${VLA_N_EXEC:-10}"
PORT="${VLA_PORT:-5555}"
DTYPE="${VLA_DTYPE:-bfloat16}"

# ── Preflight checks ───────────────────────────────────────────────
for f in "${CKPT_PATH}/config.json" "${SCHEMA_PATH}" "${KPT_META}" "${URDF_PATH}"; do
    if [[ ! -f "${f}" ]]; then
        echo "ERROR: required file not found: ${f}" >&2
        exit 1
    fi
done

echo "=== vla_inference_server ==="
echo "  ckpt:      ${CKPT_PATH}"
echo "  schema:    ${SCHEMA_PATH}"
echo "  kpt_meta:  ${KPT_META}"
echo "  urdf:      ${URDF_PATH}"
echo "  n_exec:    ${N_EXEC}"
echo "  port:      ${PORT}"
echo "  dtype:     ${DTYPE}"
echo ""

exec python /workspace/RLinf/b/x/4dwvla_ext/vla_inference_server.py \
    --ckpt-path "${CKPT_PATH}" \
    --schema-path "${SCHEMA_PATH}" \
    --kpt-meta-path "${KPT_META}" \
    --urdf-path "${URDF_PATH}" \
    --n-exec "${N_EXEC}" \
    --port "${PORT}" \
    --dtype "${DTYPE}"
