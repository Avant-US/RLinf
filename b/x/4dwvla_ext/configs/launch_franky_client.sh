#!/bin/bash
# Launch franka_vla_client inside the rlinf-4dwvla-franky container.
#
# Usage:
#   1. Start the Franky container:  bash configs/docker_run_4dwvla_franky.sh
#   2. Inside the container:        bash /workspace/RLinf/b/x/4dwvla_ext/configs/launch_franky_client.sh [level]
#
# Positional argument: validation level (0, 1, 2, 3). Default: 0.
# grperr_1.2.md §5/§7.2 + 4wvla_rlinf_eval_3A3.md §15.7 parameters applied.
#
# IMPORTANT: Must match server's --n-exec (grperr_1.2.md §9 item 6).
# IMPORTANT: Do NOT skip levels. Level 0 → 1 → 2 → 3, each must pass first.
set -euo pipefail

LEVEL="${1:-0}"

# ── Source hardware & gripper config ────────────────────────────────
ENV_FILE="/workspace/RLinf/b/x/4dwvla_ext/configs/franka_plug_eval.env"
if [[ -f "${ENV_FILE}" ]]; then
    echo "Sourcing ${ENV_FILE}"
    source "${ENV_FILE}"
else
    echo "WARNING: ${ENV_FILE} not found, using code defaults" >&2
fi

# ── Common parameters ──────────────────────────────────────────────
ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
SERVER_HOST="${VLA_SERVER_HOST:-localhost}"
SERVER_PORT="${VLA_PORT:-5555}"
N_EXEC="${VLA_N_EXEC:-10}"
TASK="${VLA_TASK:-plug into socket}"

echo "=== franka_vla_client (Level ${LEVEL}) ==="
echo "  VLA_GRIPPER_MODE:          ${VLA_GRIPPER_MODE:-binary_abs}"
echo "  FRANKA_GRIPPER_MAX_WIDTH_M: ${FRANKA_GRIPPER_MAX_WIDTH_M:-0.080}"
echo "  FRANKA_CUBE_WIDTH_M:       ${FRANKA_CUBE_WIDTH_M:-0.010}"
echo "  n_exec:                    ${N_EXEC}"
echo "  server:                    ${SERVER_HOST}:${SERVER_PORT}"
echo ""

case "${LEVEL}" in
    0)
        # Level 0: Dry Run — robot does not move, communication test only.
        # grperr_1.2.md §7.2 L0: his_len should walk 0→10→20…,
        # full_chunk_grip 50 values in server log, no close at HOME.
        echo "[Level 0] Dry run: robot will NOT move."
        exec python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py \
            --server-host "${SERVER_HOST}" \
            --server-port "${SERVER_PORT}" \
            --task "${TASK}" \
            --n-exec "${N_EXEC}" \
            --max-steps 250 \
            --dry-run
        ;;

    1)
        # Level 1: Conservative real robot — HOLD E-STOP!
        # 30 steps @ 5 Hz. Pass criteria: no MOTION GUARD, no mid-air close.
        # Setup keyboard/mouse first:
        #   source /workspace/RLinf/b/x/4dwvla_ext/setup_container_input.sh
        echo "[Level 1] Real robot, 30 steps @ 5 Hz. HOLD E-STOP!"
        echo "  Press 'r' to interrupt if anything looks wrong."
        echo ""
        read -p "E-stop ready? setup_container_input.sh sourced? [y/N] " confirm
        if [[ "${confirm}" != "y" && "${confirm}" != "Y" ]]; then
            echo "Aborted." >&2
            exit 1
        fi
        exec python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py \
            --robot-ip "${ROBOT_IP}" \
            --server-host "${SERVER_HOST}" \
            --server-port "${SERVER_PORT}" \
            --task "${TASK}" \
            --n-exec "${N_EXEC}" \
            --use-realsense \
            --max-steps 30 \
            --control-hz 5
        ;;

    2)
        # Level 2: Extended test — reset scene before each run.
        # 300 steps @ 10 Hz (then 30 Hz after first pass).
        # Pass criteria (grperr_1.2.md §7.2 L2):
        #   - At least one gripper_cmd=close
        #   - observation.state.gripper monotonically drops to <50mm
        #   - Width should start dropping within 20 steps
        #   - Post-run: wrist photo shows fingers gripping plug
        CONTROL_HZ="${VLA_CONTROL_HZ:-10}"
        MAX_STEPS="${VLA_MAX_STEPS:-300}"
        echo "[Level 2] Extended test, ${MAX_STEPS} steps @ ${CONTROL_HZ} Hz."
        echo "  Reset scene to initial pose before starting."
        echo ""
        read -p "Scene reset? E-stop ready? [y/N] " confirm
        if [[ "${confirm}" != "y" && "${confirm}" != "Y" ]]; then
            echo "Aborted." >&2
            exit 1
        fi
        exec python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py \
            --robot-ip "${ROBOT_IP}" \
            --server-host "${SERVER_HOST}" \
            --server-port "${SERVER_PORT}" \
            --task "${TASK}" \
            --n-exec "${N_EXEC}" \
            --use-realsense \
            --max-steps "${MAX_STEPS}" \
            --control-hz "${CONTROL_HZ}"
        ;;

    3)
        # Level 3: Formal 20-episode evaluation.
        # MUST pass Level 2 first (grperr_1.2.md §9 item 9).
        # Each episode: see 4wvla_rlinf_eval_3A3.md §15.8 for reset flow.
        CONTROL_HZ="${VLA_CONTROL_HZ:-10}"
        MAX_STEPS="${VLA_MAX_STEPS:-600}"
        echo "[Level 3] Formal evaluation, ${MAX_STEPS} steps @ ${CONTROL_HZ} Hz."
        echo "  Follow 4wvla_rlinf_eval_3A3.md §15.8 for episode reset flow."
        echo ""
        read -p "Level 2 passed? Scene reset? E-stop ready? [y/N] " confirm
        if [[ "${confirm}" != "y" && "${confirm}" != "Y" ]]; then
            echo "Aborted." >&2
            exit 1
        fi
        exec python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py \
            --robot-ip "${ROBOT_IP}" \
            --server-host "${SERVER_HOST}" \
            --server-port "${SERVER_PORT}" \
            --task "${TASK}" \
            --n-exec "${N_EXEC}" \
            --use-realsense \
            --max-steps "${MAX_STEPS}" \
            --control-hz "${CONTROL_HZ}"
        ;;

    *)
        echo "ERROR: unknown level '${LEVEL}'. Use 0, 1, 2, or 3." >&2
        exit 1
        ;;
esac
