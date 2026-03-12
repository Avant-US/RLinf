#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# ── Configuration (edit here) ─────────────────────────────────────
ROBOT_HOST="127.0.0.1"
ROBOT_PORT="15555"
POLICY_HOST="10.239.121.23"
POLICY_PORT="8003"
PROMPT="${1:-Pick the yellow box on the table}"

SWAP_CAMERAS="0"
RS_COLOR_FORMAT="rgb8"
WRIST_SERIAL=""
HEAD_SERIAL=""

ENABLE_ARM="1"
ENABLE_GRIPPER="1"
AUTO_RESET_BEFORE_RUN="1"

CONTROL_HZ="15"
OPEN_LOOP_HORIZON="8"
VALIDITY_MS="500"
STEPS="3000"
GRIPPER_THRESHOLD="0.5"

# ── Logging ───────────────────────────────────────────────────────
mkdir -p logs
TS="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="run_test_${TS}.jsonl"

# ── Kill any process holding RealSense cameras ────────────────────
fuser /dev/video* 2>/dev/null | tr -s ' ' '\n' | sort -u | xargs -r kill -9 2>/dev/null || true
sleep 1

# ── Run ───────────────────────────────────────────────────────────
echo "[test] prompt: $PROMPT"
echo "[test] log: logs/${LOG_FILE}"
echo "[test] mode: JOINT_VELOCITY (direct, 15Hz, no intermediate clamping)"
if [[ "$AUTO_RESET_BEFORE_RUN" == "1" ]]; then
  echo "[test] preflight: running reset_pose.py"
  python3 reset_pose.py
fi
exec env PYTHONUNBUFFERED=1 python3 -m station.run \
    --robot-host "$ROBOT_HOST" --robot-port "$ROBOT_PORT" \
    --policy-host "$POLICY_HOST" --policy-port "$POLICY_PORT" \
    --prompt "$PROMPT" \
    --use-realsense \
    --rs-color-format "$RS_COLOR_FORMAT" \
    $( [[ "$SWAP_CAMERAS" == "1" ]] && echo --swap-cameras ) \
    $( [[ -n "$WRIST_SERIAL" ]] && echo --wrist-serial "$WRIST_SERIAL" ) \
    $( [[ -n "$HEAD_SERIAL" ]] && echo --head-serial "$HEAD_SERIAL" ) \
    --control-hz "$CONTROL_HZ" \
    --open-loop-horizon "$OPEN_LOOP_HORIZON" \
    --validity-ms "$VALIDITY_MS" \
    --gripper-threshold "$GRIPPER_THRESHOLD" \
    --log-dir "logs" --log-file "$LOG_FILE" \
    --save-obs-images "logs" \
    --print-vla \
    $( [[ "$ENABLE_ARM" == "1" ]] && echo --enable-arm ) \
    $( [[ "$ENABLE_GRIPPER" == "1" ]] && echo --enable-gripper ) \
    --steps "$STEPS"
