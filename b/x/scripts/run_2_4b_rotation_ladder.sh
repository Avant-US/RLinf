#!/usr/bin/env bash
# 2.4b-rot: staged rotation ladder for FrankyCubePlaceEnv-v1 (LOG-034 / T16).
#
# Run **inside the franky container**, after:
#   source b/x/configs/setup_before_ray_5090.sh
#
# Mirrors how 2.4b (translation) was actually run -- a sequence of separate
# `diag_franky_motion.py` invocations at increasing amplitude/speed, each
# logged to its own file, stopping at the FIRST failure rather than continuing
# (every test's first action is `robot.recover_from_errors()`, so running the
# next rung after an abort clears the fault and commands the arm again
# immediately -- round-2 audit finding 2, still true here). Unlike 2.4b this
# ladder exercises an authority (rotation on the step() path) that has NEVER
# been measured on this arm, so:
#
#   * every rung starts from --test-rotation (smooth ramp+settle) before the
#     10Hz --test-rotation-waypoints reproduction of step() itself;
#   * the amplitude is capped at 0.15rad (~8.6deg) by default -- inside the
#     0.55rad orientation-fence slack with margin to spare -- and the speed
#     ladder tops out at INTERP_SPEED_RAD_S_DEFAULT (0.15rad/s), the only
#     rotation speed ever exercised on this arm without incident. Reaching
#     motion_limits.STEP_ROT_SPEED_RAD_S_DEFAULT (0.3rad/s) itself is left as a
#     deliberate follow-up once this ladder's baseline is judged safe -- 2.4b
#     never measured translation AT its step cap either (T12).
#   * the diagnostic script's own --max-dq circuit breaker (default 1.0rad/s,
#     an order of magnitude below LOG-034's 2.69rad/s incident) is left in
#     effect on every rung, not just the last one.
#
# This ladder runs at REST (wherever the arm currently is), not at the
# NEAR-SINGULAR safety-box corner LOG-034 actually tripped at. That is
# deliberate: characterising the authority in the open first, before pointing
# it at the one geometry known to be risky, is the same staged-exposure
# principle 2.4b itself used. Testing AT the reach boundary is a separate,
# more deliberate experiment -- do not fold it into this script.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "${ROOT}"
export REPO_PATH="${REPO_PATH:-${ROOT}}"
DIAG="${ROOT}/b/x/scripts/diag_franky_motion.py"
LOG_DIR="${CUBE_PLACE_LADDER_LOG_DIR:-${ROOT}/logs/2_4b_rotation_ladder}"

usage() {
  cat <<'EOF'
用法（必须已在 franky 容器内，且 source 过 setup_before_ray_5090.sh）：

  bash b/x/scripts/run_2_4b_rotation_ladder.sh              # 跑全部档位，逐档止损
  bash b/x/scripts/run_2_4b_rotation_ladder.sh --rung 2      # 只跑第 2 档
  bash b/x/scripts/run_2_4b_rotation_ladder.sh --dry-run     # 只打印将要执行的命令
  bash b/x/scripts/run_2_4b_rotation_ladder.sh --max-drz 0.10 --max-speed 0.10  # 收紧幅度/速度上限

每档都会：
  1. 记录到 logs/2_4b_rotation_ladder/rung_N_*.log
  2. 检查 verdict / alive / stop= 是否正常
  3. 失败立即停止（不跑后面的档），并打印失败档的日志尾部

跑完后打印所有档的 peak_ang_lag / peak_dq / peak_xyz_drift 汇总表，
并给出 motion_limits.STEP_ROT_SPEED_RAD_S_DEFAULT 是否在已验证范围内的结论。

环境变量：
  CUBE_PLACE_LADDER_LOG_DIR   覆盖日志目录（默认 logs/2_4b_rotation_ladder）
EOF
}

DRY_RUN=0
ONLY_RUNG=""
MAX_DRZ="0.15"
MAX_SPEED="0.15"
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --dry-run) DRY_RUN=1; shift ;;
    --rung) ONLY_RUNG="$2"; shift 2 ;;
    --max-drz) MAX_DRZ="$2"; shift 2 ;;
    --max-speed) MAX_SPEED="$2"; shift 2 ;;
    *) echo "未知参数: $1" >&2; usage; exit 2 ;;
  esac
done

mkdir -p "${LOG_DIR}"
STAMP="$(date +%Y%m%d-%H%M%S)"

# Rung definition: "label|test_flag|drz|seconds_or_ramp_speed|extra_args"
# Amplitude/speed scaled by --max-drz/--max-speed so a cautious operator can
# shrink the whole ladder with two flags rather than editing rung math.
scale_drz() { python3 -c "print(min(float('$1'), float('${MAX_DRZ}')))"; }
scale_speed() { python3 -c "print(min(float('$1'), float('${MAX_SPEED}')))"; }

RUNGS=(
  "1_ramp_slow|--test-rotation|$(scale_drz 0.05)|--seconds 2|smooth ramp, ~0.025rad/s"
  "2_ramp_mid|--test-rotation|$(scale_drz 0.10)|--rot-ramp-speed $(scale_speed 0.10)|smooth ramp, 0.10rad/s"
  "3_step_replay_slow|--test-rotation-waypoints|$(scale_drz 0.05)|--seconds 1|10Hz ZOH, ~0.05rad/s (matches a moderate action)"
  "4_step_replay_fast|--test-rotation-waypoints|$(scale_drz 0.15)|--rot-ramp-speed $(scale_speed 0.15)|10Hz ZOH at the interp-speed cap"
)

echo "=== 2.4b-rot ladder: $(date) ==="
echo "amplitude cap --max-drz=${MAX_DRZ}rad, speed cap --max-speed=${MAX_SPEED}rad/s"
echo "logs -> ${LOG_DIR}"
echo

declare -a SUMMARY
FAILED=0
for rung in "${RUNGS[@]}"; do
  IFS='|' read -r name flag drz duration_args note <<< "${rung}"
  if [[ -n "${ONLY_RUNG}" && "${name}" != "${ONLY_RUNG}"* ]]; then
    continue
  fi
  logfile="${LOG_DIR}/rung_${name}_${STAMP}.log"
  # shellcheck disable=SC2206
  duration_arr=(${duration_args})
  cmd=(python "${DIAG}" "${flag}" --yes-move --drz "${drz}" "${duration_arr[@]}")
  echo "--- rung ${name}: ${note} ---"
  echo "cmd: ${cmd[*]}"
  if [[ "${DRY_RUN}" == "1" ]]; then
    echo "(dry-run, not executed)"
    echo
    continue
  fi
  set +e
  "${cmd[@]}" >"${logfile}" 2>&1
  rc=$?
  set -e
  tail -n 3 "${logfile}"
  verdict_line="$(grep -E "verdict:" "${logfile}" || true)"
  summary_line="$(grep -E "^rotation ramp|^step\(\)|=== rotation|=== step\(\)" "${logfile}" | tail -1 || true)"
  trace_line="$(grep -E "^(rotation ramp|step\(\) rotation replay).*:.*samples=" "${logfile}" || true)"
  SUMMARY+=("${name}|rc=${rc}|${trace_line:-no trace line found}")
  if [[ ${rc} -ne 0 ]]; then
    FAILED=1
    echo
    echo "!!! rung ${name} FAILED (rc=${rc}). Full log: ${logfile}"
    echo "!!! stopping the ladder here -- see dmo_place_2 Section 6 before retrying."
    break
  fi
  if ! echo "${verdict_line}" | grep -q "alive=True"; then
    FAILED=1
    echo
    echo "!!! rung ${name} verdict says alive=False even though rc=0. Full log: ${logfile}"
    break
  fi
  echo "rung ${name}: PASS"
  echo
done

echo
echo "=== summary ==="
for line in "${SUMMARY[@]:-}"; do
  [[ -z "${line}" ]] && continue
  echo "${line}"
done

if [[ "${DRY_RUN}" == "1" ]]; then
  echo "(dry-run: nothing was executed)"
  exit 0
fi

if [[ "${FAILED}" == "1" ]]; then
  echo
  echo "LADDER FAILED. Do not raise RLINF_CUBE_STEP_ROT_SPEED or resume training"
  echo "until the failing rung's log has been read and understood (dmo_place_2 Sec 6)."
  exit 1
fi

echo
echo "All rungs PASSED up to --max-drz=${MAX_DRZ}rad / --max-speed=${MAX_SPEED}rad/s."
echo "This validates motion_limits.STEP_ROT_SPEED_RAD_S_DEFAULT only up to what was"
echo "actually run above -- read each rung's peak_ang_lag / peak|dq| in ${LOG_DIR}"
echo "before deciding whether the provisional 0.3rad/s default may be trusted, and"
echo "note this ladder ran at REST, not at the NEAR-SINGULAR box corner LOG-034"
echo "tripped at (see this script's header)."
