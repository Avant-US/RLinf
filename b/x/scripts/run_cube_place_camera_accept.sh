#!/usr/bin/env bash
# S3.3 camera acceptance for the cube-place link (b/d/frk1/dmo_place_2.md §S3.3).
#
# Runs each step and verifies its machine-readable marker instead of leaving
# the operator to eyeball logs:
#
#   8a   step8_detect_cameras.py --write-yaml --yaml-out <cube-place carrier>
#        --serials <allowlist from realworld_cube_place_sac.yaml>
#        -> extra plugged-in cameras are ignored; a wanted camera that is
#           missing fails here (requested_serials_present)
#   8b   step8_check_yaml.py --expect-gym-id FrankyCubePlaceEnv-v1
#        --expect-serials-from realworld_cube_place_sac.yaml
#        -> carrier serials == camera_detected.json == training config,
#           no placeholders, wrist_1 named, camera_names unique
#   gate diag_franky_motion.py --probe must report robot_mode=RobotMode.Idle
#   mem  MemAvailable >= 20GB (the controller worker alone needs ~16GB;
#        reusing a busy cluster got it OOM-killed in LOG-031)
#   8c   step8_test_env_camera.py --save-jpeg --require-live
#        (FCI; FrankyFrankaEnv-v1 with target=current pose => ~zero motion)
#   link run_cube_place_phase2.sh reset --with-camera
#        (FCI; FrankyCubePlaceEnv-v1 does a REAL reset-to-hover)
#
# 8c/链路自检按设计跑在 franky 容器的**单节点**集群上（§S3.3）。检测到已有
# 集群时本脚本默认中止——复用别人的集群（如 §S3.2 的双节点集群）会把
# node_rank=0 变成别的节点（控制器被调度到没有相机的容器），且其 idle
# worker 占用内存会 OOM 杀掉新 worker（LOG-031）。确认现有集群就是本容器
# 的单节点集群时才可加 --reuse-cluster。
#
# Safety: 8c and the link check hold FCI and reset the arm. Cube grasped, arm a
# few cm above the mark, user-stop UP, human at the e-stop. --no-fci runs only
# 8a+8b (no robot contact at all).
#
# Run INSIDE the franky container after:
#   source b/x/configs/setup_before_ray_5090.sh
#
# Logs go to b/x/logs/camera_accept/<timestamp>/; exit 0 only if every step
# passes.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "${ROOT}"
export REPO_PATH="${REPO_PATH:-${ROOT}}"

CAMERA_YAML="${ROOT}/b/x/configs/realworld_cube_place_camera.yaml"
CAMERA_JSON="${ROOT}/b/x/configs/camera_detected.json"
TRAIN_YAML="${ROOT}/b/x/configs/realworld_cube_place_sac.yaml"
MIN_AVAIL_MB="${CAMERA_ACCEPT_MIN_AVAIL_MB:-20480}"

NO_FCI=0
REUSE_CLUSTER=0
SERIALS=()
usage() {
  cat <<'EOF'
用法（franky 容器内，已 source setup_before_ray_5090.sh）：

  bash b/x/scripts/run_cube_place_camera_accept.sh            # 8a + 8b + 8c + 链路自检
  bash b/x/scripts/run_cube_place_camera_accept.sh --no-fci   # 只跑 8a + 8b（不碰机器人）

选项：
  --no-fci         只跑 8a + 8b（不占 FCI、不动臂）
  --reuse-cluster  复用已有 Ray 集群（仅当你确定它是本容器的单节点集群；
                   默认检测到已有集群就中止，见 LOG-031）
  --serials S...   8a 只保留这些 serial（默认取训练 YAML 的 camera_serials——
                   多插的相机自动忽略，要用的不在线会在 8a FAIL）

8c 与链路自检会占 FCI 并 reset 臂：方块夹紧、臂在标记上方几厘米、
user-stop 松开、人在急停旁。
EOF
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-fci) NO_FCI=1; shift ;;
    --reuse-cluster) REUSE_CLUSTER=1; shift ;;
    --serials)
      shift
      while [[ $# -gt 0 && "$1" != -* ]]; do SERIALS+=("$1"); shift; done
      ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

LOG_DIR="${ROOT}/b/x/logs/camera_accept/$(date +%Y%m%d_%H%M%S)"
if ! mkdir -p "${LOG_DIR}" 2>/dev/null; then
  # b/x/logs may be root-owned when it was created inside a container.
  LOG_DIR="$(mktemp -d /tmp/camera_accept.XXXXXX)"
fi

PY="$(command -v python || true)"
case "${PY}" in
  *franky-0.19.0*) ;;
  *)
    echo "ERROR: python 不是 franky-0.19.0（当前: ${PY:-none}）。" >&2
    echo "先 source b/x/configs/setup_before_ray_5090.sh" >&2
    exit 2
    ;;
esac

FAILED_STEPS=()

run_step() {
  # run_step <name> <cmd...> — tee output to a per-step log; never dies under
  # `set -e`, failure is recorded so the summary can list every failed step.
  local name="$1"; shift
  local log="${LOG_DIR}/${name}.log"
  echo ""
  echo "=== [${name}] $* ==="
  if "$@" 2>&1 | tee "${log}"; then
    echo "--- [${name}] exit 0"
    return 0
  else
    local rc=$?
    echo "--- [${name}] FAILED (exit ${rc}); log: ${log}"
    FAILED_STEPS+=("${name}")
    return 1
  fi
}

require_grep() {
  # require_grep <step-name> <log> <pattern> <what it proves>
  local name="$1" log="$2" pattern="$3" what="$4"
  if grep -q -- "${pattern}" "${log}"; then
    echo "    [${name}] verified: ${what}"
    return 0
  fi
  echo "    [${name}] MISSING '${pattern}' (${what})"
  FAILED_STEPS+=("${name}:grep")
  return 1
}

echo "camera-accept: logs -> ${LOG_DIR}"

# 8a 的 serial 白名单：默认取训练 YAML 的 camera_serials。训练配置是期望集合
# 的锚点——多插的相机自动忽略（LOG-031 的第二只 D435I），要用的不在线会在
# 8a 的 requested_serials_present 处 FAIL。
if [[ ${#SERIALS[@]} -eq 0 ]]; then
  mapfile -t SERIALS < <(python - "${TRAIN_YAML}" <<'EOF'
import sys

sys.path.insert(0, "b/x/scripts")
import yaml
from step8_checks import collect_yaml_serials

with open(sys.argv[1], encoding="utf-8") as fh:
    _, serials = collect_yaml_serials(yaml.safe_load(fh) or {})
for s in serials:
    print(s)
EOF
  )
fi
if [[ ${#SERIALS[@]} -eq 0 ]]; then
  echo "ERROR: 从 ${TRAIN_YAML} 没解析到 camera_serials" >&2
  exit 2
fi
echo "camera allowlist: ${SERIALS[*]}（来自 $(basename "${TRAIN_YAML}")，可用 --serials 覆盖）"

# ---------------------------------------------------------------- 8a + 8b (no FCI)

run_step 8a_detect \
  python b/x/scripts/step8_detect_cameras.py --write-yaml --yaml-out "${CAMERA_YAML}" \
    --serials "${SERIALS[@]}" \
  && require_grep 8a_detect "${LOG_DIR}/8a_detect.log" "RESULT Step8a PASS" \
     "相机被枚举、白名单 serial 在线且非占位、已写入 $(basename "${CAMERA_YAML}")"

run_step 8b_check_yaml \
  python b/x/scripts/step8_check_yaml.py \
    --json-in "${CAMERA_JSON}" --yaml-in "${CAMERA_YAML}" \
    --expect-gym-id FrankyCubePlaceEnv-v1 \
    --expect-serials-from "${TRAIN_YAML}" \
  && require_grep 8b_check_yaml "${LOG_DIR}/8b_check_yaml.log" "RESULT Step8b PASS" \
     "载体/JSON/训练配置三方 serial 一致、is_dummy=false、id=FrankyCubePlaceEnv-v1、wrist_1 唯一"

if [[ " ${FAILED_STEPS[*]:-} " == *" 8b_check_yaml "* ]] \
  && grep -q "train_yaml_serials_match FAIL" "${LOG_DIR}/8b_check_yaml.log"; then
  cat >&2 <<'EOF'

8b 的 train_yaml_serials_match 失败：JSON/载体 与训练配置的相机集合不一致。
8a 默认已按训练 YAML 的 serial 白名单过滤，所以这里 FAIL 通常是手工改过
某个文件、或 --serials 覆盖过。以训练 YAML 为锚点对齐后重跑；
要改用别的相机（或双相机），先改训练 YAML 的 camera_serials / camera_names
（顺序决定谁是 wrist_1）再重跑。
EOF
fi

if [[ "${NO_FCI}" == "1" ]]; then
  echo ""
  if [[ ${#FAILED_STEPS[@]} -eq 0 ]]; then
    echo "CAMERA_ACCEPT (no-fci) PASS: 8a + 8b"
    exit 0
  fi
  echo "CAMERA_ACCEPT (no-fci) FAIL: ${FAILED_STEPS[*]}" >&2
  exit 1
fi

# ---------------------------------------------------------------- mode gate (FCI steps)

run_step gate_probe python b/x/scripts/diag_franky_motion.py --probe
if ! grep -q "robot_mode=RobotMode.Idle" "${LOG_DIR}/gate_probe.log"; then
  echo "" >&2
  echo "ABORT: robot_mode 不是 Idle（见上）。松开 user-stop / 引导键、Desk 清 fault 后重跑。" >&2
  echo "8c 与链路自检没有执行。" >&2
  exit 1
fi
echo "    [gate_probe] verified: robot_mode=RobotMode.Idle"

# ---------------------------------------------------------------- memory gate

AVAIL_MB="$(awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo)"
echo "mem: MemAvailable=${AVAIL_MB}MB（要求 >= ${MIN_AVAIL_MB}MB；控制器 worker 初始化约 16GB，LOG-031 的 OOM kill 就是内存过线）"
if [[ "${AVAIL_MB}" -lt "${MIN_AVAIL_MB}" ]]; then
  echo "ABORT: 可用内存不足。先停掉占内存的集群/进程（如 §S3.2 的双节点集群：" >&2
  echo "docker exec rlinf-franky-5090 /opt/venv/franky-0.19.0/bin/ray stop;" >&2
  echo "docker exec rlinf-gpu-5090 /opt/venv/openvla/bin/ray stop），再重跑。" >&2
  exit 1
fi

# ---------------------------------------------------------------- ray lifecycle

RAY_STARTED_BY_US=0
cleanup() {
  if [[ "${RAY_STARTED_BY_US}" == "1" ]]; then
    echo "cleanup: ray stop（本脚本启动的集群）"
    ray stop >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

if ray status >/dev/null 2>&1; then
  if [[ "${REUSE_CLUSTER}" == "1" ]]; then
    echo "ray: --reuse-cluster，复用现有集群（结束时不会 ray stop）"
  else
    cat >&2 <<'EOF'
ABORT: 检测到已有 Ray 集群。8c/链路自检按设计必须在 franky 容器的**单节点**
集群上跑（dmo_place_2.md §S3.3）。复用别人的集群（如 §S3.2 的双节点集群）
的两个实际后果（LOG-031）：node_rank=0 不再是本容器，控制器 worker 会被
调度到没有相机 USB 的节点；且其 idle worker 占用内存会把新 worker OOM 杀掉。
先停掉现有集群再重跑（宿主机的 ray stop 命令见上方 mem gate 的提示）；
确认现有集群就是本容器的单节点集群时，才可加 --reuse-cluster。
EOF
    exit 1
  fi
else
  echo "ray: 无集群，启动单节点 head"
  ray start --head --port=6379 --disable-usage-stats
  RAY_STARTED_BY_US=1
fi

# ---------------------------------------------------------------- 8c (FCI, ~zero motion)

run_step 8c_env_camera \
  python b/x/scripts/step8_test_env_camera.py --save-jpeg --require-live \
  && require_grep 8c_env_camera "${LOG_DIR}/8c_env_camera.log" "RESULT Step8c PASS" \
     "wrist_1 为 uint8 128x128x3、max>0、零动作步之间帧有变化"

# -------------------------------------------------- cube-place link (FCI, real reset)

if [[ " ${FAILED_STEPS[*]} " == *" 8c_env_camera "* ]]; then
  echo "跳过链路自检：8c 未过（先修相机本身，再查 cube-place 链路的接线）"
else
  run_step cube_link \
    bash b/x/scripts/run_cube_place_phase2.sh reset --with-camera
  if [[ ! " ${FAILED_STEPS[*]} " == *" cube_link "* ]]; then
    require_grep cube_link "${LOG_DIR}/cube_link.log" "reset-only PASS" \
      "FrankyCubePlaceEnv-v1 reset 到悬停"
    require_grep cube_link "${LOG_DIR}/cube_link.log" "camera frame wrist_1:" \
      "obs['frames'] 里有 wrist_1"
    if grep -q "ALL-ZERO" "${LOG_DIR}/cube_link.log"; then
      echo "    [cube_link] FAIL: wrist_1 是 ALL-ZERO（stub 或采集失败）——8c 过而这里全零，说明 override/YAML 的相机键没传到"
      FAILED_STEPS+=("cube_link:all-zero")
    else
      echo "    [cube_link] verified: wrist_1 非全零"
    fi
  fi
fi

# ---------------------------------------------------------------- summary

echo ""
if [[ ${#FAILED_STEPS[@]} -eq 0 ]]; then
  echo "CAMERA_ACCEPT PASS: 8a + 8b + mode gate + 8c + cube-place 链路"
  echo "日志: ${LOG_DIR}"
  exit 0
fi
echo "CAMERA_ACCEPT FAIL: ${FAILED_STEPS[*]}" >&2
echo "日志: ${LOG_DIR}" >&2
exit 1
