#!/usr/bin/env bash
# Phase 2 helper for FrankyCubePlaceEnv-v1. Run **inside** the franky container
# after: source b/x/configs/setup_before_ray_5090.sh
# This script does not start Docker and does not run SAC.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "${ROOT}"
# The python scripts resolve their default pose file and sys.path from REPO_PATH.
# setup_before_ray_5090.sh exports it, but exporting it here too means this helper
# does not silently depend on having been sourced (LOG-023 finding 6).
export REPO_PATH="${REPO_PATH:-${ROOT}}"
POSE_FILE="${CUBE_PLACE_POSE_FILE:-${ROOT}/b/x/configs/cube_place_target_ee_pose.yaml}"
SCRIPT="${ROOT}/b/x/scripts/step_cube_place_robot.py"

usage() {
  cat <<'EOF'
用法（必须已在 franky 容器内，且 source 过 setup_before_ray_5090.sh）：

  bash b/x/scripts/run_cube_place_phase2.sh diag-probe  # 2.4 只读：robot_mode / TCP / 夹爪
  bash b/x/scripts/run_cube_place_phase2.sh diag        # 2.4 hold + ramp + settle（会动臂 3cm）
  bash b/x/scripts/run_cube_place_phase2.sh diag-replay # 2.4 复现 _interpolate_move 阶梯（会动臂）
  bash b/x/scripts/run_cube_place_phase2.sh diag-rot        # 2.4b-rot 单挡：姿态 ramp+settle（会动臂，LOG-034/T16）
  bash b/x/scripts/run_cube_place_phase2.sh diag-rot-replay  # 2.4b-rot 单挡：10Hz 姿态阶梯，复现 step()（会动臂）
  bash b/x/scripts/run_cube_place_phase2.sh calibrate   # 交互 open/close/getpos_euler
  bash b/x/scripts/run_cube_place_phase2.sh write-pose x y z roll pitch yaw
  bash b/x/scripts/run_cube_place_phase2.sh connect     # 2.6 只读 TCP，不创建 env
  bash b/x/scripts/run_cube_place_phase2.sh reset       # 2.7 reset 到标记上方悬停（会动臂）
  bash b/x/scripts/run_cube_place_phase2.sh box         # 2.8 reset + 零动作 + 少量下探（会动臂）

别名：connect=2c，reset=2d，box=2e，diag-probe=2a，diag=2b。
所有子命令后面多余的参数会原样传给对应的 python 脚本，例如：
  bash b/x/scripts/run_cube_place_phase2.sh diag --dz 0.05 --settle-seconds 3
  bash b/x/scripts/run_cube_place_phase2.sh reset --force-ceiling 15 --interp-speed 0.01

顺序：diag-probe → diag → (diag-replay) → connect → reset → box。
2.4 的运动体检必须先过，且必须在**真实起始位形**（夹着方块、标记上方几厘米）上做——
在别的位形/别的速度上通过，不能推出 reset 也通过（LOG-018 / LOG-019）。

2.4b-rot（LOG-034/T16，姿态阶梯，从未在这条臂上测过的授权）：
  bash b/x/scripts/run_cube_place_phase2.sh diag-rot --drz 0.05 --seconds 1
  bash b/x/scripts/run_cube_place_phase2.sh diag-rot-replay --drz 0.05 --seconds 1
  # 更省事：完整梯子 + 自动止损 + 汇总，见
  bash b/x/scripts/run_2_4b_rotation_ladder.sh --help

宿主机进容器（本脚本不代跑 docker）：
  bash /home/nvidia/bt/s/RLinf/b/x/configs/docker_run_franky_5090.sh
  # 或: docker exec -it rlinf-franky-5090 bash
  source /workspace/RLinf/b/x/configs/setup_before_ray_5090.sh

标定交互（会占 FCI，与 connect/reset/box 互斥；本机原装 Franka Hand）：
  python b/x/scripts/test_franky_controller_ext.py
  # 不要用 python -m toolkits.realworld_check.test_franky_controller
  # （upstream FrankyController 不支持 gripper_type=franka）
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 1 ]]; then
  usage
  exit 0
fi

cmd="$1"
shift

export FRANKA_ROBOT_IP="${FRANKA_ROBOT_IP:-172.16.0.2}"
export FRANKA_GRIPPER_TYPE="${FRANKA_GRIPPER_TYPE:-franka}"
export RLINF_SKIP_CAMERA="${RLINF_SKIP_CAMERA:-1}"

DIAG="${ROOT}/b/x/scripts/diag_franky_motion.py"

case "${cmd}" in
  diag-probe|2a)
    python "${DIAG}" --probe "$@"
    ;;
  diag|2b)
    # hold (liveness) + ramp with a settle window (overshoot). Deliberately no
    # --test-cartesian-motion: chaining it after a ramp reflexes on residual
    # joint velocity (LOG-018), and T1 is closed so reset does not need it.
    python "${DIAG}" --test-hold --test-impedance --yes-move --dz 0.03 "$@"
    ;;
  diag-replay)
    # Reproduce the open-loop _interpolate_move staircase that reset commands.
    python "${DIAG}" --test-waypoints --yes-move --dz 0.03 "$@"
    ;;
  diag-rot)
    # LOG-034/T16: rotation ramp+settle, xyz held fixed. Default --drz 0.05rad
    # (~2.9deg) is deliberately small -- this authority has never been
    # exercised on this arm before. Escalate with --drz / --seconds yourself,
    # or use run_2_4b_rotation_ladder.sh for the full staged ladder.
    python "${DIAG}" --test-rotation --yes-move --drz 0.05 --seconds 1 "$@"
    ;;
  diag-rot-replay)
    # LOG-034/T16: 10Hz ZOH rotation staircase -- the shape step() actually
    # commands one cycle at a time. This is the test that can confirm or
    # refute motion_limits.STEP_ROT_SPEED_RAD_S_DEFAULT.
    python "${DIAG}" --test-rotation-waypoints --yes-move --drz 0.05 --seconds 1 "$@"
    ;;
  calibrate)
    python "${ROOT}/b/x/scripts/test_franky_controller_ext.py"
    ;;
  write-pose)
    if [[ $# -lt 6 ]]; then
      echo "write-pose 需要 6 个数: x y z roll pitch yaw" >&2
      exit 2
    fi
    python "${ROOT}/b/x/scripts/write_cube_place_pose.py" "$@" --path "${POSE_FILE}"
    ;;
  connect|2c)
    python "${SCRIPT}" --pose-file "${POSE_FILE}" --connect-only "$@"
    ;;
  reset|2d)
    python "${SCRIPT}" --pose-file "${POSE_FILE}" --reset-only "$@"
    ;;
  box|2e)
    python "${SCRIPT}" --pose-file "${POSE_FILE}" --num-steps 3 --approach-steps 3 "$@"
    ;;
  *)
    echo "未知子命令: ${cmd}" >&2
    usage
    exit 2
    ;;
esac
