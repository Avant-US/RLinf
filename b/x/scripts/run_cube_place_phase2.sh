#!/usr/bin/env bash
# Phase 2 helper for FrankyCubePlaceEnv-v1. Run **inside** the franky container
# after: source b/x/configs/setup_before_ray_5090.sh
# This script does not start Docker and does not run SAC.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "${ROOT}"
POSE_FILE="${CUBE_PLACE_POSE_FILE:-${ROOT}/b/x/configs/cube_place_target_ee_pose.yaml}"
SCRIPT="${ROOT}/b/x/scripts/step_cube_place_robot.py"

usage() {
  cat <<'EOF'
用法（必须已在 franky 容器内，且 source 过 setup_before_ray_5090.sh）：

  bash b/x/scripts/run_cube_place_phase2.sh calibrate   # 交互 open/close/getpos_euler
  bash b/x/scripts/run_cube_place_phase2.sh write-pose x y z roll pitch yaw
  bash b/x/scripts/run_cube_place_phase2.sh connect     # 2.6 只读 TCP，不创建 env
  bash b/x/scripts/run_cube_place_phase2.sh reset       # 2.7 reset 到标记上方悬停（会动臂）
  bash b/x/scripts/run_cube_place_phase2.sh box         # 2.8 reset + 零动作 + 少量下探（会动臂）

别名：connect=2c，reset=2d，box=2e。

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

case "${cmd}" in
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
    python "${SCRIPT}" --pose-file "${POSE_FILE}" --connect-only
    ;;
  reset|2d)
    python "${SCRIPT}" --pose-file "${POSE_FILE}" --reset-only
    ;;
  box|2e)
    python "${SCRIPT}" --pose-file "${POSE_FILE}" --num-steps 3 --approach-steps 3
    ;;
  *)
    echo "未知子命令: ${cmd}" >&2
    usage
    exit 2
    ;;
esac
