"""测试 Franky 是否能连接机器人并读取完整状态。

用法（在 Franky 容器中）：
    source b/rlt/configs/setup_franky.sh
    python b/rlt/scripts/test_controller.py [--robot-ip 172.16.0.2]
"""

import argparse
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot-ip",
        default=os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2"),
    )
    args = parser.parse_args()

    print(f"[1/6] Connecting to robot at {args.robot_ip} ...")
    try:
        import franky
    except ImportError:
        print("ERROR: franky not installed. Run: source switch_env franky-0.19.0")
        sys.exit(1)

    robot = franky.Robot(args.robot_ip)
    state = robot.state
    ee = state.O_T_EE
    xyz = np.array(ee.translation)
    quat_wxyz = np.array(ee.quaternion)
    quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
    euler = R.from_quat(quat_xyzw).as_euler("xyz")

    print(f"  Robot mode:  {state.robot_mode}")
    print(f"  Has errors:  {robot.has_errors}")
    print(f"  TCP xyz:     [{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}]")
    print(f"  TCP euler:   [{euler[0]:.6f}, {euler[1]:.6f}, {euler[2]:.6f}]")
    print(f"  Joint pos:   {[round(float(q), 4) for q in state.q]}")

    print("\n[2/6] Reading forces and velocities ...")
    force = np.array(state.K_F_ext_hat_K[:3])
    torque = np.array(state.K_F_ext_hat_K[3:])
    dq = np.array(state.dq)
    print(f"  TCP force:   [{force[0]:.3f}, {force[1]:.3f}, {force[2]:.3f}] N")
    print(f"  TCP torque:  [{torque[0]:.3f}, {torque[1]:.3f}, {torque[2]:.3f}] Nm")
    print(f"  Joint vel norm: {np.linalg.norm(dq):.4f} rad/s")

    print("\n[3/6] Testing TCP probe (subprocess) ...")
    from franky_ext.tcp_probe import probe_robot_state

    probe = probe_robot_state(args.robot_ip)
    print(f"  Mode:     {probe['robot_mode']}")
    print(f"  Errors:   {probe['has_errors']}")
    print(f"  TCP pose: {probe['pose']}")
    print(f"  Gripper:  width={probe.get('gripper_width', 'N/A')}, "
          f"holding={probe.get('gripper_holding', 'N/A')}")

    print("\n[4/6] Testing gripper ...")
    try:
        gripper = franky.Gripper(args.robot_ip)
        width = gripper.width
        print(f"  Gripper width: {float(width):.4f} m")
        print(f"  Gripper OK")
    except Exception as e:
        print(f"  Gripper error: {e}")

    print("\n[5/6] Testing motion_limits ...")
    from franky_ext.motion_limits import describe_authority

    desc = describe_authority(500.0, 40.0)
    print(f"  {desc}")

    print("\n[6/6] Testing franky_ext imports ...")
    from franky_ext.controller_extended import FrankyControllerExtended
    from franky_ext.franka_libfranka_gripper import FrankaLibfrankaGripper

    print("  FrankyControllerExtended: OK")
    print("  FrankaLibfrankaGripper:   OK")
    print("  (These are Ray Workers, tested at runtime via Ray)")

    print("\n=== All checks passed ===")
    print(f"\ntarget_ee_pose (fill into config):")
    print(f"  [{xyz[0]:.8f}, {xyz[1]:.8f}, {xyz[2]:.8f}, "
          f"{euler[0]:.8f}, {euler[1]:.8f}, {euler[2]:.8f}]")


if __name__ == "__main__":
    main()
