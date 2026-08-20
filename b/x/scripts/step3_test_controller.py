#!/usr/bin/env python3
"""Step 3: non-interactive smoke for FrankyControllerExtended."""

import os
import sys
import time

import numpy as np
import ray

REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, REPO)
# b/x, not b/d: the extension package moved and this line was never updated. It
# went unnoticed because setup_before_ray_5090.sh puts b/x on PYTHONPATH anyway.
sys.path.insert(0, os.path.join(REPO, "b", "x"))

from franky_ext.controller_extended import FrankyControllerExtended

HOME_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


def main() -> int:
    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    gripper_type = os.environ.get("FRANKA_GRIPPER_TYPE", "franka")
    if not ray.is_initialized():
        ray.init(log_to_driver=False, logging_level="ERROR")

    print(f"Step3: FrankyControllerExtended on {robot_ip} gripper={gripper_type}")
    ctrl = FrankyControllerExtended.launch_controller(
        robot_ip=robot_ip, gripper_type=gripper_type
    )
    t0 = time.time()
    while not ctrl.is_robot_up().wait()[0]:
        time.sleep(0.5)
        if time.time() - t0 > 30:
            raise RuntimeError("robot not up after 30s")

    state = ctrl.get_state().wait()[0]
    print(f"get_state tcp_pose[:3]={state.tcp_pose[:3]}")

    ctrl.reset_joint(HOME_JOINTS).wait()
    print("home OK")

    ctrl.open_gripper().wait()
    time.sleep(1.0)
    ctrl.close_gripper().wait()
    print("open/close OK")

    ctrl.move_gripper(128).wait()
    print("move_gripper(128) OK")

    ctrl.reconfigure_compliance_params(
        {"translational_stiffness": 2000.0, "rotational_stiffness": 150.0}
    ).wait()
    print("reconfigure_compliance_params OK")

    current = ctrl.get_state().wait()[0].arm_joint_position
    target = current.copy()
    target[0] += 0.05
    ctrl.move_joints(target).wait()
    print("nudge joint0 OK")

    try:
        ctrl.cleanup().wait()
    except Exception:
        pass
    print("Step3 PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
