#!/usr/bin/env python3
"""Step 5: real robot FrankyFrankaEnv smoke (no camera)."""

import os
import sys
import time

import gymnasium as gym
import numpy as np
import ray

REPO = os.environ.get("REPO_PATH", os.path.abspath(os.path.join(__file__, "../../..")))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "d"))

import franky_ext.tasks.register  # noqa: F401

from rlinf.scheduler.hardware.robots.franka import FrankaConfig, FrankaRobot
from rlinf.scheduler import FrankaHWInfo, WorkerInfo


def main() -> int:
    os.environ["RLINF_SKIP_CAMERA"] = "1"
    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    if not ray.is_initialized():
        ray.init(log_to_driver=False, logging_level="ERROR")

    hw = FrankaHWInfo(
        config=FrankaConfig(robot_ip=robot_ip, camera_serials=["000000000000"], gripper_type="franka"),
        robot=FrankaRobot(),
    )
    worker_info = WorkerInfo(cluster_node_rank=0, rank=0)

    env = gym.make(
        "FrankyFrankaEnv-v1",
        override_cfg={
            "is_dummy": False,
            "robot_ip": robot_ip,
            "camera_serials": ["000000000000"],
            "gripper_type": "franka",
            "target_ee_pose": [0.5, 0.0, 0.1, 0.0, 0.0, 0.0],
            "reset_ee_pose": [0.5, 0.0, 0.1, 0.0, 0.0, 0.0],
        },
        worker_info=worker_info,
        hardware_info=hw,
        env_idx=0,
        env_cfg={},
    )
    obs, _ = env.reset()
    print("reset OK, tcp_pose=", obs["state"]["tcp_pose"][:3])
    zero = np.zeros(env.action_space.shape, dtype=np.float32)
    for i in range(5):
        obs, reward, term, trunc, info = env.step(zero)
        time.sleep(0.1)
    env.close()
    print("Step5 PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
