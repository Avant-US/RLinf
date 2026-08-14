#!/usr/bin/env python3
"""Step 4: dummy FrankyFrankaEnv-v1 make/reset/step."""

import os
import sys

import gymnasium as gym

REPO = os.environ.get("REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../..")))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "d"))

import franky_ext.tasks.register  # noqa: F401


def main() -> int:
    env = gym.make(
        "FrankyFrankaEnv-v1",
        override_cfg={
            "is_dummy": True,
            "camera_serials": ["000000000000"],
            "target_ee_pose": [0.5, 0.0, 0.1, 0.0, 0.0, 0.0],
        },
        worker_info=None,
        hardware_info=None,
        env_idx=0,
        env_cfg={},
    )
    obs, _ = env.reset()
    assert "state" in obs or env.config.is_dummy
    action = env.action_space.sample()
    obs, reward, term, trunc, info = env.step(action)
    assert obs is not None
    env.close()
    print("Step4 PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
