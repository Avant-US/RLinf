# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""YAML config validation for the new SingleArmReach tasks.

Per design doc r1pro6op47.md §11, we ship YAML env configs at
``examples/embodiment/config/env/realworld_galaxea_r1_pro_singlearm_reach_*.yaml``.
These tests load them with OmegaConf and instantiate the env in
dummy mode to ensure the YAML keys all flow through to the env
constructor without typos.

We do NOT exercise the full Hydra+RLinf launcher here -- that would
require Ray etc., which is out of scope for unit tests.  Instead we
read the ``init_params.override_cfg`` block and pass it to gym.make.
"""

from __future__ import annotations

from pathlib import Path

import gymnasium as gym
from omegaconf import OmegaConf

# Trigger gym registration.
import rlinf.envs.realworld.galaxear.tasks  # noqa: F401

CONFIG_DIR = (
    Path(__file__).resolve().parents[2] / "examples" / "embodiment" / "config" / "env"
)


def _load_override_cfg(yaml_name: str) -> tuple[str, dict]:
    """Read the env yaml and return (init_params.id, override_cfg dict)."""
    path = CONFIG_DIR / yaml_name
    assert path.is_file(), f"YAML missing: {path}"
    cfg = OmegaConf.load(path)
    init_params = cfg["init_params"]
    env_id = init_params["id"]
    override_cfg = OmegaConf.to_container(
        init_params.get("override_cfg", {}),
        resolve=False,  # leave ${oc.env:...} unresolved; we'll patch
    )
    # Resolve ROS_DOMAIN_ID interpolation manually so it doesn't barf if
    # the env var is not set at test time.
    if isinstance(override_cfg, dict):
        if override_cfg.get("ros_domain_id") is None:
            override_cfg["ros_domain_id"] = 41
        if isinstance(override_cfg.get("ros_domain_id"), str):
            override_cfg["ros_domain_id"] = 41
    # Force dummy regardless of YAML default (no ROS 2 in unit tests).
    override_cfg["is_dummy"] = True
    override_cfg["step_frequency"] = 1000.0  # don't sleep
    return env_id, override_cfg


# ─────────────────── M1 SingleArmReach joint yaml ──────────────


def test_singlearm_reach_joint_yaml_loads():
    env_id, _ = _load_override_cfg(
        "realworld_galaxea_r1_pro_singlearm_reach_joint.yaml"
    )
    assert env_id == "GalaxeaR1ProSingleArmReach-joint-v1"


def test_singlearm_reach_joint_yaml_env_constructs():
    env_id, override_cfg = _load_override_cfg(
        "realworld_galaxea_r1_pro_singlearm_reach_joint.yaml"
    )
    env = gym.make(env_id, override_cfg=override_cfg)
    try:
        # action_dim should be 7 (joint mode, no_gripper=true)
        assert env.action_space.shape == (7,)
        obs, _ = env.reset()
        assert "state" in obs
        assert "right_arm_qpos" in obs["state"]
        for _ in range(5):
            import numpy as np

            obs, reward, terminated, truncated, info = env.step(np.zeros(7))
            assert isinstance(reward, float)
    finally:
        env.close()


def test_singlearm_reach_joint_yaml_use_joint_mode_true():
    _, override_cfg = _load_override_cfg(
        "realworld_galaxea_r1_pro_singlearm_reach_joint.yaml"
    )
    assert override_cfg["use_joint_mode"] is True
    assert override_cfg["use_right_arm"] is True
    assert override_cfg["no_gripper"] is True


def test_singlearm_reach_joint_yaml_topics_match_design_doc():
    """The YAML uses joint mode -- the JointStateDispatcher will
    require /motion_target/target_joint_state_arm_right per
    r1pro6op47.md §3.2.6.  We can't really hit ROS 2 in a unit test,
    but we can introspect the dispatcher's required topics.
    """
    import numpy as np

    from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import (
        build_action_dispatcher,
    )
    from rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer import (
        GripperMixer,
    )

    _, override_cfg = _load_override_cfg(
        "realworld_galaxea_r1_pro_singlearm_reach_joint.yaml"
    )
    disp = build_action_dispatcher(
        use_joint_mode=override_cfg["use_joint_mode"],
        use_right_arm=override_cfg["use_right_arm"],
        use_left_arm=override_cfg.get("use_left_arm", False),
        no_gripper=override_cfg["no_gripper"],
        gripper_mixer=GripperMixer(0.0, 90.0),
        controller=None,
        q_min_right=np.asarray(override_cfg["arm_q_min_right"]),
        q_max_right=np.asarray(override_cfg["arm_q_max_right"]),
    )
    topics = disp.get_required_topics()
    assert "/motion_target/target_joint_state_arm_right" in topics
    assert "/motion_target/target_pose_arm_right" not in topics


# ─────────────────── M2 SingleArmReach EE yaml ─────────────────


def test_singlearm_reach_ee_yaml_loads():
    env_id, _ = _load_override_cfg("realworld_galaxea_r1_pro_singlearm_reach_ee.yaml")
    assert env_id == "GalaxeaR1ProSingleArmReach-ee-v1"


def test_singlearm_reach_ee_yaml_env_constructs():
    env_id, override_cfg = _load_override_cfg(
        "realworld_galaxea_r1_pro_singlearm_reach_ee.yaml"
    )
    env = gym.make(env_id, override_cfg=override_cfg)
    try:
        # action_dim should be 7 (3 xyz + 4 quat, no gripper).
        assert env.action_space.shape == (7,)
        obs, _ = env.reset()
        import numpy as np

        a = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32)
        for _ in range(3):
            obs, reward, terminated, truncated, info = env.step(a)
            assert isinstance(reward, float)
    finally:
        env.close()


def test_singlearm_reach_ee_yaml_topics_match_design_doc():
    import numpy as np

    from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import (
        build_action_dispatcher,
    )
    from rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer import (
        GripperMixer,
    )

    _, override_cfg = _load_override_cfg(
        "realworld_galaxea_r1_pro_singlearm_reach_ee.yaml"
    )
    disp = build_action_dispatcher(
        use_joint_mode=override_cfg["use_joint_mode"],
        use_right_arm=override_cfg["use_right_arm"],
        use_left_arm=override_cfg.get("use_left_arm", False),
        no_gripper=override_cfg["no_gripper"],
        gripper_mixer=GripperMixer(0.0, 90.0),
        controller=None,
        ee_min_right=np.asarray(override_cfg["ee_min_right"]),
        ee_max_right=np.asarray(override_cfg["ee_max_right"]),
    )
    topics = disp.get_required_topics()
    assert "/motion_target/target_pose_arm_right" in topics
    assert "/motion_target/target_joint_state_arm_right" not in topics
