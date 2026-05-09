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

"""Tests for the M1 / M2 SingleArmReach joint and EE tasks.

Per design doc r1pro6op47.md §10, these are the very first
real-robot RL tasks (joint mode = M1, ee+quat mode = M2).  They
must:

* Register their gym IDs ``GalaxeaR1ProSingleArmReach-{joint,ee}-v1``.
* Default to the right action mode (joint vs ee+quat).
* Default to single-arm right, no gripper, no torso, no chassis.
* Compute a sensible joint-distance / EE-distance reward.
* Run end-to-end in dummy mode without raising.
"""

from __future__ import annotations

import gymnasium as gym
import numpy as np

# Import for registration side effect.
import rlinf.envs.realworld.galaxear.tasks  # noqa: F401
from rlinf.envs.realworld.galaxear.tasks.r1_pro_single_arm_reach_ee import (
    GalaxeaR1ProSingleArmReachEeEnv,
)
from rlinf.envs.realworld.galaxear.tasks.r1_pro_single_arm_reach_joint import (
    GalaxeaR1ProSingleArmReachJointEnv,
)

# ───────────────────── Joint task registration ─────────────────


def test_joint_task_is_registered():
    """Gym ID must be importable after `import tasks`."""
    from gymnasium.envs.registration import registry

    assert "GalaxeaR1ProSingleArmReach-joint-v1" in registry


def test_ee_task_is_registered():
    from gymnasium.envs.registration import registry

    assert "GalaxeaR1ProSingleArmReach-ee-v1" in registry


# ────────────────────────── Joint task ─────────────────────────


def _joint_dummy_cfg(target_q=None):
    cfg = {
        "is_dummy": True,
        "arm_q_min_right": [-2.0] * 7,
        "arm_q_max_right": [+2.0] * 7,
        "step_frequency": 100.0,
    }
    if target_q is not None:
        cfg["target_q_right"] = list(target_q)
    return cfg


def test_joint_task_action_dim_is_7():
    env = gym.make(
        "GalaxeaR1ProSingleArmReach-joint-v1",
        override_cfg=_joint_dummy_cfg(),
    )
    assert env.action_space.shape == (7,)
    env.close()


def test_joint_task_default_use_joint_mode_true():
    env = GalaxeaR1ProSingleArmReachJointEnv(_joint_dummy_cfg())
    assert env.config.use_joint_mode is True
    assert env.config.use_right_arm is True
    assert env.config.use_left_arm is False
    assert env.config.no_gripper is True


def test_joint_task_reset_runs_in_dummy():
    env = GalaxeaR1ProSingleArmReachJointEnv(_joint_dummy_cfg())
    obs, info = env.reset()
    assert "state" in obs
    assert "right_arm_qpos" in obs["state"]


def test_joint_task_step_runs_in_dummy():
    env = GalaxeaR1ProSingleArmReachJointEnv(_joint_dummy_cfg())
    env.reset()
    obs, reward, terminated, truncated, info = env.step(np.zeros(7))
    assert isinstance(reward, float)
    # Reward should be in a sensible range: dense in [-pi, 0]-ish + 0/1 bonus.
    assert -10.0 < reward < 2.0


def test_joint_task_reward_at_target_is_positive():
    """When current q == target_q, dense ~0 and bonus 1 -> reward ~1."""
    target = np.array([0.5] * 7, dtype=np.float32)
    env = GalaxeaR1ProSingleArmReachJointEnv(_joint_dummy_cfg(target_q=target))
    env.reset()
    # Manually set state.right_arm_qpos = target, then step.
    env._state.right_arm_qpos = target.copy()
    _, reward, _, _, _ = env.step(np.zeros(7))
    # In dummy mode we set state once but env then refreshes through
    # controller (None) -> state stays.  Reward dense=0, bonus=1.
    assert reward > 0.5


def test_joint_task_reward_far_from_target_is_negative():
    target = np.array([0.0] * 7, dtype=np.float32)
    env = GalaxeaR1ProSingleArmReachJointEnv(_joint_dummy_cfg(target_q=target))
    env.reset()
    env._state.right_arm_qpos = np.array([2.0] * 7, dtype=np.float32)
    _, reward, _, _, _ = env.step(np.zeros(7))
    # ||q - q*|| = 2*sqrt(7); dense = -2; bonus = 0.
    assert reward < -1.0


def test_joint_task_task_description_is_set():
    env = GalaxeaR1ProSingleArmReachJointEnv(_joint_dummy_cfg())
    assert (
        "joint" in env.task_description.lower()
        or "right" in env.task_description.lower()
    )


# ──────────────────────────── EE task ──────────────────────────


def _ee_dummy_cfg(target_pose=None):
    cfg = {
        "is_dummy": True,
        "ee_min_right": [0.0, -0.5, 0.0],
        "ee_max_right": [0.8, 0.5, 1.0],
        "step_frequency": 100.0,
    }
    if target_pose is not None:
        cfg["target_pose_right"] = list(target_pose)
    return cfg


def test_ee_task_action_dim_is_7():
    env = gym.make(
        "GalaxeaR1ProSingleArmReach-ee-v1",
        override_cfg=_ee_dummy_cfg(),
    )
    assert env.action_space.shape == (7,)
    env.close()


def test_ee_task_default_use_joint_mode_false():
    env = GalaxeaR1ProSingleArmReachEeEnv(_ee_dummy_cfg())
    assert env.config.use_joint_mode is False
    assert env.config.use_new_dispatcher is True


def test_ee_task_step_runs_in_dummy():
    env = GalaxeaR1ProSingleArmReachEeEnv(_ee_dummy_cfg())
    env.reset()
    a = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32)  # identity quat
    obs, reward, terminated, truncated, info = env.step(a)
    assert isinstance(reward, float)


def test_ee_task_reward_at_target_is_positive():
    target = np.array([0.4, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    env = GalaxeaR1ProSingleArmReachEeEnv(_ee_dummy_cfg(target_pose=target))
    env.reset()
    env._state.right_ee_pose = target.copy()
    _, reward, _, _, _ = env.step(
        np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32),
    )
    # Within tolerance -> bonus + dense ~0
    assert reward > 0.5
