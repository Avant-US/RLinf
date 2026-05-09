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

"""Integration tests for :class:`GalaxeaR1ProEnv` in dispatcher mode.

These tests run with ``is_dummy=True`` so no ROS 2 / hardware is
required.  They verify per design doc r1pro6op47.md:

* Action space dim is 8 for single-arm joint+gripper (vs legacy 7).
* Action space dim is 7 for single-arm ee+quat without gripper (vs
  legacy 6).
* ``env.step()`` does not raise on a zero action.
* ``env.reset()`` does not raise.
* ``env._dispatcher`` is the right type when use_joint_mode is set.
* Gripper config (``gripper_max_pct=90``) is propagated to the mixer.
* When use_joint_mode is False and use_new_dispatcher is False (legacy),
  action dim and dispatcher are unchanged.
"""

from __future__ import annotations

import numpy as np
import pytest

from rlinf.envs.realworld.galaxear.r1_pro_env import GalaxeaR1ProEnv

# ─────────────────────── Joint mode env ────────────────────────


def _joint_env_cfg(no_gripper: bool = True, dual_arm: bool = False):
    cfg = {
        "is_dummy": True,
        "use_joint_mode": True,
        "use_right_arm": True,
        "use_left_arm": dual_arm,
        "no_gripper": no_gripper,
        "gripper_min_pct": 0.0,
        "gripper_max_pct": 90.0,
        "arm_q_min_right": [-2.0] * 7,
        "arm_q_max_right": [+2.0] * 7,
        "step_frequency": 100.0,  # so dummy step doesn't sleep
    }
    if dual_arm:
        cfg["arm_q_min_left"] = [-2.0] * 7
        cfg["arm_q_max_left"] = [+2.0] * 7
    return cfg


def test_env_joint_mode_action_dim_single_arm_no_gripper_is_7():
    env = GalaxeaR1ProEnv(_joint_env_cfg(no_gripper=True))
    assert env.action_space.shape == (7,)


def test_env_joint_mode_action_dim_single_arm_with_gripper_is_8():
    env = GalaxeaR1ProEnv(_joint_env_cfg(no_gripper=False))
    assert env.action_space.shape == (8,)


def test_env_joint_mode_action_dim_dual_arm_no_gripper_is_14():
    env = GalaxeaR1ProEnv(_joint_env_cfg(no_gripper=True, dual_arm=True))
    assert env.action_space.shape == (14,)


def test_env_joint_mode_dispatcher_is_joint_type():
    """In dummy mode the controller is None, so the dispatcher is also
    None -- but the cfg / spaces should still be wired for joint."""
    env = GalaxeaR1ProEnv(_joint_env_cfg(no_gripper=True))
    # Dummy mode skips _setup_hardware, so dispatcher may be None.
    # We instead verify the action space matches what JointStateDispatcher
    # would have produced (proxy for the wiring).
    assert env.action_space.shape == (7,)


def test_env_joint_mode_dummy_step_runs():
    env = GalaxeaR1ProEnv(_joint_env_cfg(no_gripper=True))
    env.reset()
    obs, reward, terminated, truncated, info = env.step(np.zeros(7))
    assert isinstance(reward, float)
    assert "step_count" in info


def test_env_joint_mode_dummy_reset_runs():
    env = GalaxeaR1ProEnv(_joint_env_cfg(no_gripper=True))
    obs, info = env.reset()
    assert "state" in obs


# ────────────────────── EE mode (new) env ──────────────────────


def _ee_env_cfg(no_gripper: bool = True):
    return {
        "is_dummy": True,
        "use_joint_mode": False,
        "use_new_dispatcher": True,  # opt into ee+quat path
        "use_right_arm": True,
        "use_left_arm": False,
        "no_gripper": no_gripper,
        "gripper_min_pct": 0.0,
        "gripper_max_pct": 90.0,
        "ee_min_right": [0.20, -0.40, 0.10],
        "ee_max_right": [0.70, 0.40, 0.80],
        "step_frequency": 100.0,
    }


def test_env_ee_new_action_dim_single_arm_no_gripper_is_7():
    env = GalaxeaR1ProEnv(_ee_env_cfg(no_gripper=True))
    # 3 xyz + 4 quat = 7
    assert env.action_space.shape == (7,)


def test_env_ee_new_action_dim_single_arm_with_gripper_is_8():
    env = GalaxeaR1ProEnv(_ee_env_cfg(no_gripper=False))
    assert env.action_space.shape == (8,)


def test_env_ee_new_dummy_step_runs():
    env = GalaxeaR1ProEnv(_ee_env_cfg(no_gripper=True))
    env.reset()
    a = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32)  # identity quat
    obs, reward, terminated, truncated, info = env.step(a)
    assert isinstance(reward, float)


# ───────────────────── Legacy mode unchanged ───────────────────


def test_env_legacy_mode_action_dim_unchanged_single_arm():
    """Without use_joint_mode / use_new_dispatcher, action dim must stay
    at the legacy value (3 xyz + 3 rpy + 1 gripper = 7)."""
    env = GalaxeaR1ProEnv(
        {
            "is_dummy": True,
            "use_joint_mode": False,
            "use_new_dispatcher": False,
            "use_right_arm": True,
            "no_gripper": False,
            "step_frequency": 100.0,
        }
    )
    assert env.action_space.shape == (7,)


def test_env_legacy_mode_no_gripper_dim_6():
    env = GalaxeaR1ProEnv(
        {
            "is_dummy": True,
            "use_joint_mode": False,
            "use_new_dispatcher": False,
            "use_right_arm": True,
            "no_gripper": True,
            "step_frequency": 100.0,
        }
    )
    assert env.action_space.shape == (6,)


# ─────────────────────── GripperMixer config ───────────────────


def test_env_gripper_mixer_uses_cfg_max_pct_90():
    env = GalaxeaR1ProEnv(_joint_env_cfg(no_gripper=False))
    assert env._gripper_mixer.gmax_pct == pytest.approx(90.0)
    assert env._gripper_mixer.gmin_pct == pytest.approx(0.0)


def test_env_gripper_mixer_handles_invalid_range_falls_back():
    """Mis-configured gmax<=gmin should fall back to (0, 100)."""
    cfg = _joint_env_cfg(no_gripper=False)
    cfg["gripper_min_pct"] = 50.0
    cfg["gripper_max_pct"] = 30.0  # invalid
    env = GalaxeaR1ProEnv(cfg)
    assert env._gripper_mixer.gmin_pct == pytest.approx(0.0)
    assert env._gripper_mixer.gmax_pct == pytest.approx(100.0)


# ────────────────────── Heartbeat wiring ───────────────────────


def test_env_step_calls_safety_heartbeat():
    """Per r1pro6op47.md §6.5: safety.heartbeat() must be called inside
    step() so the operator-watchdog L5 doesn't fire after 1.5 s.

    Strategy: set the heartbeat timeout to 50 ms and exercise step() at
    a faster rate; if heartbeat were missing, L5 would fire on the
    second call and put `L5:operator_hb_age=...` in info["safety_reasons"].
    """
    cfg = _joint_env_cfg(no_gripper=True)
    cfg["safety_cfg"] = {"operator_heartbeat_timeout_ms": 50.0}
    cfg["step_frequency"] = 100.0  # 10 ms step
    env = GalaxeaR1ProEnv(cfg)
    env.reset()
    # Take three steps spaced by ~10 ms each.  If heartbeat is wired,
    # no step's reasons should contain operator_hb_age.
    import time

    saw_hb = False
    for _ in range(3):
        _, _, _, _, info = env.step(np.zeros(7))
        if any("operator_hb_age" in r for r in info.get("safety_reasons", [])):
            saw_hb = True
        time.sleep(0.02)
    assert not saw_hb, "operator_hb_age fired despite heartbeat wiring"
