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

"""Unit tests for :class:`ActionSchema` and :func:`build_action_schema`."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from rlinf.envs.realworld.galaxear.r1_pro_action_schema import (
    ActionSchema,
    build_action_schema,
)
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (
    GalaxeaR1ProRobotState,
)


def test_action_dim_per_stage():
    s_m1 = ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    assert s_m1.action_dim == 7

    s_m2 = ActionSchema(
        has_left_arm=True, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    assert s_m2.action_dim == 14

    s_m3 = ActionSchema(
        has_left_arm=True, has_right_arm=True,
        has_torso=True, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    assert s_m3.action_dim == 18

    s_m4 = ActionSchema(
        has_left_arm=True, has_right_arm=True,
        has_torso=True, has_chassis=True,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    assert s_m4.action_dim == 21

    s_no_grip = ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=True,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    assert s_no_grip.action_dim == 6


def test_split_single_arm():
    s = ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    a = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, -0.7], dtype=np.float32)
    d = s.split(a)
    assert "right_xyz" in d and "right_rpy" in d and "right_gripper" in d
    np.testing.assert_allclose(d["right_xyz"], [0.1, 0.2, 0.3], rtol=1e-6)
    np.testing.assert_allclose(d["right_rpy"], [0.4, 0.5, 0.6], rtol=1e-6)
    assert d["right_gripper"] == pytest.approx(-0.7)


def test_split_full_action():
    s = ActionSchema(
        has_left_arm=True, has_right_arm=True,
        has_torso=True, has_chassis=True,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    a = np.arange(s.action_dim, dtype=np.float32) * 0.01
    d = s.split(a)
    for key in (
        "right_xyz", "right_rpy", "right_gripper",
        "left_xyz", "left_rpy", "left_gripper",
        "torso_twist", "chassis_twist",
    ):
        assert key in d


def test_predict_arm_ee_pose_returns_none_when_arm_off():
    s = ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    state = GalaxeaR1ProRobotState()
    assert s.predict_arm_ee_pose("left", np.zeros(7), state) is None


def test_predict_arm_ee_pose_applies_scale():
    s = ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    state = GalaxeaR1ProRobotState()
    state.right_ee_pose = np.array(
        [0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0], dtype=np.float32,
    )
    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    out = s.predict_arm_ee_pose("right", a, state)
    assert out is not None
    np.testing.assert_allclose(out[0], 0.4 + 0.05, atol=1e-6)


def test_build_action_schema_from_cfg():
    cfg = SimpleNamespace(
        use_left_arm=True,
        use_right_arm=True,
        use_torso=False,
        use_chassis=False,
        no_gripper=False,
        action_scale=[0.05, 0.10, 1.0],
        mobiman_launch_mode="pose",
    )
    s = build_action_schema(cfg)
    assert s.has_dual_arms is True
    assert s.use_joint_mode is False
    assert s.action_dim == 14
