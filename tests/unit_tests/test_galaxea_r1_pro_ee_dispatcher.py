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

"""Unit tests for :class:`EePoseDispatcher`.

Per design doc r1pro6op47.md §3.3, ee mode must:

* Linearly map xyz action ``[-1, 1]`` to absolute Cartesian inside
  the configured workspace box ``[ee_min, ee_max]``.
* L2-normalise the quaternion and force ``W >= 0`` canonical form.
* Drive the controller's ``send_arm_pose`` (NOT ``send_arm_joints``).
* Only request pose topics, not joint topics.
* Round-trip degenerate zero-quaternion to identity ``[0, 0, 0, 1]``.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest

from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import (
    DispatchResult,
    EePoseDispatcher,
    build_action_dispatcher,
)
from rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer import GripperMixer
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (
    GalaxeaR1ProRobotState,
)

# ───────────────────────── Helpers ──────────────────────────────


def _ee_min_default():
    return np.array([0.20, -0.40, 0.10], dtype=np.float32)


def _ee_max_default():
    return np.array([0.70, 0.40, 0.80], dtype=np.float32)


def _make_dispatcher(
    *,
    no_gripper: bool = False,
    use_left_arm: bool = False,
    gmin_pct: float = 0.0,
    gmax_pct: float = 90.0,
):
    ctrl = MagicMock()
    ctrl.send_arm_joints.return_value = None
    ctrl.send_arm_pose.return_value = None
    ctrl.send_gripper.return_value = None
    ctrl.get_subscription_count.return_value = 1
    mixer = GripperMixer(gmin_pct=gmin_pct, gmax_pct=gmax_pct)
    disp = EePoseDispatcher(
        controller=ctrl,
        use_right_arm=True,
        use_left_arm=use_left_arm,
        no_gripper=no_gripper,
        gripper_mixer=mixer,
        ee_min_right=_ee_min_default(),
        ee_max_right=_ee_max_default(),
        ee_min_left=_ee_min_default() if use_left_arm else None,
        ee_max_left=_ee_max_default() if use_left_arm else None,
    )
    return disp, ctrl


# ──────────────────────── Quaternion ────────────────────────────


def test_normalize_quat_zero_returns_identity():
    q = EePoseDispatcher._normalize_quat(np.zeros(4))
    np.testing.assert_allclose(q, [0.0, 0.0, 0.0, 1.0], atol=1e-6)


def test_normalize_quat_already_unit_w_pos_unchanged():
    inp = np.array([0.0, 0.0, 0.0, 1.0])
    q = EePoseDispatcher._normalize_quat(inp)
    np.testing.assert_allclose(q, inp, atol=1e-6)


def test_normalize_quat_negative_w_flipped():
    """W < 0 must be canonicalised to W >= 0 (q and -q same rotation)."""
    inp = np.array([0.0, 0.0, 0.0, -1.0])
    q = EePoseDispatcher._normalize_quat(inp)
    np.testing.assert_allclose(q, [0.0, 0.0, 0.0, 1.0], atol=1e-6)


def test_normalize_quat_arbitrary_input_l2_normalised():
    inp = np.array([1.0, 0.0, 0.0, 1.0])
    q = EePoseDispatcher._normalize_quat(inp)
    expected = np.array([0.7071, 0.0, 0.0, 0.7071])
    np.testing.assert_allclose(q, expected, atol=1e-3)
    # And unit length.
    assert float(np.linalg.norm(q)) == pytest.approx(1.0, abs=1e-6)


def test_normalize_quat_negative_w_flipped_arbitrary():
    inp = np.array([0.5, 0.0, 0.0, -0.5])
    q = EePoseDispatcher._normalize_quat(inp)
    # After L2 norm: 0.707, 0, 0, -0.707; after W>=0 flip: -0.707, 0, 0, 0.707
    np.testing.assert_allclose(q, [-0.7071, 0.0, 0.0, 0.7071], atol=1e-3)


# ────────────────────────── XYZ map ─────────────────────────────


def test_unnormalize_xyz_zero_action_is_box_centre():
    disp, _ = _make_dispatcher(no_gripper=True)
    xyz = disp._unnormalize_xyz("right", np.zeros(3))
    expected = (_ee_min_default() + _ee_max_default()) * 0.5
    np.testing.assert_allclose(xyz, expected, atol=1e-6)


def test_unnormalize_xyz_pos_one_hits_max():
    disp, _ = _make_dispatcher(no_gripper=True)
    xyz = disp._unnormalize_xyz("right", np.ones(3))
    np.testing.assert_allclose(xyz, _ee_max_default(), atol=1e-6)


def test_unnormalize_xyz_neg_one_hits_min():
    disp, _ = _make_dispatcher(no_gripper=True)
    xyz = disp._unnormalize_xyz("right", -np.ones(3))
    np.testing.assert_allclose(xyz, _ee_min_default(), atol=1e-6)


def test_unnormalize_xyz_clipped_when_out_of_range():
    disp, _ = _make_dispatcher(no_gripper=True)
    xyz = disp._unnormalize_xyz("right", np.full(3, 2.0))
    np.testing.assert_allclose(xyz, _ee_max_default(), atol=1e-6)


# ──────────────────────── Dispatch path ─────────────────────────


def test_dispatch_calls_send_arm_pose_not_send_arm_joints():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    state = GalaxeaR1ProRobotState()
    # 7-D: xyz + quat
    safe_action = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32)
    result = disp.dispatch(safe_action, state)
    ctrl.send_arm_pose.assert_called_once()
    args, _ = ctrl.send_arm_pose.call_args
    assert args[0] == "right"
    assert args[1].shape == (7,)
    ctrl.send_arm_joints.assert_not_called()
    assert result.mode == "ee"


def test_dispatch_with_gripper_calls_send_gripper_pct90():
    disp, ctrl = _make_dispatcher(no_gripper=False, gmax_pct=90.0)
    state = GalaxeaR1ProRobotState()
    safe_action = np.array([0, 0, 0, 0, 0, 0, 1, 1], dtype=np.float32)
    disp.dispatch(safe_action, state)
    args, _ = ctrl.send_gripper.call_args
    assert args[1] == pytest.approx(90.0)


def test_dispatch_pose_xyz_at_box_centre_for_zero_action():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    state = GalaxeaR1ProRobotState()
    # action: xyz = 0, quat = identity
    safe_action = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32)
    disp.dispatch(safe_action, state)
    args, _ = ctrl.send_arm_pose.call_args
    pose = args[1]
    expected_xyz = (_ee_min_default() + _ee_max_default()) * 0.5
    np.testing.assert_allclose(pose[:3], expected_xyz, atol=1e-6)
    np.testing.assert_allclose(pose[3:], [0, 0, 0, 1], atol=1e-6)


def test_dispatch_normalises_non_unit_quat_in_action():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    state = GalaxeaR1ProRobotState()
    # quat (1, 0, 0, 1) -> normalised (0.707, 0, 0, 0.707)
    safe_action = np.array([0, 0, 0, 1, 0, 0, 1], dtype=np.float32)
    disp.dispatch(safe_action, state)
    args, _ = ctrl.send_arm_pose.call_args
    pose = args[1]
    np.testing.assert_allclose(
        pose[3:],
        [0.7071, 0.0, 0.0, 0.7071],
        atol=1e-3,
    )


def test_dispatch_dual_arm_calls_send_arm_pose_twice():
    disp, ctrl = _make_dispatcher(no_gripper=True, use_left_arm=True)
    state = GalaxeaR1ProRobotState()
    # 14-D: right (xyz+quat) then left (xyz+quat)
    safe_action = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1], dtype=np.float32)
    disp.dispatch(safe_action, state)
    assert ctrl.send_arm_pose.call_count == 2
    assert ctrl.send_arm_pose.call_args_list[0][0][0] == "right"
    assert ctrl.send_arm_pose.call_args_list[1][0][0] == "left"


def test_dispatch_returns_DispatchResult():
    disp, _ = _make_dispatcher(no_gripper=True)
    state = GalaxeaR1ProRobotState()
    result = disp.dispatch(np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32), state)
    assert isinstance(result, DispatchResult)
    assert result.mode == "ee"


# ───────────────────────── reset_to_safe ────────────────────────


def test_reset_to_safe_pose_default_uses_box_centre_identity_quat():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    disp.reset_to_safe_pose(GalaxeaR1ProRobotState())
    args, _ = ctrl.send_arm_pose.call_args
    pose = args[1]
    expected_xyz = (_ee_min_default() + _ee_max_default()) * 0.5
    np.testing.assert_allclose(pose[:3], expected_xyz, atol=1e-6)
    np.testing.assert_allclose(pose[3:], [0, 0, 0, 1], atol=1e-6)


def test_reset_to_safe_pose_custom_home_pose():
    custom_home = np.array(
        [0.4, 0.1, 0.5, 0.0, 0.0, 0.0, 1.0],
        dtype=np.float32,
    )
    ctrl = MagicMock()
    ctrl.send_arm_pose.return_value = None
    ctrl.send_gripper.return_value = None
    mixer = GripperMixer(0.0, 90.0)
    disp = EePoseDispatcher(
        controller=ctrl,
        use_right_arm=True,
        use_left_arm=False,
        no_gripper=True,
        gripper_mixer=mixer,
        ee_min_right=_ee_min_default(),
        ee_max_right=_ee_max_default(),
        home_pose_right=custom_home,
    )
    disp.reset_to_safe_pose(GalaxeaR1ProRobotState())
    args, _ = ctrl.send_arm_pose.call_args
    np.testing.assert_allclose(args[1], custom_home, atol=1e-6)


# ───────────────── get_required_topics + verify_topology ────────


def test_get_required_topics_ee_with_gripper():
    disp, _ = _make_dispatcher(no_gripper=False)
    topics = disp.get_required_topics()
    assert "/motion_target/target_pose_arm_right" in topics
    assert "/motion_target/target_position_gripper_right" in topics
    assert "/motion_target/target_joint_state_arm_right" not in topics


def test_get_required_topics_ee_dual_arm():
    disp, _ = _make_dispatcher(no_gripper=True, use_left_arm=True)
    topics = disp.get_required_topics()
    assert "/motion_target/target_pose_arm_right" in topics
    assert "/motion_target/target_pose_arm_left" in topics


def test_verify_topology_raises_when_no_subscriber():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    ctrl.get_subscription_count.return_value = 0
    with pytest.raises(RuntimeError, match="no subscriber"):
        disp.verify_topology(ctrl)


# ───────────────────── Constructor validation ───────────────────


def test_constructor_rejects_wrong_xyz_length():
    mixer = GripperMixer(0.0, 90.0)
    with pytest.raises(ValueError, match="length 3"):
        EePoseDispatcher(
            controller=MagicMock(),
            use_right_arm=True,
            use_left_arm=False,
            no_gripper=True,
            gripper_mixer=mixer,
            ee_min_right=np.zeros(2),
            ee_max_right=np.ones(3),
        )


def test_constructor_rejects_inverted_box():
    mixer = GripperMixer(0.0, 90.0)
    with pytest.raises(ValueError, match="ee_max must be > ee_min"):
        EePoseDispatcher(
            controller=MagicMock(),
            use_right_arm=True,
            use_left_arm=False,
            no_gripper=True,
            gripper_mixer=mixer,
            ee_min_right=np.array([0.5, 0.5, 0.5]),
            ee_max_right=np.array([0.5, 0.5, 0.5]),
        )


def test_constructor_left_arm_requires_left_box():
    mixer = GripperMixer(0.0, 90.0)
    with pytest.raises(ValueError, match="ee_min_left"):
        EePoseDispatcher(
            controller=MagicMock(),
            use_right_arm=True,
            use_left_arm=True,
            no_gripper=True,
            gripper_mixer=mixer,
            ee_min_right=_ee_min_default(),
            ee_max_right=_ee_max_default(),
        )


# ─────────────────────────── Factory ────────────────────────────


def test_factory_use_joint_mode_false_returns_ee_dispatcher():
    mixer = GripperMixer(0.0, 90.0)
    disp = build_action_dispatcher(
        use_joint_mode=False,
        use_right_arm=True,
        use_left_arm=False,
        no_gripper=True,
        gripper_mixer=mixer,
        controller=MagicMock(),
        ee_min_right=_ee_min_default(),
        ee_max_right=_ee_max_default(),
    )
    assert disp.mode == "ee"
    assert isinstance(disp, EePoseDispatcher)


def test_factory_ee_mode_missing_box_raises():
    mixer = GripperMixer(0.0, 90.0)
    with pytest.raises(ValueError, match="ee_min_right"):
        build_action_dispatcher(
            use_joint_mode=False,
            use_right_arm=True,
            use_left_arm=False,
            no_gripper=True,
            gripper_mixer=mixer,
            controller=MagicMock(),
        )


# ─────────────────────────── action_dim ─────────────────────────


def test_action_dim_single_arm_with_gripper_is_8():
    disp, _ = _make_dispatcher(no_gripper=False)
    assert disp.per_arm_dim == 8
    assert disp.action_dim == 8


def test_action_dim_single_arm_no_gripper_is_7():
    disp, _ = _make_dispatcher(no_gripper=True)
    assert disp.per_arm_dim == 7
    assert disp.action_dim == 7
