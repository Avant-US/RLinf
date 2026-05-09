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

"""Unit tests for :class:`JointStateDispatcher`.

Per design doc r1pro6op47.md §3.2.7, joint mode must:

* Linearly map per-joint action ``[-1, 1]`` to ``[q_min, q_max]``.
* Drive the controller's ``send_arm_joints`` (NOT ``send_arm_pose``).
* Only request joint topics, not pose topics.
* Support a configurable gripper range via :class:`GripperMixer`.
* Fail fast (raise) when the topology check sees no subscriber.
"""

from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest

from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import (
    DispatchResult,
    JointStateDispatcher,
    build_action_dispatcher,
)
from rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer import GripperMixer
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (
    GalaxeaR1ProRobotState,
)

# ───────────────────────── Helpers ──────────────────────────────


def _q_min_default():
    return np.full(7, -1.0, dtype=np.float32)


def _q_max_default():
    return np.full(7, 1.0, dtype=np.float32)


def _make_dispatcher(
    *,
    no_gripper: bool = False,
    use_left_arm: bool = False,
    gmin_pct: float = 0.0,
    gmax_pct: float = 90.0,
    q_min: np.ndarray | None = None,
    q_max: np.ndarray | None = None,
):
    if q_min is None:
        q_min = _q_min_default()
    if q_max is None:
        q_max = _q_max_default()
    ctrl = MagicMock()
    # Default mock returns: send_* returns None, get_subscription_count = 1
    ctrl.send_arm_joints.return_value = None
    ctrl.send_arm_pose.return_value = None
    ctrl.send_gripper.return_value = None
    ctrl.get_subscription_count.return_value = 1
    mixer = GripperMixer(gmin_pct=gmin_pct, gmax_pct=gmax_pct)
    disp = JointStateDispatcher(
        controller=ctrl,
        use_right_arm=True,
        use_left_arm=use_left_arm,
        no_gripper=no_gripper,
        gripper_mixer=mixer,
        q_min_right=q_min,
        q_max_right=q_max,
        q_min_left=q_min if use_left_arm else None,
        q_max_left=q_max if use_left_arm else None,
    )
    return disp, ctrl


# ─────────────────────── Reverse-mapping ────────────────────────


def test_unnormalize_arm_zero_action_is_midpoint():
    disp, _ = _make_dispatcher(no_gripper=True)
    q = disp._unnormalize_arm("right", np.zeros(7))
    np.testing.assert_allclose(q, np.zeros(7), atol=1e-6)


def test_unnormalize_arm_pos_one_hits_qmax():
    disp, _ = _make_dispatcher(no_gripper=True)
    q = disp._unnormalize_arm("right", np.ones(7))
    np.testing.assert_allclose(q, np.ones(7), atol=1e-6)


def test_unnormalize_arm_neg_one_hits_qmin():
    disp, _ = _make_dispatcher(no_gripper=True)
    q = disp._unnormalize_arm("right", -np.ones(7))
    np.testing.assert_allclose(q, -np.ones(7), atol=1e-6)


def test_unnormalize_arm_clipped_when_out_of_range():
    disp, _ = _make_dispatcher(no_gripper=True)
    q = disp._unnormalize_arm("right", np.full(7, 1.5))
    np.testing.assert_allclose(q, np.ones(7), atol=1e-6)


def test_unnormalize_arm_per_joint_independent_bounds():
    """Use per-joint asymmetric bounds and check each axis independently."""
    q_min = np.array([-2.0, -1.0, 0.0, -0.5, -3.0, 0.0, -0.1], dtype=np.float32)
    q_max = np.array([+2.0, +1.0, 1.0, +0.5, +3.0, 0.5, +0.1], dtype=np.float32)
    disp, _ = _make_dispatcher(no_gripper=True, q_min=q_min, q_max=q_max)
    q = disp._unnormalize_arm("right", np.array([1, 1, 1, 1, 1, 1, 1]))
    np.testing.assert_allclose(q, q_max, atol=1e-6)
    q = disp._unnormalize_arm("right", np.array([-1, -1, -1, -1, -1, -1, -1]))
    np.testing.assert_allclose(q, q_min, atol=1e-6)
    q = disp._unnormalize_arm("right", np.array([0, 0, 0, 0, 0, 0, 0]))
    np.testing.assert_allclose(q, (q_min + q_max) * 0.5, atol=1e-6)


def test_unnormalize_gripper_uses_mixer_range_max90():
    disp, _ = _make_dispatcher(gmax_pct=90.0)
    assert disp._unnormalize_gripper(1.0) == pytest.approx(90.0)
    assert disp._unnormalize_gripper(-1.0) == pytest.approx(0.0)
    assert disp._unnormalize_gripper(0.0) == pytest.approx(45.0)


def test_unnormalize_gripper_custom_range_10_50():
    disp, _ = _make_dispatcher(gmin_pct=10.0, gmax_pct=50.0)
    assert disp._unnormalize_gripper(1.0) == pytest.approx(50.0)
    assert disp._unnormalize_gripper(-1.0) == pytest.approx(10.0)


# ──────────────────────── Dispatch path ─────────────────────────


def test_dispatch_calls_send_arm_joints_not_send_arm_pose():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    state = GalaxeaR1ProRobotState()
    # action layout: 7 joints (no gripper)
    safe_action = np.zeros(7, dtype=np.float32)
    result = disp.dispatch(safe_action, state)
    ctrl.send_arm_joints.assert_called_once()
    args, _ = ctrl.send_arm_joints.call_args
    assert args[0] == "right"
    np.testing.assert_allclose(args[1], np.zeros(7), atol=1e-6)
    ctrl.send_arm_pose.assert_not_called()
    assert result.mode == "joint"
    assert "right" in result.commands
    assert result.commands["right"]["q_target"] == [0.0] * 7


def test_dispatch_with_gripper_calls_send_gripper_with_pct90():
    disp, ctrl = _make_dispatcher(no_gripper=False, gmax_pct=90.0)
    state = GalaxeaR1ProRobotState()
    safe_action = np.zeros(8, dtype=np.float32)
    safe_action[7] = 1.0  # gripper to max business pct
    disp.dispatch(safe_action, state)
    ctrl.send_gripper.assert_called_once()
    args, _ = ctrl.send_gripper.call_args
    assert args[0] == "right"
    assert args[1] == pytest.approx(90.0)


def test_dispatch_with_gripper_neg_one_sends_pct0():
    disp, ctrl = _make_dispatcher(no_gripper=False, gmax_pct=90.0)
    state = GalaxeaR1ProRobotState()
    safe_action = np.zeros(8, dtype=np.float32)
    safe_action[7] = -1.0
    disp.dispatch(safe_action, state)
    args, _ = ctrl.send_gripper.call_args
    assert args[1] == pytest.approx(0.0)


def test_dispatch_dual_arm_calls_both_sides():
    disp, ctrl = _make_dispatcher(no_gripper=True, use_left_arm=True)
    state = GalaxeaR1ProRobotState()
    # 14-D: right 7 then left 7
    safe_action = np.zeros(14, dtype=np.float32)
    safe_action[7:] = 1.0  # left arm at qmax
    disp.dispatch(safe_action, state)
    assert ctrl.send_arm_joints.call_count == 2
    # First call right, second call left.
    first = ctrl.send_arm_joints.call_args_list[0][0]
    second = ctrl.send_arm_joints.call_args_list[1][0]
    assert first[0] == "right"
    assert second[0] == "left"


def test_dispatch_clips_action_above_one():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    state = GalaxeaR1ProRobotState()
    # action 1.5 should map to qmax (1.0), not 1.25
    safe_action = np.full(7, 1.5, dtype=np.float32)
    disp.dispatch(safe_action, state)
    args, _ = ctrl.send_arm_joints.call_args
    np.testing.assert_allclose(args[1], np.ones(7), atol=1e-6)


def test_dispatch_returns_DispatchResult():
    disp, _ = _make_dispatcher(no_gripper=True)
    state = GalaxeaR1ProRobotState()
    result = disp.dispatch(np.zeros(7, dtype=np.float32), state)
    assert isinstance(result, DispatchResult)
    assert result.mode == "joint"


# ───────────────────────── reset_to_safe ────────────────────────


def test_reset_to_safe_pose_publishes_home_q():
    home_q = np.array([0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0], dtype=np.float32)
    ctrl = MagicMock()
    ctrl.send_arm_joints.return_value = None
    ctrl.send_gripper.return_value = None
    mixer = GripperMixer(0.0, 90.0)
    disp = JointStateDispatcher(
        controller=ctrl,
        use_right_arm=True,
        use_left_arm=False,
        no_gripper=True,
        gripper_mixer=mixer,
        q_min_right=_q_min_default(),
        q_max_right=_q_max_default(),
        home_q_right=home_q,
    )
    disp.reset_to_safe_pose(GalaxeaR1ProRobotState())
    args, _ = ctrl.send_arm_joints.call_args
    assert args[0] == "right"
    np.testing.assert_allclose(args[1], home_q.tolist(), atol=1e-6)


def test_reset_to_safe_pose_default_home_is_zeros():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    disp.reset_to_safe_pose(GalaxeaR1ProRobotState())
    args, _ = ctrl.send_arm_joints.call_args
    np.testing.assert_allclose(args[1], np.zeros(7), atol=1e-6)


def test_reset_to_safe_pose_with_gripper_sends_gmin():
    disp, ctrl = _make_dispatcher(no_gripper=False, gmin_pct=10.0, gmax_pct=80.0)
    disp.reset_to_safe_pose(GalaxeaR1ProRobotState())
    ctrl.send_gripper.assert_called_once()
    args, _ = ctrl.send_gripper.call_args
    assert args[1] == pytest.approx(10.0)  # gmin_pct


# ───────────────── get_required_topics + verify_topology ────────


def test_get_required_topics_joint_with_gripper():
    disp, _ = _make_dispatcher(no_gripper=False)
    topics = disp.get_required_topics()
    assert "/motion_target/target_joint_state_arm_right" in topics
    assert "/motion_target/target_position_gripper_right" in topics
    assert "/motion_target/target_pose_arm_right" not in topics


def test_get_required_topics_no_gripper_excludes_gripper_topic():
    disp, _ = _make_dispatcher(no_gripper=True)
    topics = disp.get_required_topics()
    assert "/motion_target/target_position_gripper_right" not in topics


def test_get_required_topics_dual_arm():
    disp, _ = _make_dispatcher(no_gripper=True, use_left_arm=True)
    topics = disp.get_required_topics()
    assert "/motion_target/target_joint_state_arm_right" in topics
    assert "/motion_target/target_joint_state_arm_left" in topics


def test_verify_topology_raises_when_subscriber_count_zero():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    ctrl.get_subscription_count.return_value = 0
    with pytest.raises(RuntimeError, match="no subscriber"):
        disp.verify_topology(ctrl)


def test_verify_topology_passes_when_subscribers_present():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    ctrl.get_subscription_count.return_value = 1
    disp.verify_topology(ctrl)  # no raise


# ───────────────────── Constructor validation ───────────────────


def test_constructor_rejects_wrong_q_min_length():
    mixer = GripperMixer(0.0, 90.0)
    with pytest.raises(ValueError, match="length 7"):
        JointStateDispatcher(
            controller=MagicMock(),
            use_right_arm=True,
            use_left_arm=False,
            no_gripper=True,
            gripper_mixer=mixer,
            q_min_right=np.zeros(6),
            q_max_right=np.ones(7),
        )


def test_constructor_rejects_qmax_le_qmin():
    mixer = GripperMixer(0.0, 90.0)
    with pytest.raises(ValueError, match="q_max must be > q_min"):
        JointStateDispatcher(
            controller=MagicMock(),
            use_right_arm=True,
            use_left_arm=False,
            no_gripper=True,
            gripper_mixer=mixer,
            q_min_right=np.ones(7),
            q_max_right=np.ones(7),  # equal -> reject
        )


def test_constructor_left_arm_requires_left_bounds():
    mixer = GripperMixer(0.0, 90.0)
    with pytest.raises(ValueError, match="q_min_left"):
        JointStateDispatcher(
            controller=MagicMock(),
            use_right_arm=True,
            use_left_arm=True,  # but no q_min_left / q_max_left
            no_gripper=True,
            gripper_mixer=mixer,
            q_min_right=_q_min_default(),
            q_max_right=_q_max_default(),
        )


# ─────────────────────────── Factory ────────────────────────────


def test_factory_use_joint_mode_true_returns_joint_dispatcher():
    mixer = GripperMixer(0.0, 90.0)
    disp = build_action_dispatcher(
        use_joint_mode=True,
        use_right_arm=True,
        use_left_arm=False,
        no_gripper=True,
        gripper_mixer=mixer,
        controller=MagicMock(),
        q_min_right=_q_min_default(),
        q_max_right=_q_max_default(),
    )
    assert disp.mode == "joint"
    assert isinstance(disp, JointStateDispatcher)


def test_factory_joint_mode_missing_q_bounds_raises():
    mixer = GripperMixer(0.0, 90.0)
    with pytest.raises(ValueError, match="q_min_right"):
        build_action_dispatcher(
            use_joint_mode=True,
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


def test_action_dim_dual_arm_with_gripper_is_16():
    disp, _ = _make_dispatcher(no_gripper=False, use_left_arm=True)
    assert disp.action_dim == 16


def test_action_dim_dual_arm_no_gripper_is_14():
    disp, _ = _make_dispatcher(no_gripper=True, use_left_arm=True)
    assert disp.action_dim == 14
