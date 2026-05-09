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

"""Unit tests for :class:`JointDeltaDispatcher`.

Per design doc r1pro6op47.md §3.6.7 (test matrix), joint_delta_mode must:

* Map ``a=0`` to identity (q_target == q_current).
* Map ``a=+1`` to ``clip(q_current + delta_scale, q_min, q_max)``.
* Map ``a=-1`` to ``clip(q_current - delta_scale, q_min, q_max)``.
* Clip out-of-range action [-1, 1] before applying.
* Clip the resulting q_target to [q_min, q_max] (so q_current already at
  the limit doesn't escape it).
* Honor per-joint ``delta_scale`` vector.
* Use **absolute** ``home_q`` for ``reset_to_safe_pose`` (NOT delta).
* Keep the gripper dim **absolute** (asymmetric per §3.6.1) -- behaviour
  identical to abs JointStateDispatcher.
* Be selectable via ``build_action_dispatcher(joint_delta_mode=True)``.
* NOT be selected when ``joint_delta_mode=False`` (default).
"""

from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest

from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import (
    DispatchResult,
    JointDeltaDispatcher,
    JointStateDispatcher,
    build_action_dispatcher,
)
from rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer import GripperMixer
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (
    GalaxeaR1ProRobotState,
)

# ───────────────────────── Helpers ──────────────────────────────


def _q_min_default():
    return np.full(7, -2.0, dtype=np.float32)


def _q_max_default():
    return np.full(7, +2.0, dtype=np.float32)


def _make_dispatcher(
    *,
    no_gripper: bool = False,
    use_left_arm: bool = False,
    gmin_pct: float = 0.0,
    gmax_pct: float = 90.0,
    q_min: np.ndarray | None = None,
    q_max: np.ndarray | None = None,
    delta_scale: np.ndarray | None = None,
    delta_scale_left: np.ndarray | None = None,
    home_q_right: np.ndarray | None = None,
    home_q_left: np.ndarray | None = None,
):
    if q_min is None:
        q_min = _q_min_default()
    if q_max is None:
        q_max = _q_max_default()
    ctrl = MagicMock()
    ctrl.send_arm_joints.return_value = None
    ctrl.send_arm_pose.return_value = None
    ctrl.send_gripper.return_value = None
    ctrl.get_subscription_count.return_value = 1
    mixer = GripperMixer(gmin_pct=gmin_pct, gmax_pct=gmax_pct)
    disp = JointDeltaDispatcher(
        controller=ctrl,
        use_right_arm=True,
        use_left_arm=use_left_arm,
        no_gripper=no_gripper,
        gripper_mixer=mixer,
        q_min_right=q_min,
        q_max_right=q_max,
        q_min_left=q_min if use_left_arm else None,
        q_max_left=q_max if use_left_arm else None,
        joint_delta_scale_right=delta_scale,
        joint_delta_scale_left=delta_scale_left,
        home_q_right=home_q_right,
        home_q_left=home_q_left,
    )
    return disp, ctrl


def _state_with_qpos(right_qpos=None, left_qpos=None):
    st = GalaxeaR1ProRobotState()
    if right_qpos is not None:
        st.right_arm_qpos = np.asarray(right_qpos, dtype=np.float32)
    if left_qpos is not None:
        st.left_arm_qpos = np.asarray(left_qpos, dtype=np.float32)
    return st


# ──────────────────────── Reverse mapping ───────────────────────


def test_delta_zero_action_is_identity():
    """a=0 -> q_target=q_current (no movement)."""
    disp, _ = _make_dispatcher(no_gripper=True)
    cur_q = np.full(7, 0.5, dtype=np.float32)
    q_target = disp._compute_q_target("right", cur_q, np.zeros(7))
    np.testing.assert_allclose(q_target, cur_q, atol=1e-6)


def test_delta_pos_one_advances_by_scale():
    """a=+1 -> q_target = q_current + delta_scale."""
    scale = np.array([0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20], dtype=np.float32)
    disp, _ = _make_dispatcher(no_gripper=True, delta_scale=scale)
    cur_q = np.zeros(7, dtype=np.float32)
    q_target = disp._compute_q_target("right", cur_q, np.ones(7))
    np.testing.assert_allclose(q_target, scale, atol=1e-6)


def test_delta_neg_one_retreats_by_scale():
    """a=-1 -> q_target = q_current - delta_scale."""
    scale = np.array([0.10] * 7, dtype=np.float32)
    disp, _ = _make_dispatcher(no_gripper=True, delta_scale=scale)
    cur_q = np.zeros(7, dtype=np.float32)
    q_target = disp._compute_q_target("right", cur_q, -np.ones(7))
    np.testing.assert_allclose(q_target, -scale, atol=1e-6)


def test_delta_action_above_one_is_clipped_first():
    """a=+1.5 -> first clipped to +1, then q_target = q_current + scale."""
    scale = np.array([0.10] * 7, dtype=np.float32)
    disp, _ = _make_dispatcher(no_gripper=True, delta_scale=scale)
    cur_q = np.zeros(7, dtype=np.float32)
    q_target = disp._compute_q_target("right", cur_q, np.full(7, 1.5))
    np.testing.assert_allclose(q_target, scale, atol=1e-6)


def test_delta_q_target_clipped_to_qmax():
    """q_current already at q_max, a=+1 -> q_target stays at q_max."""
    disp, _ = _make_dispatcher(no_gripper=True)
    cur_q = _q_max_default()  # already at upper bound
    q_target = disp._compute_q_target("right", cur_q, np.ones(7))
    np.testing.assert_allclose(q_target, _q_max_default(), atol=1e-6)


def test_delta_q_target_clipped_to_qmin():
    """q_current already at q_min, a=-1 -> q_target stays at q_min."""
    disp, _ = _make_dispatcher(no_gripper=True)
    cur_q = _q_min_default()
    q_target = disp._compute_q_target("right", cur_q, -np.ones(7))
    np.testing.assert_allclose(q_target, _q_min_default(), atol=1e-6)


def test_delta_per_joint_independent_scale():
    """Per-joint scale vector applied element-wise."""
    scale = np.array([0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70], dtype=np.float32)
    disp, _ = _make_dispatcher(no_gripper=True, delta_scale=scale)
    cur_q = np.zeros(7, dtype=np.float32)
    q_target = disp._compute_q_target("right", cur_q, np.ones(7))
    np.testing.assert_allclose(q_target, scale, atol=1e-6)


def test_delta_per_joint_negative_scale_each_axis():
    """Per-joint scale with mixed sign action."""
    scale = np.array([0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70], dtype=np.float32)
    disp, _ = _make_dispatcher(no_gripper=True, delta_scale=scale)
    cur_q = np.zeros(7, dtype=np.float32)
    a = np.array([+1, -1, +1, -1, +1, -1, +1], dtype=np.float32)
    q_target = disp._compute_q_target("right", cur_q, a)
    expected = np.array([+0.10, -0.20, +0.30, -0.40, +0.50, -0.60, +0.70])
    np.testing.assert_allclose(q_target, expected, atol=1e-6)


# ──────────────────────── Dispatch path ─────────────────────────


def test_dispatch_calls_send_arm_joints_with_q_target_not_delta():
    """dispatch must send absolute q_target (post-add), not the raw delta."""
    scale = np.full(7, 0.10, dtype=np.float32)
    disp, ctrl = _make_dispatcher(no_gripper=True, delta_scale=scale)
    state = _state_with_qpos(right_qpos=np.full(7, 0.5))
    safe_action = np.ones(7, dtype=np.float32)  # +1 on all joints
    res = disp.dispatch(safe_action, state)
    ctrl.send_arm_joints.assert_called_once()
    args, _ = ctrl.send_arm_joints.call_args
    assert args[0] == "right"
    expected = np.full(7, 0.6)  # 0.5 + 0.10
    np.testing.assert_allclose(args[1], expected, atol=1e-6)
    # qvel_max passed through (default vector)
    assert len(args[2]) == 7
    # mode in result
    assert res.mode == "joint_delta"
    # commands include both q_current and delta_rad (debug aid)
    assert "right" in res.commands
    assert "q_current" in res.commands["right"]
    assert "delta_rad" in res.commands["right"]
    np.testing.assert_allclose(
        res.commands["right"]["delta_rad"], [0.10] * 7, atol=1e-6
    )


def test_dispatch_reads_state_each_call_no_caching():
    """Two consecutive dispatches should each see the state mutated by
    backend's command-mirroring (DummyBackend semantics) -- proving the
    dispatcher reads state.qpos every call instead of caching."""
    scale = np.full(7, 0.10, dtype=np.float32)
    disp, ctrl = _make_dispatcher(no_gripper=True, delta_scale=scale)
    # We cheat: mutate the state between dispatches manually.
    state = _state_with_qpos(right_qpos=np.zeros(7))
    disp.dispatch(np.ones(7, dtype=np.float32), state)
    args, _ = ctrl.send_arm_joints.call_args
    np.testing.assert_allclose(args[1], [0.10] * 7, atol=1e-6)
    # Now the operator pretends the robot reached the previous target.
    state.right_arm_qpos = np.full(7, 0.10, dtype=np.float32)
    disp.dispatch(np.ones(7, dtype=np.float32), state)
    args, _ = ctrl.send_arm_joints.call_args
    np.testing.assert_allclose(args[1], [0.20] * 7, atol=1e-6)


def test_dispatch_with_gripper_calls_send_gripper_with_pct90():
    """Gripper dim still uses absolute mixer mapping (asymmetric)."""
    disp, ctrl = _make_dispatcher(no_gripper=False, gmax_pct=90.0)
    state = _state_with_qpos(right_qpos=np.zeros(7))
    safe_action = np.zeros(8, dtype=np.float32)
    safe_action[7] = +1.0  # gripper to gmax
    disp.dispatch(safe_action, state)
    ctrl.send_gripper.assert_called_once()
    args, _ = ctrl.send_gripper.call_args
    assert args[0] == "right"
    assert args[1] == pytest.approx(90.0)


def test_dispatch_dual_arm_uses_left_delta_scale():
    """Per-side delta_scale: left arm has its own vector."""
    scale_r = np.full(7, 0.10, dtype=np.float32)
    scale_l = np.full(7, 0.30, dtype=np.float32)
    disp, ctrl = _make_dispatcher(
        no_gripper=True,
        use_left_arm=True,
        delta_scale=scale_r,
        delta_scale_left=scale_l,
    )
    state = _state_with_qpos(
        right_qpos=np.zeros(7),
        left_qpos=np.zeros(7),
    )
    safe_action = np.ones(14, dtype=np.float32)  # both arms +1
    disp.dispatch(safe_action, state)
    assert ctrl.send_arm_joints.call_count == 2
    right_args = ctrl.send_arm_joints.call_args_list[0][0]
    left_args = ctrl.send_arm_joints.call_args_list[1][0]
    assert right_args[0] == "right"
    np.testing.assert_allclose(right_args[1], [0.10] * 7, atol=1e-6)
    assert left_args[0] == "left"
    np.testing.assert_allclose(left_args[1], [0.30] * 7, atol=1e-6)


def test_dispatch_returns_DispatchResult():
    disp, _ = _make_dispatcher(no_gripper=True)
    state = _state_with_qpos(right_qpos=np.zeros(7))
    res = disp.dispatch(np.zeros(7, dtype=np.float32), state)
    assert isinstance(res, DispatchResult)
    assert res.mode == "joint_delta"


# ───────────────────────── reset_to_safe ────────────────────────


def test_reset_to_safe_pose_publishes_absolute_home_q_not_delta():
    """home reset is ALWAYS absolute -- inherited verbatim from
    JointStateDispatcher, just because the home is a known safe abs
    pose, not a delta from wherever the arm happens to be."""
    home = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    disp, ctrl = _make_dispatcher(no_gripper=True, home_q_right=home)
    disp.reset_to_safe_pose(_state_with_qpos(right_qpos=np.zeros(7)))
    args, _ = ctrl.send_arm_joints.call_args
    np.testing.assert_allclose(args[1], home.tolist(), atol=1e-6)


def test_reset_to_safe_pose_with_gripper_sends_gmin():
    """Gripper at home pose goes to gmin_pct (jaws safely apart)."""
    disp, ctrl = _make_dispatcher(no_gripper=False, gmin_pct=10.0, gmax_pct=80.0)
    disp.reset_to_safe_pose(_state_with_qpos(right_qpos=np.zeros(7)))
    ctrl.send_gripper.assert_called_once()
    args, _ = ctrl.send_gripper.call_args
    assert args[1] == pytest.approx(10.0)


# ───────────────── get_required_topics + verify_topology ────────


def test_get_required_topics_same_as_joint_abs():
    """JointDeltaDispatcher publishes to the SAME topics as
    JointStateDispatcher (both go through mobiman joint_tracker)."""
    disp, _ = _make_dispatcher(no_gripper=False)
    topics = disp.get_required_topics()
    assert "/motion_target/target_joint_state_arm_right" in topics
    assert "/motion_target/target_position_gripper_right" in topics
    # Confirm we DID inherit, not re-implemented as ee.
    assert "/motion_target/target_pose_arm_right" not in topics


def test_verify_topology_raises_when_subscriber_count_zero():
    disp, ctrl = _make_dispatcher(no_gripper=True)
    ctrl.get_subscription_count.return_value = 0
    with pytest.raises(RuntimeError, match="no subscriber"):
        disp.verify_topology(ctrl)


# ───────────────────── Constructor validation ───────────────────


def test_constructor_default_delta_scale_is_galaxea_recommended():
    """Default rad/step scale matches the design doc §3.6.5 / §3.6.8."""
    disp, _ = _make_dispatcher(no_gripper=True, delta_scale=None)
    expected = np.array(
        JointDeltaDispatcher.DEFAULT_JOINT_DELTA_SCALE,
        dtype=np.float32,
    )
    np.testing.assert_allclose(disp._joint_delta_scale_right, expected, atol=1e-7)
    # Left default copies right when not provided (single-arm case).
    np.testing.assert_allclose(disp._joint_delta_scale_left, expected, atol=1e-7)


def test_constructor_rejects_zero_delta_scale():
    """A zero-element scale would freeze that joint forever; reject."""
    with pytest.raises(ValueError, match="element-wise > 0"):
        _make_dispatcher(
            no_gripper=True,
            delta_scale=np.array([0.10, 0, 0.10, 0.10, 0.20, 0.20, 0.20]),
        )


def test_constructor_rejects_negative_delta_scale():
    with pytest.raises(ValueError, match="element-wise > 0"):
        _make_dispatcher(
            no_gripper=True,
            delta_scale=np.array([-0.10] * 7),
        )


def test_constructor_rejects_wrong_delta_scale_length():
    with pytest.raises(ValueError, match="length 7"):
        _make_dispatcher(
            no_gripper=True,
            delta_scale=np.zeros(6),  # not 7
        )


def test_constructor_inherits_qmax_le_qmin_validation():
    """Bound validation is inherited from JointStateDispatcher."""
    mixer = GripperMixer(0.0, 90.0)
    with pytest.raises(ValueError, match="q_max must be > q_min"):
        JointDeltaDispatcher(
            controller=MagicMock(),
            use_right_arm=True,
            use_left_arm=False,
            no_gripper=True,
            gripper_mixer=mixer,
            q_min_right=np.ones(7),
            q_max_right=np.ones(7),
        )


# ─────────────────────────── Factory ────────────────────────────


def test_factory_joint_delta_mode_true_returns_JointDeltaDispatcher():
    mixer = GripperMixer(0.0, 90.0)
    disp = build_action_dispatcher(
        use_joint_mode=True,
        joint_delta_mode=True,
        use_right_arm=True,
        use_left_arm=False,
        no_gripper=True,
        gripper_mixer=mixer,
        controller=MagicMock(),
        q_min_right=_q_min_default(),
        q_max_right=_q_max_default(),
    )
    assert disp.mode == "joint_delta"
    assert isinstance(disp, JointDeltaDispatcher)


def test_factory_joint_delta_mode_false_returns_JointStateDispatcher():
    """Default (joint_delta_mode=False) preserves abs path."""
    mixer = GripperMixer(0.0, 90.0)
    disp = build_action_dispatcher(
        use_joint_mode=True,
        # joint_delta_mode left at default False
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
    # And NOT the delta subclass.
    assert not isinstance(disp, JointDeltaDispatcher)


def test_factory_joint_delta_mode_passes_per_arm_scale():
    mixer = GripperMixer(0.0, 90.0)
    custom_scale_r = np.array([0.05] * 7, dtype=np.float32)
    custom_scale_l = np.array([0.15] * 7, dtype=np.float32)
    disp = build_action_dispatcher(
        use_joint_mode=True,
        joint_delta_mode=True,
        use_right_arm=True,
        use_left_arm=True,
        no_gripper=True,
        gripper_mixer=mixer,
        controller=MagicMock(),
        q_min_right=_q_min_default(),
        q_max_right=_q_max_default(),
        q_min_left=_q_min_default(),
        q_max_left=_q_max_default(),
        joint_delta_scale_right=custom_scale_r,
        joint_delta_scale_left=custom_scale_l,
    )
    np.testing.assert_allclose(disp._joint_delta_scale_right, custom_scale_r)
    np.testing.assert_allclose(disp._joint_delta_scale_left, custom_scale_l)


def test_factory_ee_mode_ignores_joint_delta_mode():
    """joint_delta_mode is silently ignored when use_joint_mode=False."""
    from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import (
        EePoseDispatcher,
    )

    mixer = GripperMixer(0.0, 90.0)
    disp = build_action_dispatcher(
        use_joint_mode=False,
        joint_delta_mode=True,  # ignored
        use_right_arm=True,
        use_left_arm=False,
        no_gripper=True,
        gripper_mixer=mixer,
        controller=MagicMock(),
        ee_min_right=np.array([0.20, -0.40, 0.10], dtype=np.float32),
        ee_max_right=np.array([0.70, 0.40, 0.80], dtype=np.float32),
    )
    assert isinstance(disp, EePoseDispatcher)


# ─────────────────────────── action_dim ─────────────────────────


def test_action_dim_single_arm_with_gripper_is_8():
    """Per-arm dim: 7 joints + 1 gripper = 8 (same as abs)."""
    disp, _ = _make_dispatcher(no_gripper=False)
    assert disp.per_arm_dim == 8
    assert disp.action_dim == 8


def test_action_dim_dual_arm_no_gripper_is_14():
    disp, _ = _make_dispatcher(no_gripper=True, use_left_arm=True)
    assert disp.action_dim == 14
