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

"""Deep unit tests for the L3a *TCP safety box* layer of
:class:`GalaxeaR1ProSafetySupervisor`.

The companion file ``test_galaxea_r1_pro_safety.py`` already
covers L1-L5 with one smoke test per layer.  This file zooms in
on L3a (:meth:`r1_pro_safety._clip_to_box` +
:meth:`_rewrite_arm_action`) because:

* L3a is the only layer that *rewrites* the action vector
  (predict-clip-invert pipeline).  A bug here corrupts every
  action without the rest of the pipeline noticing.
* The reverse-write precision (:func:`_rewrite_arm_action`) and
  multi-axis interaction were explicitly listed as untested in
  ``bt/docs/rwRL/safety_2.md`` §8.1 / §9.8.

Each test is self-contained, dummy-mode only (no Ray, no ROS2)
and runs in <100ms on CPU.
"""

from __future__ import annotations

import numpy as np
import pytest

from rlinf.envs.realworld.galaxear.r1_pro_action_schema import (
    ActionSchema,
)
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (
    GalaxeaR1ProRobotState,
)
from rlinf.envs.realworld.galaxear.r1_pro_safety import (
    GalaxeaR1ProSafetySupervisor,
    SafetyConfig,
)


# ─────────────────────── Helpers / fixtures ────────────────────


def _schema(
    *,
    has_left_arm: bool = False,
    has_torso: bool = False,
    has_chassis: bool = False,
    no_gripper: bool = False,
    action_scale=(0.05, 0.10, 1.0),
) -> ActionSchema:
    """Default schema with right arm enabled, others optional."""
    return ActionSchema(
        has_left_arm=has_left_arm,
        has_right_arm=True,
        has_torso=has_torso,
        has_chassis=has_chassis,
        no_gripper=no_gripper,
        action_scale=np.asarray(action_scale, dtype=np.float32),
    )


def _state(
    *,
    right_xyz=(0.40, 0.0, 0.30),
    right_quat=(0.0, 0.0, 0.0, 1.0),
    left_xyz=(0.40, 0.20, 0.30),
    left_quat=(0.0, 0.0, 0.0, 1.0),
) -> GalaxeaR1ProRobotState:
    """Build a state with explicit per-arm EE poses (xyz + quat xyzw).

    Defaults keep the right arm well *inside* the default safety
    box and the left EE 20 cm to the left of the right EE so L3b
    is never triggered.
    """
    st = GalaxeaR1ProRobotState()
    st.right_ee_pose = np.array(
        [*right_xyz, *right_quat], dtype=np.float32,
    )
    st.left_ee_pose = np.array(
        [*left_xyz, *left_quat], dtype=np.float32,
    )
    return st


def _supervisor_with_box(
    *,
    right_ee_min=None,
    right_ee_max=None,
    left_ee_min=None,
    left_ee_max=None,
    dual_arm_collision_enable=False,
) -> GalaxeaR1ProSafetySupervisor:
    """Build a supervisor with custom L3a box and L3b disabled
    by default (so single-axis tests stay clean)."""
    cfg = SafetyConfig()
    if right_ee_min is not None:
        cfg.right_ee_min = np.asarray(right_ee_min, dtype=np.float32)
    if right_ee_max is not None:
        cfg.right_ee_max = np.asarray(right_ee_max, dtype=np.float32)
    if left_ee_min is not None:
        cfg.left_ee_min = np.asarray(left_ee_min, dtype=np.float32)
    if left_ee_max is not None:
        cfg.left_ee_max = np.asarray(left_ee_max, dtype=np.float32)
    cfg.dual_arm_collision_enable = dual_arm_collision_enable
    return GalaxeaR1ProSafetySupervisor(cfg)


# ───────────────────── 1. No-clip happy path ───────────────────


def test_l3a_action_inside_box_does_not_clip():
    """A small action that keeps the predicted EE inside the box
    must NOT add an L3a reason and must NOT modify the action."""
    sup = _supervisor_with_box(
        right_ee_max=[0.45, 0.10, 0.40, 3.20, 0.30, 0.30],
    )
    schema = _schema()
    state = _state(right_xyz=(0.40, 0.0, 0.30))
    a = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)

    # Predicted EE x = 0.40 + 0.5 * 0.05 = 0.425 < 0.45 -> inside box.
    assert not any("L3a" in r for r in info.reason), info.reason
    np.testing.assert_allclose(info.safe_action, a, atol=1e-6)


def test_l3a_zero_action_inside_box_no_reason():
    """Zero action with EE *strictly* inside the default box must
    leave the action untouched and must not flag L3a."""
    sup = _supervisor_with_box()  # all defaults
    schema = _schema()
    state = _state(right_xyz=(0.40, 0.0, 0.30))
    info = sup.validate(np.zeros(7, dtype=np.float32), state, schema)
    assert not any("L3a" in r for r in info.reason), info.reason
    np.testing.assert_allclose(info.safe_action, 0.0, atol=1e-6)


# ───────────────────── 2. Per-axis boundary clip ────────────────


@pytest.mark.parametrize(
    "axis_idx,box_max_val,cur_val,direction,physical_step",
    [
        (0, 0.42, 0.40, +1.0, 0.05),  # +X face
        (1, 0.32, 0.30, +1.0, 0.05),  # +Y face
        (2, 0.40, 0.38, +1.0, 0.05),  # +Z face
    ],
)
def test_l3a_clips_each_xyz_max_face(
    axis_idx, box_max_val, cur_val, direction, physical_step,
):
    """Drive each of x/y/z towards a custom max face and verify the
    rewritten action lands the predicted EE *exactly* on the face.

    With ``cur = box_max - 0.02`` and ``action = +1`` (one full
    step of 0.05 m), predicted = box_max + 0.03 -> clipped to
    box_max -> norm = (box_max - cur) / scale = 0.02 / 0.05 = 0.4.
    """
    box_max = np.array([0.65, 0.35, 0.65, 3.20, 0.30, 0.30], dtype=np.float32)
    box_max[axis_idx] = box_max_val
    sup = _supervisor_with_box(right_ee_max=box_max.tolist())
    schema = _schema()

    cur_xyz = [0.40, 0.0, 0.30]
    cur_xyz[axis_idx] = cur_val
    state = _state(right_xyz=tuple(cur_xyz))

    a = np.zeros(7, dtype=np.float32)
    a[axis_idx] = direction
    info = sup.validate(a.copy(), state, schema)

    expected_norm = (box_max_val - cur_val) / 0.05  # action_scale[0]
    assert any("L3a:right_ee_box" in r for r in info.reason), info.reason
    assert info.safe_action[axis_idx] == pytest.approx(expected_norm, abs=1e-5)
    # untouched dims stay zero
    other = np.delete(info.safe_action[:6], axis_idx)
    np.testing.assert_allclose(other, 0.0, atol=1e-6)


def test_l3a_clips_x_min_face_to_inside():
    """Pulling EE in -X past the *min* face is clipped back to the
    floor.  The rewrite must produce a *negative* normalised step."""
    sup = _supervisor_with_box(
        right_ee_min=[0.28, -0.35, 0.05, -3.20, -0.30, -0.30],
    )
    schema = _schema()
    state = _state(right_xyz=(0.30, 0.0, 0.30))
    a = np.array([-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)

    # predict_x = 0.30 - 0.05 = 0.25 < 0.28 -> clipped to 0.28
    # delta_x  = 0.28 - 0.30 = -0.02; norm = -0.02 / 0.05 = -0.4
    assert any("L3a:right_ee_box" in r for r in info.reason)
    assert info.safe_action[0] == pytest.approx(-0.4, abs=1e-5)


# ───────────────────── 3. Orientation (rpy) clip ────────────────


def test_l3a_clips_yaw_max_face():
    """RPY clipping uses ``action_scale[1]`` (default 0.10 rad)."""
    sup = _supervisor_with_box(
        right_ee_max=[0.65, 0.35, 0.65, 3.20, 0.30, 0.05],
    )
    schema = _schema()
    state = _state()  # identity quat -> cur_eul = (0, 0, 0)
    a = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)

    # predict_yaw = 0 + 1.0 * 0.10 = 0.10 rad > 0.05 -> clipped to 0.05
    # delta_yaw = 0.05 - 0 = 0.05; norm = 0.05 / 0.10 = 0.5
    assert any("L3a:right_ee_box" in r for r in info.reason)
    assert info.safe_action[5] == pytest.approx(0.5, abs=1e-5)


# ───────────────────── 4. Reverse-write round-trip ─────────────


def test_l3a_rewrite_round_trip_predicted_ee_equals_clipped():
    """The rewritten action, when fed back through
    :meth:`predict_arm_ee_pose`, must yield exactly the clipped
    box-target.  This is the precision contract called out in
    ``safety_2.md`` §9.8 as previously untested.
    """
    box_max = [0.42, 0.10, 0.40, 3.20, 0.30, 0.05]
    sup = _supervisor_with_box(right_ee_max=box_max)
    schema = _schema()
    state = _state(right_xyz=(0.40, 0.05, 0.38))
    a = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)

    # Re-predict with the (rewritten) safe_action.
    nxt = schema.predict_arm_ee_pose("right", info.safe_action, state)
    assert nxt is not None

    # x / y / z should land on the clipped target (the box max).
    np.testing.assert_allclose(nxt[0], 0.42, atol=1e-5)
    np.testing.assert_allclose(nxt[1], 0.10, atol=1e-5)
    np.testing.assert_allclose(nxt[2], 0.40, atol=1e-5)
    # yaw should land on box_max[5] = 0.05 rad
    np.testing.assert_allclose(nxt[5], 0.05, atol=1e-5)


def test_l3a_rewrite_uses_action_scale_correctly():
    """Doubling ``action_scale[0]`` halves the resulting normalised
    step for a given physical clip distance."""
    schema_small = _schema(action_scale=(0.05, 0.10, 1.0))
    schema_large = _schema(action_scale=(0.10, 0.10, 1.0))
    box_max = [0.42, 0.10, 0.40, 3.20, 0.30, 0.30]
    sup = _supervisor_with_box(right_ee_max=box_max)
    state = _state(right_xyz=(0.40, 0.0, 0.30))

    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info_small = sup.validate(a.copy(), state, schema_small)
    info_large = sup.validate(a.copy(), state, schema_large)

    # Physical clip distance 0.02 m / 0.05 = 0.4   (small scale)
    # Physical clip distance 0.02 m / 0.10 = 0.2   (large scale)
    assert info_small.safe_action[0] == pytest.approx(0.4, abs=1e-5)
    assert info_large.safe_action[0] == pytest.approx(0.2, abs=1e-5)


# ───────────────────── 5. Slice isolation ──────────────────────


def test_l3a_does_not_modify_gripper_dim():
    """L3a only rewrites the per-arm xyz+rpy slice (indices 0-5).
    The gripper element at index 6 must not be touched."""
    sup = _supervisor_with_box(
        right_ee_max=[0.42, 0.10, 0.40, 3.20, 0.30, 0.30],
    )
    schema = _schema()
    state = _state(right_xyz=(0.40, 0.0, 0.30))
    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)
    assert info.safe_action[6] == pytest.approx(0.7, abs=1e-6)


def test_l3a_dual_arm_only_violating_arm_is_rewritten():
    """In a dual-arm config, clipping the right arm must not
    perturb the left-arm slice (and vice-versa)."""
    sup = _supervisor_with_box(
        right_ee_max=[0.42, 0.35, 0.65, 3.20, 0.30, 0.30],
        left_ee_max=[0.65, 0.35, 0.65, 3.20, 0.30, 0.30],
        dual_arm_collision_enable=False,
    )
    schema = _schema(has_left_arm=True)
    state = _state(
        right_xyz=(0.40, -0.20, 0.30),
        left_xyz=(0.40, 0.20, 0.30),  # 0.4 m apart -> no L3b
    )
    # Action: +x for both arms; right will exceed 0.42, left stays
    # within 0.65 (predict_left_x = 0.45 < 0.65).
    a = np.zeros(14, dtype=np.float32)
    a[0] = 1.0  # right xyz_x
    a[7] = 1.0  # left  xyz_x
    info = sup.validate(a.copy(), state, schema)

    assert any("L3a:right_ee_box" in r for r in info.reason)
    assert not any("L3a:left_ee_box" in r for r in info.reason)
    # right rewritten to 0.4, left untouched at 1.0
    assert info.safe_action[0] == pytest.approx(0.4, abs=1e-5)
    assert info.safe_action[7] == pytest.approx(1.0, abs=1e-6)


# ───────────────────── 6. Multi-axis combined ──────────────────


def test_l3a_clips_multiple_axes_simultaneously():
    """Pushing x AND z out of the box at the same time must clip
    *both* and emit only a single ``L3a:right_ee_box`` reason
    (the tag is per-arm, not per-axis)."""
    sup = _supervisor_with_box(
        right_ee_max=[0.42, 0.35, 0.40, 3.20, 0.30, 0.30],
    )
    schema = _schema()
    state = _state(right_xyz=(0.40, 0.0, 0.38))
    a = np.array([1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)

    l3a_reasons = [r for r in info.reason if r.startswith("L3a")]
    assert len(l3a_reasons) == 1
    # both axes clipped to the same physical 0.02 m offset
    assert info.safe_action[0] == pytest.approx(0.4, abs=1e-5)
    assert info.safe_action[2] == pytest.approx(0.4, abs=1e-5)


# ───────────────────── 7. Schema gating ────────────────────────


def test_l3a_skipped_when_no_arm_in_schema():
    """A torso-only schema must not run any L3a check."""
    cfg = SafetyConfig()
    cfg.dual_arm_collision_enable = False
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = ActionSchema(
        has_left_arm=False,
        has_right_arm=False,
        has_torso=True,
        has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )
    state = _state()
    info = sup.validate(np.zeros(4, dtype=np.float32), state, schema)
    assert not any("L3a" in r for r in info.reason), info.reason


# ───────────────────── 8. Audit / metrics ──────────────────────


def test_l3a_clip_sets_metrics_and_keeps_raw_action():
    """A clipped step must mark ``safety/clip_ratio = 1.0`` while
    preserving the raw action (audit contract)."""
    sup = _supervisor_with_box(
        right_ee_max=[0.42, 0.10, 0.40, 3.20, 0.30, 0.30],
    )
    schema = _schema()
    state = _state(right_xyz=(0.40, 0.0, 0.30))
    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)

    np.testing.assert_allclose(info.raw_action, a, atol=1e-6)
    assert info.metrics["safety/clip_ratio"] == pytest.approx(1.0)
    # only L3a fired -> not soft_hold / safe_stop / emergency_stop
    assert info.soft_hold is False
    assert info.safe_stop is False
    assert info.emergency_stop is False


# ───────────────────── 9. Left-arm symmetric path ──────────────


def test_l3a_left_arm_clips_at_min_face_symmetric_to_right():
    """Verify the left arm clip path mirrors the right one."""
    sup = _supervisor_with_box(
        left_ee_min=[0.28, -0.35, 0.05, -3.20, -0.30, -0.30],
        dual_arm_collision_enable=False,
    )
    schema = _schema(has_left_arm=True)
    state = _state(
        right_xyz=(0.40, -0.30, 0.30),  # parked far away
        left_xyz=(0.30, 0.20, 0.30),  # near min x
    )
    # action[7] indexes left arm xyz_x in the dual-arm layout.
    a = np.zeros(14, dtype=np.float32)
    a[7] = -1.0
    info = sup.validate(a.copy(), state, schema)

    assert any("L3a:left_ee_box" in r for r in info.reason)
    # delta_x = 0.28 - 0.30 = -0.02; norm = -0.4
    assert info.safe_action[7] == pytest.approx(-0.4, abs=1e-5)
    # right arm slice untouched
    np.testing.assert_allclose(info.safe_action[:6], 0.0, atol=1e-6)


# ───────────────────── 10. EE outside box recovery ─────────────


def test_l3a_recovery_when_current_ee_already_outside_box():
    """If the EE *starts* outside the box (e.g. operator parked
    the arm in an unsafe pose), L3a must rewrite the action so
    the next step moves it *back into* the box, even when the
    raw policy action would push further out.
    """
    sup = _supervisor_with_box(
        right_ee_max=[0.42, 0.10, 0.40, 3.20, 0.30, 0.30],
    )
    schema = _schema()
    # cur_x = 0.46 -> already 0.04 m past the 0.42 max.
    state = _state(right_xyz=(0.46, 0.0, 0.30))
    # Policy wants +x:  predict_x = 0.46 + 0.05 = 0.51, far outside.
    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)

    # Clipped target = 0.42 ; delta = 0.42 - 0.46 = -0.04 ; norm = -0.8.
    # The norm is well within [-1, 1] AND L4 cap (|0.04| <= 0.05),
    # so the rewrite should come through unmodified.
    assert any("L3a:right_ee_box" in r for r in info.reason)
    assert info.safe_action[0] == pytest.approx(-0.8, abs=1e-5)
    # Verify the post-rewrite predicted EE is strictly back at 0.42.
    nxt = schema.predict_arm_ee_pose("right", info.safe_action, state)
    np.testing.assert_allclose(nxt[0], 0.42, atol=1e-5)
