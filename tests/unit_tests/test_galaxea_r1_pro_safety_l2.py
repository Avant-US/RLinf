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

"""Deep unit tests for the L2 *per-joint limit* layer of
:class:`GalaxeaR1ProSafetySupervisor`.

L2 is split into four sub-layers (L2a/L2b/L2c/L2d) plus a gripper
sub-check; see ``bt/docs/rwRL/safety_2_joinlimit_2.md`` §4 for the
contract.

Each test is self-contained, dummy-mode only (no Ray, no ROS 2)
and runs in <100ms on CPU.  The companion file
``test_galaxea_r1_pro_safety.py`` covers L1/L3/L4/L5 with one
smoke test per layer; here we zoom in on L2.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

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


def _schema_single():
    return ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )


def _schema_dual():
    return ActionSchema(
        has_left_arm=True, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )


def _schema_full():
    """Right arm + torso + chassis (no left arm to keep dim small)."""
    return ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=True, has_chassis=True,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )


def _strict_l2_only_cfg() -> SafetyConfig:
    """Disable L3/L4/L5 so the test can assert L2 effects in isolation.

    L1 stays on (it's a dtype/finite check), L3a is disabled by
    growing the EE box to infinity, L3b/L5 are disabled via flags,
    L4 is disabled by raising every cap so it never trips.
    """
    cfg = SafetyConfig()
    # Loosen L3a so it never clips in these tests.
    cfg.right_ee_min = np.array([-100, -100, -100, -100, -100, -100],
                                dtype=np.float32)
    cfg.right_ee_max = np.array([100, 100, 100, 100, 100, 100],
                                dtype=np.float32)
    cfg.left_ee_min = cfg.right_ee_min.copy()
    cfg.left_ee_max = cfg.right_ee_max.copy()
    # Disable L3b.
    cfg.dual_arm_collision_enable = False
    # Loosen L4.
    cfg.max_linear_step_m = 100.0
    cfg.max_angular_step_rad = 100.0
    cfg.torso_v_x_max = 100.0
    cfg.torso_v_z_max = 100.0
    cfg.torso_w_pitch_max = 100.0
    cfg.torso_w_yaw_max = 100.0
    cfg.chassis_v_x_max = 100.0
    cfg.chassis_v_y_max = 100.0
    cfg.chassis_w_z_max = 100.0
    # Disable L5 watchdogs we don't want firing.
    cfg.bms_low_battery_threshold_pct = -1.0
    cfg.feedback_stale_threshold_ms = 1e9
    cfg.operator_heartbeat_timeout_ms = 1e9
    cfg.a2_fall_risk_pct = -1.0
    return cfg


def _state_with_qpos(
    *,
    right_qpos=None,
    right_qvel=None,
    left_qpos=None,
    left_qvel=None,
    torso_qpos=None,
    torso_qvel=None,
    chassis_qvel=None,
    right_gripper_pos=50.0,
    left_gripper_pos=50.0,
) -> GalaxeaR1ProRobotState:
    """Build a state with optional per-component qpos/qvel populated."""
    st = GalaxeaR1ProRobotState()
    # Identity orientation EE so L3a never falsely trips even when
    # someone forgets to grow the box.
    st.right_ee_pose = np.array([0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0],
                                dtype=np.float32)
    st.left_ee_pose = np.array([0.4, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
                               dtype=np.float32)
    if right_qpos is not None:
        st.right_arm_qpos = np.asarray(right_qpos, dtype=np.float32)
    if right_qvel is not None:
        st.right_arm_qvel = np.asarray(right_qvel, dtype=np.float32)
    if left_qpos is not None:
        st.left_arm_qpos = np.asarray(left_qpos, dtype=np.float32)
    if left_qvel is not None:
        st.left_arm_qvel = np.asarray(left_qvel, dtype=np.float32)
    if torso_qpos is not None:
        st.torso_qpos = np.asarray(torso_qpos, dtype=np.float32)
    if torso_qvel is not None:
        st.torso_qvel = np.asarray(torso_qvel, dtype=np.float32)
    if chassis_qvel is not None:
        st.chassis_qvel = np.asarray(chassis_qvel, dtype=np.float32)
    st.right_gripper_pos = float(right_gripper_pos)
    st.left_gripper_pos = float(left_gripper_pos)
    return st


# ─────────────────────────── Tests ─────────────────────────────


# 1. URDF cross-check ----------------------------------------------

def _find_urdf() -> str | None:
    candidates = [
        "/home/nvidia/galaxea/install/mobiman/share/mobiman/urdf/R1_PRO/urdf/r1_pro.urdf",
        "/home/nvidia/galaxea/install/mobiman/lib/mobiman/configs/urdfs/r1_pro_floating_right.urdf",
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return None


def test_l2_defaults_match_urdf_with_margin():
    """SafetyConfig defaults must lie strictly inside the URDF
    mechanical limits (URDF lower < q_min, q_max < URDF upper).

    Skipped on hosts where the Galaxea SDK is not installed (e.g.
    CI runners without ``/home/nvidia/galaxea/install``)."""
    urdf_path = _find_urdf()
    if urdf_path is None:
        pytest.skip("Galaxea SDK URDF not found; skipping cross-check")

    root = ET.parse(urdf_path).getroot()
    urdf_limits: dict[str, tuple[float, float]] = {}
    for j in root.findall("joint"):
        if j.get("type") != "revolute":
            continue
        name = j.get("name", "")
        if not (name.startswith("left_arm_joint")
                or name.startswith("right_arm_joint")):
            continue
        lim = j.find("limit")
        if lim is None:
            continue
        urdf_limits[name] = (
            float(lim.get("lower")),
            float(lim.get("upper")),
        )

    cfg = SafetyConfig()
    for i in range(7):
        # Right arm.
        rname = f"right_arm_joint{i + 1}"
        if rname in urdf_limits:
            urdf_lo, urdf_hi = urdf_limits[rname]
            cfg_lo = float(cfg.right_arm_q_min[i])
            cfg_hi = float(cfg.right_arm_q_max[i])
            assert cfg_lo > urdf_lo - 1e-3, (
                f"right J{i + 1}: q_min={cfg_lo} <= URDF lower={urdf_lo}"
            )
            assert cfg_hi < urdf_hi + 1e-3, (
                f"right J{i + 1}: q_max={cfg_hi} >= URDF upper={urdf_hi}"
            )

        # Left arm: only present in the integrated URDF; floating-right
        # URDF won't have it -> skip silently.
        lname = f"left_arm_joint{i + 1}"
        if lname in urdf_limits:
            urdf_lo, urdf_hi = urdf_limits[lname]
            cfg_lo = float(cfg.left_arm_q_min[i])
            cfg_hi = float(cfg.left_arm_q_max[i])
            assert cfg_lo > urdf_lo - 1e-3, (
                f"left J{i + 1}: q_min={cfg_lo} <= URDF lower={urdf_lo}"
            )
            assert cfg_hi < urdf_hi + 1e-3, (
                f"left J{i + 1}: q_max={cfg_hi} >= URDF upper={urdf_hi}"
            )


# 2. Left/right J2 mirror -------------------------------------------

def test_l2_left_right_j2_are_mirrored():
    """R1 Pro arms are mechanically mirrored on J2 only -- the
    SafetyConfig defaults must reflect that."""
    cfg = SafetyConfig()
    # J2 mirrored: right is [-3.04, +0.07], left is [-0.07, +3.04].
    assert float(cfg.right_arm_q_min[1]) == pytest.approx(-3.04, abs=1e-3)
    assert float(cfg.right_arm_q_max[1]) == pytest.approx(0.07, abs=1e-3)
    assert float(cfg.left_arm_q_min[1]) == pytest.approx(-0.07, abs=1e-3)
    assert float(cfg.left_arm_q_max[1]) == pytest.approx(3.04, abs=1e-3)
    # The other six joints are identical between the two arms.
    for i in (0, 2, 3, 4, 5, 6):
        assert float(cfg.right_arm_q_min[i]) == float(cfg.left_arm_q_min[i])
        assert float(cfg.right_arm_q_max[i]) == float(cfg.left_arm_q_max[i])


# 3. L2a warning_margin -> linear shrink -----------------------------

def test_l2a_warning_margin_triggers_proportional_scale():
    """When predicted q is inside warning_margin (but outside critical),
    L2a should scale the per-arm action proportionally to
    margin / warning_margin."""
    cfg = _strict_l2_only_cfg()
    # Use joint mode so predict_arm_qpos returns cur_q + delta.
    cfg.l2_warning_margin_rad = 0.20
    cfg.l2_critical_margin_rad = 0.05
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    # Build a joint-mode schema by hand (use_joint_mode=True).
    schema = _schema_single()
    schema.use_joint_mode = True
    # Place J6 at +0.85 rad (q_max=0.95, so margin=0.10).  An action
    # +1 with action_scale[0]=0.05 will move J6 to 0.90 -> margin
    # 0.05 (right at critical).  Pick J3 instead, plenty of room.
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.85, 0.0],
                     dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q)
    # action[5] (J6) = +1 -> delta = +0.05 -> q_next J6 = 0.90.
    a = np.zeros(7, dtype=np.float32)
    a[5] = 1.0
    info = sup.validate(a, state, schema)
    # Predicted worst margin after raw step would be q_max-0.90=0.05 == critical.
    # We want the WARNING branch only, so loosen warning to 0.20 (set above)
    # and place J6 a bit further: 0.80 -> q_next 0.85 -> margin 0.10
    # which is within warning (0.20) but outside critical (0.05).
    cur_q[5] = 0.80
    state = _state_with_qpos(right_qpos=cur_q)
    info = sup.validate(a.copy(), state, schema)

    assert info.clipped is True
    assert any("L2a:right_q_warning" in r for r in info.reason)
    # In joint mode the q_next = cur_q + delta * scale_pos.
    # delta (J6) = a[5] * action_scale[0] = 0.05 -> q_next J6 = 0.85,
    # margin = q_max(0.95) - 0.85 = 0.10 -> scale = 0.10/0.20 = 0.5.
    expected_scale = 0.5
    assert info.safe_action[5] == pytest.approx(
        a[5] * expected_scale, abs=1e-5
    )


# 4. L2a critical_margin -> scale zero -------------------------------

def test_l2a_critical_margin_freezes_arm_slice():
    """When predicted q falls inside critical_margin, L2a forces
    scale = 0 (and L2b will set soft_hold next step)."""
    cfg = _strict_l2_only_cfg()
    cfg.l2_warning_margin_rad = 0.20
    cfg.l2_critical_margin_rad = 0.05
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_single()
    schema.use_joint_mode = True
    # Start J6 at +0.93 rad (q_max=0.95) so even a +0.01 motion
    # crosses critical.
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.93, 0.0],
                     dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q)
    a = np.zeros(7, dtype=np.float32)
    a[5] = 1.0
    info = sup.validate(a.copy(), state, schema)
    assert info.clipped is True
    assert any("L2a:right_q_critical" in r for r in info.reason)
    # Scale must be 0 -> arm slice is zeroed.
    assert info.safe_action[5] == pytest.approx(0.0, abs=1e-6)


# 5. L2a Cartesian mode is a no-op ----------------------------------

def test_l2a_cartesian_mode_is_no_op_when_far_from_limits():
    """In Cartesian (default) mode, predict_arm_qpos returns the
    current qpos.  L2a must therefore not fire when the current
    qpos has plenty of margin."""
    cfg = _strict_l2_only_cfg()
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_single()  # default: use_joint_mode=False
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0],
                     dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q)
    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)
    assert not any("L2a" in r for r in info.reason)
    # Nothing else should clip either (we disabled L3/L4/L5).
    assert info.safe_action[0] == pytest.approx(1.0, abs=1e-6)


# 6. L2b qpos already inside critical -> soft_hold -------------------

def test_l2b_qpos_already_critical_triggers_soft_hold_and_zero():
    cfg = _strict_l2_only_cfg()
    cfg.l2_critical_margin_rad = 0.05
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_single()
    # J6 at +0.93 (margin to q_max=0.95 is 0.02 < 0.05 critical).
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.93, 0.0],
                     dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q)
    a = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.5], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)
    assert info.soft_hold is True
    assert any("L2b:right_qpos_critical" in r for r in info.reason)
    # Arm slice (xyz+rpy) zeroed; gripper untouched.
    assert np.allclose(info.safe_action[:6], 0.0)
    # Per-joint audit recorded for J6.
    assert "right" in info.l2_per_joint_clip
    assert "J6" in info.l2_per_joint_clip["right"]


# 7. L2c qvel warning -> scale 0.5 -----------------------------------

def test_l2c_qvel_warning_scales_action_half():
    cfg = _strict_l2_only_cfg()
    cfg.l2_qvel_warning_factor = 0.90
    cfg.l2_qvel_overspeed_factor = 1.20
    # Pick non-zero qpos so L2b doesn't fire.
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0],
                     dtype=np.float32)
    # J5 qvel = 3.8 rad/s, limit = 4.0 -> ratio = 0.95 > 0.90.
    qvel = np.array([0.0, 0.0, 0.0, 0.0, 3.8, 0.0, 0.0], dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q, right_qvel=qvel)
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_single()
    a = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)
    assert info.clipped is True
    assert info.soft_hold is False
    assert any("L2c:right_qvel_warning" in r for r in info.reason)
    # Arm slice scaled by 0.5; gripper untouched.
    assert np.allclose(info.safe_action[:6], 0.5)
    assert info.safe_action[6] == pytest.approx(1.0, abs=1e-6)


# 8. L2c qvel overspeed -> soft_hold ---------------------------------

def test_l2c_qvel_overspeed_triggers_soft_hold():
    cfg = _strict_l2_only_cfg()
    cfg.l2_qvel_warning_factor = 0.90
    cfg.l2_qvel_overspeed_factor = 1.20
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0],
                     dtype=np.float32)
    # J5 qvel = 5.0 rad/s, limit = 4.0 -> ratio = 1.25 > 1.20.
    qvel = np.array([0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0], dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q, right_qvel=qvel)
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_single()
    a = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)
    assert info.soft_hold is True
    assert any("L2c:right_qvel_overspeed" in r for r in info.reason)
    # Arm slice zeroed; gripper untouched.
    assert np.allclose(info.safe_action[:6], 0.0)


# 9. L2d chassis dead-zone ------------------------------------------

def test_l2d_chassis_dead_zone_zeros_small_cmd():
    cfg = _strict_l2_only_cfg()
    cfg.chassis_dead_zone = np.array([0.05, 0.05, 0.05], dtype=np.float32)
    cfg.chassis_qvel_max = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_full()
    # Right arm + torso + chassis -> action layout = 7 + 4 + 3 = 14.
    a = np.zeros(14, dtype=np.float32)
    # vx=0.02 (dead-zone), vy=0.5 (kept), wz=0.001 (dead-zone).
    a[11] = 0.02
    a[12] = 0.5
    a[13] = 0.001
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0],
                     dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q)
    info = sup.validate(a.copy(), state, schema)
    assert info.clipped is True
    assert any("L2d:chassis_cmd_speed_cap" in r for r in info.reason)
    # vx and wz zeroed; vy unchanged.
    assert info.safe_action[11] == pytest.approx(0.0, abs=1e-6)
    assert info.safe_action[12] == pytest.approx(0.5, abs=1e-6)
    assert info.safe_action[13] == pytest.approx(0.0, abs=1e-6)


# 10. L2d chassis cap on per-axis maximum ---------------------------

def test_l2d_chassis_cap_clips_to_qvel_max():
    cfg = _strict_l2_only_cfg()
    cfg.chassis_dead_zone = np.array([0.001, 0.001, 0.001],
                                     dtype=np.float32)
    cfg.chassis_qvel_max = np.array([0.30, 0.30, 0.40],
                                    dtype=np.float32)
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_full()
    a = np.zeros(14, dtype=np.float32)
    # Demand vx=1.0 (cap to 0.30) and wz=-1.0 (cap to -0.40).
    a[11] = 1.0
    a[13] = -1.0
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0],
                     dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q)
    info = sup.validate(a.copy(), state, schema)
    assert info.clipped is True
    assert any("L2d:chassis_cmd_speed_cap" in r for r in info.reason)
    assert info.safe_action[11] == pytest.approx(0.30, abs=1e-6)
    assert info.safe_action[13] == pytest.approx(-0.40, abs=1e-6)


# 11. Gripper position clamp ----------------------------------------

def test_l2_gripper_position_clamp_caps_to_max():
    """Use an action_scale[2]=50 so that a normalised gripper action
    of 1.0 translates to +50 %/step delta -- enough to overshoot the
    100% upper bound from a starting 95% pct.  L1 will not clip
    a=1.0, so the L2 gripper layer can see the full request and
    rewrite it back via predict-clip-rewrite."""
    cfg = _strict_l2_only_cfg()
    cfg.gripper_pct_min = 0.0
    cfg.gripper_pct_max = 100.0
    cfg.max_gripper_step_pct = 1e6  # disable rate cap for this test
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    # Use a realistic gripper scale (RLinf default uses 1.0 with a
    # different mapping; here we choose 50 so a in [-1,1] -> ±50%/step,
    # which is the natural range for the L2 gripper layer's predict-
    # clip-rewrite contract.)
    schema = ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 50.0], dtype=np.float32),
    )

    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0],
                     dtype=np.float32)
    # Gripper currently at 95.0%; a=1.0 with scale 50 -> delta=+50.
    # Predicted next pct = 145 -> cap to 100 -> new_delta = 5
    # -> new_a = 5/50 = 0.1.
    state = _state_with_qpos(right_qpos=cur_q, right_gripper_pos=95.0)
    a = np.zeros(7, dtype=np.float32)
    a[6] = 1.0
    info = sup.validate(a.copy(), state, schema)
    assert info.clipped is True
    assert any("L2:right_gripper" in r and "pos_cap" in r
               for r in info.reason)
    assert info.safe_action[6] == pytest.approx(0.1, abs=1e-5)


# 12. Gripper rate-limit --------------------------------------------

def test_l2_gripper_rate_limit_caps_step():
    cfg = _strict_l2_only_cfg()
    cfg.gripper_pct_min = 0.0
    cfg.gripper_pct_max = 100.0
    cfg.max_gripper_step_pct = 10.0
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 50.0], dtype=np.float32),
    )

    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0],
                     dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q, right_gripper_pos=20.0)
    # a[6]=1.0 with scale=50 -> demanded delta=+50%/step (above cap=10).
    a = np.zeros(7, dtype=np.float32)
    a[6] = 1.0
    info = sup.validate(a.copy(), state, schema)
    assert info.clipped is True
    assert any("L2:right_gripper" in r and "rate_cap" in r
               for r in info.reason)
    # Capped to +10%/step -> new_a = 10 / 50 = 0.2.
    assert info.safe_action[6] == pytest.approx(0.2, abs=1e-5)


# 13. Back-compat: legacy `arm_q_min/max` in YAML ---------------------

def test_back_compat_legacy_arm_q_min_max_aliases():
    """Old YAML configs that pass ``arm_q_min`` / ``arm_q_max`` should
    populate both right and left arm vectors and emit a deprecation
    warning.  Used to keep existing pre-v2 configs running."""
    legacy_min = [-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0]
    legacy_max = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
    cfg = SafetyConfig(
        arm_q_min=np.asarray(legacy_min, dtype=np.float32),
        arm_q_max=np.asarray(legacy_max, dtype=np.float32),
    )
    np.testing.assert_allclose(cfg.right_arm_q_min, legacy_min)
    np.testing.assert_allclose(cfg.right_arm_q_max, legacy_max)
    np.testing.assert_allclose(cfg.left_arm_q_min, legacy_min)
    np.testing.assert_allclose(cfg.left_arm_q_max, legacy_max)


# 14. SafetyInfo.l2_per_joint_clip is populated --------------------

def test_safety_info_l2_per_joint_clip_audit_field():
    cfg = _strict_l2_only_cfg()
    cfg.l2_critical_margin_rad = 0.05
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_single()
    # J6 at the limit -> L2b should fire.
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.93, 0.0],
                     dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q)
    a = np.zeros(7, dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)
    assert "right" in info.l2_per_joint_clip
    assert "J6" in info.l2_per_joint_clip["right"]
    # The L2b layer recorded "freeze".
    assert info.l2_per_joint_clip["right"]["J6"]["L2b"] == "freeze"


# 15. Per-arm independence: only the violating arm is rewritten ----

def test_l2a_dual_arm_only_violating_arm_is_scaled():
    cfg = _strict_l2_only_cfg()
    cfg.l2_warning_margin_rad = 0.20
    cfg.l2_critical_margin_rad = 0.05
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_dual()
    schema.use_joint_mode = True
    # Right arm J6 close to limit (will trigger L2a warning).
    right_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.80, 0.0],
                       dtype=np.float32)
    # Left arm well inside limits.
    left_q = np.array([0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0],
                      dtype=np.float32)
    state = _state_with_qpos(right_qpos=right_q, left_qpos=left_q)
    # Action layout (dual arm): right(7) + left(7) = 14.
    a = np.zeros(14, dtype=np.float32)
    a[5] = 1.0   # right J6 +1
    a[5 + 7] = 1.0   # left J6 +1
    info = sup.validate(a.copy(), state, schema)
    # right scaled by 0.5 (margin 0.10 / warning 0.20).
    assert info.safe_action[5] == pytest.approx(0.5, abs=1e-5)
    # left untouched.
    assert info.safe_action[5 + 7] == pytest.approx(1.0, abs=1e-5)
    assert any("L2a:right_q_warning" in r for r in info.reason)
    assert not any("L2a:left_q_" in r for r in info.reason)


# 16. Toggles disable the corresponding sub-layer -------------------

def test_l2_toggles_disable_sublayers():
    cfg = _strict_l2_only_cfg()
    cfg.enable_l2a_predict_q_clip = False
    cfg.enable_l2b_qpos_freeze = False
    cfg.enable_l2c_qvel_watchdog = False
    cfg.enable_l2d_cmd_speed_cap = False
    cfg.enable_l2_gripper = False
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_single()
    # A pose that *would* trigger L2b otherwise.
    cur_q = np.array([0.0, -0.5, 0.0, -1.0, 0.0, 0.93, 0.0],
                     dtype=np.float32)
    state = _state_with_qpos(right_qpos=cur_q, right_gripper_pos=99.0)
    a = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 50.0],
                 dtype=np.float32)
    info = sup.validate(a.copy(), state, schema)
    assert not any(r.startswith("L2") for r in info.reason)
