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

"""Unit tests for the 5-level :class:`GalaxeaR1ProSafetySupervisor`."""

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
    SafetyInfo,
    build_safety_config,
)


def _schema_single_arm():
    return ActionSchema(
        has_left_arm=False, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )


def _schema_dual_arm():
    return ActionSchema(
        has_left_arm=True, has_right_arm=True,
        has_torso=False, has_chassis=False,
        no_gripper=False,
        action_scale=np.array([0.05, 0.10, 1.0], dtype=np.float32),
    )


def _state_with_ee(right_xyz=(0.4, -0.1, 0.3), left_xyz=(0.4, 0.1, 0.3)):
    st = GalaxeaR1ProRobotState()
    st.right_ee_pose = np.array(
        [*right_xyz, 0.0, 0.0, 0.0, 1.0], dtype=np.float32,
    )
    st.left_ee_pose = np.array(
        [*left_xyz, 0.0, 0.0, 0.0, 1.0], dtype=np.float32,
    )
    return st


def test_l1_schema_clip_to_unit_box():
    cfg = SafetyConfig()
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    a = np.array([2.0, -2.0, 0.5, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info = sup.validate(a, _state_with_ee(), _schema_single_arm())
    assert info.clipped is True
    assert any("L1" in r for r in info.reason)
    assert info.safe_action[0] == 1.0
    assert info.safe_action[1] == -1.0


def test_l1_non_finite_action_emergency_stop():
    cfg = SafetyConfig()
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    a = np.array([np.nan, 0, 0, 0, 0, 0, 0], dtype=np.float32)
    info = sup.validate(a, _state_with_ee(), _schema_single_arm())
    assert info.emergency_stop is True
    assert any("non_finite" in r for r in info.reason)


def test_l3a_clips_outside_ee_box():
    cfg = SafetyConfig()
    cfg.right_ee_max = np.array([0.45, 0.10, 0.40, 3.20, 0.30, 0.30],
                                dtype=np.float32)
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_single_arm()
    state = _state_with_ee(right_xyz=(0.40, 0.05, 0.35))
    # +x action would push EE to 0.45 + 0.05*1.0 = 0.50 (above limit 0.45).
    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info = sup.validate(a, state, schema)
    assert info.clipped is True
    assert any("L3a" in r for r in info.reason)


def test_l3b_dual_arm_collision_soft_hold():
    cfg = SafetyConfig()
    cfg.dual_arm_collision_enable = True
    cfg.dual_arm_min_distance_m = 0.20
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    state = _state_with_ee(right_xyz=(0.40, 0.05, 0.30),
                           left_xyz=(0.40, 0.10, 0.30))
    a = np.zeros(14, dtype=np.float32)
    info = sup.validate(a, state, _schema_dual_arm())
    assert info.soft_hold is True
    assert any("dual_arm_collision" in r for r in info.reason)


def test_l4_per_step_cap():
    cfg = SafetyConfig()
    cfg.max_linear_step_m = 0.01  # Tight cap.
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    schema = _schema_single_arm()
    a = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    info = sup.validate(a, _state_with_ee(), schema)
    assert info.clipped is True
    assert any("L4:per_step_cap" in r for r in info.reason)


def test_l5_bms_low_battery_safe_stop():
    cfg = SafetyConfig()
    cfg.bms_low_battery_threshold_pct = 25.0
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    state = _state_with_ee()
    state.bms["capital_pct"] = 10.0
    info = sup.validate(np.zeros(7, dtype=np.float32), state,
                        _schema_single_arm())
    assert info.safe_stop is True
    assert any("bms_low" in r for r in info.reason)
    assert np.allclose(info.safe_action, 0.0)


def test_l5_swd_emergency_stop():
    cfg = SafetyConfig()
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    state = _state_with_ee()
    state.controller_signal["swd"] = 1
    info = sup.validate(np.zeros(7, dtype=np.float32), state,
                        _schema_single_arm())
    assert info.emergency_stop is True
    assert any("SWD" in r for r in info.reason)


def test_l5_status_errors_soft_hold():
    cfg = SafetyConfig()
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    state = _state_with_ee()
    state.status_errors = {"right": [42]}
    info = sup.validate(np.zeros(7, dtype=np.float32), state,
                        _schema_single_arm())
    assert info.soft_hold is True
    assert any("status_errors_right" in r for r in info.reason)


def test_l5_feedback_stale_soft_hold():
    cfg = SafetyConfig()
    cfg.feedback_stale_threshold_ms = 100.0
    sup = GalaxeaR1ProSafetySupervisor(cfg)
    state = _state_with_ee()
    state.feedback_age_ms = {"arm_right": 500.0}
    info = sup.validate(np.zeros(7, dtype=np.float32), state,
                        _schema_single_arm())
    assert info.soft_hold is True
    assert any("stale" in r for r in info.reason)


def test_build_safety_config_filters_unknown_keys():
    cfg = build_safety_config({
        "bms_low_battery_threshold_pct": 30.0,
        "unknown_key": 123,
    })
    assert cfg.bms_low_battery_threshold_pct == 30.0


def test_safety_info_hold_or_stop():
    info = SafetyInfo(raw_action=np.zeros(1), safe_action=np.zeros(1))
    assert info.hold_or_stop is False
    info.soft_hold = True
    assert info.hold_or_stop is True
