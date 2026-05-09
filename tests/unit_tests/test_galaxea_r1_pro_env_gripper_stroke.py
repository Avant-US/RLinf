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

"""Gripper stroke endpoints: SafetyConfig ↔ env dispatch ↔ observations."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("gymnasium")

from rlinf.envs.realworld.galaxear.r1_pro_env import GalaxeaR1ProEnv
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import GalaxeaR1ProRobotState


def _dummy_env(**safety_kwargs) -> GalaxeaR1ProEnv:
    cfg = {"is_dummy": True, "cameras": []}
    if safety_kwargs:
        cfg["safety_cfg"] = safety_kwargs
    return GalaxeaR1ProEnv(cfg)


def test_gripper_action_to_mm_default_matches_legacy():
    env = _dummy_env()
    assert env._gripper_normalized_action_to_mm(-1.0) == pytest.approx(0.0)
    assert env._gripper_normalized_action_to_mm(0.0) == pytest.approx(50.0)
    assert env._gripper_normalized_action_to_mm(1.0) == pytest.approx(100.0)


def test_gripper_action_to_mm_brs_style_endpoints():
    env = _dummy_env(
        gripper_closed_stroke_mm=10.0,
        gripper_open_stroke_mm=90.0,
    )
    assert env._gripper_normalized_action_to_mm(-1.0) == pytest.approx(10.0)
    assert env._gripper_normalized_action_to_mm(0.0) == pytest.approx(50.0)
    assert env._gripper_normalized_action_to_mm(1.0) == pytest.approx(90.0)


def test_build_state_dict_gripper_obs_uses_minus1_to_1():
    env = _dummy_env(
        gripper_closed_stroke_mm=10.0,
        gripper_open_stroke_mm=90.0,
    )
    env._state.right_gripper_pos = 50.0  # mm mid-stroke -> pos01=0.5 -> obs=0
    d = env._build_state_dict()
    assert "right_gripper_pos" in d
    assert float(d["right_gripper_pos"][0]) == pytest.approx(0.0, abs=1e-5)
    env._state.right_gripper_pos = 10.0
    d2 = env._build_state_dict()
    assert float(d2["right_gripper_pos"][0]) == pytest.approx(-1.0, abs=1e-5)
    env._state.right_gripper_pos = 90.0
    d3 = env._build_state_dict()
    assert float(d3["right_gripper_pos"][0]) == pytest.approx(1.0, abs=1e-5)


def test_get_state_vector_gripper_kw_only_endpoints():
    st = GalaxeaR1ProRobotState()
    st.right_gripper_pos = 35.0  # mm: pos01=(35-10)/80=0.3125 -> obs=-0.375
    v = st.get_state_vector(
        gripper_closed_stroke_mm=10.0,
        gripper_open_stroke_mm=90.0,
    )
    # right_qpos(7) + right_ee(7) + right_gripper(1) = 15
    assert v.shape == (15,)
    assert float(v[14]) == pytest.approx(-0.375, abs=1e-5)


def test_get_state_vector_invalid_endpoints_falls_back_to_legacy_span():
    st = GalaxeaR1ProRobotState()
    st.right_gripper_pos = 50.0  # 0/100 fallback -> pos01=0.5 -> obs=0
    v = st.get_state_vector(
        gripper_closed_stroke_mm=90.0,
        gripper_open_stroke_mm=10.0,
    )
    assert float(v[14]) == pytest.approx(0.0, abs=1e-5)


def test_invalid_safety_gripper_endpoints_fallback():
    """``open <= closed`` must not break dispatch/obs: fall back to 0/100 mm."""
    env = _dummy_env(
        gripper_closed_stroke_mm=50.0,
        gripper_open_stroke_mm=50.0,
    )
    closed, open_ = env._gripper_stroke_bounds_mm()
    assert closed == 0.0 and open_ == 100.0
    assert env._gripper_normalized_action_to_mm(1.0) == pytest.approx(100.0)
