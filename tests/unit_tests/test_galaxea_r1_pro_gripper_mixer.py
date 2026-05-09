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

"""Unit tests for :class:`GripperMixer`.

Per design doc r1pro6op47.md §3.4, the mixer must round-trip cleanly
between policy ``[-1, 1]`` and physical pct ``[0, 100]`` through a
configurable business range.  These tests cover the four boundary
cases (``-1``, ``+1``, midpoint, out-of-range), default and custom
business ranges, the inverse map, and constructor validation.
"""

from __future__ import annotations

import numpy as np
import pytest

from rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer import GripperMixer

# ───────────────────────── Forward map ─────────────────────────


def test_action11_to_pct_default_max90_pos_one_hits_90():
    mx = GripperMixer()
    assert mx.action11_to_pct(1.0) == pytest.approx(90.0, abs=1e-6)


def test_action11_to_pct_default_max90_neg_one_hits_0():
    mx = GripperMixer()
    assert mx.action11_to_pct(-1.0) == pytest.approx(0.0, abs=1e-6)


def test_action11_to_pct_default_max90_zero_hits_45():
    mx = GripperMixer()
    assert mx.action11_to_pct(0.0) == pytest.approx(45.0, abs=1e-6)


def test_action11_to_pct_clips_action_above_one():
    mx = GripperMixer(gmin_pct=0.0, gmax_pct=90.0)
    # Even if upstream forgot to clip, mixer must not return >gmax_pct.
    assert mx.action11_to_pct(2.0) == pytest.approx(90.0, abs=1e-6)


def test_action11_to_pct_clips_action_below_neg_one():
    mx = GripperMixer(gmin_pct=0.0, gmax_pct=90.0)
    assert mx.action11_to_pct(-2.0) == pytest.approx(0.0, abs=1e-6)


def test_action11_to_pct_custom_range_10_50_half_action():
    mx = GripperMixer(gmin_pct=10.0, gmax_pct=50.0)
    # a = 0 -> midpoint 30; a = 0.5 -> 30 + 0.25*40 = 40.
    assert mx.action11_to_pct(0.0) == pytest.approx(30.0, abs=1e-6)
    assert mx.action11_to_pct(0.5) == pytest.approx(40.0, abs=1e-6)


# ───────────────────────── Inverse map ─────────────────────────


def test_pct_to_obs11_midpoint_returns_zero():
    mx = GripperMixer(gmin_pct=0.0, gmax_pct=90.0)
    assert mx.pct_to_obs11(45.0) == pytest.approx(0.0, abs=1e-6)


def test_pct_to_obs11_max_returns_pos_one():
    mx = GripperMixer(gmin_pct=0.0, gmax_pct=90.0)
    assert mx.pct_to_obs11(90.0) == pytest.approx(1.0, abs=1e-6)


def test_pct_to_obs11_min_returns_neg_one():
    mx = GripperMixer(gmin_pct=0.0, gmax_pct=90.0)
    assert mx.pct_to_obs11(0.0) == pytest.approx(-1.0, abs=1e-6)


def test_pct_to_obs11_above_business_max_clamps_to_pos_one():
    """Feedback > gmax_pct (e.g. 95 mm but business max is 90) should
    saturate to +1 instead of producing >1 observations."""
    mx = GripperMixer(gmin_pct=0.0, gmax_pct=90.0)
    assert mx.pct_to_obs11(95.0) == pytest.approx(1.0, abs=1e-6)


def test_pct_to_obs11_below_business_min_clamps_to_neg_one():
    mx = GripperMixer(gmin_pct=10.0, gmax_pct=90.0)
    assert mx.pct_to_obs11(5.0) == pytest.approx(-1.0, abs=1e-6)


# ───────────────────────── Round trip ──────────────────────────


def test_roundtrip_action_to_pct_to_obs():
    """For every action in [-1, 1], pct_to_obs11(action11_to_pct(a)) == a."""
    mx = GripperMixer(gmin_pct=0.0, gmax_pct=90.0)
    for a in np.linspace(-1.0, 1.0, 11):
        pct = mx.action11_to_pct(float(a))
        obs = mx.pct_to_obs11(pct)
        assert obs == pytest.approx(float(a), abs=1e-5), (
            f"round trip broke at a={a}: pct={pct}, obs={obs}"
        )


def test_roundtrip_with_custom_range_10_50():
    mx = GripperMixer(gmin_pct=10.0, gmax_pct=50.0)
    for a in np.linspace(-1.0, 1.0, 11):
        pct = mx.action11_to_pct(float(a))
        obs = mx.pct_to_obs11(pct)
        assert obs == pytest.approx(float(a), abs=1e-5)


# ─────────────────────── Constructor ───────────────────────────


def test_constructor_rejects_inverted_range():
    with pytest.raises(ValueError, match="gmin_pct < gmax_pct"):
        GripperMixer(gmin_pct=80.0, gmax_pct=50.0)


def test_constructor_rejects_equal_bounds():
    with pytest.raises(ValueError):
        GripperMixer(gmin_pct=50.0, gmax_pct=50.0)


def test_constructor_rejects_below_physical_min():
    with pytest.raises(ValueError):
        GripperMixer(gmin_pct=-1.0, gmax_pct=90.0)


def test_constructor_rejects_above_physical_max():
    with pytest.raises(ValueError):
        GripperMixer(gmin_pct=0.0, gmax_pct=101.0)


def test_constructor_accepts_full_physical_range():
    mx = GripperMixer(gmin_pct=0.0, gmax_pct=100.0)
    assert mx.gmin_pct == 0.0
    assert mx.gmax_pct == 100.0
    assert mx.span_pct == 100.0


def test_repr_includes_bounds():
    mx = GripperMixer(gmin_pct=10.0, gmax_pct=80.0)
    s = repr(mx)
    assert "10.0" in s
    assert "80.0" in s
