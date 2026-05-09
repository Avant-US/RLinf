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

"""Unit tests for :class:`StateMonitor`.

Per design doc ``test_galaxea_r1_pro_cli_controller.md`` §9.2,
``TestStateMonitor`` group.  Verifies the format contract: each
field appears in the right place with the right tag prefix so
operators can grep `[STATE]`, `[SAFETY]`, `[TOPO]`, `[TX]` reliably.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer import (  # noqa: E402
    GripperMixer,
)
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (  # noqa: E402
    GalaxeaR1ProRobotState,
)
from rlinf.envs.realworld.galaxear.r1_pro_safety import SafetyInfo  # noqa: E402
from toolkits.realworld_check._galaxea_backends import DummyBackend  # noqa: E402
from toolkits.realworld_check._galaxea_cli_state import (  # noqa: E402
    StateMonitor,
    Tag,
    TopologySpec,
    fmt_vec,
    now_iso,
)


@pytest.fixture
def monitor():
    return StateMonitor(GripperMixer(0.0, 90.0))


# ──────────────────────── fmt_vec / now_iso ─────────────────────


def test_fmt_vec_3d_default_precision():
    s = fmt_vec([1.0, -2.5, 0.0])
    assert s == "[+1.0000, -2.5000, +0.0000]"


def test_fmt_vec_custom_precision():
    s = fmt_vec([1.0], precision=2)
    assert s == "[+1.00]"


def test_now_iso_format():
    s = now_iso()
    # Like 2026-05-08T15:30:14.123
    assert "T" in s
    assert s.count(":") == 2
    assert "." in s


# ─────────────────────── pretty_state ───────────────────────────


def test_pretty_state_includes_all_fields(monitor):
    state = GalaxeaR1ProRobotState()
    state.right_arm_qpos = np.array(
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        dtype=np.float32,
    )
    state.right_gripper_pos = 45.0
    state.right_ee_pose = np.array(
        [0.4, -0.1, 0.3, 0.0, 0.0, 0.0, 1.0],
        dtype=np.float32,
    )
    txt = monitor.pretty_state(state, side="right")
    # Tag and required fields
    assert Tag.STATE.strip() in txt
    assert "side=right" in txt
    assert "q   (rad):" in txt
    assert "qvel" in txt
    assert "ee_xyz" in txt
    assert "ee_quat" in txt
    assert "gripper:" in txt
    # Values appear (formatted)
    assert "+0.1000" in txt or "+0.10" in txt
    assert "torso_link4" in txt
    # Health line
    assert "bms=" in txt
    assert "alive=" in txt
    assert "stale_topics=" in txt


def test_pretty_state_side_left(monitor):
    state = GalaxeaR1ProRobotState()
    state.left_arm_qpos = np.array([0.5] * 7, dtype=np.float32)
    txt = monitor.pretty_state(state, side="left")
    assert "side=left" in txt
    assert "+0.5000" in txt


def test_pretty_state_no_side_prints_both(monitor):
    state = GalaxeaR1ProRobotState()
    txt = monitor.pretty_state(state, side=None)
    assert "side=right" in txt
    assert "side=left" in txt


def test_pretty_state_gripper_obs_norm_correct(monitor):
    state = GalaxeaR1ProRobotState()
    state.right_gripper_pos = 45.0  # midpoint of [0, 90] -> obs=0
    txt = monitor.pretty_state(state, side="right")
    assert "obs_norm=+0.000" in txt
    # gmax label
    assert "gmax=90" in txt


def test_pretty_state_gripper_with_custom_business_range():
    monitor = StateMonitor(GripperMixer(10.0, 80.0))
    state = GalaxeaR1ProRobotState()
    state.right_gripper_pos = 45.0  # midpoint of [10, 80] -> obs=0
    txt = monitor.pretty_state(state, side="right")
    assert "obs_norm=+0.000" in txt
    assert "gmin=10" in txt
    assert "gmax=80" in txt


def test_pretty_state_health_line_counts_stale(monitor):
    state = GalaxeaR1ProRobotState()
    state.feedback_age_ms = {
        "arm_right": 50.0,
        "arm_left": 250.0,  # > 200ms threshold -> 1 stale
        "bms": 80.0,
    }
    txt = monitor.pretty_state(state, side="right")
    assert "stale_topics=1" in txt


def test_pretty_state_health_line_counts_status_errors(monitor):
    state = GalaxeaR1ProRobotState()
    state.status_errors = {"right": [{"name": "x", "code": 1}]}
    txt = monitor.pretty_state(state, side="right")
    assert "status_errors=1" in txt


# ────────────────────── safety_overlay ──────────────────────────


def test_safety_overlay_clean_pass(monitor):
    info = SafetyInfo(
        raw_action=np.zeros(7, dtype=np.float32),
        safe_action=np.zeros(7, dtype=np.float32),
    )
    txt = monitor.safety_overlay(info)
    assert Tag.SAFETY.strip() in txt
    assert "reason_count=0" in txt
    assert "clipped=False" in txt
    assert "soft_hold=False" in txt


def test_safety_overlay_with_reasons(monitor):
    info = SafetyInfo(
        raw_action=np.zeros(7, dtype=np.float32),
        safe_action=np.zeros(7, dtype=np.float32),
        clipped=True,
        soft_hold=True,
        reason=["L2a:right_q_critical J3", "L4:per_step_cap"],
        metrics={"safety/clip_ratio": 1.0, "hw/bms_capital_pct": 95.5},
    )
    txt = monitor.safety_overlay(info)
    assert "reason_count=2" in txt
    assert "L2a:right_q_critical J3" in txt
    assert "L4:per_step_cap" in txt
    assert "metrics:" in txt
    assert "safety/clip_ratio=" in txt


def test_safety_overlay_emergency_stop(monitor):
    info = SafetyInfo(
        raw_action=np.zeros(7, dtype=np.float32),
        safe_action=np.zeros(7, dtype=np.float32),
        emergency_stop=True,
        reason=["L1:non_finite_action"],
    )
    txt = monitor.safety_overlay(info)
    assert "emergency_stop=True" in txt
    assert "L1:non_finite_action" in txt


# ─────────────────────── topology_report ────────────────────────


def test_topology_report_all_ok(monitor):
    backend = DummyBackend()
    spec = TopologySpec(
        publishers=[
            "/motion_target/target_joint_state_arm_right",
            "/motion_target/target_position_gripper_right",
        ],
        subscribers=["/hdas/feedback_arm_right"],
        optional=["/hdas/bms"],
    )
    txt = monitor.topology_report(backend, spec)
    assert Tag.TOPO.strip() in txt
    assert "backend=dummy" in txt
    assert "/motion_target/target_joint_state_arm_right" in txt
    assert "subs=  1" in txt
    assert "OK" in txt
    assert "all OK" in txt


def test_topology_report_warn_on_missing_subscriber(monitor):
    """Custom backend that returns 0 subs for one topic -> WARN."""

    class _ZeroSubsBackend(DummyBackend):
        def get_subscription_count(self, topic):
            return 0

    backend = _ZeroSubsBackend()
    spec = TopologySpec(
        publishers=["/motion_target/target_joint_state_arm_right"],
        subscribers=[],
    )
    txt = monitor.topology_report(backend, spec)
    assert "WARN" in txt
    assert "missing topics" in txt


def test_topology_report_warn_on_missing_publisher(monitor):
    class _ZeroPubsBackend(DummyBackend):
        def get_publisher_count(self, topic):
            return 0

    backend = _ZeroPubsBackend()
    spec = TopologySpec(
        publishers=[],
        subscribers=["/hdas/feedback_arm_right"],
    )
    txt = monitor.topology_report(backend, spec)
    assert "WARN" in txt


def test_topology_report_optional_does_not_warn(monitor):
    """Missing optional topics should print but NOT trigger WARN summary."""

    class _MixedBackend(DummyBackend):
        def get_publisher_count(self, topic):
            return 0 if "bms" in topic else 1

    backend = _MixedBackend()
    spec = TopologySpec(
        publishers=[],
        subscribers=["/hdas/feedback_arm_right"],
        optional=["/hdas/bms"],
    )
    txt = monitor.topology_report(backend, spec)
    assert "all OK" in txt  # still OK because bms is optional
    assert "/hdas/bms" in txt
    assert "missing" in txt


# ──────────────────────── tx_line ───────────────────────────────


def test_tx_line_arm_joints(monitor):
    rec = {
        "type": "arm_joints",
        "side": "right",
        "q_target": [0.1] * 7,
        "qvel_max": [3.0] * 7,
        "ts": 0.0,
    }
    txt = monitor.tx_line(rec)
    assert Tag.TX.strip() in txt
    assert "/motion_target/target_joint_state_arm_right" in txt
    assert "pos=[+0.1000" in txt
    assert "vmax=[+3.00" in txt


def test_tx_line_arm_pose(monitor):
    rec = {
        "type": "arm_pose",
        "side": "left",
        "pose": [0.4, -0.1, 0.3, 0.0, 0.0, 0.0, 1.0],
        "ts": 0.0,
    }
    txt = monitor.tx_line(rec)
    assert "/motion_target/target_pose_arm_left" in txt
    assert "pose=[+0.4000" in txt


def test_tx_line_gripper(monitor):
    rec = {"type": "gripper", "side": "right", "pct": 50.0, "ts": 0.0}
    txt = monitor.tx_line(rec)
    assert "/motion_target/target_position_gripper_right" in txt
    assert "pct=[ 50.00]" in txt


def test_tx_line_brake(monitor):
    rec = {"type": "brake", "on": True, "ts": 0.0}
    txt = monitor.tx_line(rec)
    assert "/motion_target/brake_mode" in txt
    assert "data=True" in txt


def test_tx_line_dry_uses_dry_tag(monitor):
    rec = {
        "type": "arm_joints",
        "side": "right",
        "q_target": [0.0] * 7,
        "qvel_max": [3.0] * 7,
        "ts": 0.0,
    }
    txt = monitor.tx_line(rec, dry=True)
    assert "[TX-DRY]" in txt
    assert "[TX]    " not in txt


def test_tx_line_unknown_record_type(monitor):
    rec = {"type": "weirdo", "ts": 0.0}
    txt = monitor.tx_line(rec)
    assert "unknown record type weirdo" in txt


# ──────────────────────── info / warn ──────────────────────────


def test_info_helper():
    s = StateMonitor.info("hello world")
    assert "[INFO]" in s
    assert "hello world" in s


def test_warn_helper():
    s = StateMonitor.warn("danger")
    assert "[WARN]" in s
    assert "danger" in s


def test_error_helper():
    s = StateMonitor.error("oops")
    assert "[ERROR]" in s
    assert "oops" in s
