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

"""Unit tests for :class:`InputParser`.

Per design doc ``test_galaxea_r1_pro_cli_controller.md`` §9.2,
``TestInputParser`` group, ~25 cases covering each verb, abbreviation,
default side, and the most likely user typos.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from toolkits.realworld_check._galaxea_cli_input import (  # noqa: E402
    CliInputError,
    InputParser,
    ParsedCommand,
)


@pytest.fixture
def parser():
    return InputParser()


# ───────────────────────── joint (abs) ─────────────────────────


def test_joint_full_arm_abs(parser):
    cmd = parser.parse("joint right 0.5 0.3 0 -1.5 0 1.8 0")
    assert cmd.verb == "joint"
    assert cmd.side == "right"
    assert cmd.payload["mode"] == "abs"
    assert cmd.payload["q_rad"] == [0.5, 0.3, 0.0, -1.5, 0.0, 1.8, 0.0]


def test_joint_alias_j_and_side_alias_r(parser):
    cmd = parser.parse("j r 0.5 0.3 0 -1.5 0 1.8 0")
    assert cmd.verb == "joint"
    assert cmd.side == "right"
    assert cmd.payload["mode"] == "abs"


def test_joint_default_side_is_right(parser):
    cmd = parser.parse("j 0.5 0.3 0 -1.5 0 1.8 0")
    assert cmd.side == "right"


def test_joint_left_side(parser):
    cmd = parser.parse("j l 0.5 0.3 0 -1.5 0 1.8 0")
    assert cmd.side == "left"


def test_joint_single_joint_abs(parser):
    cmd = parser.parse("j r 3 -1.5")
    assert cmd.payload["mode"] == "single_abs"
    assert cmd.payload["idx"] == 3
    assert cmd.payload["q_rad"] == -1.5


def test_joint_single_joint_idx_out_of_range_raises(parser):
    with pytest.raises(CliInputError, match="idx must be in"):
        parser.parse("j r 7 0.5")


def test_joint_wrong_arg_count_raises(parser):
    with pytest.raises(CliInputError, match="7 floats"):
        parser.parse("j r 0.5 0.5 0.5")  # 3 floats invalid


def test_joint_non_numeric_raises(parser):
    with pytest.raises(CliInputError, match="not a number"):
        parser.parse("j r 0.5 abc 0 -1.5 0 1.8 0")


# ───────────────────────── jn (norm) ───────────────────────────


def test_jn_full_arm_norm(parser):
    cmd = parser.parse("jn r 0.5 0.5 0 -0.5 0 0.8 0")
    assert cmd.verb == "jn"
    assert cmd.payload["mode"] == "norm"
    assert cmd.payload["q_norm"] == [0.5, 0.5, 0.0, -0.5, 0.0, 0.8, 0.0]


def test_jn_single_joint_norm(parser):
    cmd = parser.parse("jn r 0 -1.0")
    assert cmd.payload["mode"] == "single_norm"
    assert cmd.payload["idx"] == 0
    assert cmd.payload["q_norm"] == -1.0


# ───────────────────────── jd (delta) ──────────────────────────


def test_jd_delta(parser):
    cmd = parser.parse("jd r 0 0.1")
    assert cmd.verb == "jd"
    assert cmd.payload["mode"] == "delta"
    assert cmd.payload["idx"] == 0
    assert cmd.payload["delta_rad"] == 0.1


def test_jd_wrong_arg_count_raises(parser):
    with pytest.raises(CliInputError, match="<idx 0..6> <delta_rad>"):
        parser.parse("jd r 0")


def test_jd_idx_out_of_range_raises(parser):
    with pytest.raises(CliInputError, match="idx must be in"):
        parser.parse("jd r -1 0.1")


# ───────────────────────── home ────────────────────────────────


def test_home_default_side(parser):
    cmd = parser.parse("home")
    assert cmd.verb == "home"
    assert cmd.side == "right"
    assert cmd.payload == {}


def test_home_alias_h_left(parser):
    cmd = parser.parse("h l")
    assert cmd.verb == "home"
    assert cmd.side == "left"


def test_home_with_extra_args_raises(parser):
    with pytest.raises(CliInputError, match="no arguments"):
        parser.parse("home r 0.5")


# ───────────────────────── sweep / traj ────────────────────────


def test_sweep_full_args(parser):
    cmd = parser.parse("sweep r 0 -1.0 1.0 0.1 0.05")
    assert cmd.verb == "sweep"
    assert cmd.payload == {
        "idx": 0,
        "lo": -1.0,
        "hi": 1.0,
        "step": 0.1,
        "dwell_s": 0.05,
    }


def test_sweep_step_zero_raises(parser):
    with pytest.raises(CliInputError, match="step must be > 0"):
        parser.parse("sweep r 0 -1 1 0 0.1")


def test_sweep_negative_dwell_raises(parser):
    with pytest.raises(CliInputError, match="dwell"):
        parser.parse("sweep r 0 -1 1 0.1 -0.05")


def test_traj_path(parser):
    cmd = parser.parse("traj r /tmp/foo.csv")
    assert cmd.verb == "traj"
    assert cmd.payload == {"path": "/tmp/foo.csv"}


# ─────────────────────── gripper (g / gn) ──────────────────────


def test_gripper_pct(parser):
    cmd = parser.parse("g r 50")
    assert cmd.verb == "gripper"
    assert cmd.payload == {"mode": "pct", "pct": 50.0}


def test_gripper_open(parser):
    cmd = parser.parse("g r open")
    assert cmd.payload == {"mode": "open"}


def test_gripper_close(parser):
    cmd = parser.parse("g r close")
    assert cmd.payload == {"mode": "close"}


def test_gripper_alias_o_c(parser):
    assert parser.parse("g r o").payload == {"mode": "open"}
    assert parser.parse("g r c").payload == {"mode": "close"}


def test_gripper_wrong_arg_raises(parser):
    with pytest.raises(CliInputError, match="number"):
        parser.parse("g r abc")


def test_gn_norm(parser):
    cmd = parser.parse("gn r 0.5")
    assert cmd.verb == "gn"
    assert cmd.payload == {"mode": "norm", "value": 0.5}


# ───────────────────────── pose / pn ───────────────────────────


def test_pose_abs(parser):
    cmd = parser.parse("pose r 0.4 -0.1 0.3 0 0 0 1")
    assert cmd.verb == "pose"
    assert cmd.payload["mode"] == "abs"
    assert cmd.payload["pose"] == [0.4, -0.1, 0.3, 0.0, 0.0, 0.0, 1.0]


def test_pose_alias_p(parser):
    cmd = parser.parse("p r 0.4 -0.1 0.3 0 0 0 1")
    assert cmd.verb == "pose"


def test_pose_wrong_arg_count_raises(parser):
    with pytest.raises(CliInputError, match="7 floats"):
        parser.parse("pose r 0.4 -0.1 0.3")


def test_pn_norm(parser):
    cmd = parser.parse("pn r 0 0 0 0 0 0 1")
    assert cmd.verb == "pn"
    assert cmd.payload["mode"] == "norm"


# ─────────────────────────── meta ──────────────────────────────


def test_state_no_side_is_none_for_global_query(parser):
    """`state` (no side) is a global query; the REPL prints both arms."""
    cmd = parser.parse("state")
    assert cmd.verb == "state"
    assert cmd.side is None


def test_state_with_side(parser):
    cmd = parser.parse("state r")
    assert cmd.side == "right"


def test_safety_no_side(parser):
    cmd = parser.parse("safety")
    assert cmd.verb == "safety"
    assert cmd.side is None


def test_topo_alias(parser):
    cmd = parser.parse("to")
    assert cmd.verb == "topo"


def test_mode_joint(parser):
    cmd = parser.parse("mode joint")
    assert cmd.verb == "mode"
    assert cmd.payload == {"mode": "joint"}


def test_mode_ee(parser):
    cmd = parser.parse("mode ee")
    assert cmd.payload == {"mode": "ee"}


def test_mode_invalid_raises(parser):
    with pytest.raises(CliInputError, match="joint.*ee"):
        parser.parse("mode bogus")


def test_brake_on_off(parser):
    assert parser.parse("brake on").payload == {"on": True}
    assert parser.parse("br off").payload == {"on": False}


def test_brake_invalid_raises(parser):
    with pytest.raises(CliInputError, match="on.*off"):
        parser.parse("brake high")


def test_set_int(parser):
    cmd = parser.parse("set heartbeat_ms=5000")
    assert cmd.verb == "set"
    assert cmd.payload == {"k": "heartbeat_ms", "v": 5000}


def test_set_float(parser):
    cmd = parser.parse("set rate=12.5")
    assert cmd.payload == {"k": "rate", "v": 12.5}


def test_set_bool(parser):
    cmd = parser.parse("set strict=true")
    assert cmd.payload == {"k": "strict", "v": True}


def test_set_str(parser):
    cmd = parser.parse("set logfile=/tmp/x.log")
    assert cmd.payload == {"k": "logfile", "v": "/tmp/x.log"}


def test_set_missing_equals_raises(parser):
    with pytest.raises(CliInputError, match="<key>=<value>"):
        parser.parse("set heartbeat_ms 5000")


def test_help_alias_question_mark(parser):
    cmd = parser.parse("?")
    assert cmd.verb == "help"


def test_quit_aliases(parser):
    for s in ("q", "quit", "exit"):
        assert parser.parse(s).verb == "quit"


# ───────────────────── Edge cases / robustness ─────────────────


def test_empty_line_is_noop(parser):
    cmd = parser.parse("")
    assert cmd.verb == "noop"


def test_whitespace_only_is_noop(parser):
    cmd = parser.parse("   \t  ")
    assert cmd.verb == "noop"


def test_none_input_is_noop(parser):
    cmd = parser.parse(None)
    assert cmd.verb == "noop"


def test_inline_comment_stripped(parser):
    cmd = parser.parse("j r 0 0.5  # move J0 to 0.5")
    assert cmd.payload["mode"] == "single_abs"
    assert cmd.payload["idx"] == 0


def test_full_line_comment_is_noop(parser):
    cmd = parser.parse("# this is a comment")
    assert cmd.verb == "noop"


def test_unknown_verb_raises(parser):
    with pytest.raises(CliInputError, match="unknown verb"):
        parser.parse("teleport r 0 0 0")


def test_case_insensitive_verb(parser):
    cmd = parser.parse("JOINT R 0.5 0.3 0 -1.5 0 1.8 0")
    assert cmd.verb == "joint"
    assert cmd.side == "right"


def test_multiple_spaces_tolerated(parser):
    cmd = parser.parse("j   r    0.5  0.3  0  -1.5  0  1.8  0")
    assert cmd.payload["q_rad"] == [0.5, 0.3, 0.0, -1.5, 0.0, 1.8, 0.0]


def test_parsed_command_dataclass_fields():
    cmd = ParsedCommand(verb="home", side="right")
    assert cmd.verb == "home"
    assert cmd.side == "right"
    assert cmd.payload == {}
    assert cmd.raw == ""


def test_help_text_includes_all_verbs(parser):
    txt = parser.help()
    for verb in (
        "joint",
        "jn",
        "jd",
        "home",
        "sweep",
        "traj",
        "gripper",
        "gn",
        "pose",
        "pn",
        "state",
        "safety",
        "topo",
        "mode",
        "brake",
        "set",
        "help",
        "quit",
    ):
        assert verb in txt


def test_custom_verb_alias():
    """`--alias-file` would inject custom aliases at construction."""
    parser = InputParser(custom_verb_aliases={"go": "home"})
    cmd = parser.parse("go r")
    assert cmd.verb == "home"
    assert cmd.side == "right"
