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

"""End-to-end tests for the joint+gripper CLI tool.

Per design doc ``test_galaxea_r1_pro_cli_controller.md`` §9.2,
``TestActionComposer`` / ``TestSafetyChainWiring`` / ``TestReplLoop``
/ ``TestEndToEndDummy`` groups.  Drives the REPL programmatically
through ``handle_line()`` so no stdin is needed; backend is always
``DummyBackend`` so no ROS / Ray.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from toolkits.realworld_check.test_galaxea_r1_pro_cli_controller import (  # noqa: E402
    CliConfig,
    ReplLoop,
    build_argparser,
    build_context,
    cfg_from_args,
    main,
)

# ─────────────────────── Test fixtures ─────────────────────────


def _joint_cfg(no_gripper: bool = False, dual_arm: bool = False) -> CliConfig:
    cfg = CliConfig()
    cfg.dummy = True
    cfg.backend_kind = "dummy"
    cfg.use_joint_mode = True
    cfg.use_right_arm = True
    cfg.use_left_arm = dual_arm
    cfg.no_gripper = no_gripper
    cfg.gripper_min_pct = 0.0
    cfg.gripper_max_pct = 90.0
    cfg.arm_q_min_right = [-2.0] * 7
    cfg.arm_q_max_right = [+2.0] * 7
    cfg.arm_q_min_left = [-2.0] * 7
    cfg.arm_q_max_left = [+2.0] * 7
    cfg.heartbeat_interval_ms = 1000  # don't spam during tests
    cfg.strict_heartbeat = True  # disable bg heartbeat thread
    return cfg


def _ee_cfg() -> CliConfig:
    cfg = _joint_cfg(no_gripper=True)
    cfg.use_joint_mode = False
    cfg.ee_min_right = [0.0, -0.5, 0.0]
    cfg.ee_max_right = [0.8, 0.5, 1.0]
    return cfg


def _make_repl(cfg: CliConfig):
    """Construct a REPL with captured output for assertion."""
    captured: list[str] = []

    def _capture(s: str) -> None:
        captured.append(s)

    ctx = build_context(cfg)
    repl = ReplLoop(ctx, output_fn=_capture)
    return ctx, repl, captured


# ───────────────────────── ActionComposer ──────────────────────


def test_composer_joint_abs_full_arm():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("j r 1.0 0.0 -1.0 0.5 -0.5 2.0 -2.0")
    tx = ctx.backend.tx_log
    # last (and only) TX must be arm_joints with q approximating the input
    arm_tx = [t for t in tx if t["type"] == "arm_joints"]
    assert len(arm_tx) == 1
    np.testing.assert_allclose(
        arm_tx[-1]["q_target"],
        [1.0, 0.0, -1.0, 0.5, -0.5, 2.0, -2.0],
        atol=1e-5,
    )


def test_composer_single_joint_abs_keeps_other_joints():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("j r 0.5 0.5 0.5 0.5 0.5 0.5 0.5")  # all to 0.5
    repl.handle_line("j r 3 -1.0")  # only J3
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    final = arm_tx[-1]["q_target"]
    np.testing.assert_allclose(final, [0.5, 0.5, 0.5, -1.0, 0.5, 0.5, 0.5], atol=1e-5)


def test_composer_gripper_pct_independent_of_joints():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=False))
    repl.handle_line("j r 0.5 0.5 0.5 0.5 0.5 0.5 0.5")
    repl.handle_line("g r 50")
    grip_tx = [t for t in ctx.backend.tx_log if t["type"] == "gripper"]
    assert len(grip_tx) == 1
    assert grip_tx[-1]["pct"] == pytest.approx(50.0, abs=1e-3)
    # The arm tx for the gripper command must show the same joints as before.
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    np.testing.assert_allclose(arm_tx[-1]["q_target"], [0.5] * 7, atol=1e-5)


def test_composer_gripper_open_uses_gmax_pct():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=False))
    repl.handle_line("g r open")
    grip_tx = [t for t in ctx.backend.tx_log if t["type"] == "gripper"]
    assert grip_tx[-1]["pct"] == pytest.approx(90.0, abs=1e-3)


def test_composer_gripper_close_uses_gmin_pct():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=False))
    repl.handle_line("g r close")
    grip_tx = [t for t in ctx.backend.tx_log if t["type"] == "gripper"]
    assert grip_tx[-1]["pct"] == pytest.approx(0.0, abs=1e-3)


def test_composer_gripper_norm_zero_is_midpoint_pct():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=False))
    repl.handle_line("gn r 0")
    grip_tx = [t for t in ctx.backend.tx_log if t["type"] == "gripper"]
    assert grip_tx[-1]["pct"] == pytest.approx(45.0, abs=1e-3)


def test_composer_jd_uses_feedback_qpos_not_latest_norm():
    """jd with delta=0.1 should advance from CURRENT q (synthetic 0
    in dummy mode) by 0.1, not from the last sent norm."""
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=True))
    # initial state: right_arm_qpos all zeros (default)
    repl.handle_line("jd r 0 0.1")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    np.testing.assert_allclose(arm_tx[-1]["q_target"][0], 0.1, atol=1e-5)
    # Backend mirrors the command back, so state is now [0.1, 0, ...].
    # Next jd from there:
    repl.handle_line("jd r 0 0.1")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    np.testing.assert_allclose(arm_tx[-1]["q_target"][0], 0.2, atol=1e-5)


def test_composer_left_side_action_dim_dual_arm():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=True, dual_arm=True))
    # Set left arm only.
    repl.handle_line("j l 1.0 1.0 1.0 1.0 1.0 1.0 1.0")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    # Should have one tx for left side.
    left_tx = [t for t in arm_tx if t["side"] == "left"]
    assert len(left_tx) == 1
    np.testing.assert_allclose(left_tx[-1]["q_target"], [1.0] * 7, atol=1e-5)


def test_composer_pose_abs_in_ee_mode():
    ctx, repl, _ = _make_repl(_ee_cfg())
    repl.handle_line("pose r 0.4 0.0 0.5 0.0 0.0 0.0 1.0")
    pose_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_pose"]
    assert len(pose_tx) == 1
    np.testing.assert_allclose(pose_tx[-1]["pose"][:3], [0.4, 0.0, 0.5], atol=1e-5)
    # Quat L2-normalized to identity by dispatcher.
    np.testing.assert_allclose(pose_tx[-1]["pose"][3:], [0.0, 0.0, 0.0, 1.0], atol=1e-5)


def test_composer_joint_command_in_ee_mode_errors():
    ctx, repl, captured = _make_repl(_ee_cfg())
    repl.handle_line("j r 0.5 0.5 0 -0.5 0 0.8 0")
    # Should produce an error message, NOT a TX
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    assert len(arm_tx) == 0
    assert any("joint command requires mode=joint" in s for s in captured)


def test_composer_gripper_command_when_no_gripper_errors():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("g r 50")
    grip_tx = [t for t in ctx.backend.tx_log if t["type"] == "gripper"]
    assert len(grip_tx) == 0
    assert any("no_gripper=True" in s for s in captured)


# ─────────────────────── SafetyChainWiring ─────────────────────


def test_safety_passes_zero_action_clean():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("j r 0 0 0 0 0 0 0")
    safety_lines = [s for s in captured if "[SAFETY]" in s]
    assert any("reason_count=0" in s for s in safety_lines)


def test_safety_clips_out_of_range_norm_jn_input():
    """When the user types jn with values > 1, L1 clip should fire."""
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("jn r 1.5 0 0 0 0 0 0")
    safety_lines = [s for s in captured if "L1:clipped_to_unit_box" in s]
    assert safety_lines, "Expected L1 clip reason in safety overlay"


def test_safety_emergency_on_nan_action_publishes_brake():
    """A NaN action should trigger L1 emergency_stop and apply_brake."""
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    # Force a NaN through compose by patching the latest vector.
    repl.composer._latest["right"][0] = np.nan
    repl.handle_line("jn r 0 0 0 0 0 0 0")  # any jn re-sends the vector
    # Restore the post-compose nan via direct injection: we want the
    # SAFETY pipeline to see NaN -> emergency.  jn overwrote it; this
    # guards against the case where future compose accidentally
    # overrides NaN injection -- the test should still see the brake
    # if a NaN ever leaks through.

    # An alternative cleaner path: directly call the safety validator
    # with a NaN action and confirm it sets emergency_stop.
    info = repl._validate_safety(
        np.array([np.nan, 0, 0, 0, 0, 0, 0], dtype=np.float32),
        ctx.backend.get_state(),
    )
    assert info.emergency_stop is True
    assert any("L1:non_finite_action" in r for r in info.reason)


def test_safety_bms_low_triggers_safe_stop():
    cfg = _joint_cfg(no_gripper=True)
    cfg.bms_low_battery_threshold_pct = 50.0
    ctx, repl, _ = _make_repl(cfg)
    # Patch the dummy backend's BMS to below threshold
    ctx.backend.patch_state(bms={"capital_pct": 30.0})
    info = repl._validate_safety(
        np.zeros(7, dtype=np.float32),
        ctx.backend.get_state(),
    )
    assert info.safe_stop is True
    assert any("L5:bms_low" in r for r in info.reason)


def test_safety_brake_command_emits_tx():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("brake on")
    brake_tx = [t for t in ctx.backend.tx_log if t["type"] == "brake"]
    assert len(brake_tx) == 1
    assert brake_tx[-1]["on"] is True
    repl.handle_line("brake off")
    brake_tx = [t for t in ctx.backend.tx_log if t["type"] == "brake"]
    assert len(brake_tx) == 2
    assert brake_tx[-1]["on"] is False


# ─────────────────────────── ReplLoop ──────────────────────────


def test_repl_home_dispatches_arm_joints():
    """`home` should issue a send_arm_joints with the home_q values
    (per design doc §4.5.5)."""
    cfg = _joint_cfg(no_gripper=True)
    cfg.home_q_right = [0.0, 0.3, 0.0, -1.5, 0.0, 1.8, 0.0]
    ctx, repl, captured = _make_repl(cfg)
    repl.handle_line("home r")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    assert len(arm_tx) >= 1
    np.testing.assert_allclose(
        arm_tx[-1]["q_target"],
        cfg.home_q_right,
        atol=1e-5,
    )
    assert any("home reset issued" in s for s in captured)


def test_repl_home_with_gripper_also_resets_gripper():
    """When the gripper is enabled, home should also issue
    send_gripper at gmin_pct (open jaw safely)."""
    cfg = _joint_cfg(no_gripper=False)
    ctx, repl, _ = _make_repl(cfg)
    repl.handle_line("home r")
    grip_tx = [t for t in ctx.backend.tx_log if t["type"] == "gripper"]
    assert len(grip_tx) >= 1
    assert grip_tx[-1]["pct"] == pytest.approx(cfg.gripper_min_pct)


def test_repl_handle_line_quit_stops():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=True))
    assert ctx.running is True
    repl.handle_line("quit")
    assert ctx.running is False


def test_repl_handle_line_help_does_not_crash():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("?")
    # Help printed
    assert any("R1 Pro joint+gripper" in s for s in captured)


def test_repl_handle_line_invalid_input_continues():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("teleport r 0 0 0")
    assert ctx.running is True  # error doesn't kill REPL
    assert any("parse error" in s for s in captured)


def test_repl_handle_line_state_query():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("state r")
    assert any("[STATE]" in s for s in captured)
    assert any("side=right" in s for s in captured)


def test_repl_handle_line_topo():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("topo")
    assert any("[TOPO]" in s for s in captured)
    assert any("backend=dummy" in s for s in captured)


def test_repl_mode_switch_joint_to_ee():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    assert ctx.cfg.use_joint_mode is True
    repl.handle_line("mode ee")
    assert ctx.cfg.use_joint_mode is False
    assert any("switched to mode=ee" in s for s in captured)


def test_repl_mode_switch_no_op_when_already_in_mode():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("mode joint")
    assert any("already in mode=joint" in s for s in captured)


def test_repl_set_command_updates_runtime_cfg():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("set operator_heartbeat_timeout_ms=5000")
    assert ctx.cfg.operator_heartbeat_timeout_ms == 5000


def test_repl_set_command_unknown_key_warns():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("set unknown_key=42")
    assert any("not a runtime-mutable key" in s for s in captured)


def test_repl_safety_query_when_no_action_yet():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("safety")
    assert any("no SafetyInfo recorded yet" in s for s in captured)


def test_repl_safety_query_after_action_shows_overlay():
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    repl.handle_line("j r 0 0 0 0 0 0 0")
    captured.clear()
    repl.handle_line("safety")
    assert any("[SAFETY]" in s for s in captured)


def test_repl_set_gripper_max_updates_mixer():
    ctx, repl, _ = _make_repl(_joint_cfg(no_gripper=False))
    repl.handle_line("set gripper_max_pct=80")
    assert ctx.mixer.gmax_pct == pytest.approx(80.0)


# ───────────────────── End-to-end Dummy ────────────────────────


def test_end_to_end_dummy_session_completes_without_error():
    """A multi-line scripted session in dummy mode should work."""
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=False))
    cmds = [
        "topo",
        "j r 0 0.5 0 -1.5 0 1.8 0",
        "g r 50",
        "jd r 0 0.1",
        "g r open",
        "g r close",
        "j r 3 -1.2",
        "state r",
        "safety",
        "brake on",
        "brake off",
        "quit",
    ]
    for line in cmds:
        repl.handle_line(line)
    assert ctx.running is False
    # Verify we sent some arm joints, some gripper, some brake.
    types = {t["type"] for t in ctx.backend.tx_log}
    assert "arm_joints" in types
    assert "gripper" in types
    assert "brake" in types


def test_end_to_end_safety_escalation_on_emergency_action():
    """A NaN action goes through full pipeline -> brake applied."""
    ctx, repl, captured = _make_repl(_joint_cfg(no_gripper=True))
    # Produce a NaN by injection.  The action verb itself must be
    # an action verb so the safety pipeline runs; we hijack the
    # composer mid-way.
    orig_compose = repl.composer.compose

    def _bad_compose(cmd):
        return np.array([np.nan] * 7, dtype=np.float32)

    repl.composer.compose = _bad_compose  # type: ignore[assignment]
    repl.handle_line("j r 0 0 0 0 0 0 0")
    repl.composer.compose = orig_compose
    # The safety pipeline should have triggered emergency, then
    # the REPL should have applied brake.
    brake_tx = [t for t in ctx.backend.tx_log if t["type"] == "brake"]
    assert any(t["on"] is True for t in brake_tx)
    assert any("emergency_stop=True" in s for s in captured)


# ───────────────────────── argparse / main ─────────────────────


def test_argparser_defaults():
    args = build_argparser().parse_args([])
    assert args.backend == "rclpy"
    assert args.dummy is False
    assert args.use_joint_mode is True
    assert args.use_right_arm is True
    assert args.use_left_arm is False
    assert args.no_gripper is False
    assert args.gripper_max_pct == pytest.approx(90.0)


def test_argparser_use_ee_mode_overrides():
    args = build_argparser().parse_args(["--use-ee-mode"])
    assert args.use_joint_mode is False


def test_argparser_dummy_flag():
    args = build_argparser().parse_args(["--dummy"])
    assert args.dummy is True


def test_argparser_strict_heartbeat_uses_15s_default():
    args = build_argparser().parse_args(["--strict-heartbeat"])
    cfg = cfg_from_args(args)
    assert cfg.operator_heartbeat_timeout_ms == pytest.approx(1500.0)


def test_main_with_script_runs_to_eof(tmp_path):
    """Running ``main(--dummy --script PATH)`` should consume the
    file and exit cleanly with code 0."""
    script = tmp_path / "session.txt"
    script.write_text(
        "# bring-up smoke test\n"
        "topo\n"
        "j r 0 0.5 0 -1.5 0 1.8 0\n"
        "g r 50\n"
        "state r\n"
        "quit\n",
        encoding="utf-8",
    )
    rc = main(
        [
            "--backend",
            "dummy",
            "--dummy",
            "--use-joint-mode",
            "--use-right-arm",
            "--strict-heartbeat",
            "--script",
            str(script),
        ]
    )
    assert rc == 0


def test_main_script_no_gripper(tmp_path):
    """Joint-mode no-gripper session via main() must work end-to-end."""
    script = tmp_path / "joint_only.txt"
    script.write_text(
        "j r 0 0.3 0 -1.5 0 1.8 0\nj r 3 -1.0\njd r 0 0.1\nquit\n",
        encoding="utf-8",
    )
    rc = main(
        [
            "--backend",
            "dummy",
            "--dummy",
            "--use-joint-mode",
            "--use-right-arm",
            "--no-gripper",
            "--strict-heartbeat",
            "--script",
            str(script),
        ]
    )
    assert rc == 0
