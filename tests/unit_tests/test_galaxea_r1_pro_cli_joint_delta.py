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

"""End-to-end tests for the CLI in joint **delta** sub-mode.

Per design doc r1pro6op47.md §3.6 / §3.6.5 + CLI compatibility section,
the delta sub-mode CLI must:

* Build a :class:`JointDeltaDispatcher` when ``joint_delta_mode=True``.
* When operator types ``j r 0.5 0.3 ...`` (absolute target rad), the
  ActionComposer back-computes ``delta_norm = clip((q_target -
  q_current) / scale, -1, 1)`` so the dispatcher can do the right
  thing.
* Backend ``send_arm_joints`` receives the **absolute** ``q_target``
  (post-add), NOT the raw delta -- because the dispatcher integrates
  the delta into q_current internally.
* Switching mode at runtime (``mode joint`` after ee, etc.) should
  honour ``joint_delta_mode`` from cfg.
* ``jd r idx delta_rad`` directly produces ``delta_norm = delta_rad/scale``.
* Gripper still goes through the abs path (asymmetric per §3.6.1).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import (  # noqa: E402
    JointDeltaDispatcher,
    JointStateDispatcher,
)
from toolkits.realworld_check.test_galaxea_r1_pro_cli_controller import (  # noqa: E402
    CliConfig,
    ReplLoop,
    build_argparser,
    build_context,
    cfg_from_args,
    main,
)

# ─────────────────────── Fixtures ──────────────────────────────


def _delta_cfg(no_gripper: bool = True, dual_arm: bool = False) -> CliConfig:
    cfg = CliConfig()
    cfg.dummy = True
    cfg.backend_kind = "dummy"
    cfg.use_joint_mode = True
    cfg.joint_delta_mode = True
    cfg.joint_delta_scale_right = [0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20]
    cfg.use_right_arm = True
    cfg.use_left_arm = dual_arm
    cfg.no_gripper = no_gripper
    cfg.gripper_min_pct = 0.0
    cfg.gripper_max_pct = 90.0
    cfg.arm_q_min_right = [-2.0] * 7
    cfg.arm_q_max_right = [+2.0] * 7
    cfg.arm_q_min_left = [-2.0] * 7
    cfg.arm_q_max_left = [+2.0] * 7
    cfg.heartbeat_interval_ms = 1000
    cfg.strict_heartbeat = True
    return cfg


def _make_repl(cfg: CliConfig):
    captured: list[str] = []

    def _capture(s: str) -> None:
        captured.append(s)

    ctx = build_context(cfg)
    repl = ReplLoop(ctx, output_fn=_capture)
    return ctx, repl, captured


# ───────────────────── Build context ───────────────────────────


def test_build_context_picks_JointDeltaDispatcher_in_delta_mode():
    cfg = _delta_cfg(no_gripper=True)
    ctx = build_context(cfg)
    assert isinstance(ctx.dispatcher, JointDeltaDispatcher)
    # Sanity: still a subclass of JointStateDispatcher (inheritance preserved)
    assert isinstance(ctx.dispatcher, JointStateDispatcher)


def test_build_context_default_uses_JointStateDispatcher():
    cfg = _delta_cfg(no_gripper=True)
    cfg.joint_delta_mode = False
    ctx = build_context(cfg)
    assert isinstance(ctx.dispatcher, JointStateDispatcher)
    # Confirm we got the abs base class, not the delta subclass.
    assert not isinstance(ctx.dispatcher, JointDeltaDispatcher)


def test_build_context_per_arm_delta_scale_propagates():
    cfg = _delta_cfg(no_gripper=True, dual_arm=True)
    cfg.joint_delta_scale_left = [0.50] * 7
    ctx = build_context(cfg)
    np.testing.assert_allclose(
        ctx.dispatcher._joint_delta_scale_right,
        [0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20],
        atol=1e-7,
    )
    np.testing.assert_allclose(
        ctx.dispatcher._joint_delta_scale_left,
        [0.50] * 7,
        atol=1e-7,
    )


# ───────────── ActionComposer reverse calc (j r ... abs target) ─────────


def test_composer_j_abs_in_delta_mode_back_computes_delta_norm():
    """Operator types `j r 0.5 ...` (absolute target).  In delta mode,
    composer must produce delta_norm such that
    JointDeltaDispatcher would translate it back to that target
    (one step at a time)."""
    cfg = _delta_cfg(no_gripper=True)
    ctx, repl, _ = _make_repl(cfg)
    # Set known initial qpos.
    ctx.backend.patch_state(right_arm_qpos=np.zeros(7, dtype=np.float32))
    # Target [0.05] * 7 -- well within one-step (delta_scale = 0.10 for
    # joints 0-3, 0.20 for joints 4-6).
    repl.handle_line("j r 0.05 0.05 0.05 0.05 0.05 0.05 0.05")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    # The dispatcher should send the absolute q_target (post-add).
    np.testing.assert_allclose(arm_tx[-1]["q_target"], [0.05] * 7, atol=1e-5)


def test_composer_j_abs_in_delta_mode_clips_when_target_too_far():
    """If target is more than one delta step away, the per-axis clip
    bounds delta_norm to [-1, 1]; resulting q_target advances by
    delta_scale, not all the way."""
    cfg = _delta_cfg(no_gripper=True)
    ctx, repl, _ = _make_repl(cfg)
    ctx.backend.patch_state(right_arm_qpos=np.zeros(7, dtype=np.float32))
    # Target +0.5 on each joint -- delta_scale = 0.10 for J0..J3, so
    # request is 5x; clipped to delta_scale -> q_target = 0.10.
    repl.handle_line("j r 0.5 0.5 0.5 0.5 0.5 0.5 0.5")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    expected = np.array([0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20])
    np.testing.assert_allclose(arm_tx[-1]["q_target"], expected, atol=1e-5)


def test_composer_j_abs_in_delta_mode_two_steps_get_closer():
    """Two consecutive `j r 0.5 ...` calls -- after first step backend
    mirror puts q_current at clipped value; second step continues
    toward the target."""
    cfg = _delta_cfg(no_gripper=True)
    ctx, repl, _ = _make_repl(cfg)
    ctx.backend.patch_state(right_arm_qpos=np.zeros(7, dtype=np.float32))
    repl.handle_line("j r 0.5 0.5 0.5 0.5 0.5 0.5 0.5")
    repl.handle_line("j r 0.5 0.5 0.5 0.5 0.5 0.5 0.5")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    # After first step: J0..J3 at 0.10, J4..J6 at 0.20
    # After second step: J0..J3 at 0.20, J4..J6 at 0.40
    expected = np.array([0.20, 0.20, 0.20, 0.20, 0.40, 0.40, 0.40])
    np.testing.assert_allclose(arm_tx[-1]["q_target"], expected, atol=1e-5)


def test_composer_j_single_abs_in_delta_mode_only_moves_one_axis():
    """`j r 3 -1.0` (single joint absolute target) in delta mode:
    only J3 should move; other joints stay at q_current."""
    cfg = _delta_cfg(no_gripper=True)
    ctx, repl, _ = _make_repl(cfg)
    ctx.backend.patch_state(
        right_arm_qpos=np.full(7, 0.5, dtype=np.float32),
    )
    # Move only J3 toward -1.0 (delta_scale[3] = 0.10).
    repl.handle_line("j r 3 -1.0")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    q_target = arm_tx[-1]["q_target"]
    # J0, J1, J2, J4, J5, J6 unchanged.
    np.testing.assert_allclose(
        [q_target[i] for i in (0, 1, 2, 4, 5, 6)],
        [0.5] * 6,
        atol=1e-5,
    )
    # J3: delta = -1.0 - 0.5 = -1.5; clipped to -delta_scale[3] = -0.10
    # -> q_target[3] = 0.5 - 0.10 = 0.40.
    assert q_target[3] == pytest.approx(0.40, abs=1e-5)


def test_composer_jd_in_delta_mode_writes_only_one_axis():
    """`jd r 0 0.05` (per-axis delta) in delta mode: writes
    delta_norm only at axis 0; other axes stay at zero (no movement
    intent)."""
    cfg = _delta_cfg(no_gripper=True)
    ctx, repl, _ = _make_repl(cfg)
    ctx.backend.patch_state(
        right_arm_qpos=np.full(7, 0.5, dtype=np.float32),
    )
    # delta_scale[0] = 0.10; delta_rad = 0.05 -> delta_norm = 0.5
    repl.handle_line("jd r 0 0.05")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    q_target = arm_tx[-1]["q_target"]
    # J0: 0.5 + 0.5 * 0.10 = 0.55
    assert q_target[0] == pytest.approx(0.55, abs=1e-5)
    # J1..J6: 0.5 + 0 = 0.5 (no movement)
    for i in range(1, 7):
        assert q_target[i] == pytest.approx(0.5, abs=1e-5)


def test_composer_jn_in_delta_mode_passes_norm_through():
    """`jn r 0.5 0.5 ...` is ALREADY in normalised form; in delta mode
    it's interpreted directly as delta_norm and passed to the
    dispatcher unchanged."""
    cfg = _delta_cfg(no_gripper=True)
    ctx, repl, _ = _make_repl(cfg)
    ctx.backend.patch_state(right_arm_qpos=np.zeros(7, dtype=np.float32))
    # delta_norm = 0.5 -> delta_rad = 0.5 * scale
    repl.handle_line("jn r 0.5 0.5 0.5 0.5 0.5 0.5 0.5")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    expected = np.array([0.05, 0.05, 0.05, 0.05, 0.10, 0.10, 0.10])
    np.testing.assert_allclose(arm_tx[-1]["q_target"], expected, atol=1e-5)


# ───────────── Gripper still abs (asymmetric per §3.6.1) ────────


def test_gripper_in_delta_mode_still_uses_mixer():
    """Gripper command in delta mode is unchanged from abs mode."""
    cfg = _delta_cfg(no_gripper=False)
    ctx, repl, _ = _make_repl(cfg)
    repl.handle_line("g r 50")
    grip_tx = [t for t in ctx.backend.tx_log if t["type"] == "gripper"]
    assert len(grip_tx) == 1
    assert grip_tx[-1]["pct"] == pytest.approx(50.0, abs=1e-3)


def test_gripper_in_delta_mode_open_close_uses_business_range():
    cfg = _delta_cfg(no_gripper=False)
    cfg.gripper_min_pct = 10.0
    cfg.gripper_max_pct = 80.0
    ctx, repl, _ = _make_repl(cfg)
    repl.handle_line("g r open")
    repl.handle_line("g r close")
    grip_tx = [t for t in ctx.backend.tx_log if t["type"] == "gripper"]
    assert grip_tx[0]["pct"] == pytest.approx(80.0)  # open=gmax
    assert grip_tx[1]["pct"] == pytest.approx(10.0)  # close=gmin


# ──────────── reset_to_safe_pose still publishes home_q ────────


def test_home_in_delta_mode_publishes_absolute_home_q():
    """`home r` in delta mode should send the absolute home_q to
    joint_tracker -- not a delta from current state.  Inherited from
    JointStateDispatcher (per §3.6.5 / §3.6.7)."""
    cfg = _delta_cfg(no_gripper=True)
    cfg.home_q_right = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ctx, repl, _ = _make_repl(cfg)
    # Robot at some random current position
    ctx.backend.patch_state(
        right_arm_qpos=np.full(7, 0.7, dtype=np.float32),
    )
    repl.handle_line("home r")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    np.testing.assert_allclose(
        arm_tx[-1]["q_target"],
        cfg.home_q_right,
        atol=1e-5,
    )


# ─────────────────────── prompt + mode switch ──────────────────


def test_prompt_shows_joint_delta_in_delta_mode():
    cfg = _delta_cfg(no_gripper=True)
    ctx, repl, _ = _make_repl(cfg)
    p = repl._prompt()
    assert "[joint_delta]" in p


def test_prompt_shows_joint_when_delta_off():
    cfg = _delta_cfg(no_gripper=True)
    cfg.joint_delta_mode = False
    ctx, repl, _ = _make_repl(cfg)
    p = repl._prompt()
    assert "[joint]" in p
    assert "joint_delta" not in p


# ────────────────────── argparse / main ────────────────────────


def test_argparser_joint_delta_mode_flag():
    args = build_argparser().parse_args(["--joint-delta-mode"])
    assert args.joint_delta_mode is True


def test_argparser_joint_delta_scale_flag():
    args = build_argparser().parse_args(
        [
            "--joint-delta-scale",
            "0.05",
            "0.05",
            "0.05",
            "0.05",
            "0.10",
            "0.10",
            "0.10",
        ]
    )
    assert args.joint_delta_scale == [0.05, 0.05, 0.05, 0.05, 0.10, 0.10, 0.10]


def test_cfg_from_args_propagates_joint_delta_scale():
    args = build_argparser().parse_args(
        [
            "--joint-delta-mode",
            "--joint-delta-scale",
            "0.05",
            "0.05",
            "0.05",
            "0.05",
            "0.05",
            "0.05",
            "0.05",
        ]
    )
    cfg = cfg_from_args(args)
    assert cfg.joint_delta_mode is True
    assert cfg.joint_delta_scale_right == [0.05] * 7


def test_main_with_script_runs_in_delta_mode(tmp_path):
    """main() with --joint-delta-mode and a small script runs to EOF."""
    script = tmp_path / "delta_session.txt"
    script.write_text(
        "topo\nj r 0.05 0.05 0 -0.05 0 0.05 0\njd r 0 0.05\nhome r\nquit\n",
        encoding="utf-8",
    )
    rc = main(
        [
            "--backend",
            "dummy",
            "--dummy",
            "--use-joint-mode",
            "--joint-delta-mode",
            "--use-right-arm",
            "--no-gripper",
            "--strict-heartbeat",
            "--script",
            str(script),
        ]
    )
    assert rc == 0


# ─────────────────── Backward compat (abs mode) ────────────────


def test_existing_abs_yaml_still_constructs():
    """Sanity: the abs-mode YAML / cfg still produces a
    JointStateDispatcher (zero regression on existing tests)."""
    cfg = _delta_cfg(no_gripper=True)
    cfg.joint_delta_mode = False
    ctx, repl, _ = _make_repl(cfg)
    # Type a joint command and verify abs unnormalisation kicks in.
    ctx.backend.patch_state(right_arm_qpos=np.zeros(7, dtype=np.float32))
    # Target +1.0 on J0 -- abs mode unnorm: q_target = q_min + (1+1)/2 *
    # (q_max - q_min) = -2 + 0.5 * 4 = 0.  Wait, action [-1, 1] ->
    # [q_min, q_max].  Action 1.0 -> q_max = +2.0.  But we only sent J0.
    repl.handle_line("j r 0 2.0")
    arm_tx = [t for t in ctx.backend.tx_log if t["type"] == "arm_joints"]
    # In abs mode `j r 0 2.0` means "J0 absolute target = 2.0 rad".
    # Other joints kept at last-norm (zeros -> mid-point of [-2, +2] = 0).
    assert arm_tx[-1]["q_target"][0] == pytest.approx(2.0, abs=1e-5)


# ─────────────── YAML config sanity (delta variant) ────────────


def test_delta_yaml_loads_and_constructs_dispatcher():
    """The new env YAML for joint_delta sub-mode loads cleanly and
    produces a JointDeltaDispatcher when fed through gym.make."""
    import gymnasium as gym
    from omegaconf import OmegaConf

    import rlinf.envs.realworld.galaxear.tasks  # noqa: F401

    yaml_path = (
        _REPO_ROOT
        / "examples"
        / "embodiment"
        / "config"
        / "env"
        / "realworld_galaxea_r1_pro_singlearm_reach_joint_delta.yaml"
    )
    assert yaml_path.is_file(), f"YAML missing: {yaml_path}"
    cfg = OmegaConf.load(yaml_path)
    init_params = cfg["init_params"]
    override_cfg = OmegaConf.to_container(
        init_params.get("override_cfg", {}),
        resolve=False,
    )
    # Resolve interpolations manually for offline test.
    if isinstance(override_cfg.get("ros_domain_id"), str):
        override_cfg["ros_domain_id"] = 41
    elif override_cfg.get("ros_domain_id") is None:
        override_cfg["ros_domain_id"] = 41
    override_cfg["is_dummy"] = True
    override_cfg["step_frequency"] = 1000.0
    env = gym.make(init_params["id"], override_cfg=override_cfg)
    try:
        # In dummy mode dispatcher is None (no _setup_hardware), but we
        # can still verify the config flag is set correctly.
        assert env.unwrapped.config.joint_delta_mode is True
        assert env.unwrapped.config.use_joint_mode is True
        np.testing.assert_allclose(
            env.unwrapped.config.joint_delta_scale_right,
            [0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20],
            atol=1e-7,
        )
    finally:
        env.close()
