#!/usr/bin/env python3
"""Offline test: gripper semantics -- ramp identity, delta-w criterion, continuous mode.

Run inside the GPU container with the 4dwvla venv activated.

    source /opt/venv/4dwvla/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/tests/test_gripper_semantics_offline.py
"""
import sys
from pathlib import Path

import numpy as np

_ext_dir = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _ext_dir)
sys.path.insert(0, str(Path(_ext_dir).parent))  # b/x -- contains franky_ext

from franky_joint_env import (
    want_gripper_close_delta,
    continuous_gripper_decision,
    resolve_gripper_max_width_m,
    should_issue_grasp_handoff,
    W0,
    GRIPPER_DEADBAND_M,
    GRIPPER_WIDEN_DEADBAND_M,
    GRIPPER_CLOSE_DELTA_M,
    GRIPPER_OPEN_DELTA_M,
    GRASP_HANDOFF_A,
    GRIPPER_MAX_WIDTH_M,
)

# Width the hand reported throughout the 2026-09-18 600-step run, before any
# re-homing. Used as the realistic "full open" for the regression cases.
REPORTED_MAX_W = 0.0664

PASS = 0
FAIL = 0


def check(name: str, condition: bool, detail: str = ""):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}: {detail}")


# ---- T1: Ramp consistency -------------------------------------------------

def test_ramp_identity():
    """Verify the training identity a = 1 - w(t+1) / 0.08."""
    print("\n=== T1: Ramp Consistency (a = 1 - w_next / W0) ===")

    # Demo-like constraint: when a >= 0.5 (commanding a significant close),
    # the next-step width must be small.
    for a in np.linspace(0.5, 1.0, 20):
        w_next = W0 * (1.0 - a)
        check(
            f"a={a:.3f} -> w_next={w_next:.4f} < 0.045",
            w_next < 0.045,
            f"w_next={w_next:.6f}",
        )

    # Conversely: when w is wide (> 0.062 m), a must be small.
    for w in np.linspace(0.062, W0, 10):
        a_implied = 1.0 - w / W0
        check(
            f"w={w:.4f} -> a={a_implied:.4f} < 0.25",
            a_implied < 0.25,
            f"a_implied={a_implied:.6f}",
        )

    # Exact identity check on known pairs.
    pairs = [
        (0.5, 0.04),
        (0.0075, 0.0794),
        (1.0, 0.0),
    ]
    max_err = 0.0
    for a, w_next in pairs:
        a_recon = 1.0 - w_next / W0
        err = abs(a - a_recon)
        max_err = max(max_err, err)
        check(
            f"identity (a={a}, w_next={w_next}): err={err:.2e}",
            err < 1e-6,
            f"err={err:.2e}",
        )
    check(f"max identity error {max_err:.2e} < 1e-6", max_err < 1e-6)


# ---- T2: Delta-w criterion ------------------------------------------------

def test_delta_w_criterion():
    """Test want_gripper_close_delta with known inputs."""
    print("\n=== T2: Delta-w Criterion (want_gripper_close_delta) ===")

    # Case 1: small action, delta_w below threshold -> NOT close
    # delta_w = 0.0664 - 0.08*(1 - 0.174) = 0.0664 - 0.06608 = +0.00032
    result = want_gripper_close_delta(action_grip=0.174, w_meas=0.0664, currently_closed=False)
    delta_w = 0.0664 - W0 * (1.0 - 0.174)
    check(
        f"a=0.174, w=0.0664: delta_w={delta_w:.5f} < threshold -> NOT close",
        result is False,
        f"got {result}, delta_w={delta_w:.6f}, threshold={GRIPPER_CLOSE_DELTA_M}",
    )

    # Case 2: larger action, delta_w above threshold -> CLOSE
    # delta_w = 0.0664 - 0.08*(1 - 0.227) = 0.0664 - 0.06184 = +0.00456
    result = want_gripper_close_delta(action_grip=0.227, w_meas=0.0664, currently_closed=False)
    delta_w = 0.0664 - W0 * (1.0 - 0.227)
    check(
        f"a=0.227, w=0.0664: delta_w={delta_w:.5f} >= threshold -> CLOSE",
        result is True,
        f"got {result}, delta_w={delta_w:.6f}, threshold={GRIPPER_CLOSE_DELTA_M}",
    )

    # Case 3: Hysteresis -- once closed, small action keeps it closed
    # delta_w = 0.0664 - 0.08*(1 - 0.15) = 0.0664 - 0.068 = -0.0016
    # For currently_closed=True: stays closed if delta_w > -GRIPPER_OPEN_DELTA_M (-0.004)
    result = want_gripper_close_delta(action_grip=0.15, w_meas=0.0664, currently_closed=True)
    delta_w = 0.0664 - W0 * (1.0 - 0.15)
    check(
        f"hysteresis: a=0.15, currently_closed=True, delta_w={delta_w:.5f} > -{GRIPPER_OPEN_DELTA_M} -> KEEP closed",
        result is True,
        f"got {result}, delta_w={delta_w:.6f}, open_threshold={-GRIPPER_OPEN_DELTA_M}",
    )

    # Case 4: negative delta_w (action near zero) -> NOT close
    # delta_w = 0.0664 - 0.08*(1 - 0.036) = 0.0664 - 0.07712 = -0.01072
    result = want_gripper_close_delta(action_grip=0.036, w_meas=0.0664, currently_closed=False)
    delta_w = 0.0664 - W0 * (1.0 - 0.036)
    check(
        f"a=0.036, w=0.0664: delta_w={delta_w:.5f} (negative) -> NOT close",
        result is False,
        f"got {result}, delta_w={delta_w:.6f}",
    )


# ---- T3: Continuous width mapping -----------------------------------------

def test_continuous_width_mapping():
    """Test the continuous-mode w_cmd computation and grasp handoff."""
    print("\n=== T3: Continuous Width Mapping ===")

    # w_cmd = W0 * (1 - a)
    a1 = 0.227
    w_cmd_1 = W0 * (1.0 - a1)
    check(
        f"a={a1}: w_cmd={w_cmd_1:.5f} (expected 0.06184)",
        abs(w_cmd_1 - 0.06184) < 1e-6,
        f"got {w_cmd_1:.8f}",
    )
    check(
        f"a={a1} < GRASP_HANDOFF_A ({GRASP_HANDOFF_A}) -> no grasp handoff",
        a1 < GRASP_HANDOFF_A,
        f"a={a1}, threshold={GRASP_HANDOFF_A}",
    )

    # High action value triggers grasp handoff
    a2 = 0.85
    check(
        f"a={a2} >= GRASP_HANDOFF_A ({GRASP_HANDOFF_A}) -> grasp handoff",
        a2 >= GRASP_HANDOFF_A,
        f"a={a2}, threshold={GRASP_HANDOFF_A}",
    )

    # Ramp tracking: the policy asks for a narrower opening -> issue move_width.
    cmd, w_cmd = continuous_gripper_decision(0.227, REPORTED_MAX_W, REPORTED_MAX_W)
    check(
        f"a=0.227, w_meas={REPORTED_MAX_W}: delta_w=+{REPORTED_MAX_W - w_cmd:.5f} -> move_width",
        cmd == "move_width" and abs(w_cmd - 0.06184) < 1e-6,
        f"got cmd={cmd}, w_cmd={w_cmd:.6f}",
    )

    # Inside the narrowing dead zone: issue nothing.
    cmd, w_cmd = continuous_gripper_decision(0.2255, 0.062, REPORTED_MAX_W)
    check(
        f"a=0.2255, w_meas=0.062: |delta_w|<{GRIPPER_DEADBAND_M} -> hold",
        cmd == "hold",
        f"got cmd={cmd}, w_cmd={w_cmd:.6f}, delta_w={0.062 - w_cmd:+.6f}",
    )

    # Grasp handoff takes priority over the dead zone.
    cmd, _ = continuous_gripper_decision(0.85, 0.020, REPORTED_MAX_W)
    check(
        f"a=0.85 >= GRASP_HANDOFF_A ({GRASP_HANDOFF_A}) -> grasp_handoff",
        cmd == "grasp_handoff",
        f"got cmd={cmd}",
    )

    # A genuine re-open request (fingers well below the commanded width) must
    # still get through, otherwise continuous mode could never release.
    cmd, w_cmd = continuous_gripper_decision(0.10, 0.020, REPORTED_MAX_W)
    check(
        f"a=0.10, w_meas=0.020: delta_w={0.020 - w_cmd:+.5f} <= -{GRIPPER_WIDEN_DEADBAND_M} -> move_width",
        cmd == "move_width",
        f"got cmd={cmd}, w_cmd={w_cmd:.6f}",
    )


# ---- T4: Regression for the 2026-09-18 "buzzing hand" failure -------------

def test_approach_phase_issues_no_command():
    """Regression: approach-phase commands must not fight the hand's limit.

    On 2026-09-18 a 600-step run issued a blocking ``move_width`` on every
    single control step. The policy was commanding ``a ~ 0.03`` ("stay open"),
    which maps to w_cmd = 0.0773 m, while the hand reports 0.0664 m as fully
    open. Two defects combined: ``FRANKA_GRIPPER_MAX_WIDTH_M`` was filled with
    the caliper-measured 0.080 m so the clamp never bit, and the dead-zone test
    used ``abs()`` so a "want wider" residual triggered a command just like a
    "want narrower" one. The fingers buzzed against their limit and the control
    loop fell from 3.58 Hz to 2.28 Hz. See grperr_1.2.md Q1.
    """
    print("\n=== T4: Approach Phase Issues No Command (2026-09-18 regression) ===")

    # The clamp must be taken in the hand's reported units, whatever the
    # operator put in the config.
    effective = resolve_gripper_max_width_m(0.080, REPORTED_MAX_W)
    check(
        f"configured 0.080 + hand reports {REPORTED_MAX_W} -> ceiling {effective:.4f}",
        abs(effective - REPORTED_MAX_W) < 1e-9,
        f"got {effective}",
    )
    check(
        "unknown reported max falls back to the configured value",
        abs(resolve_gripper_max_width_m(0.066, None) - 0.066) < 1e-9,
    )
    check(
        "a bogus reported max (0.0) falls back to the configured value",
        abs(resolve_gripper_max_width_m(0.066, 0.0) - 0.066) < 1e-9,
    )

    # Every approach-phase action seen in that run must now be a no-op.
    for a in (0.0256, 0.0340, 0.0352, 0.05, 0.10, 0.15):
        cmd, w_cmd = continuous_gripper_decision(a, REPORTED_MAX_W, effective)
        check(
            f"a={a:.4f}, w_meas={REPORTED_MAX_W} (hand fully open) -> hold",
            cmd == "hold",
            f"got cmd={cmd}, w_cmd={w_cmd:.6f}, delta_w={REPORTED_MAX_W - w_cmd:+.6f}",
        )

    # Second line of defense. The signed dead zone alone does NOT save us from
    # a mis-filled ceiling: w_cmd = 0.0773 is 10.9 mm wider than the hand's
    # 0.0664, which clears the widening threshold, so the first step still
    # commands an unreachable target. What must not happen is repeating it --
    # position control holds its target, so the command is issued once and then
    # suppressed, which is what turns 600 blocking calls into one.
    cmd, w_cmd = continuous_gripper_decision(0.0340, REPORTED_MAX_W, 0.080)
    check(
        "mis-filled 0.080 ceiling: first step still commands the unreachable target",
        cmd == "move_width",
        f"got cmd={cmd}, w_cmd={w_cmd:.6f}",
    )
    issued = 0
    last_cmd_w = None
    for a in (0.0340, 0.0352, 0.0256, 0.0340, 0.0331, 0.0348):
        cmd, w_cmd = continuous_gripper_decision(a, REPORTED_MAX_W, 0.080, last_cmd_w)
        if cmd == "move_width":
            issued += 1
            last_cmd_w = w_cmd
    check(
        f"mis-filled ceiling over 6 approach steps: {issued} command(s) issued, not 6",
        issued == 1,
        f"issued={issued}",
    )

    # With the ceiling resolved from hardware, not even the first step fires.
    issued = 0
    last_cmd_w = None
    for a in (0.0340, 0.0352, 0.0256, 0.0340, 0.0331, 0.0348):
        cmd, w_cmd = continuous_gripper_decision(a, REPORTED_MAX_W, effective, last_cmd_w)
        if cmd == "move_width":
            issued += 1
            last_cmd_w = w_cmd
    check(
        f"resolved ceiling over the same 6 steps: {issued} command(s) issued",
        issued == 0,
        f"issued={issued}",
    )

    # Deduplication must not stall a real closing ramp: the demonstrations
    # narrow by roughly 1.75 mm per 30 Hz frame, well above the dead zone.
    issued = 0
    last_cmd_w = None
    w = REPORTED_MAX_W
    for a in np.linspace(0.20, 0.60, 12):
        cmd, w_cmd = continuous_gripper_decision(a, w, effective, last_cmd_w)
        if cmd == "move_width":
            issued += 1
            last_cmd_w = w_cmd
            w = w_cmd  # hand tracks the target
    check(
        f"closing ramp: {issued}/12 steps command a move",
        issued == 12,
        f"issued={issued}",
    )

    # And the fix must not suppress the behaviour we actually want.
    cmd, _ = continuous_gripper_decision(0.227, REPORTED_MAX_W, effective)
    check(
        "the observed peak a=0.227 still commands a narrowing move",
        cmd == "move_width",
        f"got cmd={cmd}",
    )


def test_grasp_handoff_not_gated_by_is_open():
    """Regression: a narrowed hand must still receive the force grasp.

    Continuous mode position-tracks the ramp first. At a=0.30 the commanded
    width is 56 mm, so ``gripper_is_open()`` (true only at >= 60 mm) is already
    false, while ``gripper_holding()`` is still false because 56 mm is outside
    the 10±8 mm plug window. The old gate skipped ``close_gripper()`` there.
    """
    print("\n=== T5: Grasp Handoff After the Ramp Narrows ===")

    cmd, w_cmd = continuous_gripper_decision(0.85, 0.056, REPORTED_MAX_W)
    check(
        "a=0.85 at w_meas=56 mm (below the 60 mm is_open line) -> grasp_handoff",
        cmd == "grasp_handoff" and abs(w_cmd - 0.012) < 1e-9,
        f"got cmd={cmd}, w_cmd={w_cmd:.6f}",
    )
    check(
        "not holding and no previous attempt -> issue the grasp",
        should_issue_grasp_handoff(False, now_s=10.0, last_attempt_s=float("-inf"), cooldown_s=1.0),
    )
    check(
        "already inside the plug window -> do not grasp again",
        not should_issue_grasp_handoff(True, now_s=10.0, last_attempt_s=float("-inf"), cooldown_s=1.0),
    )
    check(
        "miss 0.2 s ago is inside the 1 s cooldown -> suppress",
        not should_issue_grasp_handoff(False, now_s=10.2, last_attempt_s=10.0, cooldown_s=1.0),
    )
    check(
        "miss 1.0 s ago -> retry",
        should_issue_grasp_handoff(False, now_s=11.0, last_attempt_s=10.0, cooldown_s=1.0),
    )


def main():
    test_ramp_identity()
    test_delta_w_criterion()
    test_continuous_width_mapping()
    test_approach_phase_issues_no_command()
    test_grasp_handoff_not_gated_by_is_open()
    print(f"\n=== Gripper Semantics Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)


if __name__ == "__main__":
    main()
