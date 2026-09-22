#!/usr/bin/env python3
"""Offline test: observation state correctness in dummy mode.

Verifies that obs["state"][7] reflects the true gripper width (0.078 in dummy
mode) rather than the old bug value (0.04) that resulted from Python's falsy
``0.0 or 0.04`` pattern.

Run inside the GPU container with the 4dwvla venv activated.

    source /opt/venv/4dwvla/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/tests/test_obs_state_offline.py
"""
import sys
from pathlib import Path

import numpy as np

_ext_dir = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _ext_dir)
sys.path.insert(0, str(Path(_ext_dir).parent))  # b/x -- contains franky_ext

from franky_joint_env import FrankyJointEnv, HOME_JOINTS

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


# ---- T6: Gripper state observation ----------------------------------------

def test_obs_state_gripper_width():
    """After reset, obs['state'][7] must be 0.078 (dummy gripper width), NOT 0.04."""
    print("\n=== T6.1: Gripper Width in Observation State ===")
    env = FrankyJointEnv(is_dummy=True)
    obs, info = env.reset()

    state = obs["state"]

    # Shape check
    check("state shape is (8,)", state.shape == (8,), f"got {state.shape}")

    # All values finite
    check("all state values finite", np.all(np.isfinite(state)),
          f"non-finite at indices {np.where(~np.isfinite(state))[0].tolist()}")

    # The critical check: gripper width is 0.078, not the bug value 0.04
    gripper_val = float(state[7])
    check(
        f"state[7] == 0.078 (true gripper width), got {gripper_val:.4f}",
        abs(gripper_val - 0.078) < 1e-6,
        f"state[7]={gripper_val}, expected 0.078 (bug value was 0.04)",
    )
    check(
        f"state[7] != 0.04 (the old bug value)",
        abs(gripper_val - 0.04) > 1e-6,
        f"state[7]={gripper_val}, this IS the old bug value!",
    )

    # Arm portion matches HOME_JOINTS
    check(
        "state[:7] matches HOME_JOINTS",
        np.allclose(state[:7], HOME_JOINTS, atol=1e-6),
        f"max diff={np.abs(state[:7] - HOME_JOINTS).max():.6f}",
    )

    # Verify after a step as well
    action = env.action_space.sample()
    obs_step, _, _, _, _ = env.step(action)
    state_step = obs_step["state"]
    check(
        "state[7] == 0.078 after step too",
        abs(float(state_step[7]) - 0.078) < 1e-6,
        f"got {float(state_step[7]):.4f}",
    )

    env.close()


def test_python_falsy_behavior():
    """Document the Python falsy behavior that caused the original bug.

    The old code used ``g = gripper_width or 0.04``, which silently replaces
    a legitimate 0.0 width with the fallback. The fix uses an explicit
    None check: ``g if g is not None else ...``.
    """
    print("\n=== T6.2: Python Falsy Behavior (bug documentation) ===")

    # The bug: ``0.0 or 0.04`` evaluates to 0.04 because 0.0 is falsy.
    bug_result = 0.0 or 0.04
    check(
        "0.0 or 0.04 == 0.04 (the bug pattern)",
        bug_result == 0.04,
        f"got {bug_result}",
    )

    # Normal truthy values pass through correctly (masking the bug for non-zero)
    normal_result = 0.065 or 0.04
    check(
        "0.065 or 0.04 == 0.065 (non-zero passes through, bug hidden)",
        normal_result == 0.065,
        f"got {normal_result}",
    )

    # The fix: explicit None check preserves 0.0
    g = 0.0
    fixed_result = g if g is not None else 0.04
    check(
        "explicit None check: g=0.0, result=0.0 (not 0.04)",
        fixed_result == 0.0,
        f"got {fixed_result}",
    )

    # None correctly triggers the fallback
    g_none = None
    fallback_result = g_none if g_none is not None else 0.04
    check(
        "explicit None check: g=None, result=0.04 (fallback)",
        fallback_result == 0.04,
        f"got {fallback_result}",
    )

    # Other falsy values that would be wrong with ``or``
    for falsy_val, label in [(0, "int 0"), (0.0, "float 0.0"), (False, "False")]:
        or_result = falsy_val or 0.04
        none_result = falsy_val if falsy_val is not None else 0.04
        check(
            f"'{label} or 0.04' gives 0.04 (buggy): {or_result == 0.04}",
            or_result == 0.04,
            f"got {or_result}",
        )
        check(
            f"'{label} if not None' preserves value: {none_result == falsy_val}",
            none_result == falsy_val,
            f"got {none_result}",
        )


def main():
    test_obs_state_gripper_width()
    test_python_falsy_behavior()
    print(f"\n=== Obs State Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)


if __name__ == "__main__":
    main()
