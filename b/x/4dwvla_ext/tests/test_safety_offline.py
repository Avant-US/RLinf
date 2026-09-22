#!/usr/bin/env python3
"""Offline test: safety logic and gym.Env compliance in dummy mode.

Can run on the host machine (no Docker/robot needed, but needs gymnasium).

    pip install gymnasium numpy
    python /path/to/4dwvla_ext/tests/test_safety_offline.py
"""
import sys
from pathlib import Path

import numpy as np

_ext_dir = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _ext_dir)
sys.path.insert(0, str(Path(_ext_dir).parent))  # b/x — contains franky_ext

from franky_joint_env import (
    FrankyJointEnv, MotionGuardTripped,
    check_action_safety,
    JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER,
    ACTION_LIMIT_LOWER, ACTION_LIMIT_UPPER,
    MAX_JOINT_STEP_RAD, HOME_JOINTS,
    TRAIN_ARM_MIN, TRAIN_ARM_MAX, SAFETY_MARGIN_RAD, TRAIN_EDGE_WARN_RAD,
)

PASS = 0
FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1; print(f"  [PASS] {name}")
    else:
        FAIL += 1; print(f"  [FAIL] {name}: {detail}")


def test_check_action_safety_l1():
    """L1: Hard joint limits clipping."""
    print("\n=== T3.1: L1 Hard Joint Limits ===")
    current = HOME_JOINTS.copy()

    safe_action = current + 0.01
    clipped, warnings = check_action_safety(safe_action, current, 0)
    check("safe action no clip", np.allclose(clipped, safe_action, atol=1e-6))
    check("safe action no warnings", len(warnings) == 0)

    extreme = np.full(7, 10.0)
    clipped, warnings = check_action_safety(extreme, current, 0)
    check("extreme clipped to upper", np.all(clipped <= JOINT_LIMITS_UPPER + 1e-9))
    check("extreme has L1 warning", any("HARD LIMIT" in w for w in warnings))

    extreme_low = np.full(7, -10.0)
    clipped, warnings = check_action_safety(extreme_low, current, 0)
    check("extreme_low clipped to lower", np.all(clipped >= JOINT_LIMITS_LOWER - 1e-9))


def test_check_action_safety_l2():
    """L2: Training range + safety margin clipping."""
    print("\n=== T3.2: L2 Training Range ===")
    current = HOME_JOINTS.copy()

    beyond_train = ACTION_LIMIT_UPPER + 0.05
    beyond_train = np.clip(beyond_train, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
    clipped, warnings = check_action_safety(beyond_train, current, 0)
    check("beyond train has L2 warning", any("OUT-OF-TRAIN" in w for w in warnings))
    check("beyond train clipped", np.all(clipped <= ACTION_LIMIT_UPPER + 1e-9))


def test_check_action_safety_l3():
    """L3: Velocity limiting."""
    print("\n=== T3.3: L3 Velocity Limit ===")
    current = HOME_JOINTS.copy()

    big_step = current + 0.5
    big_step = np.clip(big_step, ACTION_LIMIT_LOWER, ACTION_LIMIT_UPPER)
    clipped, warnings = check_action_safety(big_step, current, 0)
    delta = np.abs(clipped - current)
    check("velocity limited", np.all(delta <= MAX_JOINT_STEP_RAD + 1e-6),
          f"max delta={delta.max():.4f}")
    check("velocity has L3 warning", any("VEL LIMIT" in w for w in warnings))


def test_franky_joint_env_dummy():
    """FrankyJointEnv dummy mode: gym.Env compliance."""
    print("\n=== T3.4: FrankyJointEnv Dummy Mode ===")
    env = FrankyJointEnv(is_dummy=True)

    # Spaces
    check("action_space shape", env.action_space.shape == (8,),
          f"got {env.action_space.shape}")
    check("obs_space has state", "state" in env.observation_space.spaces)
    check("obs state shape", env.observation_space["state"].shape == (8,))

    # Reset
    obs, info = env.reset()
    check("reset obs has state", "state" in obs)
    check("reset obs state shape", obs["state"].shape == (8,),
          f"got {obs['state'].shape}")
    check("reset info is dict", isinstance(info, dict))

    # Step
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    check("step obs has state", "state" in obs)
    check("step reward is float", isinstance(reward, (int, float)))
    check("step terminated bool", isinstance(terminated, bool))
    check("step truncated bool", isinstance(truncated, bool))
    check("step info has warnings", "warnings" in info)
    check("step info has step", "step" in info)

    # Multiple steps
    for i in range(5):
        obs, reward, terminated, truncated, info = env.step(action)
    check("multi step ok", info["step"] == 5, f"got step={info.get('step')}")

    # Camera frames (dummy)
    frames = env.get_camera_frames()
    check("dummy frames global", frames["global"].shape == (480, 640, 3))
    check("dummy frames wrist", frames["wrist"].shape == (480, 640, 3))

    # go_to_rest (dummy, should not raise)
    env.go_to_rest()
    check("go_to_rest dummy ok", True)

    env.close()
    check("close ok", True)


def test_action_space_bounds():
    """Verify action space bounds match safety constants."""
    print("\n=== T3.5: Action Space Bounds ===")
    env = FrankyJointEnv(is_dummy=True)
    low = env.action_space.low
    high = env.action_space.high

    check("low arm = JOINT_LIMITS_LOWER",
          np.allclose(low[:7], JOINT_LIMITS_LOWER))
    check("high arm = JOINT_LIMITS_UPPER",
          np.allclose(high[:7], JOINT_LIMITS_UPPER))
    check("low gripper = 0", low[7] == 0.0)
    check("high gripper = 1", high[7] == 1.0)
    env.close()


def test_home_joints_in_bounds():
    """HOME_JOINTS should be within all safety bounds."""
    print("\n=== T3.6: HOME Joints Validity ===")
    check("HOME within hard limits",
          np.all(HOME_JOINTS >= JOINT_LIMITS_LOWER) and
          np.all(HOME_JOINTS <= JOINT_LIMITS_UPPER))
    check("HOME within training range",
          np.all(HOME_JOINTS >= ACTION_LIMIT_LOWER) and
          np.all(HOME_JOINTS <= ACTION_LIMIT_UPPER))
    check("HOME has 7 joints", HOME_JOINTS.shape == (7,))


def test_q7_has_no_training_margin():
    """Regression: q7 must not be allowed outside the demonstrated range.

    The 600-step run of 2026-09-18 stalled with q7 = 0.4579, which is 0.026 rad
    below TRAIN_ARM_MIN[6] and therefore legal under the old 0.05 rad margin.
    Sweeping q7 against the checkpoint (tests/diag_pose_attractor.py) put the
    sign flip of the predicted action direction exactly at the training bound,
    so that margin permitted the whole inverted region. q7 now gets none.
    """
    print("\n=== T3.8: q7 Training Margin (2026-09-18 stall regression) ===")

    check("q7 margin is zero", SAFETY_MARGIN_RAD[6] == 0.0,
          f"margin={SAFETY_MARGIN_RAD[6]}")
    check("q7 action bound equals the training bound",
          abs(ACTION_LIMIT_LOWER[6] - TRAIN_ARM_MIN[6]) < 1e-9,
          f"bound={ACTION_LIMIT_LOWER[6]}, train_min={TRAIN_ARM_MIN[6]}")

    # The exact pose the arm stalled in must now be clipped back into range.
    stall = np.array([-0.3650, 0.0435, 0.2047, -1.8977, -0.1036, 2.0165, 0.4579])
    clipped, warnings = check_action_safety(stall, stall, 0)
    check("the stalled q7 is clipped up to the training bound",
          clipped[6] >= TRAIN_ARM_MIN[6] - 1e-9,
          f"clipped q7={clipped[6]:.4f}, train_min={TRAIN_ARM_MIN[6]:.4f}")
    check("clipping the stalled pose is reported as OUT-OF-TRAIN",
          any("OUT-OF-TRAIN" in w for w in warnings),
          f"warnings={warnings}")
    # q1 = -0.3650 is inside the full training set's range; only the local
    # 8-episode subset is narrower, so it must not be clipped.
    check("q1 of the stalled pose is left alone",
          abs(clipped[0] - stall[0]) < 1e-9,
          f"clipped q1={clipped[0]:.4f} vs {stall[0]:.4f}")

    # Sitting just inside the bound is legal but reported.
    edge = HOME_JOINTS.copy()
    edge[6] = TRAIN_ARM_MIN[6] + 0.5 * TRAIN_EDGE_WARN_RAD[6]
    clipped, warnings = check_action_safety(edge, edge, 1)
    check("q7 just inside the bound is not clipped",
          abs(clipped[6] - edge[6]) < 1e-9,
          f"clipped q7={clipped[6]:.4f} vs {edge[6]:.4f}")
    check("q7 just inside the bound raises TRAIN-EDGE",
          any("TRAIN-EDGE" in w for w in warnings),
          f"warnings={warnings}")

    mid = HOME_JOINTS.copy()
    _clipped, warnings = check_action_safety(mid, mid, 2)
    check("HOME raises no TRAIN-EDGE warning",
          not any("TRAIN-EDGE" in w for w in warnings),
          f"warnings={warnings}")
    check("HOME q7 is at least the edge threshold inside the bound",
          HOME_JOINTS[6] - TRAIN_ARM_MIN[6] >= TRAIN_EDGE_WARN_RAD[6],
          f"gap={HOME_JOINTS[6] - TRAIN_ARM_MIN[6]:.4f}")

    # The other six joints keep their margins, so this change cannot have
    # narrowed the reachable set anywhere else.
    for i in range(6):
        check(f"q{i+1} margin unchanged at 0.15", SAFETY_MARGIN_RAD[i] == 0.15,
              f"margin={SAFETY_MARGIN_RAD[i]}")
    check("upper bounds are unchanged for q1-q6",
          np.allclose(ACTION_LIMIT_UPPER[:6],
                      np.minimum(TRAIN_ARM_MAX[:6] + 0.15, JOINT_LIMITS_UPPER[:6])))


def test_motion_guard_tripped_exception():
    """MotionGuardTripped is a RuntimeError subclass."""
    print("\n=== T3.7: MotionGuardTripped Exception ===")
    exc = MotionGuardTripped("test trip")
    check("is RuntimeError", isinstance(exc, RuntimeError))
    check("message correct", str(exc) == "test trip")
    try:
        raise exc
    except RuntimeError:
        check("catchable as RuntimeError", True)


if __name__ == "__main__":
    test_check_action_safety_l1()
    test_check_action_safety_l2()
    test_check_action_safety_l3()
    test_franky_joint_env_dummy()
    test_action_space_bounds()
    test_home_joints_in_bounds()
    test_q7_has_no_training_margin()
    test_motion_guard_tripped_exception()
    print(f"\n=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
