#!/usr/bin/env python3
"""Offline checks for joint-impedance command generation. No robot, no franky."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_PKG = Path(__file__).resolve().parents[1]
_BX = _PKG.parent
_EXT = _BX / "4dwvla_ext"
for _path in (_BX, _EXT, _PKG):
    sys.path.insert(0, str(_path))

from franky_controller_direct import JOINT_VEL_LIMITS  # noqa: E402
from impedance_gains import (  # noqa: E402
    DEFAULT_JOINT_DAMPING,
    DEFAULT_JOINT_STIFFNESS,
    joint_velocity_feedforward,
    parse_gain_vector,
)
from impedance_joint_env import limit_equilibrium_step  # noqa: E402
from joint_impedance_controller import JointImpedanceController  # noqa: E402

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}: {detail}")


class _Target:
    def __init__(self):
        self.calls = []

    def set_target(self, q, dq=None):
        self.calls.append((np.asarray(q, dtype=np.float64).copy(), None if dq is None else np.asarray(dq).copy()))

    def stop(self):
        self.calls.append(("stop", None))


class _Robot:
    def __init__(self):
        self.moves = 0

    def move(self, _motion):
        self.moves += 1

    def stop(self):
        return None

    def recover_from_errors(self):
        return None


def test_parse_defaults_and_rejects_bad_width():
    check("empty uses default stiffness", np.allclose(
        parse_gain_vector(None, DEFAULT_JOINT_STIFFNESS, "K"), DEFAULT_JOINT_STIFFNESS
    ))
    parsed = parse_gain_vector(
        "1,2,3,4,5,6,7", DEFAULT_JOINT_DAMPING, "D"
    )
    check("seven values parse", np.allclose(parsed, np.arange(1, 8)))
    try:
        parse_gain_vector("1,2,3", DEFAULT_JOINT_DAMPING, "D")
        check("reject short vector", False, "no error")
    except ValueError:
        check("reject short vector", True)
    try:
        parse_gain_vector("1,2,3,4,5,6,-1", DEFAULT_JOINT_DAMPING, "D")
        check("reject negative", False, "no error")
    except ValueError:
        check("reject negative", True)


def test_feedforward_first_sample_and_clip():
    q = np.zeros(7)
    check(
        "first target has no dq",
        joint_velocity_feedforward(q, None, None, 1.0, JOINT_VEL_LIMITS) is None,
    )
    prev = np.zeros(7)
    target = np.full(7, 10.0)
    dq = joint_velocity_feedforward(target, prev, 0.0, 0.01, JOINT_VEL_LIMITS)
    check("dq is clipped to vel limits", dq is not None and np.allclose(dq, JOINT_VEL_LIMITS))


def _bare_controller():
    ctrl = JointImpedanceController.__new__(JointImpedanceController)
    ctrl._robot = _Robot()
    ctrl._tracker = None
    ctrl._prev_target_q = None
    ctrl._prev_target_ts = None
    ctrl._stiffness = DEFAULT_JOINT_STIFFNESS.copy()
    ctrl._damping = DEFAULT_JOINT_DAMPING.copy()
    ctrl._compensate_coriolis = True
    ctrl.guard_tripped = lambda: None
    target = _Target()
    ctrl._build_tracker = lambda: target
    return ctrl, target


def test_move_updates_tracker_and_does_not_call_robot_move():
    ctrl, target = _bare_controller()
    q = np.array([0.1, 0.0, 0.0, -1.5, 0.0, 1.5, 0.6])
    ctrl.move_joints(q)
    check("tracker created", ctrl._tracker is target)
    check("one set_target", len(target.calls) == 1)
    check("first target has no dq", target.calls[0][1] is None)
    check("robot.move not used", ctrl._robot.moves == 0)
    ctrl.move_joints(q + 0.01)
    check("second target has dq", target.calls[1][1] is not None)
    check("still no robot.move", ctrl._robot.moves == 0)


def test_reset_stops_tracker_before_waypoint():
    ctrl, target = _bare_controller()
    ctrl.move_joints(np.zeros(7))
    ctrl._robot.join_motion = lambda: None
    ctrl._robot.recover_from_errors = lambda: None
    order = []
    original = JointImpedanceController.__mro__[1].reset_joint

    def _waypoint(self, pos):
        order.append(self._tracker is None)
        check("waypoint length", len(pos) == 7)

    JointImpedanceController.__mro__[1].reset_joint = _waypoint
    try:
        ctrl.reset_joint([0.0] * 7)
    finally:
        JointImpedanceController.__mro__[1].reset_joint = original
    check("reset stopped tracker before waypoint", order == [True])
    check(
        "tracker.stop was called",
        any(isinstance(item[0], str) and item[0] == "stop" for item in target.calls),
    )


def test_wrist_gains_and_step_cap():
    check("q1 stiffness stays at the FrankyController value",
          DEFAULT_JOINT_STIFFNESS[0] == 103.75)
    check("q7 stiffness raised to 20.5", DEFAULT_JOINT_STIFFNESS[6] == 20.5)
    check("q7 damping raised to 2.66", DEFAULT_JOINT_DAMPING[6] == 2.66)
    current = np.zeros(7)
    target = np.array([0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    limited, scaled = limit_equilibrium_step(target, current)
    check("0.2 rad step is scaled", scaled)
    check(
        "scaled step is 0.05 rad",
        np.isclose(limited[0], 0.05) and np.allclose(limited[1:], 0.0),
        f"got {limited}",
    )
    small, scaled_small = limit_equilibrium_step(np.full(7, 0.01), current)
    check("0.01 rad step is unchanged", not scaled_small and np.allclose(small, 0.01))


def test_gripper_command_does_not_stack():
    ctrl, _target = _bare_controller()
    submitted = []

    class _Future:
        def done(self):
            return False

    class _Pool:
        def submit(self, fn):
            submitted.append(fn)
            return _Future()

    ctrl._gripper_future = _Future()
    ctrl._gripper_pool = _Pool()
    started = ctrl._submit_gripper("grasp", lambda: None)
    check("busy gripper is not started again", started is False and submitted == [])
    ctrl._gripper_future = None
    started = ctrl._submit_gripper("move_width", lambda: None)
    check("idle gripper starts one command", started is True and len(submitted) == 1)


def test_guard_refuses_target():
    ctrl, _target = _bare_controller()
    ctrl.guard_tripped = lambda: "fence: test"
    try:
        ctrl.move_joints(np.zeros(7))
        check("guard raises", False, "no error")
    except RuntimeError as exc:
        check("guard raises", "tripped" in str(exc))


def main():
    test_parse_defaults_and_rejects_bad_width()
    test_feedforward_first_sample_and_clip()
    test_move_updates_tracker_and_does_not_call_robot_move()
    test_reset_stops_tracker_before_waypoint()
    test_wrist_gains_and_step_cap()
    test_gripper_command_does_not_stack()
    test_guard_refuses_target()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
