#!/usr/bin/env python3
"""Offline test: ExecutedStateBuffer (修复 A) and the tightened q7 margin (修复 C).

Covers the two grperr_1.md fixes that need no hardware/GPU:
  - state_history_buffer.ExecutedStateBuffer record/drain/clear semantics.
  - franky_joint_env's per-joint SAFETY_MARGIN_RAD now flags the exact q7
    drift observed in the 2026-09-18 02:18 client log
    (b/x/4dwvla_ext/logs/client_20260918_021841_2745.log) as OUT-OF-TRAIN,
    which the old uniform 0.15 rad margin let through silently.

Can run on the host machine (no Docker/robot needed, but needs gymnasium
+ numpy, same as test_safety_offline.py).

    python /path/to/4dwvla_ext/tests/test_state_history_offline.py
"""
import sys
from pathlib import Path

import numpy as np

_ext_dir = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _ext_dir)
sys.path.insert(0, str(Path(_ext_dir).parent))  # b/x — contains franky_ext

from franky_joint_env import (  # noqa: E402
    ACTION_LIMIT_LOWER, ACTION_LIMIT_UPPER, HOME_JOINTS, check_action_safety,
)
from state_history_buffer import ExecutedStateBuffer  # noqa: E402

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


def test_buffer_record_drain():
    print("\n=== T_SH.1: ExecutedStateBuffer record/drain ===")
    buf = ExecutedStateBuffer()
    check("empty on construction", len(buf) == 0, f"len={len(buf)}")

    for i in range(10):
        buf.record(np.full(7, float(i)))
    check("len == n_exec after 10 records", len(buf) == 10, f"len={len(buf)}")

    drained = buf.drain()
    check("drain returns 10 poses", len(drained) == 10, f"got {len(drained)}")
    check("each pose has 7 floats", all(len(p) == 7 for p in drained))
    check("chronological order preserved",
          [p[0] for p in drained] == list(range(10)))
    check("buffer empty after drain", len(buf) == 0, f"len={len(buf)}")

    # Second drain (nothing recorded since) must be a no-op, not stale data.
    check("second drain is empty", buf.drain() == [])


def test_buffer_clear():
    print("\n=== T_SH.2: ExecutedStateBuffer clear (episode boundary) ===")
    buf = ExecutedStateBuffer()
    for i in range(5):
        buf.record(np.full(7, float(i)))
    buf.clear()
    check("clear empties buffer", len(buf) == 0, f"len={len(buf)}")
    check("drain after clear is empty", buf.drain() == [])


def test_buffer_accepts_various_input_shapes():
    print("\n=== T_SH.3: ExecutedStateBuffer input flexibility ===")
    buf = ExecutedStateBuffer()
    buf.record(list(range(7)))
    buf.record(np.arange(8, dtype=np.float64))  # extra (gripper) dim ignored
    drained = buf.drain()
    check("list input accepted", drained[0] == list(range(7)))
    check("8D input truncated to 7", len(drained[1]) == 7, f"got {len(drained[1])}")


def test_q7_margin_flags_observed_ood_run():
    print("\n=== T_SH.4: tightened q7 margin flags the 2026-09-18 failure ===")
    # From client_20260918_021841_2745.log: q7 converged to ~0.40 rad and
    # stayed there for hundreds of steps without a single OUT-OF-TRAIN
    # warning under the old uniform 0.15 rad margin.
    observed_bad_state = np.array(
        [-0.347, 0.216, 0.134, -2.035, 0.015, 2.349, 0.391]
    )
    check(
        "q7 lower bound is the training bound itself (no margin)",
        np.isclose(ACTION_LIMIT_LOWER[6], 0.4843, atol=1e-3),
        f"got {ACTION_LIMIT_LOWER[6]}",
    )
    check(
        "q1-q6 margins unchanged (still 0.15 rad wide)",
        np.isclose(ACTION_LIMIT_UPPER[1] - ACTION_LIMIT_LOWER[1], 0.3120 - (-0.1030) + 0.30, atol=1e-3),
        f"upper-lower[1]={ACTION_LIMIT_UPPER[1] - ACTION_LIMIT_LOWER[1]}",
    )

    clipped, warnings = check_action_safety(observed_bad_state, HOME_JOINTS.copy(), step_idx=0)
    check(
        "observed q7=0.391 now trips OUT-OF-TRAIN",
        any("OUT-OF-TRAIN" in w and "q7" in w for w in warnings),
        f"warnings={warnings}",
    )
    check(
        "clipped q7 pulled back into range",
        clipped[6] >= ACTION_LIMIT_LOWER[6] - 1e-9,
        f"clipped q7={clipped[6]}",
    )


if __name__ == "__main__":
    test_buffer_record_drain()
    test_buffer_clear()
    test_buffer_accepts_various_input_shapes()
    test_q7_margin_flags_observed_ood_run()
    print(f"\n=== T_SH Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
