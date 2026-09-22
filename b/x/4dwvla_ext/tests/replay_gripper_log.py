#!/usr/bin/env python3
"""Replay a recorded client log through the current gripper decision logic.

Answers one question about a real run: how many blocking ``move_width`` calls
would today's code have issued on the exact ``(action, w_meas)`` sequence the
robot saw? The 2026-09-18 run issued one per control step, which is what
dragged the loop from 3.58 Hz to 2.28 Hz (grperr_1.2.md Q1).

    python b/x/4dwvla_ext/tests/replay_gripper_log.py \
        b/x/4dwvla_ext/logs/client_20260918_075356_2197.log

Pass ``--reported-max`` to model a hand that reports a different full-open
width, e.g. after re-homing with dsplug/gripper_homing.py.
"""
import argparse
import re
import sys
from pathlib import Path

_ext_dir = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _ext_dir)
sys.path.insert(0, str(Path(_ext_dir).parent))  # b/x -- contains franky_ext

from franky_joint_env import (  # noqa: E402
    continuous_gripper_decision,
    resolve_gripper_max_width_m,
    GRIPPER_MAX_WIDTH_M,
)

LINE_RE = re.compile(
    r"\[step (\d+)\] gripper continuous (\w+) action=([-\d.]+) "
    r"w_meas=([-\d.eE]+) w_cmd=([-\d.]+)"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument(
        "--reported-max",
        type=float,
        default=None,
        help="full-open width the hand reports; defaults to the maximum "
        "w_meas found in the log",
    )
    args = parser.parse_args()

    records = []
    for line in args.log.read_text(encoding="utf-8", errors="replace").splitlines():
        m = LINE_RE.search(line)
        if m:
            records.append(
                (int(m.group(1)), m.group(2), float(m.group(3)), float(m.group(4)))
            )
    if not records:
        print(f"no 'gripper continuous' lines found in {args.log}")
        return 1

    reported_max = args.reported_max
    if reported_max is None:
        reported_max = max(w for _, _, _, w in records)
    ceiling = resolve_gripper_max_width_m(GRIPPER_MAX_WIDTH_M, reported_max)

    before = sum(1 for _, cmd, _, _ in records if cmd == "move_width")
    counts = {"move_width": 0, "grasp_handoff": 0, "hold": 0}
    last_cmd_w = None
    for _step, _cmd, action, w_meas in records:
        cmd, w_cmd = continuous_gripper_decision(action, w_meas, ceiling, last_cmd_w)
        counts[cmd] += 1
        if cmd == "move_width":
            last_cmd_w = w_cmd
        elif cmd == "grasp_handoff":
            last_cmd_w = None

    print(f"log:             {args.log.name}")
    print(f"steps replayed:  {len(records)}")
    print(f"w_meas range:    {min(w for *_, w in records):.5f} .. {reported_max:.5f} m")
    print(f"action range:    {min(a for *_, a, _ in records):.4f} .. "
          f"{max(a for *_, a, _ in records):.4f}")
    print(f"w_cmd ceiling:   {ceiling:.5f} m "
          f"(configured {GRIPPER_MAX_WIDTH_M:.5f}, hand reports {reported_max:.5f})")
    print()
    print(f"blocking move_width calls, as recorded: {before}")
    print(f"blocking move_width calls, replayed:    {counts['move_width']}")
    print(f"  grasp_handoff: {counts['grasp_handoff']}   hold: {counts['hold']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
