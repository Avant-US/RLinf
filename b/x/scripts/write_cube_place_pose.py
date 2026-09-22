#!/usr/bin/env python3
"""Write H1 ``target_ee_pose`` for cube-place phase 2.

Usage (inside franky container, after ``getpos_euler``):

    python b/x/scripts/write_cube_place_pose.py 0.706 0.036 0.232 -3.116 0.026 0.179

This file is the geometric anchor for everything downstream: the success region,
the hover pose, the safety box, the motion guard's fence and its orientation
anchor are all derived from it. So the checks here **refuse** rather than warn, and
``calibrated: true`` is only written for values that could plausibly have come off
the robot.

The previous version printed warnings to stderr and then wrote
``calibrated: true`` regardless -- and that flag is the only thing
``load_target_pose(require_calibrated=True)`` checks besides "6 floats, not all
zero". It therefore asserted "a human read this off the arm" for numbers that had
never been near one.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import sys

import yaml

# Four "..": the first only strips the filename. Three landed on ``<repo>/b``, so
# DEFAULT_PATH became ``<repo>/b/b/x/configs/...`` and writing the calibration
# without REPO_PATH set would fail on a missing directory (LOG-023 finding 4).
REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

from franky_ext.motion_limits import (  # noqa: E402
    PANDA_MAX_REACH_M,
    reach_radius_m,
)

DEFAULT_PATH = os.path.join(REPO, "b", "x", "configs", "cube_place_target_ee_pose.yaml")

#: Usable shell for a table-top contact pose, as a fraction of maximum reach.
#: Below the lower bound the arm is folded into itself; above the upper bound it is
#: near-singular and the impedance controller loses radial authority.
REACH_FRACTION_RANGE = (0.30, 0.90)

#: Plausible table-top contact heights for this cell, metres.
CONTACT_Z_RANGE = (0.05, 0.50)

#: How far the calibrated roll may be from "gripper pointing down" (+/-pi).
#: A quaternion pasted in place of euler angles lands near 0, which this catches.
ROLL_FROM_PI_TOL_RAD = 0.6


#: argparse's built-in negative-number detector only matches plain decimals
#: (``^-\d+$`` or ``^-\d*\.\d+$``), not scientific notation. ``getpos_euler``
#: (and numpy generally) prints values like ``-3.13747483e+00`` for angles near
#: +/-pi, so a bare ``-3.13747483e+00`` on the command line looks like an
#: unknown option to argparse rather than a negative float, silently shifting
#: every positional argument after it (LOG-025). None of this parser's flags
#: look like negative numbers, so it is safe to widen the matcher to also
#: accept exponents.
_NEGATIVE_NUMBER_RE = re.compile(r"^-\d+\.?\d*(?:[eE][+-]?\d+)?$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Write cube-place H1 target_ee_pose YAML"
    )
    parser._negative_number_matcher = _NEGATIVE_NUMBER_RE
    parser.add_argument("x", type=float, help="TCP x (m)")
    parser.add_argument("y", type=float, help="TCP y (m)")
    parser.add_argument("z", type=float, help="TCP z (m)")
    parser.add_argument("roll", type=float, help="roll (rad, euler xyz)")
    parser.add_argument("pitch", type=float, help="pitch (rad)")
    parser.add_argument("yaw", type=float, help="yaw (rad)")
    parser.add_argument(
        "--path",
        default=DEFAULT_PATH,
        help="YAML path (default: b/x/configs/cube_place_target_ee_pose.yaml)",
    )
    parser.add_argument("--notes", default="", help="Optional operator note")
    parser.add_argument(
        "--force",
        action="store_true",
        help="write even though a check failed. Only for a deliberate, understood "
        "exception -- every downstream safety bound is derived from this pose",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="skip the .bak copy of the previous calibration",
    )
    return parser.parse_args()


def _check(pose: list[float]) -> tuple[list[str], list[str]]:
    """Return ``(errors, warnings)`` for a candidate H1 pose."""
    x, y, z, roll, pitch, yaw = pose
    errors: list[str] = []
    warnings: list[str] = []

    if all(abs(v) < 1e-6 for v in pose):
        errors.append("all zeros: this is not a valid H1 pose")

    r = reach_radius_m([x, y, z])
    frac = r / PANDA_MAX_REACH_M
    if not REACH_FRACTION_RANGE[0] <= frac <= REACH_FRACTION_RANGE[1]:
        errors.append(
            f"shoulder-relative radius {r:.4f} m is {frac:.0%} of the "
            f"{PANDA_MAX_REACH_M} m maximum reach, outside the usable "
            f"{REACH_FRACTION_RANGE[0]:.0%}-{REACH_FRACTION_RANGE[1]:.0%} shell. "
            "Check the argument order (x y z roll pitch yaw) and that these are "
            "metres."
        )
    elif frac > 0.85:
        warnings.append(
            f"radius is {frac:.0%} of maximum reach: the arm is close to full "
            "extension, where the impedance controller has poor radial authority. "
            "Consider moving the mark toward the base."
        )

    if not CONTACT_Z_RANGE[0] <= z <= CONTACT_Z_RANGE[1]:
        errors.append(
            f"z={z:.4f} m is not a plausible table-top contact height "
            f"({CONTACT_Z_RANGE[0]}-{CONTACT_Z_RANGE[1]} m)"
        )

    for name, val in (("roll", roll), ("pitch", pitch), ("yaw", yaw)):
        if abs(val) > math.pi + 1e-6:
            errors.append(
                f"{name}={val:.4f} is outside [-pi, pi]. ``getpos_euler`` returns "
                "radians -- did you paste degrees?"
            )

    # The cube face must be flat on the mark, so the gripper points down and roll
    # is near +/-pi. A quaternion's qx/qy/qz pasted in place of euler angles gives
    # small values near zero, and nothing else in the pipeline would catch it: the
    # safety box and the guard's orientation fence both re-centre on whatever this
    # says, so reset would command a ~180 deg wrist flip with the cube in hand.
    if abs(abs(roll) - math.pi) > ROLL_FROM_PI_TOL_RAD:
        errors.append(
            f"roll={roll:.4f} is {abs(abs(roll) - math.pi):.3f} rad from +/-pi, i.e. "
            "the gripper is not pointing down. If you pasted a quaternion, use "
            "`getpos_euler` (6 numbers) and not `getpos` (7 numbers)."
        )
    if abs(pitch) > 0.5:
        warnings.append(f"pitch={pitch:.4f} is far from level; is the cube face flat?")

    return errors, warnings


def main() -> int:
    args = parse_args()
    pose = [args.x, args.y, args.z, args.roll, args.pitch, args.yaw]
    errors, warnings = _check(pose)

    for msg in warnings:
        print(f"WARNING: {msg}", file=sys.stderr)
    for msg in errors:
        print(f"ERROR: {msg}", file=sys.stderr)

    if errors and not args.force:
        print(
            "\nRefusing to write. Every downstream safety bound (success region, "
            "hover, safety box, motion-guard fence and orientation anchor) is "
            "derived from this pose. Re-read it with `getpos_euler` while the cube "
            "is closed in the gripper and pressed against the mark. Pass --force "
            "only if you know why a check is wrong.",
            file=sys.stderr,
        )
        return 2

    payload = {
        # Only claim calibration when the pose could plausibly have come off the
        # robot. With --force over a failed check, say so instead of lying.
        "calibrated": not errors,
        "target_ee_pose": pose,
        "notes": args.notes,
    }
    if errors:
        payload["forced_despite_errors"] = errors

    os.makedirs(os.path.dirname(args.path), exist_ok=True)
    if os.path.exists(args.path) and not args.no_backup:
        backup = args.path + ".bak"
        shutil.copy2(args.path, backup)
        print("backed up previous calibration to", backup)

    with open(args.path, "w", encoding="utf-8") as handle:
        handle.write(
            "# Auto-written by write_cube_place_pose.py. "
            "Do not use all-zero placeholders for robot reset.\n"
        )
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)

    print("wrote", args.path)
    print(f"calibrated: {str(payload['calibrated']).lower()}")
    print("target_ee_pose:", [f"{v:.6f}" for v in pose])
    if errors:
        print(
            "NOTE: written with --force over failed checks, so calibrated is false "
            "and reset/box will still refuse. Fix the pose.",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
