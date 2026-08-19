#!/usr/bin/env python3
"""Write H1 ``target_ee_pose`` for cube-place phase 2.

Usage (inside franky container, after ``getpos_euler``):

    python b/x/scripts/write_cube_place_pose.py 0.51 0.02 0.12 3.14 0.0 0.05
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml

REPO = os.environ.get("REPO_PATH", os.path.abspath(os.path.join(__file__, "../../..")))
DEFAULT_PATH = os.path.join(REPO, "b", "x", "configs", "cube_place_target_ee_pose.yaml")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Write cube-place H1 target_ee_pose YAML"
    )
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
    return parser.parse_args()


def _sanity(pose: list[float]) -> list[str]:
    warnings: list[str] = []
    x, y, z, roll, pitch, yaw = pose
    if abs(x) > 1.0 or abs(y) > 1.0:
        warnings.append(f"xy looks outside Panda reach: x={x:.4f} y={y:.4f}")
    if z < 0.02 or z > 0.55:
        warnings.append(f"z={z:.4f} m looks unlike a table-top contact TCP")
    for name, val in (("roll", roll), ("pitch", pitch), ("yaw", yaw)):
        if abs(val) > 3.5:
            warnings.append(f"{name}={val:.4f} rad; getpos_euler should be ~[-pi, pi]")
    if all(abs(v) < 1e-6 for v in pose):
        warnings.append("all zeros: this is not a valid H1 pose")
    return warnings


def main() -> int:
    args = parse_args()
    pose = [args.x, args.y, args.z, args.roll, args.pitch, args.yaw]
    warnings = _sanity(pose)
    for msg in warnings:
        print(f"WARNING: {msg}", file=sys.stderr)

    payload = {
        "calibrated": True,
        "target_ee_pose": pose,
        "notes": args.notes,
    }
    os.makedirs(os.path.dirname(args.path), exist_ok=True)
    with open(args.path, "w", encoding="utf-8") as handle:
        handle.write(
            "# Auto-written by write_cube_place_pose.py. "
            "Do not use all-zero placeholders for robot reset.\n"
        )
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)

    print("wrote", args.path)
    print("calibrated: true")
    print("target_ee_pose:", [f"{v:.6f}" for v in pose])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
