#!/usr/bin/env python3
"""Probe extreme poses at training data workspace, joint limit, and safety box edges.

Three independent probe modes (corresponding to different "box" concepts):

  workspace    -- Joint-space: training data min/max joint angles (B2)
  joint-limits -- Joint-space: URDF joint limit edges with margin (B2)
  safety-box   -- Cartesian: safety box (B3) and motion guard fence (B4) corners

Usage (inside Franky container):
    source /opt/venv/franky-0.19.0/bin/activate

    # Dry-run (compute and print, no movement):
    python /workspace/RLinf/b/x/4dwvla_ext/extreme_pose_explorer.py --dry-run

    # Move to training workspace corners:
    python /workspace/RLinf/b/x/4dwvla_ext/extreme_pose_explorer.py \
        --robot-ip 172.16.0.2 --mode workspace --speed-factor 0.03

    # All modes:
    python /workspace/RLinf/b/x/4dwvla_ext/extreme_pose_explorer.py \
        --robot-ip 172.16.0.2 --mode all --speed-factor 0.03
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np

_BX_ROOT = Path(__file__).resolve().parents[1]
if str(_BX_ROOT) not in sys.path:
    sys.path.insert(0, str(_BX_ROOT))

from franky_ext.dsplug.home_pose import load_home_joints  # noqa: E402

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    force=True,
)
logger = logging.getLogger("extreme-pose")

# ── Training data statistics (from abs_stats.json) ──────────────────────────

TRAIN_ARM_MIN  = np.array([-0.4842, -0.1030, -0.2025, -2.2044, -0.2041, 1.5702, 0.4843])
TRAIN_ARM_MAX  = np.array([ 0.0452,  0.3120,  0.4789, -1.5347,  0.0806, 2.4536, 0.9807])

# Training data TCP envelope (from abs_stats.json observation.state.ee_pos)
TRAIN_TCP_MIN  = np.array([0.534, -0.140, 0.178])
TRAIN_TCP_MAX  = np.array([0.602,  0.053, 0.517])
TRAIN_TCP_MEAN = np.array([0.565, -0.035, 0.265])

# FR3v2.1 URDF joint limits
FR3V2_LOWER = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
FR3V2_UPPER = np.array([ 2.9007,  1.8361,  2.9007, -0.1169,  2.8763, 4.6216,  3.0508])

JOINT_NAMES = ["q1", "q2", "q3", "q4", "q5", "q6", "q7"]
HOME = load_home_joints()

# BBox R_pad (B1, informational only)
BBOX_RADIUS = 0.8361
PANDA_SHOULDER_Z_M = 0.333
PANDA_MAX_REACH_M = 0.855

# ── Pose builders ────────────────────────────────────────────────────────────

def build_workspace_corners():
    """B2: Training data min/max joint angles, one joint at a time."""
    corners = []
    for i in range(7):
        for val, label in [(TRAIN_ARM_MIN[i], "train_min"), (TRAIN_ARM_MAX[i], "train_max")]:
            pose = HOME.copy()
            pose[i] = val
            corners.append({
                "name": f"{JOINT_NAMES[i]}@{label} ({val:.4f})",
                "joints": pose,
                "box_type": "B2-workspace",
                "description": f"Joint {i+1} at training data {label}, others at HOME",
            })
    return corners

def build_joint_limit_corners(margin=0.05):
    """B2: URDF joint limit edges (only joints close to training range)."""
    corners = []
    for i in range(7):
        lower_safe = FR3V2_LOWER[i] + margin
        upper_safe = FR3V2_UPPER[i] - margin

        if abs(TRAIN_ARM_MIN[i] - lower_safe) < 1.0:
            pose = HOME.copy()
            pose[i] = lower_safe
            corners.append({
                "name": f"{JOINT_NAMES[i]}@lower_limit ({lower_safe:.4f})",
                "joints": pose,
                "box_type": "B2-joint-limit",
                "description": (
                    f"Joint {i+1} at lower URDF limit + {margin}rad. "
                    f"Train min: {TRAIN_ARM_MIN[i]:.4f}, gap: {abs(TRAIN_ARM_MIN[i]-lower_safe):.3f}"
                ),
            })

        if abs(TRAIN_ARM_MAX[i] - upper_safe) < 1.0:
            pose = HOME.copy()
            pose[i] = upper_safe
            corners.append({
                "name": f"{JOINT_NAMES[i]}@upper_limit ({upper_safe:.4f})",
                "joints": pose,
                "box_type": "B2-joint-limit",
                "description": (
                    f"Joint {i+1} at upper URDF limit - {margin}rad. "
                    f"Train max: {TRAIN_ARM_MAX[i]:.4f}, gap: {abs(TRAIN_ARM_MAX[i]-upper_safe):.3f}"
                ),
            })
    return corners

def build_safety_box_corners(guard_margin=0.05, floor_margin=0.01):
    """B3/B4: Safety box and motion guard fence TCP corners."""
    corners = []
    axis_names = ["X", "Y", "Z"]

    for axis in range(3):
        for extreme, label in [(TRAIN_TCP_MIN[axis], "min"), (TRAIN_TCP_MAX[axis], "max")]:
            tcp = TRAIN_TCP_MEAN.copy()
            tcp[axis] = extreme
            fence = extreme + (guard_margin if label == "max" else -guard_margin)
            if axis == 2 and label == "min":
                fence = extreme - floor_margin
            corners.append({
                "name": f"B3_{axis_names[axis]}_{label} (TCP {extreme:.3f}m)",
                "tcp_target": tcp,
                "joints": None,
                "box_type": "B3-safety-box",
                "description": (
                    f"Safety box face: TCP {axis_names[axis]}={extreme:.4f}m. "
                    f"Guard fence at {fence:.4f}m"
                ),
            })

    # B4 fence corners
    fence_min = TRAIN_TCP_MIN - guard_margin
    fence_min[2] = TRAIN_TCP_MIN[2] - floor_margin
    fence_max = TRAIN_TCP_MAX + guard_margin

    for axis in range(3):
        for extreme, label in [(fence_min[axis], "min"), (fence_max[axis], "max")]:
            tcp = TRAIN_TCP_MEAN.copy()
            tcp[axis] = extreme
            corners.append({
                "name": f"B4_{axis_names[axis]}_{label} (fence {extreme:.3f}m)",
                "tcp_target": tcp,
                "joints": None,
                "box_type": "B4-guard-fence",
                "description": f"Motion guard fence face: TCP {axis_names[axis]}={extreme:.4f}m",
            })

    return corners

# ── Diagnostics ──────────────────────────────────────────────────────────────

def report_pose(joints):
    """Print joint-space diagnostics."""
    margin_lo = joints - FR3V2_LOWER
    margin_hi = FR3V2_UPPER - joints
    min_margin = np.minimum(margin_lo, margin_hi)
    dist = joints - HOME
    logger.info("  Joints: %s", np.round(joints, 4).tolist())
    logger.info("  Min URDF margin: %.4f rad (joint %d)", np.min(min_margin), np.argmin(min_margin)+1)
    logger.info(
        "  Max |delta| from HOME: %.4f rad (joint %d)",
        np.max(np.abs(dist)),
        np.argmax(np.abs(dist)) + 1,
    )

def report_tcp(tcp):
    """Print TCP diagnostics with B1/B6 context."""
    logger.info("  TCP: X=%.4f Y=%.4f Z=%.4f m", *tcp[:3])
    pos_norm = tcp[:3] / BBOX_RADIUS
    logger.info("  B1 norm |pos|: %.4f (info only, NOT used in Mode A)", np.linalg.norm(pos_norm))
    shoulder = np.array([0.0, 0.0, PANDA_SHOULDER_Z_M])
    reach = np.linalg.norm(tcp[:3] - shoulder)
    pct = reach / PANDA_MAX_REACH_M * 100
    tag = "NEAR-SINGULAR" if pct > 88 else "OK"
    logger.info("  B6 reach: %.3fm (%.0f%% of max) %s", reach, pct, tag)

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Franka Extreme Pose Explorer")
    parser.add_argument("--robot-ip", default="172.16.0.2")
    parser.add_argument("--mode", choices=["workspace", "joint-limits", "safety-box", "all"],
                        default="workspace")
    parser.add_argument("--speed-factor", type=float, default=0.03)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--margin", type=float, default=0.05, help="Joint limit margin (rad)")
    parser.add_argument("--guard-margin", type=float, default=0.05, help="Guard margin (m)")
    args = parser.parse_args()

    corners = []
    if args.mode in ("workspace", "all"):
        corners.extend(build_workspace_corners())
    if args.mode in ("joint-limits", "all"):
        corners.extend(build_joint_limit_corners(args.margin))
    if args.mode in ("safety-box", "all"):
        corners.extend(build_safety_box_corners(args.guard_margin))

    logger.info("=" * 60)
    logger.info("Extreme Pose Probe: %d poses, mode=%s, dry_run=%s", len(corners), args.mode, args.dry_run)
    logger.info("=== Box Scale Comparison (sanity check) ===")
    logger.info("  B1 bbox_radius: %.4f m  |  B3 safety box half-width: ~0.05 m  |  ratio: %.1fx",
                BBOX_RADIUS, BBOX_RADIUS / 0.05)
    logger.info("=" * 60)

    robot = None
    gripper = None
    if not args.dry_run:
        import franky  # pyright: ignore[reportMissingImports]
        robot = franky.Robot(args.robot_ip)
        robot.recover_from_errors()
        robot.relative_dynamics_factor = args.speed_factor
        gripper = franky.Gripper(args.robot_ip)
        logger.info("Connected to Franka at %s (speed=%.0f%%)", args.robot_ip, args.speed_factor * 100)

    for idx, corner in enumerate(corners):
        logger.info("")
        logger.info("--- Pose %d/%d [%s]: %s ---", idx+1, len(corners), corner["box_type"], corner["name"])
        logger.info("  %s", corner["description"])

        if corner.get("joints") is not None:
            report_pose(corner["joints"])
        elif corner.get("tcp_target") is not None:
            logger.info("  TCP target: %s m (requires IK or manual positioning)", np.round(corner["tcp_target"], 4).tolist())

        if args.dry_run:
            continue

        if corner.get("joints") is None:
            logger.info("  [safety-box mode] Cartesian-defined pose, skipping auto movement")
            continue

        resp = input(f"\n  Move to this pose? [y/n/q] ({idx+1}/{len(corners)}): ").strip().lower()
        if resp == "q":
            break
        if resp != "y":
            logger.info("  Skipped")
            continue

        import franky  # pyright: ignore[reportMissingImports]
        logger.info("  Moving...")
        motion = franky.JointWaypointMotion([franky.JointWaypoint(corner["joints"].tolist())])
        try:
            robot.move(motion)
            time.sleep(0.5)
            actual = np.asarray(robot.state.q[:7], dtype=np.float64)
            error = np.abs(actual - corner["joints"])
            logger.info("  Actual: %s", np.round(actual, 4).tolist())
            logger.info("  Error:  max=%.4f rad", error.max())
            tcp = np.array(robot.state.O_T_EE.translation)
            report_tcp(tcp)
        except Exception as exc:
            logger.error("  FAILED: %s: %s", type(exc).__name__, exc)
            robot.recover_from_errors()

        input("  Press Enter to continue...")

    if robot is not None:
        resp = input("\nReturn to HOME position? [y/n]: ").strip().lower()
        if resp == "y":
            import franky  # pyright: ignore[reportMissingImports]
            motion = franky.JointWaypointMotion([franky.JointWaypoint(HOME.tolist())])
            robot.move(motion)
            logger.info("Returned to HOME")

    logger.info("Probe complete")

if __name__ == "__main__":
    main()
