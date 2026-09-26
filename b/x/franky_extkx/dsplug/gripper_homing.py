#!/usr/bin/env python3
"""Inspect, and optionally re-run, the Franka Hand homing calibration.

The hand reports its full-open width from whatever its last homing
established.  On this cell that reading drifted away from reality: a caliper
measures an 80 mm finger gap at full open while libfranka reports 66.4 mm.
That 13.6 mm offset shifts the whole gripper channel, because the policy was
trained on ``a = 1 - w / 0.08``: at a true 79 mm opening the "stay open"
command is ``a = 0.008``, but at a reported 66.4 mm it is ``a = 0.170``.  See
``b/d/frk1/grperr_1.2.md`` Q1 and §4.1.

The default mode is read-only.  ``--execute`` re-runs homing, which traverses
the fingers through their entire range, so nothing may be between them and
nothing may be held.  The script asks the operator to confirm that in words.

Examples (inside the Franky container)::

    python b/x/franky_ext/dsplug/gripper_homing.py --robot-ip 172.16.0.2
    python b/x/franky_ext/dsplug/gripper_homing.py \
        --robot-ip 172.16.0.2 --execute

Before connecting, stop any RLinf/Ray/VLA process that owns the FCI session.
Homing only touches the hand; it does not move the arm.  It is nevertheless a
physical motion, so keep the E-stop within reach.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# Allow this file to run as a path from either the host checkout or the
# /workspace/RLinf bind mount.  ``franky_ext`` is an out-of-tree extension.
_BX_ROOT = Path(__file__).resolve().parents[2]
if str(_BX_ROOT) not in sys.path:
    sys.path.insert(0, str(_BX_ROOT))

LOGGER = logging.getLogger("dsplug.gripper_homing")

#: Full-open width the demonstrations were recorded with, in meters. The
#: training affine uses w0 = 0.08, and the observed maximum was 0.0794.
_DEMO_MAX_WIDTH_M = 0.0794
#: Report a mismatch once the hand disagrees with the demonstrations by more
#: than this. 5 mm shifts the "stay open" command by 5/80 = 0.06 in action
#: units, which is already comparable to the whole approach-phase spread.
_MISMATCH_WARN_M = 0.005
_CONFIRM_TOKEN = "HOME_GRIPPER"


def _describe(gripper) -> tuple[float | None, float | None, bool]:
    """Return ``(reported_max_width, current_width, holding)``."""
    max_width = gripper.max_width
    try:
        width = float(gripper.position)
    except Exception:
        width = None
    try:
        holding = bool(gripper._hardware_holding())
    except Exception:
        holding = False
    return max_width, width, holding


def _report(label: str, max_width: float | None, width: float | None, holding: bool) -> None:
    LOGGER.info(
        "%s: reported max_width=%s current width=%s holding=%s",
        label,
        f"{max_width:.4f}m" if max_width is not None else "unknown",
        f"{width:.4f}m" if width is not None else "unknown",
        holding,
    )
    if max_width is None:
        LOGGER.warning("hand did not report max_width; cannot compare with demos")
        return
    delta = _DEMO_MAX_WIDTH_M - max_width
    LOGGER.info(
        "vs demonstrations (%.4fm): delta=%+.4fm; 'stay open' command would be "
        "a=%.3f here vs a=%.3f in training",
        _DEMO_MAX_WIDTH_M,
        delta,
        1.0 - max_width / 0.08,
        1.0 - _DEMO_MAX_WIDTH_M / 0.08,
    )
    if abs(delta) > _MISMATCH_WARN_M:
        LOGGER.warning(
            "reported full open differs from the demonstrations by %.1fmm; the "
            "gripper channel is offset by %.3f in action units",
            abs(delta) * 1000.0,
            abs(delta) / 0.08,
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Inspect or re-run Franka Hand homing (read-only by default)"
    )
    parser.add_argument("--robot-ip", default="172.16.0.2")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="re-run homing after confirmation; the fingers traverse their "
        "full range, so nothing may be between them",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the interactive confirmation (only for unattended bring-up "
        "where the fingers are known to be clear)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )

    from franky_ext.franka_libfranka_gripper import FrankaLibfrankaGripper

    gripper = FrankaLibfrankaGripper(args.robot_ip)
    max_width, width, holding = _describe(gripper)
    _report("before", max_width, width, holding)

    if not args.execute:
        LOGGER.info("read-only mode; pass --execute to re-run homing")
        print("RESULT DSPLUG_GRIPPER_INSPECT PASS")
        return 0

    if holding:
        LOGGER.error(
            "hand reports holding (width=%s); homing would traverse the full "
            "range and drop whatever is held. Release it first.",
            f"{width:.4f}m" if width is not None else "unknown",
        )
        print("RESULT DSPLUG_GRIPPER_HOMING FAIL")
        return 1

    if not args.yes:
        print(
            "\nHoming moves the fingers through their entire range.\n"
            "Confirm that nothing is between the fingers and nothing is held.\n"
            f"Type {_CONFIRM_TOKEN} to proceed: ",
            end="",
        )
        if input().strip() != _CONFIRM_TOKEN:
            LOGGER.info("not confirmed; aborting without homing")
            print("RESULT DSPLUG_GRIPPER_HOMING ABORT")
            return 1

    LOGGER.info("homing (blocking, may take several seconds)...")
    gripper.homing()
    max_width_after, width_after, holding_after = _describe(gripper)
    _report("after", max_width_after, width_after, holding_after)

    if max_width is not None and max_width_after is not None:
        LOGGER.info(
            "reported max_width %.4fm -> %.4fm (%+.4fm)",
            max_width,
            max_width_after,
            max_width_after - max_width,
        )
    if max_width_after is not None:
        LOGGER.info(
            "set FRANKA_GRIPPER_MAX_WIDTH_M=%.3f in "
            "b/x/4dwvla_ext/configs/franka_plug_eval.env",
            max_width_after,
        )
    print("RESULT DSPLUG_GRIPPER_HOMING PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
