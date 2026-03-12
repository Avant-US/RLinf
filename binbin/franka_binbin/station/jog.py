"""
Manual robot command sender (station → robot_server).

This is meant for *first hardware tests* where you do NOT want the PI0.5 policy
to drive the robot yet.

Examples:
  # Hold current position (safe sanity check)
  python -m station.jog --robot-host 127.0.0.1 --hold --enable-arm --duration-s 2

  # Small delta on joint 7 for 1 second
  python -m station.jog --robot-host 127.0.0.1 --joint 7 --delta 0.01 --rate-hz 10 --duration-s 1 --enable-arm

Safety:
  - Default is NOT enabled (no motion).
  - Always keep an operator near E‑Stop for first tests.
"""

from __future__ import annotations

import argparse
import time
from typing import Tuple

from station.robot_client import RobotClient, RobotConnectionParams
from station.robot_protocol import CmdMode


def _zero7() -> Tuple[float, float, float, float, float, float, float]:
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-host", default="127.0.0.1")
    ap.add_argument("--robot-port", type=int, default=15555)
    ap.add_argument("--robot-state-port", type=int, default=0)

    ap.add_argument("--enable-arm", action="store_true", help="actually enable arm motion (DANGEROUS)")
    ap.add_argument("--enable-gripper", action="store_true", help="actually enable gripper motion (DANGEROUS)")
    ap.add_argument("--validity-ms", type=int, default=200)
    ap.add_argument("--rate-hz", type=float, default=10.0)
    ap.add_argument("--duration-s", type=float, default=1.0)
    ap.add_argument("--state-wait-s", type=float, default=2.0, help="max time to wait for first state packet")

    ap.add_argument("--hold", action="store_true", help="command current q (requires state stream)")
    ap.add_argument("--joint", type=int, default=0, help="1-7 joint index for delta mode")
    ap.add_argument("--delta", type=float, default=0.0, help="delta radians for the selected joint")
    ap.add_argument(
        "--gripper",
        type=float,
        default=None,
        help="target gripper width in meters (0.0=closed, 0.08=open). Defaults to holding current width.",
    )
    args = ap.parse_args()

    rc = RobotClient(
        RobotConnectionParams(
            host=args.robot_host,
            command_port=args.robot_port,
            state_port=(args.robot_state_port if args.robot_state_port > 0 else None),
        )
    )
    rc.connect()
    try:
        # Wait briefly for at least one state message.
        t_wait = time.time()
        st = None
        while time.time() - t_wait < float(args.state_wait_s):
            st = rc.get_latest_state()
            if st is not None:
                break
            time.sleep(0.02)

        if args.hold and st is None:
            err = rc.get_state_error() or "no state packet received"
            raise SystemExit(
                "Hold mode requires state stream; could not receive robot state. "
                f"detail={err}. "
                "Check --robot-state-port (default robot_port+1)."
            )

        period = 1.0 / max(1e-6, float(args.rate_hz))
        t_end = time.time() + float(args.duration_s)

        sent = 0
        while time.time() < t_end:
            st = rc.get_latest_state()
            if st is not None:
                q_now = tuple(float(x) for x in st.q)  # type: ignore[assignment]
                g_now = float(st.gripper)
            else:
                q_now = _zero7()
                g_now = 0.0

            if args.hold:
                q_cmd = q_now
                mode = CmdMode.JOINT_POSITION
            else:
                # Joint-delta mode: only one joint.
                j = int(args.joint)
                if j < 1 or j > 7:
                    raise SystemExit("--joint must be 1..7 for delta mode (or pass --hold)")
                dq = [0.0] * 7
                dq[j - 1] = float(args.delta)
                q_cmd = tuple(dq)  # type: ignore[assignment]
                mode = CmdMode.JOINT_DELTA

            g_cmd = g_now
            if args.gripper is not None:
                g_cmd = max(0.0, min(0.08, float(args.gripper)))

            rc.send_arm_command(
                q=q_cmd,  # absolute if hold, delta if delta-mode
                gripper=g_cmd,
                mode=mode,
                validity_ms=int(args.validity_ms),
                enable_arm=bool(args.enable_arm),
                enable_gripper=bool(args.enable_gripper),
            )
            sent += 1
            time.sleep(period)
        # Best-effort stop (holds).
        try:
            rc.send_stop()
        except Exception:
            pass
        print(
            f"sent {sent} commands (enable_arm={bool(args.enable_arm)} enable_gripper={bool(args.enable_gripper)})"
        )
    finally:
        rc.close()


if __name__ == "__main__":
    main()

