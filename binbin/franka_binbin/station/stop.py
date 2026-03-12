"""
Operator stop command.

Usage:
  python -m station.stop --robot-host <robot_pc_ip> --robot-port 5555
"""

from __future__ import annotations

import argparse

from station.robot_client import RobotClient, RobotConnectionParams


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-host", required=True)
    ap.add_argument("--robot-port", type=int, default=5555)
    ap.add_argument("--robot-state-port", type=int, default=0)
    ap.add_argument(
        "--shutdown-server",
        action="store_true",
        help=(
            "also request robot_server to exit (works by default if robot_server is bound to 127.0.0.1; "
            "otherwise start robot_server with --allow-shutdown)"
        ),
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
        rc.send_stop()
        if bool(args.shutdown_server):
            rc.send_shutdown()
    finally:
        rc.close()
    print("STOP sent." + (" Shutdown requested." if bool(args.shutdown_server) else ""))


if __name__ == "__main__":
    main()

