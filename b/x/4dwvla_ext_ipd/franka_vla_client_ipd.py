#!/usr/bin/env python3
"""4DWVLA eval client that drives the Franka arm by joint impedance.

Same IPC, cameras, keyboard, and gripper path as ``franka_vla_client.py``.
The arm difference is ``ImpedanceJointEnv``: each joint action is an
impedance equilibrium updated by ``JointImpedanceTracker.set_target``, not a
blocking ``JointWaypointMotion``.

This changes real robot motion. Keep the E-stop in reach. The default
``--control-hz 20`` and ``--n-exec 2`` match a loop that returns before the
arm arrives; a blocking waypoint client should keep its own slower defaults.

    source b/x/4dwvla_ext_ipd/configs/franka_plug_impedance.env
    python b/x/4dwvla_ext_ipd/franka_vla_client_ipd.py \\
        --robot-ip 172.16.0.2 \\
        --task "plug into socket" \\
        --use-realsense
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

_EXT_DIR = Path(__file__).resolve().parents[1] / "4dwvla_ext"
if str(_EXT_DIR) not in sys.path:
    sys.path.insert(0, str(_EXT_DIR))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from franka_vla_client import VLAEvalController  # noqa: E402
from impedance_joint_env import ImpedanceJointEnv  # noqa: E402
from keyboard_vla_eval import KeyboardVLAEvalWrapper  # noqa: E402
from vla_debug_logging import configure_logging  # noqa: E402

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    force=True,
)
logger = logging.getLogger("vla-client-ipd")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot-ip", default="172.16.0.2")
    p.add_argument("--server-host", default="localhost")
    p.add_argument("--server-port", type=int, default=5555)
    p.add_argument("--task", required=True)
    p.add_argument("--n-exec", type=int, default=2)
    p.add_argument("--control-hz", type=float, default=20.0)
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--use-realsense", action="store_true")
    p.add_argument(
        "--global-camera-serial",
        default=os.environ.get("RS_GLOBAL_SERIAL"),
    )
    p.add_argument(
        "--wrist-camera-serial",
        default=os.environ.get("RS_WRIST_SERIAL"),
    )
    p.add_argument("--log-dir", default=None)
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    configure_logging("client-ipd", args.log_dir)
    logger.info(
        "Arm backend: joint impedance. move_joints updates the tracker and "
        "returns; it does not wait for the waypoint to settle."
    )
    if args.use_realsense and (
        not args.global_camera_serial or not args.wrist_camera_serial
    ):
        logger.warning(
            "camera serial missing (global=%s wrist=%s); devices may be swapped",
            args.global_camera_serial,
            args.wrist_camera_serial,
        )

    env = None
    ctrl = None
    try:
        env = ImpedanceJointEnv(
            robot_ip=args.robot_ip,
            control_hz=args.control_hz,
            is_dummy=args.dry_run,
            use_realsense=args.use_realsense,
            camera_serials=(
                {"global": args.global_camera_serial, "wrist": args.wrist_camera_serial}
                if args.use_realsense
                else None
            ),
        )
        if not args.dry_run:
            env = KeyboardVLAEvalWrapper(env)
        else:
            logger.info("Dry-run enabled: keyboard controls are disabled.")
        ctrl = VLAEvalController(
            env=env,
            server_address=(args.server_host, args.server_port),
            task=args.task,
            n_exec=args.n_exec,
            max_steps=args.max_steps,
            control_hz=args.control_hz,
            dry_run=args.dry_run,
        )
        ctrl.connect()
        ctrl.run()
    except KeyboardInterrupt:
        logger.info("Interrupted")
    except Exception as exc:
        logger.error("Fatal: %s: %s", type(exc).__name__, exc)
        raise
    finally:
        if ctrl is not None:
            try:
                ctrl.disconnect()
            except Exception as exc:
                logger.warning("Controller disconnect failed: %s", exc)
        if env is not None:
            try:
                env.close()
            except Exception as exc:
                logger.warning("env.close() failed: %s", exc)


if __name__ == "__main__":
    main()
