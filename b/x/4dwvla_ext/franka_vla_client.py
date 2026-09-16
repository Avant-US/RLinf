#!/usr/bin/env python3
"""Franka VLA evaluation client using gym.Env interface.

Architecture:
  KeyboardVLAEvalWrapper (a/r/b/c/h keyboard, evdev KeyboardListener)
    -> FrankyJointEnv (joint-space, 8-level safety)
      -> VLAEvalController (IPC to GPU inference server)

Usage:
    source /opt/venv/franky-0.19.0/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py \
        --robot-ip 172.16.0.2 \
        --task "plug into socket" \
        --use-realsense
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from collections import deque
from multiprocessing.connection import Client

import numpy as np

sys.path.insert(0, "/workspace/RLinf/b/x/4dwvla_ext")

from franky_joint_env import FrankyJointEnv
from keyboard_vla_eval import KeyboardVLAEvalWrapper

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s", force=True)
logger = logging.getLogger("vla-client")

AUTHKEY = b"4dwvla-eval"


class VLAEvalController:
    """Orchestrates VLA evaluation using env.step() / env.reset()."""

    def __init__(self, env, server_address, task, n_exec=10, max_steps=300, dry_run=False):
        self._env = env
        self._server_address = server_address
        self._task = task
        self._n_exec = n_exec
        self._max_steps = max_steps
        self._dry_run = dry_run
        self._action_queue: deque = deque()
        self._conn = None
        self._abort = False
        self._step_count = 0
        self._total_warnings = 0
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, '_abort', True))

    def connect(self):
        logger.info("Connecting to %s:%d...", *self._server_address)
        self._conn = Client(self._server_address, authkey=AUTHKEY)
        logger.info("Connected")

    def disconnect(self):
        if self._conn:
            try: self._conn.send({"command": "shutdown"})
            except Exception: pass
            self._conn.close()
            self._conn = None

    def _request_inference(self, images, state):
        self._conn.send({
            "images": images,
            "state": {"arm": state[:7].tolist(), "gripper": [float(state[7])]},
            "task": self._task,
        })
        resp = self._conn.recv()
        if resp["status"] != "ok":
            raise RuntimeError(f"Server error: {resp['status']}")
        return resp["actions"]

    def run(self):
        logger.info("Starting: task=%r, max_steps=%d, n_exec=%d", self._task, self._max_steps, self._n_exec)
        obs, info = self._env.reset()

        while self._step_count < self._max_steps and not self._abort:
            if not self._action_queue:
                images = self._env.get_camera_frames()
                state = obs["state"]
                logger.info("[step %d] Inference (q1=%.3f, grip=%.4f)", self._step_count, state[0], state[7])
                actions = self._request_inference(images, state)
                self._action_queue.extend(actions)
                logger.info("  Received %d actions", len(actions))

            action = self._action_queue.popleft()
            action_arr = np.array(action, dtype=np.float64)

            if not self._dry_run:
                obs, reward, terminated, truncated, info = self._env.step(action_arr)
                if truncated:
                    reason = info.get("motion_guard_trip") or info.get("abort_reset") or "unknown"
                    logger.info("Episode truncated: %s", reason)
                    self._action_queue.clear()
                    input("[operator] Reset scene, then press Enter...")
                    obs, info = self._env.reset()
                    continue
                self._total_warnings += len(info.get("warnings", []))
            else:
                logger.info("[step %d] DRY RUN: q1=%.3f grip=%.2f",
                            self._step_count, action_arr[0],
                            action_arr[7] if len(action_arr) > 7 else 0.5)

            self._step_count += 1

        logger.info("Done: %d steps, %d warnings, abort=%s", self._step_count, self._total_warnings, self._abort)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot-ip", default="172.16.0.2")
    p.add_argument("--server-host", default="localhost")
    p.add_argument("--server-port", type=int, default=5555)
    p.add_argument("--task", required=True)
    p.add_argument("--n-exec", type=int, default=10)
    p.add_argument("--control-hz", type=float, default=10.0)
    p.add_argument("--max-steps", type=int, default=300)
    p.add_argument("--use-realsense", action="store_true")
    p.add_argument("--global-camera-serial", default=None)
    p.add_argument("--wrist-camera-serial", default=None)
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    env = FrankyJointEnv(
        robot_ip=args.robot_ip, control_hz=args.control_hz,
        is_dummy=args.dry_run, use_realsense=args.use_realsense,
        camera_serials={"global": args.global_camera_serial, "wrist": args.wrist_camera_serial}
        if args.use_realsense else None,
    )
    env = KeyboardVLAEvalWrapper(env)

    ctrl = VLAEvalController(
        env=env, server_address=(args.server_host, args.server_port),
        task=args.task, n_exec=args.n_exec, max_steps=args.max_steps, dry_run=args.dry_run,
    )

    try:
        ctrl.connect()
        ctrl.run()
    except KeyboardInterrupt:
        logger.info("Interrupted")
    except Exception as exc:
        logger.error("Fatal: %s: %s", type(exc).__name__, exc)
        raise
    finally:
        ctrl.disconnect()
        env.close()


if __name__ == "__main__":
    main()
