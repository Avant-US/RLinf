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

Camera serials fall back to $RS_GLOBAL_SERIAL / $RS_WRIST_SERIAL when
--global-camera-serial / --wrist-camera-serial are not given (source
configs/franka_plug_eval.env for the confirmed mapping).

Fixes applied per b/d/frk1/grperr_1.md (why the gripper never closed):
  修复 A: replays poses executed since the last inference through the
          server's keypoint-history buffer (state_history_buffer.py), so
          observation.his_len advances per control step, matching the
          30 Hz training semantics instead of lagging n_exec x behind.
  修复 B: measures and logs the *achieved* control-loop Hz (does not change
          robot motion -- franky's blocking Robot.move() is left as-is
          pending on-robot validation per §15.7).
  修复 E: notifies the server ({"command": "reset"}) on every env.reset(),
          so an aborted episode's keypoint/policy state does not leak into
          the next one.
"""
from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from collections import deque
from multiprocessing.connection import Client

import numpy as np

sys.path.insert(0, "/workspace/RLinf/b/x/4dwvla_ext")

from franky_joint_env import FrankyJointEnv
from keyboard_vla_eval import KeyboardVLAEvalWrapper
from state_history_buffer import ExecutedStateBuffer
from vla_debug_logging import configure_logging, format_array, summarize_array

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s", force=True)
logger = logging.getLogger("vla-client")

AUTHKEY = b"4dwvla-eval"

# Control-frequency observability (修复 B in b/d/frk1/grperr_1.md): the actual
# wall-clock rate can be far below --control-hz (franky's Robot.move() blocks
# until the waypoint motion settles). We only *measure and warn* here -- we do
# NOT change move_joints() to asynchronous execution, since that changes real
# robot motion behavior and must go through the §15.7 four-level on-robot
# validation procedure before being trusted, not just a code review.
_HZ_LOG_INTERVAL_STEPS = 50
_HZ_WARN_RATIO = 0.5  # warn if achieved Hz < 50% of requested Hz


class VLAEvalController:
    """Orchestrates VLA evaluation using env.step() / env.reset()."""

    def __init__(self, env, server_address, task, n_exec=10, max_steps=300,
                 control_hz=10.0, dry_run=False):
        self._env = env
        self._server_address = server_address
        self._task = task
        self._n_exec = n_exec
        self._max_steps = max_steps
        self._control_hz = control_hz
        self._dry_run = dry_run
        self._action_queue: deque = deque()
        self._conn = None
        self._abort = False
        self._step_count = 0
        self._total_warnings = 0
        self._inference_count = 0
        # R1 fix (修复 A): poses actually executed since the last inference,
        # so the server's keypoint history advances once per control step
        # instead of once per inference (see state_history_buffer.py).
        self._state_history = ExecutedStateBuffer()
        # B: wall-clock control-rate observability only (see module docstring).
        self._last_step_ts: float | None = None
        self._step_interval_sum = 0.0
        self._step_interval_count = 0
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

    def _notify_server_reset(self):
        """Tell the server to reset policy/keypoint state (修复 E).

        Without this, a mid-run 'r'-abort + reset() keeps the *server's*
        FKKeypointComputer history and policy KV cache from the previous,
        aborted episode -- they only get cleared on a fresh TCP connection.
        The server already supports {"command": "reset"}; the client just
        never called it.
        """
        if self._conn is None:
            return
        try:
            self._conn.send({"command": "reset"})
            resp = self._conn.recv()
            if resp.get("status") != "ok":
                logger.warning("Server reset returned non-ok status: %s", resp.get("status"))
        except Exception as exc:
            logger.warning("Server reset notification failed: %s", exc)

    def _reset_episode(self):
        """Reset the env, clear local executed-state history, and notify the server."""
        obs, info = self._env.reset()
        self._state_history.clear()
        self._notify_server_reset()
        self._last_step_ts = None
        return obs, info

    def _track_control_interval(self):
        """Measure the achieved control-loop wall-clock rate (修复 B, observability only).

        franky's Robot.move() blocks until the waypoint motion settles, so the
        actual rate can be far below --control-hz. We log it so that is
        visible instead of silently assumed.
        """
        now = time.monotonic()
        if self._last_step_ts is not None:
            dt = now - self._last_step_ts
            if dt > 0:
                self._step_interval_sum += dt
                self._step_interval_count += 1
        self._last_step_ts = now

        if self._step_interval_count and self._step_interval_count % _HZ_LOG_INTERVAL_STEPS == 0:
            achieved_hz = self._step_interval_count / self._step_interval_sum
            ratio = achieved_hz / self._control_hz if self._control_hz else 1.0
            log_fn = logger.warning if ratio < _HZ_WARN_RATIO else logger.info
            log_fn(
                "[step %d] control rate: achieved=%.2f Hz requested=%.1f Hz (ratio=%.2f)",
                self._step_count, achieved_hz, self._control_hz, ratio,
            )

    def _request_inference(self, images, state):
        state_arr = np.asarray(state, dtype=np.float64)
        # R1 fix (修复 A): replay poses executed since the last inference so
        # the server's keypoint-history clock advances per control step, not
        # per inference (see state_history_buffer.py / grperr_1.md §4 修复 A).
        state_history = self._state_history.drain()
        logger.info(
            "[inference %d] request: state=%s state_history_len=%d image_meta=%s task=%r",
            self._inference_count,
            format_array(state_arr),
            len(state_history),
            {name: summarize_array(image) for name, image in images.items()},
            self._task,
        )
        self._conn.send({
            "images": images,
            "state": {"arm": state_arr[:7].tolist(), "gripper": [float(state_arr[7])]},
            "state_history": state_history,
            "task": self._task,
            "protocol": 2,
        })
        resp = self._conn.recv()
        if resp["status"] != "ok":
            raise RuntimeError(f"Server error: {resp['status']}")
        actions = np.asarray(resp["actions"], dtype=np.float64)
        if actions.ndim != 2 or actions.shape[1] != 8:
            raise ValueError(f"Expected actions with shape [N, 8], got {actions.shape}")
        if not np.isfinite(actions).all():
            raise ValueError("Inference response contains NaN or infinite actions")

        state_to_plan = np.vstack([state_arr, actions])
        plan_delta = np.diff(state_to_plan, axis=0)
        logger.info(
            "[inference %d] response: shape=%s full_actions=%s "
            "delta_from_previous=%s action_meta=%s",
            self._inference_count,
            list(actions.shape),
            format_array(actions),
            format_array(plan_delta),
            summarize_array(actions),
        )
        self._inference_count += 1
        return actions.tolist()

    def run(self):
        logger.info("Starting: task=%r, max_steps=%d, n_exec=%d, control_hz=%.1f",
                    self._task, self._max_steps, self._n_exec, self._control_hz)
        obs, info = self._reset_episode()

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
            state_before = np.asarray(obs["state"], dtype=np.float64)
            action_delta = action_arr - state_before
            logger.info(
                "[step %d] execute: action=%s state_before=%s delta=%s",
                self._step_count,
                format_array(action_arr),
                format_array(state_before),
                format_array(action_delta),
            )

            # Record state *before* execution: training's his_kpts contains
            # frames strictly earlier than the current frame (see grperr_1.2.md §3.1).
            self._state_history.record(state_before[:7])

            if not self._dry_run:
                obs, reward, terminated, truncated, info = self._env.step(action_arr)
                state_after = np.asarray(obs["state"], dtype=np.float64)
                logger.info(
                    "[step %d] result: state_after=%s realized_delta=%s "
                    "reward=%s terminated=%s truncated=%s warnings=%s info=%s",
                    self._step_count,
                    format_array(state_after),
                    format_array(state_after - state_before),
                    reward,
                    terminated,
                    truncated,
                    info.get("warnings", []),
                    info,
                )
                if truncated:
                    reason = info.get("motion_guard_trip") or info.get("abort_reset") or "unknown"
                    logger.info("Episode truncated: %s", reason)
                    self._action_queue.clear()
                    input("[operator] Reset scene, then press Enter...")
                    obs, info = self._reset_episode()
                    continue
                self._total_warnings += len(info.get("warnings", []))
            else:
                state_after = np.asarray(obs["state"], dtype=np.float64)
                logger.info(
                    "[step %d] DRY RUN: state_after=%s realized_delta=%s",
                    self._step_count,
                    format_array(state_after),
                    format_array(state_after - state_before),
                )

            self._track_control_interval()
            self._step_count += 1

        achieved_hz = (
            self._step_interval_count / self._step_interval_sum
            if self._step_interval_count else float("nan")
        )
        logger.info(
            "Done: %d steps, %d warnings, abort=%s, achieved_control_hz=%.2f (requested %.1f)",
            self._step_count, self._total_warnings, self._abort, achieved_hz, self._control_hz,
        )


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
    # F fix (grperr_1.md §4 修复 F): fall back to RS_GLOBAL_SERIAL /
    # RS_WRIST_SERIAL (documented in 4wvla_rlinf_eval_3A3.md §7.3 but
    # previously never read by this script) so a config file such as
    # configs/franka_plug_eval.env can be sourced instead of retyping the
    # serials on every invocation. Confirmed mapping: global=250222073513,
    # wrist=420122070525.
    p.add_argument(
        "--global-camera-serial",
        default=os.environ.get("RS_GLOBAL_SERIAL"),
        help="RealSense serial for the global camera (default: $RS_GLOBAL_SERIAL)",
    )
    p.add_argument(
        "--wrist-camera-serial",
        default=os.environ.get("RS_WRIST_SERIAL"),
        help="RealSense serial for the wrist camera (default: $RS_WRIST_SERIAL)",
    )
    p.add_argument(
        "--log-dir",
        default=None,
        help="Directory for timestamped client logs (default: VLA_LOG_DIR or extension/logs)",
    )
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    configure_logging("client", args.log_dir)
    if args.use_realsense:
        logger.info(
            "Camera serials resolved: global=%s wrist=%s (CLI overrides "
            "$RS_GLOBAL_SERIAL/$RS_WRIST_SERIAL when given)",
            args.global_camera_serial, args.wrist_camera_serial,
        )
        if not args.global_camera_serial or not args.wrist_camera_serial:
            logger.warning(
                "--use-realsense set but a camera serial is missing "
                "(global=%s, wrist=%s); the RealSense SDK will auto-assign "
                "devices, which risks swapping global/wrist. See "
                "b/x/4dwvla_ext/configs/franka_plug_eval.env",
                args.global_camera_serial, args.wrist_camera_serial,
            )
    env = None
    ctrl = None
    try:
        env = FrankyJointEnv(
            robot_ip=args.robot_ip, control_hz=args.control_hz,
            is_dummy=args.dry_run, use_realsense=args.use_realsense,
            camera_serials={"global": args.global_camera_serial, "wrist": args.wrist_camera_serial}
            if args.use_realsense else None,
        )
        if not args.dry_run:
            env = KeyboardVLAEvalWrapper(env)
        else:
            logger.info("Dry-run enabled: keyboard controls are disabled.")

        ctrl = VLAEvalController(
            env=env, server_address=(args.server_host, args.server_port),
            task=args.task, n_exec=args.n_exec, max_steps=args.max_steps,
            control_hz=args.control_hz, dry_run=args.dry_run,
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
