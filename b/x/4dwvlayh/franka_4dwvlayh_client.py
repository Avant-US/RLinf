#!/usr/bin/env python3
"""Franka3 client for 4DWVLA (InternVLA-A1.5) via vla_inference_server.py.

Runs in the gello/franky venv.  Talks to the 4DWVLA GPU inference server
over TCP ``multiprocessing.connection`` (not WebSocket).  Robot control
(JointImpedanceTracker, Hermite interpolation, gripper logic, cameras)
is reused from deploy_plug_franka3.py in this directory.

Usage:
    source /home/nvidia/cxy_ws/gello_software/.venv/bin/activate
    # dry run (no robot, no camera):
    python franka_4dwvlayh_client.py --server-host 127.0.0.1 --server-port 5555 \
        --task "plug into socket" --dry-run --no-robot --no-camera
    # real robot:
    python franka_4dwvlayh_client.py --server-host 127.0.0.1 --server-port 5555 \
        --task "plug into socket"
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from multiprocessing.connection import Client
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from deploy_plug_franka3 import (
    DEFAULT_BLEND_STEPS,
    DEFAULT_EXECUTE_HORIZON,
    DEFAULT_INTERP_HZ,
    DEFAULT_MAX_GRIPPER_WIDTH,
    DEFAULT_MAX_JOINT_JUMP_DEG,
    DEFAULT_SPEED,
    GLOBAL_CAMERA_SERIAL,
    GRIPPER_CLOSE_G,
    GRIPPER_CLOSE_RISE,
    GRIPPER_CONFIRM_INFERS,
    GRIPPER_CONFIRM_STEPS,
    GRIPPER_OPEN_G,
    IMAGE_HW,
    JOINT_DAMPING,
    JOINT_STIFFNESS,
    MAX_JOINT_TRACKING_ERROR,
    POST_CLOSE_EXTRA_STEPS,
    WRIST_CAMERA_SERIAL,
    CameraThread,
    RunLogger,
    WrenchSampler,
    apply_gripper,
    apply_rt_hardening,
    auto_prefetch,
    blend_chunk,
    clip_target,
    coast_while,
    fake_images,
    gello_to_width,
    gripper_close_intent,
    max_joint_jump_rad,
    play_joint_segment,
    width_to_gello,
)

AUTHKEY = b"4dwvla-eval"
DEFAULT_LOG_ROOT = _HERE / "runs"


class VLAServerConnection:
    """IPC to vla_inference_server.py via multiprocessing.connection."""

    def __init__(self, host: str, port: int):
        self._conn = Client((host, port), authkey=AUTHKEY)

    def reset(self) -> dict:
        self._conn.send({"command": "reset"})
        return self._conn.recv()

    def infer(
        self,
        *,
        global_image: np.ndarray,
        wrist_image: np.ndarray,
        arm_q: np.ndarray,
        gripper_g: float,
        task: str,
        state_history: list[list[float]],
    ) -> tuple[np.ndarray, float]:
        msg = {
            "images": {
                "global": np.ascontiguousarray(global_image, dtype=np.uint8),
                "wrist": np.ascontiguousarray(wrist_image, dtype=np.uint8),
            },
            "state": {
                "arm": np.asarray(arm_q, dtype=np.float32).reshape(7).tolist(),
                "gripper": [float(gripper_g)],
            },
            "state_history": state_history,
            "task": task,
            "protocol": 2,
        }
        t0 = time.perf_counter()
        self._conn.send(msg)
        resp = self._conn.recv()
        rtt_ms = (time.perf_counter() - t0) * 1000.0
        if resp.get("status") != "ok":
            raise RuntimeError(f"Server error: {resp.get('status')}")
        actions = np.asarray(resp["actions"], dtype=np.float64)
        if actions.ndim == 1:
            actions = actions.reshape(1, -1)
        return actions, rtt_ms

    def close(self) -> None:
        try:
            self._conn.send({"command": "shutdown"})
        except Exception:
            pass
        try:
            self._conn.close()
        except Exception:
            pass


class ExecutedStateBuffer:
    """7-DOF arm poses executed since the last inference, for keypoint history."""

    def __init__(self, max_len: int = 512) -> None:
        self._buf: deque[list[float]] = deque(maxlen=max_len)

    def record(self, arm_q7) -> None:
        self._buf.append(
            [float(v) for v in np.asarray(arm_q7).reshape(-1)[:7]]
        )

    def drain(self) -> list[list[float]]:
        out = list(self._buf)
        self._buf.clear()
        return out

    def clear(self) -> None:
        self._buf.clear()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--server-host", default="127.0.0.1")
    p.add_argument("--server-port", type=int, default=5555)
    p.add_argument("--robot-ip", default="172.16.0.2")
    p.add_argument("--task", default="plug into socket")
    p.add_argument("--dry-run", action="store_true", help="Do not start the tracker")
    p.add_argument("--no-robot", action="store_true", help="Fake proprio, no franky")
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--once", action="store_true", help="One infer then exit")
    p.add_argument(
        "--wait-enter",
        action="store_true",
        help="Wait for Enter before the policy loop",
    )
    p.add_argument("--home-gripper", action="store_true")
    p.add_argument(
        "--execute-horizon",
        type=int,
        default=DEFAULT_EXECUTE_HORIZON,
    )
    p.add_argument("--speed", type=float, default=DEFAULT_SPEED)
    p.add_argument("--interp-hz", type=float, default=DEFAULT_INTERP_HZ)
    p.add_argument("--no-pipeline", action="store_true")
    p.add_argument("--prefetch", type=int, default=0)
    p.add_argument("--blend-steps", type=int, default=DEFAULT_BLEND_STEPS)
    p.add_argument(
        "--gripper-mode",
        choices=("binary", "width"),
        default="binary",
    )
    p.add_argument("--gripper-close-g", type=float, default=GRIPPER_CLOSE_G)
    p.add_argument("--gripper-close-rise", type=float, default=GRIPPER_CLOSE_RISE)
    p.add_argument(
        "--max-joint-jump-deg",
        type=float,
        default=DEFAULT_MAX_JOINT_JUMP_DEG,
    )
    p.add_argument(
        "--max-tracking-error-deg",
        type=float,
        default=np.rad2deg(MAX_JOINT_TRACKING_ERROR),
    )
    p.add_argument("--no-rt", action="store_true")
    p.add_argument("--log-dir", default=str(DEFAULT_LOG_ROOT))
    p.add_argument("--no-log", action="store_true")
    p.add_argument(
        "--control-hz",
        type=float,
        default=15.0,
        help="Nominal policy control rate (default 15 Hz).",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    running = True

    def _stop(*_a):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    if not args.no_rt and not args.no_robot:
        print("Applying RT hardening...")
        apply_rt_hardening()

    max_tracking_error_rad = float(np.deg2rad(args.max_tracking_error_deg))
    speed = float(args.speed)
    if speed <= 0:
        print(f"refusing: --speed must be > 0 (got {speed})")
        return 2
    interp_hz = float(args.interp_hz)
    hz = float(args.control_hz)
    dt = 1.0 / hz
    execute_horizon = int(args.execute_horizon)
    close_g = float(args.gripper_close_g)
    close_rise = float(args.gripper_close_rise)
    blend_steps = int(args.blend_steps)
    prefetch_arg = int(args.prefetch)

    print(f"Connecting to vla_inference_server {args.server_host}:{args.server_port} ...")
    conn = VLAServerConnection(args.server_host, args.server_port)
    print(
        f"Connected.  control_hz={hz} execute_horizon={execute_horizon} "
        f"speed={speed:.2f} play={1000.0 * dt / speed:.0f}ms/step "
        f"interp={interp_hz:.0f}Hz hermite "
        f"pipeline={'off' if args.no_pipeline else 'on'} "
        f"blend={blend_steps} gripper_mode={args.gripper_mode}"
    )

    infer_i = 0
    robot = gripper = tracker = sampler = None
    cameras: dict[str, CameraThread] = {}
    max_grip = DEFAULT_MAX_GRIPPER_WIDTH
    last_gello_g = 0.0
    prev_q = None
    prev_grip_state = None
    last_cmd_width = None
    close_streak = 0
    infer_close_streak = 0
    open_infer_streak = 0
    want_closed = False
    state_history = ExecutedStateBuffer()
    run_log: RunLogger | None = None

    if not args.no_log:
        log_meta = {
            "server_host": args.server_host,
            "server_port": args.server_port,
            "robot_ip": args.robot_ip,
            "dry_run": args.dry_run,
            "no_robot": args.no_robot,
            "prompt": args.task,
            "model": "4dwvla-vla_inference_server",
            "speed": speed,
            "interp_hz": interp_hz,
            "execute_horizon": execute_horizon,
            "pipeline": not args.no_pipeline,
            "blend_steps": blend_steps,
            "prefetch": prefetch_arg,
            "control_hz": hz,
        }
        run_log = RunLogger(Path(args.log_dir), log_meta)
        print(f"run log → {run_log.dir}")

    def log_wrench(*, phase: str, infer_idx: int) -> None:
        if run_log is None or sampler is None:
            return
        ts, w = sampler.drain()
        if ts.size:
            run_log.log_wrench(ts, w, phase=phase, infer_i=infer_idx)

    try:
        if not args.no_robot:
            import franky

            print(f"Connecting to FR3 at {args.robot_ip} ...")
            robot = franky.Robot(args.robot_ip)
            robot.recover_from_errors()
            robot.relative_dynamics_factor = 0.2
            robot.set_collision_behavior(
                [80.0, 80.0, 80.0, 80.0, 11.0, 11.0, 11.0],
                [100.0, 100.0, 100.0, 25.0, 25.0, 25.0],
            )
            gripper = franky.Gripper(args.robot_ip)
            try:
                max_grip = float(gripper.max_width) or DEFAULT_MAX_GRIPPER_WIDTH
            except Exception:
                max_grip = DEFAULT_MAX_GRIPPER_WIDTH
            print(
                f"FR3 ok. gripper={gripper.width * 1000:.1f} mm  "
                f"max={max_grip * 1000:.1f} mm"
            )
            if args.home_gripper:
                print("Homing gripper...")
                gripper.homing()
                gripper.open(speed=0.4)
            last_gello_g = width_to_gello(float(gripper.width), max_grip)
            sampler = WrenchSampler(robot)
            if not args.dry_run:
                tracker = franky.JointImpedanceTracker(
                    robot,
                    stiffness=np.array(JOINT_STIFFNESS, dtype=np.float64),
                    damping=np.array(JOINT_DAMPING, dtype=np.float64),
                    compensate_coriolis=True,
                )
                q0 = np.array(robot.state.q, dtype=np.float64)
                tracker.set_target(q0, dq=np.zeros(7))
                prev_q = q0.copy()
                print("JointImpedanceTracker holding current pose")

        if not args.no_camera:
            print("Starting cameras...")
            for name, serial in (
                ("image", GLOBAL_CAMERA_SERIAL),
                ("wrist_image", WRIST_CAMERA_SERIAL),
            ):
                cameras[name] = CameraThread(serial, name=name)
                print(f"  {name} ({serial}) ready")

        def read_images() -> tuple[np.ndarray, np.ndarray]:
            if args.no_camera:
                return fake_images()
            return cameras["image"].read_rgb(), cameras["wrist_image"].read_rgb()

        def read_arm_gripper() -> tuple[np.ndarray, float]:
            nonlocal last_gello_g
            if robot is None:
                return np.zeros(7, np.float32), last_gello_g
            s = robot.state
            q = np.array(s.q, dtype=np.float32)
            last_gello_g = width_to_gello(float(gripper.width), max_grip)
            return q, last_gello_g

        print("Sending reset to server...")
        resp = conn.reset()
        print(f"  reset → {resp.get('status')}")
        state_history.clear()

        if args.wait_enter and not args.once:
            input("Position the scene, then press Enter to run policy...")

        last_dq = np.zeros(7, dtype=np.float64)
        last_rtt_ms = 200.0
        pipeline = (not args.no_pipeline) and (not args.once)
        enable_grip = gripper is not None and not args.dry_run

        def do_infer():
            nonlocal last_gello_g
            global_img, wrist_img = read_images()
            arm_q, g = read_arm_gripper()
            history = state_history.drain()
            log_wrench(phase="policy", infer_idx=infer_i + 1)
            actions, rtt = conn.infer(
                global_image=global_img,
                wrist_image=wrist_img,
                arm_q=arm_q,
                gripper_g=g,
                task=args.task,
                state_history=history,
            )
            return actions, rtt, arm_q, g, len(history)

        def consume_reply(actions, rtt_ms, obs_arm, obs_g, n_history):
            nonlocal infer_i, infer_close_streak, open_infer_streak, want_closed

            infer_i += 1
            if actions.shape[1] < 8:
                raise RuntimeError(
                    f"server actions dim {actions.shape[1]} < 8; "
                    "need joints[7] + gripper[1]"
                )

            m = execute_horizon
            step_dt = dt / speed
            g_all = actions[:, 7].reshape(-1)
            gmax = float(np.max(g_all)) if g_all.size else 0.0
            g_rise = float(g_all[-1] - g_all[0]) if g_all.size else 0.0
            intent_close = gripper_close_intent(
                g_all, close_g=close_g, rise_min=close_rise
            )
            if intent_close:
                infer_close_streak += 1
                m = max(m, int(actions.shape[0]))
            else:
                infer_close_streak = 0

            close_idx = next(
                (i for i, gv in enumerate(g_all) if gv >= close_g), None
            )
            if close_idx is not None:
                m = max(m, close_idx + 1 + POST_CLOSE_EXTRA_STEPS)
            m = max(1, min(m, int(actions.shape[0])))
            raw_chunk = actions[:m]

            gmax_mm = gello_to_width(gmax, max_grip) * 1000.0
            if infer_close_streak >= GRIPPER_CONFIRM_INFERS:
                if not want_closed:
                    print(
                        f"  gripper: latch close  gmax={gmax:.3f} "
                        f"rise={g_rise:.3f} (streak={infer_close_streak})"
                    )
                want_closed = True
            if want_closed and not intent_close and gmax <= GRIPPER_OPEN_G:
                open_infer_streak += 1
            else:
                open_infer_streak = 0
            if open_infer_streak >= 2:
                want_closed = False

            q_now = None
            if robot is not None:
                try:
                    q_now = np.asarray(robot.state.q, dtype=np.float64)
                except Exception:
                    q_now = prev_q
            elif prev_q is not None:
                q_now = np.asarray(prev_q, dtype=np.float64)

            dq0_deg = (
                np.rad2deg(max_joint_jump_rad(raw_chunk[0, :7], q_now))
                if q_now is not None
                else float("nan")
            )
            pre = (
                auto_prefetch(rtt_ms, step_dt, m)
                if prefetch_arg <= 0
                else prefetch_arg
            )
            print(
                f"[{infer_i:4d}] rtt={rtt_ms:.0f}ms "
                f"history={n_history} "
                f"q0={np.rad2deg(raw_chunk[0, :7]).round(1)} "
                f"dq0={dq0_deg:.1f}deg "
                f"g_obs={obs_g:.3f} g0={g_all[0]:.3f} "
                f"gmax={gmax:.3f}→{gmax_mm:.1f}mm rise={g_rise:.3f} "
                f"exec={m}/{actions.shape[0]} "
                f"play={1000.0 * step_dt:.0f}ms "
                f"pipe={'Y' if pipeline else 'n'} prefetch={pre} "
                f"grip={'close' if want_closed else 'open'} "
                f"intent={'Y' if intent_close else 'n'} "
                f"g={np.round(g_all[:10], 3).tolist()}"
            )

            if run_log is not None:
                run_log.log_infer(
                    t_mono=time.monotonic(),
                    phase="policy",
                    infer_i=infer_i,
                    rtt_ms=rtt_ms,
                    server_ms=None,
                    g_obs=obs_g,
                    g0=float(g_all[0]) if g_all.size else float("nan"),
                    gmax=gmax,
                    rise=g_rise,
                    want_closed=want_closed,
                    intent_close=intent_close,
                    exec_n=m,
                    actions=actions,
                    qnow=q_now,
                )

            jump_limit_rad = np.deg2rad(float(args.max_joint_jump_deg))
            if (
                q_now is not None
                and tracker is not None
                and jump_limit_rad > 0
                and max_joint_jump_rad(raw_chunk[0, :7], q_now) > jump_limit_rad
            ):
                print(
                    f"ABORT: first joint target {dq0_deg:.1f} deg from "
                    f"measured (limit {args.max_joint_jump_deg:.0f} deg)"
                )
                return None

            chunk = blend_chunk(raw_chunk, prev_q, blend_steps)
            return chunk, step_dt

        def play_row(row, next_row, step_dt):
            nonlocal prev_q, last_dq, close_streak
            nonlocal prev_grip_state, last_cmd_width, want_closed

            g_step = float(row[7])
            if g_step >= close_g:
                close_streak += 1
            else:
                close_streak = 0
            if close_streak >= GRIPPER_CONFIRM_STEPS:
                want_closed = True

            if enable_grip:
                try:
                    prev_grip_state, _, last_cmd_width = apply_gripper(
                        gripper,
                        want_closed=want_closed,
                        g_cmd=g_step,
                        max_width=max_grip,
                        prev_state=prev_grip_state,
                        mode=args.gripper_mode,
                        last_width=last_cmd_width,
                    )
                except Exception as e:
                    print(f"  gripper error: {e}")

            q_measured = None
            if robot is not None:
                try:
                    q_measured = np.asarray(robot.state.q, dtype=np.float64)
                except Exception:
                    pass

            q_tgt = clip_target(
                row[:7],
                prev_q,
                q_measured=q_measured,
                max_tracking_error=max_tracking_error_rad,
            )
            q_start = (
                prev_q
                if prev_q is not None
                else (q_measured if q_measured is not None else q_tgt)
            )
            if next_row is not None:
                q_next = clip_target(
                    next_row[:7],
                    q_tgt,
                    q_measured=q_measured,
                    max_tracking_error=max_tracking_error_rad,
                )
                dq_end = (q_next - q_tgt) / max(step_dt, 1e-3)
            else:
                dq_end = (q_tgt - q_start) / max(step_dt, 1e-3)

            state_history.record(
                q_measured if q_measured is not None else q_tgt
            )

            prev_q, _, last_dq = play_joint_segment(
                tracker,
                q_start,
                q_tgt,
                step_dt,
                interp_hz,
                lambda: running,
                dq_start=last_dq,
                dq_end=dq_end,
            )

        pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vla-infer")
        pending = None
        try:
            actions, rtt_ms, obs_arm, obs_g, n_hist = pool.submit(do_infer).result()
            last_rtt_ms = rtt_ms
            packed = consume_reply(actions, rtt_ms, obs_arm, obs_g, n_hist)
            if packed is None:
                chunk = None
            else:
                chunk, step_dt = packed
            cursor = 0

            while running and chunk is not None:
                n = int(chunk.shape[0])
                prefetch = (
                    prefetch_arg
                    if prefetch_arg > 0
                    else auto_prefetch(last_rtt_ms, step_dt, n)
                )
                if (
                    pipeline
                    and pending is None
                    and cursor >= n - prefetch
                    and cursor < n
                ):
                    pending = pool.submit(do_infer)

                if cursor >= n:
                    if args.once:
                        break
                    if pending is not None:
                        if prev_q is not None:
                            prev_q, _, last_dq = coast_while(
                                tracker,
                                prev_q,
                                last_dq,
                                interp_hz,
                                lambda: running,
                                lambda: pending.done(),
                                max_s=1.0,
                            )
                        actions, rtt_ms, obs_arm, obs_g, n_hist = pending.result()
                        pending = None
                    else:
                        actions, rtt_ms, obs_arm, obs_g, n_hist = pool.submit(
                            do_infer
                        ).result()
                    last_rtt_ms = rtt_ms
                    packed = consume_reply(actions, rtt_ms, obs_arm, obs_g, n_hist)
                    if packed is None:
                        break
                    chunk, step_dt = packed
                    cursor = 0
                    if tracker is not None and not tracker.is_running:
                        print("Tracker stopped")
                        break
                    continue

                next_row = chunk[cursor + 1] if cursor + 1 < n else None
                play_row(chunk[cursor], next_row, step_dt)
                cursor += 1
                if tracker is not None and not tracker.is_running:
                    print("Tracker stopped")
                    break
        finally:
            if pending is not None:
                pending.cancel()
            pool.shutdown(wait=False, cancel_futures=True)

    finally:
        print("\nStopping...")
        if sampler is not None and run_log is not None:
            try:
                ts, w = sampler.drain()
                run_log.log_wrench(ts, w, phase="stop", infer_i=infer_i)
            except Exception:
                pass
        if run_log is not None:
            try:
                run_log.close()
            except Exception as e:
                print(f"  run log: {e}")
        conn.close()
        if tracker is not None:
            try:
                tracker.stop()
            except Exception as e:
                print(f"  tracker: {e}")
        if sampler is not None:
            sampler.close()
        if gripper is not None:
            try:
                gripper.stop()
            except Exception:
                pass
        if robot is not None:
            try:
                robot.recover_from_errors()
            except Exception:
                pass
        for cam in cameras.values():
            cam.close()
        print(f"Done. infers={infer_i}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
