#!/usr/bin/env python3
"""Franka3 plug_into_socket client for rlinf.serve.plug_franka3.

Runs in the gello/franky venv. Talks to the GPU serve process on localhost.
Do not import rlinf (that env has torch); protocol codec is vendored here.

Usage:
    source /home/nvidia/cxy_ws/gello_software/.venv/bin/activate
    python deploy_plug_franka3.py --host 127.0.0.1 --port 8001 --dry-run --no-robot
    python deploy_plug_franka3.py --host 127.0.0.1 --port 8001
"""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import csv
import json
import os
import signal
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import numpy as np
import websockets.sync.client

import msgpack_numpy

# Same numbers as gello teleop_collect_fr3.py / RLinf franky_controller.py.
JOINT_LIMITS_LOWER = np.array(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
)
JOINT_LIMITS_UPPER = np.array(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
)
JOINT_VEL_LIMITS = np.array([2.075, 2.075, 2.075, 2.075, 2.51, 2.51, 2.51])
JOINT_STIFFNESS = [103.75, 265.734, 227.273, 221.445, 13.5, 12.818, 5.134]
JOINT_DAMPING = [16.7, 40.263, 25.0, 12.862, 1.5, 2.0, 1.331]
MAX_JOINT_DELTA_STEP = 0.3
# Abort a chunk if step 0 is farther than this from the measured pose.
# 0.5 rad ≈ 29°. Abs models unnormalized with delta stats typically
# command ~0°, which is 1+ rad from the plug-into-socket working pose.
DEFAULT_MAX_JOINT_JUMP_DEG = 30.0

WRIST_CAMERA_SERIAL = "420122070525"
GLOBAL_CAMERA_SERIAL = "250222073513"

# Training / GELLO command: 0 = open, 1 = closed.
# franky hardware: width 0 m = closed, max_width ≈ 0.0803 m = 80.3 mm = open.
# Same idea as starVLA r1pro: never send the raw 0-1 to the motor. Binarize,
# then command hardware units (full open = max_width, full close = grasp 0 mm).
# Demo close is a slow ramp (S0 peaks ~0.28; s4a_abs ~0.16–0.24), not {0,1}.
# Receding horizon executes the first 8 steps, so the peak in steps 8–9 never
# runs and a hard 0.25 cut never fires. Latch from the *predicted* chunk:
# peak above close_g, or a clear upward ramp (g[-1]-g[0]).
DEFAULT_MAX_GRIPPER_WIDTH = 0.0803
GRIPPER_CLOSE_G = 0.18
GRIPPER_CLOSE_RISE = 0.10
GRIPPER_OPEN_G = 0.10
GRIPPER_CONFIRM_STEPS = 2
GRIPPER_CONFIRM_INFERS = 2
GRIPPER_FORCE = 50.0
GRIPPER_SPEED = 0.4
DEFAULT_EXECUTE_HORIZON = 8
POST_CLOSE_EXTRA_STEPS = 3
# Playback vs trained 15 Hz. 0.5 = each waypoint takes 2x as long.
# Inference is unchanged; only client-side execution is stretched.
DEFAULT_SPEED = 0.5
# Dense set_target between policy waypoints. 0 = one command per waypoint.
DEFAULT_INTERP_HZ = 100.0
DEFAULT_BLEND_STEPS = 4

# 0 = off. kaixin's 0.05 was Cartesian metres, not joint rad. At 15 Hz VLA
# the arm routinely lags 5–9°; a 2.9° clamp yanks the target back every step.
MAX_JOINT_TRACKING_ERROR = 0.0
RT_PRIORITY = 80

WRENCH_HZ = 100.0
IMAGE_HW = (480, 640)
DEFAULT_LOG_ROOT = Path(__file__).resolve().parent / "runs"


def apply_rt_hardening() -> None:
    """Lock memory, raise to FIFO scheduling, pin CPU affinity. Best-effort."""
    try:
        libc = ctypes.CDLL(
            ctypes.util.find_library("c") or "libc.so.6", use_errno=True
        )
        MCL_CURRENT, MCL_FUTURE = 1, 2
        if libc.mlockall(MCL_CURRENT | MCL_FUTURE) != 0:
            print(f"  rt: mlockall failed: {os.strerror(ctypes.get_errno())}")
        else:
            print("  rt: mlockall ok")
    except Exception as e:
        print(f"  rt: mlockall unavailable: {e}")
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(RT_PRIORITY))
        print(f"  rt: SCHED_FIFO priority={RT_PRIORITY}")
    except PermissionError:
        print(
            f"  rt: SCHED_FIFO denied (need rtprio>={RT_PRIORITY} in "
            "/etc/security/limits.d)"
        )
    except Exception as e:
        print(f"  rt: SCHED_FIFO failed: {e}")
    ncpu = os.cpu_count() or 1
    if ncpu >= 6:
        try:
            cpus = {0, 1} | set(range(4, ncpu))
            os.sched_setaffinity(0, cpus)
            print(f"  rt: CPU affinity → {sorted(cpus)}")
        except Exception as e:
            print(f"  rt: sched_setaffinity failed: {e}")


class CameraThread:
    """D435i color reader. Returns native 480x640 RGB uint8 (not BGR, not resized)."""

    def __init__(self, serial: str, name: str, fps: int = 30):
        import pyrealsense2 as rs

        self._name = name
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
        self._pipeline.start(config)
        self._lock = threading.Lock()
        self._frame = np.zeros((*IMAGE_HW, 3), dtype=np.uint8)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name=f"cam-{name}")
        self._thread.start()
        time.sleep(0.4)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
                bgr = np.asanyarray(frames.get_color_frame().get_data())
                rgb = bgr[:, :, ::-1].copy()
                with self._lock:
                    self._frame = rgb
            except Exception:
                pass

    def read_rgb(self) -> np.ndarray:
        with self._lock:
            return self._frame.copy()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self._pipeline.stop()
        except Exception:
            pass


class WrenchSampler:
    """100 Hz K_F_ext_hat_K buffer. Drain between infers."""

    def __init__(self, robot, hz: float = WRENCH_HZ):
        self._robot = robot
        self._dt = 1.0 / hz
        self._lock = threading.Lock()
        self._ts: list[float] = []
        self._w: list[np.ndarray] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="wrench")
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            t0 = time.perf_counter()
            try:
                wrench = np.asarray(
                    self._robot.state.K_F_ext_hat_K, dtype=np.float64
                ).reshape(6)
                now = time.monotonic()
                with self._lock:
                    self._ts.append(now)
                    self._w.append(wrench)
            except Exception:
                pass
            sleep_for = self._dt - (time.perf_counter() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def drain(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            if not self._ts:
                return (
                    np.zeros((0,), np.float64),
                    np.zeros((0, 6), np.float64),
                )
            ts = np.asarray(self._ts, dtype=np.float64)
            w = np.stack(self._w, axis=0)
            self._ts.clear()
            self._w.clear()
        return ts, w

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)


class RunLogger:
    """Append-only CSV (+ npz at close) for 100 Hz wrench and per-infer rows."""

    def __init__(self, root: Path, meta: dict):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        model = str(meta.get("model") or "unknown").replace("/", "_")
        self.dir = Path(root).expanduser() / f"{stamp}_{model}"
        self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir / "meta.json").write_text(
            json.dumps(meta, indent=2, default=str) + "\n", encoding="utf-8"
        )
        self._wf = (self.dir / "wrench.csv").open("w", newline="", encoding="utf-8")
        self._inf = (self.dir / "infer.csv").open("w", newline="", encoding="utf-8")
        self._wrench_w = csv.writer(self._wf)
        self._infer_w = csv.writer(self._inf)
        self._wrench_w.writerow(
            ["t_rel", "t_mono", "phase", "infer_i", "fx", "fy", "fz", "tx", "ty", "tz"]
        )
        self._infer_w.writerow(
            [
                "t_rel",
                "t_mono",
                "phase",
                "infer_i",
                "rtt_ms",
                "server_ms",
                "g_obs",
                "g0",
                "gmax",
                "rise",
                "grip_close",
                "intent_close",
                "exec_n",
                "action_dim",
                *[f"q0_{i}" for i in range(7)],
                *[f"qnow_{i}" for i in range(7)],
                "pred_fx",
                "pred_fy",
                "pred_fz",
                "pred_tx",
                "pred_ty",
                "pred_tz",
            ]
        )
        self._t0: float | None = None
        self._ts: list[float] = []
        self._w: list[list[float]] = []
        self._n_wrench = 0
        self._n_infer = 0

    def _rel(self, t_mono: float) -> float:
        t = float(t_mono)
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def log_wrench(
        self,
        timestamps: np.ndarray,
        wrenches: np.ndarray,
        *,
        phase: str,
        infer_i: int,
    ) -> None:
        ts = np.asarray(timestamps, dtype=np.float64).reshape(-1)
        w = np.asarray(wrenches, dtype=np.float64)
        if ts.size == 0:
            return
        w = w.reshape(ts.size, 6)
        infer_i = int(infer_i)
        for t, row in zip(ts, w):
            t = float(t)
            xyz = [float(x) for x in row.tolist()]
            self._wrench_w.writerow(
                [f"{self._rel(t):.6f}", f"{t:.6f}", phase, infer_i, *xyz]
            )
            self._ts.append(self._rel(t))
            self._w.append(xyz)
            self._n_wrench += 1
        self._wf.flush()

    def log_infer(
        self,
        *,
        t_mono: float,
        phase: str,
        infer_i: int,
        rtt_ms: float,
        server_ms: float | None,
        g_obs: float,
        g0: float,
        gmax: float,
        rise: float,
        want_closed: bool,
        intent_close: bool,
        exec_n: int,
        actions: np.ndarray,
        qnow: np.ndarray | None,
    ) -> None:
        act = np.asarray(actions, dtype=np.float64)
        if act.ndim == 1:
            act = act.reshape(1, -1)
        q0 = act[0, :7] if act.size else np.full(7, np.nan)
        if qnow is None:
            qnow_v = np.full(7, np.nan)
        else:
            qnow_v = np.asarray(qnow, dtype=np.float64).reshape(7)
        pred = np.full(6, np.nan)
        if act.shape[1] >= 14:
            pred = act[0, 8:14]
        t_mono = float(t_mono)
        self._infer_w.writerow(
            [
                f"{self._rel(t_mono):.6f}",
                f"{t_mono:.6f}",
                phase,
                int(infer_i),
                f"{float(rtt_ms):.3f}",
                "" if server_ms is None else f"{float(server_ms):.3f}",
                f"{float(g_obs):.6f}",
                f"{float(g0):.6f}",
                f"{float(gmax):.6f}",
                f"{float(rise):.6f}",
                int(bool(want_closed)),
                int(bool(intent_close)),
                int(exec_n),
                int(act.shape[1]) if act.ndim == 2 else 0,
                *[f"{float(x):.6f}" for x in q0],
                *[f"{float(x):.6f}" for x in qnow_v],
                *[f"{float(x):.6f}" for x in pred],
            ]
        )
        self._inf.flush()
        self._n_infer += 1

    def close(self) -> None:
        try:
            if self._ts:
                np.savez_compressed(
                    self.dir / "wrench.npz",
                    t_rel=np.asarray(self._ts, dtype=np.float64),
                    wrench=np.asarray(self._w, dtype=np.float64),
                )
        except Exception as e:
            print(f"  log npz: {e}")
        for fh in (self._wf, self._inf):
            try:
                fh.close()
            except Exception:
                pass
        print(
            f"  logged {self._n_wrench} wrench samples, "
            f"{self._n_infer} infers → {self.dir}"
        )


class PolicySocket:
    def __init__(self, host: str, port: int):
        uri = f"ws://{host}:{port}"
        self._ws = websockets.sync.client.connect(
            uri, compression=None, max_size=None
        )
        self._packer = msgpack_numpy.Packer()
        raw = self._ws.recv()
        if isinstance(raw, str):
            raise RuntimeError(raw)
        self.metadata = msgpack_numpy.unpackb(raw)

    def infer(self, obs: dict) -> dict:
        self._ws.send(self._packer.pack(obs))
        raw = self._ws.recv()
        if isinstance(raw, str):
            raise RuntimeError(raw)
        return msgpack_numpy.unpackb(raw)

    def close(self) -> None:
        try:
            self._ws.close()
        except Exception:
            pass


def width_to_gello(width: float, max_width: float) -> float:
    """franky meters → GELLO 0=open / 1=closed. Full open 80.3 mm → ~0."""
    mw = max(float(max_width), 1e-6)
    return float(np.clip(1.0 - float(width) / mw, 0.0, 1.0))


def gello_to_width(g: float, max_width: float) -> float:
    """GELLO 0=open / 1=closed → franky meters. 0 → 80.3 mm, 1 → 0 mm."""
    mw = max(float(max_width), 0.0)
    return float(np.clip(1.0 - float(g), 0.0, 1.0)) * mw


def gripper_close_intent(
    g_all: np.ndarray,
    *,
    close_g: float = GRIPPER_CLOSE_G,
    rise_min: float = GRIPPER_CLOSE_RISE,
    rise_peak_min: float = GRIPPER_OPEN_G,
) -> bool:
    """Whether the 10-step gripper chunk is planning to close.

    Training close is a ramp, not a step to 1.0. The peak often sits in the
    last 2 steps (dropped by execute_horizon) and stays below 0.25, which
    used to deadlock: never close → g_obs stays 0 → next chunk ramps again.
    """
    g = np.asarray(g_all, dtype=np.float64).reshape(-1)
    if g.size == 0:
        return False
    gmax = float(np.max(g))
    rise = float(g[-1] - g[0])
    if gmax >= close_g:
        return True
    return rise >= rise_min and gmax >= rise_peak_min


def apply_gripper(
    gripper,
    *,
    want_closed: bool,
    g_cmd: float,
    max_width: float,
    prev_state: int | None,
    mode: str,
    last_width: float | None,
) -> tuple[int, bool, float | None]:
    """Map 0-1 → hardware millimetres. Returns (state, just_closed, last_width).

    binary (default, R1Pro-style): {open, closed} → {max_width, grasp 0 mm}.
    width: move to (1-g)*max_width when it changes ≥3 mm; still grasp when
    want_closed so a 0.28 ramp does not stop at ~58 mm (too wide for the plug).
    """
    linear_m = gello_to_width(g_cmd, max_width)
    if mode == "width" and not want_closed:
        width = float(np.clip(linear_m, 0.0, max_width))
        if last_width is None or abs(width - last_width) >= 0.003:
            gripper.move_async(width, speed=0.3)
            print(
                f"  gripper: move {width * 1000:.1f} mm  "
                f"(g={g_cmd:.3f}, max={max_width * 1000:.1f} mm)"
            )
            last_width = width
        return 1, False, last_width
    if want_closed:
        if prev_state != 2:
            gripper.grasp_async(
                width=0.0,
                speed=GRIPPER_SPEED,
                force=GRIPPER_FORCE,
                epsilon_inner=0.08,
                epsilon_outer=0.08,
            )
            print(
                f"  gripper: grasp 0 mm  "
                f"(g={g_cmd:.3f} linear={linear_m * 1000:.1f} mm, "
                f"max={max_width * 1000:.1f} mm)"
            )
            return 2, True, 0.0
        return 2, False, last_width
    if prev_state not in (0, None):
        gripper.move_async(max_width, speed=GRIPPER_SPEED)
        print(f"  gripper: open {max_width * 1000:.1f} mm")
        return 0, False, max_width
    return 0, False, last_width


def clip_target(
    q: np.ndarray,
    prev: np.ndarray | None,
    q_measured: np.ndarray | None = None,
    max_tracking_error: float = MAX_JOINT_TRACKING_ERROR,
) -> np.ndarray:
    q = np.clip(np.asarray(q, dtype=np.float64), JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
    if prev is not None:
        delta = q - prev
        d = np.abs(delta)
        scale = np.where(
            d > MAX_JOINT_DELTA_STEP, MAX_JOINT_DELTA_STEP / np.maximum(d, 1e-9), 1.0
        )
        q = prev + delta * scale
    if q_measured is not None and max_tracking_error > 0:
        q_meas = np.asarray(q_measured, dtype=np.float64)
        q = np.clip(q, q_meas - max_tracking_error, q_meas + max_tracking_error)
    return q


def hermite_q_dq(
    q0: np.ndarray,
    dq0: np.ndarray,
    q1: np.ndarray,
    dq1: np.ndarray,
    s: float,
    duration: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Cubic Hermite: position and velocity at s in [0, 1]."""
    s = float(np.clip(s, 0.0, 1.0))
    t = max(float(duration), 1e-6)
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    dq0 = np.asarray(dq0, dtype=np.float64)
    dq1 = np.asarray(dq1, dtype=np.float64)
    q = h00 * q0 + h10 * (t * dq0) + h01 * q1 + h11 * (t * dq1)
    dh00 = 6.0 * s2 - 6.0 * s
    dh10 = 3.0 * s2 - 4.0 * s + 1.0
    dh01 = -6.0 * s2 + 6.0 * s
    dh11 = 3.0 * s2 - 2.0 * s
    dq = (dh00 * q0 + dh10 * (t * dq0) + dh01 * q1 + dh11 * (t * dq1)) / t
    return q, np.clip(dq, -JOINT_VEL_LIMITS, JOINT_VEL_LIMITS)


def play_joint_segment(
    tracker,
    q_start: np.ndarray,
    q_end: np.ndarray,
    duration: float,
    interp_hz: float,
    still_running,
    dq_start: np.ndarray | None = None,
    dq_end: np.ndarray | None = None,
) -> tuple[np.ndarray, float, np.ndarray]:
    """Hermite-interpolate q_start → q_end over duration; send at interp_hz.

    Returns (last commanded q, last timestamp, last commanded dq).
    """
    q_start = np.asarray(q_start, dtype=np.float64).reshape(7)
    q_end = np.asarray(q_end, dtype=np.float64).reshape(7)
    duration = max(float(duration), 1e-3)
    if dq_start is None:
        dq_start = (q_end - q_start) / duration
    else:
        dq_start = np.asarray(dq_start, dtype=np.float64).reshape(7)
    if dq_end is None:
        dq_end = dq_start.copy()
    else:
        dq_end = np.asarray(dq_end, dtype=np.float64).reshape(7)
    dq_start = np.clip(dq_start, -JOINT_VEL_LIMITS, JOINT_VEL_LIMITS)
    dq_end = np.clip(dq_end, -JOINT_VEL_LIMITS, JOINT_VEL_LIMITS)
    n_sub = 1 if interp_hz <= 0 else max(1, int(round(duration * float(interp_hz))))
    t0 = time.perf_counter()
    q_cmd = q_end
    dq_cmd = dq_end
    ts = t0
    for k in range(1, n_sub + 1):
        if not still_running():
            break
        s = k / n_sub
        q_cmd, dq_cmd = hermite_q_dq(
            q_start, dq_start, q_end, dq_end, s, duration
        )
        q_cmd = np.clip(q_cmd, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
        if tracker is not None:
            tracker.set_target(q_cmd, dq=dq_cmd)
        ts = time.perf_counter()
        remain = t0 + duration * s - ts
        if remain > 0:
            time.sleep(remain)
            ts = time.perf_counter()
    return q_cmd.copy(), ts, dq_cmd.copy()


def coast_while(
    tracker,
    q: np.ndarray,
    dq: np.ndarray,
    interp_hz: float,
    still_running,
    should_stop,
    max_s: float = 0.5,
) -> tuple[np.ndarray, float, np.ndarray]:
    """Keep streaming the last velocity until should_stop() or max_s."""
    q = np.asarray(q, dtype=np.float64).reshape(7)
    dq = np.clip(
        np.asarray(dq, dtype=np.float64).reshape(7),
        -JOINT_VEL_LIMITS,
        JOINT_VEL_LIMITS,
    )
    dt_tick = 1.0 / max(float(interp_hz), 15.0)
    t_end = time.perf_counter() + max(float(max_s), dt_tick)
    ts = time.perf_counter()
    while still_running() and not should_stop():
        if time.perf_counter() >= t_end:
            break
        t_tick = time.perf_counter()
        q = np.clip(q + dq * dt_tick, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
        if tracker is not None:
            tracker.set_target(q, dq=dq)
        remain = dt_tick - (time.perf_counter() - t_tick)
        if remain > 0:
            time.sleep(remain)
        ts = time.perf_counter()
    return q.copy(), ts, dq.copy()


def blend_chunk(
    chunk: np.ndarray, q_now: np.ndarray | None, n_blend: int
) -> np.ndarray:
    """Pull the start of a new chunk toward q_now so chunk boundaries do not snap.

    i=0 is not pinned to q_now (that would waste a whole step holding still).
    Weight at row i is max(0, 1 - (i+1)/n_blend).
    """
    out = np.array(chunk, dtype=np.float64, copy=True)
    if q_now is None or n_blend <= 0 or out.shape[0] == 0:
        return out
    offset = np.asarray(q_now, dtype=np.float64).reshape(7) - out[0, :7]
    n_blend = max(1, int(n_blend))
    for i in range(out.shape[0]):
        w = max(0.0, 1.0 - (i + 1) / n_blend)
        out[i, :7] = out[i, :7] + offset * w
    return out


def auto_prefetch(rtt_ms: float, step_dt: float, horizon: int) -> int:
    need = int(np.ceil((max(float(rtt_ms), 1.0) / 1000.0) / max(step_dt, 1e-3))) + 1
    cap = max(1, int(horizon) - 1)
    return int(np.clip(need, 1, cap))


def max_joint_jump_rad(q_cmd: np.ndarray, q_now: np.ndarray) -> float:
    return float(
        np.max(
            np.abs(
                np.asarray(q_cmd, dtype=np.float64) - np.asarray(q_now, dtype=np.float64)
            )
        )
    )


def fake_images() -> tuple[np.ndarray, np.ndarray]:
    black = np.zeros((*IMAGE_HW, 3), dtype=np.uint8)
    return black, black.copy()


def server_action_mode(meta: dict) -> str:
    """delta vs abs from serve metadata (with fallbacks for older servers)."""
    mode = meta.get("action_mode")
    if mode in ("delta", "abs"):
        return str(mode)
    if "delta_joints" in meta:
        return "delta" if meta["delta_joints"] else "abs"
    name = str(meta.get("model") or "")
    return "abs" if name.endswith("_abs") else "delta"


def build_obs(
    *,
    image: np.ndarray,
    wrist: np.ndarray,
    q: np.ndarray,
    gripper_g: float,
    ee_pos: np.ndarray,
    ee_quat_xyzw: np.ndarray,
    prompt: str,
    wrench_ts: np.ndarray,
    wrench: np.ndarray,
    ee_quat_convention: str,
) -> dict:
    obs = {
        "image": np.ascontiguousarray(image, dtype=np.uint8),
        "wrist_image": np.ascontiguousarray(wrist, dtype=np.uint8),
        "joint_positions": np.asarray(q, dtype=np.float32).reshape(7),
        "gripper": np.float32(gripper_g),
        "ee_pos": np.asarray(ee_pos, dtype=np.float32).reshape(3),
        "ee_quat": np.asarray(ee_quat_xyzw, dtype=np.float32).reshape(4),
        "ee_quat_convention": ee_quat_convention,
        "prompt": prompt,
    }
    if wrench_ts.size:
        obs["wrench_samples"] = np.asarray(wrench, dtype=np.float32)
        obs["wrench_timestamps"] = np.asarray(wrench_ts, dtype=np.float64)
    return obs


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8001)
    p.add_argument("--robot-ip", default="172.16.0.2")
    p.add_argument("--prompt", default="plug into socket")
    p.add_argument(
        "--ee-quat-convention",
        default="wxyz",
        choices=("wxyz", "xyzw"),
        help=(
            "Told to the server. This machine's HDF5 stored franky xyzw without "
            "reordering, so default wxyz = pass-through. Use xyzw if you want "
            "the server to convert Eigen xyzw → training wxyz."
        ),
    )
    p.add_argument("--dry-run", action="store_true", help="Do not start the tracker")
    p.add_argument("--no-robot", action="store_true", help="Fake proprio, no franky")
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--skip-tare", action="store_true")
    p.add_argument("--once", action="store_true", help="One infer then exit")
    p.add_argument(
        "--wait-enter",
        action="store_true",
        help="After tare, wait for Enter before the policy loop",
    )
    p.add_argument("--home-gripper", action="store_true")
    p.add_argument(
        "--execute-horizon",
        type=int,
        default=DEFAULT_EXECUTE_HORIZON,
        help=(
            "How many of the 10 predicted joint steps to run before the next "
            "infer (default 8). Close+lift live in the later steps, so 5 is too short."
        ),
    )
    p.add_argument(
        "--speed",
        type=float,
        default=DEFAULT_SPEED,
        help=(
            "Playback scale vs the trained 15 Hz chunk. "
            f"{DEFAULT_SPEED} (default) plays each waypoint in "
            f"{1.0 / DEFAULT_SPEED:.0f}x time; 1.0 is original. "
            "Server inference is unchanged. Try 0.35 if still jerky."
        ),
    )
    p.add_argument(
        "--interp-hz",
        type=float,
        default=DEFAULT_INTERP_HZ,
        help=(
            "Send Hermite-interpolated joint targets at this rate between "
            f"policy waypoints (default {DEFAULT_INTERP_HZ:.0f}). "
            "0 = one set_target per waypoint."
        ),
    )
    p.add_argument(
        "--no-pipeline",
        action="store_true",
        help=(
            "Do not overlap the next infer with the last few executed steps. "
            "The arm will pause ~RTT between chunks (the old stutter)."
        ),
    )
    p.add_argument(
        "--prefetch",
        type=int,
        default=0,
        help=(
            "Policy steps reserved at the end of a chunk to cover inference "
            "RTT (default 0 = auto from last RTT). Ignored with --no-pipeline."
        ),
    )
    p.add_argument(
        "--blend-steps",
        type=int,
        default=DEFAULT_BLEND_STEPS,
        help=(
            "How many waypoints at the start of a new chunk are pulled toward "
            f"the current command (default {DEFAULT_BLEND_STEPS}). 0 disables."
        ),
    )
    p.add_argument(
        "--gripper-mode",
        choices=("binary", "width"),
        default="binary",
        help=(
            "binary: R1Pro-style {open,closed} → {max_width, 0 mm grasp}. "
            "width: move to (1-g)*max_width mm; still grasp when close latches."
        ),
    )
    p.add_argument(
        "--gripper-close-g",
        type=float,
        default=GRIPPER_CLOSE_G,
        help=(
            "Latch close if any of the 10 predicted gripper steps is ≥ this "
            f"(GELLO 0=open, 1=closed; default {GRIPPER_CLOSE_G})."
        ),
    )
    p.add_argument(
        "--gripper-close-rise",
        type=float,
        default=GRIPPER_CLOSE_RISE,
        help=(
            "Also latch close if g[-1]-g[0] ≥ this and gmax ≥ "
            f"{GRIPPER_OPEN_G} (default {GRIPPER_CLOSE_RISE}). Catches the "
            "s4a_abs ramp that peaks ~0.16–0.24."
        ),
    )
    p.add_argument(
        "--require-action-mode",
        choices=("any", "delta", "abs"),
        default="any",
        help=(
            "Refuse to run if the server metadata action_mode does not match. "
            "run_abs.sh sets abs so a delta serve on the same port cannot move the arm."
        ),
    )
    p.add_argument(
        "--max-joint-jump-deg",
        type=float,
        default=DEFAULT_MAX_JOINT_JUMP_DEG,
        help=(
            "Abort before commanding if chunk[0] joints are farther than this "
            "from the measured pose (degrees). 0 disables. Default 30."
        ),
    )
    p.add_argument(
        "--max-tracking-error-deg",
        type=float,
        default=np.rad2deg(MAX_JOINT_TRACKING_ERROR),
        help=(
            "Clamp desired-vs-measured gap per joint (degrees). "
            "0 (default) disables. Try 8 only if the target races ahead "
            "in contact; 2.9° caused down/up stuttering on s0_abs."
        ),
    )
    p.add_argument(
        "--no-rt",
        action="store_true",
        help="Skip RT hardening (mlockall / SCHED_FIFO / CPU affinity).",
    )
    p.add_argument(
        "--log-dir",
        default=str(DEFAULT_LOG_ROOT),
        help=(
            "Directory for per-run wrench/infer CSVs "
            f"(default {DEFAULT_LOG_ROOT})."
        ),
    )
    p.add_argument(
        "--no-log",
        action="store_true",
        help="Do not write wrench / infer run logs.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    client_dir = Path(__file__).resolve().parent
    if str(client_dir) not in sys.path:
        sys.path.insert(0, str(client_dir))

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
    if max_tracking_error_rad > 0:
        print(
            f"tracking clamp: {args.max_tracking_error_deg:.1f} deg per joint "
            "(target vs measured)"
        )
    else:
        print("tracking clamp: off")

    speed = float(args.speed)
    if speed <= 0:
        print(f"refusing: --speed must be > 0 (got {speed})")
        return 2
    interp_hz = float(args.interp_hz)

    print(f"Connecting to serve ws://{args.host}:{args.port} ...")
    sock = PolicySocket(args.host, args.port)
    meta = sock.metadata
    hz = float(meta.get("control_hz", 15))
    execute_horizon = int(args.execute_horizon)
    dt = float(meta.get("dt", 1.0 / hz))
    action_mode = server_action_mode(meta)
    if args.require_action_mode != "any" and action_mode != args.require_action_mode:
        print(
            f"refusing: server model={meta.get('model')} action_mode={action_mode} "
            f"but client --require-action-mode={args.require_action_mode}. "
            "Delta and abs checkpoints are not interchangeable."
        )
        sock.close()
        return 2
    print(
        f"metadata model={meta.get('model')} dummy={meta.get('dummy')} "
        f"action_mode={action_mode} delta_joints={meta.get('delta_joints')} "
        f"asset_id={meta.get('asset_id')} action_dim={meta.get('action_dim')} "
        f"control_hz={hz} execute_horizon={execute_horizon} "
        f"(server suggested {meta.get('execute_horizon')}) "
        f"speed={speed:.2f} play={1000.0 * dt / speed:.0f}ms/step "
        f"interp={interp_hz:.0f}Hz hermite "
        f"pipeline={'off' if args.no_pipeline else 'on'} "
        f"blend={int(args.blend_steps)} "
        f"gripper_mode={args.gripper_mode} force_encoder={meta.get('force_encoder')} "
        f"force_ablate={meta.get('force_ablate', 'none')}"
    )

    infer_i = 0
    step_i = 0
    robot = gripper = tracker = sampler = None
    cameras: dict[str, CameraThread] = {}
    max_grip = DEFAULT_MAX_GRIPPER_WIDTH
    last_gello_g = 0.0
    prev_q = None
    prev_ts = None
    prev_grip_state = None
    last_cmd_width = None
    close_streak = 0
    infer_close_streak = 0
    open_infer_streak = 0
    want_closed = False
    close_g = float(args.gripper_close_g)
    close_rise = float(args.gripper_close_rise)
    run_log: RunLogger | None = None
    if not args.no_log:
        log_meta = {
            "host": args.host,
            "port": args.port,
            "robot_ip": args.robot_ip,
            "dry_run": args.dry_run,
            "no_robot": args.no_robot,
            "prompt": args.prompt,
            **{
                k: meta.get(k)
                for k in (
                    "model",
                    "dummy",
                    "action_mode",
                    "delta_joints",
                    "asset_id",
                    "action_dim",
                    "force_encoder",
                    "config_name",
                    "control_hz",
                    "force_ablate",
                    "force_ablate_mode",
                    "force_ablate_channels",
                    "profile_force",
                )
            },
            "speed": speed,
            "interp_hz": interp_hz,
            "execute_horizon": execute_horizon,
            "pipeline": not args.no_pipeline,
            "blend_steps": int(args.blend_steps),
            "prefetch": int(args.prefetch),
        }
        run_log = RunLogger(Path(args.log_dir), log_meta)
        print(f"run log → {run_log.dir}")

    def log_obs_wrench(obs: dict, *, phase: str, infer_i: int) -> None:
        if run_log is None:
            return
        ts = obs.get("wrench_timestamps")
        w = obs.get("wrench_samples")
        if ts is None or w is None:
            return
        run_log.log_wrench(ts, w, phase=phase, infer_i=infer_i)

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
            print(f"FR3 ok. gripper={gripper.width * 1000:.1f} mm  max={max_grip * 1000:.1f} mm")
            print(
                f"  map GELLO 0=open → {max_grip * 1000:.1f} mm,  "
                f"1=closed → 0 mm grasp  "
                f"(latch g>={args.gripper_close_g:g} or rise>={args.gripper_close_rise:g})"
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
                prev_ts = time.perf_counter()
                print("JointImpedanceTracker holding current pose")

        if not args.no_camera:
            print("Starting cameras...")
            for name, serial in (
                ("image", GLOBAL_CAMERA_SERIAL),
                ("wrist_image", WRIST_CAMERA_SERIAL),
            ):
                cameras[name] = CameraThread(serial, name=name)
                print(f"  {name} ({serial}) ready")

        def read_obs() -> dict:
            nonlocal last_gello_g
            if args.no_camera:
                image, wrist = fake_images()
            else:
                image = cameras["image"].read_rgb()
                wrist = cameras["wrist_image"].read_rgb()
            if robot is None:
                q = np.zeros(7, np.float32)
                pos = np.zeros(3, np.float32)
                quat = np.array([0, 0, 0, 1], np.float32)
                g = last_gello_g
            else:
                s = robot.state
                q = np.array(s.q, dtype=np.float32)
                pos = np.array(s.O_T_EE.translation, dtype=np.float32)
                quat = np.array(s.O_T_EE.quaternion, dtype=np.float32)
                last_gello_g = width_to_gello(float(gripper.width), max_grip)
                g = last_gello_g
            if sampler is None:
                ts, w = np.zeros((0,), np.float64), np.zeros((0, 6), np.float64)
            else:
                ts, w = sampler.drain()
            return build_obs(
                image=image,
                wrist=wrist,
                q=q,
                gripper_g=g,
                ee_pos=pos,
                ee_quat_xyzw=quat,
                prompt=args.prompt,
                wrench_ts=ts,
                wrench=w,
                ee_quat_convention=args.ee_quat_convention,
            )

        print("Sending reset...")
        reset_reply = sock.infer({"reset": True})
        print(f"  reset tare_ready={reset_reply.get('tare_ready')}")

        if not args.skip_tare and robot is not None:
            print("Tare: hold still in free space for ~1 s (streaming wrench)...")
            t_end = time.monotonic() + 1.2
            tare_ready = False
            while running and time.monotonic() < t_end:
                obs = read_obs()
                log_obs_wrench(obs, phase="tare", infer_i=0)
                reply = sock.infer(obs)
                tare_ready = bool(reply.get("tare_ready"))
                time.sleep(1.0 / hz)
            print(f"  tare_ready={tare_ready} force_frames={reply.get('force_frames')}")
            if not tare_ready:
                print("  warning: tare not locked; continuing anyway")

        if args.wait_enter and not args.once:
            input("Tare done. Position the scene, then press Enter to run policy...")

        infer_i = 0
        step_i = 0
        last_dq = np.zeros(7, dtype=np.float64)
        last_rtt_ms = 130.0
        blend_steps = int(args.blend_steps)
        prefetch_arg = int(args.prefetch)
        pipeline = (not args.no_pipeline) and (not args.once)
        enable_grip = gripper is not None and not args.dry_run

        def do_infer(obs_in: dict):
            t0 = time.perf_counter()
            reply_out = sock.infer(obs_in)
            return reply_out, (time.perf_counter() - t0) * 1000.0

        def consume_reply(reply, infer_ms, obs_in):
            nonlocal infer_i, infer_close_streak, open_infer_streak, want_closed
            infer_i += 1
            actions = np.asarray(reply["actions"], dtype=np.float64)
            if actions.ndim == 1:
                actions = actions.reshape(1, -1)
            if actions.shape[1] < 8:
                raise RuntimeError(
                    f"server actions last dim {actions.shape[1]} < 8; "
                    "need joints[7] + gripper[1]"
                )
            if infer_i == 1 and actions.shape[1] != 8:
                print(
                    f"  note: actions {tuple(actions.shape)}; using [:, :8] for "
                    "joints+gripper (trailing dims ignored, e.g. s4a_full force head)"
                )
            m = int(execute_horizon)
            step_dt = float(reply.get("dt", dt)) / speed
            g_all = np.asarray(actions[:, 7], dtype=np.float64).reshape(-1)
            g_obs = float(np.asarray(obs_in.get("gripper", np.nan)))
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
                (i for i, gv in enumerate(g_all) if gv >= close_g),
                None,
            )
            if close_idx is not None:
                m = max(m, close_idx + 1 + POST_CLOSE_EXTRA_STEPS)
            m = max(1, min(m, int(actions.shape[0])))
            raw_chunk = actions[:m]
            server_ms = (reply.get("server_timing") or {}).get("infer_ms")
            extra = f" server={server_ms:.0f}ms" if server_ms is not None else ""
            ablate = reply.get("force_ablate") or meta.get("force_ablate")
            if ablate and ablate != "none":
                extra += f" ablate={ablate}"
            gmax_mm = gello_to_width(gmax, max_grip) * 1000.0
            if infer_close_streak >= GRIPPER_CONFIRM_INFERS:
                if not want_closed:
                    print(
                        f"  gripper: latch close  "
                        f"gmax={gmax:.3f} rise={g_rise:.3f} "
                        f"(streak={infer_close_streak})"
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
            qnow_s = (
                f" qnow={np.rad2deg(q_now).round(1)}" if q_now is not None else ""
            )
            pre = auto_prefetch(infer_ms, step_dt, m) if prefetch_arg <= 0 else prefetch_arg
            print(
                f"[{infer_i:4d}] rtt={infer_ms:.0f}ms{extra} "
                f"tare={reply.get('tare_ready')} frames={reply.get('force_frames')} "
                f"q0={np.rad2deg(raw_chunk[0, :7]).round(1)}{qnow_s} "
                f"dq0={dq0_deg:.1f}deg "
                f"g_obs={g_obs:.3f} g0={g_all[0]:.3f} "
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
                    rtt_ms=infer_ms,
                    server_ms=server_ms,
                    g_obs=g_obs,
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
                    "ABORT: first joint target is "
                    f"{dq0_deg:.1f} deg from the measured pose "
                    f"(limit {args.max_joint_jump_deg:.0f} deg). "
                    "Not sending set_target. Typical cause: abs actions "
                    "unnormalized with delta quantile stats, so q0≈0°."
                )
                print(f"  now_deg={np.rad2deg(q_now).round(1)}")
                print(f"  cmd_deg={np.rad2deg(raw_chunk[0, :7]).round(1)}")
                return None
            chunk = blend_chunk(raw_chunk, prev_q, blend_steps)
            return chunk, step_dt

        def play_row(row, next_row, step_dt):
            nonlocal prev_q, prev_ts, last_dq, step_i, close_streak
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
                row[:7], prev_q,
                q_measured=q_measured,
                max_tracking_error=max_tracking_error_rad,
            )
            if prev_q is not None:
                q_start = prev_q
            elif q_measured is not None:
                q_start = q_measured
            else:
                q_start = q_tgt
            if next_row is not None:
                q_next = clip_target(
                    next_row[:7], q_tgt,
                    q_measured=q_measured,
                    max_tracking_error=max_tracking_error_rad,
                )
                dq_end = (q_next - q_tgt) / max(step_dt, 1e-3)
            else:
                dq_end = (q_tgt - q_start) / max(step_dt, 1e-3)
            prev_q, prev_ts, last_dq = play_joint_segment(
                tracker,
                q_start,
                q_tgt,
                step_dt,
                interp_hz,
                lambda: running,
                dq_start=last_dq,
                dq_end=dq_end,
            )
            step_i += 1

        pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="plug-infer")
        pending = None
        pending_obs = None
        try:
            obs = read_obs()
            log_obs_wrench(obs, phase="policy", infer_i=infer_i + 1)
            reply, infer_ms = pool.submit(do_infer, obs).result()
            last_rtt_ms = infer_ms
            packed = consume_reply(reply, infer_ms, obs)
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
                    obs = read_obs()
                    log_obs_wrench(obs, phase="policy", infer_i=infer_i + 1)
                    pending_obs = obs
                    pending = pool.submit(do_infer, obs)

                if cursor >= n:
                    if args.once:
                        break
                    if pending is not None:
                        if prev_q is not None:
                            prev_q, prev_ts, last_dq = coast_while(
                                tracker,
                                prev_q,
                                last_dq,
                                interp_hz,
                                lambda: running,
                                lambda: pending.done(),
                                max_s=1.0,
                            )
                        reply, infer_ms = pending.result()
                        obs = pending_obs
                        pending = None
                        pending_obs = None
                    else:
                        obs = read_obs()
                        log_obs_wrench(obs, phase="policy", infer_i=infer_i + 1)
                        reply, infer_ms = pool.submit(do_infer, obs).result()
                    last_rtt_ms = infer_ms
                    packed = consume_reply(reply, infer_ms, obs)
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
        sock.close()
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
