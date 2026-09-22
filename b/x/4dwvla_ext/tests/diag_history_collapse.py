#!/usr/bin/env python3
"""Probe: does the keypoint history drive the live policy into a stuck state?

The live run's very first action (empty history) led the state by 0.097 rad,
which matches what diag_input_ablation.py measures for live inputs and is close
to the demonstrations' own 0.126 rad lead. Averaged over 600 steps the lead was
only 0.016 rad. So the collapse happens *after* the first inference, as history
accumulates -- a candidate self-reinforcing loop, where a history saying
"nothing moved" makes the policy predict "nothing moves".

This feeds the server histories of increasing length and reports the resulting
lead, for two history sources:

  live: the poses the robot actually visited during the 2026-09-18 600-step run
  demo: the poses a demonstration visits over the same number of frames

If only the live history collapses the lead, the history is the mechanism and
the fix belongs in the live control path, not the model.

    /opt/venv/4dwvla/bin/python \
        /workspace/RLinf/b/x/4dwvla_ext/tests/diag_history_collapse.py \
        --dataset-path /tmp/plug_ds \
        --client-log /workspace/RLinf/b/x/4dwvla_ext/logs/client_20260918_084136_3629.log \
        --demo-global /tmp/plug_ds_h264/global.mp4 \
        --demo-wrist /tmp/plug_ds_h264/wrist.mp4
"""
from __future__ import annotations

import argparse
import glob
import re
import sys
from multiprocessing.connection import Client
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

AUTHKEY = b"4dwvla-eval"


def read_video_frame(video: Path, index: int) -> np.ndarray:
    cap = cv2.VideoCapture(str(video))
    frame = None
    for i in range(index + 1):
        ret, f = cap.read()
        if not ret:
            break
        if i == index:
            frame = f
    cap.release()
    if frame is None:
        raise RuntimeError(f"could not read frame {index} of {video}")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


def load_live_states(log: Path) -> np.ndarray:
    text = log.read_text(encoding="utf-8", errors="replace")
    rows = re.findall(r"state_after=\[([^\]]+)\]", text)
    return np.array(
        [[float(v) for v in r.replace(",", " ").split()] for r in rows]
    )[:, :7]


def run_with_history(conn, images, poses: np.ndarray, probe_arm, width, n_exec: int):
    """Accumulate ``poses`` as history n_exec at a time, then probe."""
    conn.send({"command": "reset"})
    assert conn.recv().get("status") == "ok"

    base = {
        "images": images,
        "task": "plug into socket",
        "protocol": 2,
    }
    for start in range(0, len(poses), n_exec):
        chunk = poses[start:start + n_exec]
        msg = dict(base)
        msg["state"] = {"arm": chunk[0].tolist(), "gripper": [float(width)]}
        msg["state_history"] = [p.tolist() for p in chunk]
        conn.send(msg)
        resp = conn.recv()
        if resp.get("status") != "ok":
            raise RuntimeError(f"server error: {resp.get('status')}")

    msg = dict(base)
    msg["state"] = {"arm": list(map(float, probe_arm)), "gripper": [float(width)]}
    msg["state_history"] = []
    conn.send(msg)
    resp = conn.recv()
    if resp.get("status") != "ok":
        raise RuntimeError(f"server error: {resp.get('status')}")
    actions = np.asarray(resp["actions"], dtype=np.float64)
    lead = float(np.linalg.norm(actions[0, :7] - np.asarray(probe_arm, dtype=np.float64)))
    return lead, float(actions[0, 7])


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset-path", required=True, type=Path)
    p.add_argument("--client-log", required=True, type=Path)
    p.add_argument("--demo-global", required=True, type=Path)
    p.add_argument("--demo-wrist", required=True, type=Path)
    p.add_argument("--episode", type=int, default=0)
    p.add_argument("--n-exec", type=int, default=5)
    p.add_argument("--server-host", default="localhost")
    p.add_argument("--server-port", type=int, default=5555)
    args = p.parse_args()

    data = pd.concat(
        [pd.read_parquet(x) for x in sorted(glob.glob(str(args.dataset_path / "data" / "*" / "*.parquet")))],
        ignore_index=True,
    )
    meta = pd.concat(
        [pd.read_parquet(x) for x in sorted(glob.glob(str(args.dataset_path / "meta" / "episodes" / "*" / "*.parquet")))],
        ignore_index=True,
    )
    offset = int(meta[meta["episode_index"] == args.episode]["dataset_from_index"].iloc[0])
    ep = data[data["episode_index"] == args.episode].sort_values("frame_index")
    demo_states = np.stack(ep["observation.state.arm"].values).astype(np.float64)
    demo_width = float(np.stack(ep["observation.state.gripper"].values).reshape(-1)[0])

    live_states = load_live_states(args.client_log)
    print(f"\nlive states from log: {len(live_states)}")
    print(f"demo states:          {len(demo_states)}")
    dn_live = np.linalg.norm(np.diff(live_states, axis=0), axis=1)
    dn_demo = np.linalg.norm(np.diff(demo_states, axis=0), axis=1)
    print(f"per-frame |dq|: live median={np.median(dn_live):.5f}  "
          f"demo median={np.median(dn_demo):.5f} rad")

    imgs = {
        "global": read_video_frame(args.demo_global, offset),
        "wrist": read_video_frame(args.demo_wrist, offset),
    }

    conn = Client((args.server_host, args.server_port), authkey=AUTHKEY)
    print("\nimages held fixed at the demo's frame 0, so only history varies.")
    print(f"{'hist_len':>9s} {'live lead':>11s} {'live a':>8s} "
          f"{'demo lead':>11s} {'demo a':>8s}")
    for h in (0, 5, 20, 50, 100, 200):
        ll, la = run_with_history(conn, imgs, live_states[:h], live_states[0], 0.0664, args.n_exec)
        dl, da = run_with_history(conn, imgs, demo_states[:h], demo_states[0], demo_width, args.n_exec)
        print(f"{h:9d} {ll:11.5f} {la:8.4f} {dl:11.5f} {da:8.4f}")
    conn.close()
    print("\nreference: demo recorded lead at frame 0 = 0.12619 rad, "
          "live run 600-step mean lead = 0.01590 rad")
    return 0


if __name__ == "__main__":
    sys.exit(main())
