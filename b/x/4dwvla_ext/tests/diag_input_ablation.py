#!/usr/bin/env python3
"""Probe: which live input degenerates the policy -- the images or the state?

diag_demo_replay_arm.py established that on the demonstrations' own inputs the
policy reproduces them almost exactly (cosine +1.000, gripper close within one
frame), so the checkpoint is sound and the fault is in the live input path. The
live runs differ from the demonstrations in two measured ways: the cameras are
white-balanced cool (red channel low by 16-22 grey levels) and the wrist view
sits closer to the socket, and the episode starts from HOME rather than a
demonstration's first frame.

This runs a 2x2 over {demo, live} images x {demo, live} state and reports the
one number that separated the two regimes: how far the first action of the
chunk leads the current state. The demonstrations lead by ~0.13 rad; the live
runs led by 0.016 rad, which is the signature of a stuck policy.

    /opt/venv/4dwvla/bin/python \
        /workspace/RLinf/b/x/4dwvla_ext/tests/diag_input_ablation.py \
        --dataset-path /tmp/plug_ds \
        --demo-global /tmp/plug_ds_h264/global.mp4 \
        --demo-wrist /tmp/plug_ds_h264/wrist.mp4 \
        --live-global /workspace/RLinf/b/x/4dwvla_ext/logs/cam_global_250222073513.png \
        --live-wrist /workspace/RLinf/b/x/4dwvla_ext/logs/cam_wrist_420122070525.png
"""
from __future__ import annotations

import argparse
import glob
import json
import sys
from multiprocessing.connection import Client
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

AUTHKEY = b"4dwvla-eval"
W0 = 0.08
LIVE_WIDTH_M = 0.0664


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


def read_live_image(path: Path) -> np.ndarray:
    """Load a live frame as uint8 RGB.

    ``.npy`` is preferred: capture_live_frames.py stores the exact array the
    policy would receive, so there is no question of channel order. A ``.png``
    is assumed to be BGR on disk, which is what cv2 wrote it as.
    """
    if path.suffix == ".npy":
        img = np.load(path)
        if img.shape[:2] != (480, 640):
            img = cv2.resize(img, (640, 480))
        return img.astype(np.uint8)
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"could not read {path}")
    return cv2.cvtColor(cv2.resize(img, (640, 480)), cv2.COLOR_BGR2RGB)


def probe(conn, images, arm, width, task="plug into socket"):
    """One-shot query with an empty history; returns (lead, a, actions)."""
    conn.send({"command": "reset"})
    assert conn.recv().get("status") == "ok"
    conn.send({
        "images": images,
        "state": {"arm": list(map(float, arm)), "gripper": [float(width)]},
        "state_history": [],
        "task": task,
        "protocol": 2,
    })
    resp = conn.recv()
    if resp.get("status") != "ok":
        raise RuntimeError(f"server error: {resp.get('status')}")
    actions = np.asarray(resp["actions"], dtype=np.float64)
    lead = float(np.linalg.norm(actions[0, :7] - np.asarray(arm, dtype=np.float64)))
    return lead, float(actions[0, 7]), actions


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset-path", required=True, type=Path)
    p.add_argument("--demo-global", required=True, type=Path)
    p.add_argument("--demo-wrist", required=True, type=Path)
    p.add_argument("--live-global", required=True, type=Path)
    p.add_argument("--live-wrist", required=True, type=Path)
    p.add_argument("--episode", type=int, default=0)
    p.add_argument("--frame", type=int, default=0, help="demo frame to probe")
    p.add_argument("--home-pose", type=Path, default=None)
    p.add_argument(
        "--live-state-from",
        type=Path,
        default=None,
        help="capture_meta.json whose robot_q becomes the live state, instead "
        "of HOME; use this to probe the pose the arm was actually stuck in",
    )
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
    row = meta[meta["episode_index"] == args.episode]
    offset = int(row["dataset_from_index"].iloc[0])
    ep = data[data["episode_index"] == args.episode].sort_values("frame_index")
    demo_arm = np.stack(ep["observation.state.arm"].values)[args.frame].astype(np.float64)
    demo_width = float(np.stack(ep["observation.state.gripper"].values).reshape(-1)[args.frame])
    demo_action = np.stack(ep["action.arm"].values)[args.frame].astype(np.float64)
    demo_lead = float(np.linalg.norm(demo_action - demo_arm))

    if args.live_state_from is not None:
        meta_json = json.loads(args.live_state_from.read_text(encoding="utf-8"))
        home_arm = np.asarray(meta_json["robot_q"], dtype=np.float64)
        live_label = "LIVE(captured)"
    else:
        home_path = args.home_pose or Path(
            "/workspace/RLinf/b/x/franky_ext/dsplug/home_pose.json"
        )
        home_arm = np.asarray(
            json.loads(home_path.read_text(encoding="utf-8"))["joint_position_rad"],
            dtype=np.float64,
        )
        live_label = "LIVE(HOME)"

    demo_imgs = {
        "global": read_video_frame(args.demo_global, offset + args.frame),
        "wrist": read_video_frame(args.demo_wrist, offset + args.frame),
    }
    live_imgs = {
        "global": read_live_image(args.live_global),
        "wrist": read_live_image(args.live_wrist),
    }

    print(f"\n=== Input ablation (episode {args.episode}, frame {args.frame}) ===")
    print(f"demo recorded action lead at this frame: {demo_lead:.5f} rad")
    print("live run's observed lead (600 steps):     0.01590 rad")
    print()

    conn = Client((args.server_host, args.server_port), authkey=AUTHKEY)
    cases = [
        ("demo images + demo state", demo_imgs, demo_arm, demo_width),
        ("LIVE images + demo state", live_imgs, demo_arm, demo_width),
        (f"demo images + {live_label} state", demo_imgs, home_arm, LIVE_WIDTH_M),
        (f"LIVE images + {live_label} state", live_imgs, home_arm, LIVE_WIDTH_M),
    ]
    print(f"{'case':34s} {'lead(rad)':>10s} {'a_grip':>8s}  {'dq7':>9s}  cos(demo)")
    demo_dir = demo_action - demo_arm
    results = {}
    for name, imgs, arm, width in cases:
        lead, a, actions = probe(conn, imgs, arm, width)
        d = actions[0, :7] - np.asarray(arm, dtype=np.float64)
        cos = float(d @ demo_dir / (np.linalg.norm(d) * np.linalg.norm(demo_dir)))
        results[name] = {"lead": lead, "a_grip": a, "delta": d, "cos": cos}
        print(f"{name:34s} {lead:10.5f} {a:8.4f}  {d[6]:+9.5f}  {cos:+8.3f}")
    print(f"{'(demo recorded action)':34s} {demo_lead:10.5f} {'--':>8s}  "
          f"{demo_dir[6]:+9.5f}  {1.0:+8.3f}")
    print(f"{'(live run step 0, empty history)':34s} {0.09675:10.5f} {0.0381:8.4f}  "
          f"{-0.08958:+9.5f}  {'?':>8s}")
    conn.close()

    print()
    base = results["demo images + demo state"]["lead"]
    for name in list(results)[1:]:
        r = results[name]["lead"] / base if base > 0 else float("nan")
        print(f"  {name:34s} lead is {r:5.2f}x the demo-inputs case")
    print()
    print("  per-joint action delta (rad):")
    print("    joint          " + "".join(f"{'q'+str(j+1):>10s}" for j in range(7)))
    print("    demo recorded  " + "".join(f"{v:+10.4f}" for v in demo_dir))
    for name, r in results.items():
        print(f"    {name[:14]:14s} " + "".join(f"{v:+10.4f}" for v in r["delta"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
