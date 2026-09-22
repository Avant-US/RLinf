#!/usr/bin/env python3
"""Probe: which joint's excursion stalls the policy?

diag_input_ablation.py narrowed the live failure to the arm pose rather than
the cameras: with the demonstrations' own images but the pose the arm was stuck
in, the action lead drops to 0.32x and the direction cosine goes to -0.575,
while live images on a demonstration pose stay at +0.981. So some component of
that pose is off the manifold the policy was trained on.

This holds the images fixed at the demonstration's frame 0 and varies only the
state: first restoring one joint at a time from the stuck pose back to HOME,
then walking the straight line between them. The joint whose restoration alone
recovers the lead is the one to constrain in the control path.

    /opt/venv/4dwvla/bin/python \
        /workspace/RLinf/b/x/4dwvla_ext/tests/diag_pose_attractor.py \
        --dataset-path /tmp/plug_ds \
        --demo-global /tmp/plug_ds_h264/global.mp4 \
        --demo-wrist /tmp/plug_ds_h264/wrist.mp4 \
        --stuck-from /workspace/RLinf/b/x/4dwvla_ext/logs/live_capture/capture_meta.json
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


def probe(conn, images, arm, width):
    conn.send({"command": "reset"})
    assert conn.recv().get("status") == "ok"
    conn.send({
        "images": images,
        "state": {"arm": list(map(float, arm)), "gripper": [float(width)]},
        "state_history": [],
        "task": "plug into socket",
        "protocol": 2,
    })
    resp = conn.recv()
    if resp.get("status") != "ok":
        raise RuntimeError(f"server error: {resp.get('status')}")
    actions = np.asarray(resp["actions"], dtype=np.float64)
    d = actions[0, :7] - np.asarray(arm, dtype=np.float64)
    return float(np.linalg.norm(d)), float(actions[0, 7]), d


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset-path", required=True, type=Path)
    p.add_argument("--demo-global", required=True, type=Path)
    p.add_argument("--demo-wrist", required=True, type=Path)
    p.add_argument("--stuck-from", required=True, type=Path)
    p.add_argument("--home-pose", type=Path, default=None)
    p.add_argument("--episode", type=int, default=0)
    p.add_argument("--server-host", default="localhost")
    p.add_argument("--server-port", type=int, default=5555)
    p.add_argument(
        "--sweep-joint",
        type=int,
        default=None,
        help="1-based joint to sweep from the stuck pose across and beyond the "
        "training range, to size the margin the action clamp should allow",
    )
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
    demo_arm = np.stack(ep["observation.state.arm"].values)[0].astype(np.float64)
    demo_action = np.stack(ep["action.arm"].values)[0].astype(np.float64)
    demo_dir = demo_action - demo_arm

    stuck = np.asarray(
        json.loads(args.stuck_from.read_text(encoding="utf-8"))["robot_q"],
        dtype=np.float64,
    )
    home_path = args.home_pose or Path(
        "/workspace/RLinf/b/x/franky_ext/dsplug/home_pose.json"
    )
    home = np.asarray(
        json.loads(home_path.read_text(encoding="utf-8"))["joint_position_rad"],
        dtype=np.float64,
    )

    # Training range per joint, over every frame of every episode.
    all_arm = np.stack(data["observation.state.arm"].values).astype(np.float64)
    lo, hi = all_arm.min(axis=0), all_arm.max(axis=0)

    imgs = {
        "global": read_video_frame(args.demo_global, offset),
        "wrist": read_video_frame(args.demo_wrist, offset),
    }

    print("\n=== Stuck pose vs the training range (all 8 episodes, all frames) ===")
    print("joint      stuck      HOME    train_min  train_max   stuck in range?")
    for j in range(7):
        inside = lo[j] <= stuck[j] <= hi[j]
        print(f"  q{j+1}   {stuck[j]:+9.4f} {home[j]:+9.4f} {lo[j]:+10.4f} {hi[j]:+10.4f}"
              f"   {'yes' if inside else 'NO  <-- outside'}")

    conn = Client((args.server_host, args.server_port), authkey=AUTHKEY)
    print("\n=== Restore one joint at a time from stuck -> HOME ===")
    print("images fixed at the demo's frame 0, so only the state varies.\n")
    print(f"{'state':30s} {'lead(rad)':>10s} {'a_grip':>8s} {'cos(demo)':>10s}")

    def show(label, arm):
        lead, a, d = probe(conn, imgs, arm, LIVE_WIDTH_M)
        cos = float(d @ demo_dir / (np.linalg.norm(d) * np.linalg.norm(demo_dir)))
        print(f"{label:30s} {lead:10.5f} {a:8.4f} {cos:+10.3f}")
        return lead, cos

    base_lead, base_cos = show("stuck pose (baseline)", stuck)
    per_joint = {}
    for j in range(7):
        arm = stuck.copy()
        arm[j] = home[j]
        per_joint[j] = show(f"  stuck, q{j+1} restored to HOME", arm)
    home_lead, home_cos = show("HOME", home)
    demo_lead, demo_cos = show("demo frame 0", demo_arm)

    print("\n=== Straight line from stuck to HOME ===")
    print(f"{'fraction toward HOME':30s} {'lead(rad)':>10s} {'a_grip':>8s} {'cos(demo)':>10s}")
    for f in (0.0, 0.25, 0.5, 0.75, 1.0):
        show(f"  {f:.2f}", stuck + f * (home - stuck))

    if args.sweep_joint:
        j = args.sweep_joint - 1
        print(f"\n=== Sweep q{j + 1} from the stuck pose "
              f"(training range [{lo[j]:+.4f}, {hi[j]:+.4f}]) ===")
        print(f"{'q' + str(j + 1):>10s} {'vs train_min':>13s} "
              f"{'lead(rad)':>10s} {'a_grip':>8s} {'cos(demo)':>10s}")
        values = sorted({
            *np.round(np.linspace(lo[j] - 0.06, lo[j] + 0.08, 15), 4).tolist(),
            round(float(stuck[j]), 4),
            round(float(lo[j]), 4),
        })
        for v in values:
            arm = stuck.copy()
            arm[j] = v
            lead, a, d = probe(conn, imgs, arm, LIVE_WIDTH_M)
            cos = float(d @ demo_dir / (np.linalg.norm(d) * np.linalg.norm(demo_dir)))
            tag = ""
            if abs(v - stuck[j]) < 1e-9:
                tag = "  <- stuck"
            elif abs(v - lo[j]) < 1e-9:
                tag = "  <- train_min"
            print(f"{v:10.4f} {v - lo[j]:+13.4f} {lead:10.5f} {a:8.4f} {cos:+10.3f}{tag}")
    conn.close()

    print("\n=== Which single joint recovers the most? ===")
    ranked = sorted(per_joint.items(), key=lambda kv: -kv[1][0])
    for j, (lead, cos) in ranked:
        gain = (lead - base_lead) / (home_lead - base_lead) if home_lead != base_lead else float("nan")
        print(f"  q{j+1}: lead {base_lead:.5f} -> {lead:.5f} "
              f"({gain * 100:5.1f}% of the way to HOME's {home_lead:.5f}), "
              f"cos {base_cos:+.3f} -> {cos:+.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
