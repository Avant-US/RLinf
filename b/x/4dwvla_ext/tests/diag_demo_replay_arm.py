#!/usr/bin/env python3
"""Diagnostic: does the policy reproduce the demonstrations' ARM actions?

The live runs on 2026-09-18 oscillated in place: net joint displacement was
comparable to a demonstration's but pointed in an unrelated direction (cosine
+0.09 against the demo mean), and q7 was driven below the training minimum at
step 5 and stayed there. That has two very different possible causes, and this
script separates them by feeding the policy the demonstrations' own inputs:

  - If the predicted arm actions track the recorded ones, the model and its
    preprocessing are fine and the fault is in the live input path (camera
    viewpoint, state assembly, keypoint history).
  - If they do not, the fault is upstream of the robot entirely -- checkpoint,
    normalization stats, or prompt.

Read-only with respect to the robot; it only talks to the inference server.
Unlike accept_demo_replay.py this does not send ``shutdown``, so the server
stays up for the next run.

Handles the LeRobot v3.0 layout, where every episode is concatenated into one
mp4 per camera and ``meta/episodes`` carries the frame offsets. (Note that
accept_demo_replay.py globs for per-episode video files, which this layout does
not have, so it silently replays on black images.)

    /opt/venv/4dwvla/bin/python \
        /workspace/RLinf/b/x/4dwvla_ext/tests/diag_demo_replay_arm.py \
        --dataset-path /tmp/plug_ds --episode 0 --n-exec 5
"""
from __future__ import annotations

import argparse
import glob
import json
import logging
from multiprocessing.connection import Client
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("diag-replay-arm")

AUTHKEY = b"4dwvla-eval"
W0 = 0.08


def load_episode(dataset_path: Path, episode_idx: int):
    """Return (frames, video_offset) for one episode of a v3.0 dataset."""
    data = pd.concat(
        [pd.read_parquet(p) for p in sorted(glob.glob(str(dataset_path / "data" / "*" / "*.parquet")))],
        ignore_index=True,
    )
    meta = pd.concat(
        [pd.read_parquet(p) for p in sorted(glob.glob(str(dataset_path / "meta" / "episodes" / "*" / "*.parquet")))],
        ignore_index=True,
    )
    row = meta[meta["episode_index"] == episode_idx]
    if row.empty:
        raise ValueError(f"episode {episode_idx} not in {dataset_path}")
    lo = int(row["dataset_from_index"].iloc[0])
    hi = int(row["dataset_to_index"].iloc[0])

    ep = data[data["episode_index"] == episode_idx].sort_values("frame_index")
    frames = {
        "arm": np.stack(ep["observation.state.arm"].values).astype(np.float64),
        "gripper": np.stack(ep["observation.state.gripper"].values).astype(np.float64).reshape(-1),
        "action_arm": np.stack(ep["action.arm"].values).astype(np.float64),
        "action_grip": np.stack(ep["action.gripper"].values).astype(np.float64).reshape(-1),
    }
    assert len(frames["arm"]) == hi - lo, f"{len(frames['arm'])} rows vs video span {hi - lo}"
    return frames, lo


def decode_frames(video: Path, wanted_global_indices: list[int]):
    """Sequentially decode only the frames we will actually send."""
    wanted = set(wanted_global_indices)
    out: dict[int, np.ndarray] = {}
    cap = cv2.VideoCapture(str(video))
    i = 0
    last = max(wanted)
    while cap.isOpened() and i <= last:
        ret, frame = cap.read()
        if not ret:
            break
        if i in wanted:
            out[i] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        i += 1
    cap.release()
    logger.info("decoded %d/%d requested frames from %s", len(out), len(wanted), video.name)
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset-path", required=True, type=Path)
    p.add_argument("--episode", type=int, default=0)
    p.add_argument(
        "--n-exec",
        type=int,
        default=5,
        help="must match the server's --n-exec so chunk lengths line up",
    )
    p.add_argument("--server-host", default="localhost")
    p.add_argument("--server-port", type=int, default=5555)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument(
        "--global-video",
        type=Path,
        default=None,
        help="override the global-camera video; needed when the dataset's AV1 "
        "stream cannot be decoded here (transcode it to H.264 first)",
    )
    p.add_argument("--wrist-video", type=Path, default=None, help="see --global-video")
    p.add_argument(
        "--require-images",
        action="store_true",
        help="abort instead of silently replaying on black frames",
    )
    p.add_argument(
        "--closed-loop",
        action="store_true",
        help="feed the policy its own executed poses instead of the "
        "demonstration's, modelling a perfectly tracking robot. The open-loop "
        "mode is teacher-forced and cannot expose compounding error.",
    )
    args = p.parse_args()

    frames, offset = load_episode(args.dataset_path, args.episode)
    n = len(frames["arm"])
    query_t = list(range(0, n, args.n_exec))
    logger.info("episode %d: %d frames, %d inferences", args.episode, n, len(query_t))

    gvid = args.global_video or (
        args.dataset_path / "videos" / "observation.images.global" / "chunk-000" / "file-000.mp4"
    )
    wvid = args.wrist_video or (
        args.dataset_path / "videos" / "observation.images.wrist" / "chunk-000" / "file-000.mp4"
    )
    g_imgs = decode_frames(gvid, [offset + t for t in query_t])
    w_imgs = decode_frames(wvid, [offset + t for t in query_t])
    if args.require_images and (not g_imgs or not w_imgs):
        logger.error(
            "decoded %d global / %d wrist frames; refusing to replay on black "
            "images. Transcode the video and pass --global-video/--wrist-video.",
            len(g_imgs), len(w_imgs),
        )
        return 1
    if not g_imgs or not w_imgs:
        logger.warning(
            "decoded %d global / %d wrist frames -- replaying on BLACK images; "
            "arm/gripper numbers below do not reflect the visual policy",
            len(g_imgs), len(w_imgs),
        )

    conn = Client((args.server_host, args.server_port), authkey=AUTHKEY)
    conn.send({"command": "reset"})
    resp = conn.recv()
    assert resp.get("status") == "ok", f"reset failed: {resp}"
    logger.info("server reset")

    pred_arm: list[np.ndarray] = []
    pred_grip: list[float] = []
    pred_first_of_chunk: list[tuple[int, np.ndarray]] = []
    state_history: list[list[float]] = []

    # Closed loop starts at the demonstration's first pose and thereafter uses
    # whatever the policy commanded, so error compounds exactly as on hardware.
    cl_arm = frames["arm"][0].copy()
    cl_width = float(frames["gripper"][0])

    for t in query_t:
        if args.closed_loop:
            arm, width = cl_arm, cl_width
        else:
            arm = frames["arm"][t]
            width = float(frames["gripper"][t])
        msg = {
            "images": {
                "global": g_imgs.get(offset + t, np.zeros((480, 640, 3), np.uint8)),
                "wrist": w_imgs.get(offset + t, np.zeros((480, 640, 3), np.uint8)),
            },
            "state": {"arm": arm.tolist(), "gripper": [width]},
            "state_history": state_history,
            "task": "plug into socket",
            "protocol": 2,
        }
        conn.send(msg)
        resp = conn.recv()
        if resp.get("status") != "ok":
            logger.error("server error at frame %d: %s", t, resp.get("status"))
            break
        actions = np.asarray(resp["actions"], dtype=np.float64)
        pred_first_of_chunk.append((t, actions[0, :7].copy(), arm.copy()))
        for k in range(actions.shape[0]):
            pred_arm.append(actions[k, :7])
            pred_grip.append(float(actions[k, 7]))
        # Mirror the live client: history is the poses executed since last call.
        if args.closed_loop:
            state_history = [actions[k, :7].tolist() for k in range(actions.shape[0])]
            cl_arm = actions[-1, :7].copy()
            cl_width = min(max(W0 * (1.0 - actions[-1, 7]), 0.0), W0)
        else:
            state_history = [
                frames["arm"][min(t + k, n - 1)].tolist() for k in range(args.n_exec)
            ]

    conn.close()
    if not pred_arm:
        logger.error("no predictions collected")
        return 1

    P = np.asarray(pred_arm)[:n]
    A = frames["action_arm"][: len(P)]
    S = frames["arm"][: len(P)]

    print("\n=== Predicted vs recorded ARM actions ===")
    print("joint      MAE(pred,rec)   MAE(rec,state)   ratio")
    for j in range(7):
        e = np.abs(P[:, j] - A[:, j]).mean()
        base = np.abs(A[:, j] - S[:, j]).mean()
        print(f"  q{j+1}   {e:12.5f}   {base:14.5f}   {e / base if base > 0 else float('nan'):6.2f}")

    demo_net = frames["action_arm"][-1] - frames["arm"][0]
    pred_net = P[-1] - frames["arm"][0]
    cos = float(demo_net @ pred_net / (np.linalg.norm(demo_net) * np.linalg.norm(pred_net)))
    print("\n=== Net direction over the episode ===")
    print("  demo net: " + " ".join(f"{v:+7.3f}" for v in demo_net))
    print("  pred net: " + " ".join(f"{v:+7.3f}" for v in pred_net))
    print(f"  cosine(demo, pred) = {cos:+.3f}   (live run scored +0.086)")
    print(f"  ||demo net|| = {np.linalg.norm(demo_net):.3f} rad, "
          f"||pred net|| = {np.linalg.norm(pred_net):.3f} rad")
    print(f"  q7: demo {demo_net[6]:+.3f} vs pred {pred_net[6]:+.3f} "
          f"-- {'SAME' if demo_net[6] * pred_net[6] > 0 else 'OPPOSITE'} sign")

    # Chunk-boundary discontinuity, the live run's visible shaking.
    jumps = [np.linalg.norm(a - s) for _t, a, s in pred_first_of_chunk]
    print("\n=== Chunk-boundary jump (first action vs current state) ===")
    print(f"  mean={np.mean(jumps):.5f} max={np.max(jumps):.5f} rad "
          f"(live run: mean 0.0159, max 0.0967)")

    if args.closed_loop:
        drift = [
            np.linalg.norm(s - frames["arm"][min(t, n - 1)])
            for t, _a, s in pred_first_of_chunk
        ]
        print("\n=== Closed-loop drift from the demonstration ===")
        print("  chunk  " + "".join(f"{i:>8d}" for i in range(0, len(drift), max(len(drift) // 8, 1))))
        print("  drift  " + "".join(
            f"{drift[i]:8.3f}" for i in range(0, len(drift), max(len(drift) // 8, 1))
        ))
        print(f"  final drift = {drift[-1]:.4f} rad after {len(drift)} inferences")

    demo_close = next((i for i, v in enumerate(frames["action_grip"]) if v >= 0.5), None)
    pred_close = next((i for i, v in enumerate(pred_grip) if v >= 0.5), None)
    print("\n=== Gripper channel ===")
    print(f"  demo first a>=0.5 at frame {demo_close}, pred at {pred_close}")
    print(f"  pred a range: {min(pred_grip):.4f} .. {max(pred_grip):.4f} "
          f"(live run: 0.0298 .. 0.0573)")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps({
            "episode": args.episode,
            "n_frames": n,
            "cosine_demo_pred": cos,
            "mae_per_joint": [float(np.abs(P[:, j] - A[:, j]).mean()) for j in range(7)],
            "demo_net": demo_net.tolist(),
            "pred_net": pred_net.tolist(),
            "chunk_jump_mean": float(np.mean(jumps)),
            "demo_first_close": demo_close,
            "pred_first_close": pred_close,
            "pred_grip_min": min(pred_grip),
            "pred_grip_max": max(pred_grip),
        }, indent=2), encoding="utf-8")
        logger.info("wrote %s", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
