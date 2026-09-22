#!/usr/bin/env python3
"""Demo replay acceptance test (0e): feed real demo data through the model.

A/B test that verifies the model can reproduce the demonstration gripper ramp:
  Group A (faithful): real demo images + real state (incl. real width)
  Group B (frozen):   real demo images + state with width frozen at 66.4 mm

Pass criteria (grperr_1.2.md §5 Phase 0e):
  A: first frame with a >= 0.5 within ±30 frames of demo first-close frame
  B: entire trajectory a < 0.3 (absorbing state reproduced)

Requires:
  - GPU container with 4dwvla venv
  - Checkpoint loaded in vla_inference_server.py (connect via IPC)
  - Demo dataset at DATASET_PATH

Usage:
    source /opt/venv/4dwvla/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/tests/accept_demo_replay.py \
        --server-host localhost --server-port 5555 \
        --dataset-path /home/nvidia/bt/dt/plug_into_socket_lrb_4D_8sml \
        --episode 0
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from multiprocessing.connection import Client
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vla_debug_logging import format_array

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("demo-replay")

AUTHKEY = b"4dwvla-eval"
W0 = 0.08
FROZEN_WIDTH_M = 0.0664

PASS = 0
FAIL = 0


def check(name: str, condition: bool, detail: str = ""):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}: {detail}")


def load_episode_frames(dataset_path: str, episode_idx: int):
    """Load images and state for one episode."""
    table = pq.read_table(Path(dataset_path) / "data" / "chunk-000" / "file-000.parquet")
    ep_col = np.array(table.column("episode_index").to_pylist())
    mask = ep_col == episode_idx
    indices = np.where(mask)[0]
    if len(indices) == 0:
        raise ValueError(f"Episode {episode_idx} not found")

    arm_col = table.column("observation.state.arm")
    grip_col = table.column("observation.state.gripper")
    action_grip_col = table.column("action.gripper")

    frames = []
    for i in indices:
        arm = np.array(arm_col[int(i)].as_py(), dtype=np.float64)
        grip = np.array(grip_col[int(i)].as_py(), dtype=np.float64)
        a_grip = np.array(action_grip_col[int(i)].as_py(), dtype=np.float64)
        frames.append({
            "arm": arm,
            "gripper_width": float(grip[0]),
            "action_grip": float(a_grip[0]),
        })

    # Load video frames
    videos_dir = Path(dataset_path) / "videos"
    global_video = sorted(videos_dir.glob("*global*"))[0] if list(videos_dir.glob("*global*")) else None
    wrist_video = sorted(videos_dir.glob("*wrist*"))[0] if list(videos_dir.glob("*wrist*")) else None

    # Find episode-specific videos
    ep_global = sorted(videos_dir.glob(f"chunk-000/observation.images.global_episode_{episode_idx:06d}.*"))
    ep_wrist = sorted(videos_dir.glob(f"chunk-000/observation.images.wrist_episode_{episode_idx:06d}.*"))
    if ep_global:
        global_video = ep_global[0]
    if ep_wrist:
        wrist_video = ep_wrist[0]

    global_frames = _decode_video(global_video, len(frames)) if global_video else None
    wrist_frames = _decode_video(wrist_video, len(frames)) if wrist_video else None

    return frames, global_frames, wrist_frames


def _decode_video(path: Path, expected_frames: int):
    """Decode video to list of numpy arrays (H, W, 3) uint8 RGB."""
    cap = cv2.VideoCapture(str(path))
    frames = []
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    cap.release()
    logger.info("Decoded %d frames from %s (expected %d)", len(frames), path.name, expected_frames)
    return frames


def run_replay(conn, frames, global_frames, wrist_frames,
               freeze_width: bool, n_exec: int = 10):
    """Replay demo frames through the server, return predicted actions."""
    conn.send({"command": "reset"})
    resp = conn.recv()
    assert resp.get("status") == "ok", f"Reset failed: {resp}"

    predictions = []
    state_history = []

    for t in range(0, len(frames), n_exec):
        frame = frames[min(t, len(frames) - 1)]
        arm = frame["arm"]
        width = FROZEN_WIDTH_M if freeze_width else frame["gripper_width"]
        state_8d = np.concatenate([arm, [width]])

        # Use real images if available, else black
        if global_frames and t < len(global_frames):
            img_global = global_frames[t]
        else:
            img_global = np.zeros((480, 640, 3), dtype=np.uint8)

        if wrist_frames and t < len(wrist_frames):
            img_wrist = wrist_frames[t]
        else:
            img_wrist = np.zeros((480, 640, 3), dtype=np.uint8)

        msg = {
            "images": {"global": img_global, "wrist": img_wrist},
            "state": {"arm": arm.tolist(), "gripper": [float(width)]},
            "state_history": state_history,
            "task": "plug into socket",
            "protocol": 2,
        }

        conn.send(msg)
        resp = conn.recv()
        if resp["status"] != "ok":
            logger.error("Server error at frame %d: %s", t, resp["status"])
            break

        actions = np.array(resp["actions"], dtype=np.float64)
        for step_i in range(len(actions)):
            predictions.append({
                "frame": t + step_i,
                "action_grip": float(actions[step_i, 7]) if actions.shape[1] > 7 else 0.0,
                "delta_w": float(width - W0 * (1.0 - actions[step_i, 7])) if actions.shape[1] > 7 else 0.0,
            })

        # Build state_history for next request
        state_history = []
        for step_i in range(min(n_exec, len(frames) - t)):
            f = frames[min(t + step_i, len(frames) - 1)]
            state_history.append(f["arm"].tolist())

    return predictions


def find_demo_first_close(frames):
    """Find the first frame where demo action.grip >= 0.5."""
    for i, f in enumerate(frames):
        if f["action_grip"] >= 0.5:
            return i
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--server-host", default="localhost")
    p.add_argument("--server-port", type=int, default=5555)
    p.add_argument("--dataset-path", required=True)
    p.add_argument("--episode", type=int, default=0)
    p.add_argument("--n-exec", type=int, default=10)
    p.add_argument("--output-dir", default="/workspace/RLinf/b/d/frk1/asset")
    args = p.parse_args()

    print("\n=== T9: Demo Replay A/B Acceptance Test ===")

    frames, global_frames, wrist_frames = load_episode_frames(
        args.dataset_path, args.episode
    )
    logger.info("Episode %d: %d frames", args.episode, len(frames))

    demo_first_close = find_demo_first_close(frames)
    logger.info("Demo first a>=0.5 at frame %s", demo_first_close)

    conn = Client((args.server_host, args.server_port), authkey=AUTHKEY)
    logger.info("Connected to server")

    # Group A: faithful replay (real width)
    logger.info("--- Group A: Faithful replay ---")
    pred_a = run_replay(conn, frames, global_frames, wrist_frames,
                        freeze_width=False, n_exec=args.n_exec)

    # Group B: frozen width
    logger.info("--- Group B: Frozen width (%.4f m) ---", FROZEN_WIDTH_M)
    pred_b = run_replay(conn, frames, global_frames, wrist_frames,
                        freeze_width=True, n_exec=args.n_exec)

    conn.send({"command": "shutdown"})
    conn.close()

    # Analyze Group A
    a_first_close = None
    for p_item in pred_a:
        if p_item["action_grip"] >= 0.5:
            a_first_close = p_item["frame"]
            break

    # Analyze Group B
    b_max_grip = max(p_item["action_grip"] for p_item in pred_b) if pred_b else 0.0

    print(f"\n  Group A first a>=0.5: frame {a_first_close}")
    print(f"  Demo first a>=0.5: frame {demo_first_close}")
    print(f"  Group B max grip action: {b_max_grip:.4f}")

    if demo_first_close is not None and a_first_close is not None:
        frame_diff = abs(a_first_close - demo_first_close)
        check(
            "Group A: first close within ±30 frames of demo",
            frame_diff <= 30,
            f"diff={frame_diff} (pred={a_first_close}, demo={demo_first_close})",
        )
    elif demo_first_close is not None:
        check("Group A: model produced a>=0.5", False,
              "model never reached a>=0.5 in faithful replay")
    else:
        print("  [SKIP] No a>=0.5 in demo data")

    check(
        "Group B: absorbing state reproduced (all a < 0.3)",
        b_max_grip < 0.3,
        f"max_grip={b_max_grip:.4f}",
    )

    # Save results
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    results = {
        "episode": args.episode,
        "n_frames": len(frames),
        "demo_first_close_frame": demo_first_close,
        "group_a_first_close_frame": a_first_close,
        "group_b_max_grip": b_max_grip,
        "group_a_predictions": pred_a[:50],
        "group_b_predictions": pred_b[:50],
    }
    out_file = out_dir / f"demo_replay_ep{args.episode}.json"
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)
    logger.info("Results saved to %s", out_file)

    print(f"\n=== T9 Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)


if __name__ == "__main__":
    main()
