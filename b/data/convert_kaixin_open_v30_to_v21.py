#!/usr/bin/env python3
"""Convert kaixin LeRobot v3.0 open dataset to FastWAM-compatible LeRobot v2.1 layout.

Maps decomposed v3 action/state fields back to 23-dim ``actions`` / ``state`` using
``meta/modality.json`` from r1_pro_data_convert_chassis, renames camera videos, and
writes v2.1 path templates (``episode_{index:06d}`` parquet + ``videos_backup/`` mp4).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import shutil
from pathlib import Path

import av
import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

V3_CAMERAS = {
    "head_rgb": "observation.images.head_rgb",
    "left_wrist_rgb": "observation.images.left_wrist_rgb",
    "right_wrist_rgb": "observation.images.right_wrist_rgb",
}

ACTION_PARTS = [
    "action.left_arm",
    "action.right_arm",
    "action.left_gripper",
    "action.right_gripper",
    "action.torso",
    "action.chassis.velocities",
]

STATE_PARTS = [
    "observation.state.left_arm",
    "observation.state.right_arm",
    "observation.state.left_gripper",
    "observation.state.right_gripper",
    "observation.state.torso",
]


def _extract_video_timestamps(video_path: Path) -> np.ndarray | None:
    """Extract PTS-based frame timestamps (seconds) from a video file."""
    try:
        container = av.open(str(video_path))
        stream = container.streams.video[0]
        tb = float(stream.time_base)
        pts_list = []
        for packet in container.demux(stream):
            for frame in packet.decode():
                pts_list.append(frame.pts * tb)
        container.close()
        if pts_list:
            return np.array(pts_list, dtype=np.float64)
    except Exception as e:
        logger.warning("Failed to extract timestamps from %s: %s", video_path, e)
    return None


def _stack_col(table: pa.Table, name: str) -> np.ndarray:
    col = table[name].to_numpy()
    if col.dtype == object:
        return np.stack(col).astype(np.float32)
    return np.asarray(col, dtype=np.float32)


def build_actions(table: pa.Table) -> np.ndarray:
    return np.concatenate([_stack_col(table, k) for k in ACTION_PARTS], axis=1)


def build_state(table: pa.Table) -> np.ndarray:
    chassis = _stack_col(table, "observation.state.chassis")
    return np.concatenate(
        [_stack_col(table, k) for k in STATE_PARTS] + [chassis[:, -3:]],
        axis=1,
    )


def episode_stats(name: str, arr: np.ndarray) -> dict:
    return {
        "min": arr.min(axis=0).tolist(),
        "max": arr.max(axis=0).tolist(),
        "mean": arr.mean(axis=0).tolist(),
        "std": arr.std(axis=0).tolist(),
        "count": [int(arr.shape[0])],
    }


def scalar_stats(values: np.ndarray) -> dict:
    values = np.asarray(values)
    return {
        "min": [float(values.min())],
        "max": [float(values.max())],
        "mean": [float(values.mean())],
        "std": [float(values.std())],
        "count": [int(values.shape[0])],
    }


def convert_episode(
    src_root: Path,
    dst_root: Path,
    episode_index: int,
    global_offset: int,
    fps: float,
) -> tuple[int, dict]:
    src_parquet = src_root / "data" / "chunk-000" / f"file-{episode_index:03d}.parquet"
    if not src_parquet.is_file():
        raise FileNotFoundError(src_parquet)

    table = pq.read_table(src_parquet)
    actions = build_actions(table)
    state = build_state(table)
    n = actions.shape[0]
    assert actions.shape == (n, 23), actions.shape
    assert state.shape == (n, 23), state.shape

    frame_index = _stack_col(table, "frame_index").reshape(-1).astype(np.int64)

    # Use actual video timestamps so parquet aligns with real frame PTS.
    ref_video = (
        src_root / "videos" / "observation.images.head_rgb"
        / "chunk-000" / f"file-{episode_index:03d}.mp4"
    )
    video_ts = _extract_video_timestamps(ref_video)
    if video_ts is not None and len(video_ts) >= n:
        timestamp = video_ts[frame_index]
    else:
        logger.warning(
            "Episode %d: could not extract video timestamps (got %s frames, need %d); "
            "falling back to frame_index / fps",
            episode_index,
            len(video_ts) if video_ts is not None else "None",
            n,
        )
        timestamp = frame_index.astype(np.float64) / float(fps)

    df = pd.DataFrame(
        {
            "state": [row for row in state],
            "actions": [row for row in actions],
            "timestamp": timestamp,
            "frame_index": frame_index,
            "episode_index": np.full(n, episode_index, dtype=np.int64),
            "index": np.arange(global_offset, global_offset + n, dtype=np.int64),
            "task_index": _stack_col(table, "task_index").reshape(-1).astype(np.int64),
        }
    )

    dst_parquet = dst_root / "data" / "chunk-000" / f"episode_{episode_index:06d}.parquet"
    dst_parquet.parent.mkdir(parents=True, exist_ok=True)
    df.to_parquet(dst_parquet, index=False)

    dst_video_dir = dst_root / "videos_backup" / "chunk-000"
    dst_video_dir.mkdir(parents=True, exist_ok=True)
    for short_key, v3_key in V3_CAMERAS.items():
        src_video = (
            src_root
            / "videos"
            / v3_key
            / "chunk-000"
            / f"file-{episode_index:03d}.mp4"
        )
        dst_video = dst_video_dir / f"episode_{episode_index:06d}_{short_key}.mp4"
        if dst_video.exists() or dst_video.is_symlink():
            dst_video.unlink()
        if not src_video.is_file():
            raise FileNotFoundError(src_video)
        os.symlink(src_video.resolve(), dst_video)

    stats = {
        "state": episode_stats("state", state),
        "actions": episode_stats("actions", actions),
        "timestamp": scalar_stats(df["timestamp"].to_numpy()),
        "frame_index": scalar_stats(df["frame_index"].to_numpy()),
        "episode_index": {
            "min": [episode_index],
            "max": [episode_index],
            "mean": [float(episode_index)],
            "std": [0.0],
            "count": [n],
        },
        "index": {
            "min": [global_offset],
            "max": [global_offset + n - 1],
            "mean": [float(global_offset + (n - 1) / 2.0)],
            "std": [float(np.std(df["index"].to_numpy()))],
            "count": [n],
        },
        "task_index": scalar_stats(df["task_index"].to_numpy()),
    }
    return n, stats


def write_meta(
    src_root: Path,
    dst_root: Path,
    src_info: dict,
    episodes_meta: list[dict],
    episodes_stats: list[dict],
    total_frames: int,
) -> None:
    meta_dir = dst_root / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    ref_modality = Path(
        "/mnt/r/share/zwy/datasets/r1_pro_data_v2/r1_pro_data_convert_chassis/meta/modality.json"
    )
    if ref_modality.is_file():
        shutil.copy2(ref_modality, meta_dir / "modality.json")

    features = {
        "head_rgb": {
            "dtype": "video",
            "shape": [1080, 1920, 3],
            "names": ["height", "width", "channels"],
            "info": src_info["features"]["observation.images.head_rgb"].get("info", {}),
        },
        "left_wrist_rgb": {
            "dtype": "video",
            "shape": [480, 640, 3],
            "names": ["height", "width", "channels"],
            "info": src_info["features"]["observation.images.left_wrist_rgb"].get("info", {}),
        },
        "right_wrist_rgb": {
            "dtype": "video",
            "shape": [480, 640, 3],
            "names": ["height", "width", "channels"],
            "info": src_info["features"]["observation.images.right_wrist_rgb"].get("info", {}),
        },
        "state": {"dtype": "float32", "shape": [23], "names": ["state"]},
        "actions": {"dtype": "float32", "shape": [23], "names": ["actions"]},
        "timestamp": {"dtype": "float32", "shape": [1], "names": None},
        "frame_index": {"dtype": "int64", "shape": [1], "names": None},
        "episode_index": {"dtype": "int64", "shape": [1], "names": None},
        "index": {"dtype": "int64", "shape": [1], "names": None},
        "task_index": {"dtype": "int64", "shape": [1], "names": None},
    }

    info = {
        "codebase_version": "v2.1",
        "robot_type": src_info.get("robot_type", "r1_pro"),
        "total_episodes": len(episodes_meta),
        "total_frames": total_frames,
        "total_tasks": src_info.get("total_tasks", 1),
        "total_videos": len(episodes_meta) * len(V3_CAMERAS),
        "total_chunks": 1,
        "chunks_size": src_info.get("chunks_size", 1000),
        "fps": src_info.get("fps", 12),
        "splits": {"train": f"0:{len(episodes_meta)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": (
            "videos_backup/chunk-{episode_chunk:03d}/"
            "episode_{episode_index:06d}_{video_key}.mp4"
        ),
        "features": features,
    }
    with open(meta_dir / "info.json", "w", encoding="utf-8") as f:
        json.dump(info, f, indent=4)

    shutil.copy2(src_root / "meta" / "tasks.jsonl", meta_dir / "tasks.jsonl")

    with open(meta_dir / "episodes.jsonl", "w", encoding="utf-8") as f:
        for ep in episodes_meta:
            f.write(json.dumps(ep, ensure_ascii=False) + "\n")

    with open(meta_dir / "episodes_stats.jsonl", "w", encoding="utf-8") as f:
        for item in episodes_stats:
            f.write(json.dumps(item) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert kaixin LeRobot v3.0 to v2.1")
    parser.add_argument(
        "--src",
        type=Path,
        default=Path("/mnt/r/share/kaixin/data_0530/lerobot_open_merged"),
    )
    parser.add_argument(
        "--dst",
        type=Path,
        default=Path("/mnt/r/share/kaixin/data_0530/lerobot_open_merged_v21"),
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    if args.dst.exists():
        if not args.overwrite:
            raise SystemExit(f"Destination exists: {args.dst}. Pass --overwrite to replace.")
        shutil.rmtree(args.dst)
    args.dst.mkdir(parents=True)

    with open(args.src / "meta" / "info.json", encoding="utf-8") as f:
        src_info = json.load(f)

    episodes_meta = []
    with open(args.src / "meta" / "episodes.jsonl", encoding="utf-8") as f:
        for line in f:
            ep = json.loads(line)
            episodes_meta.append(
                {
                    "episode_index": ep["episode_index"],
                    "tasks": ep["tasks"],
                    "length": ep["length"],
                }
            )
    episodes_meta.sort(key=lambda x: x["episode_index"])
    fps = float(src_info.get("fps", 12))

    total_frames = 0
    episodes_stats = []
    for ep in episodes_meta:
        ep_idx = ep["episode_index"]
        logger.info("Converting episode %d ...", ep_idx)
        n, stats = convert_episode(args.src, args.dst, ep_idx, total_frames, fps)
        if n != ep["length"]:
            logger.warning(
                "Episode %d length mismatch: meta=%d parquet=%d", ep_idx, ep["length"], n
            )
            ep["length"] = n
        episodes_stats.append({"episode_index": ep_idx, "stats": stats})
        total_frames += n

    write_meta(args.src, args.dst, src_info, episodes_meta, episodes_stats, total_frames)
    logger.info(
        "Done: %d episodes, %d frames -> %s",
        len(episodes_meta),
        total_frames,
        args.dst,
    )


if __name__ == "__main__":
    main()
