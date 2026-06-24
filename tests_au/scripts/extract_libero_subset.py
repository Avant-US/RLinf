"""Extract a small, deterministic subset of LIBERO frames for acceptance tests.

Reads the LeRobot LIBERO dataset (parquet episodes + tasks.jsonl), decodes a fixed
set of frames, and dumps a single npz batch + metadata so both openpi-JAX and
RLinf-PyTorch can consume the *same* samples without re-loading the dataset.

Usage:
  python tests_au/scripts/extract_libero_subset.py \
      --data_root  <lerobot libero dir> \
      --num_samples 32 --seed 0 \
      --out_dir tests_au/scripts/_data/libero_subset
"""

from __future__ import annotations

import argparse
import io
import json
import os
from pathlib import Path

import numpy as np

DEFAULT_DATA_ROOT = os.path.expanduser(
    "~/.cache/huggingface/lerobot/physical-intelligence/libero"
)


def _decode_image(cell) -> np.ndarray:
    """Decode a LeRobot image cell (dict with 'bytes') to uint8 HWC array."""
    from PIL import Image

    if isinstance(cell, dict):
        data = cell.get("bytes")
        if data is None and cell.get("path"):
            with open(cell["path"], "rb") as f:
                data = f.read()
        img = Image.open(io.BytesIO(data)).convert("RGB")
        return np.asarray(img, dtype=np.uint8)
    arr = np.asarray(cell)
    if arr.dtype != np.uint8:
        arr = arr.astype(np.uint8)
    return arr


def _load_tasks(data_root: Path) -> dict[int, str]:
    tasks = {}
    tasks_path = data_root / "meta" / "tasks.jsonl"
    if tasks_path.exists():
        with open(tasks_path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                obj = json.loads(line)
                tasks[int(obj["task_index"])] = obj["task"]
    return tasks


def _list_episode_files(data_root: Path) -> list[Path]:
    data_dir = data_root / "data"
    files = sorted(data_dir.rglob("episode_*.parquet"))
    if not files:
        raise FileNotFoundError(f"No episode parquet found under {data_dir}")
    return files


def extract(data_root: str, num_samples: int, seed: int, out_dir: str) -> dict:
    """Extract num_samples frames deterministically; write npz + meta. Returns meta."""
    import pandas as pd

    root = Path(data_root)
    tasks = _load_tasks(root)
    episodes = _list_episode_files(root)

    rng = np.random.default_rng(seed)
    # deterministic: take frames spread across the first few episodes
    images, wrist_images, states, actions, prompts, indices = [], [], [], [], [], []

    ep_idx = 0
    collected = 0
    while collected < num_samples and ep_idx < len(episodes):
        df = pd.read_parquet(episodes[ep_idx])
        n = len(df)
        # Spread frames within the episode; take up to the whole episode if needed
        # so the target count is reached even with short episodes / few episodes.
        per_ep_cap = max(1, n // 8)
        remaining_eps = len(episodes) - ep_idx
        # if remaining episodes can't supply enough at the capped rate, take more
        if per_ep_cap * remaining_eps < (num_samples - collected):
            per_ep_cap = n
        take = min(num_samples - collected, per_ep_cap, n)
        frame_ids = np.unique(np.linspace(0, n - 1, take, dtype=int))
        for fi in frame_ids:
            row = df.iloc[int(fi)]
            images.append(_decode_image(row["image"]))
            wrist_images.append(_decode_image(row["wrist_image"]))
            states.append(np.asarray(row["state"], dtype=np.float32))
            actions.append(np.asarray(row["actions"], dtype=np.float32))
            ti = int(row["task_index"])
            prompts.append(tasks.get(ti, ""))
            indices.append([ep_idx, int(fi)])
            collected += 1
            if collected >= num_samples:
                break
        ep_idx += 1

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    batch = {
        "image": np.stack(images),               # [N,256,256,3] uint8
        "wrist_image": np.stack(wrist_images),   # [N,256,256,3] uint8
        "state": np.stack(states),               # [N,8] float32
        "actions": np.stack(actions),            # [N,7] float32
        "indices": np.asarray(indices),          # [N,2]
    }
    np.savez(out / "batch.npz", **batch)
    with open(out / "prompts.txt", "w") as f:
        f.write("\n".join(prompts))
    meta = {
        "data_root": str(root),
        "num_samples": int(collected),
        "seed": int(seed),
        "shapes": {k: list(v.shape) for k, v in batch.items()},
        "indices": [list(map(int, idx)) for idx in indices],
        "prompts": prompts,
    }
    with open(out / "meta.json", "w") as f:
        json.dump(meta, f, indent=2)
    # silence unused rng lint while keeping seed semantics explicit
    _ = rng
    return meta


def load_subset(subset_dir: str, num_samples: int | None = None) -> dict:
    """Load a previously extracted subset (npz + prompts) as a dict of arrays."""
    sd = Path(subset_dir)
    data = dict(np.load(sd / "batch.npz"))
    prompts_path = sd / "prompts.txt"
    prompts = prompts_path.read_text().split("\n") if prompts_path.exists() else []
    if num_samples is not None:
        for k in list(data.keys()):
            data[k] = data[k][:num_samples]
        prompts = prompts[:num_samples]
    data["prompts"] = prompts
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data_root", default=DEFAULT_DATA_ROOT)
    ap.add_argument("--repo_id", default=None, help="(unused placeholder for HF repo id)")
    ap.add_argument("--num_samples", type=int, default=32)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out_dir", default="tests_au/scripts/_data/libero_subset")
    args = ap.parse_args()

    meta = extract(args.data_root, args.num_samples, args.seed, args.out_dir)
    print(f"[extract] wrote {meta['num_samples']} samples to {args.out_dir}")
    print(f"[extract] shapes: {meta['shapes']}")


if __name__ == "__main__":
    main()
