"""Recompute LIBERO normalization statistics for the RLinf openpi_au pi0.5 example.

This is the RLinf-side, fully self-contained analogue of openpi's
``b/tst/libero/compute_norm_stats_local.py``. It uses the ISOLATED
``rlinf.models.embodiment.openpi_au.dataconfig`` (a decoupled copy of the openpi
pi05_libero data config) and restricts the dataset to locally cached episodes to
avoid HuggingFace rate-limiting. The resulting ``norm_stats.json`` (state/actions
mean/std + q01/q99 quantiles, used by pi0.5 quantile normalization) is written to:

    {output_dir}/{asset_id}/norm_stats.json

where ``asset_id == physical-intelligence/libero``. Point the training config's
``actor.model.model_path`` at ``{output_dir}`` so both the data loader and
``get_model`` find these stats.

Usage:
    /mnt/r/VENV/openpi_venv/bin/python tests_au/example/libero/compute_norm_stats_au.py \
        --repo_id physical-intelligence/libero \
        --output_dir tests_au/example/libero/_ckpt/pi05_base_pt \
        --max_frames 10000
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import tqdm

import openpi.shared.normalize as normalize
import openpi.training.data_loader as _data_loader
import openpi.transforms as transforms

from rlinf.models.embodiment.openpi_au.dataconfig import get_openpi_config


class RemoveStrings(transforms.DataTransformFn):
    """Drop string-typed fields so RunningStats only sees numeric arrays."""

    def __call__(self, x: dict) -> dict:
        return {k: v for k, v in x.items() if not np.issubdtype(np.asarray(v).dtype, np.str_)}


def get_local_episode_count(root: Path) -> int:
    """Count locally cached parquet episode files under {root}/data/chunk-*/."""
    count = 0
    data_dir = root / "data"
    if data_dir.exists():
        for chunk_dir in sorted(data_dir.iterdir()):
            if chunk_dir.is_dir() and chunk_dir.name.startswith("chunk-"):
                count += sum(1 for f in chunk_dir.iterdir() if f.suffix == ".parquet")
    return count


def make_patched_create_torch_dataset(available_episodes: int):
    """Restrict create_torch_dataset to the first ``available_episodes`` episodes."""

    def patched(data_cfg, action_horizon, model_cfg):
        from lerobot.common.datasets import lerobot_dataset

        dataset_meta = lerobot_dataset.LeRobotDatasetMetadata(data_cfg.repo_id)
        dataset = lerobot_dataset.LeRobotDataset(
            data_cfg.repo_id,
            episodes=list(range(available_episodes)),
            delta_timestamps={
                key: [t / dataset_meta.fps for t in range(action_horizon)]
                for key in data_cfg.action_sequence_keys
            },
        )
        if data_cfg.prompt_from_task:
            dataset = _data_loader.TransformedDataset(
                dataset, [transforms.PromptFromLeRobotTask(dataset_meta.tasks)]
            )
        return dataset

    return patched


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config_name", default="pi05_libero")
    ap.add_argument("--repo_id", default="physical-intelligence/libero")
    ap.add_argument("--output_dir", default="tests_au/example/libero/_ckpt/pi05_base_pt")
    ap.add_argument("--batch_size", type=int, default=64)
    ap.add_argument("--max_frames", type=int, default=10000)
    ap.add_argument("--num_workers", type=int, default=2)
    args = ap.parse_args()

    from lerobot.common.constants import HF_LEROBOT_HOME
    from unittest.mock import patch

    # Build the pi05_libero data config (isolated openpi_au copy); repo_id may be a
    # HF id or a local LeRobot root. asset_id stays physical-intelligence/libero.
    config = get_openpi_config(args.config_name, repo_id=args.repo_id)
    data_config = config.data.create(config.assets_dirs, config.model)

    local_root = HF_LEROBOT_HOME / data_config.repo_id
    available = get_local_episode_count(local_root)
    print(f"[norm_stats] {available} locally cached episodes at {local_root}")
    if available == 0:
        print("ERROR: no local LIBERO data found; download the dataset first.")
        sys.exit(1)

    patched = make_patched_create_torch_dataset(available)
    with patch.object(_data_loader, "create_torch_dataset", patched):
        dataset = _data_loader.create_torch_dataset(
            data_config, config.model.action_horizon, config.model
        )
        dataset = _data_loader.TransformedDataset(
            dataset,
            [
                *data_config.repack_transforms.inputs,
                *data_config.data_transforms.inputs,
                RemoveStrings(),
            ],
        )
        if args.max_frames is not None and args.max_frames < len(dataset):
            num_batches = args.max_frames // args.batch_size
            shuffle = True
        else:
            num_batches = len(dataset) // args.batch_size
            shuffle = False
        data_loader = _data_loader.TorchDataLoader(
            dataset,
            local_batch_size=args.batch_size,
            num_workers=args.num_workers,
            shuffle=shuffle,
            num_batches=num_batches,
        )

        keys = ["state", "actions"]
        stats = {key: normalize.RunningStats() for key in keys}
        for batch in tqdm.tqdm(data_loader, total=num_batches, desc="norm stats"):
            for key in keys:
                values = np.asarray(batch[key])
                stats[key].update(values.reshape(-1, values.shape[-1]))

    norm_stats = {key: s.get_statistics() for key, s in stats.items()}

    out_dir = Path(args.output_dir) / data_config.asset_id
    print(f"[norm_stats] writing to {out_dir}")
    normalize.save(out_dir, norm_stats)
    print("[norm_stats] done.")


if __name__ == "__main__":
    main()
