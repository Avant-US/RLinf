"""Compute pushdoor normalization statistics for the RLinf openpi_au pi0.5 example.

Self-contained copy of ``tests_au/example/pushdoor/compute_norm_stats_au.py`` with
defaults pointed at the ``0622_lerobot_data_tst1`` dataset. It uses the ISOLATED
``rlinf.models.embodiment.openpi_au.dataconfig`` (``pi05_pushdoor``) and restricts the
dataset to locally cached episodes to avoid HuggingFace rate-limiting / network access,
exactly like the FSDP SFT worker does at train time.

It reuses the ``_local_episode_indices`` / ``_positional_episode_data_index`` helpers
from ``rlinf.workers.sft.fsdp_vla_sft_worker_au``. Those resolve the *officially
registered* episode set from ``meta/episodes.jsonl`` intersected with what's actually
complete on local disk, and patch around a lerobot indexing bug that only manifests for
non-zero-based episode ranges. (The ``tst1`` dataset happens to be 12 contiguous
episodes 0..11, so the patch is a no-op here, but the helpers are dataset-agnostic.)

The resulting ``norm_stats.json`` (state/actions mean/std + q01/q99 quantiles, used by
pi0.5 quantile normalization) is written to:

    {output_dir}/{asset_id}/norm_stats.json

where ``asset_id == rlinf/pushdoor_open0622`` (stable regardless of the on-disk dataset
path). Point the training config's ``actor.model.model_path`` at ``{output_dir}`` so
both the data loader and ``get_model`` find these stats.

Usage (see run_norm_stats.sh for the wrapper that sets PYTHONPATH / single-GPU / etc.):
    /mnt/r/VENV/openpi_venv/bin/python examples/au/pi/compute_norm_stats_au.py \
        --repo_id /mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data_tst1 \
        --output_dir examples/au/pi/_ckpt/pi05_pushdoor_tst1 \
        --max_frames 1024
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
from rlinf.workers.sft.fsdp_vla_sft_worker_au import (
    _local_episode_indices,
    _positional_episode_data_index,
)


class RemoveStrings(transforms.DataTransformFn):
    """Drop string-typed fields so RunningStats only sees numeric arrays."""

    def __call__(self, x: dict) -> dict:
        return {k: v for k, v in x.items() if not np.issubdtype(np.asarray(v).dtype, np.str_)}


def make_patched_create_torch_dataset(episodes: list[int]):
    """Restrict create_torch_dataset to exactly the given (already-validated) episodes."""

    def patched(data_cfg, action_horizon, model_cfg):
        from lerobot.common.datasets import lerobot_dataset

        dataset_meta = lerobot_dataset.LeRobotDatasetMetadata(data_cfg.repo_id)
        dataset = lerobot_dataset.LeRobotDataset(
            data_cfg.repo_id,
            episodes=episodes,
            delta_timestamps={
                key: [t / dataset_meta.fps for t in range(action_horizon)]
                for key in data_cfg.action_sequence_keys
            },
        )
        if episodes != list(range(len(episodes))):
            dataset.episode_data_index = _positional_episode_data_index(dataset_meta, episodes)
        if data_cfg.prompt_from_task:
            dataset = _data_loader.TransformedDataset(
                dataset, [transforms.PromptFromLeRobotTask(dataset_meta.tasks)]
            )
        return dataset

    return patched


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config_name", default="pi05_pushdoor")
    ap.add_argument("--repo_id", default="/mnt/r/DATA/SKILL/pushdoor/0622_lerobot_data_tst1")
    ap.add_argument("--output_dir", default="examples/au/pi/_ckpt/pi05_pushdoor_tst1")
    ap.add_argument("--batch_size", type=int, default=64)
    ap.add_argument("--max_frames", type=int, default=1024)
    ap.add_argument("--num_workers", type=int, default=16)
    args = ap.parse_args()

    from unittest.mock import patch

    # Build the pi05_pushdoor data config (isolated openpi_au copy); repo_id may be a
    # local LeRobot root or an HF id. asset_id stays rlinf/pushdoor_open0622 so
    # norm_stats.json lands under a stable, dataset-identifying subdirectory.
    config = get_openpi_config(args.config_name, repo_id=args.repo_id)
    data_config = config.data.create(config.assets_dirs, config.model)

    episodes = _local_episode_indices(args.repo_id)
    print(
        f"[norm_stats] {len(episodes)} locally cached+complete official episodes "
        f"for {args.repo_id}"
        + (f" (indices {episodes[0]}..{episodes[-1]})" if episodes else "")
    )
    if not episodes:
        print("ERROR: no local pushdoor data found; check the dataset path.")
        sys.exit(1)

    patched = make_patched_create_torch_dataset(episodes)
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
