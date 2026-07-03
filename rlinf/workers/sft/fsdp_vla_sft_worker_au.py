# Copyright 2025 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""SFT worker with EMA, openpi_cosine LR, param_norm — zero edits to parent."""

from __future__ import annotations

import logging
import math
import os
from typing import Any

import numpy as np
import torch
from omegaconf import DictConfig
from torch.optim.lr_scheduler import LambdaLR

import rlinf.models.embodiment.openpi_au  # noqa: F401 trigger self-registration

from rlinf.data.lerobot_paths import resolve_lerobot_repo_id
from rlinf.models.embodiment.openpi_au.ema import ModelEMA
from rlinf.workers.sft.fsdp_vla_sft_worker import FSDPVlaSftWorker

logger = logging.getLogger(__name__)


class FSDPVlaSftWorkerAu(FSDPVlaSftWorker):
    """SFT worker with EMA, openpi_cosine LR, and param_norm — zero edits to parent."""

    def build_dataloader(self, data_paths: Any, eval_dataset: bool = False):
        """Route the openpi_au model_type through the openpi data loader.

        The base ``build_dataloader`` only matches ``SupportedModel.OPENPI`` and would
        raise ``KeyError`` for ``openpi_au``. We mirror the openpi path here but use the
        isolated ``openpi_au.dataconfig`` so the copy stays fully decoupled. Any other
        model type defers to the parent implementation.

        To avoid HuggingFace 429 rate-limiting when many FSDP ranks build the dataset
        concurrently, we force HF offline mode and restrict the LeRobot dataset to the
        episodes already present in the local cache (runtime monkey-patch of openpi's
        ``create_torch_dataset`` -- no openpi source is modified). This mirrors the
        openpi JAX LIBERO reference (openpi05/b/tst/libero/train_pi05_libero.py).
        """
        if str(self.cfg.actor.model.model_type) != "openpi_au":
            return super().build_dataloader(data_paths, eval_dataset=eval_dataset)

        repo_id = resolve_lerobot_repo_id(data_paths)
        if repo_id is None:
            raise ValueError(
                "OpenPI(au) SFT requires data.train_data_paths to be set to a local "
                "dataset path or LeRobot repo id."
            )

        # Use only the local LeRobot cache; do not hit the HF API from every rank.
        os.environ.setdefault("HF_HUB_OFFLINE", "1")
        os.environ.setdefault("HF_DATASETS_OFFLINE", "1")

        import openpi.training.data_loader as openpi_data_loader

        from rlinf.models.embodiment.openpi_au.dataconfig import get_openpi_config

        config = get_openpi_config(
            self.cfg.actor.model.openpi.config_name,
            model_path=self.cfg.actor.model.model_path,
            batch_size=self.cfg.actor.micro_batch_size * self._world_size,
            repo_id=repo_id,
            data_kwargs=getattr(self.cfg.actor, "openpi_data", None),
        )

        from contextlib import nullcontext

        patch_ctx = _local_episodes_patch(openpi_data_loader, repo_id)
        with patch_ctx if patch_ctx is not None else nullcontext():
            data_loader = openpi_data_loader.create_data_loader(
                config, framework="pytorch", shuffle=True
            )
        return data_loader, data_loader.data_config()

    def get_max_steps_per_epoch(self):
        """Compute steps/epoch for openpi_au by unwrapping the openpi DataLoader."""
        if str(self.cfg.actor.model.model_type) == "openpi_au":
            if self.data_loader is None:
                return 0
            num_batches = len(self._openpi_pytorch_dataloader(self.data_loader))
            return max(1, num_batches // self.gradient_accumulation)
        return super().get_max_steps_per_epoch()

    def init_worker(self):
        super().init_worker()
        decay = self.cfg.actor.optim.get("ema_decay", None)
        if decay is not None and float(decay) > 0:
            ema_device_str = self.cfg.actor.optim.get("ema_device", None)
            ema_device = torch.device(ema_device_str) if ema_device_str else None
            self.ema = ModelEMA(self.model, decay=float(decay), device=ema_device)
        else:
            self.ema = None

    def run_training(self):
        metrics = super().run_training()
        if self.ema is not None:
            self.ema.update(self.model)
        # param_norm metric
        with torch.no_grad():
            param_norms = [p.detach().float().norm() for p in self.model.parameters() if p.requires_grad]
            if param_norms:
                pnorm = torch.norm(torch.stack(param_norms))
            else:
                pnorm = torch.tensor(0.0)
        if isinstance(metrics, dict):
            metrics["param_norm"] = float(pnorm)
        return metrics

    def save_checkpoint(self, save_path: str, step: int = 0) -> None:
        if self.ema is not None:
            backup = self.ema.swap_in(self.model)
            try:
                super().save_checkpoint(save_path, step)
            finally:
                self.ema.swap_out(self.model, backup)
            if self._rank == 0:
                ema_path = os.path.join(save_path, "ema.pt")
                torch.save(self.ema.state_dict(), ema_path)
                logger.info(f"[EMA] Saved state to {ema_path}")
            torch.distributed.barrier()
        else:
            super().save_checkpoint(save_path, step)

    def load_checkpoint(self, load_path: str) -> None:
        super().load_checkpoint(load_path)
        if self.ema is not None:
            ema_path = os.path.join(load_path, "ema.pt")
            if os.path.exists(ema_path):
                state = torch.load(ema_path, map_location="cpu", weights_only=False)
                self.ema.load_state_dict(state)
                logger.info(f"[EMA] Restored from {ema_path} (num_updates={self.ema.num_updates})")
            else:
                logger.warning(f"[EMA] No ema.pt found at {ema_path}, starting fresh EMA")

    def build_lr_scheduler(self, optimizer, optim_config):
        lr_sched = optim_config.get("lr_scheduler", None)
        if lr_sched == "openpi_cosine":
            return _build_openpi_cosine(optimizer, optim_config)
        return super().build_lr_scheduler(optimizer, optim_config)


def _local_episode_indices(repo_id: str) -> list[int]:
    """Return the largest contiguous run of *officially registered* episodes (per
    ``meta/episodes.jsonl``) whose files are fully present in the local cache.

    ``repo_id`` may be a local dataset root or an HF Hub id; resolved to an on-disk
    root via ``resolve_lerobot_dataset_root`` (checks ``{repo_id}/meta/info.json``,
    then ``HF_LEROBOT_HOME/{repo_id}/meta/info.json``). Returns [] if no local
    metadata can be found.

    Two things this deliberately does NOT assume (unlike a naive
    ``range(total_episodes)``/directory-regex scan):
      * The official episode range starts at 0 and is exactly ``[0, total_episodes)``.
        Some LeRobot v3.0 exports register a non-zero-based range (e.g. episodes
        [60, 90] out of a larger raw collection) while the very same
        ``data/chunk-*/`` directory *also* physically contains unrelated leftover
        parquet/video files for other episode indices (e.g. 0..59) that are not part
        of this dataset at all -- a naive scan would silently pick up the wrong
        episodes. We use ``LeRobotDatasetMetadata.episodes`` (parsed from
        ``meta/episodes.jsonl``) as the authoritative index set instead.
      * Only the parquet file matters. Datasets with ``dtype: video`` camera features
        (unlike LIBERO's inline ``dtype: image``) also need every camera's ``.mp4``
        present; we check ``get_video_file_path`` for every registered video key too
        (not just the ones our own policy/repack happens to read), since that is
        exactly what ``LeRobotDataset.__init__``'s "all files present" assertion
        checks.

    We still return a *contiguous run starting at the dataset's own minimum index*
    (not the raw present-and-complete set) and stop at the first incomplete episode:
    this preserves the original "avoid gaps" caution (some LeRobot versions build
    ``episode_data_index`` by walking the requested episode list in order) while
    generalizing "prefix from 0" to "prefix from wherever this dataset's official
    episodes actually start". Passing the result back as ``episodes=...`` makes
    ``LeRobotDataset``'s file-presence assert succeed, so it loads purely from local
    disk with no HF network call (offline-safe, avoids 429 at many ranks).
    """
    from rlinf.data.lerobot_paths import resolve_lerobot_dataset_root

    root = resolve_lerobot_dataset_root(repo_id)
    if not (root / "meta" / "episodes.jsonl").is_file():
        return []

    try:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDatasetMetadata

        dataset_meta = LeRobotDatasetMetadata(repo_id, root=root)
    except Exception:
        logger.warning(
            "[openpi_au] failed to read local LeRobot metadata for %s at %s",
            repo_id,
            root,
            exc_info=True,
        )
        return []

    official = sorted(dataset_meta.episodes.keys())
    if not official:
        return []

    def _is_complete(ep_idx: int) -> bool:
        if not (root / dataset_meta.get_data_file_path(ep_idx)).is_file():
            return False
        return all(
            (root / dataset_meta.get_video_file_path(ep_idx, vid_key)).is_file()
            for vid_key in dataset_meta.video_keys
        )

    result = []
    prev = None
    for ep_idx in official:
        if prev is not None and ep_idx != prev + 1:
            break  # gap in the official registration itself
        if not _is_complete(ep_idx):
            break  # first officially-registered episode missing locally
        result.append(ep_idx)
        prev = ep_idx
    return result


def _positional_episode_data_index(dataset_meta, episodes: list[int]) -> dict[str, torch.Tensor]:
    """Build an ``episode_data_index`` that ``LeRobotDataset.__getitem__`` can index
    directly by *raw* episode index (see the call site for why this is needed).

    Same cumulative from/to row offsets lerobot's own ``get_episode_data_index`` would
    compute (assuming ``self.hf_dataset`` rows are laid out in ``episodes`` order, which
    they are: ``load_hf_dataset`` builds its file list by iterating ``self.episodes``),
    just placed at position ``ep_idx`` instead of at ``episodes.index(ep_idx)``.
    """
    episode_lengths = {ep_idx: dataset_meta.episodes[ep_idx]["length"] for ep_idx in episodes}
    size = max(episodes) + 1
    from_arr = torch.zeros(size, dtype=torch.long)
    to_arr = torch.zeros(size, dtype=torch.long)
    cursor = 0
    for ep_idx in episodes:
        length = episode_lengths[ep_idx]
        from_arr[ep_idx] = cursor
        to_arr[ep_idx] = cursor + length
        cursor += length
    return {"from": from_arr, "to": to_arr}


def _local_episodes_patch(openpi_data_loader, repo_id: str):
    """Return a context manager patching openpi ``create_torch_dataset`` to use only
    locally cached episodes; ``None`` if no local episodes can be determined.
    """
    local_indices = _local_episode_indices(repo_id)
    if not local_indices:
        logger.warning(
            "[openpi_au] no local episodes found for %s; using openpi default "
            "(may hit HF / rate-limit).",
            repo_id,
        )
        return None

    from unittest.mock import patch

    import openpi.transforms as transforms

    def patched_create_torch_dataset(data_config, action_horizon, model_config):
        from lerobot.common.datasets import lerobot_dataset

        rid = data_config.repo_id
        dataset_meta = lerobot_dataset.LeRobotDatasetMetadata(rid)
        # Defensive re-check against this fresh metadata load; `local_indices` is
        # already a subset of the *official* episode set (see
        # `_local_episode_indices`), NOT a `< total_episodes` bound, since the
        # official range need not start at 0 (e.g. pushdoor registers 60..90).
        episodes = [e for e in local_indices if e in dataset_meta.episodes]
        dataset = lerobot_dataset.LeRobotDataset(
            rid,
            episodes=episodes,
            delta_timestamps={
                key: [t / dataset_meta.fps for t in range(action_horizon)]
                for key in data_config.action_sequence_keys
            },
        )
        # Work around a lerobot (0.1.0-pinned) bug: LeRobotDataset.__getitem__ looks up
        # `self.episode_data_index["from"/"to"][ep_idx]` using the *raw* episode_index
        # value from each row, but `get_episode_data_index(..., episodes)` returns a
        # dense tensor indexed by each episode's *position* in `episodes` (correct only
        # when `episodes == range(len(episodes))`). For a non-zero-based/non-contiguous
        # `episodes` list (e.g. pushdoor's [60..90]) this is an out-of-range/incorrect
        # lookup at the very first training step. Rebuild it positioned at the raw
        # index instead -- purely a runtime patch of a third-party dependency's data
        # structure, no lerobot/openpi source is modified.
        if episodes != list(range(len(episodes))):
            dataset.episode_data_index = _positional_episode_data_index(dataset_meta, episodes)
        if data_config.prompt_from_task:
            dataset = openpi_data_loader.TransformedDataset(
                dataset, [transforms.PromptFromLeRobotTask(dataset_meta.tasks)]
            )
        return dataset

    logger.info(
        "[openpi_au] restricting dataset '%s' to %d locally cached episodes (indices %d..%d)",
        repo_id,
        len(local_indices),
        local_indices[0],
        local_indices[-1],
    )
    return patch.object(openpi_data_loader, "create_torch_dataset", patched_create_torch_dataset)


def _build_openpi_cosine(optimizer, optim_config):
    """LR schedule numerically equivalent to optax.warmup_cosine_decay_schedule.

    Mirrors openpi CosineDecaySchedule exactly:
        init_value = peak_lr / (warmup_steps + 1)
        peak_value = peak_lr
        warmup_steps, decay_steps, end_value = decay_lr

    - Warmup [0, warmup): linear init -> peak.
    - Cosine [warmup, decay_steps]: peak -> end_value.
    - step >= decay_steps: constant end_value.
    When decay_lr == peak, the schedule is warmup then constant (pi05_libero shape).
    """
    peak = float(optim_config.lr)
    end_lr = float(optim_config.get("decay_lr", peak))
    warmup = int(optim_config.get("lr_warmup_steps", 0))
    decay_steps = int(optim_config.get("decay_steps", optim_config.get("total_training_steps", 30000)))
    init = peak / (warmup + 1) if warmup > 0 else peak

    def lr_lambda(step):
        # LambdaLR multiplies base_lr (== peak) by the returned factor.
        if warmup > 0 and step < warmup:
            lr = init + (peak - init) * step / warmup
            return lr / peak
        if step >= decay_steps:
            return end_lr / peak
        prog = (step - warmup) / max(1, decay_steps - warmup)
        cos_val = end_lr + 0.5 * (peak - end_lr) * (1.0 + math.cos(math.pi * prog))
        return cos_val / peak

    return LambdaLR(optimizer, lr_lambda)
