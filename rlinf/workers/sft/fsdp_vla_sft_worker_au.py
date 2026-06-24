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

import numpy as np
import torch
from omegaconf import DictConfig
from torch.optim.lr_scheduler import LambdaLR

import rlinf.models.embodiment.openpi_au  # noqa: F401 trigger self-registration

from rlinf.models.embodiment.openpi_au.ema import ModelEMA
from rlinf.workers.sft.fsdp_vla_sft_worker import FSDPVlaSftWorker

logger = logging.getLogger(__name__)


class FSDPVlaSftWorkerAu(FSDPVlaSftWorker):
    """SFT worker with EMA, openpi_cosine LR, and param_norm — zero edits to parent."""

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
