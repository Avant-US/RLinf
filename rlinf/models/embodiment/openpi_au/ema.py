"""EMA (Exponential Moving Average) for FSDP-sharded models.

Usage:
    ema = ModelEMA(model, decay=0.999)
    # after each optimizer step:
    ema.update(model)
    # before save:
    backup = ema.swap_in(model)
    save(model)
    ema.swap_out(model, backup)
"""

from __future__ import annotations

import logging
from typing import Optional

import torch
import torch.nn as nn

logger = logging.getLogger(__name__)


class ModelEMA:
    """FSDP-friendly EMA that tracks local shards only."""

    def __init__(self, model: nn.Module, decay: float = 0.999, device: Optional[torch.device] = None):
        """
        Args:
            model: The FSDP-wrapped model (post-wrap). Only requires_grad params are tracked.
            decay: EMA decay rate alpha.
            device: If set, shadow params are stored on this device (e.g. CPU for memory saving).
                    Default None = same device as the param.
        """
        self.decay = decay
        self.device = device
        self.shadow: dict[str, torch.Tensor] = {}
        self.num_updates: int = 0

        for name, param in model.named_parameters():
            if param.requires_grad:
                shadow = param.detach().clone()
                if device is not None:
                    shadow = shadow.to(device)
                self.shadow[name] = shadow

        logger.info(
            f"[ModelEMA] Initialized with decay={decay}, tracking {len(self.shadow)} params"
            f"{f' on {device}' if device else ''}"
        )

    @torch.no_grad()
    def update(self, model: nn.Module) -> None:
        """Update shadow params: shadow = decay * shadow + (1 - decay) * param."""
        d = self.decay
        for name, param in model.named_parameters():
            if not param.requires_grad:
                continue
            if name not in self.shadow:
                continue
            shadow = self.shadow[name]
            if self.device is not None:
                shadow.mul_(d).add_(param.detach().to(self.device), alpha=1.0 - d)
            else:
                shadow.mul_(d).add_(param.detach(), alpha=1.0 - d)
        self.num_updates += 1

    @torch.no_grad()
    def swap_in(self, model: nn.Module) -> dict[str, torch.Tensor]:
        """Replace model params with EMA shadow; return backup of original params."""
        backup = {}
        for name, param in model.named_parameters():
            if name in self.shadow:
                backup[name] = param.detach().clone()
                src = self.shadow[name]
                if src.device != param.device:
                    src = src.to(param.device)
                param.data.copy_(src)
        return backup

    @torch.no_grad()
    def swap_out(self, model: nn.Module, backup: dict[str, torch.Tensor]) -> None:
        """Restore original params from backup after swap_in."""
        for name, param in model.named_parameters():
            if name in backup:
                param.data.copy_(backup[name])

    def state_dict(self) -> dict:
        return {
            "decay": self.decay,
            "num_updates": self.num_updates,
            "shadow": {k: v.cpu() for k, v in self.shadow.items()},
        }

    def load_state_dict(self, state: dict) -> None:
        self.decay = state["decay"]
        self.num_updates = state.get("num_updates", 0)
        for k, v in state["shadow"].items():
            if k in self.shadow:
                target_device = self.shadow[k].device
                self.shadow[k] = v.to(target_device)
            else:
                logger.warning(f"[ModelEMA] Key {k} in saved state but not in current model, skipping.")
