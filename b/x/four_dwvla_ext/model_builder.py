"""4DWVLA model builder for RLinf -- called by get_model() registry dispatch."""

from __future__ import annotations

import logging
from typing import Optional

import torch
from omegaconf import DictConfig

logger = logging.getLogger(__name__)


def build_four_dwvla_model(
    cfg: DictConfig, torch_dtype: Optional[torch.dtype] = None
):
    """Build a 4DWVLA policy wrapped for RLinf.

    This function is registered as the model builder for model_type="4dwvla"
    via register_model() in runtime_bootstrap.py. It is called by
    rlinf.models.get_model() (models/__init__.py line 296).

    Args:
        cfg: Model config subtree (actor.model in YAML).
        torch_dtype: Target dtype (bf16/fp32), derived from cfg.precision.

    Returns:
        FourDWVLAPolicy instance (nn.Module + BasePolicy).
    """
    from four_dwvla_ext.policy_adapter import FourDWVLAPolicy

    model = FourDWVLAPolicy(cfg, torch_dtype=torch_dtype)
    logger.info(
        "4DWVLA model built from %s (action_loss_only=%s, keypoint=%s)",
        cfg.model_path,
        cfg.get("action_loss_only", True),
        cfg.get("enable_keypoint", False),
    )
    return model
