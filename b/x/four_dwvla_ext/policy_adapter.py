"""RLinf BasePolicy adapter wrapping the original 4DWVLA policy.

This is the plugin-version of doc 1's rlinf/models/embodiment/four_dwvla/policy_adapter.py.
Identical functionality, different import location.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Optional

import torch
import torch.nn as nn
from omegaconf import DictConfig, OmegaConf
from safetensors.torch import load_model

from rlinf.models.embodiment.base_policy import BasePolicy, ForwardType

logger = logging.getLogger(__name__)


class FourDWVLAPolicy(BasePolicy, nn.Module):
    """Adapter that wraps ``InternVLAA15Policy`` for RLinf's ``BasePolicy`` API.

    Responsibilities:
      1. Load 4WVLA checkpoint via ``from_pretrained`` or manual ``safetensors``
      2. Translate ``forward(forward_type=SFT, data=...)`` -> ``_inner.forward(batch)``
      3. Translate ``predict_action_batch(env_obs)`` -> ``_inner.select_action(batch)``
      4. Provide per-parameter lr_scale groups for FSDP optimizer
      5. Override stale absolute paths from config.json (pretrained_path,
         wan_checkpoint_path etc.)
    """

    def __init__(self, cfg: DictConfig, torch_dtype: Optional[torch.dtype] = None):
        nn.Module.__init__(self)

        from lerobot.policies.internvla_a1_5.configuration_internvla_a1_5 import (
            InternVLAA15Config,
        )
        from lerobot.policies.internvla_a1_5.modeling_internvla_a1_5 import (
            InternVLAA15Policy,
        )

        model_path = str(cfg.model_path)
        overrides = OmegaConf.to_container(
            cfg.get("four_dwvla", OmegaConf.create({})), resolve=True
        )

        # ── Build config from checkpoint or defaults ──────────────────────
        inner_config = self._load_inner_config(model_path)

        # ── Apply nested overrides from YAML (four_dwvla.* keys) ──────────
        for key, val in overrides.items():
            if hasattr(inner_config, key):
                setattr(inner_config, key, val)
                logger.debug("Config override: %s = %s", key, val)

        # ── Apply top-level cfg shortcuts ─────────────────────────────────
        _top_level_mappings = {
            "action_loss_only": "action_loss_only",
            "enable_keypoint": "enable_keypoint_predictor",
            "kpt_4d_mode": "kpt_4d_mode",
            "vlm_model_name": "vlm_model_name_or_path",
            "train_expert_only": "train_expert_only",
        }
        for cfg_key, config_attr in _top_level_mappings.items():
            val = cfg.get(cfg_key)
            if val is not None:
                setattr(inner_config, config_attr, val)

        # ── Override stale absolute paths from config.json ────────────────
        if getattr(inner_config, "action_loss_only", False):
            for path_attr in ("wan_checkpoint_path", "wan_config_path", "vae_path"):
                if hasattr(inner_config, path_attr):
                    current = getattr(inner_config, path_attr, "")
                    if current and not Path(current).exists():
                        setattr(inner_config, path_attr, "")
                        logger.debug(
                            "Neutralized stale path %s=%s (action_loss_only=True)",
                            path_attr, current,
                        )

        # Neutralize pretrained_path -- we load weights separately
        if hasattr(inner_config, "pretrained_path"):
            inner_config.pretrained_path = ""

        # ── Construct the inner policy (builds full model graph) ──────────
        self._inner = InternVLAA15Policy(inner_config)

        # ── Load checkpoint weights ───────────────────────────────────────
        self._load_checkpoint(model_path)

        # ── Cast to desired dtype ─────────────────────────────────────────
        if torch_dtype is not None and torch_dtype != torch.float32:
            self._inner.to(torch_dtype)

        self._cfg = cfg
        self._action_dim = cfg.get("action_dim", 8)
        self._state_dim = cfg.get("state_dim", 8)

        # ── Apply freeze strategy ─────────────────────────────────────────
        if cfg.get("train_expert_only", False):
            self.freeze_vlm()

    def _load_checkpoint(self, model_path: str):
        """Load 4WVLA checkpoint from safetensors.

        Supports both single-file and multi-shard formats.
        strict=False because WAN keys are missing from checkpoint.
        """
        ckpt_dir = Path(model_path)
        safetensors_files = sorted(ckpt_dir.glob("model*.safetensors"))
        if safetensors_files:
            for sf in safetensors_files:
                load_model(self._inner, str(sf), strict=False)
            logger.info(
                "Loaded 4WVLA checkpoint from %s (%d shard(s)).",
                model_path,
                len(safetensors_files),
            )
        else:
            logger.warning(
                "No safetensors found in %s; model uses random init.", model_path
            )

    # ── BasePolicy interface ──────────────────────────────────────────────

    def forward(self, forward_type=ForwardType.DEFAULT, **kwargs):
        """Dispatch by ForwardType. SFT path calls sft_forward()."""
        if forward_type == ForwardType.SFT:
            return self.sft_forward(**kwargs)
        elif forward_type == ForwardType.DEFAULT:
            return self.default_forward(**kwargs)
        else:
            raise NotImplementedError(f"Forward type {forward_type} not supported for 4DWVLA.")

    def default_forward(self, **kwargs):
        return self.sft_forward(**kwargs)

    def sft_forward(self, data: dict[str, Any] = None, **kwargs) -> dict:
        """Run 4WVLA SFT forward: compute multi-component loss.

        Returns dict with "loss" (scalar Tensor for backprop) and detached
        per-component loss floats for logging.
        """
        if data is None:
            data = kwargs.get("batch", kwargs)

        device = next(self._inner.parameters()).device
        dtype = next(self._inner.parameters()).dtype
        batch = {}
        for k, v in data.items():
            if isinstance(v, torch.Tensor):
                v = v.to(device)
                if v.is_floating_point():
                    v = v.to(dtype)
                batch[k] = v
            else:
                batch[k] = v

        with torch.amp.autocast(device_type="cuda", dtype=dtype):
            total_loss, loss_dict = self._inner.forward(batch)

        result = {"loss": total_loss}
        for k, v in loss_dict.items():
            if k == "loss":
                continue
            if isinstance(v, torch.Tensor):
                result[k] = v.detach()
            elif isinstance(v, (float, int)):
                result[k] = v
        return result

    def predict_action_batch(
        self,
        env_obs: dict[str, Any] = None,
        mode: str = "eval",
        **kwargs,
    ) -> tuple[torch.Tensor, dict]:
        """Predict actions for real-robot rollout evaluation."""
        self._inner.eval()
        with torch.no_grad():
            action = self._inner.select_action(env_obs)
        return action, {}

    # ── Helpers ────────────────────────────────────────────────────────────

    def freeze_vlm(self):
        """Freeze the VLM backbone for expert-only training (Phase 1 Warmup)."""
        self._inner.model.qwen3_5_with_expert.qwen3_5.requires_grad_(False)
        frozen_count = sum(
            1 for p in self._inner.model.qwen3_5_with_expert.qwen3_5.parameters()
            if not p.requires_grad
        )
        logger.info("VLM backbone frozen (%d parameters).", frozen_count)

    def get_param_groups(self, base_lr: float) -> list[dict]:
        """Return per-component parameter groups with lr_scale."""
        return self._inner.get_optim_params()

    @staticmethod
    def _load_inner_config(model_path: str):
        """Load InternVLAA15Config from checkpoint dir, handling draccus quirks.

        The checkpoint's config.json contains a ``type`` field used for
        draccus ChoiceRegistry dispatch, which causes DecodingError if
        passed to from_pretrained. We load the JSON manually and strip it.
        """
        import json
        from lerobot.policies.internvla_a1_5.configuration_internvla_a1_5 import (
            InternVLAA15Config,
        )

        config_path = Path(model_path) / "config.json"
        if not config_path.exists():
            logger.warning("No config.json in %s; using defaults.", model_path)
            return InternVLAA15Config()

        try:
            with open(config_path) as f:
                raw = json.load(f)
            # Strip registry dispatch field that causes draccus error
            raw.pop("type", None)
            config = InternVLAA15Config(**{
                k: v for k, v in raw.items()
                if hasattr(InternVLAA15Config, k) or k in InternVLAA15Config.__dataclass_fields__
            })
            logger.info("Loaded config.json from %s (%d fields)", model_path, len(raw))
            return config
        except Exception as e:
            logger.warning("Failed to parse %s: %s; using defaults.", config_path, e)
            return InternVLAA15Config()
