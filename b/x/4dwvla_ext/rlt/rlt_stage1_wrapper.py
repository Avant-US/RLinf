"""RLT Stage 1 training wrapper for InternVLAA15Policy.

Wraps the base 4DWVLA policy to add RLT loss computation
without modifying any 4DWVLA source code.
"""

import logging
import os
from typing import Any

import torch
import torch.nn as nn

try:
    from .rlt_config import RLTStage1Config
    from .rlt_token_transformer import RLTTokenTransformer
except ImportError:
    from rlt_config import RLTStage1Config
    from rlt_token_transformer import RLTTokenTransformer

logger = logging.getLogger(__name__)


class RLTStage1TrainingWrapper(nn.Module):
    """Wraps InternVLAA15Policy to add RLT Stage 1 training."""

    def __init__(self, base_policy: nn.Module, rlt_config: RLTStage1Config):
        super().__init__()
        self.base_policy = base_policy
        self.rlt_config = rlt_config

        self.rlt_module = RLTTokenTransformer(
            input_dim=rlt_config.rlt_input_dim,
            embed_dim=rlt_config.rlt_embed_dim,
            prefix_seq_len=rlt_config.rlt_prefix_seq_len,
            num_layers=rlt_config.rlt_num_layers,
            num_heads=rlt_config.rlt_num_heads,
            mlp_ratio=rlt_config.rlt_mlp_ratio,
            dropout_rate=rlt_config.rlt_dropout,
        )
        self.rlt_alpha = rlt_config.rlt_alpha
        self.rlt_image_only = rlt_config.rlt_image_only

        self._captured_prefix_out = None
        self._install_prefix_capture()

        param_count = sum(p.numel() for p in self.rlt_module.parameters())
        logger.info(
            f"RLT module created: embed_dim={rlt_config.rlt_embed_dim}, "
            f"params={param_count / 1e6:.1f}M, z_dim={self.rlt_module.z_dim}"
        )

    def _install_prefix_capture(self):
        """Wrap qwen3_5_with_expert.forward to capture prefix_out."""
        inner_model = self.base_policy.model
        expert_model = inner_model.qwen3_5_with_expert
        original_forward = expert_model.forward

        wrapper_self = self

        def wrapped_forward(*args, **kwargs):
            result = original_forward(*args, **kwargs)
            if isinstance(result, (list, tuple)) and len(result) >= 1:
                outputs_list = result[0]
                if isinstance(outputs_list, (list, tuple)) and len(outputs_list) >= 1:
                    wrapper_self._captured_prefix_out = outputs_list[0]
            return result

        expert_model.forward = wrapped_forward
        logger.info("Installed prefix_out capture on qwen3_5_with_expert.forward")

    def _compute_deploy_view_mask(self, batch: dict, prefix_out: torch.Tensor) -> torch.Tensor | None:
        """Compute deployment-view mask from labels."""
        labels = batch.get("labels")
        if labels is None:
            return None
        prefix_len = prefix_out.shape[1]
        if labels.shape[1] < prefix_len:
            return None
        mask = (labels[:, :prefix_len] == -100)
        return mask

    def _compute_image_only_mask(self, batch: dict, prefix_out: torch.Tensor) -> torch.Tensor | None:
        """Compute image-only mask from input_ids and image_token_id."""
        input_ids = batch.get("input_ids")
        if input_ids is None:
            return None
        prefix_len = prefix_out.shape[1]
        image_token_id = getattr(self.base_policy.config, "image_token_id", None)
        if image_token_id is None:
            logger.warning("image_token_id not found in config, using all prefix tokens")
            return None
        if input_ids.shape[1] < prefix_len:
            return None
        mask = (input_ids[:, :prefix_len] == image_token_id)
        return mask

    def forward(self, batch: dict) -> tuple[torch.Tensor, dict[str, Any]]:
        """Forward pass with RLT loss.

        When vla_inference_mode is set, VLA forward runs without gradient
        tracking — saves ~15 GB VRAM on single-GPU setups. RLT gradients
        are unaffected since prefix_out is already detached.
        """
        self._captured_prefix_out = None
        if getattr(self, "vla_inference_mode", False):
            infer_batch = {k: v for k, v in batch.items() if k != "labels"}
            with torch.no_grad():
                vla_output = self.base_policy.forward(infer_batch)
        else:
            vla_output = self.base_policy.forward(batch)

        if isinstance(vla_output, tuple) and len(vla_output) == 2:
            vla_loss, output_dict = vla_output
        elif isinstance(vla_output, dict):
            vla_loss = vla_output.get("loss", torch.tensor(0.0))
            output_dict = vla_output
        else:
            vla_loss = vla_output
            output_dict = {}

        prefix_out = self._captured_prefix_out
        self._captured_prefix_out = None

        if prefix_out is None:
            logger.warning("prefix_out not captured, returning VLA loss only")
            return vla_loss, output_dict

        if self.rlt_image_only:
            rlt_mask = self._compute_image_only_mask(batch, prefix_out)
        else:
            rlt_mask = self._compute_deploy_view_mask(batch, prefix_out)

        rlt_loss, rlt_info = self.rlt_module.loss(prefix_out.detach(), mask=rlt_mask)

        total_loss = rlt_loss + self.rlt_alpha * vla_loss

        output_dict["loss_rlt"] = rlt_loss.item()
        output_dict["loss_vla"] = vla_loss.item()
        output_dict["loss_total"] = total_loss.item()
        output_dict["rlt_mse"] = rlt_info["mse"].item()
        output_dict["rlt_z_rl_norm"] = rlt_info["z_rl"].norm(dim=-1).mean().item()
        output_dict["prefix_seq_len"] = prefix_out.shape[1]

        return total_loss, output_dict

    def get_rlt_params(self):
        """Return RLT module parameters (for separate optimizer)."""
        return self.rlt_module.parameters()

    def get_vla_params(self):
        """Return VLA trainable parameters (for separate optimizer)."""
        return [p for p in self.base_policy.parameters() if p.requires_grad]

    def extract_z_rl(self, batch: dict) -> torch.Tensor:
        """Extract z_rl for Stage 2 contract verification."""
        with torch.no_grad():
            self._captured_prefix_out = None
            fwd_batch = {k: v for k, v in batch.items() if k != "labels"} if getattr(self, "vla_inference_mode", False) else batch
            self.base_policy.forward(fwd_batch)
            prefix_out = self._captured_prefix_out
            self._captured_prefix_out = None

            if prefix_out is None:
                raise RuntimeError("prefix_out not captured")

            if self.rlt_image_only:
                rlt_mask = self._compute_image_only_mask(batch, prefix_out)
            else:
                rlt_mask = self._compute_deploy_view_mask(batch, prefix_out)

            rlt_dtype = next(self.rlt_module.parameters()).dtype
            z_rl = self.rlt_module.encode_flat(prefix_out.to(rlt_dtype), mask=rlt_mask)
            return z_rl

    def save_rlt_checkpoint(self, save_dir: str):
        """Save RLT module weights separately."""
        os.makedirs(save_dir, exist_ok=True)
        torch.save(self.rlt_module.state_dict(), os.path.join(save_dir, "rlt_module.pt"))
        logger.info(f"Saved RLT checkpoint to {save_dir}/rlt_module.pt")

    def load_rlt_checkpoint(self, load_dir: str):
        """Load RLT module weights."""
        import os as _os
        path = _os.path.join(load_dir, "rlt_module.pt")
        state_dict = torch.load(path, map_location="cpu", weights_only=True)
        self.rlt_module.load_state_dict(state_dict)
        logger.info(f"Loaded RLT checkpoint from {path}")
