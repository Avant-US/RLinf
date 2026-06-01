from pathlib import Path

import torch
from omegaconf import DictConfig, OmegaConf

from rlinf.models.embodiment.fastwam.fastwam_config import FastWAMConfig
from rlinf.models.embodiment.fastwam.fastwam_policy import FastWAMPolicy


def _has_full_weights(model_path):
    if model_path is None:
        return False
    p = Path(model_path)
    return (p / "model.safetensors").exists() or any(p.glob("*.pt"))


def _promote_scalar_params_to_1d(model):
    for name, param in list(model.named_parameters()):
        if param.ndim == 0:
            parts = name.rsplit(".", 1)
            if len(parts) == 2:
                parent = dict(model.named_modules())[parts[0]]
                setattr(
                    parent,
                    parts[1],
                    torch.nn.Parameter(
                        param.data.unsqueeze(0), requires_grad=param.requires_grad
                    ),
                )


def get_model(cfg: DictConfig, torch_dtype=None):
    from fastwam.runtime import create_fastwam

    torch_dtype = torch_dtype or torch.bfloat16
    model_path = cfg.get("model_path", None)

    def _to_dict(x):
        if x is None:
            return {}
        if isinstance(x, DictConfig):
            return OmegaConf.to_container(x, resolve=True)
        return dict(x)

    fastwam_model = create_fastwam(
        model_id=cfg.get("model_id", "Wan-AI/Wan2.2-TI2V-5B"),
        tokenizer_model_id=cfg.get("tokenizer_model_id", "Wan-AI/Wan2.1-T2V-1.3B"),
        tokenizer_max_len=int(cfg.get("tokenizer_max_len", 128)),
        load_text_encoder=cfg.get("load_text_encoder", False),
        proprio_dim=cfg.get("proprio_dim", None),
        video_dit_config=_to_dict(cfg.get("video_dit_config")),
        action_dit_config=_to_dict(cfg.get("action_dit_config")),
        action_dit_pretrained_path=cfg.get("action_dit_pretrained_path", None),
        skip_dit_load_from_pretrain=_has_full_weights(model_path),
        mot_checkpoint_mixed_attn=cfg.get("mot_checkpoint_mixed_attn", True),
        video_scheduler=_to_dict(cfg.get("video_scheduler")),
        action_scheduler=_to_dict(cfg.get("action_scheduler")),
        loss=_to_dict(cfg.get("loss")),
        redirect_common_files=cfg.get("redirect_common_files", True),
        model_dtype=torch_dtype,
        device="cpu",
    )

    if model_path is not None:
        ckpt_path = Path(model_path)
        if any(ckpt_path.glob("*.pt")):
            fastwam_model.load_checkpoint(str(sorted(ckpt_path.glob("*.pt"))[-1]))

    fastwam_model.vae.requires_grad_(False)
    if fastwam_model.text_encoder is not None:
        fastwam_model.text_encoder.requires_grad_(False)

    policy = FastWAMPolicy(fastwam_model, FastWAMConfig.from_hydra(cfg))
    _promote_scalar_params_to_1d(policy)
    return policy.to(dtype=torch_dtype)
