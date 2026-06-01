import os
from dataclasses import dataclass, field
from typing import Optional

from omegaconf import DictConfig


@dataclass
class FastWAMConfig:
    model_type: str = "fastwam"
    model_path: Optional[str] = None
    model_id: str = "Wan-AI/Wan2.2-TI2V-5B"
    tokenizer_model_id: str = "Wan-AI/Wan2.1-T2V-1.3B"
    tokenizer_max_len: int = 128
    load_text_encoder: bool = False
    proprio_dim: Optional[int] = None
    text_embedding_cache_dir: Optional[str] = None
    context_len: int = 128
    mot_checkpoint_mixed_attn: bool = True
    action_dit_pretrained_path: Optional[str] = None
    redirect_common_files: bool = True
    video_dit_config: dict = field(default_factory=dict)
    action_dit_config: dict = field(default_factory=dict)
    video_scheduler: dict = field(default_factory=dict)
    action_scheduler: dict = field(default_factory=dict)
    loss: dict = field(default_factory=dict)

    @classmethod
    def from_hydra(cls, cfg: DictConfig) -> "FastWAMConfig":
        from omegaconf import OmegaConf

        d = OmegaConf.to_container(cfg, resolve=True)
        known_keys = {f.name for f in cls.__dataclass_fields__.values()}
        filtered = {k: v for k, v in d.items() if k in known_keys}
        return cls(**filtered)


def validate_fastwam_sft_model_cfg(model_cfg: DictConfig) -> DictConfig:
    cache_dir = model_cfg.get("text_embedding_cache_dir", None)
    if cache_dir is not None:
        assert os.path.isdir(cache_dir), (
            f"text_embedding_cache_dir does not exist: {cache_dir}"
        )

    action_sched = model_cfg.get("action_scheduler", None)
    if action_sched is not None:
        for key in ["train_shift", "infer_shift", "num_train_timesteps"]:
            assert action_sched.get(key) is not None, (
                f"action_scheduler.{key} is required"
            )

    return model_cfg
