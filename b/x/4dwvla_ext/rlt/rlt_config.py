"""RLT Stage 1 training configuration for 4DWVLA."""

import dataclasses
import os
from pathlib import Path


@dataclasses.dataclass
class RLTStage1Config:
    # RLT module hyperparameters
    enable_rlt: bool = True
    rlt_alpha: float = 1.0
    rlt_input_dim: int = 2048
    rlt_embed_dim: int = 1024
    rlt_prefix_seq_len: int = 512
    rlt_num_layers: int = 2
    rlt_num_heads: int = 8
    rlt_mlp_ratio: float = 4.0
    rlt_dropout: float = 0.0
    rlt_image_only: bool = False
    rlt_lr: float = 1e-4

    # Training profile
    train_profile: str = "B"
    action_loss_only: bool = True
    freeze_vision_encoder: bool = True
    train_expert_only: bool = True
    freeze_keypoint_modules: bool = False
    vla_inference_mode: bool = False

    # VLA training
    vla_lr: float = 5e-5
    grad_clip_norm: float = 1.0
    warmup_steps: int = 200
    max_steps: int = 20000
    save_freq: int = 2000
    log_freq: int = 50
    micro_batch_size: int = 1
    gradient_accumulation_steps: int = 8

    # Data
    dataset_repo_id: str = "plug_into_socket_lrb_4D"
    action_mode: str = "abs"
    task_prompt: str = "plug into socket"

    # Paths (container-relative)
    base_checkpoint: str = "/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420"
    dataset_root: str = ""
    output_dir: str = "/workspace/RLinf/b/x/4dwvla_ext/rlt/outputs"

    # Checkpoint loading
    pretrained_path: str = ""

    @classmethod
    def from_yaml(cls, yaml_path: str) -> "RLTStage1Config":
        """Load config from YAML file, with environment variable expansion."""
        import yaml

        with open(yaml_path) as f:
            raw = yaml.safe_load(f)
        for k, v in raw.items():
            if isinstance(v, str) and "$" in v:
                raw[k] = os.path.expandvars(v)
        coerced = {}
        for k, v in raw.items():
            if k not in cls.__dataclass_fields__:
                continue
            ft = cls.__dataclass_fields__[k].type
            if ft is float and isinstance(v, (str, int)):
                v = float(v)
            elif ft is int and isinstance(v, (str, float)):
                v = int(v)
            elif ft is bool and isinstance(v, str):
                v = v.lower() in ("true", "1", "yes")
            coerced[k] = v
        return cls(**coerced)

    def resolve_paths(self):
        """Resolve paths using environment variables."""
        if not self.dataset_root:
            data_dir = os.environ.get("DATA_DIR", "/home/nvidia/bt/dt")
            self.dataset_root = str(Path(data_dir) / self.dataset_repo_id)
        if not self.pretrained_path:
            self.pretrained_path = self.base_checkpoint

    def apply_profile(self):
        """Apply training profile presets."""
        if self.train_profile == "A":
            self.train_expert_only = False
            self.freeze_vision_encoder = False
        elif self.train_profile == "B":
            self.train_expert_only = True
            self.freeze_vision_encoder = True
        elif self.train_profile == "C":
            self.train_expert_only = True
            self.freeze_vision_encoder = True
            self.rlt_alpha = 0.0
