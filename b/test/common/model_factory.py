"""Shared model creation, batch generation, and weight sync utilities."""

import os
import sys
import copy

import torch

FASTWAM_ROOT = os.environ.get("FASTWAM_ROOT", "/home/Luogang/SRC/Robot/FastWAM")
sys.path.insert(0, os.path.join(FASTWAM_ROOT, "src"))

MODEL_CFG = {
    "model_id": "Wan-AI/Wan2.2-TI2V-5B",
    "tokenizer_model_id": "Wan-AI/Wan2.1-T2V-1.3B",
    "tokenizer_max_len": 128,
    "load_text_encoder": False,
    "proprio_dim": 23,
    "video_dit_config": {
        "has_image_input": False, "patch_size": [1, 2, 2], "in_dim": 48,
        "hidden_dim": 3072, "ffn_dim": 14336, "freq_dim": 256, "text_dim": 4096,
        "out_dim": 48, "num_heads": 24, "attn_head_dim": 128, "num_layers": 30,
        "eps": 1e-6, "seperated_timestep": True, "require_clip_embedding": False,
        "require_vae_embedding": False, "fuse_vae_embedding_in_latents": True,
        "use_gradient_checkpointing": False, "video_attention_mask_mode": "first_frame_causal",
        "action_conditioned": False, "action_dim": 23,
        "action_group_causal_mask_mode": "group_diagonal",
    },
    "action_dit_config": {
        "action_dim": 23, "hidden_dim": 1024, "ffn_dim": 4096,
        "num_heads": 24, "attn_head_dim": 128, "num_layers": 30,
        "text_dim": 4096, "freq_dim": 256, "eps": 1e-6,
        "use_gradient_checkpointing": False,
    },
    "action_dit_pretrained_path": os.path.join(
        os.environ.get("DIFFSYNTH_MODEL_BASE_PATH", ""),
        "ActionDiT_linear_interp_Wan22_alphascale_1024hdim.pt",
    ),
    "mot_checkpoint_mixed_attn": False,
    "video_scheduler": {"train_shift": 5.0, "infer_shift": 5.0, "num_train_timesteps": 1000},
    "action_scheduler": {"train_shift": 5.0, "infer_shift": 5.0, "num_train_timesteps": 1000},
    "loss": {"lambda_action": 1.0},
}


def create_native(device="cuda:0", dtype=torch.bfloat16):
    os.environ.setdefault("DIFFSYNTH_SKIP_DOWNLOAD", "true")
    from fastwam.runtime import create_fastwam
    return create_fastwam(**MODEL_CFG, device=str(device), model_dtype=dtype)


def apply_native_train_mode(model):
    from fastwam.trainer import Wan22Trainer
    Wan22Trainer._apply_dit_only_train_mode(model)


def wrap_in_policy(native_model):
    from rlinf.models.embodiment.fastwam.fastwam_policy import FastWAMPolicy
    from rlinf.models.embodiment.fastwam.fastwam_config import FastWAMConfig
    policy = FastWAMPolicy(native_model, FastWAMConfig())
    policy.train()
    return policy


def make_batch(device, dtype, seed=42):
    gen = torch.Generator(device="cpu").manual_seed(seed)
    return {
        "video": torch.randn(1, 3, 9, 384, 320, generator=gen).to(device=device, dtype=dtype),
        "action": torch.randn(1, 32, 23, generator=gen).to(device=device, dtype=dtype),
        "proprio": torch.randn(1, 32, 23, generator=gen).to(device=device, dtype=dtype),
        "context": torch.randn(1, 128, 4096, generator=gen).to(device=device, dtype=dtype),
        "context_mask": torch.ones(1, 128, device=device, dtype=torch.bool),
        "image_is_pad": torch.zeros(1, 9, device=device, dtype=torch.bool),
        "action_is_pad": torch.zeros(1, 32, device=device, dtype=torch.bool),
    }
