"""
FastWAM numerical alignment tests: RLinf wrapper vs native FastWAM.

Tests T0–T3 from fw_sft_design_op46_4_r1pr_cp25_2tst.md.

Requirements:
  - 1 GPU
  - DIFFSYNTH_MODEL_BASE_PATH env var pointing to FastWAM pretrained weights
  - FASTWAM_ROOT / FASTWAM_PATH env vars (or defaults)

Usage:
  CUDA_VISIBLE_DEVICES=7 pytest tests/unit_tests/test_fastwam_alignment.py -v -s
"""

import os
import sys
import copy

import pytest
import torch

FASTWAM_ROOT = os.environ.get("FASTWAM_ROOT", "/home/Luogang/SRC/Robot/FastWAM")
sys.path.insert(0, os.path.join(FASTWAM_ROOT, "src"))

requires_gpu = pytest.mark.skipif(not torch.cuda.is_available(), reason="GPU required")
requires_weights = pytest.mark.skipif(
    not os.path.exists(os.environ.get("DIFFSYNTH_MODEL_BASE_PATH", "")),
    reason="DIFFSYNTH_MODEL_BASE_PATH not set or missing",
)

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

RLINF_MODEL_CFG_EXTRA = {
    "model_type": "fastwam",
    "is_lora": False,
    "redirect_common_files": True,
}


def _create_native_model(device="cpu", dtype=torch.bfloat16):
    os.environ.setdefault("DIFFSYNTH_SKIP_DOWNLOAD", "true")
    from fastwam.runtime import create_fastwam
    return create_fastwam(**MODEL_CFG, device=str(device), model_dtype=dtype)


def _create_rlinf_policy(device="cpu", dtype=torch.bfloat16):
    from omegaconf import OmegaConf
    from rlinf.models.embodiment.fastwam import get_model
    cfg = OmegaConf.create({**MODEL_CFG, **RLINF_MODEL_CFG_EXTRA})
    policy = get_model(cfg, dtype)
    return policy.to(device)


def _make_batch(device, dtype, seed=42):
    gen = torch.Generator(device="cpu").manual_seed(seed)
    B = 1
    return {
        "video": torch.randn(B, 3, 9, 384, 320, generator=gen).to(device=device, dtype=dtype),
        "action": torch.randn(B, 32, 23, generator=gen).to(device=device, dtype=dtype),
        "proprio": torch.randn(B, 32, 23, generator=gen).to(device=device, dtype=dtype),
        "context": torch.randn(B, 128, 4096, generator=gen).to(device=device, dtype=dtype),
        "context_mask": torch.ones(B, 128, device=device, dtype=torch.bool),
        "image_is_pad": torch.zeros(B, 9, device=device, dtype=torch.bool),
        "action_is_pad": torch.zeros(B, 32, device=device, dtype=torch.bool),
    }


def _strip_prefix(name, prefix):
    return name[len(prefix):] if name.startswith(prefix) else name


# ──────────────────────────────────────────────────────────────────
# T0: Trainable parameter set alignment
# ──────────────────────────────────────────────────────────────────


@requires_gpu
@requires_weights
def test_t0_trainable_params_match():
    """T0: The set of requires_grad=True parameter names must match."""
    from fastwam.trainer import Wan22Trainer

    native = _create_native_model("cpu")
    Wan22Trainer._apply_dit_only_train_mode(native)
    native_trainable = {n for n, p in native.named_parameters() if p.requires_grad}

    policy = _create_rlinf_policy("cpu")
    policy.train()
    rlinf_trainable = {
        _strip_prefix(n, "fastwam.") for n, p in policy.named_parameters() if p.requires_grad
    }

    only_native = native_trainable - rlinf_trainable
    only_rlinf = rlinf_trainable - native_trainable
    assert not only_native and not only_rlinf, (
        f"Trainable param mismatch.\n"
        f"  Native-only ({len(only_native)}): {sorted(only_native)[:5]}...\n"
        f"  RLinf-only ({len(only_rlinf)}): {sorted(only_rlinf)[:5]}..."
    )
    print(f"T0 PASS: {len(native_trainable)} trainable params match exactly.")


# ──────────────────────────────────────────────────────────────────
# T1: Single-step loss alignment (same batch + same RNG)
# ──────────────────────────────────────────────────────────────────


@requires_gpu
@requires_weights
def test_t1_single_step_loss():
    """T1: Given identical weights, batch, and RNG, loss must match.

    Strategy: create ONE native model, wrap in FastWAMPolicy (shared weights).
    Both paths use torch.amp.autocast(bf16) — matching FastWAM native
    (accelerator.autocast) and RLinf (amp_autocast.enabled=true).
    """
    device = torch.device("cuda:0")
    dtype = torch.bfloat16

    native = _create_native_model(device, dtype)
    from fastwam.trainer import Wan22Trainer
    Wan22Trainer._apply_dit_only_train_mode(native)

    from rlinf.models.embodiment.fastwam.fastwam_policy import FastWAMPolicy
    from rlinf.models.embodiment.fastwam.fastwam_config import FastWAMConfig
    policy = FastWAMPolicy(native, FastWAMConfig())
    policy.to(device)
    policy.train()

    batch = _make_batch(device, dtype, seed=123)

    # --- native forward (with autocast, matching accelerator.autocast()) ---
    rng_state = torch.cuda.get_rng_state(device)
    torch.cuda.set_rng_state(rng_state, device)
    with torch.amp.autocast("cuda", dtype=dtype):
        loss_n, dict_n = native.training_loss(batch)

    # --- rlinf forward (with autocast, matching amp_autocast.enabled=true) ---
    torch.cuda.set_rng_state(rng_state, device)
    with torch.amp.autocast("cuda", dtype=dtype):
        out_r = policy.sft_forward(data=batch)
    loss_r = out_r["loss"]

    diff = (loss_n - loss_r).abs().item()
    print(f"T1: native_loss={loss_n.item():.6f}  rlinf_loss={loss_r.item():.6f}  diff={diff:.2e}")
    assert torch.allclose(loss_n, loss_r, rtol=1e-5, atol=1e-5), (
        f"T1 FAIL: loss diff={diff:.6f}"
    )
    print("T1 PASS")


# ──────────────────────────────────────────────────────────────────
# T2: Multi-step gradient & weight alignment
# ──────────────────────────────────────────────────────────────────


@requires_gpu
@requires_weights
def test_t2_multi_step_weight_alignment():
    """T2: After 3 identical training steps, weights should closely match.

    Create one native model, deep-copy it, wrap the copy in FastWAMPolicy.
    Both use identical AdamW, identical batches, identical RNG per step.
    """
    N = 3
    device = torch.device("cuda:0")
    dtype = torch.bfloat16
    lr = 1e-4

    native = _create_native_model(device, dtype)
    from fastwam.trainer import Wan22Trainer
    Wan22Trainer._apply_dit_only_train_mode(native)

    from rlinf.models.embodiment.fastwam.fastwam_policy import FastWAMPolicy
    from rlinf.models.embodiment.fastwam.fastwam_config import FastWAMConfig

    native_copy = copy.deepcopy(native)
    Wan22Trainer._apply_dit_only_train_mode(native_copy)
    policy = FastWAMPolicy(native_copy, FastWAMConfig())
    policy.to(device)
    policy.train()

    native_params = list(native.dit.parameters())
    if native.proprio_encoder is not None:
        native_params += list(native.proprio_encoder.parameters())
    opt_n = torch.optim.AdamW(native_params, lr=lr, betas=(0.9, 0.95), weight_decay=1e-2)

    rlinf_params = [p for p in policy.parameters() if p.requires_grad]
    opt_r = torch.optim.AdamW(rlinf_params, lr=lr, betas=(0.9, 0.95), weight_decay=1e-2)

    batches = [_make_batch(device, dtype, seed=200 + i) for i in range(N)]
    rng_states = []
    for i in range(N):
        torch.cuda.manual_seed(9999 + i)
        rng_states.append(torch.cuda.get_rng_state(device))

    losses_n, losses_r = [], []
    for step in range(N):
        opt_n.zero_grad(set_to_none=True)
        torch.cuda.set_rng_state(rng_states[step], device)
        with torch.amp.autocast("cuda", dtype=dtype):
            loss_n, _ = native.training_loss(batches[step])
        loss_n.backward()
        torch.nn.utils.clip_grad_norm_(native.parameters(), 1.0)
        opt_n.step()
        losses_n.append(loss_n.detach().item())

        opt_r.zero_grad(set_to_none=True)
        torch.cuda.set_rng_state(rng_states[step], device)
        with torch.amp.autocast("cuda", dtype=dtype):
            out = policy.sft_forward(data=batches[step])
        loss_r = out["loss"]
        loss_r.backward()
        torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
        opt_r.step()
        losses_r.append(loss_r.detach().item())

        diff = abs(losses_n[-1] - losses_r[-1])
        print(f"  step {step}: native={losses_n[-1]:.6f}  rlinf={losses_r[-1]:.6f}  diff={diff:.2e}")

    native_final = native.state_dict()
    rlinf_final = policy.state_dict()
    max_diff = 0.0
    n_compared = 0
    for nk in native_final:
        rk = f"fastwam.{nk}"
        if rk in rlinf_final and native_final[nk].is_floating_point():
            d = (native_final[nk].float() - rlinf_final[rk].float()).abs().max().item()
            max_diff = max(max_diff, d)
            n_compared += 1
    print(f"T2: compared {n_compared} tensors, max weight diff = {max_diff:.2e}")
    assert max_diff < 1e-3, f"T2 FAIL: max weight diff = {max_diff}"
    print("T2 PASS")


# ──────────────────────────────────────────────────────────────────
# T3: Checkpoint interop (RLinf → native)
# ──────────────────────────────────────────────────────────────────


@requires_gpu
@requires_weights
def test_t3_checkpoint_interop():
    """T3: RLinf checkpoint → native model load → identical forward output.

    1. Create native model, wrap in policy, train 1 step.
    2. Extract state_dict via fastwam_save_helper key mapping.
    3. Load into a fresh native model.
    4. Compare forward on same batch + RNG.
    """
    device = torch.device("cuda:0")
    dtype = torch.bfloat16

    from rlinf.models.embodiment.fastwam.fastwam_policy import FastWAMPolicy
    from rlinf.models.embodiment.fastwam.fastwam_config import FastWAMConfig
    from fastwam.trainer import Wan22Trainer

    native_src = _create_native_model(device, dtype)
    Wan22Trainer._apply_dit_only_train_mode(native_src)
    policy = FastWAMPolicy(native_src, FastWAMConfig())
    policy.to(device)
    policy.train()

    batch = _make_batch(device, dtype, seed=77)
    opt = torch.optim.AdamW(
        [p for p in policy.parameters() if p.requires_grad],
        lr=1e-4, betas=(0.9, 0.95), weight_decay=1e-2,
    )
    opt.zero_grad(set_to_none=True)
    torch.cuda.manual_seed(555)
    with torch.amp.autocast("cuda", dtype=dtype):
        out = policy.sft_forward(data=batch)
    out["loss"].backward()
    opt.step()

    # --- extract mot + proprio_encoder (fastwam_save_helper logic) ---
    rlinf_sd = policy.state_dict()
    mot_sd = {k.replace("fastwam.mot.", ""): v for k, v in rlinf_sd.items() if k.startswith("fastwam.mot.")}
    pe_sd = {k.replace("fastwam.proprio_encoder.", ""): v for k, v in rlinf_sd.items() if k.startswith("fastwam.proprio_encoder.")}

    # --- load into fresh native model ---
    native_dst = _create_native_model(device, dtype)
    native_dst.mot.load_state_dict(mot_sd, strict=False)
    if pe_sd and native_dst.proprio_encoder is not None:
        native_dst.proprio_encoder.load_state_dict(pe_sd, strict=True)
    Wan22Trainer._apply_dit_only_train_mode(native_dst)

    # --- compare forward (both with autocast, matching production) ---
    test_batch = _make_batch(device, dtype, seed=999)
    rng = torch.cuda.get_rng_state(device)

    torch.cuda.set_rng_state(rng, device)
    with torch.amp.autocast("cuda", dtype=dtype):
        loss_n, _ = native_dst.training_loss(test_batch)

    torch.cuda.set_rng_state(rng, device)
    with torch.amp.autocast("cuda", dtype=dtype):
        out2 = policy.sft_forward(data=test_batch)
    loss_r = out2["loss"]

    diff = (loss_n - loss_r).abs().item()
    print(f"T3: native_loss={loss_n.item():.6f}  rlinf_loss={loss_r.item():.6f}  diff={diff:.2e}")
    assert torch.allclose(loss_n, loss_r, rtol=1e-5, atol=1e-5), f"T3 FAIL: diff={diff}"
    print("T3 PASS")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
