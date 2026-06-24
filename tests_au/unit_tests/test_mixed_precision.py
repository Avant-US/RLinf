"""Tests for §9.3 mixed precision alignment (fp32 master + bf16 compute).

Covers:
 1. torch_dtype_from_precision mapping for the config values we use.
 2. The conditional-cast decision in openpi_au/__init__.py (fp32_master_weights).
 3. Real FSDP MixedPrecision semantics: sharded master params stay fp32 while
    compute runs in bf16 (single-process FSDP, GPU-only, skippable).
 4. Adam optimizer moments stay fp32 when master weights are fp32.
"""

import os

import pytest
import torch
import torch.nn as nn


def _torch_dtype_from_precision(precision):
    """Local copy of rlinf.config.torch_dtype_from_precision (avoid heavy import)."""
    if precision in ["bf16", "bf16-mixed"]:
        return torch.bfloat16
    elif precision in [16, "16", "fp16", "16-mixed"]:
        return torch.float16
    elif precision in [32, "32", "fp32", "32-true"]:
        return torch.float32
    elif precision in [None, "null"]:
        return None
    else:
        raise ValueError(f"Could not parse precision `{precision}`")


def _should_cast_to_bf16(cfg: dict) -> bool:
    """Reproduce openpi_au/__init__.py decision: cast unless fp32_master_weights."""
    return not bool(cfg.get("fp32_master_weights", False))


def test_config_dtype_mapping():
    assert _torch_dtype_from_precision("bf16") == torch.bfloat16
    assert _torch_dtype_from_precision("fp32") == torch.float32
    assert _torch_dtype_from_precision(None) is None


def test_precision_strings_that_must_not_be_used():
    """`mixed_bf16` / `bfloat16` are NOT valid precision strings — that is exactly
    why we route fp32-master through a dedicated flag and keep precision=null."""
    with pytest.raises(ValueError):
        _torch_dtype_from_precision("mixed_bf16")
    with pytest.raises(ValueError):
        _torch_dtype_from_precision("bfloat16")


def test_conditional_cast_decision():
    assert _should_cast_to_bf16({"fp32_master_weights": True}) is False
    assert _should_cast_to_bf16({"fp32_master_weights": False}) is True
    assert _should_cast_to_bf16({}) is True  # default = legacy behavior


def test_fp32_master_keeps_param_dtype():
    """Simulate model load: with fp32_master, params stay fp32."""
    model = nn.Linear(8, 4)  # fp32 by default
    cfg = {"fp32_master_weights": True}
    if _should_cast_to_bf16(cfg):
        model = model.to(torch.bfloat16)
    assert model.weight.dtype == torch.float32


def test_legacy_cast_to_bf16():
    model = nn.Linear(8, 4)
    cfg = {"fp32_master_weights": False}
    if _should_cast_to_bf16(cfg):
        model = model.to(torch.bfloat16)
    assert model.weight.dtype == torch.bfloat16


def test_adam_moments_fp32_when_master_fp32():
    """With fp32 master weights, AdamW first/second moments are fp32."""
    model = nn.Linear(8, 4)  # fp32
    opt = torch.optim.AdamW(model.parameters(), lr=1e-3)
    x = torch.randn(2, 8)
    loss = model(x).sum()
    loss.backward()
    opt.step()
    state = opt.state[model.weight]
    assert state["exp_avg"].dtype == torch.float32
    assert state["exp_avg_sq"].dtype == torch.float32


@pytest.mark.gpu
def test_autocast_bf16_compute_keeps_fp32_master():
    """fp32 master weights + bf16 autocast compute: weights stay fp32,
    forward activations are bf16."""
    model = nn.Linear(16, 16).cuda()  # fp32 master
    x = torch.randn(4, 16, device="cuda")
    with torch.autocast(device_type="cuda", dtype=torch.bfloat16):
        out = model(x)
    assert out.dtype == torch.bfloat16, "compute should be bf16 under autocast"
    assert model.weight.dtype == torch.float32, "master weight should stay fp32"


@pytest.mark.gpu
def test_fsdp_mixed_precision_master_fp32():
    """Real single-process FSDP with MixedPrecision(param=bf16, reduce=fp32):
    the local sharded flat-param stays fp32 (master), compute casts to bf16."""
    if not torch.distributed.is_available():
        pytest.skip("torch.distributed unavailable")

    from torch.distributed.fsdp import FullyShardedDataParallel as FSDP
    from torch.distributed.fsdp import MixedPrecision

    os.environ.setdefault("MASTER_ADDR", "127.0.0.1")
    os.environ.setdefault("MASTER_PORT", "29555")
    os.environ.setdefault("RANK", "0")
    os.environ.setdefault("WORLD_SIZE", "1")
    if not torch.distributed.is_initialized():
        torch.distributed.init_process_group(backend="nccl", rank=0, world_size=1)

    try:
        model = nn.Sequential(nn.Linear(32, 32), nn.ReLU(), nn.Linear(32, 32)).cuda()
        mp = MixedPrecision(
            param_dtype=torch.bfloat16,
            reduce_dtype=torch.float32,
            buffer_dtype=torch.float32,
        )
        fsdp_model = FSDP(model, mixed_precision=mp, device_id=torch.cuda.current_device())

        # Underlying flat parameter (master) must remain fp32.
        master_dtypes = {p.dtype for p in fsdp_model.parameters()}
        assert torch.float32 in master_dtypes, f"master params not fp32: {master_dtypes}"

        # Forward computes in bf16 (output dtype bf16 under FSDP MixedPrecision).
        x = torch.randn(8, 32, device="cuda")
        out = fsdp_model(x)
        assert out.dtype == torch.bfloat16, f"compute dtype {out.dtype} != bf16"
    finally:
        if torch.distributed.is_initialized():
            torch.distributed.destroy_process_group()
