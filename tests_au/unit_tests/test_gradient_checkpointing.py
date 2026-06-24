"""Tests for §9.6 gradient checkpointing -> enlarge effective batch.

Covers:
 1. The sft_forward toggle decision (disable recompute unless flag set).
 2. Numerical equivalence: checkpointed forward yields identical gradients
    to the non-checkpointed forward (compute-for-memory trade only).
 3. Effective-batch arithmetic to reach openpi's batch_size=256.
 4. (GPU) peak activation memory is lower with checkpointing on a deep stack.
"""

import pytest
import torch
import torch.nn as nn
from torch.utils.checkpoint import checkpoint


def _should_disable_recompute(cfg: dict) -> bool:
    """Reproduce openpi_au sft_forward decision:
    disable activation recomputation UNLESS sft_gradient_checkpointing is set."""
    return not bool(cfg.get("sft_gradient_checkpointing", False))


def test_toggle_decision():
    assert _should_disable_recompute({"sft_gradient_checkpointing": True}) is False
    assert _should_disable_recompute({"sft_gradient_checkpointing": False}) is True
    assert _should_disable_recompute({}) is True  # default: legacy disable


class DeepBlockStack(nn.Module):
    def __init__(self, dim=64, n_blocks=8, use_ckpt=False):
        super().__init__()
        self.blocks = nn.ModuleList(
            [nn.Sequential(nn.Linear(dim, dim), nn.GELU(), nn.Linear(dim, dim)) for _ in range(n_blocks)]
        )
        self.use_ckpt = use_ckpt

    def forward(self, x):
        for blk in self.blocks:
            if self.use_ckpt and self.training:
                x = checkpoint(blk, x, use_reentrant=False)
            else:
                x = blk(x)
        return x


def _clone_model(src, use_ckpt):
    dst = DeepBlockStack(use_ckpt=use_ckpt)
    dst.load_state_dict(src.state_dict())
    return dst


def test_checkpoint_gradient_equivalence():
    """Checkpointed and non-checkpointed models must produce identical grads."""
    torch.manual_seed(0)
    base = DeepBlockStack(use_ckpt=False)
    ckpt_model = _clone_model(base, use_ckpt=True)

    base.train()
    ckpt_model.train()
    x = torch.randn(16, 64)

    out_base = base(x.clone())
    out_base.pow(2).mean().backward()

    out_ckpt = ckpt_model(x.clone())
    out_ckpt.pow(2).mean().backward()

    # Forward outputs identical
    torch.testing.assert_close(out_base, out_ckpt, atol=1e-6, rtol=1e-5)
    # Gradients identical
    for (n1, p1), (n2, p2) in zip(base.named_parameters(), ckpt_model.named_parameters()):
        assert n1 == n2
        torch.testing.assert_close(p1.grad, p2.grad, atol=1e-5, rtol=1e-4)


@pytest.mark.parametrize(
    "global_bs,micro_bs,world_size,expected_accum",
    [
        (256, 8, 1, 32),   # grad-ckpt lets micro grow 4->8 on one GPU
        (256, 16, 2, 8),   # bigger micro on 2 GPUs
        (256, 32, 8, 1),   # 8 GPUs, no accumulation needed
        (256, 8, 4, 8),
    ],
)
def test_effective_batch_reaches_256(global_bs, micro_bs, world_size, expected_accum):
    assert global_bs % (micro_bs * world_size) == 0
    accum = global_bs // micro_bs // world_size
    assert accum == expected_accum
    # effective batch == 256
    assert micro_bs * world_size * accum == 256


def test_micro_batch_growth_keeps_global_constant():
    """Doubling micro_batch (enabled by grad-ckpt) halves grad_accum,
    keeping the effective/global batch fixed."""
    global_bs, world_size = 256, 1
    accum_small = global_bs // 4 // world_size   # micro=4
    accum_large = global_bs // 8 // world_size   # micro=8 with ckpt
    assert 4 * accum_small == 8 * accum_large == 256


def _measure_peak_memory(use_ckpt, dim=512, n_blocks=16, batch=8192):
    """Run one fwd+bwd of a deep stack and return peak allocated bytes.
    Large batch makes activations dominate so checkpointing savings are visible."""
    torch.manual_seed(0)
    model = DeepBlockStack(dim=dim, n_blocks=n_blocks, use_ckpt=use_ckpt).cuda().train()
    x = torch.randn(batch, dim, device="cuda")
    torch.cuda.synchronize()
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()
    out = model(x)
    out.pow(2).mean().backward()
    torch.cuda.synchronize()
    peak = torch.cuda.max_memory_allocated()
    # cleanup so the next measurement is isolated
    del model, x, out
    torch.cuda.empty_cache()
    return peak


@pytest.mark.gpu
def test_peak_memory_lower_with_checkpointing():
    mem_base = _measure_peak_memory(use_ckpt=False)
    mem_ckpt = _measure_peak_memory(use_ckpt=True)
    assert mem_ckpt < mem_base, f"ckpt mem {mem_ckpt} should be < base mem {mem_base}"
