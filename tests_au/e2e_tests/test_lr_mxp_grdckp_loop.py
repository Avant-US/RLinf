"""Synthetic e2e test combining §9.2 LR + §9.3 mixed precision + §9.6 grad-ckpt.

A tiny action model is trained for a few steps with:
 - openpi_cosine LR schedule (warmup -> cosine),
 - fp32 master weights + bf16 autocast compute,
 - activation checkpointing on the block stack.
Verifies the loop runs, loss decreases, weights stay fp32, and LR follows warmup.

No openpi weights/data required (Python 3.10 friendly).
"""

import math

import pytest
import torch
import torch.nn as nn
from torch.optim.lr_scheduler import LambdaLR
from torch.utils.checkpoint import checkpoint


def _build_openpi_cosine(optimizer, lr, decay_lr, warmup, decay_steps):
    init = lr / (warmup + 1) if warmup > 0 else lr

    def lr_lambda(step):
        if warmup > 0 and step < warmup:
            return (init + (lr - init) * step / warmup) / lr
        if step >= decay_steps:
            return decay_lr / lr
        prog = (step - warmup) / max(1, decay_steps - warmup)
        return (decay_lr + 0.5 * (lr - decay_lr) * (1 + math.cos(math.pi * prog))) / lr

    return LambdaLR(optimizer, lr_lambda)


class TinyVLA(nn.Module):
    def __init__(self, img_dim=3 * 32 * 32, hidden=128, action_dim=7, n_blocks=4, use_ckpt=True):
        super().__init__()
        self.proj = nn.Linear(img_dim, hidden)
        self.blocks = nn.ModuleList(
            [nn.Sequential(nn.Linear(hidden, hidden), nn.GELU(), nn.Linear(hidden, hidden)) for _ in range(n_blocks)]
        )
        self.head = nn.Linear(hidden, action_dim)
        self.use_ckpt = use_ckpt

    def forward(self, images_bhwc, actions):
        b = images_bhwc.shape[0]
        x = self.proj(images_bhwc.reshape(b, -1))
        for blk in self.blocks:
            if self.use_ckpt and self.training:
                x = checkpoint(blk, x, use_reentrant=False)
            else:
                x = blk(x)
        pred = self.head(x)
        return (pred - actions[:, 0, :]).pow(2).mean()


def _make_batch(bs=16, device="cpu"):
    return (
        torch.rand(bs, 32, 32, 3, device=device),
        torch.randn(bs, 10, 7, device=device),
    )


@pytest.mark.gpu
def test_combined_training_loop_gpu():
    torch.manual_seed(0)
    device = "cuda"
    model = TinyVLA(use_ckpt=True).to(device)  # fp32 master
    opt = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-10)
    sched = _build_openpi_cosine(opt, lr=1e-3, decay_lr=1e-4, warmup=5, decay_steps=50)

    losses, lrs = [], []
    for step in range(40):
        model.train()
        images, actions = _make_batch(device=device)
        with torch.autocast(device_type="cuda", dtype=torch.bfloat16):
            loss = model(images, actions)
        opt.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        lrs.append(opt.param_groups[0]["lr"])
        opt.step()
        sched.step()
        losses.append(loss.item())

    # master weights stay fp32
    assert model.proj.weight.dtype == torch.float32
    # warmup ramps up; peak reached at scheduler step == warmup (index 5)
    assert lrs[0] < lrs[4], "LR should ramp during warmup"
    assert abs(lrs[5] - 1e-3) < 1e-9, "peak at end of warmup (step == warmup)"
    # cosine decay after peak
    assert lrs[6] < lrs[5], "cosine decay after peak"
    # loss decreases overall
    assert min(losses[-5:]) < losses[0]


def test_combined_training_loop_cpu():
    """Same loop on CPU without autocast (bf16 autocast on CPU is flaky)."""
    torch.manual_seed(0)
    model = TinyVLA(use_ckpt=True)
    opt = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-10)
    sched = _build_openpi_cosine(opt, lr=1e-3, decay_lr=1e-3, warmup=5, decay_steps=1_000_000)

    losses, lrs = [], []
    for step in range(30):
        model.train()
        images, actions = _make_batch()
        loss = model(images, actions)
        opt.zero_grad()
        loss.backward()
        lrs.append(opt.param_groups[0]["lr"])
        opt.step()
        sched.step()
        losses.append(loss.item())

    assert model.proj.weight.dtype == torch.float32
    assert lrs[0] < lrs[4]
    # decay_lr == peak ⇒ constant after warmup
    for lr in lrs[5:]:
        assert abs(lr - 1e-3) < 1e-9
    assert min(losses[-5:]) < losses[0]


def test_checkpoint_equivalence_in_loop():
    """A single step with/without checkpointing gives identical grads in the loop."""
    torch.manual_seed(0)
    m_ckpt = TinyVLA(use_ckpt=True)
    m_plain = TinyVLA(use_ckpt=False)
    m_plain.load_state_dict(m_ckpt.state_dict())
    m_ckpt.train()
    m_plain.train()

    images, actions = _make_batch()
    m_ckpt(images.clone(), actions.clone()).backward()
    m_plain(images.clone(), actions.clone()).backward()

    for (_, p1), (_, p2) in zip(m_ckpt.named_parameters(), m_plain.named_parameters()):
        torch.testing.assert_close(p1.grad, p2.grad, atol=1e-5, rtol=1e-4)
