"""Unit tests for openpi_cosine LR scheduler."""

import math

import pytest
import torch
from torch.optim.lr_scheduler import LambdaLR


def _build_openpi_cosine(optimizer, optim_config):
    """LR schedule numerically equivalent to optax warmup_cosine_decay_schedule.
    Duplicated here to avoid pulling in the full rlinf worker dep chain."""
    peak = float(optim_config.lr)
    end_lr = float(optim_config.get("decay_lr", peak))
    warmup = int(optim_config.get("lr_warmup_steps", 0))
    decay_steps = int(optim_config.get("decay_steps", optim_config.get("total_training_steps", 30000)))

    def lr_lambda(step):
        if warmup > 0 and step < warmup:
            return (step + 1) / warmup
        if step >= decay_steps:
            return end_lr / peak
        prog = (step - warmup) / max(1, decay_steps - warmup)
        cos_val = end_lr + 0.5 * (peak - end_lr) * (1.0 + math.cos(math.pi * prog))
        return cos_val / peak

    return LambdaLR(optimizer, lr_lambda)


class FakeOptimConfig:
    def __init__(self, peak=5e-5, end_lr=1e-6, warmup=100, decay_steps=1000):
        self.lr = peak
        self.decay_lr = end_lr
        self.lr_warmup_steps = warmup
        self.decay_steps = decay_steps
        self.total_training_steps = decay_steps

    def get(self, key, default=None):
        return getattr(self, key, default)


@pytest.fixture
def optimizer():
    model = torch.nn.Linear(10, 5)
    return torch.optim.Adam(model.parameters(), lr=5e-5)


def _collect_lrs(optimizer, config, steps):
    sched = _build_openpi_cosine(optimizer, config)
    optimizer.step()
    lrs = []
    for _ in range(steps):
        sched.step()
        lrs.append(optimizer.param_groups[0]["lr"])
    return lrs


def test_warmup_ramp(optimizer):
    config = FakeOptimConfig(peak=5e-5, end_lr=1e-6, warmup=100, decay_steps=1000)
    lrs = _collect_lrs(optimizer, config, 200)
    assert lrs[0] < lrs[50] < lrs[99], "LR should ramp during warmup"


def test_peak_at_warmup_end(optimizer):
    config = FakeOptimConfig(peak=5e-5, end_lr=1e-6, warmup=100, decay_steps=1000)
    lrs = _collect_lrs(optimizer, config, 200)
    assert abs(lrs[99] - 5e-5) < 1e-8, f"Expected peak 5e-5, got {lrs[99]}"


def test_decay_after_warmup(optimizer):
    config = FakeOptimConfig(peak=5e-5, end_lr=1e-6, warmup=100, decay_steps=1000)
    lrs = _collect_lrs(optimizer, config, 900)
    assert lrs[100] > lrs[500] > lrs[898], "LR should decay after warmup"


def test_end_lr_reached(optimizer):
    config = FakeOptimConfig(peak=5e-5, end_lr=1e-6, warmup=100, decay_steps=1000)
    lrs = _collect_lrs(optimizer, config, 1100)
    assert abs(lrs[-1] - 1e-6) < 1e-8, f"Expected end_lr 1e-6, got {lrs[-1]}"


def test_constant_when_end_eq_peak(optimizer):
    config = FakeOptimConfig(peak=5e-5, end_lr=5e-5, warmup=100, decay_steps=1000)
    lrs = _collect_lrs(optimizer, config, 500)
    for lr in lrs[100:]:
        assert abs(lr - 5e-5) < 1e-8, f"Expected constant 5e-5, got {lr}"


def test_zero_warmup(optimizer):
    config = FakeOptimConfig(peak=5e-5, end_lr=1e-6, warmup=0, decay_steps=1000)
    lrs = _collect_lrs(optimizer, config, 10)
    assert abs(lrs[0] - 5e-5) < 1e-7, f"With 0 warmup, first step should be peak; got {lrs[0]}"
