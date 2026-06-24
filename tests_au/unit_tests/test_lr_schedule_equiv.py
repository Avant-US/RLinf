"""Numerical-equivalence tests for openpi_cosine LR schedule vs optax.

Validates that _build_openpi_cosine reproduces
optax.warmup_cosine_decay_schedule(
    init_value=peak/(warmup+1), peak_value=peak,
    warmup_steps=warmup, decay_steps=decay, end_value=decay_lr)
exactly (the schedule openpi's CosineDecaySchedule uses).

When optax is unavailable, falls back to a closed-form reference implementation.
"""

import math

import pytest
import torch
from torch.optim.lr_scheduler import LambdaLR


def _build_openpi_cosine(optimizer, optim_config):
    """Copy of worker's _build_openpi_cosine (exact optax semantics)."""
    peak = float(optim_config.lr)
    end_lr = float(optim_config.get("decay_lr", peak))
    warmup = int(optim_config.get("lr_warmup_steps", 0))
    decay_steps = int(optim_config.get("decay_steps", optim_config.get("total_training_steps", 30000)))
    init = peak / (warmup + 1) if warmup > 0 else peak

    def lr_lambda(step):
        if warmup > 0 and step < warmup:
            lr = init + (peak - init) * step / warmup
            return lr / peak
        if step >= decay_steps:
            return end_lr / peak
        prog = (step - warmup) / max(1, decay_steps - warmup)
        cos_val = end_lr + 0.5 * (peak - end_lr) * (1.0 + math.cos(math.pi * prog))
        return cos_val / peak

    return LambdaLR(optimizer, lr_lambda)


def _reference_optax_closed_form(step, peak, end_lr, warmup, decay_steps):
    """Closed-form reference of optax.warmup_cosine_decay_schedule."""
    init = peak / (warmup + 1) if warmup > 0 else peak
    if warmup > 0 and step < warmup:
        return init + (peak - init) * step / warmup
    count = min(step - warmup, decay_steps - warmup)
    cos = 0.5 * (1.0 + math.cos(math.pi * count / max(1, decay_steps - warmup)))
    alpha = end_lr / peak if peak != 0 else 0.0
    return peak * ((1.0 - alpha) * cos + alpha)


class Cfg:
    def __init__(self, lr, decay_lr, warmup, decay_steps):
        self.lr = lr
        self._d = {"decay_lr": decay_lr, "lr_warmup_steps": warmup, "decay_steps": decay_steps}

    def get(self, k, default=None):
        return self._d.get(k, default)


def _collect(cfg, steps):
    model = torch.nn.Linear(2, 2)
    opt = torch.optim.SGD(model.parameters(), lr=cfg.lr)
    sched = _build_openpi_cosine(opt, cfg)
    out = []
    for _ in range(steps):
        out.append(opt.param_groups[0]["lr"])
        opt.step()
        sched.step()
    return out


@pytest.mark.parametrize(
    "peak,decay_lr,warmup,decay_steps",
    [
        (5e-5, 5e-5, 1000, 1_000_000),   # pi05_libero: warmup then ~constant
        (2.5e-5, 2.5e-6, 1000, 30000),   # CosineDecaySchedule defaults
        (1e-4, 1e-6, 500, 10000),        # generic decay
        (5e-5, 5e-5, 0, 30000),          # no warmup, constant peak
    ],
)
def test_matches_closed_form(peak, decay_lr, warmup, decay_steps):
    cfg = Cfg(peak, decay_lr, warmup, decay_steps)
    steps = min(decay_steps + 100, 3000)
    lrs = _collect(cfg, steps)
    for step, lr in enumerate(lrs):
        ref = _reference_optax_closed_form(step, peak, decay_lr, warmup, decay_steps)
        assert abs(lr - ref) < 1e-12, f"step {step}: got {lr}, ref {ref}"


def test_matches_real_optax():
    optax = pytest.importorskip("optax")
    import jax.numpy as jnp  # noqa: F401

    peak, decay_lr, warmup, decay_steps = 2.5e-5, 2.5e-6, 1000, 30000
    optax_sched = optax.warmup_cosine_decay_schedule(
        init_value=peak / (warmup + 1),
        peak_value=peak,
        warmup_steps=warmup,
        decay_steps=decay_steps,
        end_value=decay_lr,
    )
    cfg = Cfg(peak, decay_lr, warmup, decay_steps)
    lrs = _collect(cfg, 3000)
    for step, lr in enumerate(lrs):
        ref = float(optax_sched(step))
        assert abs(lr - ref) < 1e-10, f"step {step}: torch {lr}, optax {ref}"


def test_warmup_init_value():
    """First step LR == peak/(warmup+1), matching optax init_value."""
    peak, warmup = 5e-5, 1000
    cfg = Cfg(peak, peak, warmup, 1_000_000)
    lrs = _collect(cfg, 2)
    assert abs(lrs[0] - peak / (warmup + 1)) < 1e-12


def test_peak_reached_at_warmup_end():
    peak, warmup = 5e-5, 100
    cfg = Cfg(peak, 1e-6, warmup, 10000)
    lrs = _collect(cfg, warmup + 1)
    assert abs(lrs[warmup] - peak) < 1e-9


def test_constant_after_warmup_when_decay_eq_peak():
    peak, warmup = 5e-5, 100
    cfg = Cfg(peak, peak, warmup, 1_000_000)
    lrs = _collect(cfg, 500)
    for lr in lrs[warmup:]:
        assert abs(lr - peak) < 1e-9


def test_end_value_reached():
    peak, decay_lr, warmup, decay = 1e-4, 1e-6, 100, 1000
    cfg = Cfg(peak, decay_lr, warmup, decay)
    lrs = _collect(cfg, decay + 50)
    assert abs(lrs[-1] - decay_lr) < 1e-10
