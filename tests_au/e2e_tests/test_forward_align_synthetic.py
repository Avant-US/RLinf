"""Synthetic e2e for the forward-alignment comparison logic.

Uses two small PyTorch MLPs to play the roles of "JAX side" and "PyTorch side" so the
comparison/injection machinery (compare_utils) is validated without the real 3B models.
"""

import sys
from pathlib import Path

import numpy as np
import pytest
import torch
import torch.nn as nn

_scripts = str(Path(__file__).resolve().parents[1] / "scripts")
if _scripts not in sys.path:
    sys.path.insert(0, _scripts)

import compare_utils as cu


class TinyVelocityNet(nn.Module):
    """Stand-in for the action model: (x_t, time) -> v_t."""

    def __init__(self, dim=7, hidden=32, seed=0):
        super().__init__()
        torch.manual_seed(seed)
        self.net = nn.Sequential(nn.Linear(dim + 1, hidden), nn.GELU(), nn.Linear(hidden, dim))

    def forward(self, x_t, time):
        t = time.view(-1, 1).expand(x_t.shape[0], 1)
        return self.net(torch.cat([x_t, t], dim=-1))


def _flow_loss(model, actions, noise, time):
    x_t = time.view(-1, 1) * noise + (1 - time.view(-1, 1)) * actions
    u_t = noise - actions
    v_t = model(x_t, time)
    return v_t, ((v_t - u_t) ** 2).mean(dim=-1)


@pytest.fixture
def batch():
    torch.manual_seed(123)
    actions = torch.randn(8, 7)
    noise = torch.randn(8, 7)
    time = torch.rand(8) * 0.999 + 0.001
    return actions, noise, time


def test_identical_models_pass(batch):
    actions, noise, time = batch
    jax_side = TinyVelocityNet(seed=0)
    pt_side = TinyVelocityNet(seed=0)  # identical weights
    pt_side.load_state_dict(jax_side.state_dict())

    vt_j, loss_j = _flow_loss(jax_side, actions, noise, time)
    vt_p, loss_p = _flow_loss(pt_side, actions, noise, time)

    d_vt = cu.max_abs_diff(vt_j.detach().numpy(), vt_p.detach().numpy())
    d_loss = cu.max_abs_diff(loss_j.detach().numpy(), loss_p.detach().numpy())
    assert d_vt < 1e-6
    assert d_loss < 1e-6


def test_injected_noise_time_shared(batch):
    """Same injected (noise,time) -> deterministic, reproducible outputs."""
    actions, noise, time = batch
    model = TinyVelocityNet(seed=1)
    vt1, _ = _flow_loss(model, actions, noise, time)
    vt2, _ = _flow_loss(model, actions, noise, time)
    np.testing.assert_allclose(vt1.detach().numpy(), vt2.detach().numpy())


def test_perturbed_model_fails(batch):
    actions, noise, time = batch
    jax_side = TinyVelocityNet(seed=0)
    pt_side = TinyVelocityNet(seed=0)
    pt_side.load_state_dict(jax_side.state_dict())
    # perturb one weight substantially
    with torch.no_grad():
        list(pt_side.parameters())[0].add_(1.0)

    vt_j, _ = _flow_loss(jax_side, actions, noise, time)
    vt_p, _ = _flow_loss(pt_side, actions, noise, time)
    d = cu.max_abs_diff(vt_j.detach().numpy(), vt_p.detach().numpy())
    entry = cu.compare_pointwise(vt_j.detach().numpy(), vt_p.detach().numpy(), cu.tol("v_t", "bf16"))
    assert d > cu.tol("v_t", "bf16")
    assert entry["pass"] is False


def test_bf16_tolerance(batch):
    """Small bf16-level perturbation stays within the bf16 tolerance."""
    actions, noise, time = batch
    jax_side = TinyVelocityNet(seed=0)
    pt_side = TinyVelocityNet(seed=0)
    pt_side.load_state_dict(jax_side.state_dict())
    with torch.no_grad():
        for p in pt_side.parameters():
            p.add_(torch.randn_like(p) * 1e-3)  # tiny noise

    vt_j, _ = _flow_loss(jax_side, actions, noise, time)
    vt_p, _ = _flow_loss(pt_side, actions, noise, time)
    entry = cu.compare_pointwise(vt_j.detach().numpy(), vt_p.detach().numpy(), cu.tol("v_t", "bf16"))
    assert entry["pass"] is True


def test_report_overall_logic(batch):
    actions, noise, time = batch
    m = TinyVelocityNet(seed=2)
    vt, loss = _flow_loss(m, actions, noise, time)
    entries = {
        "v_t": cu.compare_pointwise(vt.detach().numpy(), vt.detach().numpy(), cu.tol("v_t", "bf16")),
        "loss": cu.compare_pointwise(loss.detach().numpy(), loss.detach().numpy(), cu.tol("loss", "bf16")),
    }
    report = cu.make_report(entries)
    assert report["overall_pass"] is True
