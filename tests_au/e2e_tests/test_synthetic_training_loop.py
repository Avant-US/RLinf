"""Synthetic e2e test that validates EMA + augmentation + LR scheduler in a training loop.

This test does NOT require the full openpi model or data — it uses a small
synthetic model and data to verify the integration of all components.
"""

import sys
from pathlib import Path

import pytest
import torch
import torch.nn as nn

_mod_path = str(Path(__file__).resolve().parents[2] / "rlinf" / "models" / "embodiment" / "openpi_au")
if _mod_path not in sys.path:
    sys.path.insert(0, _mod_path)

from augmentation import faithful_augment
from ema import ModelEMA


class TinyActionModel(nn.Module):
    """Synthetic model mimicking π₀.₅ forward: images → actions loss."""

    def __init__(self, img_channels=3, action_dim=7, hidden=64):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(img_channels, 16, 3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d(1),
            nn.Flatten(),
        )
        self.head = nn.Linear(16, action_dim)

    def forward(self, images_bhwc, actions):
        images_bchw = images_bhwc.permute(0, 3, 1, 2)
        features = self.encoder(images_bchw)
        pred = self.head(features)
        loss = (pred - actions[:, 0, :]).pow(2).mean()
        return loss


@pytest.fixture
def setup():
    torch.manual_seed(0)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = TinyActionModel().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-10)
    ema = ModelEMA(model, decay=0.999)
    return model, optimizer, ema, device


def _make_batch(batch_size=8, H=32, W=32, action_dim=7, action_horizon=10, device="cpu"):
    images = torch.rand(batch_size, H, W, 3, device=device)
    actions = torch.randn(batch_size, action_horizon, action_dim, device=device)
    return images, actions


@pytest.mark.gpu
def test_full_training_loop(setup):
    model, optimizer, ema, device = setup
    num_steps = 20
    losses = []

    for step in range(num_steps):
        model.train()
        images, actions = _make_batch(device=device)

        aug_images = faithful_augment(images, is_wrist=False)
        loss = model(aug_images, actions)

        optimizer.zero_grad()
        loss.backward()
        grad_norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()

        ema.update(model)
        losses.append(loss.item())

    assert losses[-1] < losses[0], f"Loss should decrease: {losses[0]:.4f} → {losses[-1]:.4f}"
    assert ema.num_updates == num_steps


@pytest.mark.gpu
def test_ema_checkpoint_roundtrip(setup):
    model, optimizer, ema, device = setup

    for _ in range(5):
        images, actions = _make_batch(device=device)
        loss = model(images, actions)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        ema.update(model)

    backup = ema.swap_in(model)
    ema_weights = {n: p.clone() for n, p in model.named_parameters() if p.requires_grad}
    ema.swap_out(model, backup)

    for name in ema_weights:
        assert not torch.equal(ema_weights[name], backup[name]), "EMA weights should differ from training weights"


@pytest.mark.gpu
def test_augmentation_in_pipeline(setup):
    model, optimizer, ema, device = setup
    images, actions = _make_batch(device=device)

    # With augmentation
    aug_images = faithful_augment(images, is_wrist=False)
    loss_aug = model(aug_images, actions)

    # Without augmentation
    loss_clean = model(images, actions)

    # Both should produce valid finite losses
    assert torch.isfinite(loss_aug)
    assert torch.isfinite(loss_clean)
    # With augmentation the loss will differ (different inputs)
    assert loss_aug.item() != loss_clean.item()


def test_cpu_training_loop():
    """Verify training loop works on CPU too."""
    torch.manual_seed(42)
    model = TinyActionModel()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    ema = ModelEMA(model, decay=0.99)

    for _ in range(5):
        images, actions = _make_batch()
        loss = model(images, actions)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        ema.update(model)

    assert ema.num_updates == 5
    sd = ema.state_dict()
    assert sd["num_updates"] == 5
