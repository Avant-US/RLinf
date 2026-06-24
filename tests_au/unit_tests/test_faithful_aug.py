"""Unit tests for faithful_augment (openpi_au/augmentation.py)."""

import sys
from pathlib import Path

import pytest
import torch

_mod_path = str(Path(__file__).resolve().parents[2] / "rlinf" / "models" / "embodiment" / "openpi_au")
if _mod_path not in sys.path:
    sys.path.insert(0, _mod_path)

from augmentation import (
    _per_sample_brightness,
    _per_sample_contrast,
    _per_sample_crop_resize,
    _per_sample_rotate,
    _per_sample_saturation_luminance,
    faithful_augment,
)


@pytest.fixture
def sample_batch():
    torch.manual_seed(0)
    return torch.rand(4, 64, 64, 3)


@pytest.fixture
def generator():
    g = torch.Generator()
    g.manual_seed(12345)
    return g


def test_output_shape_and_range(sample_batch, generator):
    out = faithful_augment(sample_batch, is_wrist=False, generator=generator)
    assert out.shape == sample_batch.shape
    assert out.min() >= 0.0
    assert out.max() <= 1.0


def test_per_sample_independence(sample_batch, generator):
    uniform = sample_batch[0:1].expand(4, -1, -1, -1).clone()
    out = faithful_augment(uniform, is_wrist=False, generator=generator)
    diffs = []
    for i in range(1, 4):
        diffs.append((out[0] - out[i]).abs().mean().item())
    assert max(diffs) > 0.01, f"All samples identical: diffs={diffs}"


def test_wrist_no_geometry():
    torch.manual_seed(99)
    # Use a richer image so geometry differences are visible
    img = torch.rand(4, 64, 64, 3)

    g1 = torch.Generator().manual_seed(42)
    g2 = torch.Generator().manual_seed(42)

    out_wrist = faithful_augment(img.clone(), is_wrist=True, generator=g1)
    out_ext = faithful_augment(img.clone(), is_wrist=False, generator=g2)

    # External applies crop+rotation on top of same color jitter → must differ
    diff = (out_wrist - out_ext).abs().mean().item()
    assert diff > 0.001, f"Wrist and external should differ (geometry); got diff={diff}"


def test_deterministic_with_generator(sample_batch):
    g1 = torch.Generator().manual_seed(42)
    out1 = faithful_augment(sample_batch.clone(), is_wrist=False, generator=g1)
    g2 = torch.Generator().manual_seed(42)
    out2 = faithful_augment(sample_batch.clone(), is_wrist=False, generator=g2)
    torch.testing.assert_close(out1, out2)


def test_brightness_range():
    imgs = torch.ones(1000, 4, 4, 3) * 0.5
    g = torch.Generator().manual_seed(0)
    out = _per_sample_brightness(imgs, half_range=0.3, generator=g)
    assert out.min() >= 0.5 * 0.7 - 0.001
    assert out.max() <= 0.5 * 1.3 + 0.001


def test_contrast_no_nan():
    imgs = torch.rand(1000, 4, 4, 3)
    g = torch.Generator().manual_seed(0)
    out = _per_sample_contrast(imgs, half_range=0.4, generator=g)
    assert not torch.isnan(out).any()
    assert not torch.isinf(out).any()


def test_saturation_grayscale_invariant():
    gray = torch.ones(4, 8, 8, 3) * 0.5
    g = torch.Generator().manual_seed(0)
    out = _per_sample_saturation_luminance(gray, half_range=0.5, generator=g)
    torch.testing.assert_close(out, gray, atol=1e-5, rtol=1e-5)


def test_disabled_is_identity(sample_batch):
    out = faithful_augment(
        sample_batch.clone(),
        is_wrist=True,
        brightness=0.0,
        contrast=0.0,
        saturation=0.0,
    )
    torch.testing.assert_close(out, sample_batch, atol=1e-5, rtol=1e-5)


def test_crop_changes_image(sample_batch, generator):
    out = _per_sample_crop_resize(sample_batch, crop_scale=0.8, generator=generator)
    assert out.shape == sample_batch.shape
    diff = (out - sample_batch).abs().mean()
    assert diff > 0.01


@pytest.mark.gpu
def test_gpu_smoke():
    imgs = torch.rand(8, 128, 128, 3, device="cuda")
    out = faithful_augment(imgs, is_wrist=False)
    assert out.device.type == "cuda"
    assert out.shape == imgs.shape
    assert out.min() >= 0.0
    assert out.max() <= 1.0
