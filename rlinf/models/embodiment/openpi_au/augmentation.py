"""Faithful image augmentation matching openpi JAX augmax semantics.

Key difference from RLinf baseline: per-sample randomness + luminance-based saturation.
"""

from __future__ import annotations

import torch
import torch.nn.functional as F


def faithful_augment(
    images: torch.Tensor,
    is_wrist: bool = False,
    brightness: float = 0.3,
    contrast: float = 0.4,
    saturation: float = 0.5,
    crop_scale: float = 0.95,
    rotation_degrees: float = 5.0,
    generator: torch.Generator | None = None,
) -> torch.Tensor:
    """Apply per-sample augmentation faithful to openpi JAX augmax.

    Args:
        images: [B, H, W, C] float tensor in [0, 1].
        is_wrist: If True, skip geometric augmentations (crop + rotation).
        brightness/contrast/saturation: Half-range of the jitter factor.
        crop_scale: Fraction of image to keep (0.95 -> 95%).
        rotation_degrees: Max rotation angle in degrees.
        generator: Optional torch.Generator for reproducibility.

    Returns:
        Augmented images [B, H, W, C] in [0, 1].
    """
    if not is_wrist:
        images = _per_sample_crop_resize(images, crop_scale, generator)
        images = _per_sample_rotate(images, rotation_degrees, generator)

    images = _per_sample_brightness(images, brightness, generator)
    images = _per_sample_contrast(images, contrast, generator)
    images = _per_sample_saturation_luminance(images, saturation, generator)

    return images.clamp(0.0, 1.0)


def _rand_uniform(
    low: float, high: float, shape: tuple, device: torch.device, generator: torch.Generator | None
) -> torch.Tensor:
    """Sample uniform [low, high) with given generator."""
    r = torch.rand(shape, device=device, generator=generator)
    return low + (high - low) * r


def _per_sample_crop_resize(
    images: torch.Tensor, crop_scale: float, generator: torch.Generator | None
) -> torch.Tensor:
    """Per-sample random crop + bilinear resize back to original size."""
    B, H, W, C = images.shape
    crop_h = int(H * crop_scale)
    crop_w = int(W * crop_scale)
    max_h = H - crop_h
    max_w = W - crop_w
    if max_h <= 0 or max_w <= 0:
        return images

    h_offsets = torch.randint(0, max_h + 1, (B,), device=images.device, generator=generator)
    w_offsets = torch.randint(0, max_w + 1, (B,), device=images.device, generator=generator)

    images_bchw = images.permute(0, 3, 1, 2)  # [B, C, H, W]

    y_starts = 2.0 * h_offsets.float() / H - 1.0
    y_ends = 2.0 * (h_offsets.float() + crop_h) / H - 1.0
    x_starts = 2.0 * w_offsets.float() / W - 1.0
    x_ends = 2.0 * (w_offsets.float() + crop_w) / W - 1.0

    gy = torch.linspace(0, 1, H, device=images.device).view(1, H, 1).expand(B, H, W)
    gx = torch.linspace(0, 1, W, device=images.device).view(1, 1, W).expand(B, H, W)

    grid_y = y_starts.view(B, 1, 1) + gy * (y_ends - y_starts).view(B, 1, 1)
    grid_x = x_starts.view(B, 1, 1) + gx * (x_ends - x_starts).view(B, 1, 1)
    grid = torch.stack([grid_x, grid_y], dim=-1)  # [B, H, W, 2]

    out = F.grid_sample(images_bchw, grid, mode="bilinear", padding_mode="zeros", align_corners=False)
    return out.permute(0, 2, 3, 1)  # [B, H, W, C]


def _per_sample_rotate(
    images: torch.Tensor, max_deg: float, generator: torch.Generator | None
) -> torch.Tensor:
    """Per-sample random rotation via affine grid."""
    B, H, W, C = images.shape
    device = images.device

    angles = _rand_uniform(-max_deg, max_deg, (B,), device, generator)
    angles_rad = angles * (torch.pi / 180.0)

    cos_a = torch.cos(angles_rad)  # [B]
    sin_a = torch.sin(angles_rad)  # [B]

    gy = torch.linspace(-1, 1, H, device=device).view(1, H, 1).expand(B, H, W)
    gx = torch.linspace(-1, 1, W, device=device).view(1, 1, W).expand(B, H, W)

    cos_a = cos_a.view(B, 1, 1)
    sin_a = sin_a.view(B, 1, 1)
    grid_x = gx * cos_a - gy * sin_a
    grid_y = gx * sin_a + gy * cos_a
    grid = torch.stack([grid_x, grid_y], dim=-1)

    images_bchw = images.permute(0, 3, 1, 2)
    out = F.grid_sample(images_bchw, grid, mode="bilinear", padding_mode="zeros", align_corners=False)
    return out.permute(0, 2, 3, 1)


def _per_sample_brightness(
    images: torch.Tensor, half_range: float, generator: torch.Generator | None
) -> torch.Tensor:
    """Per-sample brightness: image * factor, factor ~ U[1 - half_range, 1 + half_range]."""
    B = images.shape[0]
    factors = _rand_uniform(1.0 - half_range, 1.0 + half_range, (B, 1, 1, 1), images.device, generator)
    return images * factors


def _per_sample_contrast(
    images: torch.Tensor, half_range: float, generator: torch.Generator | None
) -> torch.Tensor:
    """Per-sample contrast: mean + (image - mean) * factor."""
    B = images.shape[0]
    factors = _rand_uniform(1.0 - half_range, 1.0 + half_range, (B, 1, 1, 1), images.device, generator)
    mean = images.mean(dim=(1, 2, 3), keepdim=True)
    return mean + (images - mean) * factors


def _per_sample_saturation_luminance(
    images: torch.Tensor, half_range: float, generator: torch.Generator | None
) -> torch.Tensor:
    """Per-sample saturation using BT.709 luminance (not RGB mean).

    luminance = 0.2126*R + 0.7152*G + 0.0722*B
    out = luminance + (image - luminance) * factor
    """
    B = images.shape[0]
    factors = _rand_uniform(1.0 - half_range, 1.0 + half_range, (B, 1, 1, 1), images.device, generator)
    lum_weights = torch.tensor([0.2126, 0.7152, 0.0722], device=images.device, dtype=images.dtype)
    luminance = (images * lum_weights.view(1, 1, 1, 3)).sum(dim=-1, keepdim=True)
    return luminance + (images - luminance) * factors
