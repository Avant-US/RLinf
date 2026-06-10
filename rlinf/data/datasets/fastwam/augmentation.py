"""Video data augmentation transforms for FastWAM SFT training.

All transforms accept [T, C, H, W] float32 tensors in [0, 1] range
and return tensors with the same dtype and value range.
Frame-temporal consistency is guaranteed by torchvision.transforms.v2
which applies the same random parameters to all frames in a 4D batch.
"""

import copy
import math
from typing import Union, List, Dict, Optional, Literal

import torch
import torch.nn as nn
import torchvision.transforms.v2 as T2
import torchvision.transforms.v2.functional as F2


class VideoAugmentation(nn.Module):
    """Base class for video augmentations.

    Args:
        p: Probability of applying the augmentation (0.0–1.0).
    """

    def __init__(self, p: float = 1.0):
        super().__init__()
        self.p = p

    def forward(self, video: torch.Tensor) -> torch.Tensor:
        assert video.ndim == 4, f"Expected [T, C, H, W], got shape {video.shape}"
        if torch.rand(1).item() > self.p:
            return video
        return self._apply(video)

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Color augmentations
# ---------------------------------------------------------------------------


class VideoColorJitter(VideoAugmentation):
    """Frame-consistent brightness/contrast/saturation/hue jitter.

    Args:
        brightness: Brightness jitter range.
        contrast: Contrast jitter range.
        saturation: Saturation jitter range.
        hue: Hue jitter range (recommend <= 0.1).
        p: Probability of applying.
    """

    def __init__(
        self,
        brightness: float = 0.0,
        contrast: float = 0.0,
        saturation: float = 0.0,
        hue: float = 0.0,
        p: float = 1.0,
    ):
        super().__init__(p=p)
        self._jitter = T2.ColorJitter(
            brightness=brightness,
            contrast=contrast,
            saturation=saturation,
            hue=hue,
        )

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        return self._jitter(video).clamp(0.0, 1.0)


class VideoRandomGrayscale(VideoAugmentation):
    """Convert entire video to grayscale with probability p."""

    def __init__(self, p: float = 0.1):
        super().__init__(p=p)
        self._gray = T2.Grayscale(num_output_channels=3)

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        return self._gray(video)


# ---------------------------------------------------------------------------
# Geometric augmentations
# ---------------------------------------------------------------------------


class VideoRandomCrop(VideoAugmentation):
    """Frame-consistent random crop.

    Crops to ``int(H * scale) x int(W * scale)``. A subsequent ``Resize``
    in the transform chain restores the target dimensions.

    Args:
        scale: Fraction of height/width to keep (0, 1].
        p: Probability of applying.
    """

    def __init__(self, scale: float = 0.95, p: float = 0.5):
        super().__init__(p=p)
        self.scale = scale

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        _, _, H, W = video.shape
        crop_h, crop_w = int(H * self.scale), int(W * self.scale)
        return T2.RandomCrop(size=(crop_h, crop_w))(video)


class VideoRandomResizedCrop(VideoAugmentation):
    """Frame-consistent random resized crop.

    Args:
        size: Output size (H, W).
        scale: Range of crop area ratio.
        ratio: Range of aspect ratio.
        p: Probability of applying.
    """

    def __init__(
        self,
        size=(224, 224),
        scale=(0.8, 1.0),
        ratio=(0.75, 1.333),
        p: float = 0.5,
    ):
        super().__init__(p=p)
        self._crop = T2.RandomResizedCrop(
            size=size,
            scale=scale,
            ratio=ratio,
            interpolation=T2.InterpolationMode.BILINEAR,
            antialias=True,
        )

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        return self._crop(video)


class VideoRandomHorizontalFlip(VideoAugmentation):
    """Frame-consistent horizontal flip.

    WARNING: For bimanual robots (e.g. R1 Pro) this swaps left/right arm
    semantics without mirroring the action space. Only use for single-arm
    robots or verified symmetric tasks.

    Args:
        p: Probability of flipping.
    """

    def __init__(self, p: float = 0.5):
        super().__init__(p=p)

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        return T2.RandomHorizontalFlip(p=1.0)(video)


class VideoRandomRotation(VideoAugmentation):
    """Frame-consistent small-angle rotation.

    Args:
        degrees: Rotation range [-degrees, +degrees].
        p: Probability of applying.
    """

    def __init__(self, degrees: float = 5.0, p: float = 0.3):
        super().__init__(p=p)
        self._rot = T2.RandomRotation(
            degrees=degrees,
            interpolation=T2.InterpolationMode.BILINEAR,
        )

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        return self._rot(video)


class VideoRandomFisheye(VideoAugmentation):
    """Frame-consistent convex bulging fisheye geometric distortion.

    Simulates a wide-angle dome-like lens by warping each frame with the same
    randomly sampled strength and optical center via
    ``torchvision.transforms.v2.functional.elastic_image``. All frames in
    ``[T, C, H, W]`` share identical warp parameters.

    Uses Euclidean distance normalized by the farthest corner so that ``k``
    has intuitive strength semantics (k=0.3 → 30% compression at the farthest
    corner). Iso-distance contours are circles, producing visible curvature
    starting from the very first pixel row/column.

    Args:
        k_range: Range ``(k_min, k_max)`` for fisheye strength, each in
            ``(0, 1)``. Typical mild-to-strong: ``(0.20, 0.45)``.
        center_jitter: Max absolute shift of the distortion center in
            normalized coordinates ``[-1, 1]``. ``0`` keeps the center fixed.
        p: Probability of applying.
    """

    def __init__(
        self,
        k_range: tuple[float, float] = (0.20, 0.45),
        center_jitter: float = 0.02,
        p: float = 0.5,
    ):
        super().__init__(p=p)
        if len(k_range) != 2 or k_range[0] > k_range[1]:
            raise ValueError(f"`k_range` must be (min, max) with min <= max, got {k_range}.")
        self.k_range = k_range
        self.center_jitter = center_jitter

    def _sample_params(self, device: torch.device, dtype: torch.dtype) -> tuple[float, float, float]:
        k_min, k_max = self.k_range
        k = torch.empty((), device=device, dtype=dtype).uniform_(k_min, k_max).item()
        if self.center_jitter > 0.0:
            center_x = torch.empty((), device=device, dtype=dtype).uniform_(
                -self.center_jitter, self.center_jitter
            ).item()
            center_y = torch.empty((), device=device, dtype=dtype).uniform_(
                -self.center_jitter, self.center_jitter
            ).item()
        else:
            center_x, center_y = 0.0, 0.0
        return k, center_x, center_y

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        _, _, height, width = video.shape
        k, center_x, center_y = self._sample_params(video.device, video.dtype)
        ys = torch.linspace(-1.0, 1.0, height, device=video.device, dtype=video.dtype)
        xs = torch.linspace(-1.0, 1.0, width, device=video.device, dtype=video.dtype)
        grid_y, grid_x = torch.meshgrid(ys, xs, indexing="ij")
        x = grid_x - center_x
        y = grid_y - center_y
        r_sq = x * x + y * y
        r_sq_max = (1.0 + abs(center_x)) ** 2 + (1.0 + abs(center_y)) ** 2
        radial_scale = (1.0 - k * r_sq / r_sq_max).clamp(min=0.05)
        disp = torch.stack([x * (radial_scale - 1), y * (radial_scale - 1)], dim=-1).unsqueeze(0)
        return F2.elastic_image(video, disp).clamp(0.0, 1.0)


class VideoRandomHorizontalLine(VideoAugmentation):
    """Draw one horizontal line at the same row on every frame.

    Simulates a fixed sensor artifact, cable, or overlay glitch. The vertical
    position (and optional color) is sampled once per ``[T, C, H, W]`` clip so
    all frames stay aligned in time.

    Args:
        line_width: Line thickness in pixels (centered on the sampled row).
        color: Optional RGB fill in ``[0, 1]``. ``None`` samples a random
            color per application.
        row_range: Fractional vertical range ``(low, high)`` in ``[0, 1]`` used
            to sample the line center. ``(0.0, 1.0)`` allows any row.
        p: Probability of applying.
    """

    def __init__(
        self,
        line_width: int = 2,
        color: tuple[float, float, float] | None = None,
        row_range: tuple[float, float] = (0.0, 1.0),
        p: float = 0.3,
    ):
        super().__init__(p=p)
        if line_width < 1:
            raise ValueError(f"`line_width` must be >= 1, got {line_width}.")
        if len(row_range) != 2 or not (0.0 <= row_range[0] <= row_range[1] <= 1.0):
            raise ValueError(
                f"`row_range` must be (low, high) within [0, 1], got {row_range}."
            )
        self.line_width = line_width
        self.color = color
        self.row_range = row_range

    def _sample_row(self, height: int, device: torch.device) -> int:
        low, high = self.row_range
        if low == high:
            frac = low
        else:
            frac = torch.empty((), device=device).uniform_(low, high).item()
        center = int(round(frac * (height - 1)))
        return max(0, min(height - 1, center))

    def _sample_color(self, channels: int, device: torch.device, dtype: torch.dtype) -> torch.Tensor:
        if self.color is not None:
            if len(self.color) != channels:
                raise ValueError(f"`color` length must be {channels}, got {len(self.color)}.")
            return torch.tensor(self.color, device=device, dtype=dtype)
        return torch.rand(channels, device=device, dtype=dtype)

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        _, channels, height, _ = video.shape
        row = self._sample_row(height, video.device)
        half = self.line_width // 2
        row_start = max(0, row - half)
        row_end = min(height, row + half + (self.line_width % 2))

        out = video.clone()
        color = self._sample_color(channels, video.device, video.dtype).view(1, channels, 1, 1)
        out[:, :, row_start:row_end, :] = color
        return out.clamp(0.0, 1.0)


class VideoRandomHorizontalLinePerFrame(VideoAugmentation):
    """Per-frame random horizontal line segments (independent across time).

    Unlike :class:`VideoRandomHorizontalLine`, each frame may have no line, a
    different row, thickness, length, and color. Useful for flickering overlay /
    dropout-like artifacts rather than a fixed sensor stripe.

    Args:
        p_line: Per-frame probability of drawing a line (else that frame is unchanged).
        line_width_range: Inclusive thickness range in pixels ``(min, max)``.
        length_frac_range: Segment length as a fraction of image width ``(min, max)``.
        row_range: Vertical center sampling range as fraction of height ``(low, high)``.
        color: Optional RGB in ``[0, 1]``. ``None`` samples a random color per line.
        p: Probability of applying this augmentation to the whole clip.
    """

    def __init__(
        self,
        p_line: float = 0.7,
        line_width_range: tuple[int, int] = (1, 4),
        length_frac_range: tuple[float, float] = (0.3, 1.0),
        row_range: tuple[float, float] = (0.0, 1.0),
        color: tuple[float, float, float] | None = (0.0, 0.0, 0.0),
        p: float = 0.3,
    ):
        super().__init__(p=p)
        if not (0.0 <= p_line <= 1.0):
            raise ValueError(f"`p_line` must be in [0, 1], got {p_line}.")
        w_min, w_max = line_width_range
        if w_min < 1 or w_max < w_min:
            raise ValueError(
                f"`line_width_range` must be (min>=1, max>=min), got {line_width_range}."
            )
        l_min, l_max = length_frac_range
        if not (0.0 < l_min <= l_max <= 1.0):
            raise ValueError(
                f"`length_frac_range` must be in (0, 1], got {length_frac_range}."
            )
        if len(row_range) != 2 or not (0.0 <= row_range[0] <= row_range[1] <= 1.0):
            raise ValueError(
                f"`row_range` must be (low, high) within [0, 1], got {row_range}."
            )
        self.p_line = p_line
        self.line_width_range = line_width_range
        self.length_frac_range = length_frac_range
        self.row_range = row_range
        self.color = color

    def _sample_row_band(
        self, height: int, line_width: int, device: torch.device
    ) -> tuple[int, int]:
        low, high = self.row_range
        if low == high:
            frac = low
        else:
            frac = torch.empty((), device=device).uniform_(low, high).item()
        center = max(0, min(height - 1, int(round(frac * (height - 1)))))
        half = line_width // 2
        row_start = max(0, center - half)
        row_end = min(height, center + half + (line_width % 2))
        return row_start, row_end

    def _sample_x_span(self, width: int, device: torch.device) -> tuple[int, int]:
        l_min, l_max = self.length_frac_range
        length_frac = torch.empty((), device=device).uniform_(l_min, l_max).item()
        seg_len = max(1, min(width, int(round(length_frac * width))))
        if seg_len >= width:
            return 0, width
        x0 = int(torch.randint(0, width - seg_len + 1, (1,), device=device).item())
        return x0, x0 + seg_len

    def _sample_line_width(self, device: torch.device) -> int:
        w_min, w_max = self.line_width_range
        return int(torch.randint(w_min, w_max + 1, (1,), device=device).item())

    def _sample_fill(
        self, channels: int, device: torch.device, dtype: torch.dtype
    ) -> torch.Tensor:
        if self.color is not None:
            if len(self.color) != channels:
                raise ValueError(f"`color` length must be {channels}, got {len(self.color)}.")
            return torch.tensor(self.color, device=device, dtype=dtype)
        return torch.rand(channels, device=device, dtype=dtype)

    def _apply_frame(self, frame: torch.Tensor) -> torch.Tensor:
        """Draw on a single frame ``[C, H, W]``."""
        if torch.rand((), device=frame.device).item() > self.p_line:
            return frame

        channels, height, width = frame.shape
        line_width = self._sample_line_width(frame.device)
        row_start, row_end = self._sample_row_band(height, line_width, frame.device)
        x0, x1 = self._sample_x_span(width, frame.device)
        fill = self._sample_fill(channels, frame.device, frame.dtype).view(channels, 1, 1)

        out = frame.clone()
        out[:, row_start:row_end, x0:x1] = fill
        return out

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        out = torch.stack([self._apply_frame(video[t]) for t in range(video.shape[0])], dim=0)
        return out.clamp(0.0, 1.0)


# ---------------------------------------------------------------------------
# Noise augmentations
# ---------------------------------------------------------------------------


class VideoGaussianNoise(VideoAugmentation):
    """Add Gaussian noise to video.

    By default the same noise pattern is applied to all frames (simulating
    fixed-pattern sensor noise). Set ``per_frame=True`` for independent
    per-frame noise (simulating random read noise).

    Args:
        std: Noise standard deviation relative to [0, 1] range.
        per_frame: Sample independent noise per frame.
        p: Probability of applying.
    """

    def __init__(self, std: float = 0.02, per_frame: bool = False, p: float = 0.3):
        super().__init__(p=p)
        self.std = std
        self.per_frame = per_frame

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        if self.per_frame:
            noise = torch.randn_like(video) * self.std
        else:
            noise = torch.randn_like(video[0:1]) * self.std
            noise = noise.expand_as(video)
        return (video + noise).clamp(0.0, 1.0)


class VideoRandomErasing(VideoAugmentation):
    """Frame-consistent random rectangular erasing.

    Args:
        scale: Range of erased area ratio.
        ratio: Range of erased area aspect ratio.
        value: Fill value (0 = black, ``"random"`` = random pixels).
        p: Probability of applying.
    """

    def __init__(
        self,
        scale=(0.02, 0.15),
        ratio=(0.3, 3.3),
        value: float = 0,
        p: float = 0.3,
    ):
        super().__init__(p=p)
        self._erase = T2.RandomErasing(
            p=1.0,
            scale=scale,
            ratio=ratio,
            value=value,
        )

    def _apply(self, video: torch.Tensor) -> torch.Tensor:
        return self._erase(video)


# ---------------------------------------------------------------------------
# Presets
# ---------------------------------------------------------------------------


class AugmentationPreset:
    """Pre-defined augmentation preset combinations."""

    PRESETS = {
        "none": [],
        "light": [
            VideoColorJitter(brightness=0.1, contrast=0.1, saturation=0.1, hue=0.03, p=0.5),
        ],
        "medium": [
            VideoRandomCrop(scale=0.95, p=0.5),
            VideoColorJitter(brightness=0.2, contrast=0.3, saturation=0.3, hue=0.05, p=0.8),
            VideoGaussianNoise(std=0.01, p=0.2),
        ],
        "strong": [
            VideoRandomCrop(scale=0.90, p=0.7),
            VideoColorJitter(brightness=0.3, contrast=0.4, saturation=0.5, hue=0.08, p=0.9),
            VideoRandomGrayscale(p=0.1),
            VideoGaussianNoise(std=0.02, p=0.3),
            VideoRandomErasing(p=0.2),
        ],
        "dreamzero": [
            VideoRandomCrop(scale=0.95, p=0.5),
            VideoColorJitter(brightness=0.3, contrast=0.4, saturation=0.5, hue=0.08, p=1.0),
        ],
    }

    @classmethod
    def get(cls, name: str) -> list:
        if name not in cls.PRESETS:
            raise ValueError(
                f"Unknown augmentation preset '{name}'. "
                f"Available: {list(cls.PRESETS.keys())}"
            )
        return [copy.deepcopy(t) for t in cls.PRESETS[name]]


# ---------------------------------------------------------------------------
# Proprioceptive augmentations (action / state domain)
# ---------------------------------------------------------------------------

FrameIndices = Union[str, List[int], Dict[str, int], Dict[str, float]]
AugmentTarget = Literal["both", "state", "action"]


class ProprioAugmentation:
    """Base class for proprioceptive (action/state) augmentations.

    Operates on the full batch dict:
        batch["action"][key]: [action_horizon, action_dim]
        batch["state"][key]:  [num_obs_steps, state_dim]
    """

    def __init__(self, p: float = 0.5,
                 exclude_dims: Optional[List[int]] = None,
                 frame_indices: FrameIndices = "all",
                 respect_pad: bool = True,
                 augment_target: AugmentTarget = "both"):
        self.p = p
        self.exclude_dims = set(exclude_dims) if exclude_dims else set()
        self.frame_indices = frame_indices
        self.respect_pad = respect_pad
        self.augment_target = augment_target

    def __call__(self, batch: dict) -> dict:
        if torch.rand(1).item() > self.p:
            return batch
        return self._apply(batch)

    def _apply(self, batch: dict) -> dict:
        raise NotImplementedError

    def _make_mask(self, ndim: int, device) -> torch.Tensor:
        mask = torch.ones(ndim, dtype=torch.bool, device=device)
        for d in self.exclude_dims:
            if 0 <= d < ndim:
                mask[d] = False
        return mask

    def _resolve_frame_indices(self, num_frames: int,
                               is_pad: Optional[torch.Tensor] = None
                               ) -> torch.Tensor:
        mask = torch.zeros(num_frames, dtype=torch.bool)

        if isinstance(self.frame_indices, str) and self.frame_indices == "all":
            mask[:] = True
        elif isinstance(self.frame_indices, list):
            for idx in self.frame_indices:
                if 0 <= idx < num_frames:
                    mask[idx] = True
        elif isinstance(self.frame_indices, dict):
            if "first_n" in self.frame_indices:
                n = min(self.frame_indices["first_n"], num_frames)
                mask[:n] = True
            elif "last_n" in self.frame_indices:
                n = min(self.frame_indices["last_n"], num_frames)
                mask[-n:] = True
            elif "random_n" in self.frame_indices or "random_frac" in self.frame_indices:
                if self.respect_pad and is_pad is not None:
                    valid_indices = torch.where(~is_pad)[0]
                else:
                    valid_indices = torch.arange(num_frames)

                num_valid = valid_indices.shape[0]
                if num_valid == 0:
                    return mask

                if "random_n" in self.frame_indices:
                    k = min(self.frame_indices["random_n"], num_valid)
                else:
                    k = min(math.ceil(self.frame_indices["random_frac"] * num_valid),
                            num_valid)

                perm = torch.randperm(num_valid)[:k]
                selected = valid_indices[perm]
                mask[selected] = True
                return mask

        if self.respect_pad and is_pad is not None:
            mask = mask & ~is_pad

        return mask


class ProprioRandomOffset(ProprioAugmentation):
    """Per-sample random joint offset to simulate calibration drift."""

    def __init__(self, offset_range: float = 0.02,
                 exclude_dims: Optional[List[int]] = None,
                 frame_indices: FrameIndices = "all",
                 respect_pad: bool = True,
                 augment_target: AugmentTarget = "both",
                 p: float = 0.5):
        super().__init__(p=p, exclude_dims=exclude_dims,
                         frame_indices=frame_indices,
                         respect_pad=respect_pad,
                         augment_target=augment_target)
        self.offset_range = offset_range

    def _apply(self, batch: dict) -> dict:
        state_is_pad = batch.get("state_is_pad", None)
        action_is_pad = batch.get("action_is_pad", None)

        for key in list(batch.get("action", {}).keys()):
            action = batch["action"][key]
            state = batch["state"][key]
            ndim = action.shape[-1]
            dim_mask = self._make_mask(ndim, action.device)

            offset = torch.zeros(ndim, device=action.device, dtype=action.dtype)
            offset[dim_mask] = torch.empty(
                dim_mask.sum().item(), device=action.device, dtype=action.dtype
            ).uniform_(-self.offset_range, self.offset_range)

            if self.augment_target in ("both", "state"):
                state_frame_mask = self._resolve_frame_indices(
                    state.shape[0], state_is_pad)
                state_offset = offset.unsqueeze(0) * state_frame_mask.unsqueeze(1).to(
                    dtype=action.dtype, device=action.device)
                batch["state"][key] = state + state_offset

            if self.augment_target in ("both", "action"):
                action_frame_mask = self._resolve_frame_indices(
                    action.shape[0], action_is_pad)
                action_offset = offset.unsqueeze(0) * action_frame_mask.unsqueeze(1).to(
                    dtype=action.dtype, device=action.device)
                batch["action"][key] = action + action_offset

        return batch


class ProprioRandomScale(ProprioAugmentation):
    """Per-sample random gain to simulate actuator variation. Only scales action."""

    def __init__(self, scale_range: float = 0.05,
                 exclude_dims: Optional[List[int]] = None,
                 frame_indices: FrameIndices = "all",
                 respect_pad: bool = True,
                 augment_target: AugmentTarget = "both",
                 p: float = 0.3):
        super().__init__(p=p, exclude_dims=exclude_dims,
                         frame_indices=frame_indices,
                         respect_pad=respect_pad,
                         augment_target=augment_target)
        self.scale_range = scale_range

    def _apply(self, batch: dict) -> dict:
        action_is_pad = batch.get("action_is_pad", None)

        for key in list(batch.get("action", {}).keys()):
            action = batch["action"][key]
            ndim = action.shape[-1]
            dim_mask = self._make_mask(ndim, action.device)

            scale = torch.ones(ndim, device=action.device, dtype=action.dtype)
            scale[dim_mask] = 1.0 + torch.empty(
                dim_mask.sum().item(), device=action.device, dtype=action.dtype
            ).uniform_(-self.scale_range, self.scale_range)

            frame_mask = self._resolve_frame_indices(
                action.shape[0], action_is_pad)

            frame_w = frame_mask.unsqueeze(1).to(
                dtype=action.dtype, device=action.device)
            effective_scale = scale.unsqueeze(0) * frame_w + (1.0 - frame_w)

            batch["action"][key] = action * effective_scale
        return batch


class ProprioRandomDeadzone(ProprioAugmentation):
    """Per-sample random deadzone to simulate gear backlash. Only affects action."""

    def __init__(self, deadzone_max: float = 0.005,
                 exclude_dims: Optional[List[int]] = None,
                 frame_indices: FrameIndices = "all",
                 respect_pad: bool = True,
                 augment_target: AugmentTarget = "both",
                 p: float = 0.2):
        super().__init__(p=p, exclude_dims=exclude_dims,
                         frame_indices=frame_indices,
                         respect_pad=respect_pad,
                         augment_target=augment_target)
        self.deadzone_max = deadzone_max

    def _apply(self, batch: dict) -> dict:
        action_is_pad = batch.get("action_is_pad", None)

        for key in list(batch.get("action", {}).keys()):
            action = batch["action"][key]
            ndim = action.shape[-1]
            dim_mask = self._make_mask(ndim, action.device)

            thresh = torch.zeros(ndim, device=action.device, dtype=action.dtype)
            thresh[dim_mask] = torch.empty(
                dim_mask.sum().item(), device=action.device, dtype=action.dtype
            ).uniform_(0, self.deadzone_max)

            frame_mask = self._resolve_frame_indices(
                action.shape[0], action_is_pad)

            dead = action.abs() < thresh.unsqueeze(0)
            frame_w = frame_mask.unsqueeze(1).to(
                dtype=torch.bool, device=action.device)
            dead = dead & frame_w

            batch["action"][key] = action.masked_fill(dead, 0.0)
        return batch


class ProprioRandomNoise(ProprioAugmentation):
    """Per-frame independent random noise on selected frames."""

    def __init__(self, noise_std: float = 0.01,
                 exclude_dims: Optional[List[int]] = None,
                 frame_indices: FrameIndices = "all",
                 respect_pad: bool = True,
                 augment_target: AugmentTarget = "both",
                 p: float = 0.5):
        super().__init__(p=p, exclude_dims=exclude_dims,
                         frame_indices=frame_indices,
                         respect_pad=respect_pad,
                         augment_target=augment_target)
        self.noise_std = noise_std

    def _apply(self, batch: dict) -> dict:
        state_is_pad = batch.get("state_is_pad", None)
        action_is_pad = batch.get("action_is_pad", None)

        for key in list(batch.get("action", {}).keys()):
            action = batch["action"][key]
            state = batch["state"][key]
            ndim = action.shape[-1]
            dim_mask = self._make_mask(ndim, action.device)
            dim_indices = dim_mask.nonzero(as_tuple=True)[0]

            if self.augment_target in ("both", "state"):
                frame_mask = self._resolve_frame_indices(
                    state.shape[0], state_is_pad)
                noise = torch.zeros_like(state)
                selected = frame_mask.nonzero(as_tuple=True)[0]
                if selected.numel() > 0:
                    n_sel, n_dim = selected.shape[0], dim_indices.shape[0]
                    noise_vals = torch.randn(
                        n_sel, n_dim, device=state.device, dtype=state.dtype
                    ) * self.noise_std
                    noise[selected.unsqueeze(1), dim_indices.unsqueeze(0)] = noise_vals
                batch["state"][key] = state + noise

            if self.augment_target in ("both", "action"):
                frame_mask = self._resolve_frame_indices(
                    action.shape[0], action_is_pad)
                noise = torch.zeros_like(action)
                selected = frame_mask.nonzero(as_tuple=True)[0]
                if selected.numel() > 0:
                    n_sel, n_dim = selected.shape[0], dim_indices.shape[0]
                    noise_vals = torch.randn(
                        n_sel, n_dim, device=action.device, dtype=action.dtype
                    ) * self.noise_std
                    noise[selected.unsqueeze(1), dim_indices.unsqueeze(0)] = noise_vals
                batch["action"][key] = action + noise

        return batch
