"""FrankaAbsoluteJointCodec — 8D canonical <-> physical action conversion.

All training quantities (ref_chunk, actor output, BC target, critic action,
replay action) live in canonical [-1,1] domain. Only decoded to physical
domain right before env execution.

Stats source: Stage0 checkpoint stats.json -> franka_plug -> action.arm/action.gripper
"""
from __future__ import annotations

import json
import logging
from pathlib import Path

import torch
from torch import Tensor

logger = logging.getLogger(__name__)

DEFAULT_ARM_MIN = [-0.4862719178199768, -0.10739399492740631, -0.20248545706272125,
                   -2.216602325439453, -0.2730485796928406, 1.6490293741226196,
                   0.36953240633010864]
DEFAULT_ARM_MAX = [0.05982524901628494, 0.3328738212585449, 0.480135977268219,
                   -1.5293786525726318, 0.11044661700725555, 2.5172624588012695,
                   1.1021002531051636]
DEFAULT_GRIP_MIN = 0.0074404762126505375
DEFAULT_GRIP_MAX = 1.0


class FrankaAbsoluteJointCodec:
    """Bidirectional 8D codec: physical <-> canonical [-1,1]."""

    def __init__(
        self,
        stats_path: str | None = None,
        arm_min: list[float] | None = None,
        arm_max: list[float] | None = None,
        grip_min: float = DEFAULT_GRIP_MIN,
        grip_max: float = DEFAULT_GRIP_MAX,
    ):
        if stats_path is not None:
            with open(stats_path) as f:
                stats = json.load(f)
            s = stats["franka_plug"]
            arm_min = s["action.arm"]["min"]
            arm_max = s["action.arm"]["max"]
            grip_min = s["action.gripper"]["min"][0]
            grip_max = s["action.gripper"]["max"][0]

        self.arm_min = torch.tensor(arm_min or DEFAULT_ARM_MIN, dtype=torch.float32)
        self.arm_max = torch.tensor(arm_max or DEFAULT_ARM_MAX, dtype=torch.float32)
        self.grip_min = grip_min
        self.grip_max = grip_max

        arm_range = self.arm_max - self.arm_min
        if (arm_range <= 0).any():
            raise ValueError(f"arm_max <= arm_min for some joints: range={arm_range}")

    def encode_physical(self, action_8d: Tensor) -> Tensor:
        """physical 8D -> canonical [-1,1].

        Args:
            action_8d: [..., 8] -- 7D arm (radians) + 1D gripper [0,1]
        Returns:
            canonical: [..., 8] -- all in [-1, 1]
        """
        arm = action_8d[..., :7]
        grip = action_8d[..., 7:8]

        dev = action_8d.device
        lo = self.arm_min.to(dev)
        hi = self.arm_max.to(dev)

        arm_can = 2.0 * (arm - lo) / (hi - lo) - 1.0
        grip_can = 2.0 * (grip - self.grip_min) / (self.grip_max - self.grip_min) - 1.0

        return torch.cat([arm_can, grip_can], dim=-1)

    def decode_canonical(self, action_canonical: Tensor) -> Tensor:
        """canonical [-1,1] -> physical 8D.

        Args:
            action_canonical: [..., 8]
        Returns:
            physical: [..., 8] -- 7D arm (radians) + 1D gripper [0,1]
        """
        arm_can = action_canonical[..., :7]
        grip_can = action_canonical[..., 7:8]

        dev = action_canonical.device
        lo = self.arm_min.to(dev)
        hi = self.arm_max.to(dev)

        arm_phys = lo + (arm_can + 1.0) / 2.0 * (hi - lo)
        grip_phys = self.grip_min + (grip_can + 1.0) / 2.0 * (self.grip_max - self.grip_min)

        return torch.cat([arm_phys, grip_phys], dim=-1)

    def model_to_canonical(
        self,
        action_32d: Tensor,
        action_mean: Tensor,
        action_std: Tensor,
    ) -> tuple[Tensor, dict]:
        """model-normalized 32D -> canonical 8D.

        Args:
            action_32d: [B, H, 32] model-normalized
            action_mean: [32] or [8]
            action_std:  [32] or [8]
        Returns:
            canonical: [B, H, 8]
            metrics: {clip_rate, pad_max, ...}
        """
        assert action_32d.ndim == 3, f"Expected [B,H,32], got {action_32d.shape}"
        assert torch.isfinite(action_32d).all(), "Non-finite values in action_32d"

        pad = action_32d[..., 8:]
        pad_max = pad.abs().max().item()
        if pad_max > 5.0:
            logger.warning("Large pad values: max=%.3f", pad_max)

        action_8d_norm = action_32d[..., :8]

        mean = action_mean[:8].to(action_8d_norm.device)
        std = action_std[:8].to(action_8d_norm.device)
        action_phys = action_8d_norm * std + mean

        canonical = self.encode_physical(action_phys)

        clip_rate = ((canonical.abs() > 1.0).float().mean()).item()

        return canonical, {
            "clip_rate": clip_rate,
            "pad_max": pad_max,
            "canonical_min": canonical.min().item(),
            "canonical_max": canonical.max().item(),
        }
