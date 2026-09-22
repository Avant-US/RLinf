"""Online distribution monitor: compares live state/action against training stats."""
from __future__ import annotations

import json
import logging
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

CHANNEL_NAMES = ["q1", "q2", "q3", "q4", "q5", "q6", "q7", "grip"]


class DistributionMonitor:
    """Compare per-step 8D state/action against training [q01, q99] range."""

    def __init__(self, stats_path: str) -> None:
        raw = json.loads(Path(stats_path).read_text())

        # Compose 8D vectors by concatenating arm (7D) + gripper (1D).
        self._state_mean = np.array(raw["observation.state.arm"]["mean"] + raw["observation.state.gripper"]["mean"])
        self._state_std = np.array(raw["observation.state.arm"]["std"] + raw["observation.state.gripper"]["std"])
        self._state_q01 = np.array(raw["observation.state.arm"]["q01"] + raw["observation.state.gripper"]["q01"])
        self._state_q99 = np.array(raw["observation.state.arm"]["q99"] + raw["observation.state.gripper"]["q99"])

        self._action_mean = np.array(raw["action.arm"]["mean"] + raw["action.gripper"]["mean"])
        self._action_std = np.array(raw["action.arm"]["std"] + raw["action.gripper"]["std"])
        self._action_q01 = np.array(raw["action.arm"]["q01"] + raw["action.gripper"]["q01"])
        self._action_q99 = np.array(raw["action.arm"]["q99"] + raw["action.gripper"]["q99"])

        self._state_acc: _ChannelAccumulator | None = None
        self._action_acc: _ChannelAccumulator | None = None
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Clear per-episode accumulators."""
        self._state_acc = _ChannelAccumulator(8)
        self._action_acc = _ChannelAccumulator(8)

    # ------------------------------------------------------------------
    def check_state(self, state_8d: np.ndarray, step: int) -> dict:
        """Return per-channel z-score and out-of-range flags for one state."""
        return self._check(
            state_8d, step, "state",
            self._state_mean, self._state_std,
            self._state_q01, self._state_q99,
            self._state_acc,
        )

    def check_action(self, action_8d: np.ndarray, step: int) -> dict:
        """Return per-channel z-score and out-of-range flags for one action."""
        return self._check(
            action_8d, step, "action",
            self._action_mean, self._action_std,
            self._action_q01, self._action_q99,
            self._action_acc,
        )

    # ------------------------------------------------------------------
    def episode_summary(self) -> str:
        """Return a formatted table summarising per-channel OOR statistics."""
        lines = [
            "Distribution Monitor  --  Episode Summary",
            "=" * 72,
            f"{'signal':<8} {'ch':<6} {'steps':>6} {'OOR':>6} {'OOR%':>7} {'max|z|':>8}",
            "-" * 72,
        ]
        for label, acc in [("state", self._state_acc), ("action", self._action_acc)]:
            for i, name in enumerate(CHANNEL_NAMES):
                total = acc.total_steps
                oor = acc.oor_count[i]
                pct = 100.0 * oor / total if total > 0 else 0.0
                max_z = acc.max_abs_z[i]
                lines.append(f"{label:<8} {name:<6} {total:>6d} {oor:>6d} {pct:>6.1f}% {max_z:>8.2f}")
            lines.append("-" * 72)
        return "\n".join(lines)

    # ------------------------------------------------------------------
    @staticmethod
    def _check(
        values: np.ndarray,
        step: int,
        label: str,
        mean: np.ndarray,
        std: np.ndarray,
        q01: np.ndarray,
        q99: np.ndarray,
        acc: _ChannelAccumulator,
    ) -> dict:
        v = np.asarray(values, dtype=np.float64).ravel()
        safe_std = np.where(std > 0, std, 1.0)
        z = (v - mean) / safe_std
        below = v < q01
        above = v > q99
        oor = below | above

        acc.update(z, oor)

        if oor.any():
            parts = []
            for i in range(len(CHANNEL_NAMES)):
                if oor[i]:
                    direction = "below q01" if below[i] else "above q99"
                    parts.append(f"{CHANNEL_NAMES[i]}={v[i]:.4f} z={z[i]:+.2f} ({direction})")
            logger.warning("step %d %s OOR: %s", step, label, "; ".join(parts))

        return {
            "z_scores": z,
            "below_q01": below,
            "above_q99": above,
            "any_oor": bool(oor.any()),
        }


class _ChannelAccumulator:
    """Tracks per-channel out-of-range counts and max |z| across an episode."""

    def __init__(self, n_channels: int) -> None:
        self.total_steps: int = 0
        self.oor_count = np.zeros(n_channels, dtype=np.int64)
        self.max_abs_z = np.zeros(n_channels, dtype=np.float64)

    def update(self, z: np.ndarray, oor: np.ndarray) -> None:
        self.total_steps += 1
        self.oor_count += oor.astype(np.int64)
        np.maximum(self.max_abs_z, np.abs(z), out=self.max_abs_z)
