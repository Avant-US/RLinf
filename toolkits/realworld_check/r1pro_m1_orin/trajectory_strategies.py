# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Trajectory synthesis strategies for the R1 Pro M1 joint-reach collector.

The first version of ``collect.py`` ignored ``target_q`` during data
generation: it emitted ``np.random.uniform(-1, 1, size=7)`` every step,
so trajectories were 7-D goal-blind random walks that almost never
reached the configured target.  Both SFT (no expert demos) and SAC
(extremely sparse positive reward) suffered.

This module replaces per-step uniform noise with goal-aware *strategies*
selected via the classic **Strategy Pattern**:

  * :class:`TrajectoryStrategy` — abstract interface.
  * Five concrete strategies covering linear, smoothed-linear, jittered,
    spline, and goal-biased random-walk synthesis.
  * :class:`StrategyMix` — weighted aggregator (one strategy is sampled
    per episode for episode-level diversity).
  * :func:`build_strategy` — YAML-spec factory.

All strategies emit *raw* normalised actions in :math:`[-1, 1]^7`; the
``SafetyConfigActionProjector`` in :mod:`runtime` still clamps every step
against the hard SafetyConfig limits, so adding new strategies cannot
violate safety.

Dependencies are intentionally NumPy-only (Profile A from
``bt/docs/rwRL/genwaypoint_1.md``); no scipy / rclpy / ROS.
"""
from __future__ import annotations

import abc
from typing import TYPE_CHECKING, Any, Callable

import numpy as np

if TYPE_CHECKING:
    # Avoid pulling rclpy / rlinf at module-load time so this module is
    # importable in plain unit tests on a dev machine without ROS.
    from .runtime import SafetyConfigActionProjector

__all__ = [
    "GoalBiasedRandomWalkStrategy",
    "LinearJitterStrategy",
    "LinearStrategy",
    "MinJerkStrategy",
    "SplineStrategy",
    "StrategyMix",
    "TrajectoryStrategy",
    "build_strategy",
    "list_strategy_names",
]


# ─── Abstract base ────────────────────────────────────────────────
class TrajectoryStrategy(abc.ABC):
    """Per-episode strategy producing raw normalised actions.

    Lifecycle:

      1. ``StrategyMix.pick(rng)`` selects one instance.
      2. ``strategy.reset(start_q=..., target_q=..., projector=..., max_steps=..., rng=...)``
         is called once per episode (right after the env reset).
      3. ``strategy.act(t=t, q_current=q)`` is called each env step.

    Concrete strategies should be **pure NumPy + Python**: no rclpy, no
    scipy.  Heavier dependencies belong in a separate module gated on
    Profile B / C of ``genwaypoint_1.md``.
    """

    name: str = "abstract"

    @abc.abstractmethod
    def reset(
        self,
        *,
        start_q: np.ndarray,
        target_q: np.ndarray,
        projector: SafetyConfigActionProjector,
        max_steps: int,
        rng: np.random.Generator,
    ) -> None:
        """Initialise per-episode internal state."""

    @abc.abstractmethod
    def act(
        self, *, t: int, q_current: np.ndarray
    ) -> np.ndarray:
        """Return a raw normalised action in :math:`[-1, 1]^7` for step ``t``."""


# ─── Concrete strategies ──────────────────────────────────────────
class LinearStrategy(TrajectoryStrategy):
    """Aim directly at ``target_q`` every step.

    Combined with the projector's ``step_cap``, the actual robot motion
    is a constant-rate linear interpolation from ``start_q`` to
    ``target_q`` (rate :math:`= v_{\\max} \\cdot \\text{dt} \\cdot
    \\text{safety\\_step\\_scale}`).  Always reaches the target.
    """

    name = "linear"

    def reset(self, *, start_q, target_q, projector, max_steps, rng):
        self._target_norm = projector.normalize(target_q).astype(np.float32)

    def act(self, *, t, q_current):
        return self._target_norm.copy()


class MinJerkStrategy(TrajectoryStrategy):
    """Aim along the line at the minimum-jerk parameter.

    Maps :math:`s = t / N` to:

    .. math:: \\tau(s) = 10 s^3 - 15 s^4 + 6 s^5

    so that :math:`\\dot{\\tau}(0) = \\dot{\\tau}(1) = 0` and the aim
    point :math:`q_{\\text{aim}}(s) = q_{\\text{start}} + \\tau(s)
    (q_{\\text{target}} - q_{\\text{start}})` accelerates / decelerates
    smoothly.  Always reaches the target.

    Because the projector enforces a per-step velocity cap the actual
    motion remains bounded, but the **emitted action** is non-trivially
    different from :class:`LinearStrategy` — early aim points are close
    to the start, late aim points to the target — giving SFT/SAC richer
    state-action coverage than a single fixed aim vector.
    """

    name = "min_jerk"

    def reset(self, *, start_q, target_q, projector, max_steps, rng):
        self._start_q = np.asarray(start_q, dtype=np.float32).reshape(7)
        self._target_q = np.asarray(target_q, dtype=np.float32).reshape(7)
        self._proj = projector
        self._max_steps = max(int(max_steps), 1)

    def act(self, *, t, q_current):
        s = float(np.clip(t / self._max_steps, 0.0, 1.0))
        tau = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
        aim = self._start_q + tau * (self._target_q - self._start_q)
        aim = np.clip(aim, self._proj.safe_lo, self._proj.safe_hi)
        return self._proj.normalize(aim)


class LinearJitterStrategy(TrajectoryStrategy):
    """Aim at ``target_q`` plus per-step Gaussian noise on the action.

    Each axis: :math:`a_i = \\text{clip}(\\text{target\\_norm}_i +
    \\sigma \\cdot \\mathcal{N}(0, 1), -1, 1)`.  The projector still
    caps the per-step displacement, so noise translates into a small
    "wobble" around the linear path while the average motion still
    converges to the target.
    """

    name = "linear_jitter"

    def __init__(self, sigma: float = 0.10) -> None:
        self.sigma = float(sigma)

    def reset(self, *, start_q, target_q, projector, max_steps, rng):
        self._target_norm = projector.normalize(target_q).astype(np.float32)
        self._rng = rng

    def act(self, *, t, q_current):
        noise = self._rng.normal(0.0, self.sigma, size=7).astype(np.float32)
        return np.clip(self._target_norm + noise, -1.0, 1.0).astype(np.float32)


class SplineStrategy(TrajectoryStrategy):
    """Random via-points + cubic Hermite (Catmull-Rom) interpolation.

    On reset:

      1. Sample :math:`n_v` via-points with :math:`\\alpha_k \\sim
         \\mathcal{U}(0.15, 0.85)` along :math:`s\\in[0,1]`.
      2. For each via-point: ``via_q = line_q(α) + N(0, noise_scale)``
         clipped to ``[safe_lo, safe_hi]``.
      3. Build knot list ``[start_q, via_q_1, ..., via_q_{n_v}, target_q]``
         indexed by ``[0, α_1, ..., α_{n_v}, 1]``.
      4. Use cubic Hermite (Catmull-Rom tangents) for smooth interpolation.

    On ``act(t)``: aim at ``spline(t / max_steps)``.  First and last
    knots are the start / target, so the path is start- and
    target-anchored regardless of via-point randomness.
    """

    name = "spline"

    DEFAULT_NOISE_SCALE = (0.20, 0.10, 0.20, 0.10, 0.20, 0.06, 0.15)

    def __init__(
        self,
        num_via_min: int = 1,
        num_via_max: int = 3,
        noise_scale: list[float] | tuple[float, ...] | None = None,
    ) -> None:
        if num_via_min < 0 or num_via_max < num_via_min:
            raise ValueError(
                f"num_via_min/max must satisfy 0 <= min <= max, got "
                f"{num_via_min}/{num_via_max}"
            )
        self.num_via_min = int(num_via_min)
        self.num_via_max = int(num_via_max)
        self.noise_scale = np.asarray(
            noise_scale if noise_scale is not None else self.DEFAULT_NOISE_SCALE,
            dtype=np.float32,
        ).reshape(7)

    def reset(self, *, start_q, target_q, projector, max_steps, rng):
        start_q = np.asarray(start_q, dtype=np.float32).reshape(7)
        target_q = np.asarray(target_q, dtype=np.float32).reshape(7)
        n_via = int(rng.integers(self.num_via_min, self.num_via_max + 1))
        if n_via > 0:
            alphas = np.sort(
                rng.uniform(0.15, 0.85, size=n_via)
            ).astype(np.float32)
        else:
            alphas = np.empty((0,), dtype=np.float32)
        knots_alpha = np.concatenate([[0.0], alphas, [1.0]]).astype(np.float32)
        knots = np.zeros((knots_alpha.size, 7), dtype=np.float32)
        knots[0] = start_q
        knots[-1] = target_q
        for k, a in enumerate(alphas, start=1):
            line_q = start_q + float(a) * (target_q - start_q)
            noise = (
                rng.normal(0.0, 1.0, size=7).astype(np.float32) * self.noise_scale
            )
            via_q = np.clip(line_q + noise, projector.safe_lo, projector.safe_hi)
            knots[k] = via_q
        self._knots_alpha = knots_alpha
        self._knots = knots
        self._max_steps = max(int(max_steps), 1)
        self._proj = projector

    def _spline_eval(self, alpha: float) -> np.ndarray:
        ka = self._knots_alpha
        kn = self._knots
        a = float(np.clip(alpha, 0.0, 1.0))
        idx = int(np.searchsorted(ka, a, side="right") - 1)
        idx = min(max(idx, 0), len(ka) - 2)
        a0, a1 = float(ka[idx]), float(ka[idx + 1])
        if a1 - a0 < 1e-9:
            return kn[idx + 1].astype(np.float32, copy=True)
        u = (a - a0) / (a1 - a0)
        # Catmull-Rom-style tangents (clamped at endpoints).
        p0 = kn[idx - 1] if idx - 1 >= 0 else kn[idx]
        p1 = kn[idx]
        p2 = kn[idx + 1]
        p3 = kn[idx + 2] if idx + 2 < len(kn) else kn[idx + 1]
        m1 = 0.5 * (p2 - p0)
        m2 = 0.5 * (p3 - p1)
        h00 = 2 * u**3 - 3 * u**2 + 1
        h10 = u**3 - 2 * u**2 + u
        h01 = -2 * u**3 + 3 * u**2
        h11 = u**3 - u**2
        return (h00 * p1 + h10 * m1 + h01 * p2 + h11 * m2).astype(np.float32)

    def act(self, *, t, q_current):
        s = float(np.clip(t / self._max_steps, 0.0, 1.0))
        aim = self._spline_eval(s)
        aim = np.clip(aim, self._proj.safe_lo, self._proj.safe_hi)
        return self._proj.normalize(aim)


class GoalBiasedRandomWalkStrategy(TrajectoryStrategy):
    """RRT-style: each step independently samples 'aim at target' or 'random'.

    .. math::
        a_t = \\begin{cases}
          \\text{normalize}(q_{\\text{target}}) & \\text{w.p. } p_g \\\\
          \\mathcal{U}([-1, 1]^7)               & \\text{otherwise}
        \\end{cases}

    Setting ``p_g = 0`` recovers the original goal-blind random walk;
    keep a small slice (e.g. weight 0.1) for entropy / exploration data.
    """

    name = "random_walk"

    def __init__(
        self, goal_bias: float = 0.5, low: float = -1.0, high: float = 1.0
    ) -> None:
        if not 0.0 <= goal_bias <= 1.0:
            raise ValueError(f"goal_bias must lie in [0, 1], got {goal_bias}")
        self.goal_bias = float(goal_bias)
        self.low = float(low)
        self.high = float(high)

    def reset(self, *, start_q, target_q, projector, max_steps, rng):
        self._target_norm = projector.normalize(target_q).astype(np.float32)
        self._rng = rng

    def act(self, *, t, q_current):
        if self._rng.uniform(0.0, 1.0) < self.goal_bias:
            return self._target_norm.copy()
        return self._rng.uniform(
            self.low, self.high, size=7
        ).astype(np.float32)


# ─── Mix + factory ────────────────────────────────────────────────
class StrategyMix:
    """Weighted aggregator over :class:`TrajectoryStrategy` instances.

    ``pick(rng)`` returns one strategy chosen with probability
    proportional to its weight; the same instance may be picked again on
    a later episode.  Each pick is reset by the caller, so strategies
    are stateless across episodes.
    """

    def __init__(self, items: list[tuple[float, TrajectoryStrategy]]) -> None:
        if not items:
            raise ValueError("StrategyMix needs at least one strategy")
        weights = np.array([float(w) for w, _ in items], dtype=np.float64)
        if (weights < 0).any() or weights.sum() <= 0:
            raise ValueError(
                "StrategyMix weights must be non-negative and sum > 0"
            )
        self._weights = weights / weights.sum()
        self._strategies = [s for _, s in items]

    def pick(self, rng: np.random.Generator) -> TrajectoryStrategy:
        idx = int(rng.choice(len(self._strategies), p=self._weights))
        return self._strategies[idx]

    @property
    def names(self) -> list[str]:
        return [s.name for s in self._strategies]

    @property
    def weights(self) -> list[float]:
        return self._weights.tolist()

    def describe(self) -> str:
        return ", ".join(
            f"{name}={w:.2f}"
            for name, w in zip(self.names, self.weights, strict=True)
        )


_FACTORIES: dict[str, Callable[..., TrajectoryStrategy]] = {
    "linear": lambda **kw: LinearStrategy(),
    "min_jerk": lambda **kw: MinJerkStrategy(),
    "linear_jitter": lambda **kw: LinearJitterStrategy(
        sigma=float(kw.get("sigma", 0.10))
    ),
    "spline": lambda **kw: SplineStrategy(
        num_via_min=int(kw.get("num_via_min", 1)),
        num_via_max=int(kw.get("num_via_max", 3)),
        noise_scale=kw.get("noise_scale", None),
    ),
    "random_walk": lambda **kw: GoalBiasedRandomWalkStrategy(
        goal_bias=float(kw.get("goal_bias", 0.5))
    ),
}


def list_strategy_names() -> list[str]:
    return list(_FACTORIES.keys())


def _default_spec() -> list[dict[str, Any]]:
    """Sensible default mix when YAML omits ``data_generation.strategies``."""
    return [
        {"name": "linear", "weight": 1.0},
        {"name": "min_jerk", "weight": 2.0},
        {"name": "linear_jitter", "weight": 1.0, "params": {"sigma": 0.10}},
        {"name": "spline", "weight": 3.0,
         "params": {"num_via_min": 1, "num_via_max": 3}},
        {"name": "random_walk", "weight": 0.5,
         "params": {"goal_bias": 0.5}},
    ]


def build_strategy(spec: dict[str, Any] | list | None) -> StrategyMix:
    """Build a :class:`StrategyMix` from a YAML / dict spec.

    Accepted shapes::

        # 1. Long form (recommended) — list of {name, weight, params}.
        data_generation:
          strategies:
            - {name: linear,        weight: 1.0}
            - {name: min_jerk,      weight: 2.0}
            - {name: spline,        weight: 3.0, params: {num_via_max: 4}}
            - {name: random_walk,   weight: 0.5, params: {goal_bias: 0.5}}

        # 2. Short form — dict of name -> weight (uses default params).
        data_generation:
          strategies: {linear: 1, min_jerk: 2, spline: 3}

        # 3. Container with explicit ``items`` key (back-compat).
        data_generation:
          strategies:
            items:
              - {name: linear, weight: 1}
              - ...

        # 4. Omitted entirely → ``_default_spec()``.

    Unknown strategy names raise ``ValueError`` with the list of valid
    names so YAML typos are caught at startup, not mid-episode.
    """
    if spec is None or (hasattr(spec, "__len__") and len(spec) == 0):
        spec_items = _default_spec()
    elif isinstance(spec, dict) and "items" in spec:
        spec_items = list(spec["items"])
    elif isinstance(spec, dict):
        spec_items = [
            {"name": str(k), "weight": float(v)} for k, v in spec.items()
        ]
    elif isinstance(spec, list):
        spec_items = list(spec)
    else:
        raise TypeError(
            "data_generation.strategies must be list / dict / None, got "
            f"{type(spec).__name__}"
        )

    items: list[tuple[float, TrajectoryStrategy]] = []
    for raw in spec_items:
        if not isinstance(raw, dict) or "name" not in raw:
            raise ValueError(
                "Each strategy entry must be a dict with at least 'name'; "
                f"got {raw!r}"
            )
        name = str(raw["name"])
        if name not in _FACTORIES:
            raise ValueError(
                f"Unknown trajectory strategy '{name}'. Valid names: "
                + ", ".join(_FACTORIES.keys())
            )
        weight = float(raw.get("weight", 1.0))
        params = dict(raw.get("params") or {})
        items.append((weight, _FACTORIES[name](**params)))
    return StrategyMix(items)
