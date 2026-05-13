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
"""Phase 1 (variant): fixed-target / random-start data collection.

Sister script to :mod:`collect`.  ``collect.py`` sweeps user-defined
``(start_q, target_q)`` pairs from ``data_generation.pairs``.  This
script instead:

  1. Reads a single fixed ``target_q`` (CLI ``--target-q`` overrides
     ``data_generation.random_start.fixed_target_q``; if neither is
     given, falls back to ``data_generation.anchor_pair.target_q``).
  2. Samples ``N`` diverse ``start_q`` candidates uniformly inside
     ``[safe_lo, safe_hi]`` with three rejection-sampling constraints:

       a. ``min_distance_rad <= ||start - target||_2 <= max_distance_rad``
          — every start is reachable within ``max_num_steps`` but not
          trivially close to the target.
       b. ``||start_i - start_j||_2 >= min_pairwise_dist_rad`` — global
          diversity (no two starts cluster).
       c. Optional ``fixed_axes`` pin selected joints to constants
          (default: J7 = 0.0) so the random-start dataset stays in
          subspaces where the policy actually needs to learn.

  3. For each ``(random_start_i, fixed_target)`` pair, runs the same
     :class:`StrategyMix` (linear / min_jerk / linear_jitter / spline /
     random_walk) used by :mod:`collect`, ensuring goal-aware,
     safety-clamped trajectories.

The replay buffer is saved to a separate file (default
``replay_buffer_phase1_random_start.npz``) so it does not collide with
the buffer produced by :mod:`collect`.

Why a separate collector? Phase 3 (anchor RL) wants buffer data that
already covers the anchor's *state distribution* (the policy will see
many starts close to the home pose, but the user may move the anchor's
``start_q`` between runs).  Diversifying starts at fixed target
trains the policy on a "many roads, one destination" generalisation
pattern, which dramatically improves robustness when the human
operator manually positions the arm before each rollout.

Safety: the projector clamps every emitted action against the
SafetyConfig limits regardless of the random start, and the start
itself is asserted inside ``[safe_lo, safe_hi]`` before any motion
command is issued.  Setting ``min_distance_rad`` too small produces
trivial-but-safe data; setting it too large to reach in
``max_num_steps`` simply wastes episodes (no hardware risk).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .runtime import (
    ReplayBuffer,
    SafetyConfigActionProjector,
    build_env,
    build_projector,
    flatten_obs,
    get,
    load_yaml,
    preflight,
    resolve_ros_env,
    safe_move_to_q,
    safe_step,
    set_seed,
)
from .trajectory_strategies import StrategyMix, build_strategy


# ─── Random-start sampling ─────────────────────────────────────────
def sample_random_starts(
    *,
    num_starts: int,
    target_q: np.ndarray,
    proj: SafetyConfigActionProjector,
    rng: np.random.Generator,
    min_distance_rad: float,
    max_distance_rad: float,
    min_pairwise_dist_rad: float,
    max_attempts_per_start: int,
    fixed_axes: list[int] | None = None,
    fixed_axes_values: list[float] | None = None,
) -> list[np.ndarray]:
    """Rejection-sample ``num_starts`` diverse ``start_q`` candidates.

    Sampling strategy: shell-around-target.

    Naive uniform sampling in ``[safe_lo, safe_hi]^7`` has acceptance
    probability ``V_shell / V_safe`` which is :math:`\\sim 10^{-4}` for
    the R1 Pro right arm: the safe joint range is wide (e.g. J1 spans
    ~5 rad) but the desired distance band ``[0.15, 0.50] rad`` is a
    thin shell around ``target_q``.  We therefore sample directly in
    that shell:

      1. Direction :math:`\\hat{u} \\sim \\text{Uniform}(S^{d-1})` — a
         unit vector on the :math:`d`-sphere where :math:`d` is the
         number of *free* axes (axes not in ``fixed_axes``).  Done by
         sampling :math:`u \\sim \\mathcal{N}(0, I_d)` and dividing by
         :math:`\\|u\\|`.
      2. Radius :math:`r \\sim \\text{Uniform}[d_{\\min}, d_{\\max}]`.
      3. Candidate :math:`q = q_{\\text{target}} + r \\cdot \\hat{u}`
         on free axes; pin ``fixed_axes`` to ``fixed_axes_values``.
      4. Reject if outside ``[safe_lo, safe_hi]`` (axis-aligned safe
         box) or if pairwise distance to already-accepted starts is
         below ``min_pairwise_dist_rad``.

    By construction every candidate satisfies the distance band
    exactly, so rejection only handles the safety-box and diversity
    constraints — acceptance rate is typically > 50%.

    Parameters
    ----------
    num_starts:
        Number of accepted starts to return.
    target_q:
        Fixed end-point used as the shell centre.
    proj:
        Projector providing ``safe_lo`` / ``safe_hi`` for the safety
        rejection.  All accepted candidates satisfy
        ``proj.assert_inside``.
    rng:
        Reproducible NumPy generator seeded from
        ``seed + random_start.seed_offset``.
    min_distance_rad / max_distance_rad:
        L2 distance band ``[d_min, d_max]`` between each accepted
        start and ``target_q`` on the *free* axes; ensures reachable
        within ``env.max_num_steps`` and non-trivially far from target.
    min_pairwise_dist_rad:
        Minimum L2 distance between any two accepted starts.
    max_attempts_per_start:
        Per-start rejection budget; hitting this raises with helpful
        diagnostics so the user knows to relax constraints.
    fixed_axes / fixed_axes_values:
        Optional axes to pin to specified values (e.g. ``[6]`` /
        ``[0.0]`` to keep J7 at zero — same DoF reduction recommended
        in the v3.2 anchor-pair design).
    """
    if num_starts <= 0:
        raise ValueError(f"num_starts must be > 0, got {num_starts}")
    if not (0.0 <= min_distance_rad <= max_distance_rad):
        raise ValueError(
            "Require 0 <= min_distance_rad <= max_distance_rad; got "
            f"min={min_distance_rad}, max={max_distance_rad}"
        )

    target_q = np.asarray(target_q, dtype=np.float32).reshape(7)
    proj.assert_inside("random_start.fixed_target_q", target_q)

    fixed_axes = [int(a) for a in (fixed_axes or [])]
    fixed_values = [float(v) for v in (fixed_axes_values or [])]
    if len(fixed_axes) != len(fixed_values):
        raise ValueError(
            "fixed_axes and fixed_axes_values must have equal length; got "
            f"{len(fixed_axes)} vs {len(fixed_values)}"
        )
    for a in fixed_axes:
        if not 0 <= a < 7:
            raise ValueError(f"fixed_axes entries must be in [0, 7); got {a}")
    for ax, val in zip(fixed_axes, fixed_values, strict=True):
        if not (proj.safe_lo[ax] <= val <= proj.safe_hi[ax]):
            raise ValueError(
                f"fixed_axes[{ax}] = {val} is outside the safe range "
                f"[{proj.safe_lo[ax]:.3f}, {proj.safe_hi[ax]:.3f}]; "
                "pick a value inside the SafetyConfig margin."
            )

    free_axes = np.array(
        [i for i in range(7) if i not in set(fixed_axes)], dtype=np.int64
    )
    if free_axes.size == 0:
        raise ValueError(
            "All 7 axes are pinned by fixed_axes; cannot sample any start."
        )
    d_free = int(free_axes.size)

    accepted: list[np.ndarray] = []
    attempts = 0
    total_budget = int(max_attempts_per_start) * int(num_starts)
    while len(accepted) < num_starts and attempts < total_budget:
        attempts += 1
        # Step 1: uniform direction on (d_free - 1)-sphere.
        u = rng.normal(0.0, 1.0, size=d_free).astype(np.float32)
        u_norm = float(np.linalg.norm(u))
        if u_norm < 1e-9:
            continue
        u /= u_norm
        # Step 2: radius in [d_min, d_max].
        r = float(rng.uniform(min_distance_rad, max_distance_rad))
        # Step 3: build candidate; pin fixed axes to their values, set
        # free axes to target + r * u.
        cand = np.empty(7, dtype=np.float32)
        cand[free_axes] = target_q[free_axes] + r * u
        for ax, val in zip(fixed_axes, fixed_values, strict=True):
            cand[ax] = val
        # Step 4a: safety-box rejection.
        if not (
            np.all(cand >= proj.safe_lo) and np.all(cand <= proj.safe_hi)
        ):
            continue
        # Step 4b: pairwise diversity rejection.
        if accepted:
            d_pair = min(
                float(np.linalg.norm(cand - a)) for a in accepted
            )
            if d_pair < min_pairwise_dist_rad:
                continue
        accepted.append(cand)

    if len(accepted) < num_starts:
        raise RuntimeError(
            f"Could only accept {len(accepted)}/{num_starts} random starts "
            f"after {attempts} attempts. Likely causes:\n"
            f"  * min_pairwise_dist_rad ({min_pairwise_dist_rad}) too large "
            f"vs. shell thickness ({max_distance_rad - min_distance_rad})\n"
            f"  * max_distance_rad ({max_distance_rad}) places most of the "
            f"shell outside the safe box (target is near a joint limit)\n"
            f"  * fixed_axes pinning leaves too few free axes\n"
            "Relax the constraints in config.yaml under "
            "data_generation.random_start.* and retry."
        )
    return accepted


# ─── Argument parsing ─────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "R1 Pro M1 joint reach — Phase 1 variant: "
            "fixed target + random diverse starts."
        )
    )
    default_cfg = Path(__file__).with_name("config.yaml")
    p.add_argument("--config", default=str(default_cfg))
    p.add_argument(
        "--target-q",
        type=str,
        default=None,
        help=(
            "Comma-separated 7-DoF absolute target q (rad), e.g. "
            "'0.12,-0.30,0.08,-0.55,0.05,0.15,0.0'. Overrides "
            "data_generation.random_start.fixed_target_q and "
            "anchor_pair.target_q."
        ),
    )
    p.add_argument(
        "--num-starts",
        type=int,
        default=None,
        help="Override data_generation.random_start.num_starts.",
    )
    p.add_argument(
        "--episodes-per-pair",
        type=int,
        default=None,
        help=(
            "Override data_generation.random_start.episodes_per_pair "
            "(falls back to data_generation.episodes_per_pair)."
        ),
    )
    p.add_argument(
        "--buffer-name",
        type=str,
        default=None,
        help="Override the saved buffer filename.",
    )
    return p.parse_args()


def _parse_target_q_str(s: str) -> list[float]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if len(parts) != 7:
        raise ValueError(
            f"--target-q must have 7 comma-separated floats, got {len(parts)}"
        )
    try:
        return [float(p) for p in parts]
    except ValueError as e:
        raise ValueError(f"--target-q parse error: {e}") from None


# ─── Main ─────────────────────────────────────────────────────────
def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)
    resolve_ros_env(cfg)

    set_seed(int(get(cfg, "seed")))
    proj = build_projector(cfg)

    # Resolve the fixed target_q (CLI > random_start.fixed_target_q > anchor).
    if args.target_q is not None:
        target_q_list = _parse_target_q_str(args.target_q)
        target_src = "CLI --target-q"
    else:
        target_q_list = (
            get(cfg, "data_generation.random_start.fixed_target_q", None)
            or get(cfg, "data_generation.anchor_pair.target_q")
        )
        target_src = "config.yaml"
    target_q = np.asarray(target_q_list, dtype=np.float32).reshape(7)
    proj.assert_inside("fixed target_q", target_q)
    print(f"[TARGET] {target_q.tolist()}  (source: {target_src})")
    #########################################
    # import time
    # time.sleep(10)
    # env = build_env(cfg, start_q=None, target_q=target_q_list)
    # time.sleep(5)
    # env.step(env.config.home_q_right)
    # time.sleep(5)
    # env.step(env._target_q_right)
    # time.sleep(5)
    # env.step(env.config.home_q_right)
    # time.sleep(5)
    # exit(100)
    # prient("EXIT...EXIT...EXIT...")
    ##########################################

    # Sampling parameters (with sensible defaults if section missing).
    rs = dict(get(cfg, "data_generation.random_start") or {})
    num_starts = int(args.num_starts or rs.get("num_starts", 8))
    min_d = float(rs.get("min_distance_rad", 0.15))
    max_d = float(rs.get("max_distance_rad", 0.50))
    min_pair = float(rs.get("min_pairwise_dist_rad", 0.10))
    max_attempts = int(rs.get("max_attempts_per_start", 500))
    fixed_axes = list(rs.get("fixed_axes") or [])
    fixed_values = list(rs.get("fixed_axes_values") or [])
    seed_offset = int(rs.get("seed_offset", 7919))

    seed_root = int(get(cfg, "seed"))
    sample_rng = np.random.default_rng(seed_root + seed_offset)
    starts = sample_random_starts(
        num_starts=num_starts,
        target_q=target_q,
        proj=proj,
        rng=sample_rng,
        min_distance_rad=min_d,
        max_distance_rad=max_d,
        min_pairwise_dist_rad=min_pair,
        max_attempts_per_start=max_attempts,
        fixed_axes=fixed_axes,
        fixed_axes_values=fixed_values,
    )
    print(f"[SAMPLE] accepted {len(starts)} diverse starts:")
    for i, s in enumerate(starts):
        d = float(np.linalg.norm(s - target_q))
        print(
            f"  start[{i:>2}]: q={[round(float(x), 3) for x in s]}  "
            f"||q-target||={d:.3f}rad"
        )

    episodes_per_pair = int(
        args.episodes_per_pair
        or rs.get("episodes_per_pair")
        or get(cfg, "data_generation.episodes_per_pair")
    )
    buffer_name = str(
        args.buffer_name
        or rs.get("buffer_save_name", "replay_buffer_phase1_random_start.npz")
    )

    # Strategy mix (reuses the same YAML knob as collect.py).
    strategy_mix: StrategyMix = build_strategy(
        get(cfg, "data_generation.strategies")
    )
    print(f"[STRATEGY] mix = [{strategy_mix.describe()}]")

    out_dir = Path(str(get(cfg, "runtime.out_dir")))
    out_dir.mkdir(parents=True, exist_ok=True)
    buf = ReplayBuffer(
        obs_dim=int(get(cfg, "sac.obs_dim")),
        action_dim=int(get(cfg, "sac.action_dim")),
        size=int(get(cfg, "sac.replay_size")),
    )
    max_steps = int(get(cfg, "env.override_cfg.max_num_steps"))

    target_q_list = target_q.tolist()
    
    for pair_idx, start_q_np in enumerate(starts):
        start_q = start_q_np.tolist()
        print("\n" + "=" * 64)
        print(
            f"[COLLECT-RS] Pair {pair_idx + 1}/{len(starts)}: "
            f"start={[round(x, 3) for x in start_q]}  "
            f"target={[round(x, 3) for x in target_q_list]}"
        )
        env = build_env(cfg, start_q=start_q, target_q=target_q_list)
        preflight(
            env,
            proj,
            start_q,
            target_q_list,
            sub_wait_timeout_sec=float(
                get(cfg, "runtime.preflight_sub_wait_timeout_sec", 15.0)
            ),
            sub_poll_interval_sec=float(
                get(cfg, "runtime.preflight_sub_poll_interval_sec", 0.5)
            ),
        )
        
        try:
            for ep in range(1, episodes_per_pair + 1):
                # Per-episode RNG: deterministic from (seed, seed_offset, pair, ep).
                ep_rng = np.random.default_rng(
                    seed_root + seed_offset + 1_000_003 * pair_idx + ep
                )
                strategy = strategy_mix.pick(ep_rng)
                strategy.reset(
                    start_q=start_q_np,
                    target_q=target_q,
                    projector=proj,
                    max_steps=max_steps,
                    rng=ep_rng,
                )

                obs = safe_move_to_q(env, proj, start_q_np, cfg)
                ep_transitions = 0
                ep_return = 0.0
                last_q = start_q_np.copy()
                for t in range(max_steps):
                    st = env._controller.get_state().wait()[0]
                    q_current = np.asarray(
                        st.right_arm_qpos, dtype=np.float32
                    )
                    raw_action = strategy.act(t=t, q_current=q_current).astype(
                        np.float32
                    )
                    nxt_obs, reward, term, trunc, info, safe_a = safe_step(
                        env, proj, raw_action
                    )
                    next_obs = flatten_obs(nxt_obs)
                    done = bool(term or trunc)
                    buf.add(obs, safe_a, reward, next_obs, done)
                    obs = next_obs
                    last_q = q_current
                    ep_transitions += 1
                    ep_return += float(reward)
                    if info.get("safe_pause") or done:
                        break
                pos_err = float(np.linalg.norm(last_q - target_q))
                print(
                    f"  [COLLECT-RS] pair={pair_idx + 1} ep={ep:>3} "
                    f"strategy={strategy.name:<13} "
                    f"transitions={ep_transitions:>3} "
                    f"return={ep_return:+.3f} "
                    f"pos_err={pos_err:.3f}rad "
                    f"buffer={buf.count}"
                )
        finally:
            try:
                env._controller.apply_brake(True).wait()
            except Exception:
                pass
            try:
                env._controller.shutdown()
            except Exception:
                pass

    save_path = out_dir / buffer_name
    buf.save(save_path)
    print(
        f"\n[COLLECT-RS] done: {buf.count} transitions from "
        f"{len(starts)} random starts x {episodes_per_pair} episodes "
        f"-> {save_path}"
    )


if __name__ == "__main__":
    main()
