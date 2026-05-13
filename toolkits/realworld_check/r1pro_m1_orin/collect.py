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
"""Phase 1: multi-pair data collection for R1 Pro M1 joint reach.

Sweeps every ``(start_q, target_q)`` in ``data_generation.pairs`` and
records ``(obs, safe_action, reward, next_obs, done)`` tuples produced by
a *goal-aware trajectory strategy* (Strategy Pattern, see
:mod:`.trajectory_strategies`) through the SafetyConfigActionProjector.
The anchor pair must appear in the list verbatim (validated up front),
so the emitted buffer always contains data from the exact pair used by
Phase 3.

Why a strategy mix matters: the original implementation emitted
``np.random.uniform(-1, 1)`` per step, ignoring ``target_q`` during
trajectory generation.  In 7-D, a step-capped random walk almost never
visits a specific target, so the buffer contained no goal-reaching
transitions.  The strategy mix (linear / min_jerk / linear_jitter /
spline / goal-biased random_walk) ensures every episode reaches
``target_q`` while still providing path-shape diversity.  Safety remains
enforced by the projector regardless of which strategy is active.
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


def validate_pairs(
    pairs: list[dict],
    anchor: dict,
    proj: SafetyConfigActionProjector,
) -> None:
    """Assert every pair is safe and the anchor pair is present verbatim."""
    if not pairs:
        raise ValueError("data_generation.pairs is empty")

    anchor_start = np.asarray(anchor["start_q"], dtype=np.float32).reshape(7)
    anchor_target = np.asarray(anchor["target_q"], dtype=np.float32).reshape(7)
    proj.assert_inside("anchor_pair.start_q", anchor_start)
    proj.assert_inside("anchor_pair.target_q", anchor_target)

    found = False
    for i, pair in enumerate(pairs):
        s = np.asarray(pair["start_q"], dtype=np.float32).reshape(7)
        t = np.asarray(pair["target_q"], dtype=np.float32).reshape(7)
        proj.assert_inside(f"pairs[{i}].start_q", s)
        proj.assert_inside(f"pairs[{i}].target_q", t)
        if np.allclose(s, anchor_start, atol=1e-6) and np.allclose(
            t, anchor_target, atol=1e-6
        ):
            found = True
    if not found:
        raise ValueError(
            "anchor_pair must appear verbatim in data_generation.pairs:\n"
            f"  anchor.start_q  = {anchor['start_q']}\n"
            f"  anchor.target_q = {anchor['target_q']}\n"
            "Open config.yaml and ensure one entry of `pairs` matches."
        )
    print(
        f"[VALIDATE] {len(pairs)} pairs in safe range; anchor pair found."
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="R1 Pro M1 joint reach — Phase 1 (multi-pair collect)."
    )
    default_cfg = Path(__file__).with_name("config.yaml")
    p.add_argument("--config", default=str(default_cfg))
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)

    # Reconcile ROS env vars vs. cfg so the shell's ROS_DOMAIN_ID /
    # RMW_IMPLEMENTATION (where mobiman is actually running) wins.
    # Also writes the effective values back into env.override_cfg so
    # LocalR1ProController does not later overwrite ROS_DOMAIN_ID with a
    # stale config default.
    resolve_ros_env(cfg)

    set_seed(int(get(cfg, "seed")))
    proj = build_projector(cfg)

    pairs = list(get(cfg, "data_generation.pairs") or [])
    anchor = get(cfg, "data_generation.anchor_pair") or {}
    episodes_per_pair = int(get(cfg, "data_generation.episodes_per_pair"))
    buffer_name = str(
        get(cfg, "data_generation.buffer_save_name", "replay_buffer_phase1.npz")
    )

    # Build the trajectory strategy mix once.  The mix is reused across
    # pairs and episodes; per-episode seeding is derived from the global
    # seed so runs are reproducible.
    strategy_mix: StrategyMix = build_strategy(
        get(cfg, "data_generation.strategies")
    )
    print(f"[STRATEGY] mix = [{strategy_mix.describe()}]")

    validate_pairs(pairs, anchor, proj)

    out_dir = Path(str(get(cfg, "runtime.out_dir")))
    out_dir.mkdir(parents=True, exist_ok=True)

    buf = ReplayBuffer(
        obs_dim=int(get(cfg, "sac.obs_dim")),
        action_dim=int(get(cfg, "sac.action_dim")),
        size=int(get(cfg, "sac.replay_size")),
    )
    max_steps = int(get(cfg, "env.override_cfg.max_num_steps"))

    for pair_idx, pair in enumerate(pairs):
        start_q = pair["start_q"]
        target_q = pair["target_q"]
        print("\n" + "=" * 64)
        print(
            f"[COLLECT] Pair {pair_idx + 1}/{len(pairs)}: "
            f"start={start_q}  target={target_q}"
        )
        env = build_env(cfg, start_q=start_q, target_q=target_q)
        preflight(
            env,
            proj,
            start_q,
            target_q,
            sub_wait_timeout_sec=float(
                get(cfg, "runtime.preflight_sub_wait_timeout_sec", 15.0)
            ),
            sub_poll_interval_sec=float(
                get(cfg, "runtime.preflight_sub_poll_interval_sec", 0.5)
            ),
        )
        start_q_np = np.asarray(start_q, dtype=np.float32)

        target_q_np = np.asarray(target_q, dtype=np.float32)
        seed_root = int(get(cfg, "seed"))
        try:
            for ep in range(1, episodes_per_pair + 1):
                # Per-episode RNG so jitter / via-points / random_walk
                # are reproducible from (seed, pair_idx, ep).
                ep_rng = np.random.default_rng(
                    seed_root + 1_000_003 * pair_idx + ep
                )
                strategy = strategy_mix.pick(ep_rng)
                strategy.reset(
                    start_q=start_q_np,
                    target_q=target_q_np,
                    projector=proj,
                    max_steps=max_steps,
                    rng=ep_rng,
                )

                obs = safe_move_to_q(env, proj, start_q_np, cfg)
                ep_transitions = 0
                ep_return = 0.0
                last_q = start_q_np.copy()
                for t in range(max_steps):
                    # Pull q_current for strategies that may want it
                    # (current strategies ignore it, but the interface
                    # is stable for future closed-loop policies).
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
                pos_err = float(np.linalg.norm(last_q - target_q_np))
                print(
                    f"  [COLLECT] pair={pair_idx + 1} ep={ep:>3} "
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
        f"\n[COLLECT] done: {buf.count} transitions from "
        f"{len(pairs)} pairs x {episodes_per_pair} episodes -> {save_path}"
    )


if __name__ == "__main__":
    main()
