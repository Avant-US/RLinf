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
"""Phase 3 (variant): online SAC with random-start, fixed-target.

Sister script to :mod:`train_sac`.  ``train_sac.py`` resets to a single
fixed ``start_q`` every episode, so the learned policy only knows one
trajectory.  This script instead resets to a fresh random ``start_q``
each episode while keeping ``target_q`` fixed, training a generalising
policy that can reach the target from *any* safe start.

Why a separate script? The SAC math is identical, but three operational
choices change:

  1. **Reset distribution.** ``safe_move_to_q`` is called with a
     per-episode ``start_q`` sampled inside the SafetyConfig safe box,
     constrained to a distance band around ``target_q``.  The shell
     sampler from :func:`collect_random_start.sample_random_starts`
     is reused so the training and data-collection distributions
     match, which is critical for off-policy SAC stability.
  2. **Two pool modes.**

       * ``fresh`` — sample a brand-new start every episode.  Maximum
         coverage; recommended once the buffer warm-start is stable.
       * ``fixed`` — pre-sample a pool of ``pool_size`` starts at
         script init and cycle / shuffle through them.  Easier to
         debug because each start is visited deterministically.

  3. **Held-out evaluation.** A separate pool of evaluation starts is
     sampled at script init (with a different RNG offset) and never
     used for training transitions.  Every ``eval_every_episodes``
     rollouts the script runs deterministic rollouts on this pool to
     measure *generalisation* (training success ≠ generalisation).

The SAC objectives are the standard double-Q + entropy formulation
implemented in :mod:`train_sac`; we import :func:`make_sac`,
:func:`act`, and :func:`update` from there to avoid drift.

Observation is **target-agnostic** (right_arm_qpos + right_ee_pose =
14-D); since ``target_q`` is fixed across all episodes the policy
implicitly memorises it via gradient descent, the same way
:mod:`train_sac` does.  If you ever want to randomise the target as
well, you must add ``target_q`` to the observation (goal-conditioned
policy, +7 dims) and rebuild the MLPPolicy with the new ``obs_dim``.
"""
from __future__ import annotations

import argparse
import copy
from collections import deque
from pathlib import Path

import numpy as np
import torch

from .collect_random_start import sample_random_starts
from .runtime import (
    ReplayBuffer,
    build_env,
    build_projector,
    choose_device,
    flatten_obs,
    get,
    load_yaml,
    preflight,
    resolve_ros_env,
    safe_move_to_q,
    safe_step,
    set_seed,
)
from .train_sac import act, make_sac, update


# ─── Start sampler ────────────────────────────────────────────────
class StartSampler:
    """Per-episode start_q provider.

    Two modes:

      * ``fresh``  : ``next_start(ep)`` returns a brand-new sample
        from the shell distribution every call.  Resampling is
        deterministic given ``rng_seed`` and the call index.
      * ``fixed``  : at construction we draw ``pool_size`` starts;
        ``next_start(ep)`` cycles through them (optionally
        re-shuffling each "epoch" of ``len(pool)`` episodes).

    Either mode reuses :func:`collect_random_start.sample_random_starts`
    so the safety-box check, distance-band check, and ``fixed_axes``
    pinning behave identically to the data-collection pipeline.
    """

    def __init__(
        self,
        *,
        mode: str,
        target_q: np.ndarray,
        proj,
        rng_seed: int,
        pool_size: int,
        min_distance_rad: float,
        max_distance_rad: float,
        min_pairwise_dist_rad: float,
        max_attempts_per_start: int,
        fixed_axes: list[int] | None,
        fixed_axes_values: list[float] | None,
        shuffle_each_epoch: bool,
    ) -> None:
        if mode not in {"fresh", "fixed"}:
            raise ValueError(f"mode must be 'fresh' or 'fixed', got {mode!r}")
        self.mode = mode
        self.target_q = np.asarray(target_q, dtype=np.float32).reshape(7)
        self.proj = proj
        self.min_distance_rad = float(min_distance_rad)
        self.max_distance_rad = float(max_distance_rad)
        self.min_pairwise_dist_rad = float(min_pairwise_dist_rad)
        self.max_attempts_per_start = int(max_attempts_per_start)
        self.fixed_axes = list(fixed_axes or [])
        self.fixed_axes_values = list(fixed_axes_values or [])
        self.shuffle_each_epoch = bool(shuffle_each_epoch)
        self._rng_seed = int(rng_seed)
        # ``fixed`` mode is the only one with a persistent pool; in
        # ``fresh`` mode we keep ``pool_size`` only to validate that
        # the user chose at least 1 (sanity guard).
        if mode == "fixed":
            if pool_size <= 0:
                raise ValueError(
                    f"pool_size must be > 0 in fixed mode, got {pool_size}"
                )
            self.pool: list[np.ndarray] = sample_random_starts(
                num_starts=pool_size,
                target_q=self.target_q,
                proj=proj,
                rng=np.random.default_rng(self._rng_seed),
                min_distance_rad=self.min_distance_rad,
                max_distance_rad=self.max_distance_rad,
                min_pairwise_dist_rad=self.min_pairwise_dist_rad,
                max_attempts_per_start=self.max_attempts_per_start,
                fixed_axes=self.fixed_axes,
                fixed_axes_values=self.fixed_axes_values,
            )
            self._order = list(range(len(self.pool)))
            self._epoch_rng = np.random.default_rng(self._rng_seed + 1)
            if self.shuffle_each_epoch:
                self._epoch_rng.shuffle(self._order)
        else:
            self.pool = []

    def next_start(self, ep: int) -> tuple[np.ndarray, int]:
        """Return ``(start_q, pool_idx)``; ``pool_idx`` is -1 for fresh mode."""
        if self.mode == "fresh":
            ep_rng = np.random.default_rng(self._rng_seed + 1_000_003 * ep)
            start = sample_random_starts(
                num_starts=1,
                target_q=self.target_q,
                proj=self.proj,
                rng=ep_rng,
                min_distance_rad=self.min_distance_rad,
                max_distance_rad=self.max_distance_rad,
                min_pairwise_dist_rad=0.0,
                max_attempts_per_start=self.max_attempts_per_start,
                fixed_axes=self.fixed_axes,
                fixed_axes_values=self.fixed_axes_values,
            )[0]
            return start, -1
        # fixed
        n = len(self.pool)
        position_in_epoch = (ep - 1) % n
        if position_in_epoch == 0 and ep > 1 and self.shuffle_each_epoch:
            self._epoch_rng.shuffle(self._order)
        idx = self._order[position_in_epoch]
        return self.pool[idx].copy(), idx


# ─── Held-out eval pool ───────────────────────────────────────────
def sample_eval_pool(
    *,
    target_q: np.ndarray,
    proj,
    rng_seed: int,
    pool_size: int,
    min_distance_rad: float,
    max_distance_rad: float,
    min_pairwise_dist_rad: float,
    max_attempts_per_start: int,
    fixed_axes: list[int] | None,
    fixed_axes_values: list[float] | None,
) -> list[np.ndarray]:
    """Same sampler as training, but with a *different* seed offset."""
    return sample_random_starts(
        num_starts=pool_size,
        target_q=target_q,
        proj=proj,
        rng=np.random.default_rng(rng_seed),
        min_distance_rad=min_distance_rad,
        max_distance_rad=max_distance_rad,
        min_pairwise_dist_rad=min_pairwise_dist_rad,
        max_attempts_per_start=max_attempts_per_start,
        fixed_axes=fixed_axes,
        fixed_axes_values=fixed_axes_values,
    )


def evaluate_on_pool(
    *,
    sac,
    env,
    proj,
    cfg: dict,
    target_q: np.ndarray,
    eval_starts: list[np.ndarray],
    episodes_per_start: int,
    max_steps: int,
) -> dict[str, float]:
    """Deterministic rollouts on held-out starts; no buffer writes.

    Returns
    -------
    dict with keys ``mean_return``, ``mean_steps``, ``success_rate``,
    ``mean_pos_err``.
    """
    rets, steps, succs, errs = [], [], [], []
    for s_idx, s in enumerate(eval_starts):
        for _ in range(int(episodes_per_start)):
            obs = safe_move_to_q(env, proj, s, cfg)
            ep_ret = 0.0
            ep_steps = 0
            success = False
            last_q = s.copy()
            for _t in range(max_steps):
                raw_action = act(sac, obs, deterministic=True)
                nxt_obs, reward, term, trunc, info, _safe_a = safe_step(
                    env, proj, raw_action
                )
                obs = flatten_obs(nxt_obs)
                ep_ret += float(reward)
                ep_steps += 1
                st = env._controller.get_state().wait()[0]
                last_q = np.asarray(st.right_arm_qpos, dtype=np.float32)
                if info.get("success") and not success:
                    success = True
                if info.get("safe_pause") or term or trunc:
                    break
            rets.append(ep_ret)
            steps.append(ep_steps)
            succs.append(int(success))
            errs.append(float(np.linalg.norm(last_q - target_q)))
    return {
        "mean_return": float(np.mean(rets)) if rets else 0.0,
        "mean_steps": float(np.mean(steps)) if steps else 0.0,
        "success_rate": float(np.mean(succs)) if succs else 0.0,
        "mean_pos_err": float(np.mean(errs)) if errs else 0.0,
    }


# ─── CLI ──────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "R1 Pro M1 joint reach — Phase 3 variant: online SAC with "
            "random-start, fixed-target."
        )
    )
    default_cfg = Path(__file__).with_name("config.yaml")
    p.add_argument("--config", default=str(default_cfg))
    p.add_argument(
        "--sft-checkpoint",
        default=None,
        help="Path to Phase 2 SFT checkpoint (default: from cfg if present).",
    )
    p.add_argument(
        "--phase1-buffer",
        default=None,
        help=(
            "Path to Phase 1 .npz buffer (default: random-start buffer if "
            "present, else fixed-pair buffer)."
        ),
    )
    p.add_argument(
        "--target-q",
        type=str,
        default=None,
        help=(
            "Comma-separated 7-DoF absolute target q (rad). Overrides "
            "data_generation.random_start.fixed_target_q and "
            "anchor_pair.target_q."
        ),
    )
    p.add_argument(
        "--pool-mode",
        choices=("fresh", "fixed"),
        default=None,
        help="Override random_start_train.pool_mode.",
    )
    p.add_argument(
        "--pool-size",
        type=int,
        default=None,
        help="Override random_start_train.pool_size (fixed mode only).",
    )
    return p.parse_args()


def _parse_target_q_str(s: str) -> list[float]:
    parts = [p.strip() for p in s.split(",") if p.strip()]
    if len(parts) != 7:
        raise ValueError(
            f"--target-q must have 7 comma-separated floats, got {len(parts)}"
        )
    return [float(p) for p in parts]


# ─── Main ─────────────────────────────────────────────────────────
def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)
    resolve_ros_env(cfg)

    set_seed(int(get(cfg, "seed")))
    device = choose_device(cfg)
    print(f"[TRAIN-RS] device={device} torch={torch.__version__}")

    out_dir = Path(str(get(cfg, "runtime.out_dir")))
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── Resolve fixed target_q (CLI > random_start.* > anchor) ────
    if args.target_q is not None:
        target_q_list = _parse_target_q_str(args.target_q)
        target_src = "CLI --target-q"
    else:
        target_q_list = (
            get(cfg, "random_start_train.fixed_target_q", None)
            or get(cfg, "data_generation.random_start.fixed_target_q", None)
            or get(cfg, "data_generation.anchor_pair.target_q")
        )
        target_src = "config.yaml"
    target_q = np.asarray(target_q_list, dtype=np.float32).reshape(7)
    print(f"[TRAIN-RS] fixed target_q = {target_q.tolist()} (source: {target_src})")

    proj = build_projector(cfg)
    proj.assert_inside("fixed target_q", target_q)

    # ── Build start samplers ──────────────────────────────────────
    rs_cfg = dict(get(cfg, "random_start_train") or {})
    pool_mode = str(args.pool_mode or rs_cfg.get("pool_mode", "fresh"))
    pool_size = int(args.pool_size or rs_cfg.get("pool_size", 16))
    seed_root = int(get(cfg, "seed"))
    train_seed_offset = int(rs_cfg.get("seed_offset", 13127))
    eval_seed_offset = int(rs_cfg.get("eval_seed_offset", 50021))

    train_sampler = StartSampler(
        mode=pool_mode,
        target_q=target_q,
        proj=proj,
        rng_seed=seed_root + train_seed_offset,
        pool_size=pool_size,
        min_distance_rad=float(rs_cfg.get("min_distance_rad", 0.10)),
        max_distance_rad=float(rs_cfg.get("max_distance_rad", 0.50)),
        min_pairwise_dist_rad=float(rs_cfg.get("min_pairwise_dist_rad", 0.05)),
        max_attempts_per_start=int(rs_cfg.get("max_attempts_per_start", 500)),
        fixed_axes=rs_cfg.get("fixed_axes") or [],
        fixed_axes_values=rs_cfg.get("fixed_axes_values") or [],
        shuffle_each_epoch=bool(rs_cfg.get("shuffle_pool_each_epoch", True)),
    )
    if pool_mode == "fixed":
        print(
            f"[TRAIN-RS] training pool: {len(train_sampler.pool)} starts "
            f"(shuffle_each_epoch={train_sampler.shuffle_each_epoch})"
        )
        for i, s in enumerate(train_sampler.pool):
            d = float(np.linalg.norm(s - target_q))
            print(
                f"    train[{i:>2}]: q={[round(float(x), 3) for x in s]}  "
                f"||q-target||={d:.3f}rad"
            )
    else:
        print("[TRAIN-RS] training pool: fresh (resampled per episode)")

    # Held-out eval pool (always fixed; sampled with a different seed).
    eval_pool_size = int(rs_cfg.get("eval_pool_size", 4))
    eval_every = int(rs_cfg.get("eval_every_episodes", 0))
    eval_eps_per_start = int(rs_cfg.get("eval_episodes_per_start", 1))
    eval_pool: list[np.ndarray] = []
    if eval_every > 0 and eval_pool_size > 0:
        eval_pool = sample_eval_pool(
            target_q=target_q,
            proj=proj,
            rng_seed=seed_root + eval_seed_offset,
            pool_size=eval_pool_size,
            min_distance_rad=float(rs_cfg.get("min_distance_rad", 0.10)),
            max_distance_rad=float(rs_cfg.get("max_distance_rad", 0.50)),
            min_pairwise_dist_rad=float(rs_cfg.get("min_pairwise_dist_rad", 0.05)),
            max_attempts_per_start=int(rs_cfg.get("max_attempts_per_start", 500)),
            fixed_axes=rs_cfg.get("fixed_axes") or [],
            fixed_axes_values=rs_cfg.get("fixed_axes_values") or [],
        )
        print(f"[TRAIN-RS] held-out eval pool: {len(eval_pool)} starts")
        for i, s in enumerate(eval_pool):
            d = float(np.linalg.norm(s - target_q))
            print(
                f"    eval[{i:>2}]:  q={[round(float(x), 3) for x in s]}  "
                f"||q-target||={d:.3f}rad"
            )
    else:
        print("[TRAIN-RS] held-out eval disabled")

    # ── Build env and preflight ──────────────────────────────────
    # The env's home_q_right is unused at runtime (we drive to the
    # per-episode start via ``safe_move_to_q``), but we set it to the
    # first sampled start so build_env's logging and any later
    # safe-pose introspection see a sensible value.
    first_start, _ = train_sampler.next_start(1)
    env = build_env(
        cfg,
        start_q=first_start.tolist(),
        target_q=target_q.tolist(),
    )
    preflight(
        env,
        proj,
        first_start.tolist(),
        target_q.tolist(),
        sub_wait_timeout_sec=float(
            get(cfg, "runtime.preflight_sub_wait_timeout_sec", 15.0)
        ),
        sub_poll_interval_sec=float(
            get(cfg, "runtime.preflight_sub_poll_interval_sec", 0.5)
        ),
    )

    # ── SAC + warm starts ────────────────────────────────────────
    sac = make_sac(cfg, device)

    sft_ckpt = args.sft_checkpoint
    if sft_ckpt is None:
        candidate = out_dir / str(
            get(cfg, "sft.checkpoint_name", "sft_pretrained.pt")
        )
        sft_ckpt = str(candidate) if candidate.exists() else None
    if sft_ckpt:
        ckpt = torch.load(sft_ckpt, map_location=device)
        sac.actor.load_state_dict(ckpt["model"], strict=False)
        sac.target = copy.deepcopy(sac.actor).to(device)
        for p in sac.target.parameters():
            p.requires_grad_(False)
        print(f"[TRAIN-RS] loaded SFT checkpoint <- {sft_ckpt}")
    else:
        print("[TRAIN-RS] no SFT checkpoint loaded (cold start)")

    # Phase 1 buffer warm start: prefer the random-start buffer if it
    # exists, otherwise fall back to the fixed-pair one.  The CLI flag
    # always wins.
    buf = ReplayBuffer(
        obs_dim=int(get(cfg, "sac.obs_dim")),
        action_dim=int(get(cfg, "sac.action_dim")),
        size=int(get(cfg, "sac.replay_size")),
    )
    if args.phase1_buffer:
        p1_path = Path(args.phase1_buffer)
    else:
        rs_buf = out_dir / str(
            get(
                cfg,
                "data_generation.random_start.buffer_save_name",
                "replay_buffer_phase1_random_start.npz",
            )
        )
        fixed_buf = out_dir / str(
            get(
                cfg,
                "data_generation.buffer_save_name",
                "replay_buffer_phase1.npz",
            )
        )
        p1_path = rs_buf if rs_buf.exists() else fixed_buf
    if p1_path.exists():
        buf.load(p1_path)
        print(f"[TRAIN-RS] loaded Phase 1 buffer <- {p1_path} (n={buf.count})")
    else:
        print(f"[TRAIN-RS][WARN] Phase 1 buffer missing at {p1_path}")

    # ── Training loop ────────────────────────────────────────────
    bs = int(get(cfg, "sac.batch_size"))
    update_after = int(get(cfg, "sac.update_after"))
    updates_per_step = int(get(cfg, "sac.updates_per_env_step"))
    total_episodes = int(get(cfg, "train.total_episodes"))
    save_every = int(get(cfg, "train.save_every_episodes"))
    max_steps = int(get(cfg, "env.override_cfg.max_num_steps"))

    # Moving-window success rate (training, on-policy stochastic).
    success_window = deque(maxlen=int(rs_cfg.get("success_window", 20)))
    # Global step counter currently unused; reserved for time-based
    # logging extensions (e.g. wall-clock learning curves).

    try:
        for ep in range(1, total_episodes + 1):
            start_q_np, pool_idx = train_sampler.next_start(ep)
            d_start = float(np.linalg.norm(start_q_np - target_q))
            obs = safe_move_to_q(env, proj, start_q_np, cfg)

            ep_return = 0.0
            ep_transitions = 0
            success = False
            last_q = start_q_np.copy()
            stats: dict[str, float] = {}
            for _t in range(max_steps):
                raw_action = act(sac, obs, deterministic=False)
                nxt_obs, reward, term, trunc, info, safe_a = safe_step(
                    env, proj, raw_action
                )
                next_obs_np = flatten_obs(nxt_obs)
                done = bool(term or trunc)
                buf.add(obs, safe_a, reward, next_obs_np, done)
                obs = next_obs_np
                ep_transitions += 1
                ep_return += float(reward)
                st = env._controller.get_state().wait()[0]
                last_q = np.asarray(st.right_arm_qpos, dtype=np.float32)

                if buf.count >= update_after:
                    for _ in range(updates_per_step):
                        batch = buf.sample(bs, device)
                        stats = update(sac, batch)

                if info.get("success") and not success:
                    success = True
                if info.get("safe_pause") or done:
                    break

            success_window.append(int(success))
            pos_err = float(np.linalg.norm(last_q - target_q))
            win_rate = (
                sum(success_window) / max(len(success_window), 1)
                if success_window
                else 0.0
            )
            print(
                f"[TRAIN-RS] ep={ep:>3}/{total_episodes} "
                f"pool_idx={pool_idx if pool_idx >= 0 else '*':>3} "
                f"d_start={d_start:.3f} "
                f"ret={ep_return:+.3f} steps={ep_transitions:>3} "
                f"succ={int(success)} winN={win_rate:.2f} "
                f"pos_err={pos_err:.3f} buf={buf.count} "
                f"alpha={sac.log_alpha.exp().item():.3f} "
                f"q_loss={stats.get('q_loss', float('nan')):.4f} "
                f"actor_loss={stats.get('actor_loss', float('nan')):.4f}"
            )

            if eval_every > 0 and ep % eval_every == 0 and eval_pool:
                m = evaluate_on_pool(
                    sac=sac,
                    env=env,
                    proj=proj,
                    cfg=cfg,
                    target_q=target_q,
                    eval_starts=eval_pool,
                    episodes_per_start=eval_eps_per_start,
                    max_steps=max_steps,
                )
                print(
                    f"  [EVAL]   ep={ep} "
                    f"succ_rate={m['success_rate']:.2f} "
                    f"mean_ret={m['mean_return']:+.3f} "
                    f"mean_steps={m['mean_steps']:.1f} "
                    f"mean_pos_err={m['mean_pos_err']:.3f}rad"
                )

            if ep % save_every == 0:
                ckpt_path = out_dir / f"sac_random_start_ep{ep}.pt"
                torch.save(
                    {
                        "actor": sac.actor.state_dict(),
                        "target": sac.target.state_dict(),
                        "log_alpha": sac.log_alpha.detach().cpu(),
                        "episode": ep,
                        "buffer_count": buf.count,
                        "target_q": target_q.tolist(),
                        "pool_mode": pool_mode,
                    },
                    ckpt_path,
                )
                print(f"[TRAIN-RS] checkpoint saved -> {ckpt_path}")
    finally:
        try:
            env._controller.apply_brake(True).wait()
        except Exception:
            pass
        try:
            env._controller.shutdown()
        except Exception:
            pass

    final_path = out_dir / "sac_random_start_final.pt"
    torch.save(
        {
            "actor": sac.actor.state_dict(),
            "target": sac.target.state_dict(),
            "log_alpha": sac.log_alpha.detach().cpu(),
            "episode": total_episodes,
            "buffer_count": buf.count,
            "target_q": target_q.tolist(),
            "pool_mode": pool_mode,
        },
        final_path,
    )
    print(f"[TRAIN-RS] final model saved -> {final_path}")


if __name__ == "__main__":
    main()
