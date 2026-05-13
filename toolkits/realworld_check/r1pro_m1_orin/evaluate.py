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
"""Phase 4: deterministic evaluation of a trained SAC actor on the anchor pair.

Loads ``sac_final.pt`` (or any checkpoint passed with ``--checkpoint``),
locks the env to ``data_generation.anchor_pair``, and reports per-episode
success and average return over ``eval.episodes`` rollouts.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch

from .runtime import (
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
from .train_sac import act, make_policy


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="R1 Pro M1 joint reach — Phase 4 (evaluate)."
    )
    default_cfg = Path(__file__).with_name("config.yaml")
    p.add_argument("--config", default=str(default_cfg))
    p.add_argument(
        "--checkpoint",
        default=None,
        help="SAC checkpoint to evaluate (default: <out_dir>/sac_final.pt).",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)
    resolve_ros_env(cfg)

    set_seed(int(get(cfg, "seed")) + 7)
    device = choose_device(cfg)
    print(f"[EVAL] device={device} torch={torch.__version__}")

    out_dir = Path(str(get(cfg, "runtime.out_dir")))
    ckpt_path = (
        Path(args.checkpoint) if args.checkpoint else out_dir / "sac_final.pt"
    )
    if not ckpt_path.exists():
        raise FileNotFoundError(f"checkpoint not found: {ckpt_path}")

    anchor = get(cfg, "data_generation.anchor_pair") or {}
    start_q = anchor.get("start_q")
    target_q = anchor.get("target_q")
    if start_q is None or target_q is None:
        raise ValueError("data_generation.anchor_pair must define start_q and target_q")
    start_q_np = np.asarray(start_q, dtype=np.float32)

    proj = build_projector(cfg)
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

    actor = make_policy(cfg, device)
    state = torch.load(ckpt_path, map_location=device)
    actor.load_state_dict(state["actor"], strict=False)
    actor.eval()

    # ``act`` only needs ``actor``, ``device`` from the SAC dataclass.
    class _S:  # minimal shim used only by ``act``
        pass

    sac_shim = _S()
    sac_shim.actor = actor
    sac_shim.device = device

    n_eval = int(get(cfg, "eval.episodes"))
    max_steps = int(get(cfg, "env.override_cfg.max_num_steps"))
    successes = 0
    returns: list[float] = []

    try:
        for ep in range(1, n_eval + 1):
            obs = safe_move_to_q(env, proj, start_q_np, cfg)
            ep_return = 0.0
            ep_success = False
            ep_steps = 0
            for t in range(max_steps):
                raw_action = act(sac_shim, obs, deterministic=True)
                nxt_obs, reward, term, trunc, info, _ = safe_step(
                    env, proj, raw_action
                )
                obs = flatten_obs(nxt_obs)
                ep_return += float(reward)
                ep_steps += 1
                if info.get("success") and not ep_success:
                    ep_success = True
                if info.get("safe_pause") or term or trunc:
                    break
            successes += int(ep_success)
            returns.append(ep_return)
            print(
                f"[EVAL] ep={ep:>3}/{n_eval} "
                f"return={ep_return:+.3f} steps={ep_steps:>3} "
                f"success={int(ep_success)}"
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

    rate = successes / max(n_eval, 1)
    avg_ret = float(np.mean(returns)) if returns else float("nan")
    print(
        f"\n[EVAL] success_rate={rate * 100:.1f}% "
        f"({successes}/{n_eval}) avg_return={avg_ret:+.3f}"
    )


if __name__ == "__main__":
    main()
