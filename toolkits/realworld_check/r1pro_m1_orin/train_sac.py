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
"""Phase 3: online SAC on a single anchor pair, warm-started by Phase 1.

The script:

  1. Builds the env locked to the anchor (start_q, target_q).
  2. Builds an :class:`MLPPolicy` (actor + double-Q heads, ``q_head_type=default``)
     and a target Q-net via deep-copy.
  3. Optionally loads SFT pre-trained weights (``--sft-checkpoint``).
  4. Pre-fills the replay buffer from the Phase 1 ``.npz`` so SAC immediately
     sees diverse multi-pair transitions plus on-policy anchor-pair data.
  5. Runs ``train.total_episodes`` of single-process online SAC with
     SafetyConfig-projected actions and segmented safe resets.

SAC objectives (per-step):

  Critic:  :math:`\\mathcal{L}_Q = \\tfrac{1}{2}\\sum_i
    \\mathbb{E}[(Q_i(s,a) - y)^2]`,
    :math:`y = r + \\gamma (1-d) \\bigl[\\min_i Q_i^{\\text{tgt}}(s', a')
    - \\alpha \\log \\pi(a'|s')\\bigr]`

  Actor:   :math:`\\mathcal{L}_\\pi = \\mathbb{E}\\bigl[\\alpha
    \\log \\pi(a|s) - \\min_i Q_i(s, a)\\bigr]`

  Alpha:   :math:`\\mathcal{L}_\\alpha = -\\mathbb{E}\\bigl[
    \\log \\alpha (\\log \\pi(a|s) + \\bar{\\mathcal{H}})\\bigr]`
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

from .runtime import (
    ForwardType,
    MLPPolicy,
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


def make_policy(cfg: dict, device: torch.device) -> MLPPolicy:
    return MLPPolicy(
        obs_dim=int(get(cfg, "sac.obs_dim")),
        action_dim=int(get(cfg, "sac.action_dim")),
        num_action_chunks=1,
        add_value_head=False,
        add_q_head=True,
        q_head_type="default",
    ).to(device)


@dataclass
class SAC:
    actor: MLPPolicy
    target: MLPPolicy
    actor_opt: torch.optim.Optimizer
    q_opt: torch.optim.Optimizer
    log_alpha: torch.Tensor
    alpha_opt: torch.optim.Optimizer | None
    gamma: float
    tau: float
    target_entropy: float
    auto_alpha: bool
    device: torch.device


def make_sac(cfg: dict, device: torch.device) -> SAC:
    actor = make_policy(cfg, device)
    target = copy.deepcopy(actor).to(device)
    for p in target.parameters():
        p.requires_grad_(False)

    lr = float(get(cfg, "sac.lr"))
    actor_opt = torch.optim.Adam(
        list(actor.backbone.parameters())
        + list(actor.actor_mean.parameters())
        + list(actor.actor_logstd.parameters()),
        lr=lr,
    )
    q_opt = torch.optim.Adam(actor.q_head.parameters(), lr=lr)

    auto_alpha = bool(get(cfg, "sac.auto_alpha"))
    target_entropy = float(get(cfg, "sac.target_entropy"))
    log_alpha = torch.tensor(
        float(np.log(float(get(cfg, "sac.alpha")))),
        device=device,
        requires_grad=auto_alpha,
    )
    alpha_opt = (
        torch.optim.Adam([log_alpha], lr=lr) if auto_alpha else None
    )
    return SAC(
        actor=actor,
        target=target,
        actor_opt=actor_opt,
        q_opt=q_opt,
        log_alpha=log_alpha,
        alpha_opt=alpha_opt,
        gamma=float(get(cfg, "sac.gamma")),
        tau=float(get(cfg, "sac.tau")),
        target_entropy=target_entropy,
        auto_alpha=auto_alpha,
        device=device,
    )


def act(
    sac: SAC,
    obs_np: np.ndarray,
    *,
    deterministic: bool = False,
) -> np.ndarray:
    sac.actor.eval()
    with torch.no_grad():
        states = torch.as_tensor(
            obs_np, dtype=torch.float32, device=sac.device
        ).unsqueeze(0)
        if deterministic:
            feat = sac.actor.backbone(states)
            mean = sac.actor.actor_mean(feat)
            action = torch.tanh(mean)
        else:
            action, _, _ = sac.actor.forward(
                forward_type=ForwardType.SAC, obs={"states": states}
            )
    sac.actor.train()
    return action.squeeze(0).detach().cpu().numpy().astype(np.float32)


def update(sac: SAC, batch: dict[str, torch.Tensor]) -> dict[str, float]:
    obs = batch["obs"]
    act_b = batch["act"]
    rew = batch["rew"]
    next_obs = batch["next_obs"]
    done = batch["done"]
    alpha = sac.log_alpha.exp().detach()

    # ── Critic update ─────────────────────────────────────────────
    with torch.no_grad():
        next_action, next_logp, _ = sac.target.forward(
            forward_type=ForwardType.SAC, obs={"states": next_obs}
        )
        next_logp_sum = next_logp.sum(dim=-1, keepdim=True)
        target_q_vals = sac.target.forward(
            forward_type=ForwardType.SAC_Q,
            obs={"states": next_obs},
            actions=next_action,
        )
        min_target_q = target_q_vals.min(dim=-1, keepdim=True).values
        target_y = rew + sac.gamma * (1.0 - done) * (
            min_target_q - alpha * next_logp_sum
        )

    q_vals = sac.actor.forward(
        forward_type=ForwardType.SAC_Q,
        obs={"states": obs},
        actions=act_b,
    )
    q1 = q_vals[:, 0:1]
    q2 = q_vals[:, 1:2]
    q_loss = 0.5 * ((q1 - target_y) ** 2 + (q2 - target_y) ** 2).mean()
    sac.q_opt.zero_grad(set_to_none=True)
    q_loss.backward()
    sac.q_opt.step()

    # ── Actor update ──────────────────────────────────────────────
    new_action, new_logp, _ = sac.actor.forward(
        forward_type=ForwardType.SAC, obs={"states": obs}
    )
    new_logp_sum = new_logp.sum(dim=-1, keepdim=True)
    new_q_vals = sac.actor.forward(
        forward_type=ForwardType.SAC_Q,
        obs={"states": obs},
        actions=new_action,
    )
    new_min_q = new_q_vals.min(dim=-1, keepdim=True).values
    actor_loss = (alpha * new_logp_sum - new_min_q).mean()
    sac.actor_opt.zero_grad(set_to_none=True)
    actor_loss.backward()
    sac.actor_opt.step()

    # ── Alpha update ──────────────────────────────────────────────
    alpha_loss_value = 0.0
    if sac.auto_alpha and sac.alpha_opt is not None:
        alpha_loss = -(
            sac.log_alpha
            * (new_logp_sum.detach() + sac.target_entropy)
        ).mean()
        sac.alpha_opt.zero_grad(set_to_none=True)
        alpha_loss.backward()
        sac.alpha_opt.step()
        alpha_loss_value = float(alpha_loss.detach())

    # ── Polyak target update ──────────────────────────────────────
    with torch.no_grad():
        for p, tp in zip(
            sac.actor.parameters(), sac.target.parameters(), strict=True
        ):
            tp.data.mul_(1.0 - sac.tau).add_(sac.tau * p.data)

    return {
        "q_loss": float(q_loss.detach()),
        "actor_loss": float(actor_loss.detach()),
        "alpha": float(alpha.detach()),
        "alpha_loss": alpha_loss_value,
        "logp_mean": float(new_logp_sum.mean().detach()),
    }


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="R1 Pro M1 joint reach — Phase 3 (online SAC)."
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
        help="Path to Phase 1 .npz buffer (default: from cfg).",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)
    resolve_ros_env(cfg)

    set_seed(int(get(cfg, "seed")))
    device = choose_device(cfg)
    print(f"[TRAIN] device={device} torch={torch.__version__}")

    out_dir = Path(str(get(cfg, "runtime.out_dir")))
    out_dir.mkdir(parents=True, exist_ok=True)

    anchor = get(cfg, "data_generation.anchor_pair") or {}
    start_q = anchor.get("start_q")  #@#TODO这是固定起点,我们要改成任意起点都可以到终点的
    target_q = anchor.get("target_q")
    if start_q is None or target_q is None:
        raise ValueError("data_generation.anchor_pair must define start_q and target_q")
    start_q_np = np.asarray(start_q, dtype=np.float32)
    print(f"[TRAIN] anchor start_q={start_q_np.tolist()}")
    print(f"[TRAIN] anchor target_q={list(target_q)}")

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

    sac = make_sac(cfg, device)

    # Optional SFT warm start.
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
        print(f"[TRAIN] loaded SFT checkpoint <- {sft_ckpt}")
    else:
        print("[TRAIN] no SFT checkpoint loaded (cold start)")

    # Phase 1 buffer warm start.
    buf = ReplayBuffer(
        obs_dim=int(get(cfg, "sac.obs_dim")),
        action_dim=int(get(cfg, "sac.action_dim")),
        size=int(get(cfg, "sac.replay_size")),
    )
    p1_path = (
        Path(args.phase1_buffer)
        if args.phase1_buffer
        else out_dir
        / str(
            get(
                cfg,
                "data_generation.buffer_save_name",
                "replay_buffer_phase1.npz",
            )
        )
    )
    if p1_path.exists():
        buf.load(p1_path)
    else:
        print(f"[TRAIN][WARN] Phase 1 buffer missing at {p1_path}")

    bs = int(get(cfg, "sac.batch_size"))
    update_after = int(get(cfg, "sac.update_after"))
    updates_per_step = int(get(cfg, "sac.updates_per_env_step"))
    total_episodes = int(get(cfg, "train.total_episodes"))
    save_every = int(get(cfg, "train.save_every_episodes"))
    max_steps = int(get(cfg, "env.override_cfg.max_num_steps"))

    global_step = 0
    try:
        for ep in range(1, total_episodes + 1):
            obs = safe_move_to_q(env, proj, start_q_np, cfg)
            ep_return = 0.0
            ep_transitions = 0
            success = False
            for t in range(max_steps):
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
                global_step += 1

                if buf.count >= update_after:
                    for _ in range(updates_per_step):
                        batch = buf.sample(bs, device)
                        stats = update(sac, batch)
                else:
                    stats = {}

                if info.get("success") and not success:
                    success = True
                if info.get("safe_pause") or done:
                    break

            print(
                f"[TRAIN] ep={ep:>3}/{total_episodes} "
                f"ret={ep_return:+.3f} steps={ep_transitions:>3} "
                f"success={int(success)} buffer={buf.count} "
                f"alpha={sac.log_alpha.exp().item():.3f} "
                f"q_loss={stats.get('q_loss', float('nan')):.4f} "
                f"actor_loss={stats.get('actor_loss', float('nan')):.4f}"
            )

            if ep % save_every == 0:
                ckpt_path = out_dir / f"sac_ep{ep}.pt"
                torch.save(
                    {
                        "actor": sac.actor.state_dict(),
                        "target": sac.target.state_dict(),
                        "log_alpha": sac.log_alpha.detach().cpu(),
                        "episode": ep,
                        "buffer_count": buf.count,
                    },
                    ckpt_path,
                )
                print(f"[TRAIN] checkpoint saved -> {ckpt_path}")
    finally:
        try:
            env._controller.apply_brake(True).wait()
        except Exception:
            pass
        try:
            env._controller.shutdown()
        except Exception:
            pass

    final_path = out_dir / "sac_final.pt"
    torch.save(
        {
            "actor": sac.actor.state_dict(),
            "target": sac.target.state_dict(),
            "log_alpha": sac.log_alpha.detach().cpu(),
            "episode": total_episodes,
            "buffer_count": buf.count,
        },
        final_path,
    )
    print(f"[TRAIN] final model saved -> {final_path}")


if __name__ == "__main__":
    main()
