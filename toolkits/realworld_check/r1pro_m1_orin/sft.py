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
"""Phase 2 (optional): behaviour-cloning pre-training over Phase 1 buffer.

Reuses ``rlinf.models.embodiment.mlp_policy.MLPPolicy.sft_forward`` directly
(MSE between predicted ``actor_mean(backbone(state))`` and the recorded
``safe_action`` from collect). Saves a checkpoint that ``train_sac`` will
load (``--sft-checkpoint`` or auto-detect by name) to seed the actor before
online RL kicks in.

Loss:

  .. math::
      \\mathcal{L}_{\\mathrm{SFT}} = \\frac{1}{N}\\sum_{i=1}^{N}
        \\bigl\\| \\pi_\\theta(s_i) - a_i \\bigr\\|_2^2
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch

from .runtime import (
    ForwardType,
    MLPPolicy,
    choose_device,
    get,
    load_yaml,
    set_seed,
)


def make_policy(cfg: dict, device: torch.device) -> MLPPolicy:
    """Same MLPPolicy shape used by SAC, so SFT weights load directly."""
    return MLPPolicy(
        obs_dim=int(get(cfg, "sac.obs_dim")),
        action_dim=int(get(cfg, "sac.action_dim")),
        num_action_chunks=1,
        add_value_head=False,
        add_q_head=True,
        q_head_type="default",
    ).to(device)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="R1 Pro M1 joint reach — Phase 2 (BC pre-training)."
    )
    default_cfg = Path(__file__).with_name("config.yaml")
    p.add_argument("--config", default=str(default_cfg))
    p.add_argument(
        "--buffer",
        default=None,
        help="Override path to Phase 1 .npz buffer (default: from cfg).",
    )
    p.add_argument(
        "--output",
        default=None,
        help="Override SFT checkpoint output path (default: from cfg).",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_yaml(args.config)

    set_seed(int(get(cfg, "seed")))
    device = choose_device(cfg)
    print(f"[SFT] device={device} torch={torch.__version__}")

    out_dir = Path(str(get(cfg, "runtime.out_dir")))
    out_dir.mkdir(parents=True, exist_ok=True)

    buffer_path = Path(args.buffer) if args.buffer else (
        out_dir
        / str(get(cfg, "data_generation.buffer_save_name", "replay_buffer_phase1.npz"))
    )
    if not buffer_path.exists():
        raise FileNotFoundError(
            f"Phase 1 buffer not found at {buffer_path}; run collect first."
        )

    data = np.load(buffer_path)
    obs_all = torch.as_tensor(data["obs"], dtype=torch.float32, device=device)
    act_all = torch.as_tensor(data["act"], dtype=torch.float32, device=device)
    n = obs_all.shape[0]
    if n == 0:
        raise RuntimeError(f"empty buffer at {buffer_path}")
    print(f"[SFT] loaded {n} (obs, act) pairs from {buffer_path}")

    model = make_policy(cfg, device)
    optimizer = torch.optim.Adam(
        model.parameters(), lr=float(get(cfg, "sft.lr"))
    )
    bs = int(get(cfg, "sft.batch_size"))
    epochs = int(get(cfg, "sft.epochs"))

    for epoch in range(1, epochs + 1):
        perm = torch.randperm(n, device=device)
        total_loss = 0.0
        steps = 0
        for i in range(0, n - bs + 1, bs):
            idx = perm[i : i + bs]
            batch_obs = obs_all[idx]
            batch_act = act_all[idx]
            # MLPPolicy.sft_forward returns per-element MSE; mean it.
            loss_per_elem = model.forward(
                forward_type=ForwardType.SFT,
                data={"states": batch_obs, "action": batch_act},
            )
            loss = loss_per_elem.mean()

            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
            total_loss += float(loss.detach())
            steps += 1
        avg = total_loss / max(steps, 1)
        print(f"[SFT] epoch={epoch:>3}/{epochs} avg_mse_loss={avg:.6f}")

    ckpt_path = (
        Path(args.output)
        if args.output
        else out_dir / str(get(cfg, "sft.checkpoint_name", "sft_pretrained.pt"))
    )
    ckpt_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {"model": model.state_dict(), "cfg": cfg, "n_samples": n},
        ckpt_path,
    )
    print(f"[SFT] saved -> {ckpt_path}")


if __name__ == "__main__":
    main()
