#!/usr/bin/env python3
"""Draw ThinkAct dual-system training pipeline (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT = Path(__file__).resolve().parent / "fig_thinkact_pipeline.png"


def box(ax, xy, w, h, text, fc="#E8F1FB", ec="#2F5D8C", fs=8.5):
    x, y = xy
    p = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.02,rounding_size=0.08",
        linewidth=1.5,
        facecolor=fc,
        edgecolor=ec,
    )
    ax.add_patch(p)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, wrap=True)


def arrow(ax, start, end, color="#333333"):
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=12,
            linewidth=1.4,
            color=color,
        )
    )


def main():
    fig, ax = plt.subplots(figsize=(12.2, 6.2))
    ax.set_xlim(0, 12.5)
    ax.set_ylim(0, 6.4)
    ax.axis("off")
    ax.set_title(
        "ThinkAct Dual-System Pipeline — Reinforced Visual Latent Planning",
        fontsize=13,
        pad=10,
    )

    # Stage banner
    box(ax, (0.2, 5.5), 5.8, 0.7, "Stage A: Reinforce MLLM (GRPO)", fc="#F3E5F5", ec="#6A1B9A", fs=10)
    box(ax, (6.4, 5.5), 5.8, 0.7, "Stage B: Adapt DiT (IL), freeze MLLM", fc="#E3F2FD", ec="#1565C0", fs=10)

    # Stage A flow
    box(ax, (0.3, 3.6), 2.0, 1.3, "Obs o_t\n+ Instruction l", fc="#F5F5F5", ec="#555555")
    box(ax, (2.6, 3.6), 2.4, 1.3, "MLLM F_theta\nsample group\nz_1..z_M", fc="#E1BEE7", ec="#6A1B9A")
    box(ax, (5.2, 3.6), 2.2, 1.3, "Decode plan\ntau = [p_k]\n+ CoT text", fc="#CE93D8", ec="#6A1B9A")
    box(ax, (7.6, 3.6), 2.4, 1.3, "Rewards\nr_goal, r_traj\nr_format", fc="#C8E6C9", ec="#2E7D32")
    box(ax, (10.2, 3.6), 2.0, 1.3, "GRPO update\nA_i group-rel\n+ KL", fc="#A5D6A7", ec="#2E7D32")

    arrow(ax, (2.3, 4.25), (2.6, 4.25))
    arrow(ax, (5.0, 4.25), (5.2, 4.25))
    arrow(ax, (7.4, 4.25), (7.6, 4.25))
    arrow(ax, (10.0, 4.25), (10.2, 4.25))

    # Bridge
    box(
        ax,
        (3.5, 2.35),
        5.5,
        0.85,
        "Compress reasoning into visual plan latent c_t  (cached offline for Stage B)",
        fc="#FFF8E1",
        ec="#F9A825",
        fs=9,
    )
    arrow(ax, (6.3, 3.6), (6.3, 3.2), color="#F9A825")

    # Stage B flow
    box(ax, (0.8, 0.45), 2.2, 1.4, "Frozen MLLM\nc_t = F_theta(o,l)\n(no grad)", fc="#ECEFF1", ec="#546E7A")
    box(ax, (3.4, 0.45), 2.4, 1.4, "Latent projector\n(Q-Former)\n+ state encoder", fc="#BBDEFB", ec="#1565C0")
    box(ax, (6.2, 0.45), 2.6, 1.4, "DiT / Diffusion\nPolicy pi_phi\n~432M", fc="#90CAF9", ec="#1565C0")
    box(ax, (9.2, 0.45), 2.6, 1.4, "IL loss\nL = ell(pi(c_t,o,l), a)\nupdate phi only", fc="#64B5F6", ec="#0D47A1")

    arrow(ax, (3.0, 1.15), (3.4, 1.15))
    arrow(ax, (5.8, 1.15), (6.2, 1.15))
    arrow(ax, (8.8, 1.15), (9.2, 1.15))
    arrow(ax, (6.25, 2.35), (1.9, 1.85), color="#F9A825")

    ax.text(
        0.3,
        0.08,
        "Key: reasoning is optimized with action-aligned visual rewards; control is adapted by conditioning on c_t.",
        fontsize=8,
        color="#444444",
    )
    fig.tight_layout()
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
