#!/usr/bin/env python3
"""Draw ThinkAct reward composition (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT = Path(__file__).resolve().parent / "fig_thinkact_rewards.png"


def box(ax, xy, w, h, text, fc="#E8F1FB", ec="#2F5D8C", fs=9):
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


def arrow(ax, start, end):
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=12,
            linewidth=1.4,
            color="#333333",
        )
    )


def main():
    fig, ax = plt.subplots(figsize=(11.5, 5.8))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 6)
    ax.axis("off")
    ax.set_title(
        "ThinkAct Action-Aligned Visual Rewards for GRPO",
        fontsize=13,
        pad=8,
    )

    box(ax, (0.3, 4.2), 3.2, 1.3, "Predicted trajectory\ntau = [p_1 .. p_K]\n(K=8 keypoints)", fc="#E1BEE7", ec="#6A1B9A")
    box(ax, (0.3, 2.4), 3.2, 1.3, "Detector GT traj\ntau_hat from LLARVA\n(or hand detector)", fc="#F5F5F5", ec="#555555")

    box(
        ax,
        (4.2, 4.2),
        3.4,
        1.3,
        "Goal reward r_goal\n0.5 * (f(p1,p1_hat)\n+ f(pK,pK_hat))",
        fc="#C8E6C9",
        ec="#2E7D32",
    )
    box(
        ax,
        (4.2, 2.4),
        3.4,
        1.3,
        "Trajectory reward r_traj\nmax(0, 1 - DTW(tau, tau_hat))",
        fc="#A5D6A7",
        ec="#2E7D32",
    )
    box(
        ax,
        (4.2, 0.6),
        3.4,
        1.3,
        "Format reward r_format\n<think>/<answer> tags\n(+ QA accuracy/ROUGE)",
        fc="#FFE0B2",
        ec="#EF6C00",
    )

    box(
        ax,
        (8.2, 2.2),
        3.5,
        2.4,
        "Total reward\n\nr = 0.9 r_visual\n  + 0.1 r_format\n\nr_visual =\n  0.5 r_goal\n+ 0.5 r_traj",
        fc="#FFF9C4",
        ec="#F9A825",
        fs=10,
    )

    arrow(ax, (3.5, 4.85), (4.2, 4.85))
    arrow(ax, (3.5, 3.0), (4.2, 3.0))
    arrow(ax, (1.9, 4.2), (1.9, 3.7))
    arrow(ax, (7.6, 4.85), (8.2, 3.9))
    arrow(ax, (7.6, 3.05), (8.2, 3.4))
    arrow(ax, (7.6, 1.25), (8.2, 2.6))

    ax.text(
        0.3,
        0.15,
        "Ablation: removing goal/traj leaves only QA rewards -> near SFT; action-aligned visual feedback is critical.",
        fontsize=8,
        color="#444444",
    )
    fig.tight_layout()
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
