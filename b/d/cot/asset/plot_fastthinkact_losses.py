#!/usr/bin/env python3
"""Draw Fast-ThinkAct student loss composition (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT = Path(__file__).resolve().parent / "fig_fastthinkact_losses.png"


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
    fig, ax = plt.subplots(figsize=(11.8, 5.6))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 5.8)
    ax.axis("off")
    ax.set_title(
        "Fast-ThinkAct Student Objectives (Preference + Visual Distill)",
        fontsize=13,
        pad=8,
    )

    box(ax, (0.3, 3.8), 3.0, 1.4, "Teacher rollouts\ntau+ / tau-\n(best / worst A)", fc="#E1BEE7", ec="#6A1B9A")
    box(ax, (0.3, 1.8), 3.0, 1.4, "Student latents z\n+ spatial tokens\n(s_1..s_K)", fc="#FFECB3", ec="#F9A825")

    box(
        ax,
        (4.0, 4.0),
        3.5,
        1.3,
        "L_verb (DPO-style)\nprefer decode tau+\nover tau- given z",
        fc="#C8E6C9",
        ec="#2E7D32",
    )
    box(
        ax,
        (4.0, 2.3),
        3.5,
        1.3,
        "L_distill\n||h_answer^T - h||^2\ntransfer visual plan",
        fc="#A5D6A7",
        ec="#2E7D32",
    )
    box(
        ax,
        (4.0, 0.5),
        3.5,
        1.3,
        "L_ans\nsum ||MLP(h(s_i))-p_hat||^2\nparallel waypoints",
        fc="#81C784",
        ec="#1B5E20",
    )

    box(
        ax,
        (8.2, 1.8),
        3.5,
        2.6,
        "L_student =\n  L_verb\n+ L_distill\n+ L_ans\n\nThen L_IL on DiT/RDT\n(freeze student F)",
        fc="#FFF9C4",
        ec="#F9A825",
        fs=10,
    )

    arrow(ax, (3.3, 4.5), (4.0, 4.65))
    arrow(ax, (3.3, 2.5), (4.0, 2.95))
    arrow(ax, (3.3, 2.3), (4.0, 1.15))
    arrow(ax, (7.5, 4.65), (8.2, 3.6))
    arrow(ax, (7.5, 2.95), (8.2, 3.2))
    arrow(ax, (7.5, 1.15), (8.2, 2.6))

    ax.text(
        0.3,
        0.12,
        "Ablation: drop L_verb then L_distill -> large drops; naive CoT-SFT underperforms preference distill.",
        fontsize=8,
        color="#444444",
    )
    fig.tight_layout()
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
