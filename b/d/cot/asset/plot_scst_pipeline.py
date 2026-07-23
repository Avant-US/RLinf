#!/usr/bin/env python3
"""Draw SCST one-step training pipeline (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = Path(__file__).resolve().parent / "fig_scst_pipeline.png"


def box(ax, xy, w, h, text, fc="#E8F1FB", ec="#2F5D8C"):
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
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=9, wrap=True)


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
    fig, ax = plt.subplots(figsize=(11.5, 5.2))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 5.5)
    ax.axis("off")
    ax.set_title(
        "Self-Critical Sequence Training (SCST) — One Training Step",
        fontsize=13,
        pad=8,
    )

    box(ax, (0.3, 3.5), 2.2, 1.2, "Image\n+ CNN encoder", fc="#F5F5F5", ec="#555555")
    box(ax, (3.0, 3.9), 2.4, 1.0, "Sample caption\nw^s ~ p_theta", fc="#DCEEFF")
    box(ax, (3.0, 2.4), 2.4, 1.0, "Greedy caption\nw_hat = argmax p", fc="#FFE8CC")
    box(ax, (6.0, 3.9), 2.2, 1.0, "Reward r(w^s)\ne.g. CIDEr", fc="#DCEEFF")
    box(ax, (6.0, 2.4), 2.2, 1.0, "Baseline r(w_hat)\n(no grad)", fc="#FFE8CC")
    box(ax, (8.7, 3.0), 2.8, 1.4, "Advantage\nA = r(w^s) - r(w_hat)", fc="#E6F6E6", ec="#2E7D32")
    box(
        ax,
        (3.8, 0.4),
        4.6,
        1.2,
        "REINFORCE update\nL = -A * log p_theta(w^s)\n(push up if A>0, suppress if A<0)",
        fc="#F3E5F5",
        ec="#6A1B9A",
    )

    arrow(ax, (2.5, 4.1), (3.0, 4.4))
    arrow(ax, (2.5, 3.9), (3.0, 2.9))
    arrow(ax, (5.4, 4.4), (6.0, 4.4))
    arrow(ax, (5.4, 2.9), (6.0, 2.9))
    arrow(ax, (8.2, 4.4), (8.7, 3.9))
    arrow(ax, (8.2, 2.9), (8.7, 3.5))
    arrow(ax, (10.1, 3.0), (7.5, 1.6))

    ax.text(
        0.3,
        0.35,
        "Key idea: baseline = model's own test-time inference (greedy),\n"
        "so learning is harmonized with decoding and no learned critic is needed.",
        fontsize=8,
        color="#444444",
    )
    fig.tight_layout()
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
