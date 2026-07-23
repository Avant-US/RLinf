#!/usr/bin/env python3
"""Draw ThinkAct asynchronous slow/fast control (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

OUT = Path(__file__).resolve().parent / "fig_thinkact_async.png"


def box(ax, xy, w, h, text, fc="#E8F1FB", ec="#2F5D8C", fs=9):
    x, y = xy
    p = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.02,rounding_size=0.06",
        linewidth=1.4,
        facecolor=fc,
        edgecolor=ec,
    )
    ax.add_patch(p)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs)


def arrow(ax, start, end, color="#333333", lw=1.4):
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=11,
            linewidth=lw,
            color=color,
        )
    )


def main():
    fig, ax = plt.subplots(figsize=(11.8, 4.8))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 5)
    ax.axis("off")
    ax.set_title(
        "Asynchronous Reasoning: One Visual Plan Latent Covers N Control Steps",
        fontsize=12,
        pad=8,
    )

    # Slow track
    box(ax, (0.3, 3.3), 2.4, 1.1, "Slow: MLLM\nreason @ t", fc="#E1BEE7", ec="#6A1B9A")
    box(ax, (3.0, 3.3), 2.6, 1.1, "Visual plan\nlatent c_t", fc="#CE93D8", ec="#6A1B9A")
    ax.add_patch(Rectangle((5.9, 3.45), 5.6, 0.8, facecolor="#F3E5F5", edgecolor="#6A1B9A", lw=1.3))
    ax.text(8.7, 3.85, "Reuse same c_t for i in [t, t+N)", ha="center", va="center", fontsize=9)

    arrow(ax, (2.7, 3.85), (3.0, 3.85))
    arrow(ax, (5.6, 3.85), (5.9, 3.85))

    # Fast track
    box(ax, (0.3, 1.1), 2.4, 1.1, "Fast: DiT\ncontrol loop", fc="#BBDEFB", ec="#1565C0")
    xs = [3.0, 4.5, 6.0, 7.5, 9.0]
    labels = ["a_t", "a_t+1", "...", "a_t+N-2", "a_t+N-1"]
    for x, lab in zip(xs, labels):
        box(ax, (x, 1.15), 1.2, 1.0, lab, fc="#90CAF9", ec="#1565C0", fs=8)
        if x > xs[0]:
            arrow(ax, (x - 0.15, 1.65), (x, 1.65), color="#1565C0", lw=1.1)

    # Condition arrows from c_t band to actions
    for x in xs:
        arrow(ax, (x + 0.6, 3.45), (x + 0.6, 2.15), color="#F9A825", lw=1.0)

    ax.text(
        0.3,
        0.35,
        "Paper defaults: N=15 (SimplerEnv), N=75 (LIBERO). Enables slow thinking + fast control without per-step CoT decode.",
        fontsize=8.5,
        color="#444444",
    )
    ax.text(8.7, 0.85, "condition on c_t", fontsize=8, color="#F57F17", ha="center")
    fig.tight_layout()
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
