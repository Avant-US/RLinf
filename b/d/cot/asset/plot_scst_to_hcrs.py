#!/usr/bin/env python3
"""Map SCST captioning recipe onto CoT+DiT HCRS VLA (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = Path(__file__).resolve().parent / "fig_scst_to_hcrs.png"


def box(ax, xy, w, h, text, fc, ec):
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
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=8.5)


def arrow(ax, a, b, color="#333"):
    ax.add_patch(
        FancyArrowPatch(
            a, b, arrowstyle="-|>", mutation_scale=11, linewidth=1.3, color=color
        )
    )


def main():
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.5))
    for ax in axes:
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.axis("off")

    ax = axes[0]
    ax.set_title("SCST (Image Captioning)", fontsize=12, pad=6)
    box(ax, (0.5, 7.2), 3.2, 1.6, "CNN + LSTM\npolicy p_theta", "#E3F2FD", "#1565C0")
    box(ax, (4.5, 7.6), 2.6, 1.0, "sample w^s", "#BBDEFB", "#1565C0")
    box(ax, (4.5, 6.2), 2.6, 1.0, "greedy w_hat", "#FFE0B2", "#EF6C00")
    box(ax, (7.5, 7.6), 2.2, 1.0, "CIDEr r(w^s)", "#C8E6C9", "#2E7D32")
    box(ax, (7.5, 6.2), 2.2, 1.0, "CIDEr r(w_hat)", "#FFECB3", "#F9A825")
    box(ax, (2.5, 3.5), 5.0, 1.6, "A = r(w^s)-r(w_hat)\nL = -A log p(w^s)", "#F3E5F5", "#6A1B9A")
    box(ax, (2.5, 1.2), 5.0, 1.4, "XE cold-start\nthen SCST (lower LR)", "#ECEFF1", "#546E7A")
    arrow(ax, (3.7, 7.8), (4.5, 8.1))
    arrow(ax, (3.7, 7.5), (4.5, 6.7))
    arrow(ax, (7.1, 8.1), (7.5, 8.1))
    arrow(ax, (7.1, 6.7), (7.5, 6.7))
    arrow(ax, (8.6, 7.6), (6.5, 5.1))
    arrow(ax, (8.6, 6.2), (6.5, 4.8))

    ax = axes[1]
    ax.set_title("HCRS transfer (CoT + DiT VLA)", fontsize=12, pad=6)
    box(ax, (0.4, 7.2), 3.4, 1.6, "QwenVL\nCoT policy pi_VLM", "#E3F2FD", "#1565C0")
    box(ax, (4.3, 7.6), 2.6, 1.0, "sample CoT z^s", "#BBDEFB", "#1565C0")
    box(ax, (4.3, 6.2), 2.6, 1.0, "greedy CoT z_hat", "#FFE0B2", "#EF6C00")
    box(ax, (7.3, 7.5), 2.4, 1.2, "R = -MSE(z)\n(+ format)", "#C8E6C9", "#2E7D32")
    box(ax, (7.3, 5.9), 2.4, 1.2, "R_hat = -MSE(z_hat)", "#FFECB3", "#F9A825")
    box(
        ax,
        (0.5, 3.2),
        4.2,
        1.8,
        "VLM: A = R - R_hat\nL_VLM = -A log pi(z)\n(SCST / GRPO-K)",
        "#F3E5F5",
        "#6A1B9A",
    )
    box(
        ax,
        (5.2, 3.2),
        4.4,
        1.8,
        "DiT: c = stopgrad(encode(z))\nL_DiT = FM-MSE\n(pathwise, not REINFORCE)",
        "#E8F5E9",
        "#1B5E20",
    )
    box(
        ax,
        (1.5, 0.8),
        7.0,
        1.4,
        "Cold-start: CoT SFT (like XE) -> DiT adapt -> joint HCRS Stage2",
        "#ECEFF1",
        "#546E7A",
    )
    arrow(ax, (3.8, 7.9), (4.3, 8.1))
    arrow(ax, (3.8, 7.5), (4.3, 6.7))
    arrow(ax, (6.9, 8.1), (7.3, 8.1))
    arrow(ax, (6.9, 6.7), (7.3, 6.5))
    arrow(ax, (7.3, 7.5), (4.0, 5.0), color="#6A1B9A")
    arrow(ax, (7.3, 6.6), (6.5, 5.0), color="#1B5E20")

    fig.suptitle(
        "Transferring SCST Ideas to CoT+DiT VLA (HCRS)",
        fontsize=13,
        y=0.98,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
