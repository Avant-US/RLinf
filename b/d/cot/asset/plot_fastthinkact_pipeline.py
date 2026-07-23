#!/usr/bin/env python3
"""Draw Fast-ThinkAct teacher-student pipeline (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT = Path(__file__).resolve().parent / "fig_fastthinkact_pipeline.png"


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
    fig, ax = plt.subplots(figsize=(12.4, 6.4))
    ax.set_xlim(0, 12.6)
    ax.set_ylim(0, 6.6)
    ax.axis("off")
    ax.set_title(
        "Fast-ThinkAct Pipeline — Verbalizable Latent Planning",
        fontsize=13,
        pad=10,
    )

    box(ax, (0.2, 5.55), 6.0, 0.7, "Stage A: Teacher GRPO + Student Latent Distill", fc="#F3E5F5", ec="#6A1B9A", fs=10)
    box(ax, (6.5, 5.55), 5.8, 0.7, "Stage B: Freeze student; IL DiT / RDT", fc="#E3F2FD", ec="#1565C0", fs=10)

    # Teacher row
    box(ax, (0.3, 3.7), 2.0, 1.3, "Obs o_t\n+ Instruction l", fc="#F5F5F5", ec="#555555")
    box(ax, (2.5, 3.7), 2.5, 1.3, "Teacher F^T\ntextual CoT\nGRPO (visual r)", fc="#E1BEE7", ec="#6A1B9A")
    box(ax, (5.2, 3.7), 2.3, 1.3, "Pick tau+/tau-\nby advantage A", fc="#CE93D8", ec="#6A1B9A")

    # Student row
    box(ax, (2.5, 1.9), 2.5, 1.3, "Student F_theta\nlatent CoT z\nM continuous vecs", fc="#FFECB3", ec="#F9A825")
    box(ax, (5.2, 1.9), 2.3, 1.3, "Verbalizer V_psi\nDPO-style L_verb", fc="#FFE082", ec="#F9A825")
    box(ax, (7.7, 1.9), 2.4, 1.3, "Spatial tokens\nK waypoints\nL_distill+L_ans", fc="#FFD54F", ec="#F57F17")

    # Action
    box(ax, (7.7, 3.7), 2.4, 1.3, "Plan latent c_t\nfrom early KV\nof spatial toks", fc="#BBDEFB", ec="#1565C0")
    box(ax, (10.3, 2.6), 2.0, 1.6, "DiT / RDT\npi_phi\nL_IL freeze F", fc="#90CAF9", ec="#0D47A1")

    arrow(ax, (2.3, 4.35), (2.5, 4.35))
    arrow(ax, (5.0, 4.35), (5.2, 4.35))
    arrow(ax, (6.35, 3.7), (3.75, 3.2), color="#6A1B9A")
    arrow(ax, (5.0, 2.55), (5.2, 2.55))
    arrow(ax, (7.5, 2.55), (7.7, 2.55))
    arrow(ax, (8.9, 3.2), (8.9, 3.7), color="#1565C0")
    arrow(ax, (10.1, 4.35), (10.3, 3.6))
    arrow(ax, (10.1, 2.55), (10.3, 3.1))

    ax.text(
        0.3,
        0.55,
        "Inference: only student F_theta + pi_phi (M latent steps + K spatial tokens).\n"
        "Verbalizer is train-only (optional for interpretability).",
        fontsize=8.5,
        color="#444444",
    )
    ax.text(0.3, 0.15, "Key: compress long textual CoT into continuous latents; keep action-aligned visual planning.", fontsize=8, color="#444444")
    fig.tight_layout()
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
