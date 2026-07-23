#!/usr/bin/env python3
"""Compare sequence-training methods (schematic English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent / "fig_scst_vs_baselines.png"


def main():
    methods = [
        "Teacher-\nForcing (XE)",
        "MIXER\n(learned baseline)",
        "Actor-Critic\n(value net)",
        "SCST\n(greedy baseline)",
    ]
    # Schematic scores 1-5 for qualitative properties (higher = better for training)
    metrics = {
        "Optimizes true metric": [1, 4, 4, 5],
        "Low gradient variance": [5, 3, 3, 4],
        "No extra critic net": [5, 3, 1, 5],
        "Train/test decode align": [1, 3, 3, 5],
        "Impl. simplicity": [5, 2, 2, 4],
    }

    x = np.arange(len(methods))
    width = 0.15
    colors = ["#4C78A8", "#F58518", "#54A24B", "#E45756", "#B279A2"]

    fig, ax = plt.subplots(figsize=(11, 5.2))
    for i, (name, vals) in enumerate(metrics.items()):
        ax.bar(x + (i - 2) * width, vals, width, label=name, color=colors[i])

    ax.set_ylabel("Schematic score (higher is better)")
    ax.set_ylim(0, 5.8)
    ax.set_xticks(x)
    ax.set_xticklabels(methods, fontsize=9)
    ax.set_title(
        "Sequence Training Methods: Qualitative Comparison\n"
        "(SCST paper lineage; scores are illustrative, not measured)",
        fontsize=12,
    )
    ax.legend(loc="upper right", fontsize=8, ncol=2)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.text(
        0.01,
        -0.18,
        "Paper evidence: SCST > MIXER on CIDEr; CL often unnecessary; "
        "greedy baseline needs only one extra forward pass.",
        transform=ax.transAxes,
        fontsize=8,
        color="#444444",
    )
    fig.tight_layout()
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
