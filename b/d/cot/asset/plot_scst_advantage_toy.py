#!/usr/bin/env python3
"""Toy SCST advantages: samples vs greedy baseline (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent / "fig_scst_advantage_toy.png"


def main():
    # Toy rewards: lower MSE => higher reward R = -mse
    labels = ["greedy\n(baseline)", "sample-1", "sample-2", "sample-3", "sample-4"]
    mse = np.array([0.42, 0.28, 0.55, 0.35, 0.48])
    R = -mse
    R_hat = R[0]
    A = R - R_hat
    A[0] = 0.0  # baseline itself

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))

    ax = axes[0]
    colors = ["#FFB74D"] + ["#64B5F6"] * 4
    ax.bar(labels, R, color=colors, edgecolor="#333333")
    ax.axhline(R_hat, color="#E65100", linestyle="--", linewidth=1.5, label="R(greedy)")
    ax.set_ylabel("Reward R = -MSE")
    ax.set_title("Toy rewards for CoT-conditioned DiT fit")
    ax.legend(fontsize=8)
    ax.grid(axis="y", linestyle="--", alpha=0.3)

    ax = axes[1]
    a_colors = ["#BDBDBD"] + [
        "#66BB6A" if a > 0 else "#EF5350" for a in A[1:]
    ]
    ax.bar(labels, A, color=a_colors, edgecolor="#333333")
    ax.axhline(0.0, color="#333333", linewidth=1.0)
    ax.set_ylabel("Advantage A = R - R(greedy)")
    ax.set_title("SCST advantages (push up green, suppress red)")
    for i, a in enumerate(A):
        ax.text(i, a + (0.01 if a >= 0 else -0.02), f"{a:+.2f}", ha="center", fontsize=8)
    ax.grid(axis="y", linestyle="--", alpha=0.3)

    fig.suptitle(
        "SCST-style Advantage for HCRS: R = -MSE(z), baseline = greedy CoT",
        fontsize=12,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
