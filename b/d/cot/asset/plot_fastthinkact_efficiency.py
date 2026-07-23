#!/usr/bin/env python3
"""Draw textual CoT vs latent CoT latency tradeoff (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent / "fig_fastthinkact_efficiency.png"


def main():
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11.8, 4.6))

    # Left: schematic latency bars (paper-reported relatives)
    methods = ["ThinkAct-7B", "MolmoAct-7B", "ThinkAct-3B", "Fast-ThinkAct-3B"]
    # Approximate relative latency from paper claims:
    # Fast vs ThinkAct-7B: -89.3% => ~0.107x; vs MolmoAct similar; vs ThinkAct-3B: 7x faster => ~1/7
    latency = [1.0, 0.89, 0.70, 0.10]  # normalized to ThinkAct-7B ~1.0 (illustrative)
    colors = ["#CE93D8", "#90CAF9", "#BA68C8", "#FFD54F"]
    bars = ax0.barh(methods[::-1], latency[::-1], color=colors[::-1], edgecolor="#333333")
    ax0.set_xlabel("Relative inference latency (illustrative, paper: up to -89.3%)")
    ax0.set_title("Reasoning Latency Comparison")
    ax0.set_xlim(0, 1.15)
    for b, v in zip(bars, latency[::-1]):
        ax0.text(v + 0.02, b.get_y() + b.get_height() / 2, f"{v:.2f}x", va="center", fontsize=9)

    # Right: M ablation cartoon (paper: M=6 best; extremes worse)
    M = np.array([1, 6, 30, 100])
    # Qualitative scores normalized around paper narrative (not exact numbers)
    score = np.array([0.86, 1.0, 0.94, 0.90])
    ax1.plot(M, score, "o-", color="#F57F17", linewidth=2, markersize=8)
    ax1.axvline(6, color="#2E7D32", linestyle="--", linewidth=1.2, label="Chosen M=6")
    ax1.set_xscale("log")
    ax1.set_xlabel("Latent reasoning steps M (log scale)")
    ax1.set_ylabel("Relative task score (schematic)")
    ax1.set_title("Ablation: Latent Steps M")
    ax1.set_ylim(0.8, 1.05)
    ax1.legend(fontsize=8, loc="lower right")
    ax1.grid(True, alpha=0.3)

    fig.suptitle(
        "Fast-ThinkAct Efficiency: Compact Latent CoT vs Verbose Text CoT",
        fontsize=12,
        y=1.02,
    )
    fig.tight_layout()
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
