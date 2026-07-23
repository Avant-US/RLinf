#!/usr/bin/env python3
"""Compare ThinkAct vs HCRS design choices (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent / "fig_thinkact_vs_hcrs.png"


def main():
    fig, ax = plt.subplots(figsize=(11.8, 5.6))
    ax.axis("off")
    ax.set_title("ThinkAct vs HCRS (CoT + DiT) — Design Comparison", fontsize=13, pad=10)

    headers = ["Dimension", "ThinkAct", "HCRS (proposed)"]
    rows = [
        ["Architecture", "Dual: MLLM + DiT", "Dual: VLM CoT + DiT"],
        ["Reasoning signal", "Sampled CoT -> latent c_t", "Sampled CoT z (stopgrad)"],
        ["Reward for VLM", "r_goal + r_traj + format", "R = -MSE (or env / mix)"],
        ["VLM optimizer", "GRPO (group rel. + KL)", "REINFORCE / GRPO / SCST"],
        ["DiT training", "IL; freeze MLLM", "MSE; can joint-update"],
        ["Bridge", "Q-Former on c_t", "embed / pool / re-prompt"],
        ["Async control", "1 plan covers N steps", "Same idea recommended"],
        ["Cold start", "SFT CoT + traj format", "SFT CoT then RL"],
    ]

    col_w = [0.22, 0.39, 0.39]
    x0, y0 = 0.02, 0.88
    row_h = 0.095

    # header
    x = x0
    for j, (h, w) in enumerate(zip(headers, col_w)):
        ax.add_patch(
            plt.Rectangle(
                (x, y0),
                w,
                row_h,
                transform=ax.transAxes,
                facecolor="#37474F",
                edgecolor="white",
                linewidth=1.2,
            )
        )
        ax.text(
            x + w / 2,
            y0 + row_h / 2,
            h,
            transform=ax.transAxes,
            ha="center",
            va="center",
            color="white",
            fontsize=10,
            fontweight="bold",
        )
        x += w

    colors = ["#ECEFF1", "#F3E5F5", "#E3F2FD"]
    for i, row in enumerate(rows):
        y = y0 - (i + 1) * row_h
        x = x0
        for j, (cell, w) in enumerate(zip(row, col_w)):
            fc = colors[j] if j > 0 else ("#FAFAFA" if i % 2 == 0 else "#F0F0F0")
            ax.add_patch(
                plt.Rectangle(
                    (x, y),
                    w,
                    row_h,
                    transform=ax.transAxes,
                    facecolor=fc,
                    edgecolor="white",
                    linewidth=1.2,
                )
            )
            ax.text(
                x + w / 2,
                y + row_h / 2,
                cell,
                transform=ax.transAxes,
                ha="center",
                va="center",
                fontsize=8.5,
            )
            x += w

    ax.text(
        0.5,
        0.04,
        "Takeaway: reuse ThinkAct dual-system + GRPO scaffolding; replace visual rewards with -MSE when optimizing for action fit.",
        transform=ax.transAxes,
        ha="center",
        fontsize=9,
        color="#333333",
    )
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
