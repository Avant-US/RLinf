#!/usr/bin/env python3
"""Compare ThinkAct / Fast-ThinkAct / HCRS (English labels)."""

from pathlib import Path

import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent / "fig_fastthinkact_vs_hcrs.png"


def main():
    fig, ax = plt.subplots(figsize=(12.2, 5.8))
    ax.axis("off")
    ax.set_title(
        "ThinkAct vs Fast-ThinkAct vs HCRS — Design Comparison",
        fontsize=13,
        pad=10,
    )

    headers = ["Dimension", "ThinkAct", "Fast-ThinkAct", "HCRS"]
    rows = [
        ["Reasoning form", "Long textual CoT", "M latent vectors", "Sampled text CoT z"],
        ["Train signal", "GRPO visual rewards", "Teacher GRPO + distill", "R=-MSE (+fmt/env)"],
        ["Student objective", "N/A (same model)", "L_verb+L_distill+L_ans", "REINFORCE/GRPO"],
        ["Inference tokens", "Long AR text", "M+K compact", "Can be long AR"],
        ["DiT condition", "Q-Former on c_t", "Early KV of spatial", "sg(encode(z))"],
        ["Freeze VLM@IL", "Yes", "Yes", "Stage1 yes; Stage2 no"],
        ["Speed focus", "Capability first", "Up to ~9x faster", "Capability first"],
        ["Discrete bottleneck", "Yes (text CoT)", "No at student infer", "Yes (needed for RF)"],
    ]

    col_w = [0.18, 0.27, 0.28, 0.27]
    x0, y0 = 0.01, 0.88
    row_h = 0.09

    for j, (h, w) in enumerate(zip(headers, col_w)):
        x = x0 + sum(col_w[:j])
        ax.add_patch(
            plt.Rectangle(
                (x, y0),
                w,
                row_h,
                transform=ax.transAxes,
                facecolor="#37474F",
                edgecolor="white",
                linewidth=1.0,
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
            fontsize=9,
            fontweight="bold",
        )

    colors = ["#FAFAFA", "#F3E5F5", "#FFF8E1", "#E3F2FD"]
    for i, row in enumerate(rows):
        y = y0 - (i + 1) * row_h
        for j, (cell, w) in enumerate(zip(row, col_w)):
            x = x0 + sum(col_w[:j])
            fc = colors[j] if j > 0 else ("#FFFFFF" if i % 2 == 0 else "#F5F5F5")
            ax.add_patch(
                plt.Rectangle(
                    (x, y),
                    w,
                    row_h,
                    transform=ax.transAxes,
                    facecolor=fc,
                    edgecolor="white",
                    linewidth=1.0,
                )
            )
            ax.text(
                x + w / 2,
                y + row_h / 2,
                cell,
                transform=ax.transAxes,
                ha="center",
                va="center",
                fontsize=7.8,
            )

    ax.text(
        0.5,
        0.04,
        "Takeaway: Fast-ThinkAct is a deployment compressor for ThinkAct-style reasoning; "
        "HCRS needs discrete CoT for REINFORCE, then can distill like Fast-ThinkAct for speed.",
        transform=ax.transAxes,
        ha="center",
        fontsize=8.5,
        color="#333333",
    )
    fig.savefig(OUT, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
