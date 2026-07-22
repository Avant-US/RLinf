#!/usr/bin/env python3
"""Generate figures for GRPO VLM loss/gradient chapter.

All figure labels are in English. Outputs PNGs next to this script.
Formulas match RLinf: A = (R - mean) / (std + 1e-6), L = max(-A*r, -A*clip(r)).
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT_DIR = Path(__file__).resolve().parent
EPS = 1e-6
CLIP_EPS = 0.2

# Consistent style
plt.rcParams.update(
    {
        "font.size": 11,
        "axes.titlesize": 13,
        "axes.labelsize": 11,
        "figure.dpi": 140,
        "savefig.dpi": 160,
        "savefig.bbox": "tight",
        "axes.grid": True,
        "grid.alpha": 0.25,
    }
)


def grpo_advantages(rewards: np.ndarray) -> np.ndarray:
    """Match rlinf/algorithms/advantages.py compute_grpo_advantages."""
    r = np.asarray(rewards, dtype=np.float64)
    mu = r.mean()
    sigma = r.std(ddof=1) if r.size > 1 else 0.0  # torch.std default is unbiased
    return (r - mu) / (sigma + EPS)


def clipped_token_loss(advantage: float, ratio: float, clip_eps: float = CLIP_EPS) -> float:
    """Match compute_ppo_actor_loss token term: max(-A*r, -A*clip(r))."""
    clipped = float(np.clip(ratio, 1.0 - clip_eps, 1.0 + clip_eps))
    return max(-advantage * ratio, -advantage * clipped)


def save(fig: plt.Figure, name: str) -> None:
    path = OUT_DIR / name
    fig.savefig(path)
    plt.close(fig)
    print(f"wrote {path}")


def fig_reward_to_adv_bar() -> None:
    rewards = np.array([1.0, 0.0, 1.0, 0.0])
    adv = grpo_advantages(rewards)
    labels = [f"resp-{i}" for i in range(1, 5)]
    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.bar(x - width / 2, rewards, width, label="Reward R", color="#4C78A8")
    ax.bar(x + width / 2, adv, width, label="Advantage A (GRPO)", color="#F58518")
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Value")
    ax.set_title("Same VQA Prompt, G=4: Reward vs GRPO Advantage")
    ax.legend(loc="upper right")
    mu, sigma = rewards.mean(), rewards.std(ddof=1)
    ax.text(
        0.02,
        0.95,
        f"mu={mu:.2f}, sigma={sigma:.3f}\nA=(R-mu)/(sigma+1e-6)",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )
    save(fig, "fig_reward_to_adv_bar.png")


def fig_reward_scenarios_adv() -> None:
    scenarios = {
        "0/1 baseline\n[1,0,1,0]": [1, 0, 1, 0],
        "shifted+scaled\n[2,1,2,1]": [2, 1, 2, 1],
        "high contrast\n[1,0,0,0]": [1, 0, 0, 0],
        "all correct\n[1,1,1,1]": [1, 1, 1, 1],
        "all wrong\n[0,0,0,0]": [0, 0, 0, 0],
        "signed continuous\n[0.8,-0.2,0.3,-0.5]": [0.8, -0.2, 0.3, -0.5],
        "translate +2\n[3,2,3,2]": [3, 2, 3, 2],
    }
    names = list(scenarios.keys())
    adv_mat = np.stack([grpo_advantages(np.array(v, dtype=float)) for v in scenarios.values()])

    fig, ax = plt.subplots(figsize=(10.5, 4.8))
    im = ax.imshow(adv_mat, cmap="RdBu_r", aspect="auto", vmin=-2.0, vmax=2.0)
    ax.set_xticks(range(4))
    ax.set_xticklabels([f"resp-{i}" for i in range(1, 5)])
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names)
    ax.set_title("How Reward Scenarios Change GRPO Advantage A")
    for i in range(adv_mat.shape[0]):
        for j in range(adv_mat.shape[1]):
            ax.text(j, i, f"{adv_mat[i, j]:.2f}", ha="center", va="center", color="black", fontsize=9)
    cbar = fig.colorbar(im, ax=ax, fraction=0.03, pad=0.02)
    cbar.set_label("Advantage A")
    ax.text(
        0.0,
        -0.18,
        "Note: [1,0,1,0], [2,1,2,1], and [3,2,3,2] share nearly identical A "
        "(GRPO is relative; shift/scale invariance).",
        transform=ax.transAxes,
        fontsize=9,
    )
    save(fig, "fig_reward_scenarios_adv.png")


def fig_adv_to_token_loss() -> None:
    # Fix ratio near 1 (on-policy), and also a drifted ratio to show clip.
    ratios = np.array([0.7, 0.85, 1.0, 1.15, 1.4])
    pos_a = 1.0
    neg_a = -1.0

    loss_pos = [clipped_token_loss(pos_a, r) for r in ratios]
    loss_neg = [clipped_token_loss(neg_a, r) for r in ratios]
    loss_pos_unclip = [-pos_a * r for r in ratios]
    loss_neg_unclip = [-neg_a * r for r in ratios]

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.2), sharey=False)

    ax = axes[0]
    ax.plot(ratios, loss_pos_unclip, "--", color="#4C78A8", label="A=+1 unclipped")
    ax.plot(ratios, loss_pos, "-o", color="#4C78A8", label="A=+1 clipped")
    ax.plot(ratios, loss_neg_unclip, "--", color="#E45756", label="A=-1 unclipped")
    ax.plot(ratios, loss_neg, "-o", color="#E45756", label="A=-1 clipped")
    ax.axvline(1.0, color="gray", linestyle=":", linewidth=1)
    ax.axvline(1.0 - CLIP_EPS, color="gray", linestyle=":", linewidth=0.8)
    ax.axvline(1.0 + CLIP_EPS, color="gray", linestyle=":", linewidth=0.8)
    ax.set_xlabel("Importance ratio r = exp(logpi - logpi_old)")
    ax.set_ylabel("Token policy loss L_t")
    ax.set_title("Positive A encourages; Negative A suppresses")
    ax.legend(fontsize=8)

    ax = axes[1]
    # Per-response loss at r=1: L = -A (since clip unused)
    rewards = np.array([1.0, 0.0, 1.0, 0.0])
    adv = grpo_advantages(rewards)
    token_loss = -adv  # at ratio=1
    labels = [f"resp-{i}" for i in range(1, 5)]
    colors = ["#4C78A8" if a > 0 else "#E45756" for a in adv]
    ax.bar(labels, token_loss, color=colors)
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_ylabel("L_t at r=1 (= -A)")
    ax.set_title("Per-response token loss (shared A over tokens)")
    for i, (a, lt) in enumerate(zip(adv, token_loss)):
        ax.text(i, lt + (0.05 if lt >= 0 else -0.12), f"A={a:.2f}", ha="center", fontsize=9)

    fig.suptitle("From Advantage A to Token Loss (GRPO = clipped PG)", y=1.02)
    save(fig, "fig_adv_to_token_loss.png")


def fig_vlm_grad_flow() -> None:
    fig, ax = plt.subplots(figsize=(10.0, 5.2))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6)
    ax.axis("off")
    ax.set_title("Gradient Flow into QwenVL under GRPO (conceptual)", pad=12)

    boxes = [
        (0.3, 4.2, 2.2, 1.1, "scalar L_actor\n(token-mean)", "#E45756"),
        (3.0, 4.2, 2.4, 1.1, "response logprobs\n& logits", "#F58518"),
        (5.8, 4.2, 2.0, 1.1, "LLM layers\n(+ LM head)", "#54A24B"),
        (8.2, 4.2, 1.5, 1.1, "text\nembeds", "#4C78A8"),
        (5.8, 1.6, 2.0, 1.1, "vision projector", "#72B7B2"),
        (8.2, 1.6, 1.5, 1.1, "vision\nencoder", "#B279A2"),
        (0.3, 1.6, 3.5, 1.1, "Only response tokens have\ndirect PG loss (mask=1)", "#FF9DA6"),
        (0.3, 0.2, 9.4, 0.9, "Image / prompt tokens: no direct CE; indirect grad via attention to response logits", "#BAB0AC"),
    ]
    for x, y, w, h, text, color in boxes:
        patch = FancyBboxPatch(
            (x, y),
            w,
            h,
            boxstyle="round,pad=0.02,rounding_size=0.15",
            facecolor=color,
            edgecolor="black",
            alpha=0.55,
            linewidth=1.2,
        )
        ax.add_patch(patch)
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=9)

    arrows = [
        ((2.5, 4.75), (3.0, 4.75)),
        ((5.4, 4.75), (5.8, 4.75)),
        ((7.8, 4.75), (8.2, 4.75)),
        ((6.8, 4.2), (6.8, 2.7)),
        ((7.8, 2.15), (8.2, 2.15)),
        ((2.1, 4.2), (2.1, 2.7)),
    ]
    for (x1, y1), (x2, y2) in arrows:
        ax.add_patch(
            FancyArrowPatch(
                (x1, y1),
                (x2, y2),
                arrowstyle="-|>",
                mutation_scale=14,
                linewidth=1.5,
                color="black",
            )
        )
    ax.text(7.1, 3.35, "indirect", fontsize=8, style="italic")
    save(fig, "fig_vlm_grad_flow.png")


def fig_group_collapse() -> None:
    cases = {
        "diverse\n[1,0,1,0]": [1, 0, 1, 0],
        "all correct\n[1,1,1,1]": [1, 1, 1, 1],
        "all wrong\n[0,0,0,0]": [0, 0, 0, 0],
    }
    fig, axes = plt.subplots(1, 3, figsize=(10.8, 3.8), sharey=True)
    for ax, (name, rewards) in zip(axes, cases.items()):
        r = np.array(rewards, dtype=float)
        a = grpo_advantages(r)
        # Proxy "gradient magnitude" ~ mean |A| (at r=1, |dL/dlogpi| ~ |A|)
        grad_mag = np.abs(a).mean()
        x = np.arange(4)
        ax.bar(x, a, color=["#4C78A8" if v >= 0 else "#E45756" for v in a])
        ax.axhline(0, color="black", linewidth=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels([f"r{i+1}" for i in range(4)])
        ax.set_title(name)
        ax.set_ylabel("Advantage A" if ax is axes[0] else "")
        ax.text(
            0.5,
            0.92,
            f"mean|A|={grad_mag:.3f}",
            transform=ax.transAxes,
            ha="center",
            va="top",
            fontsize=10,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
        )
    fig.suptitle("Group Collapse: identical rewards => A~0 => almost no VLM gradient", y=1.05)
    save(fig, "fig_group_collapse.png")


def fig_vlm_loss_pipeline() -> None:
    """Extra pipeline figure requested by plan item 9.2."""
    fig, ax = plt.subplots(figsize=(11.0, 3.6))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 3)
    ax.axis("off")
    ax.set_title("VLM Loss Pipeline under GRPO (teacher-forcing, not generation)", pad=8)

    steps = [
        (0.2, "1. Vision2Seq\nforward\n(logits)", "#4C78A8"),
        (2.5, "2. slice response\n+ logprob\n(-CE)", "#72B7B2"),
        (4.8, "3. ratio &\nclipped PG\nwith A", "#F58518"),
        (7.1, "4. token-mean\nscalar loss", "#E45756"),
        (9.4, "5. backward\nto VLM", "#B279A2"),
    ]
    for x, text, color in steps:
        patch = FancyBboxPatch(
            (x, 0.7),
            2.0,
            1.8,
            boxstyle="round,pad=0.02,rounding_size=0.12",
            facecolor=color,
            edgecolor="black",
            alpha=0.55,
        )
        ax.add_patch(patch)
        ax.text(x + 1.0, 1.6, text, ha="center", va="center", fontsize=9)
    for x in [2.2, 4.5, 6.8, 9.1]:
        ax.add_patch(
            FancyArrowPatch(
                (x, 1.6),
                (x + 0.3, 1.6),
                arrowstyle="-|>",
                mutation_scale=12,
                color="black",
                linewidth=1.4,
            )
        )
    ax.text(
        6.0,
        0.25,
        "A is broadcast to every response token; prompt/image tokens are masked out of PG.",
        ha="center",
        fontsize=9,
    )
    save(fig, "fig_vlm_loss_pipeline.png")


def main() -> None:
    fig_reward_to_adv_bar()
    fig_reward_scenarios_adv()
    fig_adv_to_token_loss()
    fig_vlm_grad_flow()
    fig_group_collapse()
    fig_vlm_loss_pipeline()
    print("done")


if __name__ == "__main__":
    main()
