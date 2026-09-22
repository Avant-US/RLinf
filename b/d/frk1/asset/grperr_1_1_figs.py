#!/usr/bin/env python3
"""Figures for grperr_1.1.md. Run with a matplotlib-capable interpreter:

    /home/nvidia/miniconda3/envs/fastwam/bin/python grperr_1_1_figs.py
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

ASSET = Path(__file__).resolve().parent
CACHE = ASSET / "grperr_1_1_data.npz"
CLOSE_THR = 0.5
WMAX = 0.08


def main() -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = np.load(CACHE)
    ag = d["demo_ag"]
    wg = d["demo_w"]
    ei = d["demo_ei"]
    fi = d["demo_fi"]
    arm = d["demo_arm"]
    l2a = d["l2a_action"]
    l2a_s = d["l2a_state"]
    l2b = d["l2b_action"]
    dry = d["dry_action"]
    chunks = d["l2a_chunks"]
    delay = d["delay_in_chunk"]
    hist_bins = d["hist_bins"]
    hist_counts = d["hist_counts"]

    # ── Fig 1: gripper command, demo vs L2 vs dry-run ─────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(12.4, 3.8), sharey=True)
    for e in np.unique(ei):
        m = ei == e
        axes[0].plot(fi[m] / 30.0, ag[m], lw=1.0, alpha=0.75)
    axes[0].axhline(CLOSE_THR, color="k", ls="--", lw=1)
    axes[0].set_title("Demonstrations (8 eps, 30 Hz)")
    axes[0].set_xlabel("episode time [s]")
    axes[0].set_ylabel("action.gripper  (1 = closed)")
    axes[0].grid(alpha=0.25)
    axes[0].set_ylim(-0.05, 1.08)

    axes[1].plot(np.arange(len(l2a)), l2a[:, 7], lw=1.4, color="#d62728", label="L2 run A")
    axes[1].plot(np.arange(len(l2b)), l2b[:, 7], lw=1.0, color="#ff7f0e", alpha=0.8, label="L2 run B")
    axes[1].axhline(CLOSE_THR, color="k", ls="--", lw=1, label="binary close 0.5")
    axes[1].axhline(0.18, color="#2ca02c", ls=":", lw=1.2, label="ramp-start 0.18")
    axes[1].set_title("On-robot after A–F (300 steps, ~3.6 Hz)")
    axes[1].set_xlabel("control step")
    axes[1].legend(fontsize=7)
    axes[1].grid(alpha=0.25)

    axes[2].plot(np.arange(len(dry)), dry[:, 7], lw=1.2, color="#9467bd")
    axes[2].axhline(CLOSE_THR, color="k", ls="--", lw=1)
    axes[2].set_title("Dry-run, black images, frozen HOME")
    axes[2].set_xlabel("control step")
    axes[2].grid(alpha=0.25)
    fig.suptitle("The policy can emit close (dry-run 0.74) but stays at 0.03–0.23 on real images")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(ASSET / "fig1_1_grip_cmd.png", dpi=150)
    plt.close(fig)

    # ── Fig 2: width vs action (closed-loop ramp) + eval overlay ──────────
    fig, ax = plt.subplots(figsize=(7.2, 5.0))
    ax.scatter(wg * 1000, ag, s=6, alpha=0.18, c="#1f77b4", label="demo frames")
    w_line = np.linspace(0, 0.08, 50)
    ax.plot(w_line * 1000, 1 - w_line / WMAX, color="k", lw=1.2, label=r"$a = 1 - w/0.08$")
    ax.axhline(CLOSE_THR, color="k", ls="--", lw=1)
    ax.axvline(66.4, color="#d62728", ls="--", lw=1.4, label="eval hardware open 66.4 mm")
    ax.scatter(
        [66.4], [0.227], s=80, c="#d62728", zorder=5, marker="*",
        label="eval peak action 0.227 (still at 66.4 mm)",
    )
    ax.scatter(
        [66.4], [0.035], s=40, c="#ff7f0e", zorder=5,
        label="eval early-step action ~0.035",
    )
    ax.set_xlabel("observed gripper width w [mm]")
    ax.set_ylabel("action.gripper")
    ax.set_title("0.5 is a mid-ramp command: demos only emit it after w already fell to ~38 mm")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(alpha=0.25)
    ax.set_xlim(-2, 85)
    fig.tight_layout()
    fig.savefig(ASSET / "fig1_1_width_vs_action.png", dpi=150)
    plt.close(fig)

    # ── Fig 3: receding-horizon delay of close inside the 50-step chunk ───
    fig, ax = plt.subplots(figsize=(7.4, 3.8))
    ax.hist(delay, bins=np.arange(0, 51, 1), color="#4c78a8", edgecolor="white")
    ax.axvline(10, color="#d62728", lw=2, label="n_exec = 10 (executed prefix)")
    ax.set_xlabel("first chunk index where demo action.gripper ≥ 0.5")
    ax.set_ylabel("count (pre-grasp frames, 8 episodes)")
    ax.set_title("80% of pre-grasp 50-step plans put the close AFTER the executed prefix")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25, axis="y")
    fig.tight_layout()
    fig.savefig(ASSET / "fig1_1_receding_horizon.png", dpi=150)
    plt.close(fig)

    # ── Fig 4: within-chunk gripper by slot ────────────────────────────────
    fig, ax = plt.subplots(figsize=(7.4, 3.8))
    g = chunks[:, :, 7]
    valid = ~np.isnan(g[:, 0])
    g = g[valid]
    slots = np.arange(10)
    ax.plot(slots, np.nanmean(g, axis=0), "o-", color="#1f77b4", label="mean over 30 requests")
    ax.fill_between(
        slots, np.nanmin(g, axis=0), np.nanmax(g, axis=0),
        color="#1f77b4", alpha=0.18, label="min–max",
    )
    ax.axhline(CLOSE_THR, color="k", ls="--", lw=1)
    ax.set_xlabel("index inside the executed 10-step prefix of the 50-step chunk")
    ax.set_ylabel("action.gripper")
    ax.set_title("Close intent grows along the chunk — and we throw away steps 10–49")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)
    ax.set_ylim(0, 1.05)
    fig.tight_layout()
    fig.savefig(ASSET / "fig1_1_chunk_slots.png", dpi=150)
    plt.close(fig)

    # ── Fig 5: q6 / q7 vs demo grasp ───────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.8))
    close_m = ag >= 0.5
    axes[0].hist(arm[close_m, 5], bins=20, color="#2ca02c", alpha=0.7, label="demo frames with a≥0.5")
    axes[0].axvline(l2a_s[-1, 5], color="#d62728", lw=2, label=f"eval end q6={l2a_s[-1,5]:.3f}")
    axes[0].set_xlabel("q6 [rad]")
    axes[0].set_ylabel("count")
    axes[0].set_title("Wrist flexion (q6) at grasp")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.25, axis="y")

    q7_lo = arm[:, 6].min()
    axes[1].hist(arm[close_m, 6], bins=20, color="#2ca02c", alpha=0.7, label="demo frames with a≥0.5")
    axes[1].axvline(q7_lo, color="#2ca02c", ls="--", lw=1.2, label=f"demo q7 min {q7_lo:.3f}")
    axes[1].axvline(0.4344, color="#d62728", lw=2, label="eval q7 clipped 0.434")
    axes[1].axvline(np.nanmedian(l2a[:, 6]), color="#ff7f0e", ls=":", lw=1.6,
                    label=f"eval q7 command median {np.nanmedian(l2a[:,6]):.3f}")
    axes[1].set_xlabel("q7 [rad]")
    axes[1].set_title("Wrist roll (q7) at grasp")
    axes[1].legend(fontsize=7)
    axes[1].grid(alpha=0.25, axis="y")
    fig.tight_layout()
    fig.savefig(ASSET / "fig1_1_q6q7.png", dpi=150)
    plt.close(fig)

    # ── Fig 6: action histogram (bimodal) ──────────────────────────────────
    fig, ax = plt.subplots(figsize=(7.2, 3.6))
    centers = 0.5 * (hist_bins[:-1] + hist_bins[1:])
    ax.bar(centers, hist_counts, width=0.045, color="#4c78a8", edgecolor="white")
    ax.axvline(CLOSE_THR, color="k", ls="--", lw=1, label="binary threshold 0.5")
    ax.axvline(0.227, color="#d62728", lw=2, label="eval peak 0.227")
    ax.set_xlabel("action.gripper")
    ax.set_ylabel("demo frames")
    ax.set_title("Gripper action is bimodal (open ~0.02 vs closed ~1.0); 0.23 sits in the sparse valley")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25, axis="y")
    fig.tight_layout()
    fig.savefig(ASSET / "fig1_1_bimodal.png", dpi=150)
    plt.close(fig)

    print("wrote fig1_1_*.png")


if __name__ == "__main__":
    main()
