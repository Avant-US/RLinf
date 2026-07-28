"""Figures for d4a_solutioin_1_c_mvpc2.md (MVPC2: InternVLA-A1.5 + 4D geometry).

All in-figure text is English by project convention.
Run:  python make_figures.py
Outputs: candidate_comparison.png, foresight_upgrade.png, marginal_gains.png
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 9,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "figure.dpi": 150,
})

C_PICK = "#1b7837"      # selected
C_ALT = "#4a6fa5"       # alternatives
C_BAD = "#b2182b"       # eliminated / weakness
C_NEU = "#7f7f7f"
OUT = "."


# --------------------------------------------------------------------------
# Figure 1: candidate comparison (radar + LIBERO-Plus dimension breakdown)
# --------------------------------------------------------------------------
def fig_candidates():
    fig = plt.figure(figsize=(13.5, 5.4))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.05, 1.25, 1.0], wspace=0.44)

    # ---- (a) radar over selection axes -----------------------------------
    ax = fig.add_subplot(gs[0, 0], projection="polar")
    axes_lbl = ["Benchmark\nbreadth", "Dual-arm\n3-cam fit", "Training-stack\nopenness",
                "Headroom", "4D\nhackability"]
    n = len(axes_lbl)
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False).tolist()
    ang += ang[:1]

    # scores are this document's assessment (tag [D]), 0-5
    cand = {
        "InternVLA-A1.5 (selected)": ([5, 5, 5, 4, 5], C_PICK, 2.4, 1.0),
        "Kairos 3.1-4B":             ([5, 5, 2, 3, 3], C_ALT, 1.3, 0.85),
        "MolmoAct2":                 ([2, 4, 5, 3, 2], "#d98c00", 1.3, 0.85),
        "RLDX-1":                    ([4, 1, 4, 3, 2], "#7b3294", 1.3, 0.85),
        "MotuBrain":                 ([3, 5, 0, 3, 1], C_BAD, 1.3, 0.7),
        "GEAR-VLA":                  ([2, 4, 0, 2, 2], "#999999", 1.1, 0.6),
    }
    for name, (v, c, lw, a) in cand.items():
        vv = v + v[:1]
        ax.plot(ang, vv, lw=lw, color=c, alpha=a, label=name)
        if "selected" in name:
            ax.fill(ang, vv, color=c, alpha=0.16)
    ax.set_xticks(ang[:-1])
    ax.set_xticklabels(axes_lbl, fontsize=7.2)
    ax.tick_params(axis="x", pad=9)
    ax.set_ylim(0, 5)
    ax.set_yticks([1, 2, 3, 4, 5])
    ax.set_yticklabels(["1", "2", "3", "4", "5"], fontsize=6.5, color="#666")
    ax.set_title("(a) Selection axes (0-5, this document's scoring)",
                 fontsize=9.5, pad=18, weight="bold")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.10),
              fontsize=6.8, ncol=2, frameon=False)

    # ---- (b) LIBERO-Plus per-dimension -----------------------------------
    ax = fig.add_subplot(gs[0, 1])
    dims = ["Camera", "Robot", "Language", "Light", "Background", "Noise", "Layout"]
    internvla = [83.1, 55.1, 86.9, 96.4, 98.2, 95.6, 85.2]
    pi05 = [78.4, 73.6, 80.8, 96.2, 94.1, 89.0, 84.5]
    cosmos = [75.8, 63.3, 81.7, 96.5, 88.9, 92.7, 82.2]

    x = np.arange(len(dims))
    w = 0.27
    ax.bar(x - w, internvla, w, label="InternVLA-A1.5 (84.8)", color=C_PICK)
    ax.bar(x, pi05, w, label="$\\pi_{0.5}$ (84.4)", color=C_ALT)
    ax.bar(x + w, cosmos, w, label="Cosmos-Policy (82.2)", color=C_NEU)

    # highlight the robot gap
    ax.axvspan(0.55, 1.45, color=C_BAD, alpha=0.08)
    ax.annotate("", xy=(1 - w, 56.5), xytext=(1 - w, 72.4),
                arrowprops=dict(arrowstyle="<->", color=C_BAD, lw=1.8))
    ax.annotate("-18.5 pp vs $\\pi_{0.5}$  ...  weakest of the 7 axes",
                xy=(1 - w, 66.0), xytext=(1.75, 112.0),
                color=C_BAD, fontsize=8, weight="bold", ha="left", va="center",
                arrowprops=dict(arrowstyle="->", color=C_BAD, lw=1.3,
                                connectionstyle="arc3,rad=0.28"))

    ax.set_xticks(x)
    ax.set_xticklabels(dims, rotation=25, ha="right", fontsize=8)
    ax.set_ylabel("Success rate (%)")
    ax.set_ylim(0, 122)
    ax.set_title("(b) LIBERO-Plus per-dimension: one structural gap\nhidden by a competitive total",
                 fontsize=9.5, weight="bold")
    ax.legend(fontsize=7.3, loc="upper center", bbox_to_anchor=(0.5, -0.22),
              ncol=3, frameon=False, columnspacing=1.2, handlelength=1.4)

    # ---- (c) hard-constraint elimination ---------------------------------
    ax = fig.add_subplot(gs[0, 2])
    ax.axis("off")
    ax.set_title("(c) Why the single-benchmark leaders\nwere eliminated",
                 fontsize=9.5, weight="bold")

    rows = [
        ("QuoVLA",     "LIBERO-Plus 90.3 (#1)",  "RoboTwin 2.0 only 45.1/58.6", "dual-arm collapse"),
        ("MotuBrain",  "RoboTwin 95.8/96.1 (#1)", "no code, no weights", "H1 + H2 fail"),
        ("Kairos",     "LIBERO-Plus 89.0",       "no training entry script", "H1 fail"),
        ("RLDX-1",     "robot axis 91.8 (#1)",   "no RoboTwin, not 3-cam", "H3 fail"),
        ("GEAR-VLA",   "self-reported 88.7",     "anonymous project page", "H1 + H2 fail"),
    ]
    y = 0.90
    for name, strength, killer, tag in rows:
        ax.add_patch(FancyBboxPatch((0.02, y - 0.125), 0.96, 0.125,
                                    boxstyle="round,pad=0.008",
                                    fc="#f7f7f7", ec="#cccccc", lw=0.8,
                                    transform=ax.transAxes))
        ax.text(0.05, y - 0.030, name, fontsize=8.6, weight="bold",
                transform=ax.transAxes, va="center")
        ax.text(0.05, y - 0.065, strength, fontsize=7.0, color="#2a7a2a",
                transform=ax.transAxes, va="center")
        ax.text(0.05, y - 0.098, "x  " + killer, fontsize=7.0, color=C_BAD,
                transform=ax.transAxes, va="center")
        ax.text(0.96, y - 0.030, tag, fontsize=6.5, color="#888",
                transform=ax.transAxes, va="center", ha="right", style="italic")
        y -= 0.155

    ax.add_patch(FancyBboxPatch((0.02, y - 0.115), 0.96, 0.115,
                                boxstyle="round,pad=0.008",
                                fc="#e6f4ea", ec=C_PICK, lw=1.6,
                                transform=ax.transAxes))
    ax.text(0.05, y - 0.030, "InternVLA-A1.5", fontsize=8.6, weight="bold",
            color=C_PICK, transform=ax.transAxes, va="center")
    ax.text(0.05, y - 0.070, "top tier on all 6 axes; H1-H4 all satisfied",
            fontsize=7.0, color=C_PICK, transform=ax.transAxes, va="center")
    ax.text(0.05, y - 0.098, "broadest, not per-benchmark #1",
            fontsize=6.5, color="#555", transform=ax.transAxes,
            va="center", style="italic")

    fig.suptitle("Starting-point selection under hard constraints: dual-arm + 3 cameras "
                 "+ open training code + open weights",
                 fontsize=11, weight="bold", y=0.99)
    fig.savefig(f"{OUT}/candidate_comparison.png", bbox_inches="tight",
                facecolor="white")
    plt.close(fig)
    print("wrote candidate_comparison.png")


# --------------------------------------------------------------------------
# Figure 2: 2D foresight -> 4D foresight
# --------------------------------------------------------------------------
def fig_foresight():
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(13.5, 5.6))
    for ax in (ax0, ax1):
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.axis("off")

    def box(ax, x, y, w, h, text, fc, ec, fs=8, weight="normal", tc="black"):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.12",
                                    fc=fc, ec=ec, lw=1.5))
        ax.text(x + w / 2, y + h / 2, text, ha="center", va="center",
                fontsize=fs, weight=weight, color=tc)

    def arrow(ax, p1, p2, color="#444", ls="-", lw=1.6, rad=0.0):
        ax.add_patch(FancyArrowPatch(p1, p2, arrowstyle="-|>", mutation_scale=13,
                                     color=color, lw=lw, linestyle=ls,
                                     connectionstyle=f"arc3,rad={rad}"))

    # ---------------- BEFORE ----------------
    ax0.set_title("BEFORE  -  2D RGB foresight (InternVLA-A1.5 as released)",
                  fontsize=10.5, weight="bold", pad=10)
    box(ax0, 0.4, 8.0, 9.2, 1.1, "3 cameras (head / left / right)  +  language  +  proprio state",
        "#eef2f7", "#8fa8c8")
    box(ax0, 0.4, 6.3, 4.2, 1.2, "Qwen3.5-2B VLM (MoT)", "#dce6f2", "#4a6fa5", fs=9)
    box(ax0, 5.4, 6.3, 4.2, 1.2, "Unified expert 460M", "#dce6f2", "#4a6fa5", fs=9)
    box(ax0, 0.4, 4.5, 4.2, 1.1, "action queries\n-> flow matching -> $a_{t:t+H}$",
        "#e8f2e8", "#5a9e5a")
    box(ax0, 5.4, 4.5, 4.2, 1.1, "foresight tokens $Q_f$  ->  $Z_f$",
        "#d1ecf1", "#0c5460", fs=9, weight="bold")
    box(ax0, 5.4, 2.6, 4.2, 1.2, "WAN2.2-5B  (frozen)\nnative cross-attention",
        "#fae3e3", "#b2182b")
    box(ax0, 5.4, 0.9, 4.2, 1.0, "$\\mathcal{L}_{video}$ :  RGB only", "#fae3e3", "#b2182b",
        fs=9, weight="bold", tc=C_BAD)

    arrow(ax0, (5.0, 8.0), (2.5, 7.5))
    arrow(ax0, (4.6, 6.9), (5.4, 6.9))
    arrow(ax0, (6.5, 6.3), (6.5, 5.6))
    arrow(ax0, (5.9, 6.3), (2.5, 5.6))
    arrow(ax0, (7.5, 4.5), (7.5, 3.8))
    arrow(ax0, (7.5, 2.6), (7.5, 1.9))
    arrow(ax0, (9.6, 1.9), (9.6, 4.5), color=C_BAD, ls="--", lw=1.4, rad=-0.5)
    ax0.text(2.5, 2.6,
             "WAN is frozen:\ngradient reaches the policy\nONLY through $Z_f$\n"
             "-> $Z_f$ is the single\ninformation bottleneck",
             fontsize=7.6, color=C_BAD, ha="center", va="center")

    ax0.text(5.0, 0.15,
             "$Z_f$ is constrained to encode future APPEARANCE only\n"
             "-> no metric geometry anywhere in the foresight path",
             ha="center", fontsize=8.5, color=C_BAD, style="italic")

    # ---------------- AFTER ----------------
    ax1.set_title("AFTER  -  4D foresight (S1 + S2 + S3)",
                  fontsize=10.5, weight="bold", pad=10)
    box(ax1, 0.4, 8.0, 9.2, 1.1,
        "3 cameras + ray embedding / PRoPE  (S3)  +  language + proprio",
        "#fff6d9", "#d39e00")
    box(ax1, 0.4, 6.3, 4.2, 1.2, "Qwen3.5-2B VLM (MoT)", "#dce6f2", "#4a6fa5", fs=9)
    box(ax1, 5.4, 6.3, 4.2, 1.2, "Unified expert 460M", "#dce6f2", "#4a6fa5", fs=9)
    box(ax1, 0.4, 4.5, 4.2, 1.1, "action queries\n-> flow matching -> $a_{t:t+H}$",
        "#e8f2e8", "#5a9e5a")
    box(ax1, 5.4, 4.5, 4.2, 1.1, "foresight tokens $Q_f$  ->  $Z_f$",
        "#d1ecf1", "#0c5460", fs=9, weight="bold")
    box(ax1, 0.4, 2.6, 4.2, 1.2,
        "S2  point-track head\n$\\widehat{\\Delta P} \\in R^{H \\times N_p \\times 3}$",
        "#d4edda", "#155724", fs=8.5, weight="bold")
    box(ax1, 5.4, 2.6, 4.2, 1.2, "WAN2.2-5B  (frozen)\n+ S1 depth branch",
        "#fae3e3", "#b2182b")
    box(ax1, 5.4, 0.9, 4.2, 1.0, "$\\mathcal{L}_{video}^{4D}$ :  RGB  +  inverse depth",
        "#d1ecf1", "#0c5460", fs=9, weight="bold", tc="#0c5460")
    box(ax1, 0.4, 0.9, 4.2, 1.0, "$\\mathcal{L}_{pt}$ :  3D displacement\n(robot FK + scene pts)",
        "#d4edda", "#155724", fs=8.5, weight="bold", tc="#155724")

    arrow(ax1, (5.0, 8.0), (2.5, 7.5))
    arrow(ax1, (4.6, 6.9), (5.4, 6.9))
    arrow(ax1, (6.5, 6.3), (6.5, 5.6))
    arrow(ax1, (5.9, 6.3), (2.5, 5.6))
    arrow(ax1, (7.5, 4.5), (7.5, 3.8))
    arrow(ax1, (7.5, 2.6), (7.5, 1.9))
    arrow(ax1, (2.5, 2.6), (2.5, 1.9))
    arrow(ax1, (5.6, 4.5), (4.3, 3.8), color="#155724", ls="--", lw=1.5, rad=0.2)
    ax1.text(5.0, 4.12, "zero-init FCCA", fontsize=7, color="#155724",
             ha="center", va="bottom", weight="bold")
    arrow(ax1, (9.6, 1.9), (9.6, 4.5), color="#0c5460", ls="--", lw=1.6, rad=-0.5)

    ax1.text(5.0, 0.15,
             "$Z_f$ must now encode future GEOMETRY as well\n"
             "-> depth, 3D point motion and calibrated multi-view structure",
             ha="center", fontsize=8.5, color="#155724", style="italic", weight="bold")

    fig.suptitle("Core idea: the foresight path is an information bottleneck - "
                 "raise what the frozen teacher must reconstruct, and $Z_f$ must encode it",
                 fontsize=11, weight="bold", y=1.0)
    fig.savefig(f"{OUT}/foresight_upgrade.png", bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote foresight_upgrade.png")


# --------------------------------------------------------------------------
# Figure 3: marginal gains, decay law, headroom
# --------------------------------------------------------------------------
def fig_gains():
    fig = plt.figure(figsize=(13.5, 4.8))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.15, 1.0, 1.1], wspace=0.30)

    # ---- (a) literature vs discounted expectation -------------------------
    ax = fig.add_subplot(gs[0, 0])
    layers = ["S2\n3D point\ntracks", "S1\n4D\nteacher", "S3\ncamera\ngeometry",
              "S4\ntest-time\nimagination", "FK\nsafety\nlayer"]
    lit = [13.2, 4.8, 3.5, 1.8, 0.0]
    neutral = [2.5, 1.5, 1.5, 1.0, 0.5]
    pess = [0.5, 0.0, -1.0, 0.0, 0.0]
    opt = [5.0, 3.0, 3.0, 2.0, 1.5]

    x = np.arange(len(layers))
    ax.bar(x - 0.21, lit, 0.34, label="Best literature single-point",
           color="#c9d6e8", edgecolor="#4a6fa5")
    ax.bar(x + 0.21, neutral, 0.34, label="This document, neutral",
           color=C_PICK, alpha=0.9)
    ax.errorbar(x + 0.21, neutral,
                yerr=[np.array(neutral) - np.array(pess),
                      np.array(opt) - np.array(neutral)],
                fmt="none", ecolor="#333", capsize=3.5, lw=1.2)
    ax.axhline(0, color="#333", lw=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(layers, fontsize=7.5)
    ax.set_ylabel("Absolute gain (pp)")
    ax.set_title("(a) Literature gains are NOT additive\n(bars = discounted, whiskers = pess/opt)",
                 fontsize=9.5, weight="bold")
    ax.legend(fontsize=7.2, loc="upper right")

    # ---- (b) decay law ----------------------------------------------------
    ax = fig.add_subplot(gs[0, 1])
    p = np.linspace(0.40, 0.97, 300)
    alpha = 1.97
    ref_p, ref_d = 0.536, 14.0
    curve = ref_d * ((1 - p) / (1 - ref_p)) ** alpha
    ax.plot(p * 100, curve, color=C_ALT, lw=2.2,
            label="$\\Delta \\propto (1-p_0)^{\\alpha},\\ \\alpha \\approx 1.97$")

    obs = [(53.6, 14.0, "ELAN4D on $\\pi_0$"),
           (73.6, 4.6, "ELAN4D on $\\pi_{0.5}$")]
    for px, py, lb in obs:
        ax.plot(px, py, "o", color=C_BAD, ms=8, zorder=5)
        ax.annotate(lb, (px, py), textcoords="offset points", xytext=(8, 8),
                    fontsize=7.5, color=C_BAD)

    y_start = ref_d * ((1 - 0.848) / (1 - ref_p)) ** alpha
    ax.plot(84.8, y_start, "*", color=C_PICK, ms=17, zorder=6)
    ax.annotate(f"InternVLA-A1.5\n84.8 -> extrapolated\n+{y_start:.1f} pp",
                (84.8, y_start), textcoords="offset points", xytext=(-72, 26),
                fontsize=7.8, color=C_PICK, weight="bold")
    ax.set_xlabel("Baseline success rate $p_0$ (%)")
    ax.set_ylabel("Expected gain (pp)")
    ax.set_title("(b) Gains decay with baseline strength\n(2-point fit; treat as a lower-bound intuition)",
                 fontsize=9.5, weight="bold")
    ax.legend(fontsize=7.5)
    ax.set_ylim(0, 16)

    # ---- (c) remaining headroom per LIBERO-Plus axis ----------------------
    ax = fig.add_subplot(gs[0, 2])
    dims = ["Robot", "Camera", "Layout", "Language", "Noise", "Light", "Background"]
    start = [55.1, 83.1, 85.2, 86.9, 95.6, 96.4, 98.2]
    best = [91.8, 96.6, 89.3, 93.6, 97.2, 98.6, 98.2]
    y = np.arange(len(dims))
    gap = np.array(best) - np.array(start)

    ax.barh(y, start, color="#c9d6e8", edgecolor="#4a6fa5", label="InternVLA-A1.5")
    ax.barh(y, gap, left=start, color=C_BAD, alpha=0.55,
            label="headroom to best known")
    for i, (s, g) in enumerate(zip(start, gap)):
        if g > 1.5:
            ax.text(s + g + 0.8, i, f"+{g:.1f}", va="center", fontsize=7.5,
                    color=C_BAD, weight="bold")
    ax.set_yticks(y)
    ax.set_yticklabels(dims, fontsize=8)
    ax.set_xlim(0, 112)
    ax.set_xlabel("Success rate (%)")
    ax.set_title("(c) Where the room actually is\nrobot axis alone = 36.7 pp",
                 fontsize=9.5, weight="bold")
    ax.legend(fontsize=7.2, loc="lower right")
    ax.invert_yaxis()

    fig.suptitle("Expected returns: discounted, decaying, and concentrated in one axis",
                 fontsize=11, weight="bold", y=1.02)
    fig.savefig(f"{OUT}/marginal_gains.png", bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote marginal_gains.png")


if __name__ == "__main__":
    fig_candidates()
    fig_foresight()
    fig_gains()
    print("done")
