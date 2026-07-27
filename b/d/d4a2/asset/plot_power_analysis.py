"""Statistical power figures for d4a_solutioin_1_c_mvpa.md.

Two panels:
  (a) Minimum detectable difference (MDD) vs number of training seeds, for
      several levels of seed-to-seed standard deviation. Shaded bands mark the
      effect sizes reported in the literature for in-distribution vs
      out-of-distribution geometry gains.
  (b) Wilson 95% CI half-width vs rollout count, at several baseline success
      rates, with the same literature effect sizes overlaid.

The point of the figure: moving the endpoint from ID mean success rate to OOD
robustness buys about one order of magnitude more statistical power than any
realistic increase in compute.

Usage:  python plot_power_analysis.py
Output: mdd_vs_seeds.png  (in the same directory)
"""

from __future__ import annotations

import numpy as np
from scipy import stats
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ALPHA = 0.05
POWER = 0.80
Z_ALPHA = stats.norm.ppf(1 - ALPHA / 2)
Z_BETA = stats.norm.ppf(POWER)

# Literature effect sizes (see doc §0, §2.3).
ID_GAIN = (2.0, 4.0)      # e.g. Spatial Forcing +1.4, ELAN4D LIBERO +0.8, 3D-Mix LIBERO +1.55
OOD_GAIN = (10.0, 50.0)   # e.g. ELAN4D LIBERO-Plus +14.0, 3D-Mix SIMPLER +10.42, real-robot +30~50


def mdd_pp(n_rollout: int, sigma_s_pp: float, k_seeds: np.ndarray, p: float = 0.85) -> np.ndarray:
    """Minimum detectable difference in percentage points.

    Variance of one seed's observed success rate ~ sigma_s^2 + p(1-p)/N.
    Variance of the K-seed mean ~ that, divided by K. Two independent arms.
    """
    sigma_s = sigma_s_pp / 100.0
    var_one_seed = sigma_s**2 + p * (1 - p) / n_rollout
    se_diff = np.sqrt(2.0 * var_one_seed / k_seeds)
    return (Z_ALPHA + Z_BETA) * se_diff * 100.0


def wilson_halfwidth_pp(n: np.ndarray, p: float, z: float = 1.959963985) -> np.ndarray:
    """Half-width of the Wilson score interval, in percentage points."""
    denom = 1.0 + z**2 / n
    half = (z / denom) * np.sqrt(p * (1 - p) / n + z**2 / (4 * n**2))
    return half * 100.0


def main() -> None:
    fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(13.0, 5.2))

    # ---------------- Panel (a): MDD vs seeds ----------------
    k = np.arange(2, 13)
    for sigma_s, style in [(1.0, "-o"), (2.0, "-s"), (3.0, "-^"), (5.0, "-d")]:
        ax_a.plot(
            k,
            mdd_pp(500, sigma_s, k),
            style,
            markersize=5,
            linewidth=1.8,
            label=rf"$\sigma_s$ = {sigma_s:.0f} pp",
        )

    ax_a.axhspan(*ID_GAIN, alpha=0.16, color="tab:red", zorder=0)
    ax_a.axhspan(*OOD_GAIN, alpha=0.16, color="tab:green", zorder=0)

    ax_a.set_yscale("log")
    ax_a.set_xlabel("Number of training seeds per arm (K)")
    ax_a.set_ylabel("Minimum detectable difference (pp)")
    ax_a.set_title(
        "(a) MDD vs seeds\nN = 500 rollouts/seed, p = 0.85, "
        r"$\alpha$ = 0.05, power = 0.80",
        fontsize=10,
    )
    ax_a.set_xticks(k)
    ax_a.set_ylim(1.6, 90)
    ax_a.set_yticks([2, 3, 5, 10, 20, 50])
    ax_a.get_yaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax_a.grid(True, which="both", alpha=0.25, linewidth=0.6)
    ax_a.legend(loc="lower left", fontsize=9, framealpha=0.95)

    # Annotate the decisive comparison: K=3 with sigma_s=2pp.
    mdd_k3 = mdd_pp(500, 2.0, np.array([3]))[0]
    ax_a.annotate(
        f"K=3, $\\sigma_s$=2pp: MDD = {mdd_k3:.1f} pp\ncannot resolve a 3-4 pp ID gain,\nbut easily resolves a 14 pp OOD gain",
        xy=(3, mdd_k3),
        xytext=(4.2, 30.0),
        fontsize=9,
        arrowprops=dict(arrowstyle="->", linewidth=1.1, color="black"),
        bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="gray", alpha=0.92),
    )

    # ---------------- Panel (b): Wilson half-width vs rollouts ----------------
    n = np.logspace(np.log10(25), np.log10(2000), 200)
    for p_base, style in [(0.50, "-"), (0.75, "--"), (0.85, "-."), (0.95, ":")]:
        ax_b.plot(
            n,
            wilson_halfwidth_pp(n, p_base),
            style,
            linewidth=2.0,
            label=f"baseline p = {p_base:.2f}",
        )

    ax_b.axhspan(*ID_GAIN, alpha=0.16, color="tab:red", zorder=0)
    ax_b.axhspan(*OOD_GAIN, alpha=0.16, color="tab:green", zorder=0)

    for n_mark, tag in [(50, "LIBERO\nper-task"), (500, "LIBERO\nper-suite")]:
        ax_b.axvline(n_mark, color="gray", linewidth=0.9, linestyle=":", zorder=0)
        ax_b.text(n_mark * 1.08, 0.95, tag, fontsize=8.5, color="dimgray", va="bottom")

    ax_b.set_xscale("log")
    ax_b.set_yscale("log")
    ax_b.set_ylim(0.85, 90)
    ax_b.set_xlabel("Rollouts per arm (N)")
    ax_b.set_ylabel("Wilson 95% CI half-width (pp)")
    ax_b.set_title(
        "(b) Single-arm uncertainty vs rollout count\n"
        "shaded bands = same literature effect sizes as (a)",
        fontsize=10,
    )
    ax_b.set_xticks([25, 50, 100, 200, 500, 1000, 2000])
    ax_b.set_yticks([1, 2, 5, 10, 20, 50])
    ax_b.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax_b.get_yaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax_b.grid(True, which="both", alpha=0.25, linewidth=0.6)
    ax_b.legend(loc="upper right", fontsize=9, framealpha=0.95)

    band_handles = [
        Line2D([0], [0], color="tab:red", alpha=0.4, linewidth=9,
               label="ID gain reported in literature"),
        Line2D([0], [0], color="tab:green", alpha=0.4, linewidth=9,
               label="OOD gain reported in literature"),
    ]
    fig.legend(
        handles=band_handles,
        loc="lower center",
        ncol=2,
        fontsize=9.5,
        frameon=False,
        bbox_to_anchor=(0.5, -0.005),
    )

    fig.suptitle(
        "Changing the endpoint beats buying compute: "
        "ID gains sit inside the noise floor, OOD gains sit far above it",
        fontsize=12.5,
        y=0.99,
    )
    fig.tight_layout(rect=(0, 0.05, 1, 0.955))
    fig.savefig("mdd_vs_seeds.png", dpi=170)
    print("wrote mdd_vs_seeds.png")

    # Print the tables that appear in the document, so the numbers stay in sync.
    print("\nMDD table (N=500/seed, p=0.85), pp:")
    print("sigma_s |   K=2   K=3   K=5   K=8  K=10")
    for sigma_s in (1.0, 2.0, 3.0, 5.0):
        vals = mdd_pp(500, sigma_s, np.array([2, 3, 5, 8, 10]))
        print(f"  {sigma_s:>4.0f}pp | " + "  ".join(f"{v:5.1f}" for v in vals))

    print("\nWilson 95% half-width (pp):")
    print("     N |  p=.50  p=.80  p=.90  p=.95")
    for n_val in (25, 50, 100, 200, 500, 1000, 2000):
        vals = [wilson_halfwidth_pp(np.array([n_val]), p)[0] for p in (0.50, 0.80, 0.90, 0.95)]
        print(f"{n_val:>6} | " + "  ".join(f"{v:5.1f}" for v in vals))


if __name__ == "__main__":
    main()
