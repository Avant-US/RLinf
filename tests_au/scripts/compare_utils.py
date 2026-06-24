"""Pure-numpy comparison utilities for the acceptance pipeline.

No torch/jax dependency — only numpy (+ optional scipy/matplotlib). This keeps the
module importable and unit-testable in any environment.
"""

from __future__ import annotations

import json
import math
import os
from typing import Iterable, Sequence

import numpy as np

# Tolerance table: (metric_name, precision) -> threshold.
# Falls back to per-precision default if a specific name is not listed.
#
# NOTE (empirical, see rlinfpi_accept_1.md §G): openpi-JAX computes pi05 in bfloat16
# regardless of how params are *stored* (the model config dtype is "bfloat16"). The
# JAX self-difference between bf16-stored and fp32-stored params on the 3B model is
# ~4e-2 on v_t — i.e. there is no true-fp32 JAX reference. The PyTorch port matches
# JAX-bf16 to within that intrinsic noise floor (v_t ~3.6e-2 < 4.1e-2 JAX self-noise),
# and the training-relevant `loss` matches to <5e-3. Hence:
#   * `loss` is the PRIMARY tight metric (bf16 < 1e-2).
#   * `v_t` is a secondary diagnostic with a bf16 tolerance set above the noise floor.
#   * fp32 thresholds are diagnostic only; the canonical acceptance precision is bf16.
_TOL_TABLE = {
    ("img_emb", "fp32"): 1e-4,
    ("img_emb", "bf16"): 2e-2,
    ("lang_emb", "fp32"): 1e-4,
    ("lang_emb", "bf16"): 2e-2,
    ("adarms_cond", "fp32"): 1e-4,
    ("adarms_cond", "bf16"): 2e-2,
    ("suffix_out", "fp32"): 1e-3,
    ("suffix_out", "bf16"): 5e-2,
    ("v_t", "fp32"): 1e-1,   # diagnostic: no true-fp32 JAX reference (see note)
    ("v_t", "bf16"): 6e-2,   # above JAX bf16-vs-fp32 self-noise floor (~4.1e-2)
    ("loss", "fp32"): 5e-2,  # diagnostic
    ("loss", "bf16"): 1e-2,  # PRIMARY tight metric
}
_DEFAULT_TOL = {"fp32": 1e-1, "bf16": 2e-2}


def tol(name: str, precision: str) -> float:
    """Return the comparison threshold for a given tap name and precision."""
    if (name, precision) in _TOL_TABLE:
        return _TOL_TABLE[(name, precision)]
    # match by prefix (e.g. "suffix_out[3]" -> "suffix_out")
    base = name.split("[")[0]
    if (base, precision) in _TOL_TABLE:
        return _TOL_TABLE[(base, precision)]
    if precision not in _DEFAULT_TOL:
        raise ValueError(f"Unknown precision `{precision}` (expected fp32/bf16)")
    return _DEFAULT_TOL[precision]


def _as_float_array(x) -> np.ndarray:
    return np.asarray(x, dtype=np.float64)


def max_abs_diff(a, b) -> float:
    """Maximum elementwise absolute difference between two arrays."""
    a = _as_float_array(a)
    b = _as_float_array(b)
    if a.shape != b.shape:
        raise ValueError(f"shape mismatch: {a.shape} vs {b.shape}")
    if a.size == 0:
        return 0.0
    return float(np.max(np.abs(a - b)))


def compare_pointwise(xs, ys, tol_val: float) -> dict:
    """Pointwise comparison: every element within tol_val."""
    xs = _as_float_array(xs)
    ys = _as_float_array(ys)
    diff = max_abs_diff(xs, ys)
    return {"kind": "pointwise", "max_abs_diff": diff, "tol": tol_val, "pass": diff < tol_val}


def compare_relative(x: float, y: float, tol_val: float) -> dict:
    """Relative error |x-y|/max(|y|, eps) < tol_val."""
    x = float(x)
    y = float(y)
    denom = max(abs(y), 1e-12)
    rel = abs(x - y) / denom
    return {"kind": "relative", "rel_err": rel, "x": x, "y": y, "tol": tol_val, "pass": rel < tol_val}


def _rankdata(a: np.ndarray) -> np.ndarray:
    """Average-rank of elements (ties get mean rank). Avoids scipy dependency."""
    a = np.asarray(a, dtype=np.float64)
    order = np.argsort(a, kind="mergesort")
    ranks = np.empty(len(a), dtype=np.float64)
    ranks[order] = np.arange(1, len(a) + 1, dtype=np.float64)
    # handle ties by averaging
    _, inv, counts = np.unique(a, return_inverse=True, return_counts=True)
    sums = np.zeros(len(counts), dtype=np.float64)
    np.add.at(sums, inv, ranks)
    avg = sums / counts
    return avg[inv]


def spearman_corr(xs, ys) -> float:
    """Spearman rank correlation coefficient (no scipy)."""
    xs = _as_float_array(xs)
    ys = _as_float_array(ys)
    if len(xs) != len(ys):
        raise ValueError("length mismatch")
    if len(xs) < 2:
        return 1.0
    rx = _rankdata(xs)
    ry = _rankdata(ys)
    rx = rx - rx.mean()
    ry = ry - ry.mean()
    denom = math.sqrt(float(np.sum(rx * rx)) * float(np.sum(ry * ry)))
    if denom == 0.0:
        return 1.0  # constant series treated as perfectly correlated
    return float(np.sum(rx * ry) / denom)


def compare_spearman(xs, ys, min_corr: float) -> dict:
    """Trend comparison: Spearman correlation >= min_corr."""
    corr = spearman_corr(xs, ys)
    return {"kind": "spearman", "corr": corr, "min_corr": min_corr, "pass": corr >= min_corr}


def compare_magnitude(xs, ys, ratio: Sequence[float]) -> dict:
    """Magnitude comparison: mean(|xs|)/mean(|ys|) within [lo, hi]."""
    xs = _as_float_array(xs)
    ys = _as_float_array(ys)
    lo, hi = float(ratio[0]), float(ratio[1])
    mx = float(np.mean(np.abs(xs)))
    my = float(np.mean(np.abs(ys)))
    if my < 1e-12:
        r = float("inf") if mx >= 1e-12 else 1.0
    else:
        r = mx / my
    return {"kind": "magnitude", "ratio": r, "bounds": [lo, hi], "mean_x": mx, "mean_y": my,
            "pass": lo <= r <= hi}


def make_report(entries: dict) -> dict:
    """Aggregate per-metric entries into a report.

    Each entry may carry a ``blocking`` flag (default True). ``overall_pass`` is True
    iff all *blocking* entries pass; non-blocking entries that fail are reported as
    warnings and do not gate the result (matches rlinfpi_accept_1.md §H judematrix).
    """
    blocking = {k: e for k, e in entries.items() if e.get("blocking", True)}
    overall = all(bool(e.get("pass", False)) for e in blocking.values())
    warnings = [k for k, e in entries.items()
                if not e.get("blocking", True) and not e.get("pass", False)]
    return {"entries": entries, "overall_pass": overall,
            "num_pass": sum(1 for e in entries.values() if e.get("pass")),
            "num_total": len(entries),
            "num_blocking": len(blocking),
            "warnings": warnings}


def dump_report(report: dict, path: str) -> None:
    """Write a report dict to JSON (creates parent dirs)."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump(report, f, indent=2, default=_json_default)


def _json_default(o):
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


def plot_curves(series: dict, path: str, title: str = "acceptance curves") -> bool:
    """Plot named curves (dict of label->1d-array) to `path`. Returns True if saved.

    Gracefully returns False if matplotlib is unavailable.
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    n = len(series)
    fig, axes = plt.subplots(1, max(1, n), figsize=(5 * max(1, n), 4), squeeze=False)
    for ax, (label, arrs) in zip(axes[0], series.items()):
        for sub_label, arr in arrs.items():
            ax.plot(np.asarray(arr), label=sub_label, marker="o", markersize=3)
        ax.set_title(label)
        ax.set_xlabel("step")
        ax.legend()
        ax.grid(True, alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return True


def pretty_print_report(report: dict) -> str:
    """Render a report as a human-readable table string."""
    lines = []
    lines.append(f"{'metric':<24}{'kind':<12}{'value':<16}{'pass':<6}")
    lines.append("-" * 58)
    for name, e in report.get("entries", {}).items():
        kind = e.get("kind", "?")
        if kind == "pointwise":
            val = f"{e['max_abs_diff']:.3e}"
        elif kind == "relative":
            val = f"{e['rel_err']:.3e}"
        elif kind == "spearman":
            val = f"{e['corr']:.3f}"
        elif kind == "magnitude":
            val = f"{e['ratio']:.3f}"
        else:
            val = "?"
        lines.append(f"{name:<24}{kind:<12}{val:<16}{'PASS' if e.get('pass') else 'FAIL':<6}")
    lines.append("-" * 58)
    warnings = report.get("warnings", [])
    if warnings:
        lines.append(f"WARNINGS (non-blocking): {', '.join(warnings)}")
    lines.append(f"OVERALL: {'PASS' if report.get('overall_pass') else 'FAIL'} "
                 f"({report.get('num_pass')}/{report.get('num_total')} pass; "
                 f"{report.get('num_blocking', report.get('num_total'))} blocking)")
    return "\n".join(lines)
