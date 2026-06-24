"""Unit tests for tests_au/scripts/compare_utils.py (pure numpy)."""

import sys
from pathlib import Path

import numpy as np
import pytest

_scripts = str(Path(__file__).resolve().parents[1] / "scripts")
if _scripts not in sys.path:
    sys.path.insert(0, _scripts)

import compare_utils as cu


def test_max_abs_diff():
    a = np.array([1.0, 2.0, 3.0])
    b = np.array([1.0, 2.5, 3.0])
    assert cu.max_abs_diff(a, b) == pytest.approx(0.5)


def test_max_abs_diff_shape_mismatch():
    with pytest.raises(ValueError):
        cu.max_abs_diff(np.zeros(3), np.zeros(4))


def test_max_abs_diff_empty():
    assert cu.max_abs_diff(np.array([]), np.array([])) == 0.0


def test_compare_pointwise_pass():
    r = cu.compare_pointwise(np.zeros(5), np.full(5, 1e-6), tol_val=1e-4)
    assert r["pass"] is True
    assert r["max_abs_diff"] == pytest.approx(1e-6)


def test_compare_pointwise_fail():
    r = cu.compare_pointwise(np.zeros(5), np.full(5, 1e-2), tol_val=1e-4)
    assert r["pass"] is False


def test_compare_relative():
    r = cu.compare_relative(1.05, 1.0, tol_val=0.10)
    assert r["pass"] is True
    assert r["rel_err"] == pytest.approx(0.05)
    r2 = cu.compare_relative(1.2, 1.0, tol_val=0.10)
    assert r2["pass"] is False


def test_compare_relative_zero_denominator():
    # y == 0 protected by eps; small x stays finite
    r = cu.compare_relative(0.0, 0.0, tol_val=0.10)
    assert r["pass"] is True


def test_spearman_monotone():
    xs = [1, 2, 3, 4, 5]
    ys = [2, 4, 6, 8, 10]  # perfectly monotone
    assert cu.spearman_corr(xs, ys) == pytest.approx(1.0)


def test_spearman_reverse():
    xs = [1, 2, 3, 4, 5]
    ys = [5, 4, 3, 2, 1]
    assert cu.spearman_corr(xs, ys) == pytest.approx(-1.0)


def test_compare_spearman_pass_fail():
    xs = list(range(10))
    ys = [v + 0.1 * ((-1) ** v) for v in xs]  # mostly increasing
    assert cu.compare_spearman(xs, ys, min_corr=0.8)["pass"] is True
    assert cu.compare_spearman(xs, list(reversed(xs)), min_corr=0.8)["pass"] is False


def test_compare_magnitude():
    r = cu.compare_magnitude(np.full(10, 2.0), np.full(10, 1.0), ratio=(0.5, 2.0))
    assert r["ratio"] == pytest.approx(2.0)
    assert r["pass"] is True
    r2 = cu.compare_magnitude(np.full(10, 10.0), np.full(10, 1.0), ratio=(0.5, 2.0))
    assert r2["pass"] is False


def test_tol_table():
    # loss is the primary tight bf16 metric
    assert cu.tol("loss", "bf16") == 1e-2
    # v_t bf16 tolerance is above the JAX bf16-vs-fp32 noise floor (see §G)
    assert cu.tol("v_t", "bf16") == 6e-2
    # prefix match for indexed taps
    assert cu.tol("suffix_out[3]", "fp32") == 1e-3
    # default fallback per precision
    assert cu.tol("unknown_metric", "bf16") == 2e-2
    with pytest.raises(ValueError):
        cu.tol("v_t", "weird_precision")


def test_make_report_aggregation():
    entries = {
        "a": {"pass": True},
        "b": {"pass": True},
    }
    rep = cu.make_report(entries)
    assert rep["overall_pass"] is True
    assert rep["num_pass"] == 2

    entries["c"] = {"pass": False}
    rep2 = cu.make_report(entries)
    assert rep2["overall_pass"] is False
    assert rep2["num_pass"] == 2
    assert rep2["num_total"] == 3


def test_dump_report_roundtrip(tmp_path):
    import json

    entries = {"v_t": cu.compare_pointwise(np.zeros(3), np.zeros(3), 1e-4)}
    rep = cu.make_report(entries)
    out = tmp_path / "rep.json"
    cu.dump_report(rep, str(out))
    loaded = json.loads(out.read_text())
    assert loaded["overall_pass"] is True


def test_pretty_print_report():
    entries = {"v_t": cu.compare_pointwise(np.zeros(3), np.zeros(3), 1e-4)}
    txt = cu.pretty_print_report(cu.make_report(entries))
    assert "OVERALL: PASS" in txt
    assert "v_t" in txt


def test_plot_curves(tmp_path):
    series = {
        "loss": {"jax": [1.0, 0.8, 0.6], "pt": [1.1, 0.82, 0.61]},
        "lr": {"jax": [1e-5, 2e-5, 3e-5], "pt": [1e-5, 2e-5, 3e-5]},
    }
    out = tmp_path / "curves.png"
    ok = cu.plot_curves(series, str(out), title="test")
    # matplotlib may be unavailable; if saved, file must exist
    if ok:
        assert out.exists()
