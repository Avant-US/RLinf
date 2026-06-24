"""Smoke tests for the acceptance pipeline plumbing (no real 3B models / sim)."""

import io
import json
import sys
from pathlib import Path

import numpy as np
import pytest

_scripts = str(Path(__file__).resolve().parents[1] / "scripts")
if _scripts not in sys.path:
    sys.path.insert(0, _scripts)

import compare_utils as cu
import extract_libero_subset as ex


def _make_mini_dataset(root: Path, n_episodes=2, frames=8):
    import pandas as pd
    from PIL import Image

    (root / "meta").mkdir(parents=True, exist_ok=True)
    (root / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
    with open(root / "meta" / "tasks.jsonl", "w") as f:
        f.write(json.dumps({"task_index": 0, "task": "pick up the cube"}) + "\n")

    def _png(seed):
        rng = np.random.default_rng(seed)
        arr = (rng.random((48, 48, 3)) * 255).astype(np.uint8)
        buf = io.BytesIO()
        Image.fromarray(arr).save(buf, format="PNG")
        return buf.getvalue()

    for ep in range(n_episodes):
        rows = [{
            "image": {"bytes": _png(ep * 50 + fi), "path": None},
            "wrist_image": {"bytes": _png(ep * 50 + fi + 25), "path": None},
            "state": np.arange(8, dtype=np.float32),
            "actions": np.arange(7, dtype=np.float32),
            "timestamp": float(fi), "frame_index": fi, "episode_index": ep,
            "index": ep * frames + fi, "task_index": 0,
        } for fi in range(frames)]
        pd.DataFrame(rows).to_parquet(root / "data" / "chunk-000" / f"episode_{ep:06d}.parquet")


def test_extract_then_compare(tmp_path):
    """Full lightweight chain: extract subset -> load -> run compare_utils."""
    root = tmp_path / "mini"
    _make_mini_dataset(root)
    out = tmp_path / "subset"
    ex.extract(str(root), num_samples=6, seed=0, out_dir=str(out))
    data = ex.load_subset(str(out))
    assert data["image"].shape[0] == 6

    # simulate two "sides" with a tiny controllable diff and compare
    a = data["actions"].astype(np.float64)
    b = a + 1e-3
    entry = cu.compare_pointwise(a, b, cu.tol("loss", "bf16"))
    assert entry["pass"] is True
    rep = cu.make_report({"loss": entry})
    assert rep["overall_pass"] is True


def test_report_json_schema(tmp_path):
    entries = {
        "v_t": {"kind": "pointwise", "max_abs_diff": 0.01, "tol": 0.06, "pass": True, "blocking": True},
        "loss": {"kind": "pointwise", "max_abs_diff": 0.004, "tol": 0.01, "pass": True, "blocking": True},
    }
    rep = cu.make_report(entries)
    out = tmp_path / "rep.json"
    cu.dump_report(rep, str(out))
    loaded = json.loads(out.read_text())
    for key in ["entries", "overall_pass", "num_pass", "num_total", "num_blocking"]:
        assert key in loaded
    assert loaded["overall_pass"] is True


def test_blocking_vs_warning():
    entries = {
        "hard": {"pass": True, "blocking": True},
        "soft": {"pass": False, "blocking": False},
    }
    rep = cu.make_report(entries)
    assert rep["overall_pass"] is True       # soft failure doesn't gate
    assert "soft" in rep["warnings"]

    entries2 = {"hard": {"pass": False, "blocking": True}}
    assert cu.make_report(entries2)["overall_pass"] is False


def test_scripts_importable():
    """All acceptance scripts import without side effects (no heavy model load)."""
    import ablation_runner  # noqa: F401
    import eval_compare  # noqa: F401
    import forward_align  # noqa: F401
    import train_compare  # noqa: F401

    assert hasattr(forward_align, "build_inputs")
    assert hasattr(train_compare, "run_pt_train")
    assert hasattr(eval_compare, "run_smoke")


def test_run_acceptance_script_exists():
    sh = Path(__file__).resolve().parents[1] / "scripts" / "run_acceptance.sh"
    assert sh.exists()
    text = sh.read_text()
    assert "L1 forward" in text and "L2 training" in text and "L3 inference" in text
