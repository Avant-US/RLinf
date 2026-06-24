"""Unit tests for extract_libero_subset.py using a synthetic mini dataset."""

import io
import json
import sys
from pathlib import Path

import numpy as np
import pytest

_scripts = str(Path(__file__).resolve().parents[1] / "scripts")
if _scripts not in sys.path:
    sys.path.insert(0, _scripts)

import extract_libero_subset as ex


def _make_mini_dataset(root: Path, n_episodes=3, frames=10):
    """Create a tiny LeRobot-like dataset (parquet + tasks.jsonl)."""
    import pandas as pd
    from PIL import Image

    (root / "meta").mkdir(parents=True, exist_ok=True)
    (root / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)

    with open(root / "meta" / "tasks.jsonl", "w") as f:
        f.write(json.dumps({"task_index": 0, "task": "pick up the cube"}) + "\n")
        f.write(json.dumps({"task_index": 1, "task": "open the drawer"}) + "\n")

    def _png_bytes(seed):
        rng = np.random.default_rng(seed)
        arr = (rng.random((32, 32, 3)) * 255).astype(np.uint8)
        buf = io.BytesIO()
        Image.fromarray(arr).save(buf, format="PNG")
        return buf.getvalue()

    for ep in range(n_episodes):
        rows = []
        for fi in range(frames):
            rows.append({
                "image": {"bytes": _png_bytes(ep * 100 + fi), "path": None},
                "wrist_image": {"bytes": _png_bytes(ep * 100 + fi + 50), "path": None},
                "state": np.arange(8, dtype=np.float32) + ep,
                "actions": np.arange(7, dtype=np.float32) + fi,
                "timestamp": float(fi),
                "frame_index": fi,
                "episode_index": ep,
                "index": ep * frames + fi,
                "task_index": ep % 2,
            })
        df = pd.DataFrame(rows)
        df.to_parquet(root / "data" / "chunk-000" / f"episode_{ep:06d}.parquet")


@pytest.fixture
def mini_dataset(tmp_path):
    root = tmp_path / "mini_libero"
    _make_mini_dataset(root)
    return root


def test_deterministic_indices(mini_dataset, tmp_path):
    out1 = tmp_path / "o1"
    out2 = tmp_path / "o2"
    m1 = ex.extract(str(mini_dataset), num_samples=6, seed=0, out_dir=str(out1))
    m2 = ex.extract(str(mini_dataset), num_samples=6, seed=0, out_dir=str(out2))
    assert m1["indices"] == m2["indices"]


def test_subset_shapes(mini_dataset, tmp_path):
    out = tmp_path / "o"
    m = ex.extract(str(mini_dataset), num_samples=6, seed=0, out_dir=str(out))
    assert m["num_samples"] == 6
    data = ex.load_subset(str(out))
    assert data["image"].shape == (6, 32, 32, 3)
    assert data["image"].dtype == np.uint8
    assert data["state"].shape == (6, 8)
    assert data["actions"].shape == (6, 7)
    assert len(data["prompts"]) == 6


def test_meta_roundtrip(mini_dataset, tmp_path):
    out = tmp_path / "o"
    ex.extract(str(mini_dataset), num_samples=4, seed=1, out_dir=str(out))
    meta = json.loads((out / "meta.json").read_text())
    assert meta["seed"] == 1
    assert meta["num_samples"] == 4
    assert "image" in meta["shapes"]
    assert len(meta["prompts"]) == 4


def test_fixed_order(mini_dataset, tmp_path):
    out = tmp_path / "o"
    ex.extract(str(mini_dataset), num_samples=5, seed=0, out_dir=str(out))
    d1 = ex.load_subset(str(out))
    d2 = ex.load_subset(str(out))
    np.testing.assert_array_equal(d1["actions"], d2["actions"])


def test_load_subset_num_samples_limit(mini_dataset, tmp_path):
    out = tmp_path / "o"
    ex.extract(str(mini_dataset), num_samples=8, seed=0, out_dir=str(out))
    data = ex.load_subset(str(out), num_samples=3)
    assert data["image"].shape[0] == 3
    assert len(data["prompts"]) == 3


def test_prompts_nonempty(mini_dataset, tmp_path):
    out = tmp_path / "o"
    ex.extract(str(mini_dataset), num_samples=4, seed=0, out_dir=str(out))
    data = ex.load_subset(str(out))
    assert all(isinstance(p, str) and len(p) > 0 for p in data["prompts"])
