"""Unit tests for norm-stats loading, grad_accum arithmetic, and config assertions."""

import json
import pathlib
import tempfile

import numpy as np
import pytest


def test_norm_stats_json_roundtrip():
    stats = {
        "norm_stats": {
            "actions": {
                "mean": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                "std": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                "q01": [-2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0],
                "q99": [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
            },
            "state": {
                "mean": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "std": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                "q01": [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0],
                "q99": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            },
        }
    }

    with tempfile.TemporaryDirectory() as tmpdir:
        asset_dir = pathlib.Path(tmpdir) / "assets" / "physical-intelligence--libero"
        asset_dir.mkdir(parents=True)
        json_path = asset_dir / "norm_stats.json"
        with open(json_path, "w") as f:
            json.dump(stats, f)

        with open(json_path) as f:
            loaded = json.load(f)

        assert "norm_stats" in loaded
        ns = loaded["norm_stats"]
        assert "actions" in ns and "state" in ns
        np.testing.assert_allclose(ns["actions"]["mean"], stats["norm_stats"]["actions"]["mean"])


@pytest.mark.parametrize(
    "global_bs,micro_bs,world_size,expected_accum",
    [
        (256, 4, 1, 64),
        (256, 4, 2, 32),
        (256, 4, 4, 16),
        (256, 4, 8, 8),
        (256, 8, 4, 8),
        (128, 4, 4, 8),
        (32, 4, 1, 8),
        (32, 2, 2, 8),
    ],
)
def test_grad_accum_arithmetic(global_bs, micro_bs, world_size, expected_accum):
    assert global_bs % (micro_bs * world_size) == 0
    accum = global_bs // micro_bs // world_size
    assert accum == expected_accum


def test_grad_accum_indivisible():
    global_bs = 256
    micro_bs = 5
    world_size = 1
    assert global_bs % (micro_bs * world_size) != 0


def test_get_openpi_config_pi05_libero():
    try:
        from rlinf.models.embodiment.openpi_au.dataconfig import get_openpi_config
    except ImportError:
        pytest.skip("openpi_au dataconfig not importable")

    config = get_openpi_config("pi05_libero")
    assert config.model.pi05 is True
    assert config.model.action_horizon == 10
    assert config.model.discrete_state_input is False
    assert config.batch_size == 256
    assert config.ema_decay == 0.999
    assert config.optimizer.clip_gradient_norm == 1.0
    assert config.lr_schedule.peak_lr == 5e-5
    assert config.lr_schedule.warmup_steps == 10_000


def test_get_openpi_config_pi05_robotwin():
    try:
        from rlinf.models.embodiment.openpi_au.dataconfig import get_openpi_config
    except ImportError:
        pytest.skip("openpi_au dataconfig not importable")

    config = get_openpi_config("pi05_aloha_robotwin")
    assert config.model.pi05 is True
    assert config.model.discrete_state_input is True


def test_config_json_schema():
    required_fields = {"action_dim", "action_horizon", "paligemma_variant", "action_expert_variant", "precision"}
    sample = {
        "action_dim": 7,
        "action_horizon": 10,
        "paligemma_variant": "gemma_2b_lora",
        "action_expert_variant": "gemma_300m",
        "precision": "bfloat16",
    }
    assert required_fields.issubset(set(sample.keys()))


def test_asset_id_preserved_on_repo_override():
    try:
        from rlinf.models.embodiment.openpi_au.dataconfig import get_openpi_config
    except ImportError:
        pytest.skip("openpi_au dataconfig not importable")

    config = get_openpi_config("pi05_libero", repo_id="/local/data/libero")
    assert config.data.assets.asset_id == "physical-intelligence/libero"
    assert config.data.repo_id == "/local/data/libero"
