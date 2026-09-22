#!/usr/bin/env python3
"""Offline test: verify stats composition and action dimension handling.

Validates that:
- T12.1: stats.json sub-field keys can be composed into full observation.state
         and action keys via schema feature_mapping
- T12.2: Composed stats match per-field stats (element-wise concatenation)
- T12.3: Action dimension from composed stats matches expected 8D
- T12.4: Model output_features.action.shape (32D padded) != actual action dim (8D),
         confirming the need for dimension slicing

Run on any machine with Python 3.10+ and numpy:

    python /path/to/4dwvla_ext/tests/test_stats_composition_offline.py

Or in the GPU container (4dwvla venv):

    source /opt/venv/4dwvla/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/tests/test_stats_composition_offline.py
"""
import json
import sys
from pathlib import Path

import numpy as np

PASS = 0
FAIL = 0

STATS_JSON = Path("/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/stats.json")
CONFIG_JSON = Path("/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/config.json")
STATS_KEY = "franka_plug"

STATE_SUB_KEYS = ["observation.state.arm", "observation.state.gripper"]
ACTION_SUB_KEYS = ["action.arm", "action.gripper"]
EXPECTED_STATE_DIM = 8
EXPECTED_ACTION_DIM = 8


def check(name: str, condition: bool, detail: str = ""):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}: {detail}")


def load_stats():
    if not STATS_JSON.exists():
        return None
    with open(STATS_JSON) as f:
        stats = json.load(f)
    return stats.get(STATS_KEY, stats)


def test_sub_field_keys_exist():
    """T12.1: Verify stats.json has sub-field keys and lacks composed keys."""
    print("\n=== T12.1: Stats Sub-field Key Structure ===")

    selected = load_stats()
    if selected is None:
        print("  [SKIP] stats.json not accessible")
        return

    for key in STATE_SUB_KEYS + ACTION_SUB_KEYS:
        check(f"sub-field '{key}' exists in stats",
              key in selected,
              f"key not found")

    check("composed 'observation.state' absent from stats",
          "observation.state" not in selected,
          "unexpectedly found — composition not needed")

    check("composed 'action' absent from stats",
          "action" not in selected,
          "unexpectedly found — composition not needed")


def test_stats_composition():
    """T12.2: Verify that composing sub-field stats produces correct dimensions."""
    print("\n=== T12.2: Stats Composition Correctness ===")

    selected = load_stats()
    if selected is None:
        print("  [SKIP] stats.json not accessible")
        return

    for composed_name, sub_keys, expected_dim in [
        ("observation.state", STATE_SUB_KEYS, EXPECTED_STATE_DIM),
        ("action", ACTION_SUB_KEYS, EXPECTED_ACTION_DIM),
    ]:
        sub_means = []
        sub_stds = []
        for k in sub_keys:
            if k not in selected:
                print(f"  [SKIP] sub-key {k} missing")
                return
            sub_means.append(np.asarray(selected[k]["mean"]))
            sub_stds.append(np.asarray(selected[k]["std"]))

        composed_mean = np.concatenate(sub_means)
        composed_std = np.concatenate(sub_stds)

        check(f"{composed_name} composed mean dim == {expected_dim}",
              composed_mean.shape[0] == expected_dim,
              f"got {composed_mean.shape[0]}")

        check(f"{composed_name} composed std dim == {expected_dim}",
              composed_std.shape[0] == expected_dim,
              f"got {composed_std.shape[0]}")

        check(f"{composed_name} composed mean[0] == {sub_keys[0]} mean[0]",
              np.isclose(composed_mean[0], sub_means[0][0]),
              f"got {composed_mean[0]} vs {sub_means[0][0]}")

        last_sub_dim = sub_means[-1].shape[0]
        check(f"{composed_name} composed mean[-1] == {sub_keys[-1]} mean[-1]",
              np.isclose(composed_mean[-1], sub_means[-1][-1]),
              f"got {composed_mean[-1]} vs {sub_means[-1][-1]}")

        check(f"{composed_name} all std > 0",
              np.all(composed_std > 0),
              f"found zero/negative std: {composed_std}")


def test_action_dim_mismatch():
    """T12.3: Verify model output dim (32) != actual action dim (8)."""
    print("\n=== T12.3: Action Dimension Mismatch Detection ===")

    if not CONFIG_JSON.exists():
        print("  [SKIP] config.json not accessible")
        return

    with open(CONFIG_JSON) as f:
        cfg = json.load(f)

    output_features = cfg.get("output_features", {})
    action_shape = output_features.get("action", {}).get("shape", [])

    check("output_features.action.shape exists",
          len(action_shape) > 0,
          "action shape not found in config")

    if action_shape:
        model_action_dim = action_shape[0]
        check(f"model output action dim ({model_action_dim}) > actual action dim ({EXPECTED_ACTION_DIM})",
              model_action_dim > EXPECTED_ACTION_DIM,
              f"model_dim={model_action_dim}, expected > {EXPECTED_ACTION_DIM}")

        check(f"model output action dim == max_action_dim (32)",
              model_action_dim == 32,
              f"got {model_action_dim}")


def test_normalization_roundtrip():
    """T12.4: Verify normalize → unnormalize roundtrip with composed stats."""
    print("\n=== T12.4: Normalization Roundtrip ===")

    selected = load_stats()
    if selected is None:
        print("  [SKIP] stats.json not accessible")
        return

    for composed_name, sub_keys in [
        ("observation.state", STATE_SUB_KEYS),
        ("action", ACTION_SUB_KEYS),
    ]:
        sub_means = [np.asarray(selected[k]["mean"]) for k in sub_keys
                     if k in selected]
        sub_stds = [np.asarray(selected[k]["std"]) for k in sub_keys
                    if k in selected]
        if len(sub_means) != len(sub_keys):
            print(f"  [SKIP] missing sub-keys for {composed_name}")
            continue

        mean = np.concatenate(sub_means)
        std = np.concatenate(sub_stds)

        raw = mean.copy()
        eps = 1e-6
        normalized = (raw - mean) / (std + eps)
        unnormalized = normalized * (std + eps) + mean

        check(f"{composed_name} mean normalizes to ~0",
              np.allclose(normalized, 0, atol=1e-5),
              f"max deviation: {np.max(np.abs(normalized)):.6f}")

        check(f"{composed_name} roundtrip recovers original",
              np.allclose(unnormalized, raw, atol=1e-6),
              f"max deviation: {np.max(np.abs(unnormalized - raw)):.8f}")


def main():
    test_sub_field_keys_exist()
    test_stats_composition()
    test_action_dim_mismatch()
    test_normalization_roundtrip()
    print(f"\n=== T12 Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)


if __name__ == "__main__":
    main()
