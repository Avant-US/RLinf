#!/usr/bin/env python3
"""Weighted-average merge of multiple FastWAM checkpoints.

--weights takes the action_loss of each checkpoint; the script
computes 1/loss for each, normalizes, and uses those as blend weights
(lower action_loss → higher weight).

Example:
    python b/scripts/merge_fastwam_ckpts.py \
        --ckpts ckpt_a/fastwam_native.pt ckpt_b/fastwam_native.pt \
        --weights 0.0144 0.0135 \
        --output merged/fastwam_native.pt
"""

import argparse
import os
from pathlib import Path

import torch


def merge_dicts(dicts, weights):
    """Recursively weighted-average all tensors in a list of nested dicts."""
    ref = dicts[0]
    merged = {}
    for key in ref:
        values = [d[key] for d in dicts]
        if isinstance(values[0], torch.Tensor):
            acc = torch.zeros_like(values[0], dtype=torch.float32)
            for v, w in zip(values, weights):
                acc += v.float() * w
            merged[key] = acc.to(values[0].dtype)
        elif isinstance(values[0], dict):
            merged[key] = merge_dicts(values, weights)
        else:
            merged[key] = values[0]
    return merged


def main():
    parser = argparse.ArgumentParser(description="Merge FastWAM checkpoints by weighted average (inverse action_loss)")
    parser.add_argument("--ckpts", nargs="+", required=True, help="Paths to fastwam_native.pt files")
    parser.add_argument("--weights", nargs="+", type=float, required=True, help="action_loss of each checkpoint (lower → higher merge weight)")
    parser.add_argument("--output", required=True, help="Output path for merged checkpoint")
    args = parser.parse_args()

    assert len(args.ckpts) == len(args.weights), (
        f"Number of checkpoints ({len(args.ckpts)}) must match number of weights ({len(args.weights)})"
    )
    assert len(args.ckpts) >= 2, "Need at least 2 checkpoints to merge"

    inv = [1.0 / w for w in args.weights]
    total = sum(inv)
    norm_weights = [w / total for w in inv]

    print("Merge plan:")
    for path, loss, w in zip(args.ckpts, args.weights, norm_weights):
        print(f"  {Path(path).parent.name}/{Path(path).name}  action_loss={loss}  weight={w:.4f}")

    checkpoints = []
    for i, path in enumerate(args.ckpts):
        print(f"Loading checkpoint {i+1}/{len(args.ckpts)}: {path}")
        ckpt = torch.load(path, map_location="cpu", weights_only=True)
        checkpoints.append(ckpt)

    ref_keys = set(checkpoints[0].keys())
    for i, ckpt in enumerate(checkpoints[1:], 1):
        assert set(ckpt.keys()) == ref_keys, f"Checkpoint {i} has different top-level keys"

    print("Merging weights...")
    merged = merge_dicts(checkpoints, norm_weights)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    print(f"Saving to {args.output}")
    torch.save(merged, args.output)

    # Quick verification
    print("\nVerification:")
    saved = torch.load(args.output, map_location="cpu", weights_only=True)

    def count_tensors(d):
        n = 0
        for v in d.values():
            if isinstance(v, torch.Tensor):
                n += 1
            elif isinstance(v, dict):
                n += count_tensors(v)
        return n

    n = count_tensors(saved)
    print(f"  Total tensor keys: {n}")
    print(f"  step: {saved.get('step')}")
    print(f"  torch_dtype: {saved.get('torch_dtype')}")

    # Spot-check one tensor
    def get_first_tensor(d, prefix=""):
        for k, v in d.items():
            key = f"{prefix}{k}" if prefix else k
            if isinstance(v, torch.Tensor):
                return key, v
            elif isinstance(v, dict):
                result = get_first_tensor(v, key + ".")
                if result:
                    return result
        return None

    key, val = get_first_tensor(saved)
    orig_vals = []
    for ckpt in checkpoints:
        # Navigate the nested dict structure matching how get_first_tensor found it
        v = ckpt
        remaining = key
        while remaining:
            for k in v:
                if remaining == k:
                    v = v[k]
                    remaining = ""
                    break
                elif remaining.startswith(k + "."):
                    v = v[k]
                    remaining = remaining[len(k) + 1:]
                    break
        orig_vals.append(v)

    expected = sum(v.float() * w for v, w in zip(orig_vals, norm_weights)).to(val.dtype)
    match = torch.equal(val, expected)
    print(f"  Spot-check '{key}': {'PASS' if match else 'FAIL'}")
    print("Done.")


if __name__ == "__main__":
    main()
