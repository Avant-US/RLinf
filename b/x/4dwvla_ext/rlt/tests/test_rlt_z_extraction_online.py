#!/usr/bin/env python3
"""T-RLT9: z_rl Extraction Verification — Stage 2 contract tests.

Usage (inside GPU container, venv activated):
    cd /workspace/RLinf
    python b/x/4dwvla_ext/rlt/tests/test_rlt_z_extraction_online.py

Requires:
    - GPU
    - Checkpoint at /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420
    - Dataset at /home/nvidia/data/plug_into_socket_lrb_4D_8sml (or symlinked)
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
RLT_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(RLT_DIR))
sys.path.insert(0, str(RLT_DIR.parent))

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")

results = []


def record(name, passed, detail=""):
    status = "PASS" if passed else "FAIL"
    results.append((name, status, detail))
    print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))


def setup():
    """Build model + dataset, return (wrapper, dataloader)."""
    import torch
    from train_4dwvla_rlt_stage1 import (
        load_rlt_config, load_train_pipeline_config,
        build_model, build_dataset,
    )
    import argparse

    args = argparse.Namespace(
        config=str(RLT_DIR / "configs" / "rlt_stage1_franka_plug.yaml"),
        max_steps=5, dataset_root="/home/nvidia/data",
        dataset_repo_id="plug_into_socket_lrb_4D_8sml",
        save_freq=0, output_dir="/tmp/rlt_t9_test", log_freq=1,
    )

    rlt_cfg = load_rlt_config(args)
    train_cfg = load_train_pipeline_config(
        ckpt_path=rlt_cfg.pretrained_path,
        dataset_root="/home/nvidia/data",
        repo_id="plug_into_socket_lrb_4D_8sml",
    )
    train_cfg.steps = 5

    wrapper = build_model(train_cfg, rlt_cfg)
    wrapper = wrapper.cuda()
    wrapper.eval()

    dataset, dataloader, _ = build_dataset(train_cfg)
    return wrapper, dataloader


def t9_1_z_rl_shape(wrapper, dataloader):
    """T9.1: extract_z_rl returns shape [B, D_z] = [B, 1024]."""
    import torch
    batch = next(iter(dataloader))
    batch = {k: v.cuda() if hasattr(v, "cuda") else v for k, v in batch.items()}

    with torch.no_grad():
        z_rl = wrapper.extract_z_rl(batch)

    expected_dim = wrapper.rlt_module.embed_dim
    ok = z_rl.ndim == 2 and z_rl.shape[1] == expected_dim
    record("T9.1 z_rl_shape", ok,
           f"shape={list(z_rl.shape)}, expected=[B, {expected_dim}]")
    return z_rl, batch


def t9_2_z_rl_finite(z_rl):
    """T9.2: z_rl values are finite (no NaN/Inf)."""
    import torch
    no_nan = not torch.isnan(z_rl).any().item()
    no_inf = not torch.isinf(z_rl).any().item()
    record("T9.2 z_rl_finite", no_nan and no_inf,
           f"nan={torch.isnan(z_rl).sum().item()}, inf={torch.isinf(z_rl).sum().item()}")


def t9_3_z_rl_deterministic(wrapper, batch):
    """T9.3: Same input → same z_rl output (deterministic)."""
    import torch
    wrapper.eval()
    with torch.no_grad():
        z1 = wrapper.extract_z_rl(batch)
        z2 = wrapper.extract_z_rl(batch)

    max_diff = (z1 - z2).abs().max().item()
    record("T9.3 z_rl_deterministic", max_diff < 1e-6,
           f"max_diff={max_diff:.2e}")


def t9_4_z_rl_varies(wrapper, dataloader):
    """T9.4: z_rl differs across batches (not constant)."""
    import torch
    z_list = []
    dl_iter = iter(dataloader)
    for i in range(min(3, len(dataloader))):
        batch = next(dl_iter)
        batch = {k: v.cuda() if hasattr(v, "cuda") else v for k, v in batch.items()}
        with torch.no_grad():
            z = wrapper.extract_z_rl(batch)
        z_list.append(z.cpu())

    if len(z_list) < 2:
        record("T9.4 z_rl_varies", False, "need >= 2 batches")
        return

    all_z = torch.cat(z_list, dim=0)
    std = all_z.std().item()
    record("T9.4 z_rl_varies", std > 0.01,
           f"std={std:.4f}, n_samples={all_z.shape[0]}")


def t9_5_z_rl_norm(wrapper, dataloader):
    """T9.5: z_rl norm is reasonable (not exploding)."""
    import torch
    z_list = []
    dl_iter = iter(dataloader)
    for i in range(min(5, len(dataloader))):
        batch = next(dl_iter)
        batch = {k: v.cuda() if hasattr(v, "cuda") else v for k, v in batch.items()}
        with torch.no_grad():
            z = wrapper.extract_z_rl(batch)
        z_list.append(z.cpu())

    all_z = torch.cat(z_list, dim=0)
    norms = all_z.norm(dim=-1)
    mean_norm = norms.mean().item()
    max_norm = norms.max().item()
    record("T9.5 z_rl_norm", mean_norm < 100,
           f"mean_norm={mean_norm:.2f}, max_norm={max_norm:.2f}")


def main():
    print("=" * 60)
    print("T-RLT9: z_rl Extraction Verification")
    print("=" * 60)

    try:
        wrapper, dataloader = setup()
    except Exception as e:
        print(f"  [FAIL] setup — {e}")
        print(f"\n=== Results: 0 passed, 5 failed ===")
        return 1

    try:
        z_rl, batch = t9_1_z_rl_shape(wrapper, dataloader)
        t9_2_z_rl_finite(z_rl)
        t9_3_z_rl_deterministic(wrapper, batch)
        t9_4_z_rl_varies(wrapper, dataloader)
        t9_5_z_rl_norm(wrapper, dataloader)
    except Exception as e:
        record("unexpected", False, f"EXCEPTION: {e}")

    passed = sum(1 for _, s, _ in results if s == "PASS")
    failed = sum(1 for _, s, _ in results if s == "FAIL")
    print(f"\n=== Results: {passed} passed, {failed} failed ===")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
