"""Ablation driver (rlinfpi_accept_1.md §D): run a few-step PyTorch training loop under
different config variants and report mean +/- std of the key metrics.

Variants (each runs `--repeats` times with different seeds):
  precision : fp32-master + bf16-compute  vs  pure bf16
  lr        : openpi_cosine               vs  constant
  ema       : ema on (0.999)              vs  off

Each run reuses train_compare.run_pt_train-style logic via subprocess so heavy model
loads stay isolated. Output: per-variant mean/std of final loss, grad_norm, param_norm.

Usage:
  python tests_au/scripts/ablation_runner.py --ablation precision \
      --subset_path tests_au/scripts/_data/libero_subset \
      --pt_ckpt <PT ckpt> --num_steps 8 --repeats 2 \
      --out_dir tests_au/scripts/_out/ablation
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent

# Runnable few-step ablations (precision M1, lr M2). The EMA ablation (H1) requires
# full-length training + eval to manifest and is therefore driven by the full training
# pipeline rather than this few-step driver (see rlinfpi_accept_1.md §D / §G).
ABLATIONS = {
    "precision": [
        {"name": "fp32_master_bf16_compute"},
        {"name": "pure_bf16", "pure_bf16": True},
    ],
    "lr": [
        {"name": "openpi_cosine", "lr_scheduler": "openpi_cosine"},
        {"name": "constant", "lr_scheduler": "constant"},
    ],
}


def _run_one(args, variant, seed):
    """Run a single few-step training and return its metric log dict."""
    out = str(Path(args.out_dir) / f"_run_{variant['name']}_seed{seed}.json")
    cmd = [
        sys.executable, str(_HERE / "train_compare.py"), "--side", "pt_train",
        "--subset_path", args.subset_path, "--jax_config", args.jax_config,
        "--pt_ckpt", args.pt_ckpt, "--num_steps", str(args.num_steps),
        "--batch_size", str(args.batch_size), "--warmup", str(args.warmup),
        "--out", out,
    ]
    env = dict(os.environ)
    env["AU_SEED"] = str(seed)
    if variant.get("pure_bf16"):
        env["AU_PURE_BF16"] = "1"
    if variant.get("lr_scheduler"):
        env["AU_LR_SCHEDULER"] = variant["lr_scheduler"]
    # A failed variant (e.g. pure-bf16 is incompatible with openpi's fp32 action head)
    # is a meaningful ablation result, not a crash. Capture and report it.
    proc = subprocess.run(cmd, env=env, capture_output=True, text=True)
    if proc.returncode != 0:
        tail = (proc.stderr or proc.stdout or "").strip().splitlines()[-1:] or ["unknown error"]
        return {"status": "failed", "reason": tail[-1]}
    with open(out) as f:
        log = json.load(f)
    log["status"] = "ok"
    return log


def _mean_std(values):
    if len(values) < 2:
        return {"mean": float(values[0]) if values else 0.0, "std": 0.0}
    return {"mean": float(statistics.mean(values)), "std": float(statistics.pstdev(values))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ablation", choices=list(ABLATIONS) + ["all"], default="precision")
    ap.add_argument("--subset_path", default="tests_au/scripts/_data/libero_subset")
    ap.add_argument("--jax_config", default="pi05_libero")
    ap.add_argument("--pt_ckpt", required=True)
    ap.add_argument("--num_steps", type=int, default=8)
    ap.add_argument("--batch_size", type=int, default=2)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--out_dir", default="tests_au/scripts/_out/ablation")
    args = ap.parse_args()

    Path(args.out_dir).mkdir(parents=True, exist_ok=True)
    ablations = list(ABLATIONS) if args.ablation == "all" else [args.ablation]

    summary = {}
    for ab in ablations:
        summary[ab] = {}
        for variant in ABLATIONS[ab]:
            final_losses, final_gnorms, final_pnorms = [], [], []
            fail_reason = None
            for seed in range(args.repeats):
                log = _run_one(args, variant, seed)
                if log.get("status") == "failed":
                    fail_reason = log.get("reason", "unknown")
                    break
                final_losses.append(log["loss"][-1])
                final_gnorms.append(log["grad_norm"][-1])
                final_pnorms.append(log["param_norm"][-1])
            if fail_reason is not None:
                summary[ab][variant["name"]] = {"status": "failed", "reason": fail_reason}
            else:
                summary[ab][variant["name"]] = {
                    "status": "ok",
                    "final_loss": _mean_std(final_losses),
                    "final_grad_norm": _mean_std(final_gnorms),
                    "final_param_norm": _mean_std(final_pnorms),
                }

    out_path = Path(args.out_dir) / "ablation_summary.json"
    with open(out_path, "w") as f:
        json.dump(summary, f, indent=2)

    # human-readable table
    for ab, variants in summary.items():
        print(f"\n=== ablation: {ab} ===")
        print(f"{'variant':<28}{'final_loss(mean+/-std)':<28}{'grad_norm':<16}")
        for name, m in variants.items():
            if m.get("status") == "failed":
                print(f"{name:<28}FAILED: {m['reason'][:50]}")
                continue
            fl = m["final_loss"]
            gn = m["final_grad_norm"]
            print(f"{name:<28}{fl['mean']:.4f}+/-{fl['std']:.4f}{'':<10}{gn['mean']:.3f}")
    print(f"\n[ablation] summary -> {out_path}")


if __name__ == "__main__":
    main()
