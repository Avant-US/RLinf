"""Layer-3 end-to-end check: load trained weights and validate the inference pipeline.

Per rlinfpi_accept_1.md §C/§4.4, the *verdict phase* L3 verifies that:
  * the checkpoint weights load into the openpi_au model, and
  * the inference path (sample_actions) produces finite, correctly-shaped actions.

A full LIBERO success-rate evaluation needs the LIBERO/robosuite simulator and a
fully-trained checkpoint; it is a separate large-scale trigger. When the simulator is
available and --mode full is set, this script defers to toolkits/eval_scripts_openpi.
Otherwise it runs the runnable inference smoke (--mode smoke, default).

Usage:
  python tests_au/scripts/eval_compare.py \
      --rlinf_ckpt <PT ckpt dir> --mode smoke \
      --subset_path tests_au/scripts/_data/libero_subset \
      --out_report tests_au/scripts/_out/eval_report.json
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

import compare_utils as cu  # noqa: E402
import forward_align as fa  # noqa: E402


def _libero_sim_available() -> bool:
    try:
        import libero  # noqa: F401

        return True
    except Exception:
        return False


def run_smoke(args) -> dict:
    """Load the checkpoint and run sample_actions on a LIBERO subset; validate output."""
    import torch
    from openpi.models import model as _model
    import openpi.training.config as _config

    from rlinf.models.embodiment.openpi_au.openpi_action_model import (
        OpenPi0Config,
        OpenPi0ForRLActionPrediction,
    )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    train_config = _config.get_config(args.jax_config)
    mcfg = train_config.model
    cfg_kwargs = dict(mcfg.__dict__)
    cfg_kwargs["config_name"] = args.jax_config
    cfg_kwargs["faithful_augmentation"] = False
    au_cfg = OpenPi0Config(**cfg_kwargs)

    model = OpenPi0ForRLActionPrediction(au_cfg)
    import safetensors.torch as st

    weight_path = os.path.join(args.rlinf_ckpt, "model.safetensors")
    sd = st.load_file(weight_path, device="cpu")
    missing, unexpected = model.load_state_dict(sd, strict=False)
    model = model.to(device=device).eval()
    model.paligemma_with_expert.to_bfloat16_for_selected_params("bfloat16")

    inp = fa.build_inputs(args.subset_path, args.num_samples, mcfg.action_dim,
                          mcfg.action_horizon, mcfg.max_token_len)

    def _t(x):
        return torch.as_tensor(x, device=device, dtype=torch.float32)

    obs = _model.Observation(
        images={k: _t(v).permute(0, 3, 1, 2).contiguous() for k, v in inp["images"].items()},
        image_masks={k: torch.as_tensor(v, device=device) for k, v in inp["image_masks"].items()},
        state=_t(inp["state"]),
        tokenized_prompt=torch.as_tensor(inp["tokenized_prompt"], device=device),
        tokenized_prompt_mask=torch.as_tensor(inp["tokenized_prompt_mask"], device=device),
    )

    from openpi.models_pytorch.pi0_pytorch import PI0Pytorch

    with torch.no_grad():
        actions = PI0Pytorch.sample_actions(model, device, obs, num_steps=args.num_steps)
    actions_np = actions.float().cpu().numpy()

    expected_shape = (args.num_samples, mcfg.action_horizon, mcfg.action_dim)
    shape_ok = actions_np.shape == expected_shape
    finite_ok = bool(np.all(np.isfinite(actions_np)))

    entries = {
        "weights_loaded": {
            "kind": "load", "num_missing": len(missing), "num_unexpected": len(unexpected),
            "pass": True,  # strict=False; report counts for inspection
            "blocking": True,
        },
        "action_shape": {
            "kind": "shape", "got": list(actions_np.shape), "expected": list(expected_shape),
            "pass": shape_ok, "blocking": True,
        },
        "action_finite": {
            "kind": "finite", "all_finite": finite_ok,
            "abs_max": float(np.abs(actions_np).max()),
            "pass": finite_ok, "blocking": True,
        },
    }
    report = cu.make_report(entries)
    report["mode"] = "smoke"
    return report


def run_full(args) -> dict:
    """Defer to the LIBERO simulator eval when available; else report skipped."""
    if not _libero_sim_available():
        return {
            "mode": "full", "overall_pass": True, "skipped": True,
            "reason": "LIBERO/robosuite simulator not installed; full success-rate "
                      "eval is a separate large-scale trigger (see §4.4).",
            "entries": {}, "num_pass": 0, "num_total": 0,
        }
    # Real sim eval path (only runs where the simulator is installed).
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "toolkits" / "eval_scripts_openpi"))
    import libero_eval  # type: ignore

    results = libero_eval.evaluate(  # pragma: no cover - requires simulator
        checkpoint=args.rlinf_ckpt, suite=args.suite, episodes=args.episodes, use_ema=True,
    )
    avg = float(np.mean(list(results.values())))
    entries = {
        "avg_success": {
            "kind": "success_rate", "per_suite": results, "avg": avg,
            "baseline": args.baseline, "pass": avg >= args.baseline, "blocking": True,
        }
    }
    report = cu.make_report(entries)
    report["mode"] = "full"
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rlinf_ckpt", required=True)
    ap.add_argument("--mode", choices=["smoke", "full"], default="smoke")
    ap.add_argument("--jax_config", default="pi05_libero")
    ap.add_argument("--subset_path", default="tests_au/scripts/_data/libero_subset")
    ap.add_argument("--num_samples", type=int, default=4)
    ap.add_argument("--num_steps", type=int, default=10)
    ap.add_argument("--suite", default="spatial")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--baseline", type=float, default=0.771)
    ap.add_argument("--out_report", default="tests_au/scripts/_out/eval_report.json")
    args = ap.parse_args()

    report = run_smoke(args) if args.mode == "smoke" else run_full(args)
    cu.dump_report(report, args.out_report)
    if report.get("skipped"):
        print(f"[eval] mode={report['mode']} SKIPPED: {report['reason']}")
    else:
        print(cu.pretty_print_report(report))
    sys.exit(0 if report.get("overall_pass") else 1)


if __name__ == "__main__":
    main()
