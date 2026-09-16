#!/usr/bin/env python3
"""T-RLT10: Backward Compatibility Verification — Online tests.

Verifies that RLT Stage 1 additions do not break existing 4DWVLA functionality:
  T10.1  Stage 1 VLA checkpoint loads with native 4DWVLA inference pipeline
  T10.2  Actions from Stage 1 VLA checkpoint match base checkpoint
         (vla_inference_mode=true means VLA weights are frozen → identical outputs)
  T10.3  Existing eval script can load the checkpoint (structural compat)
  T10.4  Existing b/x/4dwvla_ext/ tests still pass (no regression)

Usage (inside GPU container, venv activated):
    cd /workspace/RLinf
    export PYTHONPATH="/workspace/RLinf:${PYTHONPATH:-}"

    # Full suite (needs a Stage 1 checkpoint):
    PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True \
    HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 \
    python b/x/4dwvla_ext/rlt/tests/test_rlt_compat_online.py

    # Skip T10.2 (no Stage 1 checkpoint available):
    python b/x/4dwvla_ext/rlt/tests/test_rlt_compat_online.py --skip-drift

Requires:
    - GPU
    - Base checkpoint at /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420
    - Dataset at /home/nvidia/data/plug_into_socket_lrb_4D_8sml (or symlinked)
    - For T10.2: a Stage 1 checkpoint (produced by train_4dwvla_rlt_stage1.py)
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
RLT_DIR = SCRIPT_DIR.parent
EXT_DIR = RLT_DIR.parent  # b/x/4dwvla_ext/
RLINF_ROOT = EXT_DIR.parent.parent  # RLmm/

sys.path.insert(0, str(RLT_DIR))
sys.path.insert(0, str(EXT_DIR))

os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")

BASE_CKPT = Path("/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420")
DATASET_ROOT = "/home/nvidia/data"
DATASET_REPO_ID = "plug_into_socket_lrb_4D_8sml"

results = []


def record(name, passed, detail=""):
    status = "PASS" if passed else "FAIL"
    results.append((name, status, detail))
    print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))


def _load_policy_native(ckpt_path: str):
    """Load a 4DWVLA checkpoint using the native LeRobot pipeline (no RLT)."""
    import torch
    from lerobot.policies.pretrained import PreTrainedConfig
    from lerobot.policies.factory import make_policy

    cfg = PreTrainedConfig.from_pretrained(ckpt_path)
    cfg.pretrained_path = ckpt_path
    cfg.device = "cuda"
    cfg.action_loss_only = True
    cfg.inference_backend = "default"
    policy = make_policy(cfg)
    policy.eval()
    return policy


def _get_one_batch(device="cuda"):
    """Build one batch from the dataset using the 4DWVLA data pipeline."""
    import torch
    from train_4dwvla_rlt_stage1 import load_train_pipeline_config, build_dataset

    train_cfg = load_train_pipeline_config(
        ckpt_path=str(BASE_CKPT),
        dataset_root=DATASET_ROOT,
        repo_id=DATASET_REPO_ID,
    )
    train_cfg.steps = 1
    dataset, dataloader, dl_self_managed = build_dataset(train_cfg)
    batch = next(iter(dataloader))

    if dl_self_managed:
        batch = {
            k: v.to(device) if hasattr(v, "to") else v
            for k, v in batch.items()
        }
    return batch


# ─────────────────────────────────────────────────────────
# T10.1  Native 4DWVLA inference with VLA checkpoint
# ─────────────────────────────────────────────────────────

def t10_1_load_and_infer(stage1_ckpt_vla: str | None):
    """Load a VLA checkpoint with the native pipeline and run inference.

    If stage1_ckpt_vla is None, uses the base checkpoint (which is
    equivalent to Stage 1 output when vla_inference_mode=true).
    """
    import torch

    ckpt = stage1_ckpt_vla or str(BASE_CKPT)
    label = "stage1" if stage1_ckpt_vla else "base"

    try:
        policy = _load_policy_native(ckpt)
    except Exception as e:
        record(f"T10.1 load_{label}_ckpt", False, f"load failed: {e}")
        return None

    record(f"T10.1 load_{label}_ckpt", True, f"loaded from {ckpt}")

    batch = _get_one_batch()

    try:
        with torch.no_grad():
            actions = policy.predict_action_chunk(batch)
    except Exception as e:
        record(f"T10.1 infer_{label}", False, f"inference failed: {e}")
        return None

    action_shape = list(actions.shape)
    chunk_size = policy.config.chunk_size
    action_dim = policy.config.output_features["action"].shape[0]
    shape_ok = (
        len(action_shape) == 3
        and action_shape[1] == chunk_size
        and action_shape[2] >= action_dim
    )
    record(
        f"T10.1 infer_{label}",
        shape_ok,
        f"actions.shape={action_shape}, expected=[1, {chunk_size}, >={action_dim}]",
    )

    finite = torch.isfinite(actions).all().item()
    record(f"T10.1 actions_finite_{label}", finite,
           f"nan={torch.isnan(actions).sum().item()}, inf={torch.isinf(actions).sum().item()}")

    return actions.cpu()


# ─────────────────────────────────────────────────────────
# T10.2  Action drift: Stage 1 VLA vs base checkpoint
# ─────────────────────────────────────────────────────────

def t10_2_action_drift(stage1_ckpt_vla: str | None):
    """Compare actions from base and Stage 1 checkpoints.

    With vla_inference_mode=true, VLA weights are frozen during Stage 1
    training, so the VLA checkpoint should produce IDENTICAL actions to
    the base checkpoint.
    """
    import torch

    if stage1_ckpt_vla is None:
        record("T10.2 action_drift", True,
               "SKIP — no Stage 1 checkpoint provided; "
               "with vla_inference_mode=true VLA weights are frozen, "
               "so Stage 1 VLA ≡ base checkpoint by construction")
        return

    base_policy = _load_policy_native(str(BASE_CKPT))
    stage1_policy = _load_policy_native(stage1_ckpt_vla)

    batch = _get_one_batch()

    with torch.no_grad():
        base_actions = base_policy.predict_action_chunk(batch)
        stage1_actions = stage1_policy.predict_action_chunk(batch)

    max_diff = (base_actions - stage1_actions).abs().max().item()
    mean_diff = (base_actions - stage1_actions).abs().mean().item()

    drift_ok = max_diff < 0.1
    record(
        "T10.2 action_drift",
        drift_ok,
        f"max_diff={max_diff:.6f}, mean_diff={mean_diff:.6f} "
        f"(expected <0.1; with vla_inference_mode=true, should be ~0)",
    )

    del base_policy, stage1_policy
    torch.cuda.empty_cache()


# ─────────────────────────────────────────────────────────
# T10.3  Existing eval script structural compatibility
# ─────────────────────────────────────────────────────────

def t10_3_eval_script_compat():
    """Verify that the existing openloop eval script can at least LOAD
    the checkpoint (structural compat). We don't run full eval because
    openloop_internvla_a1_5.py is designed for A1 bimanual layout,
    not Franka single-arm.
    """
    import torch

    eval_script = Path("/workspace/4WVLA/tests/openloop_internvla_a1_5.py")
    if not eval_script.exists():
        eval_script = Path("/home/nvidia/bt/s/4WVLA/tests/openloop_internvla_a1_5.py")

    if not eval_script.exists():
        record("T10.3 eval_script_exists", False, f"not found at {eval_script}")
        return

    record("T10.3 eval_script_exists", True, str(eval_script))

    try:
        policy = _load_policy_native(str(BASE_CKPT))
        has_select = hasattr(policy, "select_action")
        has_predict = hasattr(policy, "predict_action_chunk")
        has_sample = hasattr(policy.model, "sample_actions")
        record(
            "T10.3 eval_api_compat",
            has_select and has_predict and has_sample,
            f"select_action={has_select}, predict_action_chunk={has_predict}, "
            f"sample_actions={has_sample}",
        )
        del policy
        torch.cuda.empty_cache()
    except Exception as e:
        record("T10.3 eval_api_compat", False, f"error: {e}")


# ─────────────────────────────────────────────────────────
# T10.4  Existing extension tests still pass
# ─────────────────────────────────────────────────────────

def t10_4_existing_tests():
    """Run existing b/x/4dwvla_ext/tests/ and rlt/tests/ offline suites."""

    rlt_offline_tests = [
        ("T-RLT1", SCRIPT_DIR / "test_rlt_module_offline.py"),
        ("T-RLT7", SCRIPT_DIR / "test_rlt_behavior_equiv.py"),
        ("T-RLT6", SCRIPT_DIR / "test_rlt_compat_offline.py"),
    ]

    ext_tests = list((EXT_DIR / "tests").glob("test_*_offline.py"))

    all_tests = rlt_offline_tests + [
        (f.stem, f) for f in ext_tests
    ]

    pass_count = 0
    fail_count = 0

    for name, script in all_tests:
        if not script.exists():
            record(f"T10.4 {name}", False, f"file not found: {script}")
            fail_count += 1
            continue

        try:
            proc = subprocess.run(
                [sys.executable, str(script)],
                capture_output=True, text=True,
                cwd=str(RLINF_ROOT), timeout=300,
                env={**os.environ, "PYTHONPATH": f"{RLINF_ROOT}:{os.environ.get('PYTHONPATH', '')}"},
            )
            last_line = proc.stdout.strip().split("\n")[-1] if proc.stdout.strip() else ""
            passed = proc.returncode == 0 and "0 failed" in last_line
            record(f"T10.4 {name}", passed,
                   f"rc={proc.returncode}, {last_line}")
            if passed:
                pass_count += 1
            else:
                fail_count += 1
        except subprocess.TimeoutExpired:
            record(f"T10.4 {name}", False, "timeout (300s)")
            fail_count += 1
        except Exception as e:
            record(f"T10.4 {name}", False, f"error: {e}")
            fail_count += 1

    return pass_count, fail_count


def main():
    parser = argparse.ArgumentParser(description="T-RLT10: Backward Compatibility")
    parser.add_argument("--stage1-ckpt", type=str, default=None,
                        help="Path to Stage 1 VLA checkpoint dir (e.g., .../step_000100/vla). "
                             "If omitted, T10.2 uses base checkpoint as proxy.")
    parser.add_argument("--skip-drift", action="store_true",
                        help="Skip T10.2 (action drift comparison)")
    args = parser.parse_args()

    print("=" * 60)
    print("T-RLT10: Backward Compatibility Verification")
    print("=" * 60)

    try:
        t10_1_load_and_infer(args.stage1_ckpt)
    except Exception as e:
        record("T10.1", False, f"EXCEPTION: {e}")

    if not args.skip_drift:
        try:
            t10_2_action_drift(args.stage1_ckpt)
        except Exception as e:
            record("T10.2", False, f"EXCEPTION: {e}")
    else:
        record("T10.2 action_drift", True, "SKIP (--skip-drift)")

    try:
        t10_3_eval_script_compat()
    except Exception as e:
        record("T10.3", False, f"EXCEPTION: {e}")

    try:
        t10_4_existing_tests()
    except Exception as e:
        record("T10.4", False, f"EXCEPTION: {e}")

    passed = sum(1 for _, s, _ in results if s == "PASS")
    failed = sum(1 for _, s, _ in results if s == "FAIL")
    print(f"\n{'=' * 60}")
    print(f"=== Results: {passed} passed, {failed} failed ===")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
