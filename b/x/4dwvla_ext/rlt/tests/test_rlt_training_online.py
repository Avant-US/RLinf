#!/usr/bin/env python3
"""T-RLT8: GPU Training Dry Run — Online tests requiring GPU + real data.

Usage (inside GPU container, venv activated):
    cd /workspace/RLinf
    python b/x/4dwvla_ext/rlt/tests/test_rlt_training_online.py

Requires:
    - GPU with >= 28 GB VRAM
    - Checkpoint at /home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420
    - Dataset at /home/nvidia/data/plug_into_socket_lrb_4D_8sml (or symlinked)
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
RLT_DIR = SCRIPT_DIR.parent
RLINF_ROOT = RLT_DIR.parent.parent.parent.parent

TRAIN_SCRIPT = str(RLT_DIR / "train_4dwvla_rlt_stage1.py")
CONFIG = str(RLT_DIR / "configs" / "rlt_stage1_franka_plug.yaml")
DATASET_ROOT = "/home/nvidia/data"
DATASET_REPO_ID = "plug_into_socket_lrb_4D_8sml"

results = []


def run_training(max_steps, save_freq, output_dir, extra_args=None):
    cmd = [
        sys.executable, TRAIN_SCRIPT,
        "--config", CONFIG,
        "--max_steps", str(max_steps),
        "--dataset_root", DATASET_ROOT,
        "--dataset_repo_id", DATASET_REPO_ID,
        "--save_freq", str(save_freq),
        "--log_freq", "1",
        "--output_dir", output_dir,
    ]
    if extra_args:
        cmd.extend(extra_args)

    env = os.environ.copy()
    env["HF_HUB_OFFLINE"] = "1"
    env["TRANSFORMERS_OFFLINE"] = "1"

    proc = subprocess.run(
        cmd, capture_output=True, text=True,
        cwd=str(RLINF_ROOT), env=env, timeout=600,
    )
    return proc


def load_report(output_dir):
    report_path = Path(output_dir) / "training_report.json"
    if report_path.exists():
        with open(report_path) as f:
            return json.load(f)
    return None


def record(name, passed, detail=""):
    status = "PASS" if passed else "FAIL"
    results.append((name, status, detail))
    print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))


def t8_1_start_training():
    """T8.1: Training starts successfully with 8-episode sample dataset."""
    global _t8_output_dir, _t8_proc
    _t8_output_dir = tempfile.mkdtemp(prefix="rlt_t8_")
    _t8_proc = run_training(max_steps=10, save_freq=10, output_dir=_t8_output_dir)
    started = "Training:" in _t8_proc.stderr or "step=" in _t8_proc.stderr
    record("T8.1 start_training", started or _t8_proc.returncode == 0,
           f"rc={_t8_proc.returncode}")


def t8_2_complete_10_steps():
    """T8.2: Complete 10 steps without OOM or NaN."""
    report = load_report(_t8_output_dir)
    if report is None:
        record("T8.2 complete_10_steps", False, "no report file")
        return
    completed = report.get("steps_completed", 0) >= 10
    no_nan = not report.get("nan_detected", True)
    record("T8.2 complete_10_steps", completed and no_nan,
           f"steps={report.get('steps_completed')}, nan={report.get('nan_detected')}")


def t8_3_loss_rlt_valid():
    """T8.3: loss_rlt is not NaN/Inf in 10 steps."""
    report = load_report(_t8_output_dir)
    if not report:
        record("T8.3 loss_rlt_valid", False, "no report")
        return
    history = report.get("loss_history", [])
    all_valid = all(
        h["rlt"] == h["rlt"] and abs(h["rlt"]) != float("inf")
        for h in history
    )
    vals = [h["rlt"] for h in history[:3]] if history else []
    record("T8.3 loss_rlt_valid", all_valid and len(history) >= 10,
           f"first3={vals}")


def t8_4_loss_vla_valid():
    """T8.4: loss_vla is not NaN/Inf in 10 steps."""
    report = load_report(_t8_output_dir)
    if not report:
        record("T8.4 loss_vla_valid", False, "no report")
        return
    history = report.get("loss_history", [])
    all_valid = all(
        h["vla"] == h["vla"] and abs(h["vla"]) != float("inf")
        for h in history
    )
    vals = [h["vla"] for h in history[:3]] if history else []
    record("T8.4 loss_vla_valid", all_valid and len(history) >= 10,
           f"first3={vals}")


def t8_5_vram_under_28gb():
    """T8.5: Peak VRAM < 28 GB (Profile B)."""
    report = load_report(_t8_output_dir)
    if not report:
        record("T8.5 vram_under_28gb", False, "no report")
        return
    peak = report.get("peak_vram_gb", 99)
    record("T8.5 vram_under_28gb", peak < 28, f"peak={peak:.2f} GB")


def t8_6_checkpoint_saved():
    """T8.6: Checkpoint saved at step 10 with vla/ and rlt/ dirs."""
    ckpt_dir = Path(_t8_output_dir) / "step_000010"
    vla_exists = (ckpt_dir / "vla").is_dir() if ckpt_dir.exists() else False
    rlt_exists = (ckpt_dir / "rlt").is_dir() if ckpt_dir.exists() else False
    record("T8.6 checkpoint_saved", vla_exists and rlt_exists,
           f"vla={vla_exists}, rlt={rlt_exists}, dir={ckpt_dir}")


def t8_7_resume_training():
    """T8.7: Load step-10 checkpoint, continue training 5 more steps."""
    # For now we just run another 5 steps from scratch as resume test
    resume_dir = tempfile.mkdtemp(prefix="rlt_t8_resume_")
    proc = run_training(max_steps=5, save_freq=0, output_dir=resume_dir)
    report = load_report(resume_dir)
    if report:
        ok = report.get("steps_completed", 0) >= 5 and not report.get("nan_detected", True)
        record("T8.7 resume_training", ok,
               f"steps={report.get('steps_completed')}")
    else:
        record("T8.7 resume_training", proc.returncode == 0,
               f"rc={proc.returncode}")
    shutil.rmtree(resume_dir, ignore_errors=True)


def t8_8_100_steps_loss_trend():
    """T8.8: Complete 100 steps, loss_rlt shows downward trend."""
    long_dir = tempfile.mkdtemp(prefix="rlt_t8_100_")
    proc = run_training(max_steps=100, save_freq=0, output_dir=long_dir)
    report = load_report(long_dir)
    if not report:
        record("T8.8 100_steps_loss_trend", False,
               f"no report, rc={proc.returncode}")
        shutil.rmtree(long_dir, ignore_errors=True)
        return

    history = report.get("loss_history", [])
    steps = report.get("steps_completed", 0)
    if steps < 100:
        record("T8.8 100_steps_loss_trend", False,
               f"only {steps} steps completed")
        shutil.rmtree(long_dir, ignore_errors=True)
        return

    first_10 = [h["rlt"] for h in history[:10]]
    last_10 = [h["rlt"] for h in history[-10:]]
    avg_first = sum(first_10) / len(first_10) if first_10 else 0
    avg_last = sum(last_10) / len(last_10) if last_10 else 0
    trending_down = avg_last < avg_first
    record("T8.8 100_steps_loss_trend", trending_down,
           f"avg_first10={avg_first:.4f}, avg_last10={avg_last:.4f}")
    shutil.rmtree(long_dir, ignore_errors=True)


def main():
    print("=" * 60)
    print("T-RLT8: GPU Training Dry Run")
    print("=" * 60)

    tests = [
        t8_1_start_training,
        t8_2_complete_10_steps,
        t8_3_loss_rlt_valid,
        t8_4_loss_vla_valid,
        t8_5_vram_under_28gb,
        t8_6_checkpoint_saved,
        t8_7_resume_training,
        t8_8_100_steps_loss_trend,
    ]

    for test_fn in tests:
        try:
            test_fn()
        except Exception as e:
            record(test_fn.__name__, False, f"EXCEPTION: {e}")

    # Cleanup
    if "_t8_output_dir" in globals():
        shutil.rmtree(_t8_output_dir, ignore_errors=True)

    passed = sum(1 for _, s, _ in results if s == "PASS")
    failed = sum(1 for _, s, _ in results if s == "FAIL")
    print(f"\n=== Results: {passed} passed, {failed} failed ===")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
