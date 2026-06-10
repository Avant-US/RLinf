"""Checkpoint loading, key mapping, and diff reporting utilities."""

import re
import torch


def rlinf_sd_to_native(rlinf_sd):
    """Convert RLinf FSDP state_dict keys to native FastWAM checkpoint format."""
    mot_sd, pe_sd = {}, {}
    for k, v in rlinf_sd.items():
        if k.startswith("fastwam.mot."):
            mot_sd[k.replace("fastwam.mot.", "")] = v
        elif k.startswith("fastwam.proprio_encoder."):
            pe_sd[k.replace("fastwam.proprio_encoder.", "")] = v
    return mot_sd, pe_sd


def compute_weight_diff(sd_a, sd_b, prefix_a="", prefix_b="fastwam."):
    """Compute max absolute diff between two state dicts."""
    max_diff = 0.0
    n_compared = 0
    diffs = {}
    for ka in sd_a:
        kb = prefix_b + ka if prefix_a == "" else ka.replace(prefix_a, prefix_b, 1)
        if kb in sd_b and sd_a[ka].is_floating_point():
            d = (sd_a[ka].float() - sd_b[kb].float()).abs().max().item()
            max_diff = max(max_diff, d)
            n_compared += 1
            if d > 1e-6:
                diffs[ka] = d
    return max_diff, n_compared, diffs


def parse_native_loss_log(log_path):
    """Parse FastWAM native training stdout for loss values."""
    losses = {}
    with open(log_path) as f:
        for line in f:
            m = re.search(r"\[train\].*step=(\d+)/\d+\s+loss=([\d.]+)", line)
            if m:
                step, loss = int(m.group(1)), float(m.group(2))
                losses[step] = loss
    return losses


def parse_rlinf_loss_log(log_path):
    """Parse RLinf training stdout (tqdm) for loss values."""
    losses = {}
    with open(log_path) as f:
        for line in f:
            m = re.search(r"(\d+)/\d+.*train/loss=([\d.]+)", line)
            if m:
                step, loss = int(m.group(1)), float(m.group(2))
                losses[step] = loss
    return losses


def report(test_name, passed, details=""):
    status = "PASS" if passed else "FAIL"
    print(f"\n{'='*60}")
    print(f"  {test_name}: {status}")
    if details:
        print(f"  {details}")
    print(f"{'='*60}\n")
    return passed
