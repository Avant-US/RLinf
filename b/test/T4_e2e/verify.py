#!/usr/bin/env python3
"""T4: Compare end-to-end loss curves from native and RLinf training logs."""
import sys, os, re
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from common.compare_utils import report

RESULTS_DIR = os.environ.get("T4_RESULTS_BASE", "/mnt/r/tmp/fw_test") + "/T4"


def parse_native_losses(log_path):
    losses = {}
    with open(log_path) as f:
        for line in f:
            m = re.search(r"step=(\d+)/\d+\s+loss=([\d.]+)", line)
            if m:
                losses[int(m.group(1))] = float(m.group(2))
    return losses


def parse_rlinf_losses(log_path):
    losses = {}
    with open(log_path) as f:
        for line in f:
            m = re.search(r"(\d+)/\d+.*train/loss=([\d.]+)", line)
            if m:
                losses[int(m.group(1))] = float(m.group(2))
    return losses


def main():
    native_log = os.path.join(RESULTS_DIR, "native", "train.log")
    rlinf_log = os.path.join(RESULTS_DIR, "rlinf", "train.log")

    if not os.path.exists(native_log):
        report("T4 E2E Loss Curve", False, f"Native log not found: {native_log}")
        sys.exit(1)
    if not os.path.exists(rlinf_log):
        report("T4 E2E Loss Curve", False, f"RLinf log not found: {rlinf_log}")
        sys.exit(1)

    losses_n = parse_native_losses(native_log)
    losses_r = parse_rlinf_losses(rlinf_log)

    if not losses_n:
        report("T4 E2E Loss Curve", False, "No native loss entries parsed")
        sys.exit(1)
    if not losses_r:
        report("T4 E2E Loss Curve", False, "No RLinf loss entries parsed")
        sys.exit(1)

    first_n = losses_n[min(losses_n)]
    first_r = losses_r[min(losses_r)]
    last_n = losses_n[max(losses_n)]
    last_r = losses_r[max(losses_r)]
    mean_n = sum(losses_n.values()) / len(losses_n)
    mean_r = sum(losses_r.values()) / len(losses_r)

    print(f"  Native: {len(losses_n)} steps, first={first_n:.4f}, last={last_n:.4f}, mean={mean_n:.4f}")
    print(f"  RLinf:  {len(losses_r)} steps, first={first_r:.4f}, last={last_r:.4f}, mean={mean_r:.4f}")

    checks = []

    # Check 1: initial loss same magnitude
    if max(first_n, first_r) > 0:
        init_rel = abs(first_n - first_r) / max(first_n, first_r)
        checks.append(("initial loss rel diff", init_rel, init_rel < 0.3))

    # Check 2: both have finite losses
    all_finite = all(0 < v < 1000 for v in losses_n.values()) and all(0 < v < 1000 for v in losses_r.values())
    checks.append(("all losses finite", None, all_finite))

    # Check 3: mean loss same order of magnitude
    if max(mean_n, mean_r) > 0:
        mean_rel = abs(mean_n - mean_r) / max(mean_n, mean_r)
        checks.append(("mean loss rel diff", mean_rel, mean_rel < 0.3))

    # Check 4: both completed enough steps
    checks.append(("native steps", len(losses_n), len(losses_n) >= 10))
    checks.append(("rlinf steps", len(losses_r), len(losses_r) >= 10))

    # Check 5: both show loss decay (last 10 mean < first 10 mean)
    sorted_n = [losses_n[k] for k in sorted(losses_n)]
    sorted_r = [losses_r[k] for k in sorted(losses_r)]
    if len(sorted_n) >= 20:
        native_decays = sum(sorted_n[-10:]) / 10 < sum(sorted_n[:10]) / 10
        checks.append(("native loss decays", native_decays, native_decays))
    if len(sorted_r) >= 20:
        rlinf_decays = sum(sorted_r[-10:]) / 10 < sum(sorted_r[:10]) / 10
        checks.append(("rlinf loss decays", rlinf_decays, rlinf_decays))

    all_passed = all(c[2] for c in checks)
    details = "\n".join(f"    {name}: {f'{val:.4f}' if isinstance(val, float) else val} {'✓' if ok else '✗'}"
                        for name, val, ok in checks)
    report("T4 E2E Loss Curve", all_passed, details)
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
