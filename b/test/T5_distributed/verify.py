#!/usr/bin/env python3
"""T5: Compare RLinf 1-GPU vs 4-GPU training consistency."""
import sys, os, re
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from common.compare_utils import report

RESULTS_DIR = os.environ.get("T5_RESULTS_BASE", "/mnt/r/tmp/fw_test") + "/T5"


def parse_losses(log_path):
    losses = {}
    with open(log_path) as f:
        for line in f:
            m = re.search(r"(\d+)/\d+.*train/loss=([\d.]+)", line)
            if m:
                losses[int(m.group(1))] = float(m.group(2))
    return losses


def main():
    log_1 = os.path.join(RESULTS_DIR, "1gpu", "train.log")
    log_4 = os.path.join(RESULTS_DIR, "4gpu", "train.log")

    for path, label in [(log_1, "1-GPU"), (log_4, "4-GPU")]:
        if not os.path.exists(path):
            report("T5 Distributed", False, f"{label} log not found: {path}")
            sys.exit(1)

    losses_1 = parse_losses(log_1)
    losses_4 = parse_losses(log_4)

    if not losses_1 or not losses_4:
        report("T5 Distributed", False, f"Parsed 1-GPU={len(losses_1)}, 4-GPU={len(losses_4)} entries")
        sys.exit(1)

    mean_1 = sum(losses_1.values()) / len(losses_1)
    mean_4 = sum(losses_4.values()) / len(losses_4)
    print(f"  1-GPU: {len(losses_1)} steps, mean={mean_1:.4f}")
    print(f"  4-GPU: {len(losses_4)} steps, mean={mean_4:.4f}")

    checks = []
    checks.append(("1-GPU completed", len(losses_1), len(losses_1) >= 10))
    checks.append(("4-GPU completed", len(losses_4), len(losses_4) >= 10))
    checks.append(("all 1-GPU finite", None, all(0 < v < 1000 for v in losses_1.values())))
    checks.append(("all 4-GPU finite", None, all(0 < v < 1000 for v in losses_4.values())))

    if max(mean_1, mean_4) > 0:
        rel = abs(mean_1 - mean_4) / max(mean_1, mean_4)
        checks.append(("mean loss rel diff", rel, rel < 0.5))

    all_passed = all(c[2] for c in checks)
    details = "\n".join(f"    {name}: {f'{val:.4f}' if isinstance(val, float) else val} {'✓' if ok else '✗'}"
                        for name, val, ok in checks)
    report("T5 Distributed", all_passed, details)
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
