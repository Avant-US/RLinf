#!/usr/bin/env python3
"""T0: Verify trainable parameter sets match between native and RLinf."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from common.model_factory import create_native, apply_native_train_mode, wrap_in_policy
from common.compare_utils import report

def main():
    native = create_native("cpu")
    apply_native_train_mode(native)
    native_trainable = {n for n, p in native.named_parameters() if p.requires_grad}

    policy = wrap_in_policy(native)
    rlinf_trainable = {n.replace("fastwam.", "", 1) for n, p in policy.named_parameters() if p.requires_grad}

    only_native = native_trainable - rlinf_trainable
    only_rlinf = rlinf_trainable - native_trainable

    passed = not only_native and not only_rlinf
    details = f"{len(native_trainable)} trainable params"
    if not passed:
        details += f"\n  Native-only: {sorted(only_native)[:5]}\n  RLinf-only: {sorted(only_rlinf)[:5]}"
    report("T0 Trainable Params", passed, details)
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()
