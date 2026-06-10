#!/usr/bin/env python3
"""T1: Single-step loss alignment — same model, same batch, same RNG."""
import sys, os, torch
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from common.model_factory import create_native, apply_native_train_mode, wrap_in_policy, make_batch
from common.compare_utils import report

def main():
    device = torch.device("cuda:0")
    dtype = torch.bfloat16

    native = create_native(device, dtype)
    apply_native_train_mode(native)
    policy = wrap_in_policy(native)

    batch = make_batch(device, dtype, seed=123)
    rng = torch.cuda.get_rng_state(device)

    torch.cuda.set_rng_state(rng, device)
    with torch.amp.autocast("cuda", dtype=dtype):
        loss_n, dict_n = native.training_loss(batch)

    torch.cuda.set_rng_state(rng, device)
    with torch.amp.autocast("cuda", dtype=dtype):
        out_r = policy.sft_forward(data=batch)
    loss_r = out_r["loss"]

    diff = (loss_n - loss_r).abs().item()
    passed = diff < 1e-5
    report("T1 Single-Step Loss", passed,
           f"native={loss_n.item():.6f}  rlinf={loss_r.item():.6f}  diff={diff:.2e}")
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()
