#!/usr/bin/env python3
"""T3: Checkpoint interop — RLinf checkpoint loads into native model, forward matches."""
import sys, os, torch
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from common.model_factory import create_native, apply_native_train_mode, wrap_in_policy, make_batch
from common.compare_utils import rlinf_sd_to_native, report

def main():
    device = torch.device("cuda:0")
    dtype = torch.bfloat16

    native_src = create_native(device, dtype)
    apply_native_train_mode(native_src)
    policy = wrap_in_policy(native_src)

    batch = make_batch(device, dtype, seed=77)
    opt = torch.optim.AdamW([p for p in policy.parameters() if p.requires_grad],
                             lr=1e-4, betas=(0.9, 0.95), weight_decay=1e-2)
    opt.zero_grad(set_to_none=True)
    torch.cuda.manual_seed(555)
    with torch.amp.autocast("cuda", dtype=dtype):
        out = policy.sft_forward(data=batch)
    out["loss"].backward()
    opt.step()

    mot_sd, pe_sd = rlinf_sd_to_native(policy.state_dict())

    native_dst = create_native(device, dtype)
    native_dst.mot.load_state_dict(mot_sd, strict=False)
    if pe_sd and native_dst.proprio_encoder is not None:
        native_dst.proprio_encoder.load_state_dict(pe_sd, strict=True)
    apply_native_train_mode(native_dst)

    test_batch = make_batch(device, dtype, seed=999)
    rng = torch.cuda.get_rng_state(device)

    torch.cuda.set_rng_state(rng, device)
    with torch.amp.autocast("cuda", dtype=dtype):
        loss_n, _ = native_dst.training_loss(test_batch)

    torch.cuda.set_rng_state(rng, device)
    with torch.amp.autocast("cuda", dtype=dtype):
        out2 = policy.sft_forward(data=test_batch)
    loss_r = out2["loss"]

    diff = (loss_n - loss_r).abs().item()
    passed = diff < 1e-5
    report("T3 Checkpoint Interop", passed,
           f"native={loss_n.item():.6f}  rlinf={loss_r.item():.6f}  diff={diff:.2e}")
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()
