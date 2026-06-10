#!/usr/bin/env python3
"""T2: Multi-step gradient & weight alignment (3 steps)."""
import sys, os, copy, torch
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from common.model_factory import create_native, apply_native_train_mode, wrap_in_policy, make_batch
from common.compare_utils import compute_weight_diff, report

N = 3
LR = 1e-4

def main():
    device = torch.device("cuda:0")
    dtype = torch.bfloat16

    native = create_native(device, dtype)
    apply_native_train_mode(native)

    native_copy = copy.deepcopy(native)
    apply_native_train_mode(native_copy)
    policy = wrap_in_policy(native_copy)

    native_params = list(native.dit.parameters())
    if native.proprio_encoder is not None:
        native_params += list(native.proprio_encoder.parameters())
    opt_n = torch.optim.AdamW(native_params, lr=LR, betas=(0.9, 0.95), weight_decay=1e-2)
    opt_r = torch.optim.AdamW([p for p in policy.parameters() if p.requires_grad],
                               lr=LR, betas=(0.9, 0.95), weight_decay=1e-2)

    batches = [make_batch(device, dtype, seed=200 + i) for i in range(N)]
    rng_states = []
    for i in range(N):
        torch.cuda.manual_seed(9999 + i)
        rng_states.append(torch.cuda.get_rng_state(device))

    losses_n, losses_r = [], []
    for step in range(N):
        opt_n.zero_grad(set_to_none=True)
        torch.cuda.set_rng_state(rng_states[step], device)
        with torch.amp.autocast("cuda", dtype=dtype):
            loss_n, _ = native.training_loss(batches[step])
        loss_n.backward()
        torch.nn.utils.clip_grad_norm_(native.parameters(), 1.0)
        opt_n.step()
        losses_n.append(loss_n.detach().item())

        opt_r.zero_grad(set_to_none=True)
        torch.cuda.set_rng_state(rng_states[step], device)
        with torch.amp.autocast("cuda", dtype=dtype):
            out = policy.sft_forward(data=batches[step])
        loss_r = out["loss"]
        loss_r.backward()
        torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
        opt_r.step()
        losses_r.append(loss_r.detach().item())

        diff = abs(losses_n[-1] - losses_r[-1])
        print(f"  step {step}: native={losses_n[-1]:.6f}  rlinf={losses_r[-1]:.6f}  diff={diff:.2e}")

    max_diff, n_compared, _ = compute_weight_diff(native.state_dict(), policy.state_dict())
    passed = max_diff < 1e-3
    report("T2 Multi-Step Weights", passed,
           f"{n_compared} tensors compared, max weight diff = {max_diff:.2e}")
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()
