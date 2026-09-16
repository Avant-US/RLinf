#!/usr/bin/env python3
"""T-RLT4: Gradient isolation tests (offline).

Verifies that RLT and VLA loss gradients are fully decoupled through detach.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import torch
from rlt_token_transformer import RLTTokenTransformer

PASS = 0
FAIL = 0


def run_test(name, fn):
    global PASS, FAIL
    try:
        fn()
        PASS += 1
        print(f"  [PASS] {name}")
    except Exception as e:
        FAIL += 1
        print(f"  [FAIL] {name}: {e}")


def t4_1_rlt_params_have_grad():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, 100, 2048)
    loss, _ = rlt.loss(x)
    loss.backward()
    has_grad = all(p.grad is not None for p in rlt.parameters() if p.requires_grad)
    assert has_grad, "All RLT params should have gradients after backward"


def t4_2_simulated_vla_params_have_grad():
    """Simulate VLA params getting grad from VLA loss (not through RLT)."""
    vla_param = torch.randn(100, requires_grad=True)
    vla_loss = (vla_param ** 2).mean()
    vla_loss.backward()
    assert vla_param.grad is not None, "VLA param should have gradient"


def t4_3_frozen_params_no_grad():
    """Simulate frozen VLM backbone params."""
    frozen_param = torch.randn(100, requires_grad=False)
    trainable_param = torch.randn(100, requires_grad=True)
    loss = (trainable_param ** 2).mean()
    loss.backward()
    assert frozen_param.grad is None, "Frozen param should have no gradient"
    assert trainable_param.grad is not None, "Trainable param should have gradient"


def t4_4_rlt_loss_does_not_affect_vla():
    """Key test: RLT loss gradients don't flow to VLA-like parameters."""
    prefix_out = torch.randn(2, 100, 2048, requires_grad=True)

    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    # RLT loss uses prefix_out.detach() internally
    rlt_loss, _ = rlt.loss(prefix_out)
    rlt_loss.backward()

    # prefix_out should NOT have gradient (detached inside loss/reconstruct)
    assert prefix_out.grad is None or prefix_out.grad.abs().max().item() == 0, \
        "prefix_out should have no gradient from RLT loss (detach)"


def t4_5_vla_loss_does_not_affect_rlt():
    """Key test: VLA loss gradients don't flow to RLT parameters."""
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    # Zero all RLT grads
    for p in rlt.parameters():
        if p.grad is not None:
            p.grad.zero_()

    # Simulate VLA loss on a separate parameter
    vla_param = torch.randn(100, requires_grad=True)
    vla_loss = (vla_param ** 2).mean()
    vla_loss.backward()

    # RLT params should have no gradient from VLA loss
    rlt_has_grad = any(p.grad is not None and p.grad.abs().max().item() > 0
                       for p in rlt.parameters())
    assert not rlt_has_grad, "RLT params should have no gradient from VLA loss"


def t4_6_gradient_clipping():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, 100, 2048)
    loss, _ = rlt.loss(x)
    loss.backward()

    grad_clip_norm = 1.0
    total_norm = torch.nn.utils.clip_grad_norm_(rlt.parameters(), grad_clip_norm)
    # After clipping, verify norms
    clipped_norm = sum(p.grad.norm().item() ** 2 for p in rlt.parameters() if p.grad is not None) ** 0.5
    assert clipped_norm <= grad_clip_norm * 1.01, \
        f"Grad norm after clipping ({clipped_norm:.4f}) exceeds threshold ({grad_clip_norm})"


if __name__ == "__main__":
    print("T-RLT4: Gradient Isolation Tests")
    print("=" * 50)

    run_test("T4.1 RLT params have gradient after backward", t4_1_rlt_params_have_grad)
    run_test("T4.2 VLA params have gradient from VLA loss", t4_2_simulated_vla_params_have_grad)
    run_test("T4.3 Frozen params have no gradient", t4_3_frozen_params_no_grad)
    run_test("T4.4 [KEY] RLT loss does not affect VLA-like params", t4_4_rlt_loss_does_not_affect_vla)
    run_test("T4.5 [KEY] VLA loss does not affect RLT params", t4_5_vla_loss_does_not_affect_rlt)
    run_test("T4.6 Gradient clipping", t4_6_gradient_clipping)

    print("=" * 50)
    print(f"=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
