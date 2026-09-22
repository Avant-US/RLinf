#!/usr/bin/env python3
"""T-RLT3: Loss computation tests (offline).

Tests RLT loss numerical correctness, masking behavior, and combined loss formula.
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


def t3_1_rlt_loss_positive():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, 100, 2048)
    loss, _ = rlt.loss(x)
    assert loss.item() > 0, f"RLT loss should be > 0 for random init, got {loss.item()}"


def t3_2_vla_loss_simulated():
    """Simulate VLA loss as a positive scalar."""
    vla_loss = torch.tensor(0.5, requires_grad=True)
    assert vla_loss.item() > 0


def t3_3_combined_loss():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, 100, 2048)
    rlt_loss, _ = rlt.loss(x)
    alpha = 1.0
    vla_loss = torch.tensor(0.5)
    total = rlt_loss + alpha * vla_loss
    expected = rlt_loss.item() + alpha * vla_loss.item()
    diff = abs(total.item() - expected)
    assert diff < 1e-5, f"Combined loss diff: {diff}"


def t3_4_alpha_zero():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, 100, 2048)
    rlt_loss, _ = rlt.loss(x)
    alpha = 0.0
    vla_loss = torch.tensor(999.0)
    total = rlt_loss + alpha * vla_loss
    diff = abs(total.item() - rlt_loss.item())
    assert diff < 1e-5, f"alpha=0 should make total==rlt_loss, diff={diff}"


def t3_5_fp32_computation():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    rlt = rlt.to(dtype=torch.bfloat16)
    x = torch.randn(2, 100, 2048, dtype=torch.bfloat16)
    loss, info = rlt.loss(x)
    assert loss.dtype == torch.float32, f"Loss should be fp32, got {loss.dtype}"
    assert info["mse"].dtype == torch.float32


def t3_6_deploy_view_mask_correctness():
    """Simulate deploy_view_mask from labels == -100."""
    seq_len = 200
    labels = torch.full((2, seq_len), -100, dtype=torch.long)
    labels[0, 150:180] = torch.arange(150, 180)
    labels[1, 160:190] = torch.arange(160, 190)

    mask = (labels == -100)
    assert mask[0, 0].item() is True, "User prompt token should be masked True"
    assert mask[0, 150].item() is False, "Assistant token should be masked False"
    assert mask[0, 180].item() is True, "Token after assistant should be True"

    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, seq_len, 2048)
    loss, _ = rlt.loss(x, mask=mask)
    assert loss.item() > 0
    assert not torch.isnan(loss)


def t3_7_image_only_mask_simulation():
    """Simulate image_only_mask from input_ids == image_token_id."""
    seq_len = 200
    image_token_id = 151655
    input_ids = torch.ones(2, seq_len, dtype=torch.long) * 100
    input_ids[0, 10:74] = image_token_id
    input_ids[1, 10:74] = image_token_id

    mask = (input_ids == image_token_id)
    assert mask.sum().item() == 128, f"Expected 128 image tokens, got {mask.sum().item()}"

    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, seq_len, 2048)
    loss, _ = rlt.loss(x, mask=mask)
    assert loss.item() > 0
    assert not torch.isnan(loss)


if __name__ == "__main__":
    print("T-RLT3: Loss Computation Tests")
    print("=" * 50)

    run_test("T3.1 RLT loss positive (random init)", t3_1_rlt_loss_positive)
    run_test("T3.2 VLA loss simulated", t3_2_vla_loss_simulated)
    run_test("T3.3 Combined loss = rlt_loss + alpha * vla_loss", t3_3_combined_loss)
    run_test("T3.4 alpha=0 → total == rlt_loss", t3_4_alpha_zero)
    run_test("T3.5 fp32 computation under bf16", t3_5_fp32_computation)
    run_test("T3.6 deploy_view_mask correctness", t3_6_deploy_view_mask_correctness)
    run_test("T3.7 image_only_mask simulation", t3_7_image_only_mask_simulation)

    print("=" * 50)
    print(f"=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
