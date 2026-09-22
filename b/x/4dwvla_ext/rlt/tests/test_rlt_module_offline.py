#!/usr/bin/env python3
"""T-RLT1: RLT module unit tests (offline, no real robot needed).

Tests RLTTokenTransformer construction, forward pass shapes, loss computation,
and numerical properties.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import torch
from rlt_token_transformer import (
    GeGLU,
    RLTSelfAttentionLayer,
    RLTTokenDecoder,
    RLTTokenEncoder,
    RLTTokenTransformer,
)

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


def t1_1_encoder_construction():
    enc = RLTTokenEncoder(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                          num_layers=2, num_heads=8, mlp_ratio=4.0)
    param_count = sum(p.numel() for p in enc.parameters())
    assert param_count > 50_000_000, f"Expected >50M params, got {param_count/1e6:.1f}M"
    assert param_count < 150_000_000, f"Expected <150M params, got {param_count/1e6:.1f}M"


def t1_2_decoder_construction():
    dec = RLTTokenDecoder(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                          num_layers=2, num_heads=8, mlp_ratio=4.0)
    param_count = sum(p.numel() for p in dec.parameters())
    assert param_count > 50_000_000, f"Expected >50M params, got {param_count/1e6:.1f}M"


def t1_3_transformer_construction():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8, mlp_ratio=4.0)
    param_count = sum(p.numel() for p in rlt.parameters())
    assert param_count > 100_000_000, f"Expected >100M params, got {param_count/1e6:.1f}M"
    assert param_count < 300_000_000, f"Expected <300M params, got {param_count/1e6:.1f}M"


def t1_4_encoder_forward_no_mask():
    enc = RLTTokenEncoder(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                          num_layers=2, num_heads=8)
    x = torch.randn(2, 200, 2048)
    out = enc(x)
    assert out.shape == (2, 1, 1024), f"Expected (2,1,1024), got {out.shape}"


def t1_5_encoder_forward_with_mask():
    enc = RLTTokenEncoder(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                          num_layers=2, num_heads=8)
    x = torch.randn(2, 200, 2048)
    mask = torch.ones(2, 200, dtype=torch.bool)
    mask[0, 150:] = False
    out = enc(x, mask=mask)
    assert out.shape == (2, 1, 1024), f"Expected (2,1,1024), got {out.shape}"


def t1_6_encoder_seq_len_exceeds():
    enc = RLTTokenEncoder(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                          num_layers=2, num_heads=8)
    x = torch.randn(2, 600, 2048)
    try:
        enc(x)
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "exceeds" in str(e).lower(), f"Unexpected error: {e}"


def t1_7_decoder_forward():
    dec = RLTTokenDecoder(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                          num_layers=2, num_heads=8)
    rl_tokens = torch.randn(2, 1, 1024)
    target = torch.randn(2, 200, 2048)
    out = dec(rl_tokens, target)
    assert out.shape == (2, 200, 2048), f"Expected (2,200,2048), got {out.shape}"


def t1_8_encode_flat():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, 200, 2048)
    z = rlt.encode_flat(x)
    assert z.shape == (2, 1024), f"Expected (2,1024), got {z.shape}"


def t1_9_loss_returns():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, 200, 2048)
    loss, info = rlt.loss(x)
    assert loss.dim() == 0, f"Loss should be scalar, got dim={loss.dim()}"
    assert loss.item() > 0, f"Loss should be >0 for random init, got {loss.item()}"
    assert "mse" in info
    assert "z_rl" in info


def t1_10_loss_fp32():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, 100, 2048)
    loss, info = rlt.loss(x)
    assert loss.dtype == torch.float32, f"Loss dtype should be float32, got {loss.dtype}"


def t1_11_bf16_input():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    rlt = rlt.to(dtype=torch.bfloat16)
    x = torch.randn(2, 100, 2048, dtype=torch.bfloat16)
    loss, info = rlt.loss(x)
    assert loss.dtype == torch.float32, f"Loss should be fp32 even with bf16 input, got {loss.dtype}"
    assert not torch.isnan(loss), "Loss should not be NaN with bf16"


def t1_12_masked_loss():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    x = torch.randn(2, 100, 2048)

    mask_all_true = torch.ones(2, 100, dtype=torch.bool)
    loss_all, _ = rlt.loss(x, mask=mask_all_true)
    assert loss_all.item() > 0, "Loss with all-True mask should be >0"

    mask_all_false = torch.zeros(2, 100, dtype=torch.bool)
    loss_none, _ = rlt.loss(x, mask=mask_all_false)
    assert loss_none.item() == 0.0, f"Loss with all-False mask should be 0, got {loss_none.item()}"


def t1_13_determinism():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    rlt.eval()
    x = torch.randn(2, 100, 2048)
    with torch.no_grad():
        loss1, info1 = rlt.loss(x)
        loss2, info2 = rlt.loss(x)
    diff = (loss1 - loss2).abs().item()
    assert diff < 1e-6, f"Determinism failed: diff={diff}"


if __name__ == "__main__":
    print("T-RLT1: RLT Module Unit Tests")
    print("=" * 50)

    run_test("T1.1 Encoder construction (embed_dim=1024, input_dim=2048)", t1_1_encoder_construction)
    run_test("T1.2 Decoder construction", t1_2_decoder_construction)
    run_test("T1.3 Transformer construction", t1_3_transformer_construction)
    run_test("T1.4 Encoder forward (no mask)", t1_4_encoder_forward_no_mask)
    run_test("T1.5 Encoder forward (with mask)", t1_5_encoder_forward_with_mask)
    run_test("T1.6 Encoder seq_len exceeds prefix_seq_len", t1_6_encoder_seq_len_exceeds)
    run_test("T1.7 Decoder forward shape", t1_7_decoder_forward)
    run_test("T1.8 encode_flat → flat z_rl", t1_8_encode_flat)
    run_test("T1.9 loss() returns (scalar, dict)", t1_9_loss_returns)
    run_test("T1.10 loss() fp32 conversion", t1_10_loss_fp32)
    run_test("T1.11 bf16 input stability", t1_11_bf16_input)
    run_test("T1.12 masked loss (all True vs all False)", t1_12_masked_loss)
    run_test("T1.13 determinism check", t1_13_determinism)

    print("=" * 50)
    print(f"=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
