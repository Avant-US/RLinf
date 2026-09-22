#!/usr/bin/env python3
"""T-RLT5: Checkpoint save/load tests (offline).

Verifies RLT module can be saved and loaded with state_dict roundtrip.
"""

import sys
import tempfile
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


def t5_1_save_creates_file():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    with tempfile.TemporaryDirectory() as td:
        save_path = Path(td) / "rlt_module.pt"
        torch.save(rlt.state_dict(), str(save_path))
        assert save_path.exists(), "rlt_module.pt should exist after save"
        assert save_path.stat().st_size > 0, "rlt_module.pt should not be empty"


def t5_2_state_dict_keys():
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    sd = rlt.state_dict()
    keys = set(sd.keys())
    has_encoder = any(k.startswith("encoder.") for k in keys)
    has_decoder = any(k.startswith("decoder.") for k in keys)
    assert has_encoder, "State dict should contain encoder.* keys"
    assert has_decoder, "State dict should contain decoder.* keys"


def t5_3_load_params_match():
    rlt1 = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                               num_layers=2, num_heads=8)
    with tempfile.TemporaryDirectory() as td:
        save_path = Path(td) / "rlt_module.pt"
        torch.save(rlt1.state_dict(), str(save_path))

        rlt2 = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                                   num_layers=2, num_heads=8)
        sd = torch.load(str(save_path), map_location="cpu", weights_only=True)
        rlt2.load_state_dict(sd)

        for (n1, p1), (n2, p2) in zip(rlt1.named_parameters(), rlt2.named_parameters()):
            diff = (p1 - p2).abs().max().item()
            assert diff < 1e-7, f"Param {n1} differs after load: {diff}"


def t5_4_z_rl_after_load():
    rlt1 = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                               num_layers=2, num_heads=8)
    rlt1.eval()
    torch.manual_seed(42)
    x = torch.randn(2, 100, 2048)

    with torch.no_grad():
        z1 = rlt1.encode_flat(x)

    with tempfile.TemporaryDirectory() as td:
        torch.save(rlt1.state_dict(), str(Path(td) / "rlt_module.pt"))
        rlt2 = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                                   num_layers=2, num_heads=8)
        rlt2.load_state_dict(torch.load(str(Path(td) / "rlt_module.pt"),
                                        map_location="cpu", weights_only=True))
        rlt2.eval()
        with torch.no_grad():
            z2 = rlt2.encode_flat(x)

    diff = (z1 - z2).abs().max().item()
    assert diff < 1e-6, f"z_rl after load differs by {diff}"


def t5_5_no_rlt_keys_in_separate_save():
    """VLA save (simulated) should not contain RLT keys if saved separately."""
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    vla_sd = {"backbone.weight": torch.randn(100, 100)}
    rlt_sd = rlt.state_dict()
    # Verify no overlap
    overlap = set(vla_sd.keys()) & set(rlt_sd.keys())
    assert len(overlap) == 0, f"VLA and RLT state dicts should not overlap: {overlap}"


def t5_6_roundtrip_full():
    """Save → load → forward → z_rl consistency."""
    rlt = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                              num_layers=2, num_heads=8)
    rlt.eval()
    torch.manual_seed(99)
    x = torch.randn(2, 150, 2048)

    with torch.no_grad():
        loss_before, info_before = rlt.loss(x)

    with tempfile.TemporaryDirectory() as td:
        torch.save(rlt.state_dict(), str(Path(td) / "rlt_module.pt"))
        rlt2 = RLTTokenTransformer(input_dim=2048, embed_dim=1024, prefix_seq_len=512,
                                   num_layers=2, num_heads=8)
        rlt2.load_state_dict(torch.load(str(Path(td) / "rlt_module.pt"),
                                        map_location="cpu", weights_only=True))
        rlt2.eval()
        with torch.no_grad():
            loss_after, info_after = rlt2.loss(x)

    diff_loss = abs(loss_before.item() - loss_after.item())
    assert diff_loss < 1e-5, f"Loss roundtrip diff: {diff_loss}"
    diff_z = (info_before["z_rl"] - info_after["z_rl"]).abs().max().item()
    assert diff_z < 1e-5, f"z_rl roundtrip diff: {diff_z}"


if __name__ == "__main__":
    print("T-RLT5: Checkpoint Save/Load Tests")
    print("=" * 50)

    run_test("T5.1 save creates rlt_module.pt", t5_1_save_creates_file)
    run_test("T5.2 state_dict has encoder.* and decoder.* keys", t5_2_state_dict_keys)
    run_test("T5.3 load_state_dict params match", t5_3_load_params_match)
    run_test("T5.4 z_rl consistent after load", t5_4_z_rl_after_load)
    run_test("T5.5 VLA and RLT state dicts don't overlap", t5_5_no_rlt_keys_in_separate_save)
    run_test("T5.6 Full roundtrip (save→load→forward→z_rl)", t5_6_roundtrip_full)

    print("=" * 50)
    print(f"=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
