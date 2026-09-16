#!/usr/bin/env python3
"""T-RLT7: Behavior equivalence between ported and original RLT module.

Verifies that the copy in b/x/4dwvla_ext/rlt/ produces identical outputs
to the original in rlinf/models/embodiment/modules/.
"""

import sys
from pathlib import Path

PASS = 0
FAIL = 0

# Paths
EXT_DIR = Path(__file__).resolve().parent.parent
RLINF_ROOT = EXT_DIR.parent.parent.parent.parent  # rlt -> 4dwvla_ext -> x -> b -> RLmm


def run_test(name, fn):
    global PASS, FAIL
    try:
        fn()
        PASS += 1
        print(f"  [PASS] {name}")
    except Exception as e:
        FAIL += 1
        print(f"  [FAIL] {name}: {e}")


def _load_both():
    """Load both ported and original RLT modules."""
    import importlib.util

    # Load ported version
    ported_path = EXT_DIR / "rlt_token_transformer.py"
    spec_p = importlib.util.spec_from_file_location("rlt_ported", str(ported_path))
    mod_p = importlib.util.module_from_spec(spec_p)
    spec_p.loader.exec_module(mod_p)

    # Load original version
    orig_path = RLINF_ROOT / "rlinf" / "models" / "embodiment" / "modules" / "rlt_token_transformer.py"
    spec_o = importlib.util.spec_from_file_location("rlt_orig", str(orig_path))
    mod_o = importlib.util.module_from_spec(spec_o)
    spec_o.loader.exec_module(mod_o)

    return mod_p, mod_o


def _make_pair(mod_p, mod_o, seed=42):
    """Create a pair of RLT modules with shared state_dict."""
    import torch
    torch.manual_seed(seed)
    ported = mod_p.RLTTokenTransformer(
        input_dim=2048, embed_dim=1024, prefix_seq_len=512,
        num_layers=2, num_heads=8, mlp_ratio=4.0)
    orig = mod_o.RLTTokenTransformer(
        input_dim=2048, embed_dim=1024, prefix_seq_len=512,
        num_layers=2, num_heads=8, mlp_ratio=4.0)
    orig.load_state_dict(ported.state_dict())
    ported.eval()
    orig.eval()
    return ported, orig


def t7_1_output_equivalence():
    import torch
    mod_p, mod_o = _load_both()
    ported, orig = _make_pair(mod_p, mod_o)

    torch.manual_seed(123)
    x = torch.randn(2, 200, 2048)
    with torch.no_grad():
        recon_p, z_p = ported.reconstruct(x)
        recon_o, z_o = orig.reconstruct(x)

    diff_recon = (recon_p - recon_o).abs().max().item()
    diff_z = (z_p - z_o).abs().max().item()
    assert diff_recon < 1e-5, f"Reconstruction diff: {diff_recon}"
    assert diff_z < 1e-5, f"z_rl diff: {diff_z}"


def t7_2_loss_equivalence():
    import torch
    mod_p, mod_o = _load_both()
    ported, orig = _make_pair(mod_p, mod_o)

    torch.manual_seed(456)
    x = torch.randn(2, 200, 2048)
    with torch.no_grad():
        loss_p, info_p = ported.loss(x)
        loss_o, info_o = orig.loss(x)

    diff = (loss_p - loss_o).abs().item()
    assert diff < 1e-5, f"Loss diff: {diff}"


def t7_3_param_count():
    mod_p, mod_o = _load_both()
    ported = mod_p.RLTTokenTransformer(
        input_dim=2048, embed_dim=1024, prefix_seq_len=512,
        num_layers=2, num_heads=8)
    orig = mod_o.RLTTokenTransformer(
        input_dim=2048, embed_dim=1024, prefix_seq_len=512,
        num_layers=2, num_heads=8)

    count_p = sum(p.numel() for p in ported.parameters())
    count_o = sum(p.numel() for p in orig.parameters())
    assert count_p == count_o, f"Param count mismatch: ported={count_p} vs orig={count_o}"


def t7_4_state_dict_keys():
    mod_p, mod_o = _load_both()
    ported = mod_p.RLTTokenTransformer(
        input_dim=2048, embed_dim=1024, prefix_seq_len=512,
        num_layers=2, num_heads=8)
    orig = mod_o.RLTTokenTransformer(
        input_dim=2048, embed_dim=1024, prefix_seq_len=512,
        num_layers=2, num_heads=8)

    keys_p = set(ported.state_dict().keys())
    keys_o = set(orig.state_dict().keys())
    assert keys_p == keys_o, f"State dict keys differ: {keys_p.symmetric_difference(keys_o)}"


def t7_5_gradient_equivalence():
    import torch
    mod_p, mod_o = _load_both()
    ported, orig = _make_pair(mod_p, mod_o)
    ported.train()
    orig.train()

    torch.manual_seed(789)
    x = torch.randn(2, 100, 2048)
    loss_p, _ = ported.loss(x)
    loss_p.backward()
    loss_o, _ = orig.loss(x)
    loss_o.backward()

    for (name_p, param_p), (name_o, param_o) in zip(
        ported.named_parameters(), orig.named_parameters()
    ):
        if param_p.grad is not None and param_o.grad is not None:
            grad_diff = (param_p.grad - param_o.grad).abs().max().item()
            assert grad_diff < 1e-4, f"Grad diff for {name_p}: {grad_diff}"


if __name__ == "__main__":
    print("T-RLT7: Behavior Equivalence Tests")
    print("=" * 50)

    run_test("T7.1 Output equivalence (reconstruct)", t7_1_output_equivalence)
    run_test("T7.2 Loss equivalence", t7_2_loss_equivalence)
    run_test("T7.3 Parameter count match", t7_3_param_count)
    run_test("T7.4 State dict keys match", t7_4_state_dict_keys)
    run_test("T7.5 Gradient equivalence", t7_5_gradient_equivalence)

    print("=" * 50)
    print(f"=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
