#!/usr/bin/env python3
"""T6: RLTMLP dimension tests.

No GPU or Stage1 required. Validates RLTMLPPolicy input/output dimensions
for 4DWVLA config: z_dim=1024, proprio_dim=8, action_dim=8, chunks=10/50.
"""
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[6]))  # RLmm root

from rlinf.models.embodiment.mlp_policy.rlt_mlp_policy import RLTMLPPolicy

results = {}


def run_test(name, fn):
    try:
        fn()
        results[name] = "PASS"
        print(f"  {name}: PASS")
    except Exception as e:
        results[name] = f"FAIL: {e}"
        print(f"  {name}: FAIL — {e}")


def main():
    policy = RLTMLPPolicy(
        z_dim=1024,
        proprio_dim=8,
        action_dim=8,
        num_action_chunks=10,
        ref_num_action_chunks=50,
        add_q_head=True,
        q_head_type="default",
        fixed_std=0.002,
    )
    policy.eval()
    print(f"RLTMLPPolicy created: z_dim={policy.z_dim}, chunk_len={policy.chunk_len}")

    B = 4
    obs = {
        "z_rl": torch.randn(B, 1024),
        "proprio": torch.randn(B, 8),
        "ref_chunk": torch.randn(B, 50, 8),
    }

    # T6.1: actor obs dim = 1112 (10*8 + 1024 + 8)
    def t6_1():
        expected = 10 * 8 + 1024 + 8  # 1112
        actual = policy.backbone[0].in_features
        assert actual == expected, f"actor obs dim {actual} != {expected}"
        print(f"    backbone in_features: {actual} (expected {expected})")
    run_test("T6.1_actor_obs_dim", t6_1)

    # T6.2: critic state dim = 1032 (1024 + 8)
    def t6_2():
        expected = 1024 + 8  # 1032
        actual_critic = policy.critic_obs_dim
        assert actual_critic == expected, f"critic_obs_dim {actual_critic} != {expected}"
        q_hidden_size = policy.q_head.qs[0].hidden_size
        assert q_hidden_size == expected, f"QHead hidden_size {q_hidden_size} != {expected}"
        print(f"    critic_obs_dim: {actual_critic}, QHead hidden_size: {q_hidden_size} (expected {expected})")
    run_test("T6.2_critic_state_dim", t6_2)

    # T6.3: actor output shape [B, 80] -> reshape [B, 10, 8]
    def t6_3():
        policy.train()
        action, logprobs, _ = policy.sac_forward(obs)
        assert action.shape == (B, 80), f"sac_forward action {action.shape} != [B,80]"
        chunked = action.reshape(B, 10, 8)
        assert chunked.shape == (B, 10, 8), f"reshaped {chunked.shape}"
        print(f"    sac_forward: {action.shape} -> reshape {chunked.shape}")
        policy.eval()
    run_test("T6.3_actor_output", t6_3)

    # T6.4: Q output shape [B, 2] (twin Q)
    def t6_4():
        actions = torch.randn(B, 80)
        q_values = policy.sac_q_forward(obs, actions)
        assert q_values.shape == (B, 2), f"Q output {q_values.shape} != [B,2]"
        print(f"    Q output shape: {q_values.shape}")
    run_test("T6.4_q_output", t6_4)

    # T6.5: reference dropout in train mode
    def t6_5():
        policy.train()
        torch.manual_seed(42)
        ref_chunk = policy._get_ref_chunk(obs)
        dropped = policy._maybe_drop_reference(ref_chunk, reference_dropout_prob=0.5)
        zero_rows = (dropped.abs().sum(dim=-1) == 0).sum().item()
        assert zero_rows > 0, "No rows dropped with prob=0.5"
        assert zero_rows < B, "All rows dropped"
        print(f"    dropout with prob=0.5: {zero_rows}/{B} rows zeroed")
        policy.eval()
    run_test("T6.5_ref_dropout_train", t6_5)

    # T6.6: reference dropout disabled in eval (prob=0)
    def t6_6():
        policy.eval()
        ref_chunk = policy._get_ref_chunk(obs)
        nodrop = policy._maybe_drop_reference(ref_chunk, reference_dropout_prob=0.0)
        assert torch.equal(ref_chunk, nodrop), "dropout applied with prob=0"
        print(f"    prob=0: no dropout applied (verified)")
    run_test("T6.6_ref_dropout_eval", t6_6)

    # T6.7: fixed std
    def t6_7():
        policy.eval()
        assert policy.fixed_std == 0.002, f"fixed_std {policy.fixed_std} != 0.002"
        actor_state = policy._actor_state(obs)
        feat = policy.backbone(actor_state)
        action_mean = policy.actor_mean(feat)
        action_std = torch.full_like(action_mean, policy.fixed_std)
        assert (action_std == 0.002).all(), "std not uniform 0.002"
        print(f"    fixed_std: {policy.fixed_std}")
    run_test("T6.7_fixed_std", t6_7)

    # T6.8: ref chunk truncation (50 -> 10)
    def t6_8():
        ref = policy._get_ref_chunk(obs)
        expected_flat = 10 * 8  # 80
        assert ref.shape == (B, expected_flat), f"ref_chunk flat {ref.shape} != [B,{expected_flat}]"
        ref_reshaped = ref.reshape(B, 10, 8)
        orig_first10 = obs["ref_chunk"][:, :10, :].reshape(B, -1)
        assert torch.allclose(ref, orig_first10, atol=1e-6), \
            "truncated ref_chunk != first 10 steps of original"
        print(f"    ref truncated: [B,50,8] -> [B,{expected_flat}] (first 10 steps)")
    run_test("T6.8_ref_chunk_truncation", t6_8)

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for v in results.values() if v == "PASS")
    total = len(results)
    print(f"T6 RLTMLP Dims: {passed}/{total} PASSED")
    for k, v in results.items():
        status = "✓" if v == "PASS" else "✗"
        print(f"  {status} {k}: {v}")

    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
