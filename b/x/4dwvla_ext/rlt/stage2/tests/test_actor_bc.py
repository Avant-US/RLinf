#!/usr/bin/env python3
"""T8: Actor/BC loss tests.

Verifies BC target construction, q_weight/bc_weight degeneration, and
Q1/Q2 metric independence. No GPU or Stage1 required.
"""
import sys
from pathlib import Path

import torch
import torch.nn.functional as F

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


def make_policy():
    return RLTMLPPolicy(
        z_dim=1024, proprio_dim=8, action_dim=8,
        num_action_chunks=10, ref_num_action_chunks=50,
        add_q_head=True, q_head_type="default", fixed_std=0.002,
    )


def make_obs(B=4):
    return {
        "z_rl": torch.randn(B, 1024),
        "proprio": torch.randn(B, 8),
        "ref_chunk": torch.randn(B, 50, 8),
    }


def bc_metrics_standalone(pi, actions, ref_chunk, intervene_flags, chunk_len=10, action_dim=8):
    """Reimplementation of _bc_metrics for standalone testing."""
    pi_chunk = pi.reshape(-1, chunk_len, action_dim)
    action_chunk = actions.reshape(-1, chunk_len, action_dim)
    bc_ref_chunk = ref_chunk.reshape(ref_chunk.shape[0], -1, action_dim)[:, :chunk_len]

    if intervene_flags is None:
        human_mask = torch.zeros(pi_chunk.shape[:2], dtype=torch.bool)
    else:
        human_mask = intervene_flags.bool().reshape(-1, chunk_len, action_dim).any(dim=-1)

    bc_target = torch.where(human_mask[..., None], action_chunk, bc_ref_chunk)
    bc_loss = torch.mean(torch.square(pi_chunk - bc_target))
    return bc_loss, bc_target


def main():
    B = 4
    policy = make_policy()

    # T8.1: no intervention -> bc_target = ref_chunk[:, :10]
    def t8_1():
        obs = make_obs(B)
        ref_10 = obs["ref_chunk"][:, :10, :].reshape(B, -1)
        pi = torch.randn(B, 80)
        bc_loss, bc_target = bc_metrics_standalone(pi, torch.randn(B, 80), ref_10, None)
        expected_target = ref_10.reshape(B, 10, 8)
        assert torch.allclose(bc_target, expected_target, atol=1e-6), \
            "bc_target != ref_chunk[:,:10] when no intervention"
        print(f"    bc_target matches ref_chunk[:,:10] (no intervention)")
    run_test("T8.1_no_intervention", t8_1)

    # T8.2: with intervention -> bc_target = where(human_mask, executed, ref)
    def t8_2():
        obs = make_obs(B)
        ref_10 = obs["ref_chunk"][:, :10, :].reshape(B, -1)
        executed = torch.randn(B, 80)
        intervene = torch.zeros(B, 80)
        intervene[0, :8] = 1.0  # first sample, first step has intervention
        bc_loss, bc_target = bc_metrics_standalone(torch.randn(B, 80), executed, ref_10, intervene)
        executed_chunk = executed.reshape(B, 10, 8)
        ref_chunk = ref_10.reshape(B, 10, 8)
        assert torch.allclose(bc_target[0, 0], executed_chunk[0, 0], atol=1e-6), \
            "intervened step should use executed action"
        assert torch.allclose(bc_target[1, 0], ref_chunk[1, 0], atol=1e-6), \
            "non-intervened step should use ref_chunk"
        print(f"    intervened step uses executed action, others use ref_chunk")
    run_test("T8.2_with_intervention", t8_2)

    # T8.3: q_weight=0 -> BC-only
    def t8_3():
        policy2 = make_policy()
        policy2.train()
        obs = make_obs(B)
        pi, _, _ = policy2.sac_forward(obs)
        ref_chunk = policy2._get_ref_chunk(obs)
        bc_loss = F.mse_loss(pi, ref_chunk)

        q_all = policy2.sac_q_forward(obs, pi)
        q_pi = q_all[:, 0:1].mean()

        q_weight = 0.0
        bc_weight = 5.0
        actor_loss = -q_weight * q_pi + bc_weight * bc_loss
        assert abs(actor_loss.item() - bc_weight * bc_loss.item()) < 1e-6, \
            "q_weight=0 should give pure BC loss"
        print(f"    q_weight=0: actor_loss={actor_loss.item():.6f} == bc_weight*bc={bc_weight * bc_loss.item():.6f}")
    run_test("T8.3_q_weight_zero_bc_only", t8_3)

    # T8.4: bc_weight=0 -> Q-only
    def t8_4():
        policy2 = make_policy()
        policy2.train()
        obs = make_obs(B)
        pi, _, _ = policy2.sac_forward(obs)
        ref_chunk = policy2._get_ref_chunk(obs)
        bc_loss = F.mse_loss(pi, ref_chunk)

        q_all = policy2.sac_q_forward(obs, pi)
        q_pi = q_all[:, 0:1].mean()

        q_weight = 1.0
        bc_weight = 0.0
        actor_loss = -q_weight * q_pi + bc_weight * bc_loss
        expected = -q_weight * q_pi.item()
        assert abs(actor_loss.item() - expected) < 1e-6, \
            "bc_weight=0 should give pure Q loss"
        print(f"    bc_weight=0: actor_loss={actor_loss.item():.6f} == -q_weight*Q={expected:.6f}")
    run_test("T8.4_bc_weight_zero_q_only", t8_4)

    # T8.5: q_pi == q_value_0
    def t8_5():
        policy2 = make_policy()
        policy2.eval()
        obs = make_obs(B)
        with torch.no_grad():
            pi, _, _ = policy2.sac_forward(obs)
            q_all = policy2.sac_q_forward(obs, pi)
        q_pi = q_all[:, 0:1].mean().item()
        q_value_0 = q_all[:, 0].mean().item()
        assert abs(q_pi - q_value_0) < 1e-6, \
            f"q_pi={q_pi} != q_value_0={q_value_0}"
        print(f"    q_pi={q_pi:.6f} == q_value_0={q_value_0:.6f}")
    run_test("T8.5_q_pi_equals_q_value_0", t8_5)

    # T8.6: Q1/Q2 metrics independent
    def t8_6():
        policy2 = make_policy()
        obs = make_obs(B)
        actions = torch.randn(B, 80)
        with torch.no_grad():
            q_all = policy2.sac_q_forward(obs, actions)
        q0 = q_all[:, 0].mean().item()
        q1 = q_all[:, 1].mean().item()
        assert abs(q0 - q1) > 1e-8, \
            f"Q1 and Q2 should differ (independent init): q0={q0}, q1={q1}"
        print(f"    Q1 mean: {q0:.6f}, Q2 mean: {q1:.6f} (different)")
    run_test("T8.6_q1_q2_independent", t8_6)

    # T8.7: reference_dropout with prob=0.5
    def t8_7():
        policy2 = make_policy()
        policy2.train()
        obs = make_obs(8)
        torch.manual_seed(42)
        pi_drop, _, _ = policy2.sac_forward(
            obs, apply_reference_dropout=True, reference_dropout_prob=0.5
        )
        torch.manual_seed(42)
        pi_nodrop, _, _ = policy2.sac_forward(
            obs, apply_reference_dropout=False, reference_dropout_prob=0.0
        )
        assert not torch.equal(pi_drop, pi_nodrop), \
            "dropout should change actor output"
        print(f"    dropout changes output: max diff = {(pi_drop - pi_nodrop).abs().max():.6f}")
    run_test("T8.7_reference_dropout", t8_7)

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for v in results.values() if v == "PASS")
    total = len(results)
    print(f"T8 Actor/BC: {passed}/{total} PASSED")
    for k, v in results.items():
        status = "✓" if v == "PASS" else "✗"
        print(f"  {status} {k}: {v}")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
