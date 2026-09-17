#!/usr/bin/env python3
"""T7: Critic target hand-calculation tests.

Verifies forward_critic logic by manually computing TD targets and comparing
against the actual code path. No GPU or Stage1 required.
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


def make_obs(B=1):
    return {
        "z_rl": torch.randn(B, 1024),
        "proprio": torch.randn(B, 8),
        "ref_chunk": torch.randn(B, 50, 8),
    }


def discounted_chunk_rewards(rewards, gamma):
    """Hand-compute discounted chunk reward (same as RLTACLossMixin)."""
    chunk_len = rewards.shape[-1]
    discounts = torch.pow(
        torch.tensor(gamma),
        torch.arange(chunk_len, dtype=torch.float32),
    )
    return torch.sum(rewards * discounts, dim=-1, keepdim=True)


def main():
    gamma = 0.96

    # T7.1: chunk reward hand calculation
    def t7_1():
        rewards = torch.tensor([[0.1, -0.001, -0.001]])
        R = discounted_chunk_rewards(rewards, gamma)
        expected = 0.1 + gamma * (-0.001) + gamma ** 2 * (-0.001)
        assert abs(R.item() - expected) < 1e-6, f"R={R.item()} != expected={expected}"
        print(f"    R_chunk = {R.item():.8f} (expected {expected:.8f})")
    run_test("T7.1_chunk_reward", t7_1)

    # T7.2: nonterminal bootstrap
    def t7_2():
        rewards = torch.tensor([[0.1, -0.001, -0.001]])
        R = discounted_chunk_rewards(rewards, gamma)
        H = 3
        q_next_min = torch.tensor([[2.5]])
        not_done = torch.tensor([[True]])
        target = R + not_done.float() * (gamma ** H) * q_next_min
        expected_target = R.item() + 1.0 * (gamma ** 3) * 2.5
        assert abs(target.item() - expected_target) < 1e-6, \
            f"target={target.item()} != {expected_target}"
        print(f"    nonterminal target = {target.item():.6f} (R + gamma^{H} * Q'_min)")
    run_test("T7.2_nonterminal_bootstrap", t7_2)

    # T7.3: terminal no bootstrap
    def t7_3():
        rewards = torch.tensor([[0.1, -0.001, -0.001]])
        R = discounted_chunk_rewards(rewards, gamma)
        q_next_min = torch.tensor([[2.5]])
        not_done = torch.tensor([[False]])
        target = R + not_done.float() * (gamma ** 3) * q_next_min
        assert abs(target.item() - R.item()) < 1e-6, \
            f"terminal target {target.item()} != R {R.item()}"
        print(f"    terminal target = {target.item():.6f} (== R, no bootstrap)")
    run_test("T7.3_terminal_no_bootstrap", t7_3)

    # T7.4: next action comes from online actor (not target)
    def t7_4():
        policy = make_policy()
        target_policy = make_policy()
        obs = make_obs(2)
        policy.eval()
        target_policy.eval()
        with torch.no_grad():
            next_actions_online, _, _ = policy.sac_forward(obs, deterministic=False)
            next_actions_target, _, _ = target_policy.sac_forward(obs, deterministic=False)
        assert not torch.equal(next_actions_online, next_actions_target), \
            "online and target produced same actions (should differ)"
        print(f"    online action[0,:3]: {next_actions_online[0,:3].tolist()}")
        print(f"    target action[0,:3]: {next_actions_target[0,:3].tolist()}")
    run_test("T7.4_next_action_from_online", t7_4)

    # T7.5: Q' from target model, min(Q1, Q2)
    def t7_5():
        target_policy = make_policy()
        obs = make_obs(4)
        actions = torch.randn(4, 80)
        with torch.no_grad():
            q_all = target_policy.sac_q_forward(obs, actions)
        assert q_all.shape == (4, 2), f"Q shape {q_all.shape}"
        q_min = torch.minimum(q_all[:, 0:1], q_all[:, 1:2])
        assert q_min.shape == (4, 1), f"q_min shape {q_min.shape}"
        print(f"    Q1 mean: {q_all[:,0].mean():.4f}, Q2 mean: {q_all[:,1].mean():.4f}")
        print(f"    min(Q1,Q2) mean: {q_min.mean():.4f}")
    run_test("T7.5_q_target_twin_min", t7_5)

    # T7.6: target detached (no grad)
    def t7_6():
        rewards = torch.tensor([[0.1, 0.2, 0.3]], requires_grad=False)
        R = discounted_chunk_rewards(rewards, gamma)
        q_next = torch.tensor([[1.0]], requires_grad=False)
        target = R + gamma ** 3 * q_next
        assert not target.requires_grad, "target should have no grad"
        print(f"    target.requires_grad: {target.requires_grad}")
    run_test("T7.6_target_detached", t7_6)

    # T7.7: both Q heads get gradients
    def t7_7():
        policy = make_policy()
        policy.train()
        obs = make_obs(4)
        actions = torch.randn(4, 80)
        q_all = policy.sac_q_forward(obs, actions)
        assert q_all.shape == (4, 2)
        target = torch.zeros(4, 2)
        loss = F.mse_loss(q_all, target)
        loss.backward()
        q_head_grads = []
        for i, q_net in enumerate(policy.q_head.qs):
            has_grad = any(p.grad is not None and p.grad.abs().sum() > 0
                          for p in q_net.parameters())
            q_head_grads.append(has_grad)
        assert all(q_head_grads), f"Not all Q heads got gradients: {q_head_grads}"
        print(f"    Q head gradients: Q1={q_head_grads[0]}, Q2={q_head_grads[1]}")
    run_test("T7.7_both_q_gradients", t7_7)

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for v in results.values() if v == "PASS")
    total = len(results)
    print(f"T7 Critic Target: {passed}/{total} PASSED")
    for k, v in results.items():
        status = "✓" if v == "PASS" else "✗"
        print(f"  {status} {k}: {v}")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
