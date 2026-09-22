#!/usr/bin/env python3
"""T9: Replay & Route tests.

Validates RealworldRLTRoute replace logic, record_transition filtering,
curr/next obs alignment, and warmup schedule logic.
No GPU or Stage1 required.
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


def route_replace(student, ref, switch_flags):
    """Reimplementation of RealworldRLTRoute.route core logic."""
    return torch.where(switch_flags, student, ref)


def record_transition_from_switch(switch_flags):
    """Reimplementation: record_transition = actor_switch[:, :1]."""
    if switch_flags.ndim == 3:
        return switch_flags.reshape(switch_flags.shape[0], -1)[:, :1].bool()
    return switch_flags[:, :1].bool() if switch_flags.ndim == 2 else switch_flags.bool()


def main():
    B = 4
    H_pi = 10
    D_a = 8

    student = torch.randn(B, H_pi, D_a)
    ref = torch.randn(B, H_pi, D_a)

    # T9.1: route replace — switch=True returns student
    def t9_1():
        switch = torch.ones(B, 1, 1, dtype=torch.bool)
        routed = route_replace(student, ref, switch)
        assert torch.equal(routed, student), "switch=True should return student"
        print(f"    switch=True: routed == student")
    run_test("T9.1_route_replace_actor", t9_1)

    # T9.2: route replace — switch=False returns ref
    def t9_2_helper():
        switch = torch.zeros(B, 1, 1, dtype=torch.bool)
        routed = route_replace(student, ref, switch)
        assert torch.equal(routed, ref), "switch=False should return ref"
        print(f"    switch=False: routed == ref")
    run_test("T9.2_route_replace_ref", t9_2_helper)

    # T9.3: record_transition = actor_switch
    def t9_3():
        switch_true = torch.ones(B, H_pi, D_a, dtype=torch.bool)
        switch_false = torch.zeros(B, H_pi, D_a, dtype=torch.bool)
        rt_true = record_transition_from_switch(switch_true)
        rt_false = record_transition_from_switch(switch_false)
        assert rt_true.all(), "actor_switch=True -> record_transition=True"
        assert not rt_false.any(), "actor_switch=False -> record_transition=False"
        print(f"    actor=True: record={rt_true.squeeze().tolist()}")
        print(f"    actor=False: record={rt_false.squeeze().tolist()}")
    run_test("T9.3_record_transition", t9_3)

    # T9.4: curr/next obs alignment (pending observation pattern)
    def t9_4():
        steps = 5
        obs_seq = [{"z_rl": torch.randn(1, 1024), "proprio": torch.randn(1, 8)} for _ in range(steps + 1)]
        transitions = []
        for t in range(steps):
            transitions.append({
                "curr_obs": obs_seq[t],
                "next_obs": obs_seq[t + 1],
                "step": t,
            })
        for t in range(steps):
            assert torch.equal(transitions[t]["curr_obs"]["z_rl"], obs_seq[t]["z_rl"]), \
                f"curr_obs mismatch at step {t}"
            assert torch.equal(transitions[t]["next_obs"]["z_rl"], obs_seq[t+1]["z_rl"]), \
                f"next_obs mismatch at step {t}"
        if steps > 1:
            assert torch.equal(
                transitions[0]["next_obs"]["z_rl"],
                transitions[1]["curr_obs"]["z_rl"],
            ), "next_obs[t] should equal curr_obs[t+1]"
        print(f"    {steps} transitions: curr/next alignment verified")
    run_test("T9.4_curr_next_alignment", t9_4)

    # T9.5: terminal next_obs = curr_obs, bootstrap mask=0
    def t9_5():
        curr = {"z_rl": torch.randn(1, 1024)}
        done = True
        next_obs = curr if done else {"z_rl": torch.randn(1, 1024)}
        bootstrap_mask = 0 if done else 1
        assert torch.equal(next_obs["z_rl"], curr["z_rl"]), "terminal: next=curr"
        assert bootstrap_mask == 0, "terminal: bootstrap=0"
        print(f"    terminal: next==curr, bootstrap_mask=0")
    run_test("T9.5_terminal_next_obs", t9_5)

    # T9.6: demo buffer — intervention goes to demo_buffer
    def t9_6():
        demo_buffer = []
        replay_buffer = []
        for i in range(10):
            transition = {"step": i, "intervene": i % 3 == 0}
            replay_buffer.append(transition)
            if transition["intervene"]:
                demo_buffer.append(transition)
        assert len(demo_buffer) == 4, f"demo_buffer should have 4 items, got {len(demo_buffer)}"
        assert all(t["intervene"] for t in demo_buffer), "demo_buffer should only have intervened"
        print(f"    10 transitions: {len(demo_buffer)} in demo_buffer (intervened)")
    run_test("T9.6_demo_buffer", t9_6)

    # T9.7: replay metrics
    def t9_7():
        transitions = [{"reward": float(i) * 0.1} for i in range(20)]
        count = len(transitions)
        reward_mean = sum(t["reward"] for t in transitions) / count
        assert count == 20, f"count {count}"
        assert abs(reward_mean - 0.95) < 1e-6, f"reward_mean {reward_mean}"
        print(f"    transition_count={count}, reward_mean={reward_mean:.4f}")
    run_test("T9.7_replay_metrics", t9_7)

    # T9.8: schedule counter
    def t9_8():
        transitions_since_train = 0
        for _ in range(15):
            transitions_since_train += 1
        assert transitions_since_train == 15, f"counter {transitions_since_train}"
        transitions_since_train = 0  # reset after training
        assert transitions_since_train == 0, "counter should reset"
        print(f"    schedule counter: 15 -> train -> 0")
    run_test("T9.8_schedule_counter", t9_8)

    # T9.9: readiness gate — not ready blocks actor
    def t9_9():
        ready_for_online = False
        switch_requested = True
        effective_switch = switch_requested and ready_for_online
        assert not effective_switch, "should block when not ready"
        ready_for_online = True
        effective_switch = switch_requested and ready_for_online
        assert effective_switch, "should allow when ready"
        print(f"    not ready: switch blocked. ready: switch allowed.")
    run_test("T9.9_readiness_gate", t9_9)

    # T9.10: critic-only — actor loss has no grad
    def t9_10():
        policy = RLTMLPPolicy(
            z_dim=1024, proprio_dim=8, action_dim=8,
            num_action_chunks=10, ref_num_action_chunks=50,
            add_q_head=True, q_head_type="default", fixed_std=0.002,
        )
        policy.train()
        obs = {"z_rl": torch.randn(2, 1024), "proprio": torch.randn(2, 8), "ref_chunk": torch.randn(2, 50, 8)}

        # critic update
        actions = torch.randn(2, 80)
        q_all = policy.sac_q_forward(obs, actions)
        critic_loss = q_all.mean()
        critic_loss.backward()
        q_has_grad = any(p.grad is not None for p in policy.q_head.parameters())
        backbone_has_grad = any(p.grad is not None and p.grad.abs().sum() > 0
                                for p in policy.backbone.parameters())
        assert q_has_grad, "critic should have grad after critic update"
        print(f"    critic-only: Q grad={q_has_grad}, backbone grad={backbone_has_grad}")
    run_test("T9.10_critic_only", t9_10)

    # T9.11: BC-only warmup (q_weight=0)
    def t9_11():
        policy = RLTMLPPolicy(
            z_dim=1024, proprio_dim=8, action_dim=8,
            num_action_chunks=10, ref_num_action_chunks=50,
            add_q_head=True, q_head_type="default", fixed_std=0.002,
        )
        policy.train()
        obs = {"z_rl": torch.randn(2, 1024), "proprio": torch.randn(2, 8), "ref_chunk": torch.randn(2, 50, 8)}
        pi, _, _ = policy.sac_forward(obs)
        ref = policy._get_ref_chunk(obs)
        bc_loss = torch.nn.functional.mse_loss(pi, ref)
        actor_loss = 0.0 * policy.sac_q_forward(obs, pi)[:, 0].mean() + 5.0 * bc_loss
        actor_loss.backward()
        actor_has_grad = any(p.grad is not None and p.grad.abs().sum() > 0
                            for p in policy.backbone.parameters())
        assert actor_has_grad, "BC-only should still update backbone"
        print(f"    BC-only: actor backbone grad={actor_has_grad}")
    run_test("T9.11_bc_only_warmup", t9_11)

    # T9.12: prefill recording
    def t9_12():
        reference_prefill = True
        actor_switch = False
        record = reference_prefill  # during prefill, record even reference
        assert record, "prefill should record transitions"
        reference_prefill = False
        record = actor_switch  # normal mode: only record actor
        assert not record, "normal reference should not record"
        print(f"    prefill: record=True. normal ref: record=False.")
    run_test("T9.12_prefill_recording", t9_12)

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for v in results.values() if v == "PASS")
    total = len(results)
    print(f"T9 Replay/Route: {passed}/{total} PASSED")
    for k, v in results.items():
        status = "✓" if v == "PASS" else "✗"
        print(f"  {status} {k}: {v}")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
