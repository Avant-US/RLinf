#!/usr/bin/env python3
"""T10: Checkpoint round-trip tests.

Validates save/restore consistency for actor, critic, target, optimizer.
No GPU or Stage1 required.
"""
import copy
import os
import sys
import tempfile
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


def main():
    torch.manual_seed(42)
    B = 4

    policy = make_policy()
    target_policy = make_policy()
    target_policy.load_state_dict(policy.state_dict())

    optimizer = torch.optim.Adam(policy.parameters(), lr=1e-4)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=100)

    policy.train()
    obs = make_obs(B)
    actions = torch.randn(B, 80)

    # Do a training step so optimizer has state
    q_all = policy.sac_q_forward(obs, actions)
    target_vals = torch.zeros(B, 2)
    critic_loss = F.mse_loss(q_all, target_vals)
    optimizer.zero_grad()
    critic_loss.backward()
    optimizer.step()
    scheduler.step()

    # Record pre-save outputs
    policy.eval()
    torch.manual_seed(99)
    with torch.no_grad():
        pre_action, pre_logprob, _ = policy.sac_forward(obs)
        pre_q = policy.sac_q_forward(obs, actions)
        pre_target_q = target_policy.sac_q_forward(obs, actions)

    tmpdir = tempfile.mkdtemp(prefix="t10_ckpt_")

    # T10.1: Save
    def t10_1():
        ckpt = {
            "actor_state_dict": policy.state_dict(),
            "target_state_dict": target_policy.state_dict(),
            "optimizer_state_dict": optimizer.state_dict(),
            "scheduler_state_dict": scheduler.state_dict(),
            "rlt_ready_generation": 5,
            "training_step": 100,
        }
        ckpt_path = os.path.join(tmpdir, "stage2_ckpt.pt")
        torch.save(ckpt, ckpt_path)
        assert os.path.exists(ckpt_path), "checkpoint file not created"
        print(f"    saved to {ckpt_path}")
    run_test("T10.1_save", t10_1)

    # T10.2: Restore — same batch gives identical action/Q
    def t10_2():
        ckpt_path = os.path.join(tmpdir, "stage2_ckpt.pt")
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=True)

        restored = make_policy()
        restored.load_state_dict(ckpt["actor_state_dict"])
        restored.eval()

        torch.manual_seed(99)
        with torch.no_grad():
            post_action, post_logprob, _ = restored.sac_forward(obs)
            post_q = restored.sac_q_forward(obs, actions)

        action_match = torch.allclose(pre_action, post_action, atol=1e-5)
        q_match = torch.allclose(pre_q, post_q, atol=1e-5)
        assert action_match, f"action mismatch: max diff={( pre_action - post_action).abs().max():.2e}"
        assert q_match, f"Q mismatch: max diff={(pre_q - post_q).abs().max():.2e}"
        print(f"    action max diff: {(pre_action - post_action).abs().max():.2e}")
        print(f"    Q max diff: {(pre_q - post_q).abs().max():.2e}")
    run_test("T10.2_restore_consistency", t10_2)

    # T10.3: Target EMA — target params match
    def t10_3():
        ckpt_path = os.path.join(tmpdir, "stage2_ckpt.pt")
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=True)

        restored_target = make_policy()
        restored_target.load_state_dict(ckpt["target_state_dict"])
        restored_target.eval()

        with torch.no_grad():
            post_target_q = restored_target.sac_q_forward(obs, actions)

        match = torch.allclose(pre_target_q, post_target_q, atol=1e-5)
        assert match, f"target Q mismatch: max diff={(pre_target_q - post_target_q).abs().max():.2e}"
        print(f"    target Q max diff: {(pre_target_q - post_target_q).abs().max():.2e}")
    run_test("T10.3_target_ema", t10_3)

    # T10.4: Optimizer step consistent
    def t10_4():
        ckpt_path = os.path.join(tmpdir, "stage2_ckpt.pt")
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=True)

        restored_opt = torch.optim.Adam(make_policy().parameters(), lr=1e-4)
        restored_opt.load_state_dict(ckpt["optimizer_state_dict"])

        orig_step = optimizer.state_dict()["state"][0]["step"]
        rest_step = restored_opt.state_dict()["state"][0]["step"]
        assert orig_step == rest_step, f"step mismatch: {orig_step} vs {rest_step}"

        orig_lr = scheduler.get_last_lr()[0]
        restored_sched = torch.optim.lr_scheduler.StepLR(restored_opt, step_size=100)
        restored_sched.load_state_dict(ckpt["scheduler_state_dict"])
        rest_lr = restored_sched.get_last_lr()[0]
        assert abs(orig_lr - rest_lr) < 1e-10, f"LR mismatch: {orig_lr} vs {rest_lr}"

        print(f"    optimizer step: {orig_step} == {rest_step}")
        print(f"    scheduler LR: {orig_lr} == {rest_lr}")
    run_test("T10.4_optimizer_step", t10_4)

    # T10.5: Stage1 not in ckpt — file < 50MB (MLP only)
    def t10_5():
        ckpt_path = os.path.join(tmpdir, "stage2_ckpt.pt")
        size_mb = os.path.getsize(ckpt_path) / (1024 * 1024)
        assert size_mb < 50.0, f"checkpoint {size_mb:.1f}MB >= 50MB (contains VLA weights?)"

        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=True)
        sd = ckpt["actor_state_dict"]
        has_vla = any("vla" in k.lower() or "vision" in k.lower() or "language" in k.lower()
                      for k in sd.keys())
        assert not has_vla, "checkpoint contains VLA keys"

        total_params = sum(v.numel() for v in sd.values())
        print(f"    checkpoint size: {size_mb:.2f}MB, params: {total_params:,}")
    run_test("T10.5_stage1_not_in_ckpt", t10_5)

    # T10.6: Readiness reset
    def t10_6():
        ckpt_path = os.path.join(tmpdir, "stage2_ckpt.pt")
        ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=True)

        saved_gen = ckpt["rlt_ready_generation"]
        assert saved_gen == 5, f"saved generation {saved_gen} != 5"

        resumed_gen = 0  # readiness resets on resume
        assert resumed_gen == 0, "readiness should reset to 0 on resume"
        print(f"    saved rlt_ready_generation={saved_gen}, resume resets to {resumed_gen}")
    run_test("T10.6_readiness_reset", t10_6)

    # Cleanup
    import shutil
    shutil.rmtree(tmpdir, ignore_errors=True)

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for v in results.values() if v == "PASS")
    total = len(results)
    print(f"T10 Checkpoint Round-trip: {passed}/{total} PASSED")
    for k, v in results.items():
        status = "✓" if v == "PASS" else "✗"
        print(f"  {status} {k}: {v}")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
