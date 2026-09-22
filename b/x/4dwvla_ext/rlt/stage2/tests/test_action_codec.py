#!/usr/bin/env python3
"""T5: Action Codec tests — FrankaAbsoluteJointCodec.

No GPU or Stage1 required. Can run on host or in container.
"""
import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from action_codec import FrankaAbsoluteJointCodec

STATS_PATHS = [
    "/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/stats.json",       # host
    "/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420/stats.json",         # container
]
STATS_PATH = next((p for p in STATS_PATHS if Path(p).exists()), None)

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
    stats_exists = STATS_PATH is not None
    if stats_exists:
        codec = FrankaAbsoluteJointCodec(stats_path=STATS_PATH)
    else:
        codec = FrankaAbsoluteJointCodec()
    print(f"Codec initialized (from stats: {stats_exists})")
    print(f"  arm_min: {codec.arm_min.tolist()}")
    print(f"  arm_max: {codec.arm_max.tolist()}")
    print(f"  grip_min: {codec.grip_min}, grip_max: {codec.grip_max}")

    # T5.1: arm min -> -1
    def t5_1():
        phys = torch.cat([codec.arm_min, torch.tensor([codec.grip_min])])
        can = codec.encode_physical(phys.unsqueeze(0))
        assert torch.allclose(can[0, :7], torch.full((7,), -1.0), atol=1e-5), \
            f"arm min encoded to {can[0,:7]} not -1"
        print(f"    arm min -> canonical: {can[0,:7].tolist()}")
    run_test("T5.1_arm_min_to_neg1", t5_1)

    # T5.2: arm max -> +1
    def t5_2():
        phys = torch.cat([codec.arm_max, torch.tensor([codec.grip_max])])
        can = codec.encode_physical(phys.unsqueeze(0))
        assert torch.allclose(can[0, :7], torch.full((7,), 1.0), atol=1e-5), \
            f"arm max encoded to {can[0,:7]} not +1"
        print(f"    arm max -> canonical: {can[0,:7].tolist()}")
    run_test("T5.2_arm_max_to_pos1", t5_2)

    # T5.3: gripper 0 -> -1
    def t5_3():
        phys = torch.cat([codec.arm_min, torch.tensor([0.0])])
        can = codec.encode_physical(phys.unsqueeze(0))
        expected_g = 2.0 * (0.0 - codec.grip_min) / (codec.grip_max - codec.grip_min) - 1.0
        assert abs(can[0, 7].item() - expected_g) < 1e-5, \
            f"gripper 0 encoded to {can[0,7]} not {expected_g}"
        print(f"    gripper 0.0 -> canonical: {can[0,7].item():.6f} (expected {expected_g:.6f})")
    run_test("T5.3_gripper_0_to_neg1", t5_3)

    # T5.4: gripper 1 -> +1
    def t5_4():
        phys = torch.cat([codec.arm_min, torch.tensor([1.0])])
        can = codec.encode_physical(phys.unsqueeze(0))
        expected_g = 2.0 * (1.0 - codec.grip_min) / (codec.grip_max - codec.grip_min) - 1.0
        assert abs(can[0, 7].item() - expected_g) < 1e-5, \
            f"gripper 1 encoded to {can[0,7]} not {expected_g}"
        print(f"    gripper 1.0 -> canonical: {can[0,7].item():.6f} (expected {expected_g:.6f})")
    run_test("T5.4_gripper_1_to_pos1", t5_4)

    # T5.5: canonical round-trip
    def t5_5():
        torch.manual_seed(42)
        arm_lo = codec.arm_min.unsqueeze(0).expand(1000, -1)
        arm_hi = codec.arm_max.unsqueeze(0).expand(1000, -1)
        arm_rand = arm_lo + torch.rand(1000, 7) * (arm_hi - arm_lo)
        grip_rand = torch.rand(1000, 1) * (codec.grip_max - codec.grip_min) + codec.grip_min
        phys = torch.cat([arm_rand, grip_rand], dim=-1)
        can = codec.encode_physical(phys)
        recovered = codec.decode_canonical(can)
        max_err = (phys - recovered).abs().max().item()
        assert max_err < 1e-5, f"Round-trip max error {max_err}"
        print(f"    1000 random round-trip max error: {max_err:.2e}")
    run_test("T5.5_canonical_roundtrip", t5_5)

    # T5.6: model 32D -> canonical 8D
    def t5_6():
        import json
        if STATS_PATH is None:
            print("    SKIP: stats.json not found")
            return
        with open(STATS_PATH) as f:
            stats = json.load(f)
        s = stats["franka_plug"]
        arm_mean = torch.tensor(s["action.arm"]["mean"])
        arm_std = torch.tensor(s["action.arm"]["std"])
        grip_mean = torch.tensor(s["action.gripper"]["mean"])
        grip_std = torch.tensor(s["action.gripper"]["std"])
        full_mean = torch.cat([arm_mean, grip_mean, torch.zeros(24)])
        full_std = torch.cat([arm_std, grip_std, torch.ones(24)])

        torch.manual_seed(99)
        action_32d = torch.randn(2, 50, 32) * 0.5
        canonical, metrics = codec.model_to_canonical(action_32d, full_mean, full_std)
        assert canonical.shape == (2, 50, 8), f"Shape {canonical.shape}"
        assert canonical.min() >= -1.5, f"canonical min {canonical.min()}"
        assert canonical.max() <= 1.5, f"canonical max {canonical.max()}"
        print(f"    canonical shape: {canonical.shape}, range: [{metrics['canonical_min']:.3f}, {metrics['canonical_max']:.3f}]")
        print(f"    clip_rate: {metrics['clip_rate']:.4f}, pad_max: {metrics['pad_max']:.3f}")
    run_test("T5.6_model32d_to_canonical", t5_6)

    # T5.7: pad anomaly detection
    def t5_7():
        import json
        if STATS_PATH is None:
            print("    SKIP: stats.json not found")
            return
        with open(STATS_PATH) as f:
            stats = json.load(f)
        s = stats["franka_plug"]
        arm_mean = torch.tensor(s["action.arm"]["mean"])
        arm_std = torch.tensor(s["action.arm"]["std"])
        grip_mean = torch.tensor(s["action.gripper"]["mean"])
        grip_std = torch.tensor(s["action.gripper"]["std"])
        full_mean = torch.cat([arm_mean, grip_mean, torch.zeros(24)])
        full_std = torch.cat([arm_std, grip_std, torch.ones(24)])

        action_32d = torch.zeros(1, 10, 32)
        action_32d[..., 10] = 100.0  # large pad value
        canonical, metrics = codec.model_to_canonical(action_32d, full_mean, full_std)
        assert metrics["pad_max"] > 5.0, f"pad_max {metrics['pad_max']} should trigger warning"
        print(f"    pad anomaly detected: pad_max={metrics['pad_max']:.1f}")
    run_test("T5.7_pad_anomaly", t5_7)

    # T5.8: reference clip rate
    def t5_8():
        torch.manual_seed(42)
        arm_lo = codec.arm_min.unsqueeze(0).unsqueeze(0).expand(10, 50, -1)
        arm_hi = codec.arm_max.unsqueeze(0).unsqueeze(0).expand(10, 50, -1)
        arm_rand = arm_lo + torch.rand(10, 50, 7) * (arm_hi - arm_lo)
        grip_rand = torch.rand(10, 50, 1) * (codec.grip_max - codec.grip_min) + codec.grip_min
        ref_phys = torch.cat([arm_rand, grip_rand], dim=-1)
        ref_can = codec.encode_physical(ref_phys)
        clip_rate = ((ref_can.abs() > 1.0).float().mean()).item()
        assert clip_rate < 0.001, f"clip_rate {clip_rate} >= 0.1%"
        print(f"    reference clip rate: {clip_rate:.6f} (< 0.1%)")
    run_test("T5.8_reference_clip_rate", t5_8)

    # T5.9: env decode gripper polarity
    def t5_9():
        can_close = torch.tensor([[0.0] * 7 + [1.0]])
        can_open = torch.tensor([[0.0] * 7 + [-1.0]])
        phys_close = codec.decode_canonical(can_close)
        phys_open = codec.decode_canonical(can_open)
        assert phys_close[0, 7].item() > 0.9, f"canonical +1 -> gripper {phys_close[0,7]} not close"
        assert phys_open[0, 7].item() < 0.1, f"canonical -1 -> gripper {phys_open[0,7]} not open"
        print(f"    canonical +1 -> gripper {phys_close[0,7]:.4f} (close)")
        print(f"    canonical -1 -> gripper {phys_open[0,7]:.4f} (open)")
    run_test("T5.9_env_decode_gripper", t5_9)

    # T5.10: safety post-decode
    def t5_10():
        can = torch.zeros(1, 8)
        phys = codec.decode_canonical(can)
        arm_mid = (codec.arm_min + codec.arm_max) / 2
        assert torch.allclose(phys[0, :7], arm_mid, atol=1e-5), \
            f"midpoint decode mismatch"
        assert torch.isfinite(phys).all(), "non-finite"
        print(f"    midpoint decode: {phys[0,:7].tolist()}")
    run_test("T5.10_safety_post_decode", t5_10)

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for v in results.values() if v == "PASS")
    total = len(results)
    print(f"T5 Action Codec: {passed}/{total} PASSED")
    for k, v in results.items():
        status = "✓" if v == "PASS" else "✗"
        print(f"  {status} {k}: {v}")

    if passed == total:
        print("\nT5 ALL PASSED")
        return 0
    else:
        print(f"\nT5 FAILED ({total - passed} failures)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
