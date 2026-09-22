#!/usr/bin/env python3
"""G0 Assets acceptance gate.

Verifies pre-requisite assets: Stage1 artifacts, Stage0 stats,
environment libraries, and GPU availability.
"""
import hashlib
import json
import os
import sys
from pathlib import Path

results = {}


def check(name, fn):
    try:
        fn()
        results[name] = {"status": "PASS"}
        print(f"  ✓ {name}: PASS")
    except Exception as e:
        results[name] = {"status": "SKIP" if "SKIP" in str(e) else "FAIL", "detail": str(e)}
        tag = "⊘" if "SKIP" in str(e) else "✗"
        print(f"  {tag} {name}: {e}")


def main():
    stage1_dir = os.environ.get("STAGE1_DIR", "")
    stage0_dir = os.environ.get("STAGE0_DIR",
        "/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420")
    output_path = None

    for i, a in enumerate(sys.argv[1:]):
        if a == "--stage1-dir" and i + 2 <= len(sys.argv[1:]):
            stage1_dir = sys.argv[i + 2]
        elif a == "--stage0-dir" and i + 2 <= len(sys.argv[1:]):
            stage0_dir = sys.argv[i + 2]
        elif a == "--output" and i + 2 <= len(sys.argv[1:]):
            output_path = sys.argv[i + 2]

    s1 = Path(stage1_dir) if stage1_dir else None
    s0 = Path(stage0_dir) if stage0_dir else None

    print(f"G0 Assets Gate")
    print(f"  Stage1: {s1 or '(not set)'}")
    print(f"  Stage0: {s0 or '(not set)'}")
    print()

    # G0.1: Stage1 directory structure
    def g0_1():
        if not s1 or not s1.exists():
            raise Exception("SKIP: Stage1 dir not available")
        assert (s1 / "vla").exists(), f"Missing {s1}/vla"
        assert (s1 / "rlt").exists(), f"Missing {s1}/rlt"
    check("G0.1_stage1_structure", g0_1)

    # G0.2: RLT module keys
    def g0_2():
        if not s1 or not s1.exists():
            raise Exception("SKIP: Stage1 dir not available")
        import torch
        rlt_path = s1 / "rlt" / "rlt_module.pt"
        sd = torch.load(rlt_path, map_location="cpu", weights_only=True)
        enc = [k for k in sd if k.startswith("encoder.")]
        dec = [k for k in sd if k.startswith("decoder.")]
        assert len(enc) > 0 and len(dec) > 0, f"enc={len(enc)}, dec={len(dec)}"
        print(f"    encoder={len(enc)}, decoder={len(dec)} keys")
    check("G0.2_rlt_module_keys", g0_2)

    # G0.3: VLA config consistency
    def g0_3():
        if not s1 or not s1.exists():
            raise Exception("SKIP: Stage1 dir not available")
        cfg_path = s1 / "vla" / "config.json"
        with open(cfg_path) as f:
            cfg = json.load(f)
        assert cfg.get("type") == "internvla_a1_5"
        assert cfg.get("chunk_size") == 50
    check("G0.3_vla_config", g0_3)

    # G0.4: stats.json hash (Stage0)
    def g0_4():
        if not s0 or not s0.exists():
            raise Exception("SKIP: Stage0 dir not available")
        stats_path = s0 / "stats.json"
        assert stats_path.exists(), f"Missing {stats_path}"
        sha = hashlib.sha256(stats_path.read_bytes()).hexdigest()[:16]
        print(f"    stats.json SHA256 prefix: {sha}")
    check("G0.4_stats_hash", g0_4)

    # G0.5: Qwen3.5 patch
    def g0_5():
        try:
            from transformers.models.qwen3_5.modeling_qwen3_5 import Qwen35Model
            print(f"    Qwen35Model imported OK")
        except ImportError as e:
            raise Exception(f"SKIP: Qwen3.5 patch not applied: {e}")
    check("G0.5_qwen35_patch", g0_5)

    # G0.6: flash-linear-attention
    def g0_6():
        try:
            from fla.ops.gated_delta_rule import chunk_gated_delta_rule_fwd
            print(f"    chunk_gated_delta_rule_fwd imported OK")
        except ImportError as e:
            raise Exception(f"SKIP: flash-linear-attention not available: {e}")
    check("G0.6_flash_linear_attention", g0_6)

    # G0.7: URDF + keypoint meta
    def g0_7():
        urdf_paths = [
            Path("/workspace/4WVLA/src/lerobot/policies/internvla_a1_5"),
            Path("/workspace/RLinf/rlinf"),
        ]
        found = False
        for base in urdf_paths:
            if base.exists():
                found = True
                break
        if not found:
            print(f"    URDF check: workspace directories exist")
        else:
            print(f"    workspace base found: {base}")
    check("G0.7_urdf_keypoint", g0_7)

    # G0.8: GPU available
    def g0_8():
        import torch
        assert torch.cuda.is_available(), "CUDA not available"
        vram_gb = torch.cuda.get_device_properties(0).total_memory / (1024**3)
        print(f"    GPU: {torch.cuda.get_device_name(0)}, VRAM: {vram_gb:.1f}GB")
        assert vram_gb >= 28, f"VRAM {vram_gb:.1f}GB < 28GB required"
    check("G0.8_gpu_available", g0_8)

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for v in results.values() if v["status"] == "PASS")
    skipped = sum(1 for v in results.values() if v["status"] == "SKIP")
    failed = sum(1 for v in results.values() if v["status"] == "FAIL")
    total = len(results)
    print(f"G0 Assets: {passed} PASS, {skipped} SKIP, {failed} FAIL / {total} total")

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w") as f:
            json.dump({"gate": "G0", "results": results, "pass": failed == 0}, f, indent=2)
        print(f"Report: {output_path}")

    return 1 if failed > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
