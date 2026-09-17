#!/usr/bin/env python3
"""T1: Stage1 strict load tests.

Validates Stage1 artifact integrity checks. Sub-tests that need real
Stage1 outputs are skipped when STAGE1_DIR is unavailable.
"""
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[6]))  # RLmm root

results = {}

STAGE1_DIR = os.environ.get("STAGE1_DIR", "")
STAGE0_DIR_PATHS = [
    "/home/nvidia/ckpts/4wvlaFrk/plug/4wvlaFrkPlugCkp010420",  # container
    "/home/nvidia/bt/ckp/4wvlaFrk/plug/4wvlaFrkPlugCkp010420",  # host
]
STAGE0_DIR = next((p for p in STAGE0_DIR_PATHS if Path(p).exists()), "")


def run_test(name, fn):
    try:
        fn()
        results[name] = "PASS"
        print(f"  {name}: PASS")
    except Exception as e:
        results[name] = f"FAIL: {e}"
        print(f"  {name}: FAIL — {e}")


def verify_stage1_structure(stage1_dir):
    s1 = Path(stage1_dir)
    vla_dir = s1 / "vla"
    rlt_dir = s1 / "rlt"
    if not vla_dir.exists():
        raise FileNotFoundError(f"Missing {vla_dir}")
    if not rlt_dir.exists():
        raise FileNotFoundError(f"Missing {rlt_dir}")
    for f in ["config.json", "model.safetensors", "stats.json"]:
        if not (vla_dir / f).exists():
            raise FileNotFoundError(f"Missing {vla_dir / f}")
    if not (rlt_dir / "rlt_module.pt").exists():
        raise FileNotFoundError(f"Missing {rlt_dir / 'rlt_module.pt'}")
    return True


def load_rlt_module(rlt_path, strict=True):
    sd = torch.load(rlt_path, map_location="cpu", weights_only=True)
    encoder_keys = [k for k in sd if k.startswith("encoder.")]
    decoder_keys = [k for k in sd if k.startswith("decoder.")]
    if strict:
        if len(encoder_keys) == 0:
            raise RuntimeError("No encoder.* keys in rlt_module.pt")
        if len(decoder_keys) == 0:
            raise RuntimeError("No decoder.* keys in rlt_module.pt")
    return sd, encoder_keys, decoder_keys


def main():
    stage1_available = STAGE1_DIR and Path(STAGE1_DIR).exists()
    print(f"STAGE1_DIR: {STAGE1_DIR or '(not set)'} — available: {stage1_available}")
    print(f"STAGE0_DIR: {STAGE0_DIR or '(not found)'}")

    # T1.1: correct Stage1 load
    def t1_1():
        if not stage1_available:
            print("    SKIP: Stage1 outputs not available")
            results["T1.1_correct_load"] = "SKIP"
            return
        verify_stage1_structure(STAGE1_DIR)
        rlt_path = Path(STAGE1_DIR) / "rlt" / "rlt_module.pt"
        sd, enc, dec = load_rlt_module(rlt_path)
        print(f"    VLA + RLT loaded: encoder={len(enc)} keys, decoder={len(dec)} keys")
    run_test("T1.1_correct_load", t1_1)

    # T1.2: missing rlt_module.pt → FileNotFoundError
    def t1_2():
        tmpdir = tempfile.mkdtemp(prefix="t1_mock_")
        try:
            vla_dir = Path(tmpdir) / "vla"
            rlt_dir = Path(tmpdir) / "rlt"
            vla_dir.mkdir()
            rlt_dir.mkdir()
            (vla_dir / "config.json").write_text("{}")
            (vla_dir / "model.safetensors").write_bytes(b"")
            (vla_dir / "stats.json").write_text("{}")
            # rlt_module.pt intentionally missing
            try:
                verify_stage1_structure(tmpdir)
                raise AssertionError("Should have raised FileNotFoundError")
            except FileNotFoundError as e:
                print(f"    correctly raised: {e}")
        finally:
            shutil.rmtree(tmpdir)
    run_test("T1.2_missing_rlt_module", t1_2)

    # T1.3: partial RLT keys → RuntimeError with strict=True
    def t1_3():
        tmpdir = tempfile.mkdtemp(prefix="t1_partial_")
        try:
            rlt_path = Path(tmpdir) / "rlt_module.pt"
            partial_sd = {"decoder.weight": torch.randn(10, 10)}  # no encoder keys
            torch.save(partial_sd, rlt_path)
            try:
                load_rlt_module(rlt_path, strict=True)
                raise AssertionError("Should have raised RuntimeError for missing encoder keys")
            except RuntimeError as e:
                print(f"    correctly raised: {e}")
        finally:
            shutil.rmtree(tmpdir)
    run_test("T1.3_partial_rlt_keys", t1_3)

    # T1.4: z_dim mismatch → size error
    def t1_4():
        from rlinf.models.embodiment.mlp_policy.rlt_mlp_policy import RLTMLPPolicy
        policy = RLTMLPPolicy(
            z_dim=1024, proprio_dim=8, action_dim=8,
            num_action_chunks=10, ref_num_action_chunks=50,
            add_q_head=True, q_head_type="default", fixed_std=0.002,
        )
        wrong_policy = RLTMLPPolicy(
            z_dim=512, proprio_dim=8, action_dim=8,
            num_action_chunks=10, ref_num_action_chunks=50,
            add_q_head=True, q_head_type="default", fixed_std=0.002,
        )
        try:
            wrong_policy.load_state_dict(policy.state_dict(), strict=True)
            raise AssertionError("Should have raised size mismatch error")
        except RuntimeError as e:
            assert "size mismatch" in str(e).lower() or "Error" in str(e), \
                f"Unexpected error: {e}"
            print(f"    correctly raised size mismatch: {str(e)[:120]}")
    run_test("T1.4_z_dim_mismatch", t1_4)

    # T1.5: Stage0 path has no rlt/ directory
    def t1_5():
        if not STAGE0_DIR:
            # Create a mock Stage0 dir
            tmpdir = tempfile.mkdtemp(prefix="t1_stage0_")
            try:
                (Path(tmpdir) / "stats.json").write_text("{}")
                try:
                    verify_stage1_structure(tmpdir)
                    raise AssertionError("Should detect missing vla/ or rlt/ in Stage0")
                except FileNotFoundError as e:
                    print(f"    correctly raised: {e}")
            finally:
                shutil.rmtree(tmpdir)
        else:
            rlt_exists = (Path(STAGE0_DIR) / "rlt").exists()
            assert not rlt_exists, f"Stage0 should not have rlt/ directory"
            try:
                verify_stage1_structure(STAGE0_DIR)
                raise AssertionError("Stage0 should fail Stage1 validation")
            except FileNotFoundError as e:
                print(f"    Stage0 correctly rejected: {e}")
    run_test("T1.5_stage0_rejected", t1_5)

    # T1.6: VLA config consistency
    def t1_6():
        if not stage1_available:
            print("    SKIP: Stage1 outputs not available")
            results["T1.6_config_consistency"] = "SKIP"
            return
        cfg_path = Path(STAGE1_DIR) / "vla" / "config.json"
        with open(cfg_path) as f:
            cfg = json.load(f)
        assert cfg.get("enable_keypoint_predictor") == True, \
            f"enable_keypoint_predictor={cfg.get('enable_keypoint_predictor')}"
        assert cfg.get("chunk_size") == 50, f"chunk_size={cfg.get('chunk_size')}"
        assert cfg.get("type") == "internvla_a1_5", f"type={cfg.get('type')}"
        print(f"    config: keypoint={cfg['enable_keypoint_predictor']}, chunk={cfg['chunk_size']}, type={cfg['type']}")
    run_test("T1.6_config_consistency", t1_6)

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for v in results.values() if v == "PASS")
    skipped = sum(1 for v in results.values() if v == "SKIP")
    total = len(results)
    print(f"T1 Stage1 Strict Load: {passed}/{total} PASSED, {skipped} SKIPPED")
    for k, v in results.items():
        status = "✓" if v == "PASS" else ("⊘" if v == "SKIP" else "✗")
        print(f"  {status} {k}: {v}")

    failures = sum(1 for v in results.values() if v.startswith("FAIL"))
    return 1 if failures > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
