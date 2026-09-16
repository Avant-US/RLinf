#!/usr/bin/env python3
"""Offline test: verify FK keypoint computation matches training conventions.

Run inside the GPU container with the 4dwvla venv activated.

    source /opt/venv/4dwvla/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/tests/test_fk_keypoints_offline.py
"""
import sys
from pathlib import Path

import numpy as np

_ext_dir = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _ext_dir)

from fk_keypoints import FKKeypointComputer

URDF = "/workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf"
KPT_META = "/workspace/RLinf/b/d/frk1/plug/keypoints_meta.json"

HOME_Q = np.array([-0.24, 0.15, 0.19, -2.06, -0.06, 2.20, 0.70])

PASS = 0
FAIL = 0


def check(name: str, condition: bool, detail: str = ""):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}: {detail}")


def test_shape_and_meta():
    print("\n=== T_FK.1: Shape and Metadata ===")
    fk = FKKeypointComputer(URDF, KPT_META, history_max_len=200)
    check("num_joints == 8", fk.num_joints == 8, f"got {fk.num_joints}")
    check("kpt_dim == 7 (pos_rot)", fk.kpt_dim == 7, f"got {fk.kpt_dim}")
    check("history_max_len == 200", fk.history_max_len == 200)

    kpt = fk.compute(HOME_Q)
    check("output shape (8, 7)", kpt.shape == (8, 7), f"got {kpt.shape}")


def test_normalization():
    print("\n=== T_FK.2: Normalization Conventions ===")
    fk = FKKeypointComputer(URDF, KPT_META, history_max_len=200)
    kpt = fk.compute(HOME_Q)

    for j in range(8):
        quat = kpt[j, 3:]
        qnorm = np.linalg.norm(quat)
        check(
            f"joint{j} quat unit norm",
            np.isclose(qnorm, 1.0, atol=1e-5),
            f"norm={qnorm}",
        )
        check(f"joint{j} qw >= 0 (hemisphere)", quat[3] >= 0, f"qw={quat[3]}")

    pos_all = kpt[:, :3]
    max_pos = np.abs(pos_all).max()
    check(
        "positions in reasonable range (< 2.0 after /bbox_radius)",
        max_pos < 2.0,
        f"max={max_pos}",
    )


def test_history_buffer():
    print("\n=== T_FK.3: History Buffer ===")
    fk = FKKeypointComputer(URDF, KPT_META, history_max_len=200)

    his, hlen = fk.step(HOME_Q)
    check("after 1 step: his_len == 1", hlen == 1)
    check("his shape (200, 8, 7)", his.shape == (200, 8, 7), f"got {his.shape}")
    check("first frame non-zero", np.any(his[0] != 0))
    check("second frame zero (padding)", np.all(his[1] == 0))

    for i in range(9):
        fk.step(HOME_Q + 0.01 * i)
    _, hlen10 = fk.step(HOME_Q)
    check("after 11 steps: his_len == 11", hlen10 == 11)

    fk.reset()
    _, hlen_reset = fk.step(HOME_Q)
    check("after reset + 1 step: his_len == 1", hlen_reset == 1)


def test_determinism():
    print("\n=== T_FK.4: Determinism ===")
    fk = FKKeypointComputer(URDF, KPT_META, history_max_len=200)
    kpt1 = fk.compute(HOME_Q)
    kpt2 = fk.compute(HOME_Q)
    check("same input → same output", np.allclose(kpt1, kpt2, atol=1e-7))


def main():
    test_shape_and_meta()
    test_normalization()
    test_history_buffer()
    test_determinism()
    print(f"\n=== T_FK Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)


if __name__ == "__main__":
    main()
