#!/usr/bin/env python3
"""Offline test (0d): verify FK keypoint computation matches dataset ground truth.

Feeds demonstration `observation.state.arm` through FKKeypointComputer.compute()
and compares against the same frame's `observation.keypoint_3d` (56D = 8x7).

Requires pytorch_kinematics — run inside the GPU container with the 4dwvla venv.

    source /opt/venv/4dwvla/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/tests/test_fk_matches_dataset.py
"""
import sys
from pathlib import Path

import numpy as np

_ext_dir = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _ext_dir)
_bx_root = str(Path(__file__).resolve().parents[2])
sys.path.insert(0, _bx_root)

URDF = "/workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf"
KPT_META = "/workspace/RLinf/b/d/frk1/plug/keypoints_meta.json"
DATASET = "/home/nvidia/bt/dt/plug_into_socket_lrb_4D_8sml/data/chunk-000/file-000.parquet"

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


def test_fk_matches_dataset():
    """Compare FK output vs dataset keypoint_3d for sampled frames."""
    import pyarrow.parquet as pq
    from fk_keypoints import FKKeypointComputer

    print("\n=== T5: FK vs Dataset Keypoint Alignment ===")

    if not Path(URDF).exists():
        print(f"  [SKIP] URDF not found: {URDF}")
        return
    if not Path(KPT_META).exists():
        print(f"  [SKIP] Keypoints meta not found: {KPT_META}")
        return
    if not Path(DATASET).exists():
        print(f"  [SKIP] Dataset not found: {DATASET}")
        return

    fk = FKKeypointComputer(URDF, KPT_META, history_max_len=200)
    table = pq.read_table(DATASET)
    n_rows = len(table)

    arm_col = table.column("observation.state.arm")
    kpt_col = table.column("observation.keypoint_3d")

    sample_indices = list(range(0, min(n_rows, 100), 5))
    if n_rows > 100:
        sample_indices += list(range(100, n_rows, 50))

    max_pos_err = 0.0
    max_quat_err = 0.0
    n_checked = 0

    for i in sample_indices:
        arm_q = np.array(arm_col[i].as_py(), dtype=np.float32)
        kpt_gt = np.array(kpt_col[i].as_py(), dtype=np.float32).reshape(8, 7)
        kpt_pred = fk.compute(arm_q)

        pos_err = np.abs(kpt_pred[:, :3] - kpt_gt[:, :3]).max()
        max_pos_err = max(max_pos_err, pos_err)

        for j in range(8):
            q_pred = kpt_pred[j, 3:]
            q_gt = kpt_gt[j, 3:]
            if q_gt[3] < 0:
                q_gt = -q_gt
            quat_err = min(
                np.abs(q_pred - q_gt).max(),
                np.abs(q_pred + q_gt).max(),
            )
            max_quat_err = max(max_quat_err, quat_err)

        n_checked += 1

    print(f"  Checked {n_checked} frames out of {n_rows}")
    print(f"  Max position error: {max_pos_err:.2e}")
    print(f"  Max quaternion error: {max_quat_err:.2e}")

    check(
        "position error < 1e-4",
        max_pos_err < 1e-4,
        f"max_pos_err={max_pos_err:.2e}",
    )
    check(
        "quaternion error < 1e-4 (after hemisphere normalization)",
        max_quat_err < 1e-4,
        f"max_quat_err={max_quat_err:.2e}",
    )


def main():
    test_fk_matches_dataset()
    print(f"\n=== T5 Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)


if __name__ == "__main__":
    main()
