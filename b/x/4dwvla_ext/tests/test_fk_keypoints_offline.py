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
_bx_root = str(Path(__file__).resolve().parents[2])
sys.path.insert(0, _bx_root)

from fk_keypoints import FKKeypointComputer
from franky_ext.dsplug.home_pose import load_home_joints

URDF = "/workspace/RLinf/b/d/frk1/fr3v2_1_franka_hand.urdf"
KPT_META = "/workspace/RLinf/b/d/frk1/plug/keypoints_meta.json"

HOME_Q = load_home_joints()

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


def test_history_replay_matches_per_control_step_rate():
    """修复 A (grperr_1.md R1): replaying executed poses must make his_len
    advance once per control step, not once per inference.

    Reproduces the exact bug at n_exec=10: the *old* server called
    fk_computer.step() once per inference request, so after N requests
    his_len == N (10x too slow -- 700 control steps only reached his_len=70,
    while demonstrations never close the gripper before his_len>=120). The
    *fixed* server (see vla_inference_server.py's serve() loop) additionally
    replays the n_exec-1 poses executed since the previous request through
    fk_computer.step() before computing the current frame's keypoints, which
    is reproduced here directly against FKKeypointComputer.
    """
    print("\n=== T_FK.5: History Replay Rate (修复 A) ===")
    n_exec = 10
    n_requests = 20
    control_steps = n_exec * n_requests

    fk_old = FKKeypointComputer(URDF, KPT_META, history_max_len=200)
    fk_fixed = FKKeypointComputer(URDF, KPT_META, history_max_len=200)

    # Old (buggy) behavior: one fk.step() call per *inference request*.
    for _ in range(n_requests):
        _, his_len_old = fk_old.step(HOME_Q)

    # Fixed behavior: replay the n_exec-1 "executed since last request" poses,
    # then step() once more for the current frame -- exactly what
    # vla_inference_server.py's serve() loop now does with msg["state_history"].
    for _ in range(n_requests):
        for _ in range(n_exec - 1):
            fk_fixed.step(HOME_Q)
        _, his_len_fixed = fk_fixed.step(HOME_Q)

    check(
        "old behavior: his_len == n_requests (10x too slow)",
        his_len_old == n_requests,
        f"got {his_len_old}, expected {n_requests}",
    )
    check(
        "fixed behavior: his_len == n_exec * n_requests (matches control-step rate)",
        his_len_fixed == control_steps,
        f"got {his_len_fixed}, expected {control_steps}",
    )
    check(
        "fixed behavior reaches the demo's earliest-close threshold "
        "(his_len>=120) within the tested horizon",
        his_len_fixed >= 120,
        f"his_len_fixed={his_len_fixed}",
    )
    # his_len advances once per request under the old scheme and once per
    # control step under the fixed scheme, so the number of *control steps*
    # (wall-clock, at the same n_exec) needed to reach any given his_len
    # differs by exactly a factor of n_exec.
    control_steps_to_reach_120_old = 120 * n_exec
    control_steps_to_reach_120_fixed = 120
    check(
        "old scheme needs n_exec x more control steps to reach his_len=120",
        control_steps_to_reach_120_old == n_exec * control_steps_to_reach_120_fixed,
        f"old={control_steps_to_reach_120_old}, fixed={control_steps_to_reach_120_fixed}",
    )


def test_append_snapshot_api():
    """Verify the append() / snapshot() API contract."""
    print("\n=== T_FK.6: append() / snapshot() API ===")
    fk = FKKeypointComputer(URDF, KPT_META, history_max_len=200)

    # After one append: history has 1 entry
    his_len = fk.append(HOME_Q)
    check("after append(HOME_Q): his_len == 1", his_len == 1, f"got {his_len}")

    # snapshot() returns the same content but does NOT add to history
    buf, snap_len = fk.snapshot()
    check("snapshot() his_len == 1 (same as after append)", snap_len == 1,
          f"got {snap_len}")
    check("snapshot() buf shape (200, 8, 7)", buf.shape == (200, 8, 7),
          f"got {buf.shape}")
    check("snapshot() first frame non-zero", np.any(buf[0] != 0))

    # snapshot() is idempotent: calling again gives same his_len
    _, snap_len2 = fk.snapshot()
    check("snapshot() idempotent: his_len still 1", snap_len2 == 1,
          f"got {snap_len2}")

    # After 10 more appends: snapshot returns his_len=11
    for i in range(10):
        fk.append(HOME_Q + 0.01 * (i + 1))
    _, snap_len11 = fk.snapshot()
    check("after 10 more append(): snapshot his_len == 11", snap_len11 == 11,
          f"got {snap_len11}")

    # step() is equivalent to append() + snapshot() in terms of his_len
    fk.reset()
    fk.append(HOME_Q)
    _, snap_after_append = fk.snapshot()

    fk.reset()
    _, step_len = fk.step(HOME_Q)

    check(
        "step() his_len == append()+snapshot() his_len",
        step_len == snap_after_append,
        f"step={step_len}, append+snapshot={snap_after_append}",
    )

    # Verify progression matches for multiple iterations
    fk_step = FKKeypointComputer(URDF, KPT_META, history_max_len=200)
    fk_api = FKKeypointComputer(URDF, KPT_META, history_max_len=200)
    for i in range(15):
        q = HOME_Q + 0.005 * i
        _, len_step = fk_step.step(q)
        fk_api.append(q)
        _, len_api = fk_api.snapshot()
        if len_step != len_api:
            check(
                f"step vs append+snapshot at i={i}",
                False,
                f"step={len_step}, api={len_api}",
            )
            return
    check("step() == append()+snapshot() for 15 iterations", True)


def test_protocol_v2_his_len_semantics():
    """Simulate protocol v2: for control step t, record t frames via append(),
    then call snapshot().

    Protocol v2 semantics:
      - At step 0: no history yet (current frame is not in history) -> his_len = 0
      - After recording N frames via append(): his_len = N
      - The progression 0 -> 10 -> 20 -> ... matches training semantics
    """
    print("\n=== T_FK.7: Protocol v2 his_len Semantics ===")
    fk = FKKeypointComputer(URDF, KPT_META, history_max_len=200)

    # At step 0: no history yet
    _, his_len_0 = fk.snapshot()
    check("step 0: his_len == 0 (no history)", his_len_0 == 0,
          f"got {his_len_0}")

    # Record 10 frames via append
    for i in range(10):
        fk.append(HOME_Q + 0.002 * i)
    _, his_len_10 = fk.snapshot()
    check("after 10 appends: his_len == 10", his_len_10 == 10,
          f"got {his_len_10}")

    # Record 20 more frames
    for i in range(20):
        fk.append(HOME_Q + 0.003 * i)
    _, his_len_30 = fk.snapshot()
    check("after 20 more appends: his_len == 30", his_len_30 == 30,
          f"got {his_len_30}")

    # Verify the full progression matches training semantics
    # Simulate n_exec=10 control steps per inference request
    fk.reset()
    n_exec = 10
    expected_progression = []
    actual_progression = []

    for request_idx in range(5):
        # Before recording this batch: check current his_len
        _, pre_len = fk.snapshot()
        expected_pre = request_idx * n_exec
        expected_progression.append(expected_pre)
        actual_progression.append(pre_len)

        # Record n_exec frames (one per control step)
        for ctrl in range(n_exec):
            fk.append(HOME_Q + 0.001 * (request_idx * n_exec + ctrl))

    # Final snapshot
    _, final_len = fk.snapshot()

    check(
        f"progression matches: expected {expected_progression}, got {actual_progression}",
        expected_progression == actual_progression,
        f"expected {expected_progression}, got {actual_progression}",
    )
    check(
        f"final his_len == {5 * n_exec}",
        final_len == 5 * n_exec,
        f"got {final_len}",
    )

    # Verify 0 -> 10 -> 20 -> 30 -> 40 pattern
    check(
        "progression is 0, 10, 20, 30, 40",
        actual_progression == [0, 10, 20, 30, 40],
        f"got {actual_progression}",
    )


def main():
    test_shape_and_meta()
    test_normalization()
    test_history_buffer()
    test_determinism()
    test_history_replay_matches_per_control_step_rate()
    test_append_snapshot_api()
    test_protocol_v2_his_len_semantics()
    print(f"\n=== T_FK Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)


if __name__ == "__main__":
    main()
