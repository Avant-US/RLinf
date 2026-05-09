# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the CLI backend layer.

Per design doc ``test_galaxea_r1_pro_cli_controller.md`` §9.2.  Covers
:class:`DummyBackend` exhaustively (CI runs it), and verifies that
:class:`RayBackend` / :class:`RclpyBackend` constructed in dummy mode
satisfy the same ABC and produce identical TX log shape -- the
"duck-type equivalence" property the design doc relies on.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

# Make toolkits importable as a top-level package (matches how the CLI
# entrypoint runs).  PYTHONPATH is set in CI; locally we add the path
# at collection time.
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from toolkits.realworld_check._galaxea_backends import (  # noqa: E402
    ControllerBackend,
    DummyBackend,
    RayBackend,
    RclpyBackend,
    build_backend,
)

# ───────────────────────── DummyBackend ────────────────────────


def test_dummy_backend_kind_is_dummy():
    be = DummyBackend()
    assert be.kind == "dummy"


def test_dummy_backend_send_arm_joints_records_tx_log():
    be = DummyBackend()
    be.send_arm_joints(
        "right", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], [3, 3, 3, 3, 5, 5, 5]
    )
    assert len(be.tx_log) == 1
    rec = be.tx_log[0]
    assert rec["type"] == "arm_joints"
    assert rec["side"] == "right"
    np.testing.assert_allclose(rec["q_target"], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    assert rec["qvel_max"] == [3.0, 3.0, 3.0, 3.0, 5.0, 5.0, 5.0]


def test_dummy_backend_send_arm_joints_reflects_state():
    """Dummy mode mirrors the command back into the synthetic state so
    that consecutive ``state`` queries show progress -- improves UX
    when running ``--dummy``."""
    be = DummyBackend()
    q = [0.5, 0.3, 0.0, -1.5, 0.0, 1.8, 0.0]
    be.send_arm_joints("right", q, [3] * 7)
    state = be.get_state()
    np.testing.assert_allclose(state.right_arm_qpos, q, atol=1e-6)


def test_dummy_backend_send_arm_pose_records_and_reflects():
    be = DummyBackend()
    pose = [0.4, -0.1, 0.3, 0.0, 0.0, 0.0, 1.0]
    be.send_arm_pose("right", pose)
    assert be.tx_log[0]["type"] == "arm_pose"
    np.testing.assert_allclose(be.tx_log[0]["pose"], pose)
    np.testing.assert_allclose(be.get_state().right_ee_pose, pose)


def test_dummy_backend_send_gripper_clips_to_physical_range():
    be = DummyBackend()
    # Defensive clip to [0, 100] even if upstream sent a wild value.
    be.send_gripper("right", 150.0)
    assert be.tx_log[0]["pct"] == pytest.approx(100.0)
    be.send_gripper("right", -5.0)
    assert be.tx_log[1]["pct"] == pytest.approx(0.0)
    be.send_gripper("right", 45.0)
    assert be.tx_log[2]["pct"] == pytest.approx(45.0)


def test_dummy_backend_send_gripper_reflects_into_state():
    be = DummyBackend()
    be.send_gripper("right", 42.0)
    assert be.get_state().right_gripper_pos == pytest.approx(42.0)


def test_dummy_backend_apply_brake_records():
    be = DummyBackend()
    be.apply_brake(True)
    be.apply_brake(False)
    assert be.tx_log[0]["type"] == "brake"
    assert be.tx_log[0]["on"] is True
    assert be.tx_log[1]["on"] is False


def test_dummy_backend_get_subscription_count_always_one():
    be = DummyBackend()
    # All topics return 1 in dummy mode (so topo check passes in CI).
    assert be.get_subscription_count("/anything") == 1
    assert be.get_subscription_count("/other") == 1


def test_dummy_backend_get_publisher_count_always_one():
    be = DummyBackend()
    assert be.get_publisher_count("/anything") == 1


def test_dummy_backend_get_state_is_alive_true():
    """L5 watchdog needs is_alive=True in dummy mode so heartbeat-only
    tests don't false-fail."""
    be = DummyBackend()
    assert be.get_state().is_alive is True


def test_dummy_backend_set_state_replaces_snapshot():
    from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (
        GalaxeaR1ProRobotState,
    )

    be = DummyBackend()
    custom = GalaxeaR1ProRobotState()
    custom.right_arm_qpos = np.full(7, 0.5, dtype=np.float32)
    be.set_state(custom)
    np.testing.assert_allclose(be.get_state().right_arm_qpos, np.full(7, 0.5))


def test_dummy_backend_patch_state_partial_update():
    be = DummyBackend()
    be.patch_state(right_gripper_pos=77.0)
    assert be.get_state().right_gripper_pos == pytest.approx(77.0)


def test_dummy_backend_shutdown_idempotent():
    be = DummyBackend()
    be.shutdown()
    be.shutdown()  # must not raise


def test_dummy_backend_send_arm_joints_left_side_state():
    be = DummyBackend()
    q = [0.1] * 7
    be.send_arm_joints("left", q, [3] * 7)
    np.testing.assert_allclose(be.get_state().left_arm_qpos, q)


def test_dummy_backend_implements_abc():
    """Every abstract method on ControllerBackend must be implemented."""
    be = DummyBackend()
    assert isinstance(be, ControllerBackend)


# ──────────────────────── RayBackend (dummy) ───────────────────


def test_ray_backend_dummy_does_not_init_ray():
    """RayBackend(is_dummy=True) must not call ray.init -- it should
    be safe to construct without ray installed."""
    be = RayBackend(is_dummy=True)
    assert be.kind == "ray"


def test_ray_backend_dummy_send_arm_joints_works():
    be = RayBackend(is_dummy=True)
    be.send_arm_joints("right", [0.1] * 7, [3] * 7)
    np.testing.assert_allclose(
        be.get_state().right_arm_qpos,
        [0.1] * 7,
    )


def test_ray_backend_dummy_send_arm_pose_works():
    be = RayBackend(is_dummy=True)
    pose = [0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
    be.send_arm_pose("right", pose)
    np.testing.assert_allclose(be.get_state().right_ee_pose, pose)


def test_ray_backend_dummy_send_gripper_works():
    be = RayBackend(is_dummy=True)
    be.send_gripper("right", 50.0)
    assert be.get_state().right_gripper_pos == pytest.approx(50.0)


def test_ray_backend_dummy_apply_brake_works():
    be = RayBackend(is_dummy=True)
    be.apply_brake(True)  # no raise -- delegates to dummy


def test_ray_backend_dummy_topo_count_one():
    be = RayBackend(is_dummy=True)
    assert be.get_subscription_count("/whatever") == 1


def test_ray_backend_dummy_shutdown_safe():
    be = RayBackend(is_dummy=True)
    be.shutdown()
    be.shutdown()


# ──────────────────────── RclpyBackend (dummy) ──────────────────


def test_rclpy_backend_dummy_does_not_init_ros():
    """RclpyBackend(is_dummy=True) must NOT import rclpy -- it should
    be importable on machines with no ROS."""
    be = RclpyBackend(is_dummy=True)
    assert be.kind == "rclpy"


def test_rclpy_backend_dummy_send_arm_joints_in_tx_log():
    be = RclpyBackend(is_dummy=True)
    be.send_arm_joints("right", [0.1] * 7, [3] * 7)
    assert len(be.tx_log) == 1
    assert be.tx_log[0]["type"] == "arm_joints"


def test_rclpy_backend_dummy_send_gripper_clips():
    be = RclpyBackend(is_dummy=True)
    be.send_gripper("right", 150.0)
    assert be.tx_log[0]["pct"] == pytest.approx(100.0)


def test_rclpy_backend_dummy_apply_brake():
    be = RclpyBackend(is_dummy=True)
    be.apply_brake(True)
    assert be.tx_log[0]["type"] == "brake"
    assert be.tx_log[0]["on"] is True


def test_rclpy_backend_dummy_topo_count_one():
    be = RclpyBackend(is_dummy=True)
    assert be.get_subscription_count("/whatever") == 1
    assert be.get_publisher_count("/whatever") == 1


def test_rclpy_backend_dummy_shutdown_idempotent():
    be = RclpyBackend(is_dummy=True)
    be.shutdown()
    be.shutdown()


# ───────────────────────── Factory ──────────────────────────────


def test_build_backend_dummy_kind():
    be = build_backend(kind="dummy")
    assert be.kind == "dummy"


def test_build_backend_is_dummy_overrides_kind():
    """is_dummy=True wins regardless of kind."""
    be = build_backend(kind="ray", is_dummy=True)
    assert be.kind == "dummy"


def test_build_backend_ray_dummy_when_dummy_flag():
    """When kind='ray' but is_dummy=True, returns DummyBackend (not
    RayBackend in dummy mode), to keep CI deterministic."""
    be = build_backend(kind="ray", is_dummy=True)
    assert isinstance(be, DummyBackend)


def test_build_backend_rclpy_dummy_when_dummy_flag():
    be = build_backend(kind="rclpy", is_dummy=True)
    assert isinstance(be, DummyBackend)


def test_build_backend_unknown_kind_raises():
    with pytest.raises(ValueError, match="Unknown backend"):
        build_backend(kind="bogus")


def test_build_backend_kind_case_insensitive():
    be = build_backend(kind="DUMMY")
    assert be.kind == "dummy"
    be2 = build_backend(kind="  Dummy ")
    assert be2.kind == "dummy"


# ─────────────── Duck-type equivalence: dispatcher ──────────────


def test_dummy_backend_works_as_dispatcher_controller():
    """Critical equivalence: JointStateDispatcher must accept a
    DummyBackend in place of a real GalaxeaR1ProController, because
    the whole CLI design rests on this duck-typing."""
    from rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher import (
        JointStateDispatcher,
    )
    from rlinf.envs.realworld.galaxear.r1_pro_gripper_mixer import (
        GripperMixer,
    )

    be = DummyBackend(use_right_arm=True)
    mixer = GripperMixer(0.0, 90.0)
    disp = JointStateDispatcher(
        controller=be,
        use_right_arm=True,
        use_left_arm=False,
        no_gripper=True,
        gripper_mixer=mixer,
        q_min_right=np.full(7, -2.0, dtype=np.float32),
        q_max_right=np.full(7, +2.0, dtype=np.float32),
    )
    # Action 0 -> mid-point 0.0 rad on each joint.
    res = disp.dispatch(np.zeros(7, dtype=np.float32), be.get_state())
    assert res.mode == "joint"
    # The dispatcher must have called send_arm_joints on our backend.
    assert len(be.tx_log) == 1
    assert be.tx_log[0]["type"] == "arm_joints"
    np.testing.assert_allclose(be.tx_log[0]["q_target"], np.zeros(7), atol=1e-6)
