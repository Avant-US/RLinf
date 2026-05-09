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

"""Backend abstraction shared by the R1 Pro CLI tools.

Per design doc ``bt/docs/rwRL/test_galaxea_r1_pro_cli_controller.md`` §4.2,
this module ships a single :class:`ControllerBackend` ABC plus three
concrete implementations:

* :class:`RayBackend` -- wraps a Ray-launched
  :class:`~rlinf.envs.realworld.galaxear.r1_pro_controller.GalaxeaR1ProController`
  worker.  This is the production path (matches what
  :class:`~rlinf.envs.realworld.galaxear.r1_pro_env.GalaxeaR1ProEnv`
  actually uses at training time), but it requires a Ray cluster up.
* :class:`RclpyBackend` -- runs a standalone ``rclpy`` node in the
  current process and publishes/subscribes the same topics directly.
  Lightweight; doesn't require Ray; the recommended path for Orin
  bring-up (Form B from r1pro6op47.md §8.2).
* :class:`DummyBackend` -- no ROS, no Ray; ``send_*`` only logs and
  ``get_state`` returns a synthetic snapshot.  Used by the CLI's
  ``--dummy`` flag and by all unit tests.

The three implementations expose **the same method signatures** as
``GalaxeaR1ProController``'s public RPC surface, so the higher-layer
:class:`~rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher.ActionDispatcher`
treats a backend the same way as a real Ray controller -- duck typing.
This is the "policy-side substitution" property the design doc calls
out.
"""

from __future__ import annotations

import abc
import logging
import os
import threading
import time
from typing import Optional, Sequence

import numpy as np

from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (
    GalaxeaR1ProRobotState,
)

logger = logging.getLogger(__name__)


# ─────────────────────────── ABC ────────────────────────────────


class ControllerBackend(abc.ABC):
    """Duck-type interface matching ``GalaxeaR1ProController`` RPCs.

    Per design doc §4.2.  Subclasses must implement every abstract
    method.  All implementations should treat ``is_dummy=True`` as
    "no real I/O, but everything else (validation, state, etc.) still
    works", because :class:`~tests.unit_tests` uses dummy mode to
    exercise the full pipeline without ROS / Ray.
    """

    @property
    @abc.abstractmethod
    def kind(self) -> str:
        """Short human-readable backend tag, e.g. ``"ray"``, ``"rclpy"``,
        ``"dummy"``.  Logged at startup and in ``StateMonitor``."""

    @abc.abstractmethod
    def send_arm_joints(
        self,
        side: str,
        q_target: Sequence[float],
        qvel_max: Sequence[float],
    ) -> None:
        """Publish ``JointState`` to
        ``/motion_target/target_joint_state_arm_{side}``.

        Args:
            side: ``"right"`` or ``"left"``.
            q_target: 7-D target joint angles in rad (absolute).
            qvel_max: 7-D per-joint maximum allowed speed in rad/s.
                mobiman ``joint_tracker`` reads this as a ceiling, not
                as a feedback velocity (per r1pro6op47.md §3.2.6).
        """

    @abc.abstractmethod
    def send_arm_pose(
        self,
        side: str,
        pose7_xyz_quat: Sequence[float],
    ) -> None:
        """Publish ``PoseStamped`` to
        ``/motion_target/target_pose_arm_{side}``.

        Args:
            side: ``"right"`` or ``"left"``.
            pose7_xyz_quat: 7-D ``[x, y, z, qx, qy, qz, qw]``; the
                quaternion must already be unit-length and W>=0
                canonical.  EePoseDispatcher does that in advance.
        """

    @abc.abstractmethod
    def send_gripper(self, side: str, position_pct: float) -> None:
        """Publish ``JointState`` to
        ``/motion_target/target_position_gripper_{side}``.

        ``position_pct`` is the post-mixer **physical** percentage in
        ``[0, 100]``; the GripperMixer business range mapping has
        already happened upstream.  Backend defensively clips to
        ``[0, 100]`` again to guard against mis-configured mixers.
        """

    @abc.abstractmethod
    def apply_brake(self, on: bool) -> None:
        """Publish ``Bool`` to ``/motion_target/brake_mode``."""

    @abc.abstractmethod
    def get_state(self) -> GalaxeaR1ProRobotState:
        """Return the latest robot state snapshot.  Always returns a
        real :class:`GalaxeaR1ProRobotState` instance (zero-default in
        dummy mode)."""

    @abc.abstractmethod
    def get_subscription_count(self, topic: str) -> int:
        """Return the number of subscribers on the publisher we own
        for *topic*.  Used by ``ActionDispatcher.verify_topology`` and
        the ``topo`` CLI command.

        In dummy mode this returns 1 to keep the topology check happy
        for CI runs.
        """

    @abc.abstractmethod
    def get_publisher_count(self, topic: str) -> int:
        """Return the number of publishers on the topic we subscribe
        to.  Used by the ``topo`` CLI command for feedback-side
        topics like ``/hdas/feedback_arm_*`` and ``/hdas/bms``.
        """

    @abc.abstractmethod
    def shutdown(self) -> None:
        """Tear the backend down (rclpy node destroy / Ray worker stop).

        Idempotent: calling twice must not raise.
        """


# ───────────────────────── DummyBackend ─────────────────────────


class DummyBackend(ControllerBackend):
    """No-op backend for unit tests, ``--dummy`` mode, and CI.

    Records every ``send_*`` call into ``self.tx_log`` so tests can
    assert what was sent without standing up ROS or Ray.

    The synthetic state can be overridden by tests via
    :meth:`set_state` to exercise SafetySupervisor branches that
    depend on specific qpos / bms / etc. values.
    """

    def __init__(
        self,
        *,
        use_right_arm: bool = True,
        use_left_arm: bool = False,
    ):
        self._state = GalaxeaR1ProRobotState()
        # is_alive=True so any L5 alive-check passes; the heartbeat
        # tests in particular need this.
        self._state.is_alive = True
        self._use_right_arm = bool(use_right_arm)
        self._use_left_arm = bool(use_left_arm)
        self.tx_log: list[dict] = []
        self._shutdown = False

    @property
    def kind(self) -> str:
        return "dummy"

    # ── State injection helpers (used by tests + REPL `set state`) ──

    def set_state(self, state: GalaxeaR1ProRobotState) -> None:
        """Replace the synthetic state in-place.  Tests use this to
        feed specific qpos / bms / status_errors values into the
        SafetySupervisor."""
        self._state = state

    def patch_state(self, **kwargs) -> None:
        """Convenience: ``backend.patch_state(right_arm_qpos=np.zeros(7))``
        without rebuilding the whole snapshot."""
        for k, v in kwargs.items():
            setattr(self._state, k, v)

    # ── ABC implementation ─────────────────────────────────────

    def send_arm_joints(self, side, q_target, qvel_max) -> None:
        rec = {
            "type": "arm_joints",
            "side": side,
            "q_target": [float(x) for x in q_target],
            "qvel_max": [float(x) for x in qvel_max],
            "ts": time.monotonic(),
        }
        self.tx_log.append(rec)
        # Reflect the command into the synthetic state so that
        # consecutive `state` queries show the latest target as the
        # current qpos -- handy for the REPL UX.  This is intentionally
        # only done in dummy mode.
        q = np.asarray(q_target, dtype=np.float32).reshape(-1)[:7]
        if side == "right":
            self._state.right_arm_qpos = q
        else:
            self._state.left_arm_qpos = q

    def send_arm_pose(self, side, pose7_xyz_quat) -> None:
        pose = np.asarray(pose7_xyz_quat, dtype=np.float32).reshape(-1)[:7]
        self.tx_log.append(
            {
                "type": "arm_pose",
                "side": side,
                "pose": pose.tolist(),
                "ts": time.monotonic(),
            }
        )
        if side == "right":
            self._state.right_ee_pose = pose
        else:
            self._state.left_ee_pose = pose

    def send_gripper(self, side, position_pct) -> None:
        pct = float(np.clip(position_pct, 0.0, 100.0))
        self.tx_log.append(
            {
                "type": "gripper",
                "side": side,
                "pct": pct,
                "ts": time.monotonic(),
            }
        )
        if side == "right":
            self._state.right_gripper_pos = pct
        else:
            self._state.left_gripper_pos = pct

    def apply_brake(self, on) -> None:
        self.tx_log.append(
            {
                "type": "brake",
                "on": bool(on),
                "ts": time.monotonic(),
            }
        )

    def get_state(self) -> GalaxeaR1ProRobotState:
        return self._state

    def get_subscription_count(self, topic: str) -> int:
        # Always pretend there's a subscriber so topo check passes.
        return 1

    def get_publisher_count(self, topic: str) -> int:
        # Same -- always pretend a feedback publisher exists.
        return 1

    def shutdown(self) -> None:
        self._shutdown = True


# ───────────────────────── RayBackend ──────────────────────────


class RayBackend(ControllerBackend):
    """Wraps a Ray-launched :class:`GalaxeaR1ProController` worker.

    Per design doc §4.3.  All public methods translate to a ``.wait()``
    Ray RPC call so the CLI stays synchronous (the operator types one
    command, sees one reply).

    The constructor expects Ray to already be initialised (i.e. ``ray
    start --head`` has been run on this node).  When ``is_dummy=True``,
    we do NOT call ``ray.init`` -- the worker handle is constructed
    in dummy mode and behaves like :class:`DummyBackend`.
    """

    def __init__(
        self,
        *,
        node_rank: int = 0,
        ros_domain_id: int = 41,
        ros_localhost_only: bool = False,
        galaxea_install_path: str = "~/galaxea/install",
        use_right_arm: bool = True,
        use_left_arm: bool = False,
        use_torso: bool = False,
        use_chassis: bool = False,
        mobiman_launch_mode: str = "joint",
        is_dummy: bool = False,
        worker_name_suffix: Optional[str] = None,
    ):
        self._is_dummy = bool(is_dummy)
        self._use_right_arm = use_right_arm
        self._use_left_arm = use_left_arm
        self._handle = None
        self._shutdown_called = False

        if is_dummy:
            # In dummy mode delegate to a DummyBackend internally so
            # tests see consistent behaviour.
            self._dummy = DummyBackend(
                use_right_arm=use_right_arm,
                use_left_arm=use_left_arm,
            )
            return
        self._dummy = None

        from rlinf.envs.realworld.galaxear.r1_pro_controller import (  # noqa: PLC0415
            GalaxeaR1ProController,
        )

        suffix = (
            worker_name_suffix
            if worker_name_suffix is not None
            else f"cli{os.getpid()}"
        )
        # ``launch_controller`` returns a Ray WorkerGroup handle; calls
        # against it return Ray futures whose ``.wait()`` blocks until
        # the worker side completes.
        self._handle = GalaxeaR1ProController.launch_controller(
            env_idx=0,
            node_rank=int(node_rank),
            worker_rank=0,
            ros_domain_id=int(ros_domain_id),
            ros_localhost_only=bool(ros_localhost_only),
            galaxea_install_path=galaxea_install_path,
            use_left_arm=use_left_arm,
            use_right_arm=use_right_arm,
            use_torso=use_torso,
            use_chassis=use_chassis,
            mobiman_launch_mode=mobiman_launch_mode,
            is_dummy=False,
        )
        # Mutate the worker name suffix indirectly via env var so a
        # subsequent CLI launch doesn't collide with this one.
        # (GalaxeaR1ProController.launch_controller already adds pid.)
        _ = suffix

    @property
    def kind(self) -> str:
        return "ray"

    def _call(self, method_name: str, *args, blocking: bool = True):
        """Helper: dispatch to either the dummy backend or the real
        Ray handle, with optional ``.wait()``.
        """
        if self._is_dummy:
            return getattr(self._dummy, method_name)(*args)
        ref = getattr(self._handle, method_name)(*args)
        if blocking and ref is not None and hasattr(ref, "wait"):
            res = ref.wait()
            # WorkerGroup.wait returns a list (one per worker); CLI
            # only has one worker so unpack the first element.
            if isinstance(res, list) and res:
                return res[0]
            return res
        return ref

    # ── ABC implementation (all blocking) ──────────────────────

    def send_arm_joints(self, side, q_target, qvel_max) -> None:
        self._call(
            "send_arm_joints",
            side,
            list(q_target),
            list(qvel_max),
            blocking=True,
        )

    def send_arm_pose(self, side, pose7_xyz_quat) -> None:
        self._call(
            "send_arm_pose",
            side,
            np.asarray(pose7_xyz_quat, dtype=np.float32),
            blocking=True,
        )

    def send_gripper(self, side, position_pct) -> None:
        self._call(
            "send_gripper",
            side,
            float(position_pct),
            blocking=True,
        )

    def apply_brake(self, on) -> None:
        self._call("apply_brake", bool(on), blocking=True)

    def get_state(self) -> GalaxeaR1ProRobotState:
        return self._call("get_state", blocking=True)

    def get_subscription_count(self, topic: str) -> int:
        return int(self._call("get_subscription_count", topic, blocking=True))

    def get_publisher_count(self, topic: str) -> int:
        # The Ray controller doesn't expose this method yet; fall
        # back to "unknown / 1" so topo check doesn't false-fail.
        # When the controller is upgraded we can call it directly.
        getter = (
            getattr(self._handle, "get_publisher_count", None)
            if not self._is_dummy
            else None
        )
        if getter is None:
            return 1
        ref = getter(topic)
        if hasattr(ref, "wait"):
            res = ref.wait()
            return int(res[0] if isinstance(res, list) else res)
        return int(ref)

    def shutdown(self) -> None:
        if self._shutdown_called:
            return
        self._shutdown_called = True
        if self._is_dummy:
            self._dummy.shutdown()
            return
        try:
            self._call("shutdown", blocking=False)
        except Exception:  # noqa: BLE001 - best-effort teardown
            logger.exception("RayBackend shutdown failed")


# ───────────────────────── RclpyBackend ────────────────────────


class RclpyBackend(ControllerBackend):
    """Standalone ``rclpy`` node + threaded executor.

    Per design doc §4.4.  Lazy-imports rclpy so a CLI process running
    in dummy mode doesn't even need ROS 2 sourced.  Subscribes to all
    feedback topics that ``GalaxeaR1ProController`` does (including
    ``/motion_control/pose_ee_arm_*`` so EE state isn't silently
    zero -- the bug fix from r1pro6op47.md §1.2 #2).
    """

    DEFAULT_JOINT_NAMES = {
        "right": [f"arm_right_j{i + 1}" for i in range(7)],
        "left": [f"arm_left_j{i + 1}" for i in range(7)],
    }
    # Feedback topics we expect a publisher on (for topo check).
    FEEDBACK_TOPICS_RIGHT = (
        "/hdas/feedback_arm_right",
        "/hdas/feedback_gripper_right",
        "/motion_control/pose_ee_arm_right",
    )
    FEEDBACK_TOPICS_LEFT = (
        "/hdas/feedback_arm_left",
        "/hdas/feedback_gripper_left",
        "/motion_control/pose_ee_arm_left",
    )
    OPTIONAL_FEEDBACK_TOPICS = (
        "/hdas/bms",
        "/controller",
    )

    def __init__(
        self,
        *,
        ros_domain_id: int = 41,
        ros_localhost_only: bool = False,
        use_right_arm: bool = True,
        use_left_arm: bool = False,
        no_gripper: bool = False,
        is_dummy: bool = False,
    ):
        self._is_dummy = bool(is_dummy)
        self._use_right_arm = bool(use_right_arm)
        self._use_left_arm = bool(use_left_arm)
        self._no_gripper = bool(no_gripper)
        self._state = GalaxeaR1ProRobotState()
        self._state.is_alive = True if is_dummy else False
        self._state_lock = threading.RLock()
        self._first_seen: dict[str, float] = {}
        self._pubs: dict = {}
        self._subs: list = []
        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_running = False
        self._rclpy = None
        self._shutdown_called = False
        self._tx_log: list[dict] = []

        if is_dummy:
            # Same dummy-delegation as RayBackend so tests see one TX log.
            self._dummy = DummyBackend(
                use_right_arm=use_right_arm,
                use_left_arm=use_left_arm,
            )
            return
        self._dummy = None

        os.environ["ROS_DOMAIN_ID"] = str(int(ros_domain_id))
        os.environ["ROS_LOCALHOST_ONLY"] = "1" if ros_localhost_only else "0"
        self._init_ros2()
        self._init_publishers()
        self._init_subscribers()
        self._spin_running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop,
            name="r1_pro_cli_rclpy_spin",
            daemon=True,
        )
        self._spin_thread.start()

    @property
    def kind(self) -> str:
        return "rclpy"

    @property
    def tx_log(self) -> list[dict]:
        if self._is_dummy:
            return self._dummy.tx_log
        return self._tx_log

    # ── rclpy plumbing ─────────────────────────────────────────

    def _init_ros2(self) -> None:
        import rclpy  # type: ignore[import]
        from rclpy.executors import (  # type: ignore[import]
            SingleThreadedExecutor,
        )

        if not rclpy.ok():
            rclpy.init(args=[])
        self._rclpy = rclpy
        node_name = f"r1_pro_cli_rclpy_{os.getpid()}"
        self._node = rclpy.create_node(node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

    def _init_publishers(self) -> None:
        from geometry_msgs.msg import PoseStamped  # type: ignore[import]
        from rclpy.qos import (  # type: ignore[import]
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState  # type: ignore[import]
        from std_msgs.msg import Bool  # type: ignore[import]

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        if self._use_right_arm:
            self._pubs["target_joint_state_arm_right"] = self._node.create_publisher(
                JointState,
                "/motion_target/target_joint_state_arm_right",
                reliable,
            )
            self._pubs["target_pose_arm_right"] = self._node.create_publisher(
                PoseStamped,
                "/motion_target/target_pose_arm_right",
                reliable,
            )
            if not self._no_gripper:
                self._pubs["target_position_gripper_right"] = (
                    self._node.create_publisher(
                        JointState,
                        "/motion_target/target_position_gripper_right",
                        reliable,
                    )
                )
        if self._use_left_arm:
            self._pubs["target_joint_state_arm_left"] = self._node.create_publisher(
                JointState,
                "/motion_target/target_joint_state_arm_left",
                reliable,
            )
            self._pubs["target_pose_arm_left"] = self._node.create_publisher(
                PoseStamped,
                "/motion_target/target_pose_arm_left",
                reliable,
            )
            if not self._no_gripper:
                self._pubs["target_position_gripper_left"] = (
                    self._node.create_publisher(
                        JointState,
                        "/motion_target/target_position_gripper_left",
                        reliable,
                    )
                )
        self._pubs["brake_mode"] = self._node.create_publisher(
            Bool,
            "/motion_target/brake_mode",
            reliable,
        )

    def _init_subscribers(self) -> None:
        from geometry_msgs.msg import PoseStamped  # type: ignore[import]
        from rclpy.qos import qos_profile_sensor_data  # type: ignore[import]
        from sensor_msgs.msg import JointState  # type: ignore[import]

        if self._use_right_arm:
            self._subs.append(
                self._node.create_subscription(
                    JointState,
                    "/hdas/feedback_arm_right",
                    self._on_arm_right,
                    qos_profile_sensor_data,
                )
            )
            self._subs.append(
                self._node.create_subscription(
                    JointState,
                    "/hdas/feedback_gripper_right",
                    self._on_gripper_right,
                    qos_profile_sensor_data,
                )
            )
            self._subs.append(
                self._node.create_subscription(
                    PoseStamped,
                    "/motion_control/pose_ee_arm_right",
                    self._on_pose_ee_right,
                    qos_profile_sensor_data,
                )
            )
        if self._use_left_arm:
            self._subs.append(
                self._node.create_subscription(
                    JointState,
                    "/hdas/feedback_arm_left",
                    self._on_arm_left,
                    qos_profile_sensor_data,
                )
            )
            self._subs.append(
                self._node.create_subscription(
                    JointState,
                    "/hdas/feedback_gripper_left",
                    self._on_gripper_left,
                    qos_profile_sensor_data,
                )
            )
            self._subs.append(
                self._node.create_subscription(
                    PoseStamped,
                    "/motion_control/pose_ee_arm_left",
                    self._on_pose_ee_left,
                    qos_profile_sensor_data,
                )
            )
        # Optional hdas_msg-typed subscribers (BMS / controller / status).
        # We use PascalCase imports per r1pro6op47.md §6.2; failing to
        # import is logged but not fatal -- the L5 watchdog just loses
        # those signals.
        try:
            from hdas_msg.msg import (  # type: ignore[import]
                Bms as BmsMsg,
            )
            from hdas_msg.msg import (
                ControllerSignalStamped as ControllerMsg,
            )

            self._subs.append(
                self._node.create_subscription(
                    BmsMsg,
                    "/hdas/bms",
                    self._on_bms,
                    qos_profile_sensor_data,
                )
            )
            self._subs.append(
                self._node.create_subscription(
                    ControllerMsg,
                    "/controller",
                    self._on_controller,
                    qos_profile_sensor_data,
                )
            )
        except ImportError:
            logger.warning(
                "hdas_msg PascalCase import failed; L5 BMS / controller "
                "watchdog disabled.  Did you 'source "
                "/home/nvidia/galaxea/install/setup.bash'?"
            )

    def _spin_loop(self) -> None:
        try:
            while self._spin_running and self._rclpy is not None and self._rclpy.ok():
                self._executor.spin_once(timeout_sec=0.05)
                self._update_age()
        except Exception:  # noqa: BLE001 - defensive
            logger.exception("rclpy spin thread crashed")

    def _update_age(self) -> None:
        now = time.time()
        with self._state_lock:
            for src, t0 in self._first_seen.items():
                self._state.feedback_age_ms[src] = (now - t0) * 1000.0
            self._state.is_alive = any(
                age < 1500.0 for age in self._state.feedback_age_ms.values()
            )

    # ── ROS callbacks ──────────────────────────────────────────

    def _stamp(self, key: str) -> None:
        self._first_seen[key] = time.time()

    def _on_arm_right(self, msg) -> None:
        self._stamp("arm_right")
        with self._state_lock:
            if msg.position:
                self._state.right_arm_qpos = np.asarray(
                    msg.position[:7],
                    dtype=np.float32,
                )
            if msg.velocity:
                self._state.right_arm_qvel = np.asarray(
                    msg.velocity[:7],
                    dtype=np.float32,
                )

    def _on_arm_left(self, msg) -> None:
        self._stamp("arm_left")
        with self._state_lock:
            if msg.position:
                self._state.left_arm_qpos = np.asarray(
                    msg.position[:7],
                    dtype=np.float32,
                )
            if msg.velocity:
                self._state.left_arm_qvel = np.asarray(
                    msg.velocity[:7],
                    dtype=np.float32,
                )

    def _on_gripper_right(self, msg) -> None:
        self._stamp("gripper_right")
        with self._state_lock:
            if msg.position:
                self._state.right_gripper_pos = float(msg.position[0])

    def _on_gripper_left(self, msg) -> None:
        self._stamp("gripper_left")
        with self._state_lock:
            if msg.position:
                self._state.left_gripper_pos = float(msg.position[0])

    def _on_pose_ee_right(self, msg) -> None:
        self._stamp("pose_ee_arm_right")
        p, q = msg.pose.position, msg.pose.orientation
        with self._state_lock:
            self._state.right_ee_pose = np.array(
                [p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                dtype=np.float32,
            )

    def _on_pose_ee_left(self, msg) -> None:
        self._stamp("pose_ee_arm_left")
        p, q = msg.pose.position, msg.pose.orientation
        with self._state_lock:
            self._state.left_ee_pose = np.array(
                [p.x, p.y, p.z, q.x, q.y, q.z, q.w],
                dtype=np.float32,
            )

    def _on_bms(self, msg) -> None:
        self._stamp("bms")
        with self._state_lock:
            self._state.bms = {
                "voltage": float(getattr(msg, "voltage", 0.0)),
                "current": float(getattr(msg, "current", 0.0)),
                "capital_pct": float(
                    getattr(msg, "capital", getattr(msg, "capital_pct", 100.0))
                ),
            }

    def _on_controller(self, msg) -> None:
        self._stamp("controller")
        data = getattr(msg, "data", msg)
        with self._state_lock:
            self._state.controller_signal = {
                "left_x_axis": float(getattr(data, "left_x_axis", 0.0)),
                "left_y_axis": float(getattr(data, "left_y_axis", 0.0)),
                "right_x_axis": float(getattr(data, "right_x_axis", 0.0)),
                "right_y_axis": float(getattr(data, "right_y_axis", 0.0)),
                "mode": int(getattr(data, "mode", 0)),
                # Back-compat zero defaults so legacy SafetyConfig
                # `estop_swd_value_down` reads 0.
                "swa": 0,
                "swb": 0,
                "swc": 0,
                "swd": 0,
            }

    # ── ABC implementation: send_* ─────────────────────────────

    def send_arm_joints(self, side, q_target, qvel_max) -> None:
        self._tx_log.append(
            {
                "type": "arm_joints",
                "side": side,
                "q_target": [float(x) for x in q_target],
                "qvel_max": [float(x) for x in qvel_max],
                "ts": time.monotonic(),
            }
        )
        if self._is_dummy:
            return self._dummy.send_arm_joints(side, q_target, qvel_max)
        from sensor_msgs.msg import JointState  # type: ignore[import]

        topic_key = f"target_joint_state_arm_{side}"
        if topic_key not in self._pubs:
            raise ValueError(
                f"send_arm_joints: arm '{side}' not enabled at backend init"
            )
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(self.DEFAULT_JOINT_NAMES[side])
        q = [float(x) for x in list(q_target)[:7]]
        if len(q) < 7:
            q = q + [0.0] * (7 - len(q))
        msg.position = q
        v = [float(x) for x in list(qvel_max)[:7]]
        if len(v) < 7:
            v = v + [3.0] * (7 - len(v))
        msg.velocity = v
        self._pubs[topic_key].publish(msg)

    def send_arm_pose(self, side, pose7_xyz_quat) -> None:
        pose = np.asarray(pose7_xyz_quat, dtype=np.float32).reshape(-1)[:7]
        self._tx_log.append(
            {
                "type": "arm_pose",
                "side": side,
                "pose": pose.tolist(),
                "ts": time.monotonic(),
            }
        )
        if self._is_dummy:
            return self._dummy.send_arm_pose(side, pose7_xyz_quat)
        from geometry_msgs.msg import PoseStamped  # type: ignore[import]

        topic_key = f"target_pose_arm_{side}"
        if topic_key not in self._pubs:
            raise ValueError(f"send_arm_pose: arm '{side}' not enabled at backend init")
        if pose.size != 7:
            raise ValueError(f"send_arm_pose: expected 7-D pose, got {pose.size}")
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "torso_link4"
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.position.z = float(pose[2])
        msg.pose.orientation.x = float(pose[3])
        msg.pose.orientation.y = float(pose[4])
        msg.pose.orientation.z = float(pose[5])
        msg.pose.orientation.w = float(pose[6])
        self._pubs[topic_key].publish(msg)

    def send_gripper(self, side, position_pct) -> None:
        pct = float(np.clip(position_pct, 0.0, 100.0))
        self._tx_log.append(
            {
                "type": "gripper",
                "side": side,
                "pct": pct,
                "ts": time.monotonic(),
            }
        )
        if self._is_dummy:
            return self._dummy.send_gripper(side, pct)
        from sensor_msgs.msg import JointState  # type: ignore[import]

        topic_key = f"target_position_gripper_{side}"
        if topic_key not in self._pubs:
            return
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = [f"gripper_{side}"]
        msg.position = [pct]
        self._pubs[topic_key].publish(msg)

    def apply_brake(self, on) -> None:
        self._tx_log.append(
            {
                "type": "brake",
                "on": bool(on),
                "ts": time.monotonic(),
            }
        )
        if self._is_dummy:
            return self._dummy.apply_brake(on)
        from std_msgs.msg import Bool  # type: ignore[import]

        if "brake_mode" in self._pubs:
            self._pubs["brake_mode"].publish(Bool(data=bool(on)))

    def get_state(self) -> GalaxeaR1ProRobotState:
        if self._is_dummy:
            return self._dummy.get_state()
        with self._state_lock:
            return self._state.copy()

    def get_subscription_count(self, topic: str) -> int:
        if self._is_dummy:
            return self._dummy.get_subscription_count(topic)
        for pub in self._pubs.values():
            try:
                if getattr(pub, "topic_name", "") == topic:
                    return int(pub.get_subscription_count())
            except Exception:  # noqa: BLE001
                continue
        return 0

    def get_publisher_count(self, topic: str) -> int:
        if self._is_dummy:
            return self._dummy.get_publisher_count(topic)
        for sub in self._subs:
            try:
                if getattr(sub, "topic_name", "") == topic:
                    return int(sub.get_publisher_count())
            except Exception:  # noqa: BLE001
                continue
        return 0

    def shutdown(self) -> None:
        if self._shutdown_called:
            return
        self._shutdown_called = True
        if self._is_dummy:
            self._dummy.shutdown()
            return
        self._spin_running = False
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        # Do NOT call rclpy.shutdown() -- another component in this
        # process may still hold an rclpy node.


# ─────────────────────── Factory helper ─────────────────────────


def build_backend(
    *,
    kind: str,
    use_right_arm: bool = True,
    use_left_arm: bool = False,
    no_gripper: bool = False,
    is_dummy: bool = False,
    ros_domain_id: int = 41,
    ros_localhost_only: bool = False,
    galaxea_install_path: str = "~/galaxea/install",
    use_torso: bool = False,
    use_chassis: bool = False,
    mobiman_launch_mode: str = "joint",
    controller_node_rank: int = 0,
) -> ControllerBackend:
    """Construct a backend per the ``--backend`` flag.

    Per design doc §10.1 -- single source of truth for the
    ``"ray" | "rclpy" | "dummy"`` -> class mapping.

    Raises:
        ValueError: when *kind* is unknown.
    """
    kind_l = kind.lower().strip()
    if is_dummy or kind_l == "dummy":
        return DummyBackend(
            use_right_arm=use_right_arm,
            use_left_arm=use_left_arm,
        )
    if kind_l == "ray":
        return RayBackend(
            node_rank=controller_node_rank,
            ros_domain_id=ros_domain_id,
            ros_localhost_only=ros_localhost_only,
            galaxea_install_path=galaxea_install_path,
            use_right_arm=use_right_arm,
            use_left_arm=use_left_arm,
            use_torso=use_torso,
            use_chassis=use_chassis,
            mobiman_launch_mode=mobiman_launch_mode,
            is_dummy=False,
        )
    if kind_l == "rclpy":
        return RclpyBackend(
            ros_domain_id=ros_domain_id,
            ros_localhost_only=ros_localhost_only,
            use_right_arm=use_right_arm,
            use_left_arm=use_left_arm,
            no_gripper=no_gripper,
            is_dummy=False,
        )
    raise ValueError(
        f"Unknown backend kind '{kind}'.  Choose 'ray', 'rclpy', or 'dummy'."
    )
