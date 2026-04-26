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

"""ROS 2 controller worker for Galaxea R1 Pro.

Mirrors :class:`rlinf.envs.realworld.franka.franka_controller.FrankaController`
but uses ROS 2 (``rclpy``) instead of ROS 1 (``rospy``) and publishes
to mobiman's ``/motion_target/...`` topics rather than the Cartesian
impedance controller.

Designed to live on the **Orin** (the only node with the CAN bus and
the Galaxea SDK ``install/`` tree).  EnvWorker on the GPU server calls
this controller through Ray RPC: ``self._controller.method(args).wait()``.
The class is a :class:`rlinf.scheduler.Worker` subclass so all RLinf
scheduler primitives (``launch_controller(node_rank=...)`` ->
``NodePlacementStrategy``) work out of the box.

All ROS 2 imports are deferred to :meth:`__init__` so callers on the
GPU server (which often does not have ROS 2 installed) can import this
module without errors.

Cf. design doc §6.2 and §G.7 for details.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from contextlib import contextmanager
from typing import Optional

import numpy as np

from rlinf.scheduler import Cluster, NodePlacementStrategy, Worker

from .r1_pro_robot_state import GalaxeaR1ProRobotState

_logger = logging.getLogger(__name__)


class GalaxeaR1ProController(Worker):
    """ROS 2 controller worker for Galaxea R1 Pro.

    Use :meth:`launch_controller` to instantiate the worker on the
    target node (typically Orin).  The returned handle is a Ray
    WorkerGroup — call methods like
    ``handle.get_state().wait()[0]``.
    """

    @staticmethod
    def launch_controller(
        env_idx: int = 0,
        node_rank: int = 1,
        worker_rank: int = 0,
        ros_domain_id: int = 72,
        ros_localhost_only: bool = False,
        galaxea_install_path: str = "~/galaxea/install",
        use_left_arm: bool = False,
        use_right_arm: bool = True,
        use_torso: bool = False,
        use_chassis: bool = False,
        mobiman_launch_mode: str = "pose",
        is_dummy: bool = False,
    ) -> "GalaxeaR1ProController":
        """Launch a controller worker on *node_rank* and return its handle.

        Args:
            env_idx: Index of the env worker requesting this controller.
            node_rank: Cluster node where the controller process runs.
                Default ``1`` is the Orin in the recommended 2-node
                deployment; pass ``0`` for the Form B variant where
                the controller co-locates with the EnvWorker on the
                GPU server.
            worker_rank: Rank of the env worker (used for unique name).
            ros_domain_id: ``ROS_DOMAIN_ID`` to enforce.
            ros_localhost_only: When False (default), allow cross-host
                DDS discovery.
            galaxea_install_path: Path to the Galaxea SDK install dir.
            use_left_arm / use_right_arm / use_torso / use_chassis:
                Stage flags; controller only creates the matching
                publishers / subscribers.
            mobiman_launch_mode: ``pose`` / ``joint`` / ``hybrid``.
            is_dummy: When True the worker still gets created on the
                target node but skips all ROS 2 init; useful for
                ``is_dummy=True`` env runs that still want to verify
                the cross-node placement path.
        """
        cluster = Cluster()
        placement = NodePlacementStrategy(node_ranks=[node_rank])
        return GalaxeaR1ProController.create_group(
            ros_domain_id=ros_domain_id,
            ros_localhost_only=ros_localhost_only,
            galaxea_install_path=galaxea_install_path,
            use_left_arm=use_left_arm,
            use_right_arm=use_right_arm,
            use_torso=use_torso,
            use_chassis=use_chassis,
            mobiman_launch_mode=mobiman_launch_mode,
            is_dummy=is_dummy,
        ).launch(
            cluster=cluster,
            placement_strategy=placement,
            name=f"GalaxeaR1ProController-{worker_rank}-{env_idx}",
        )

    def __init__(
        self,
        ros_domain_id: int = 72,
        ros_localhost_only: bool = False,
        galaxea_install_path: str = "~/galaxea/install",
        use_left_arm: bool = False,
        use_right_arm: bool = True,
        use_torso: bool = False,
        use_chassis: bool = False,
        mobiman_launch_mode: str = "pose",
        is_dummy: bool = False,
    ) -> None:
        super().__init__()
        self._ros_domain_id = ros_domain_id
        self._ros_localhost_only = ros_localhost_only
        self._galaxea_install_path = os.path.expanduser(galaxea_install_path)
        self._use_left_arm = use_left_arm
        self._use_right_arm = use_right_arm
        self._use_torso = use_torso
        self._use_chassis = use_chassis
        self._mobiman_launch_mode = mobiman_launch_mode
        self._is_dummy = is_dummy

        # State buffer (Ray RPC return target).
        self._state = GalaxeaR1ProRobotState()
        self._state.is_alive = True if is_dummy else False
        self._state_lock = threading.RLock()
        self._feedback_first_seen: dict[str, float] = {}

        # Lazy ROS 2 handles.
        self._rclpy = None
        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_running = False
        self._pubs: dict = {}
        self._subs: list = []

        if is_dummy:
            self.log_info(
                "GalaxeaR1ProController (dummy) up; no ROS 2 init."
            )
            return

        try:
            self._init_ros2_node()
            self._init_publishers()
            self._init_subscribers()
        except Exception as e:
            self.log_error(
                f"GalaxeaR1ProController ROS 2 init failed: {e}"
            )
            raise

        self._spin_running = True
        self._spin_thread = threading.Thread(
            target=self._spin_thread_fn,
            name="r1_pro_controller_spin",
            daemon=True,
        )
        self._spin_thread.start()

        self.log_info(
            "GalaxeaR1ProController up: arms=%s torso=%s chassis=%s "
            "domain_id=%d localhost_only=%s mode=%s pid=%d"
            % (
                ("L" if use_left_arm else "")
                + ("R" if use_right_arm else ""),
                use_torso, use_chassis, ros_domain_id, ros_localhost_only,
                mobiman_launch_mode, os.getpid(),
            )
        )

    # ── ROS 2 init helpers ──────────────────────────────────────

    def _init_ros2_node(self) -> None:
        os.environ["ROS_DOMAIN_ID"] = str(self._ros_domain_id)
        os.environ["ROS_LOCALHOST_ONLY"] = (
            "1" if self._ros_localhost_only else "0"
        )

        import rclpy  # type: ignore[import]
        from rclpy.executors import MultiThreadedExecutor  # type: ignore

        if not rclpy.ok():
            rclpy.init(args=[])
        self._rclpy = rclpy
        node_name = f"rlinf_galaxea_r1_pro_controller_{os.getpid()}"
        self._node = rclpy.create_node(node_name)
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)

    def _init_publishers(self) -> None:
        from geometry_msgs.msg import PoseStamped, Twist, TwistStamped  # type: ignore
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # type: ignore
        from sensor_msgs.msg import JointState  # type: ignore
        from std_msgs.msg import Bool  # type: ignore

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if self._use_right_arm:
            self._pubs["target_pose_arm_right"] = self._node.create_publisher(
                PoseStamped,
                "/motion_target/target_pose_arm_right",
                reliable,
            )
            self._pubs["target_joint_state_arm_right"] = (
                self._node.create_publisher(
                    JointState,
                    "/motion_target/target_joint_state_arm_right",
                    reliable,
                )
            )
            self._pubs["target_position_gripper_right"] = (
                self._node.create_publisher(
                    JointState,
                    "/motion_target/target_position_gripper_right",
                    reliable,
                )
            )
        if self._use_left_arm:
            self._pubs["target_pose_arm_left"] = self._node.create_publisher(
                PoseStamped,
                "/motion_target/target_pose_arm_left",
                reliable,
            )
            self._pubs["target_joint_state_arm_left"] = (
                self._node.create_publisher(
                    JointState,
                    "/motion_target/target_joint_state_arm_left",
                    reliable,
                )
            )
            self._pubs["target_position_gripper_left"] = (
                self._node.create_publisher(
                    JointState,
                    "/motion_target/target_position_gripper_left",
                    reliable,
                )
            )
        if self._use_torso:
            self._pubs["target_speed_torso"] = self._node.create_publisher(
                TwistStamped,
                "/motion_target/target_speed_torso",
                reliable,
            )
        if self._use_chassis:
            self._pubs["target_speed_chassis"] = self._node.create_publisher(
                Twist,
                "/motion_target/target_speed_chassis",
                reliable,
            )
            self._pubs["chassis_acc_limit"] = self._node.create_publisher(
                Twist,
                "/motion_target/chassis_acc_limit",
                reliable,
            )
            self._pubs["brake_mode"] = self._node.create_publisher(
                Bool,
                "/motion_target/brake_mode",
                reliable,
            )

    def _init_subscribers(self) -> None:
        from rclpy.callback_groups import (  # type: ignore
            MutuallyExclusiveCallbackGroup,
        )
        from rclpy.qos import qos_profile_sensor_data  # type: ignore
        from sensor_msgs.msg import Imu, JointState  # type: ignore

        cb_state = MutuallyExclusiveCallbackGroup()
        cb_safety = MutuallyExclusiveCallbackGroup()

        if self._use_right_arm:
            self._subs.append(self._node.create_subscription(
                JointState, "/hdas/feedback_arm_right",
                self._on_arm_right_feedback,
                qos_profile_sensor_data,
                callback_group=cb_state,
            ))
            self._subs.append(self._node.create_subscription(
                JointState, "/hdas/feedback_gripper_right",
                self._on_gripper_right_feedback,
                qos_profile_sensor_data,
                callback_group=cb_state,
            ))
        if self._use_left_arm:
            self._subs.append(self._node.create_subscription(
                JointState, "/hdas/feedback_arm_left",
                self._on_arm_left_feedback,
                qos_profile_sensor_data,
                callback_group=cb_state,
            ))
            self._subs.append(self._node.create_subscription(
                JointState, "/hdas/feedback_gripper_left",
                self._on_gripper_left_feedback,
                qos_profile_sensor_data,
                callback_group=cb_state,
            ))
        if self._use_torso:
            self._subs.append(self._node.create_subscription(
                JointState, "/hdas/feedback_torso",
                self._on_torso_feedback,
                qos_profile_sensor_data,
                callback_group=cb_state,
            ))
        if self._use_chassis:
            self._subs.append(self._node.create_subscription(
                JointState, "/hdas/feedback_chassis",
                self._on_chassis_feedback,
                qos_profile_sensor_data,
                callback_group=cb_state,
            ))
        # IMU (best-effort)
        self._subs.append(self._node.create_subscription(
            Imu, "/hdas/imu_torso",
            lambda msg: self._on_imu(msg, "torso"),
            qos_profile_sensor_data,
            callback_group=cb_state,
        ))
        self._subs.append(self._node.create_subscription(
            Imu, "/hdas/imu_chassis",
            lambda msg: self._on_imu(msg, "chassis"),
            qos_profile_sensor_data,
            callback_group=cb_state,
        ))

        # Optional: Galaxea custom messages (graceful fallback when
        # `hdas_msg` is not available on the controller node).
        try:
            from hdas_msg.msg import (  # type: ignore[import]
                bms as BmsMsg,
                controller_signal_stamped as ControllerSignalMsg,
                feedback_status as StatusMsg,
            )
            self._subs.append(self._node.create_subscription(
                BmsMsg, "/hdas/bms",
                self._on_bms,
                qos_profile_sensor_data,
                callback_group=cb_safety,
            ))
            self._subs.append(self._node.create_subscription(
                ControllerSignalMsg, "/controller",
                self._on_controller_signal,
                qos_profile_sensor_data,
                callback_group=cb_safety,
            ))
            if self._use_right_arm:
                self._subs.append(self._node.create_subscription(
                    StatusMsg, "/hdas/feedback_status_arm_right",
                    lambda m: self._on_status(m, "right"),
                    qos_profile_sensor_data,
                    callback_group=cb_safety,
                ))
            if self._use_left_arm:
                self._subs.append(self._node.create_subscription(
                    StatusMsg, "/hdas/feedback_status_arm_left",
                    lambda m: self._on_status(m, "left"),
                    qos_profile_sensor_data,
                    callback_group=cb_safety,
                ))
        except ImportError:
            self.log_warning(
                "hdas_msg not importable; SWD / BMS / status_errors L5 "
                "watchdog hooks disabled.  Install Galaxea SDK + colcon "
                "build hdas_msg on this node, or rely on geometry "
                "safety (L1-L4) only."
            )

    def _spin_thread_fn(self) -> None:
        try:
            while self._spin_running and self._rclpy is not None and self._rclpy.ok():
                self._executor.spin_once(timeout_sec=0.05)
                self._update_feedback_age()
        except Exception as e:  # pragma: no cover - defensive
            self.log_error(f"Controller spin thread crashed: {e}")

    def _update_feedback_age(self) -> None:
        now = time.time()
        with self._state_lock:
            for src, t0 in self._feedback_first_seen.items():
                self._state.feedback_age_ms[src] = (now - t0) * 1000.0
            # Aliveness: at least one arm feedback fresh in last 500 ms.
            any_alive = any(
                age < 500.0
                for age in self._state.feedback_age_ms.values()
            )
            self._state.is_alive = any_alive

    # ── ROS 2 callbacks ─────────────────────────────────────────

    def _stamp_first_seen(self, key: str) -> None:
        self._feedback_first_seen[key] = time.time()

    def _on_arm_right_feedback(self, msg) -> None:
        self._stamp_first_seen("arm_right")
        with self._state_lock:
            if msg.position:
                self._state.right_arm_qpos = np.asarray(
                    msg.position[:7], dtype=np.float32,
                )
            if msg.velocity:
                self._state.right_arm_qvel = np.asarray(
                    msg.velocity[:7], dtype=np.float32,
                )
            if msg.effort:
                self._state.right_arm_qtau = np.asarray(
                    msg.effort[:7], dtype=np.float32,
                )

    def _on_arm_left_feedback(self, msg) -> None:
        self._stamp_first_seen("arm_left")
        with self._state_lock:
            if msg.position:
                self._state.left_arm_qpos = np.asarray(
                    msg.position[:7], dtype=np.float32,
                )
            if msg.velocity:
                self._state.left_arm_qvel = np.asarray(
                    msg.velocity[:7], dtype=np.float32,
                )
            if msg.effort:
                self._state.left_arm_qtau = np.asarray(
                    msg.effort[:7], dtype=np.float32,
                )

    def _on_gripper_right_feedback(self, msg) -> None:
        self._stamp_first_seen("gripper_right")
        with self._state_lock:
            if msg.position:
                self._state.right_gripper_pos = float(msg.position[0])
            if msg.velocity:
                self._state.right_gripper_vel = float(msg.velocity[0])

    def _on_gripper_left_feedback(self, msg) -> None:
        self._stamp_first_seen("gripper_left")
        with self._state_lock:
            if msg.position:
                self._state.left_gripper_pos = float(msg.position[0])
            if msg.velocity:
                self._state.left_gripper_vel = float(msg.velocity[0])

    def _on_torso_feedback(self, msg) -> None:
        self._stamp_first_seen("torso")
        with self._state_lock:
            if msg.position:
                self._state.torso_qpos = np.asarray(
                    msg.position[:4], dtype=np.float32,
                )
            if msg.velocity:
                self._state.torso_qvel = np.asarray(
                    msg.velocity[:4], dtype=np.float32,
                )

    def _on_chassis_feedback(self, msg) -> None:
        self._stamp_first_seen("chassis")
        with self._state_lock:
            if msg.position:
                self._state.chassis_qpos = np.asarray(
                    msg.position[:3], dtype=np.float32,
                )
            if msg.velocity:
                self._state.chassis_qvel = np.asarray(
                    msg.velocity[:3], dtype=np.float32,
                )

    def _on_imu(self, msg, where: str) -> None:
        target = (
            self._state.imu_torso if where == "torso"
            else self._state.imu_chassis
        )
        with self._state_lock:
            target["orient"] = np.array(
                [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ],
                dtype=np.float32,
            )
            target["ang_vel"] = np.array(
                [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                dtype=np.float32,
            )
            target["lin_acc"] = np.array(
                [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
                dtype=np.float32,
            )

    def _on_bms(self, msg) -> None:
        with self._state_lock:
            self._state.bms.update(
                {
                    "voltage": float(getattr(msg, "voltage", 0.0)),
                    "current": float(getattr(msg, "current", 0.0)),
                    "capital_pct": float(
                        getattr(msg, "capital", getattr(msg, "capital_pct", 100.0))
                    ),
                    "temperature": float(getattr(msg, "temperature", 25.0)),
                }
            )

    def _on_controller_signal(self, msg) -> None:
        with self._state_lock:
            data = getattr(msg, "data", msg)
            self._state.controller_signal = {
                "swa": int(getattr(data, "swa", 0)),
                "swb": int(getattr(data, "swb", 0)),
                "swc": int(getattr(data, "swc", 0)),
                "swd": int(getattr(data, "swd", 0)),
                "mode": int(getattr(data, "mode", 0)),
            }

    def _on_status(self, msg, side: str) -> None:
        with self._state_lock:
            errs = getattr(msg, "errors", []) or []
            self._state.status_errors[side] = [int(e) for e in errs]

    # ── Public API (RPC-callable) ───────────────────────────────

    def get_state(self) -> GalaxeaR1ProRobotState:
        """Return a snapshot of the current robot state."""
        with self._state_lock:
            return self._state.copy()

    def is_robot_up(self) -> bool:
        """Return True when at least one feedback topic is fresh."""
        with self._state_lock:
            return bool(self._state.is_alive)

    def send_arm_pose(self, side: str, pose_xyz_quat: np.ndarray) -> None:
        if self._is_dummy:
            return
        from geometry_msgs.msg import PoseStamped  # type: ignore
        topic = f"target_pose_arm_{side}"
        if topic not in self._pubs:
            raise ValueError(
                f"send_arm_pose: arm '{side}' is not enabled at controller init"
            )
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "torso_link4"
        pose = np.asarray(pose_xyz_quat, dtype=np.float32).reshape(-1)
        if pose.size != 7:
            raise ValueError(
                f"send_arm_pose expects [x y z qx qy qz qw]; got len={pose.size}"
            )
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.position.z = float(pose[2])
        msg.pose.orientation.x = float(pose[3])
        msg.pose.orientation.y = float(pose[4])
        msg.pose.orientation.z = float(pose[5])
        msg.pose.orientation.w = float(pose[6])
        self._pubs[topic].publish(msg)

    def send_arm_joints(self, side: str, qpos: list) -> None:
        if self._is_dummy:
            return
        from sensor_msgs.msg import JointState  # type: ignore
        topic = f"target_joint_state_arm_{side}"
        if topic not in self._pubs:
            raise ValueError(
                f"send_arm_joints: arm '{side}' is not enabled at controller init"
            )
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.position = [float(x) for x in list(qpos)[:7]]
        self._pubs[topic].publish(msg)

    def send_gripper(self, side: str, position_pct: float) -> None:
        if self._is_dummy:
            return
        from sensor_msgs.msg import JointState  # type: ignore
        topic = f"target_position_gripper_{side}"
        if topic not in self._pubs:
            return
        msg = JointState()
        msg.position = [float(np.clip(position_pct, 0.0, 100.0))]
        msg.name = [f"gripper_{side}"]
        self._pubs[topic].publish(msg)

    def send_torso_twist(self, twist4: np.ndarray) -> None:
        if self._is_dummy or "target_speed_torso" not in self._pubs:
            return
        from geometry_msgs.msg import TwistStamped  # type: ignore
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        v = np.asarray(twist4, dtype=np.float32).reshape(-1)
        if v.size < 4:
            v = np.concatenate([v, np.zeros(4 - v.size, dtype=np.float32)])
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.z = float(v[1])
        msg.twist.angular.y = float(v[2])
        msg.twist.angular.z = float(v[3])
        self._pubs["target_speed_torso"].publish(msg)

    def send_chassis_twist(self, twist3: np.ndarray) -> None:
        if self._is_dummy or "target_speed_chassis" not in self._pubs:
            return
        from geometry_msgs.msg import Twist  # type: ignore
        v = np.asarray(twist3, dtype=np.float32).reshape(-1)
        if v.size < 3:
            v = np.concatenate([v, np.zeros(3 - v.size, dtype=np.float32)])
        msg = Twist()
        msg.linear.x = float(v[0])
        msg.linear.y = float(v[1])
        msg.angular.z = float(v[2])
        self._pubs["target_speed_chassis"].publish(msg)

    def apply_brake(self, on: bool) -> None:
        if self._is_dummy or "brake_mode" not in self._pubs:
            return
        from std_msgs.msg import Bool  # type: ignore
        self._pubs["brake_mode"].publish(Bool(data=bool(on)))

    def go_to_rest(
        self, side: str, qpos: list, timeout_s: float = 5.0
    ) -> bool:
        """Send joint tracker to *qpos* and wait for convergence.

        Returns ``True`` once each joint is within 0.03 rad of target,
        ``False`` on timeout.  Used by the env's reset choreography.
        """
        if self._is_dummy:
            return True
        self.send_arm_joints(side, qpos)
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            with self._state_lock:
                cur = self._state.get_arm_qpos(side)
            target = np.asarray(qpos, dtype=np.float32).reshape(-1)[:7]
            if cur.size and target.size and np.allclose(target, cur, atol=0.03):
                return True
            time.sleep(0.05)
        return False

    def clear_errors(self, side: str) -> None:
        """Soft-clear arm errors by re-publishing current pose.

        The Galaxea SDK does not expose a uniform `ErrorRecoveryActionGoal`
        equivalent; the pragmatic approach is to publish a hold-current
        pose and let mobiman re-anchor.  Operators may need to power-cycle
        for hard errors -- see Runbook §13.8.4.
        """
        if self._is_dummy:
            return
        with self._state_lock:
            cur_q = self._state.get_arm_qpos(side).tolist()
        self.send_arm_joints(side, cur_q)

    def shutdown(self) -> None:
        self._spin_running = False
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        # Do NOT call rclpy.shutdown() here in case other workers
        # share the same process.

    @contextmanager
    def hold_state_lock(self):
        """Test helper: yield the state lock + state for assertions."""
        with self._state_lock:
            yield self._state
