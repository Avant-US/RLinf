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

"""Command-line driver for the Galaxea R1 Pro right arm with a
client-side TCP safety box.

This is a sibling of :mod:`toolkits.realworld_check.test_franka_controller`
and :mod:`toolkits.realworld_check.test_turtle2_controller` but adds:

* Three pose input formats:

  * ``--pose-euler  X Y Z R P Y``  -- absolute 6-D pose (``torso_link4``)
  * ``--pose-quat   X Y Z QX QY QZ QW``  -- absolute 7-D pose
  * ``--pose-delta  DX DY DZ DRX DRY DRZ DGRIP``  -- normalised
    ``[-1, 1]^7`` step.  This path runs the **production**
    :class:`GalaxeaR1ProSafetySupervisor` (L1-L5) for parity
    with ``GalaxeaR1ProEnv.step``.

* A configurable safety box.  When a target falls outside the box
  the script prints an ``L3a:right_ee_box`` warning (matching the
  exact tag from :func:`r1_pro_safety._clip_to_box`) and **truncates**
  the target to the box edge before sending it on to the controller.

* Two backends behind a thin abstract base:

  * ``--backend ray``  -- wraps :class:`GalaxeaR1ProController`
    via Ray (the production path; matches Franka/Turtle2 scripts).
  * ``--backend rclpy``  -- a standalone ``rclpy`` node that
    publishes directly to ``/motion_target/target_pose_arm_right``
    without Ray.  Lighter for bring-up on the Orin.

* Both single-shot (``--pose-*``) and an interactive REPL.

Defaults for the safety box come from
:class:`rlinf.envs.realworld.galaxear.r1_pro_safety.SafetyConfig`.
See ``bt/docs/rwRL/safety_2.md`` for the full L1-L5 design and the
known gap on EE pose subscription (§9.11).

Examples
--------

Real robot, Ray backend, conservative box::

    ray start --head --port=6379
    export ROS_DOMAIN_ID=72
    python toolkits/realworld_check/test_galaxea_r1_pro_controller.py \\
        --backend ray --node-rank 0 \\
        --box-min 0.25 -0.25 0.10 \\
        --box-max 0.55  0.25 0.55

Direct rclpy publisher (no Ray)::

    source /opt/ros/humble/setup.bash
    python toolkits/realworld_check/test_galaxea_r1_pro_controller.py \\
        --backend rclpy

Dummy single-shot demo (no robot, no Ray cluster) showing a clipped
out-of-box pose::

    python toolkits/realworld_check/test_galaxea_r1_pro_controller.py \\
        --backend ray --dummy \\
        --pose-euler 0.80 0.0 0.30 -3.14 0.0 0.0
"""

from __future__ import annotations
import os
import argparse
import logging
import shlex
import sys
import time
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from rlinf.envs.realworld.galaxear.r1_pro_action_schema import ActionSchema
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (
    GalaxeaR1ProRobotState,
)
from rlinf.envs.realworld.galaxear.r1_pro_safety import (
    GalaxeaR1ProSafetySupervisor,
    SafetyConfig,
)

_logger = logging.getLogger("r1_pro_arm_cli")
_AXIS_NAMES: Tuple[str, ...] = ("x", "y", "z", "roll", "pitch", "yaw")


# ────────────────────────── Safety helpers ─────────────────────────


def clip_pose_to_box(
    target_xyzrpy: np.ndarray,
    box_min: np.ndarray,
    box_max: np.ndarray,
) -> Tuple[np.ndarray, List[str]]:
    """Clip an absolute EE pose to the safety box.

    Mirrors the production L3a semantics in
    :func:`rlinf.envs.realworld.galaxear.r1_pro_safety._clip_to_box`
    but operates on an *absolute* ``[x y z roll pitch yaw]`` vector
    (rather than the predict-clip-rewrite pipeline that operates on
    normalised action deltas).

    Args:
        target_xyzrpy: Desired EE pose, shape ``(6,)``.
        box_min: Lower box corner, shape ``(6,)``.
        box_max: Upper box corner, shape ``(6,)``.

    Returns:
        A pair ``(clipped, violations)`` where ``violations`` is a
        list of human-readable warning strings.  When the input is
        fully inside the box the list is empty.  The warning tag
        ``L3a:right_ee_box`` matches the production
        :func:`SafetySupervisor._clip_to_box` reason string for log
        cross-referencing.
    """
    target = np.asarray(target_xyzrpy, dtype=np.float32).reshape(-1)[:6]
    lo = np.asarray(box_min, dtype=np.float32).reshape(-1)[:6]
    hi = np.asarray(box_max, dtype=np.float32).reshape(-1)[:6]
    clipped = np.clip(target, lo, hi)
    violations: List[str] = []
    for i, name in enumerate(_AXIS_NAMES):
        if abs(float(target[i]) - float(clipped[i])) > 1e-6:
            face = "max" if target[i] > clipped[i] else "min"
            violations.append(
                f"L3a:right_ee_box {name}={float(target[i]):+.4f} -> "
                f"{float(clipped[i]):+.4f} (clipped to {face} face)"
            )
    return clipped, violations


def euler_to_quat(rpy: np.ndarray) -> np.ndarray:
    """Convert ``[roll pitch yaw]`` (xyz convention) to ``[qx qy qz qw]``."""
    return R.from_euler(
        "xyz", np.asarray(rpy, dtype=np.float64).reshape(-1)
    ).as_quat().astype(np.float32)


def quat_to_euler(quat: np.ndarray) -> np.ndarray:
    """Convert ``[qx qy qz qw]`` to ``[roll pitch yaw]`` (xyz)."""
    return R.from_quat(
        np.asarray(quat, dtype=np.float64).reshape(-1)
    ).as_euler("xyz").astype(np.float32)


def is_finite_vec(v: np.ndarray) -> bool:
    """Return True iff every element is finite."""
    return bool(np.all(np.isfinite(np.asarray(v, dtype=np.float64))))


# ─────────────────────────── Backends ──────────────────────────────


class ControllerBackend(ABC):
    """Backend-neutral interface used by the CLI / REPL."""

    name: str = "base"

    @abstractmethod
    def send_pose(self, pose7: np.ndarray) -> None:
        """Publish a 7-D EE target ``[x y z qx qy qz qw]``."""

    def send_gripper(self, pct: float) -> None:
        """Optional: set gripper opening ``0..100``.  Default no-op."""
        return None

    def apply_brake(self, on: bool) -> None:
        """Optional: brake mode publisher.  Default no-op."""
        return None

    def go_to_rest(
        self, qpos: List[float], timeout_s: float = 5.0
    ) -> bool:
        """Optional: joint-tracker reset.  Default no-op (returns True)."""
        return True

    def get_state(self) -> Optional[GalaxeaR1ProRobotState]:
        """Optional: latest robot snapshot.  Returns None on backends
        that do not subscribe to feedback."""
        return None

    def shutdown(self) -> None:
        return None


class RayBackend(ControllerBackend):
    """Wraps :class:`GalaxeaR1ProController` via its Ray launch helper.

    All RPCs use ``.wait()`` for synchronous semantics, matching
    :mod:`toolkits.realworld_check.test_franka_controller`.
    """

    name = "ray"

    def __init__(
        self,
        *,
        is_dummy: bool,
        node_rank: int,
        ros_domain_id: int,
        ros_localhost_only: bool,
    ) -> None:
        try:
            from rlinf.envs.realworld.galaxear.r1_pro_controller import (
                GalaxeaR1ProController,
            )
        except Exception as e:  # pragma: no cover - env-specific
            raise RuntimeError(
                f"Failed to import GalaxeaR1ProController ({e}). "
                "Did you install RLinf with the embodied target?"
            ) from e
        self._is_dummy = bool(is_dummy)
        try:
            self._handle = GalaxeaR1ProController.launch_controller(
                env_idx=0,
                node_rank=node_rank,
                worker_rank=0,
                ros_domain_id=ros_domain_id,
                ros_localhost_only=ros_localhost_only,
                use_left_arm=False,
                use_right_arm=True,
                use_torso=False,
                use_chassis=False,
                mobiman_launch_mode="pose",
                is_dummy=is_dummy,
            )
        except Exception as e:
            raise RuntimeError(
                f"GalaxeaR1ProController.launch_controller failed: {e}\n"
                "Is the Ray cluster running?  Try:\n"
                "  ray start --head --port=6379\n"
                "Or pass --dummy for offline testing."
            ) from e

    def _wait_until_ready(self, timeout_s: float = 30.0) -> None:
        if self._is_dummy:
            return
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                if bool(self._handle.is_robot_up().wait()[0]):
                    return
            except Exception:
                pass
            time.sleep(0.5)
        _logger.warning(
            "Robot did not report alive within %.1fs; continuing "
            "anyway (dummy=%s).", timeout_s, self._is_dummy,
        )

    def send_pose(self, pose7: np.ndarray) -> None:
        self._handle.send_arm_pose("right", np.asarray(pose7)).wait()

    def send_gripper(self, pct: float) -> None:
        self._handle.send_gripper("right", float(pct)).wait()

    def apply_brake(self, on: bool) -> None:
        self._handle.apply_brake(bool(on)).wait()

    def go_to_rest(
        self, qpos: List[float], timeout_s: float = 5.0
    ) -> bool:
        return bool(
            self._handle.go_to_rest(
                "right", list(qpos), float(timeout_s),
            ).wait()[0]
        )

    def get_state(self) -> Optional[GalaxeaR1ProRobotState]:
        try:
            return self._handle.get_state().wait()[0]
        except Exception as e:
            _logger.warning("get_state RPC failed: %s", e)
            return None


class RclpyBackend(ControllerBackend):
    """Standalone ``rclpy`` publisher node (no Ray, no feedback subs).

    Only knows how to publish three target topics:

    * ``/motion_target/target_pose_arm_right``  (PoseStamped)
    * ``/motion_target/target_position_gripper_right``  (JointState)
    * ``/motion_target/brake_mode``  (Bool)

    State queries (``get_state`` / ``go_to_rest`` convergence wait)
    are not supported by this lightweight backend.
    """

    name = "rclpy"

    def __init__(self, *, ros_domain_id: int, is_dummy: bool) -> None:
        assert ros_domain_id is not None and ros_domain_id > 0, "ROS_DOMAIN_ID must be set and greater than 0"
        os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
        self._is_dummy = bool(is_dummy)
        if is_dummy:
            self._rclpy = None
            self._node = None
            self._pub_pose = None
            self._pub_grip = None
            self._pub_brake = None
            return
        try:
            import rclpy  # type: ignore[import]
            from geometry_msgs.msg import PoseStamped  # type: ignore
            from rclpy.qos import (  # type: ignore
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from sensor_msgs.msg import JointState  # type: ignore
            from std_msgs.msg import Bool  # type: ignore
        except Exception as e:  # pragma: no cover - env-specific
            raise RuntimeError(
                f"rclpy import failed ({e}). "
                "Source your ROS 2 setup first, e.g.\n"
                "  source /opt/ros/humble/setup.bash"
            ) from e
        if not rclpy.ok():
            rclpy.init(args=[])
        self._rclpy = rclpy
        self._PoseStamped = PoseStamped
        self._JointState = JointState
        self._Bool = Bool
        self._node = rclpy.create_node(
            f"rlinf_r1_pro_arm_cli_{int(time.time())}"
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_pose = self._node.create_publisher(
            PoseStamped, "/motion_target/target_pose_arm_right", qos,
        )
        self._pub_grip = self._node.create_publisher(
            JointState,
            "/motion_target/target_position_gripper_right",
            qos,
        )
        self._pub_brake = self._node.create_publisher(
            Bool, "/motion_target/brake_mode", qos,
        )

    def send_pose(self, pose7: np.ndarray) -> None:
        if self._is_dummy or self._node is None:
            return
        p = np.asarray(pose7, dtype=np.float32).reshape(-1)
        if p.size != 7:
            raise ValueError(
                f"send_pose expects 7 floats [x y z qx qy qz qw]; got {p.size}"
            )
        msg = self._PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "torso_link4"
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = float(p[2])
        msg.pose.orientation.x = float(p[3])
        msg.pose.orientation.y = float(p[4])
        msg.pose.orientation.z = float(p[5])
        msg.pose.orientation.w = float(p[6])
        self._pub_pose.publish(msg)

    def send_gripper(self, pct: float) -> None:
        if self._is_dummy or self._node is None:
            return
        msg = self._JointState()
        msg.position = [float(np.clip(pct, 0.0, 100.0))]
        msg.name = ["gripper_right"]
        self._pub_grip.publish(msg)

    def apply_brake(self, on: bool) -> None:
        if self._is_dummy or self._node is None:
            return
        self._pub_brake.publish(self._Bool(data=bool(on)))

    def go_to_rest(
        self, qpos: List[float], timeout_s: float = 5.0
    ) -> bool:
        if self._is_dummy:
            # Mirror the no-op contract of send_pose / apply_brake in
            # dummy mode: succeed silently so home / zero exercises run
            # without a real joint-tracker publisher.
            return True
        _logger.warning(
            "go_to_rest is not supported by the rclpy backend "
            "(no joint-tracker publisher).  Use --backend ray instead."
        )
        return False

    def get_state(self) -> Optional[GalaxeaR1ProRobotState]:
        return None

    def shutdown(self) -> None:
        if self._is_dummy or self._node is None:
            return
        try:
            self._node.destroy_node()
        except Exception:
            pass


# ───────────────────────── CLI / REPL core ─────────────────────────


class ArmSafetyCLI:
    """Owns the safety box, the supervisor (for delta path) and the
    backend.  Provides ``send_pose_euler`` / ``send_pose_quat`` /
    ``send_pose_delta`` plus a simple REPL."""

    def __init__(
        self,
        backend: ControllerBackend,
        box_min: np.ndarray,
        box_max: np.ndarray,
        action_scale: np.ndarray,
        home_qpos: List[float],
        zero_qpos: List[float],
    ) -> None:
        self._backend = backend
        self._box_min = np.asarray(box_min, dtype=np.float32).reshape(-1)
        self._box_max = np.asarray(box_max, dtype=np.float32).reshape(-1)
        self._action_scale = np.asarray(
            action_scale, dtype=np.float32,
        ).reshape(-1)
        self._home_qpos = list(home_qpos)
        self._zero_qpos = list(zero_qpos)
        # L2 reference limits, sourced from the production SafetyConfig.
        _q_cfg = SafetyConfig()
        self._joint_q_min = np.asarray(_q_cfg.arm_q_min, dtype=np.float32)
        self._joint_q_max = np.asarray(_q_cfg.arm_q_max, dtype=np.float32)
        self._validate_box()

        # Lazily-built supervisor and schema for the delta path.
        self._schema = ActionSchema(
            has_left_arm=False,
            has_right_arm=True,
            has_torso=False,
            has_chassis=False,
            no_gripper=False,
            action_scale=self._action_scale.copy(),
        )
        self._supervisor = self._build_supervisor()

        # Last commanded EE pose (for echo / setpos quaternion handover).
        self._last_pose7: Optional[np.ndarray] = None

    # ── Box plumbing ───────────────────────────────────────────

    def _validate_box(self) -> None:
        if np.any(self._box_min >= self._box_max):
            bad = [
                _AXIS_NAMES[i]
                for i in range(6)
                if self._box_min[i] >= self._box_max[i]
            ]
            raise ValueError(
                f"box_min must be strictly less than box_max on every "
                f"axis; offending axes: {bad}\n"
                f"  min = {self._box_min.tolist()}\n"
                f"  max = {self._box_max.tolist()}"
            )

    def _build_supervisor(self) -> GalaxeaR1ProSafetySupervisor:
        cfg = SafetyConfig()
        cfg.right_ee_min = self._box_min.copy()
        cfg.right_ee_max = self._box_max.copy()
        # Single-arm CLI: no L3b dual-arm collision path.
        cfg.dual_arm_collision_enable = False
        # Disable the operator heartbeat watchdog (safety_2.md §9.4):
        # this CLI is interactive and the user IS the operator, so we
        # do not want soft_hold to fire after 1.5 s of typing.
        cfg.operator_heartbeat_timeout_ms = 999_999.0
        return GalaxeaR1ProSafetySupervisor(cfg)

    def update_box_min(self, new_min: np.ndarray) -> None:
        self._box_min = np.asarray(new_min, dtype=np.float32).reshape(-1)
        self._validate_box()
        self._supervisor = self._build_supervisor()

    def update_box_max(self, new_max: np.ndarray) -> None:
        self._box_max = np.asarray(new_max, dtype=np.float32).reshape(-1)
        self._validate_box()
        self._supervisor = self._build_supervisor()

    @property
    def box_min(self) -> np.ndarray:
        return self._box_min

    @property
    def box_max(self) -> np.ndarray:
        return self._box_max

    # ── Send paths ─────────────────────────────────────────────

    def _send_clipped_xyzrpy(self, target_xyzrpy: np.ndarray) -> bool:
        """Common path for ``setpose`` / ``setposq``.

        Returns True iff a pose was published.
        """
        if not is_finite_vec(target_xyzrpy):
            print("[ERROR] L1:non_finite_action -- refusing to send.")
            return False
        clipped, violations = clip_pose_to_box(
            target_xyzrpy, self._box_min, self._box_max,
        )
        for v in violations:
            print(f"[WARN]  {v}")
        quat = euler_to_quat(clipped[3:])
        pose7 = np.concatenate([clipped[:3], quat]).astype(np.float32)
        self._backend.send_pose(pose7)
        self._last_pose7 = pose7
        print(
            "[INFO]  Sent pose7 = ["
            + ", ".join(f"{float(x):+.4f}" for x in pose7)
            + "]"
        )
        return True

    def send_pose_euler(self, xyzrpy: np.ndarray) -> bool:
        """``setpose x y z r p y``."""
        return self._send_clipped_xyzrpy(np.asarray(xyzrpy, dtype=np.float32))

    def send_pose_quat(self, xyz_quat: np.ndarray) -> bool:
        """``setposq x y z qx qy qz qw``."""
        v = np.asarray(xyz_quat, dtype=np.float32).reshape(-1)
        if v.size != 7:
            print(f"[ERROR] setposq needs 7 floats; got {v.size}")
            return False
        if not is_finite_vec(v):
            print("[ERROR] L1:non_finite_action -- refusing to send.")
            return False
        # Re-normalise quaternion (defensive: user may type unit-norm-ish).
        q = v[3:].astype(np.float64)
        n = float(np.linalg.norm(q))
        if n < 1e-9:
            print(
                "[ERROR] quaternion has zero norm; cannot interpret "
                "orientation."
            )
            return False
        q = (q / n).astype(np.float32)
        rpy = quat_to_euler(q)
        target = np.concatenate([v[:3], rpy]).astype(np.float32)
        return self._send_clipped_xyzrpy(target)

    def send_pose_delta(self, delta7: np.ndarray) -> bool:
        """``setdelta dx dy dz drx dry drz dgrip`` -- runs the FULL
        production safety pipeline (L1-L5) for env-step parity.

        Requires a backend that exposes :meth:`get_state`; falls back
        to a zero-EE state on backends that do not (e.g. rclpy direct,
        or §9.11 EE-pose-not-subscribed)."""
        a = np.asarray(delta7, dtype=np.float32).reshape(-1)
        if a.size != 7:
            print(f"[ERROR] setdelta needs 7 floats; got {a.size}")
            return False
        state = self._backend.get_state()
        if state is None:
            state = GalaxeaR1ProRobotState()
            print(
                "[NOTE]  Backend does not expose state; using a zero EE "
                "snapshot.  Predicted EE = action * scale (origin-anchored)."
            )
        info = self._supervisor.validate(a, state, self._schema)
        for r in info.reason:
            tag = (
                "[ERROR]" if info.emergency_stop
                else "[WARN] " if (info.safe_stop or info.soft_hold)
                else "[INFO] "
            )
            print(f"{tag} {r}")
        if info.emergency_stop:
            self._backend.apply_brake(True)
            print("[EMERG] apply_brake(True); refusing to publish target.")
            return False
        if info.safe_stop:
            self._backend.apply_brake(True)
            print("[SAFE]  apply_brake(True); refusing to publish target.")
            return False
        if info.soft_hold:
            print("[HOLD]  L3b/L5 hold; not publishing this step.")
            return False
        target = self._schema.predict_arm_ee_pose(
            "right", info.safe_action, state,
        )
        if target is None:
            print("[ERROR] schema returned no target; aborting.")
            return False
        # The supervisor has already L3a-clipped the predicted EE via
        # the predict-clip-rewrite path; predict-again here gives us
        # the final target in xyz+rpy form for the controller.
        quat = euler_to_quat(target[3:])
        pose7 = np.concatenate([target[:3], quat]).astype(np.float32)
        self._backend.send_pose(pose7)
        self._last_pose7 = pose7
        print(
            "[INFO]  Sent pose7 = ["
            + ", ".join(f"{float(x):+.4f}" for x in pose7)
            + "]"
        )
        return True

    # ── REPL ──────────────────────────────────────────────────

    def print_box(self) -> None:
        print(
            "Safety box (torso_link4 frame):\n"
            f"  min = ["
            + ", ".join(f"{float(x):+.4f}" for x in self._box_min)
            + "]\n"
            f"  max = ["
            + ", ".join(f"{float(x):+.4f}" for x in self._box_max)
            + "]"
        )

    def print_state(self) -> None:
        st = self._backend.get_state()
        if st is None:
            print(
                "[N/A]   Backend does not expose state.  rclpy backend "
                "publishes only; use --backend ray for feedback."
            )
            return
        print(
            f"  right_ee_pose       = {np.round(st.right_ee_pose, 4).tolist()}"
            f"\n  right_arm_qpos      = {np.round(st.right_arm_qpos, 4).tolist()}"
            f"\n  right_gripper_pos   = {st.right_gripper_pos:.2f}"
            f"\n  bms_capital_pct     = "
            f"{st.bms.get('capital_pct', float('nan')):.1f}"
            f"\n  controller.swd      = {st.controller_signal.get('swd', 0)}"
            f"\n  feedback_age_ms     = "
            f"{ {k: round(v, 1) for k, v in st.feedback_age_ms.items()} }"
            f"\n  is_alive            = {st.is_alive}"
        )

    def print_pos(self) -> None:
        st = self._backend.get_state()
        if st is None or float(np.linalg.norm(st.right_ee_pose[:3])) < 1e-9:
            note = (
                "[N/A]   EE pose unavailable.  Either backend lacks "
                "feedback, or the controller has not subscribed to "
                "/motion_control/pose_ee_arm_right (see safety_2.md "
                "§9.11)."
            )
            if self._last_pose7 is not None:
                print(note)
                print(
                    "        Last commanded pose7 = ["
                    + ", ".join(f"{float(x):+.4f}" for x in self._last_pose7)
                    + "]"
                )
            else:
                print(note)
            return
        rpy = quat_to_euler(st.right_ee_pose[3:])
        print(
            "  right_ee_pose (xyz + quat) = ["
            + ", ".join(f"{float(x):+.4f}" for x in st.right_ee_pose)
            + "]\n"
            "  right_ee_pose (xyz + rpy)  = ["
            + ", ".join(
                f"{float(x):+.4f}"
                for x in np.concatenate([st.right_ee_pose[:3], rpy])
            )
            + "]"
        )

    # ── Home / Zero (joint-space) ──────────────────────────────

    def _check_qpos_within_limits(self, qpos) -> List[str]:
        """L2 sanity check using ``SafetyConfig.arm_q_min/q_max``.

        Returns a list of human-readable violation strings.  The tag
        prefix ``L2:right_arm_q`` mirrors the reason naming convention
        used by :class:`GalaxeaR1ProSafetySupervisor` (see §4.2 / §9.1
        of ``bt/docs/rwRL/safety_2.md``).
        """
        q = np.asarray(qpos, dtype=np.float32).reshape(-1)[:7]
        if q.size != 7:
            return [f"L2:right_arm_q expected 7 floats, got {q.size}"]
        violations: List[str] = []
        for i in range(7):
            qi = float(q[i])
            lo = float(self._joint_q_min[i])
            hi = float(self._joint_q_max[i])
            # Inclusive comparison: a target sitting *at* the limit
            # leaves zero control margin for tracking noise / overshoot,
            # which the L2 design intent treats as unsafe.
            if qi <= lo or qi >= hi:
                violations.append(
                    f"L2:right_arm_q J{i + 1}={qi:+.4f} not strictly "
                    f"inside ({lo:+.4f},{hi:+.4f})"
                )
        return violations

    def goto_qpos(
        self,
        label: str,
        qpos: List[float],
        *,
        require_confirm: bool = False,
        skip_joint_check: bool = False,
        timeout_s: float = 5.0,
    ) -> bool:
        """Common joint-tracker path used by :meth:`home` / :meth:`zero`.

        Args:
            label: Display name (``"HOME"`` / ``"ZERO"``).
            qpos: 7-D right-arm joint configuration in radians.
            require_confirm: When True and ``stdin`` is a TTY, prompt
                for an explicit ``YES`` confirmation before sending.
            skip_joint_check: When True, bypass the L2 limit check
                (the violations are still printed as warnings).
            timeout_s: Per-axis convergence timeout for the controller.

        Returns:
            True iff a target was successfully sent and accepted.
            False on input error, refusal, cancellation, or timeout.
        """
        q = np.asarray(qpos, dtype=np.float32).reshape(-1)
        if q.size != 7:
            print(
                f"[ERROR] {label} qpos needs 7 floats; got {q.size}.  "
                "Refusing to send."
            )
            return False
        if not is_finite_vec(q):
            print(
                f"[ERROR] L1:non_finite_qpos -- refusing to send {label}."
            )
            return False
        formatted = "[" + ", ".join(f"{float(x):+.4f}" for x in q) + "]"
        print(f"[INFO]  Target {label} qpos = {formatted}")
        violations = self._check_qpos_within_limits(q)
        for v in violations:
            print(f"[WARN]  {v}")
        if violations and not skip_joint_check:
            print(
                f"[ERROR] {label} qpos violates SafetyConfig joint "
                "limits; refusing to send.  Pass --zero-skip-joint-check "
                "(single-shot) or 'zero --force' (REPL) to override after "
                "verifying physical clearance."
            )
            return False
        if require_confirm and sys.stdin.isatty():
            try:
                ans = input(
                    f"Confirm move to {label}? Type 'YES' to proceed: "
                ).strip()
            except (EOFError, KeyboardInterrupt):
                print(f"\n[INFO]  {label} cancelled.")
                return False
            if ans != "YES":
                print(f"[INFO]  {label} cancelled.")
                return False
        ok = self._backend.go_to_rest(list(q), timeout_s=float(timeout_s))
        print(
            f"  go_to_rest({label}) -> "
            f"{'OK' if ok else 'TIMEOUT/UNSUPPORTED'}"
        )
        return bool(ok)

    def home(self) -> bool:
        """Drive the right arm to the configured Home Position."""
        return self.goto_qpos("HOME", self._home_qpos)

    def zero(self, *, force: bool = False) -> bool:
        """Drive the right arm to the configured Zero Position.

        Requires a TTY ``YES`` confirmation in REPL mode.  Refuses to
        send by default if the target violates the SafetyConfig joint
        limits; pass ``force=True`` to override.
        """
        return self.goto_qpos(
            "ZERO",
            self._zero_qpos,
            require_confirm=True,
            skip_joint_check=force,
        )

    def set_home_qpos(self, qpos: List[float]) -> None:
        q = np.asarray(qpos, dtype=np.float32).reshape(-1)
        if q.size != 7 or not is_finite_vec(q):
            print("[ERROR] set-home expects 7 finite floats.")
            return
        self._home_qpos = [float(x) for x in q]
        print(f"  HOME qpos updated to {self._home_qpos}")

    def set_zero_qpos(self, qpos: List[float]) -> None:
        q = np.asarray(qpos, dtype=np.float32).reshape(-1)
        if q.size != 7 or not is_finite_vec(q):
            print("[ERROR] set-zero expects 7 finite floats.")
            return
        self._zero_qpos = [float(x) for x in q]
        print(f"  ZERO qpos updated to {self._zero_qpos}")

    def print_homes(self) -> None:
        def _fmt(label: str, q: List[float]) -> None:
            print(
                f"  {label} qpos = ["
                + ", ".join(f"{float(x):+.4f}" for x in q)
                + "]"
            )
            for v in self._check_qpos_within_limits(q):
                print(f"    {v}")
        _fmt("HOME", self._home_qpos)
        _fmt("ZERO", self._zero_qpos)
        print(
            "  q_min = ["
            + ", ".join(f"{float(x):+.4f}" for x in self._joint_q_min)
            + "]"
        )
        print(
            "  q_max = ["
            + ", ".join(f"{float(x):+.4f}" for x in self._joint_q_max)
            + "]"
        )

    def brake(self, on: bool) -> None:
        self._backend.apply_brake(on)
        print(f"  apply_brake({on})")

    def gripper(self, pct: float) -> None:
        self._backend.send_gripper(pct)
        print(f"  send_gripper({pct:.1f})")

    # ── Command dispatch ──────────────────────────────────────

    _HELP = """\
Commands (all numbers are space-separated floats unless noted):

  setpose  X Y Z R P Y           absolute 6-D pose (rpy in radians)
  setposq  X Y Z QX QY QZ QW     absolute 7-D pose (quaternion xyzw)
  setdelta DX DY DZ DRX DRY DRZ DGRIP
                                 normalised [-1,1]^7 step; runs the full
                                 production L1-L5 safety pipeline.
  getbox                         print current safety box
  setbox-min X Y Z [R P Y]       redefine box lower corner
  setbox-max X Y Z [R P Y]       redefine box upper corner
  getpos                         print current EE pose (or last commanded)
  getstate                       print simplified robot snapshot
  home                           drive arm to Home Position (joint-space)
  zero [--force]                 drive arm to Zero Position (joint-space);
                                 prompts for YES confirmation in TTY;
                                 refuses if L2 joint limits are violated
                                 unless --force is supplied
  set-home Q1 Q2 Q3 Q4 Q5 Q6 Q7  redefine HOME qpos at runtime
  set-zero Q1 Q2 Q3 Q4 Q5 Q6 Q7  redefine ZERO qpos at runtime
  gethomes                       print HOME / ZERO qpos and L2 status
  brake on|off                   apply / release brake
  gripper PCT                    set gripper opening (0-100)
  help, ?                        this help
  q, quit, exit                  exit
"""

    def _cmd_setpose(self, args: List[str]) -> None:
        if len(args) != 6:
            print("Usage: setpose X Y Z R P Y")
            return
        try:
            xyzrpy = np.array([float(x) for x in args], dtype=np.float32)
        except ValueError as e:
            print(f"[ERROR] could not parse 6 floats: {e}")
            return
        self.send_pose_euler(xyzrpy)

    def _cmd_setposq(self, args: List[str]) -> None:
        if len(args) != 7:
            print("Usage: setposq X Y Z QX QY QZ QW")
            return
        try:
            v = np.array([float(x) for x in args], dtype=np.float32)
        except ValueError as e:
            print(f"[ERROR] could not parse 7 floats: {e}")
            return
        self.send_pose_quat(v)

    def _cmd_setdelta(self, args: List[str]) -> None:
        if len(args) != 7:
            print("Usage: setdelta DX DY DZ DRX DRY DRZ DGRIP")
            return
        try:
            a = np.array([float(x) for x in args], dtype=np.float32)
        except ValueError as e:
            print(f"[ERROR] could not parse 7 floats: {e}")
            return
        self.send_pose_delta(a)

    def _cmd_setbox(self, side: str, args: List[str]) -> None:
        if len(args) not in (3, 6):
            print(f"Usage: setbox-{side} X Y Z [R P Y]")
            return
        try:
            new = [float(x) for x in args]
        except ValueError as e:
            print(f"[ERROR] could not parse floats: {e}")
            return
        cur = (self._box_min if side == "min" else self._box_max).copy()
        if len(new) == 3:
            cur[:3] = np.asarray(new, dtype=np.float32)
        else:
            cur[:6] = np.asarray(new, dtype=np.float32)
        try:
            if side == "min":
                self.update_box_min(cur)
            else:
                self.update_box_max(cur)
        except ValueError as e:
            print(f"[ERROR] {e}")
            return
        self.print_box()

    def _cmd_brake(self, args: List[str]) -> None:
        if len(args) != 1 or args[0] not in ("on", "off", "1", "0"):
            print("Usage: brake on|off")
            return
        self.brake(args[0] in ("on", "1"))

    def _cmd_gripper(self, args: List[str]) -> None:
        if len(args) != 1:
            print("Usage: gripper PCT (0-100)")
            return
        try:
            pct = float(args[0])
        except ValueError as e:
            print(f"[ERROR] could not parse float: {e}")
            return
        self.gripper(pct)

    def _cmd_zero(self, args: List[str]) -> None:
        force = False
        # Accept either "zero --force" or "zero force" / "zero -f".
        if args and args[0] in ("--force", "-f", "force"):
            force = True
            args = args[1:]
        if args:
            print("Usage: zero [--force]")
            return
        self.zero(force=force)

    def _cmd_set_home(self, args: List[str]) -> None:
        if len(args) != 7:
            print("Usage: set-home Q1 Q2 Q3 Q4 Q5 Q6 Q7")
            return
        try:
            q = [float(x) for x in args]
        except ValueError as e:
            print(f"[ERROR] could not parse 7 floats: {e}")
            return
        self.set_home_qpos(q)

    def _cmd_set_zero(self, args: List[str]) -> None:
        if len(args) != 7:
            print("Usage: set-zero Q1 Q2 Q3 Q4 Q5 Q6 Q7")
            return
        try:
            q = [float(x) for x in args]
        except ValueError as e:
            print(f"[ERROR] could not parse 7 floats: {e}")
            return
        self.set_zero_qpos(q)

    def repl(self) -> None:
        print(self._HELP)
        self.print_box()
        while True:
            try:
                line = input("r1pro> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            try:
                tokens = shlex.split(line)
            except ValueError as e:
                print(f"[ERROR] could not tokenise: {e}")
                continue
            cmd, args = tokens[0].lower(), tokens[1:]
            try:
                if cmd in ("q", "quit", "exit"):
                    break
                if cmd in ("help", "?"):
                    print(self._HELP)
                elif cmd == "setpose":
                    self._cmd_setpose(args)
                elif cmd == "setposq":
                    self._cmd_setposq(args)
                elif cmd == "setdelta":
                    self._cmd_setdelta(args)
                elif cmd == "getbox":
                    self.print_box()
                elif cmd == "setbox-min":
                    self._cmd_setbox("min", args)
                elif cmd == "setbox-max":
                    self._cmd_setbox("max", args)
                elif cmd == "getpos":
                    self.print_pos()
                elif cmd == "getstate":
                    self.print_state()
                elif cmd == "home":
                    self.home()
                elif cmd == "zero":
                    self._cmd_zero(args)
                elif cmd in ("set-home", "sethome"):
                    self._cmd_set_home(args)
                elif cmd in ("set-zero", "setzero"):
                    self._cmd_set_zero(args)
                elif cmd in ("gethomes", "gethome", "homes"):
                    self.print_homes()
                elif cmd == "brake":
                    self._cmd_brake(args)
                elif cmd == "gripper":
                    self._cmd_gripper(args)
                else:
                    print(
                        f"Unknown command: {cmd!r}.  Type 'help' for "
                        "the list."
                    )
            except KeyboardInterrupt:
                print("\n[INTR]  command interrupted")
                continue
            except Exception as e:  # pragma: no cover - REPL safety net
                print(f"[ERROR] {type(e).__name__}: {e}")


# ──────────────────────────── argparse ─────────────────────────────


_DEFAULT_BOX_MIN = SafetyConfig().right_ee_min.copy()
_DEFAULT_BOX_MAX = SafetyConfig().right_ee_max.copy()
_DEFAULT_ACTION_SCALE = (0.05, 0.10, 1.0)
# Home Position: operator-defined "ready" joint configuration.
# Mirrors :data:`GalaxeaR1ProRobotConfig.joint_reset_qpos_right`.
_DEFAULT_HOME_QPOS = (0.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.0)
# Zero Position: kinematic origin of the right arm (all joint angles = 0).
# For Galaxea A2 this lies on / outside the default
# ``SafetyConfig.arm_q_min/q_max`` (q4_max = 0.0, q6_min = -0.1), so the
# CLI's L2 sanity check refuses to send it unless the operator passes
# ``--zero-skip-joint-check`` (single-shot) or ``zero --force`` (REPL).
_DEFAULT_ZERO_QPOS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# Backward-compatible alias for the deprecated ``--joint-reset-qpos`` flag.
_DEFAULT_JOINT_RESET_QPOS = _DEFAULT_HOME_QPOS


def _parse_box_arg(values: List[float], name: str) -> np.ndarray:
    """Allow either 3 (xyz only) or 6 (xyzrpy) floats."""
    if len(values) == 3:
        # Inherit rpy from the matching default.
        base = (
            _DEFAULT_BOX_MIN if name == "min" else _DEFAULT_BOX_MAX
        ).copy()
        base[:3] = np.asarray(values, dtype=np.float32)
        return base
    if len(values) == 6:
        return np.asarray(values, dtype=np.float32)
    raise argparse.ArgumentTypeError(
        f"--box-{name} expects 3 or 6 floats; got {len(values)}"
    )


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="test_galaxea_r1_pro_controller",
        description=(
            "CLI driver for the Galaxea R1 Pro right arm with a "
            "client-side TCP safety box (L3a)."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--backend",
        choices=("ray", "rclpy"),
        default="ray",
        help="Controller backend.",
    )
    p.add_argument(
        "--dummy",
        action="store_true",
        help="Do not actually publish; useful for offline testing.",
    )
    p.add_argument(
        "--node-rank",
        type=int,
        default=0,
        help="Cluster node rank for the Ray backend "
             "(0=co-located, 1=Orin in the recommended 2-node setup).",
    )
    p.add_argument(
        "--ros-domain-id",
        type=int,
        default=None,
        help="ROS_DOMAIN_ID for the controller / publishers.",
    )
    p.add_argument(
        "--ros-localhost-only",
        action="store_true",
        help="Set ROS_LOCALHOST_ONLY=1 (Ray backend only).",
    )
    p.add_argument(
        "--box-min",
        nargs="+",
        type=float,
        default=None,
        help="Safety-box lower corner: 'X Y Z' or 'X Y Z R P Y'.",
    )
    p.add_argument(
        "--box-max",
        nargs="+",
        type=float,
        default=None,
        help="Safety-box upper corner: 'X Y Z' or 'X Y Z R P Y'.",
    )
    p.add_argument(
        "--action-scale",
        nargs=3,
        type=float,
        metavar=("POS", "ORI", "GRIP"),
        default=list(_DEFAULT_ACTION_SCALE),
        help="Per-step action scale used by the delta path.",
    )
    p.add_argument(
        "--home-qpos",
        nargs=7,
        type=float,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        default=list(_DEFAULT_HOME_QPOS),
        help="Home Position joint configuration (rad) used by 'home'.",
    )
    p.add_argument(
        "--zero-qpos",
        nargs=7,
        type=float,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        default=list(_DEFAULT_ZERO_QPOS),
        help="Zero Position joint configuration (rad) used by 'zero'.",
    )
    p.add_argument(
        "--zero-skip-joint-check",
        action="store_true",
        help="Skip the L2 joint-limit sanity check when going to zero. "
             "Use ONLY after physically verifying the zero pose is "
             "reachable and collision-free.",
    )
    p.add_argument(
        "--joint-reset-qpos",
        nargs=7,
        type=float,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        default=None,
        help="DEPRECATED: alias of --home-qpos.  Will be removed.",
    )
    g = p.add_mutually_exclusive_group()
    g.add_argument(
        "--pose-euler",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "R", "P", "Y"),
        help="Single-shot: send absolute 6-D pose then exit.",
    )
    g.add_argument(
        "--pose-quat",
        nargs=7,
        type=float,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Single-shot: send absolute 7-D quaternion pose then exit.",
    )
    g.add_argument(
        "--pose-delta",
        nargs=7,
        type=float,
        metavar=("DX", "DY", "DZ", "DRX", "DRY", "DRZ", "DGRIP"),
        help="Single-shot: send normalised [-1,1]^7 delta then exit.",
    )
    g.add_argument(
        "--home",
        action="store_true",
        help="Single-shot: drive the right arm to Home Position then exit.",
    )
    g.add_argument(
        "--zero",
        action="store_true",
        help="Single-shot: drive the right arm to Zero Position then exit.",
    )
    p.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable DEBUG-level logging on the script's logger.",
    )
    return p


# ────────────────────────────── main ───────────────────────────────


def _build_backend(args: argparse.Namespace) -> ControllerBackend:
    if args.backend == "ray":
        return RayBackend(
            is_dummy=bool(args.dummy),
            node_rank=int(args.node_rank),
            ros_domain_id=int(args.ros_domain_id),
            ros_localhost_only=bool(args.ros_localhost_only),
        )
    if args.backend == "rclpy":
        return RclpyBackend(
            ros_domain_id=int(args.ros_domain_id),
            is_dummy=bool(args.dummy),
        )
    raise AssertionError(f"unknown backend: {args.backend!r}")


def main(argv: Optional[List[str]] = None) -> int:
    args = build_argparser().parse_args(argv)
    if args.ros_domain_id is None:
        args.ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", 41))
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    box_min = (
        _parse_box_arg(args.box_min, "min")
        if args.box_min is not None
        else _DEFAULT_BOX_MIN.copy()
    )
    box_max = (
        _parse_box_arg(args.box_max, "max")
        if args.box_max is not None
        else _DEFAULT_BOX_MAX.copy()
    )

    # --joint-reset-qpos is a deprecated alias of --home-qpos.  Honour it
    # only when the operator explicitly passes it (the default is None).
    home_qpos = list(args.home_qpos)
    if args.joint_reset_qpos is not None:
        print(
            "[DEPR]  --joint-reset-qpos is deprecated; use --home-qpos. "
            "Falling back to the supplied joint-reset-qpos this run.",
            file=sys.stderr,
        )
        home_qpos = list(args.joint_reset_qpos)

    try:
        cli = ArmSafetyCLI(
            backend=_build_backend(args),
            box_min=box_min,
            box_max=box_max,
            action_scale=np.asarray(args.action_scale, dtype=np.float32),
            home_qpos=home_qpos,
            zero_qpos=list(args.zero_qpos),
        )
    except (ValueError, RuntimeError) as e:
        print(f"[FATAL] {e}", file=sys.stderr)
        return 2

    print(
        f"[INFO]  Backend: {args.backend}"
        f"{' (dummy)' if args.dummy else ''}"
    )
    cli.print_box()

    # If a single-shot --pose-* / --home / --zero flag is set, send and exit.
    try:
        if args.pose_euler is not None:
            ok = cli.send_pose_euler(
                np.asarray(args.pose_euler, dtype=np.float32)
            )
            return 0 if ok else 1
        if args.pose_quat is not None:
            ok = cli.send_pose_quat(
                np.asarray(args.pose_quat, dtype=np.float32)
            )
            return 0 if ok else 1
        if args.pose_delta is not None:
            ok = cli.send_pose_delta(
                np.asarray(args.pose_delta, dtype=np.float32)
            )
            return 0 if ok else 1
        if args.home:
            if isinstance(cli._backend, RayBackend):
                cli._backend._wait_until_ready(timeout_s=30.0)
            ok = cli.home()
            return 0 if ok else 1
        if args.zero:
            if isinstance(cli._backend, RayBackend):
                cli._backend._wait_until_ready(timeout_s=30.0)
            ok = cli.zero(force=bool(args.zero_skip_joint_check))
            return 0 if ok else 1

        # Wait for the controller to come up before opening the REPL
        # (ray backend only; rclpy / dummy go straight to the prompt).
        if isinstance(cli._backend, RayBackend):
            cli._backend._wait_until_ready(timeout_s=30.0)

        cli.repl()
        return 0
    except KeyboardInterrupt:
        print("\n[INTR]  exiting on Ctrl-C")
        return 130
    finally:
        try:
            cli._backend.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
