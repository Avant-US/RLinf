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

"""Strategy-pattern action dispatcher for Galaxea R1 Pro.

Per design doc r1pro6op47.md §3.1, the dispatcher translates a
normalised policy action vector ``[-1, 1]^D`` into concrete ROS 2
publishes via :class:`GalaxeaR1ProController`.  It hides the
joint-mode vs ee-mode split from :class:`GalaxeaR1ProEnv`, so the env
just calls ``dispatcher.dispatch(safe_action, state)`` and never
branches on the action mode itself.

Two concrete dispatchers are provided here:

* :class:`JointStateDispatcher` -- 7 joint targets + 1 gripper per arm.
  Reverse-mapped from ``[-1, 1]`` to absolute joint positions inside
  ``[q_min, q_max]`` (per-arm, per-joint).  This is the recommended
  mode for Pi0.5 on R1 Pro since most published VLA training data
  (LIBERO / Behavior style) is naturally in joint space, and we sidestep
  IK singularity issues entirely.

* :class:`EePoseDispatcher` -- 3 xyz + 4 quaternion (xyzw) + 1 gripper
  per arm.  Reverse-mapped from ``[-1, 1]`` xyz to absolute Cartesian
  in a workspace box ``[ee_min, ee_max]``; quaternion is L2-normalised
  and forced into ``W >= 0`` canonical form to avoid the policy
  outputting ``q`` and ``-q`` on alternating steps (same rotation,
  different sign).

Both dispatchers expose :meth:`get_required_topics` so the env can
fail-fast at startup if the corresponding mobiman subscriber chain is
not running (the most common bring-up bug, see design §1.2 #9).

The action layout is **always 8 dims per arm with gripper enabled**
(7 joints + 1 gripper, OR 3 xyz + 4 quat + 1 gripper) and **7 dims per
arm without gripper** (7 joints, OR 3 xyz + 4 quat -- yes, ee mode
without gripper is still 7 dims because we drop the gripper dim only).
"""

from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from .r1_pro_gripper_mixer import GripperMixer
from .r1_pro_robot_state import GalaxeaR1ProRobotState

_logger = logging.getLogger(__name__)


# ──────────────────────── DispatchResult ────────────────────────


class DispatchResult:
    """Snapshot of one dispatch call -- what was sent and any per-side
    metadata.  Used for metrics / logging; the ``Worker.method`` Ray
    refs (if returned by send_*) live in ``ref_handles`` for callers
    that want to ``wait()``.

    Attributes:
        mode: ``"joint"`` or ``"ee"``.
        commands: dict ``side -> dict`` of what was sent, e.g.
            ``{"right": {"q_target": [0.1, ...], "gripper_pct": 45.0}}``.
            Useful for tensorboard / wandb action histograms.
        ref_handles: list of Ray ObjectRef-like handles returned by
            ``controller.send_*``; callers may ``ref.wait()`` if they
            need a synchronous barrier (typically only on reset).
    """

    def __init__(
        self,
        mode: str,
        commands: Optional[dict] = None,
        ref_handles: Optional[list] = None,
    ):
        self.mode = mode
        self.commands = commands if commands is not None else {}
        self.ref_handles = ref_handles if ref_handles is not None else []


# ──────────────────────── Abstract base ─────────────────────────


class ActionDispatcher:
    """Strategy interface for action -> ROS 2 translation.

    Subclasses must implement :meth:`dispatch`, :meth:`reset_to_safe_pose`,
    and :meth:`get_required_topics`.  :meth:`verify_topology` is
    provided as a default impl that polls the controller for subscriber
    counts on every required topic.
    """

    mode: str = "abstract"

    def __init__(
        self,
        *,
        use_right_arm: bool,
        use_left_arm: bool,
        no_gripper: bool,
        gripper_mixer: GripperMixer,
        controller=None,
    ):
        self._use_right_arm = bool(use_right_arm)
        self._use_left_arm = bool(use_left_arm)
        self._no_gripper = bool(no_gripper)
        self._gripper_mixer = gripper_mixer
        self._controller = controller

    @property
    def per_arm_dim(self) -> int:
        """Per-arm action width.  Subclasses must override."""
        raise NotImplementedError

    @property
    def action_dim(self) -> int:
        d = 0
        if self._use_right_arm:
            d += self.per_arm_dim
        if self._use_left_arm:
            d += self.per_arm_dim
        return max(d, 1)

    def split(self, action: np.ndarray) -> dict:
        """Split a flat action vector into per-side numpy arrays.

        Returned dict may contain the keys ``right`` / ``left``, each
        mapping to the per-arm slice of length :attr:`per_arm_dim`.
        Subclasses can override if they need richer keying (e.g. EE
        breaks per-arm 8-D into ``xyz``, ``quat``, ``gripper``).
        """
        a = np.asarray(action, dtype=np.float32).reshape(-1)
        out: dict = {}
        idx = 0
        if self._use_right_arm:
            out["right"] = a[idx : idx + self.per_arm_dim]
            idx += self.per_arm_dim
        if self._use_left_arm:
            out["left"] = a[idx : idx + self.per_arm_dim]
            idx += self.per_arm_dim
        return out

    def dispatch(
        self,
        safe_action: np.ndarray,
        state: GalaxeaR1ProRobotState,
    ) -> DispatchResult:
        raise NotImplementedError

    def reset_to_safe_pose(self, state: GalaxeaR1ProRobotState) -> None:
        raise NotImplementedError

    def get_required_topics(self) -> list[str]:
        raise NotImplementedError

    def verify_topology(self, controller=None) -> None:
        """Raise :class:`RuntimeError` when any required topic has no
        subscriber.  Called once at env init by
        :class:`GalaxeaR1ProEnv._setup_hardware`.

        ``controller`` defaults to the dispatcher's own controller.
        """
        ctrl = controller if controller is not None else self._controller
        if ctrl is None:
            return
        get_count = getattr(ctrl, "get_subscription_count", None)
        if get_count is None:
            _logger.info(
                "Controller has no get_subscription_count(); skipping "
                "verify_topology.  Old controllers without this method "
                "predate r1pro6op47.md §3.1; please upgrade.",
            )
            return
        bad = []
        for topic in self.get_required_topics():
            try:
                n = int(get_count(topic))
            except Exception as e:  # noqa: BLE001 - defensive RPC call
                _logger.warning(f"verify_topology: count {topic} -> {e}")
                continue
            if n <= 0:
                bad.append(topic)
        if bad:
            raise RuntimeError(
                f"[{self.mode}] required topics with no subscriber: "
                + ", ".join(bad)
                + ". Did you forget to launch the corresponding mobiman "
                "node? See design doc §1.2 #9."
            )


# ───────────────────── JointStateDispatcher ─────────────────────


class JointStateDispatcher(ActionDispatcher):
    """Joint-space dispatcher: 7 absolute joint targets per arm + gripper.

    Per arm action layout:

        ``[q0, q1, q2, q3, q4, q5, q6, gripper]``  (8-D with gripper)
        ``[q0, q1, q2, q3, q4, q5, q6]``           (7-D no gripper)

    Each ``qi`` in ``[-1, 1]`` is linearly mapped onto the per-joint
    range ``[q_min[i], q_max[i]]``.  Gripper goes through
    :class:`GripperMixer` for the configurable ``[gmin_pct, gmax_pct]``.

    ROS 2 publishes:

    * ``/motion_target/target_joint_state_arm_{side}``
      (``sensor_msgs/JointState``, ``position`` 7-D, ``velocity`` is
      the per-axis maximum allowed by mobiman joint_tracker)
    * ``/motion_target/target_position_gripper_{side}`` (when gripper)

    Args:
        controller: A :class:`GalaxeaR1ProController` handle.
        use_right_arm / use_left_arm: Stage flags.
        no_gripper: When True drop the gripper dim.
        gripper_mixer: Required even when ``no_gripper=True`` (we keep a
            single mixer for symmetry; it's just unused).
        q_min_right / q_max_right / q_min_left / q_max_left: Per-arm
            7-D joint limits in rad.  **Must be passed explicitly** --
            we deliberately do not default to URDF parsing here so that
            unit tests do not need a real Galaxea SDK install.
        q_vel_max: 7-D per-joint velocity envelope (rad/s) written to
            ``JointState.velocity`` for mobiman.  Defaults to the
            Galaxea-recommended ``[3, 3, 3, 3, 5, 5, 5]``.
        home_q_right / home_q_left: 7-D safe initial joint angles
            published by :meth:`reset_to_safe_pose`.  Defaults to zeros
            (matches the SDK ``starting_config`` of zeros).
    """

    mode = "joint"

    DEFAULT_QVEL_MAX = (3.0, 3.0, 3.0, 3.0, 5.0, 5.0, 5.0)

    def __init__(
        self,
        *,
        controller,
        use_right_arm: bool,
        use_left_arm: bool,
        no_gripper: bool,
        gripper_mixer: GripperMixer,
        q_min_right: np.ndarray,
        q_max_right: np.ndarray,
        q_min_left: Optional[np.ndarray] = None,
        q_max_left: Optional[np.ndarray] = None,
        q_vel_max: Optional[np.ndarray] = None,
        home_q_right: Optional[np.ndarray] = None,
        home_q_left: Optional[np.ndarray] = None,
    ):
        super().__init__(
            use_right_arm=use_right_arm,
            use_left_arm=use_left_arm,
            no_gripper=no_gripper,
            gripper_mixer=gripper_mixer,
            controller=controller,
        )
        self._q_min_right = self._as7(q_min_right, "q_min_right")
        self._q_max_right = self._as7(q_max_right, "q_max_right")
        self._validate_bounds(
            self._q_min_right,
            self._q_max_right,
            "right arm",
        )
        if use_left_arm:
            if q_min_left is None or q_max_left is None:
                raise ValueError(
                    "use_left_arm=True but q_min_left / q_max_left missing"
                )
            self._q_min_left = self._as7(q_min_left, "q_min_left")
            self._q_max_left = self._as7(q_max_left, "q_max_left")
            self._validate_bounds(
                self._q_min_left,
                self._q_max_left,
                "left arm",
            )
        else:
            self._q_min_left = np.zeros(7, dtype=np.float32)
            self._q_max_left = np.zeros(7, dtype=np.float32)

        if q_vel_max is None:
            q_vel_max = np.asarray(self.DEFAULT_QVEL_MAX, dtype=np.float32)
        self._q_vel_max = self._as7(q_vel_max, "q_vel_max")

        self._home_q_right = (
            self._as7(home_q_right, "home_q_right")
            if home_q_right is not None
            else np.zeros(7, dtype=np.float32)
        )
        self._home_q_left = (
            self._as7(home_q_left, "home_q_left")
            if home_q_left is not None
            else np.zeros(7, dtype=np.float32)
        )

    # ── helpers ────────────────────────────────────────────────

    @staticmethod
    def _as7(arr, name: str) -> np.ndarray:
        v = np.asarray(arr, dtype=np.float32).reshape(-1)
        if v.size != 7:
            raise ValueError(f"{name} must have length 7, got {v.size}")
        return v

    @staticmethod
    def _validate_bounds(q_min: np.ndarray, q_max: np.ndarray, label: str):
        if not np.all(q_max > q_min):
            raise ValueError(
                f"{label}: q_max must be > q_min element-wise. "
                f"q_min={q_min.tolist()}, q_max={q_max.tolist()}"
            )

    @property
    def per_arm_dim(self) -> int:
        return 7 if self._no_gripper else 8

    def _q_bounds(self, side: str):
        if side == "right":
            return self._q_min_right, self._q_max_right
        return self._q_min_left, self._q_max_left

    def _unnormalize_arm(self, side: str, a7: np.ndarray) -> np.ndarray:
        a = np.clip(np.asarray(a7, dtype=np.float32).reshape(-1)[:7], -1.0, 1.0)
        print(f">>>_unnormalize_arm()/a: {a}")
        if a.size != 7:
            raise ValueError(f"_unnormalize_arm needs 7 dims, got {a.size}")
        q_min, q_max = self._q_bounds(side)
        print(f">>>_unnormalize_arm()/q_min: {q_min}")
        print(f">>>_unnormalize_arm()/q_max: {q_max}")
        x = (a + 1.0).astype(np.float32)
        y = 0.5 * (q_max - q_min).astype(np.float32)
        # r = q_min + x * y
        print(f">>> [a + 1.0]>>{x}, [0.5 * (q_max - q_min)]>>{y}")
        print(f">>> (a + 1.0) * 0.5 * (q_max - q_min)>>{x * y}")
        print(f">>> q_min + (a + 1.0) * 0.5 * (q_max - q_min)>>{q_min.astype(np.float32) + x * y}")
        return q_min.astype(np.float32) + x * y
        # return (q_min + (a + 1.0) * 0.5 * (q_max - q_min)).astype(np.float32)
        # return   (q_min + (a + 1.0) * 0.5 * (q_max - q_min))

    def _unnormalize_gripper(self, a: float) -> float:
        return float(self._gripper_mixer.action11_to_pct(a))

    # ── core API ───────────────────────────────────────────────

    def dispatch(
        self,
        safe_action: np.ndarray,
        state: GalaxeaR1ProRobotState,
    ) -> DispatchResult:
        d = self.split(safe_action)
        commands: dict = {}
        refs: list = []

        for side in ("right", "left"):
            if side not in d:
                continue
            slc = d[side]
            q7 = slc[:7]
            q_target = self._unnormalize_arm(side, q7)
            cmd: dict = {"q_target": q_target.tolist()}
            ref = self._safe_send_arm_joints(side, q_target)
            if ref is not None:
                refs.append(ref)
            if not self._no_gripper and slc.size >= 8:
                pct = self._unnormalize_gripper(float(slc[7]))
                cmd["gripper_pct"] = pct
                ref_g = self._safe_send_gripper(side, pct)
                if ref_g is not None:
                    refs.append(ref_g)
            commands[side] = cmd
        return DispatchResult(self.mode, commands, refs)

    def reset_to_safe_pose(self, state: GalaxeaR1ProRobotState) -> None:
        if self._controller is None:
            return
        if self._use_right_arm:
            self._safe_send_arm_joints(
                "right",
                self._home_q_right,
                blocking=True,
            )
            if not self._no_gripper:
                self._safe_send_gripper(
                    "right",
                    self._gripper_mixer.gmin_pct,
                    blocking=True,
                )
        if self._use_left_arm:
            self._safe_send_arm_joints(
                "left",
                self._home_q_left,
                blocking=True,
            )
            if not self._no_gripper:
                self._safe_send_gripper(
                    "left",
                    self._gripper_mixer.gmin_pct,
                    blocking=True,
                )

    def get_required_topics(self) -> list[str]:
        topics: list[str] = []
        if self._use_right_arm:
            topics.append("/motion_target/target_joint_state_arm_right")
            if not self._no_gripper:
                topics.append("/motion_target/target_position_gripper_right")
        if self._use_left_arm:
            topics.append("/motion_target/target_joint_state_arm_left")
            if not self._no_gripper:
                topics.append("/motion_target/target_position_gripper_left")
        return topics

    # ── private send helpers (handle Ray refs uniformly) ─────────

    def _safe_send_arm_joints(
        self,
        side: str,
        q_target: np.ndarray,
        blocking: bool = False,
    ):
        if self._controller is None:
            return None
        try:
            ref = self._controller.send_arm_joints(
                side,
                q_target.tolist(),
                self._q_vel_max.tolist(),
            )
        except TypeError:
            # Older controllers don't accept qvel_max kwarg.
            ref = self._controller.send_arm_joints(side, q_target.tolist())
        if blocking and ref is not None and hasattr(ref, "wait"):
            try:
                ref.wait()
            except Exception:  # noqa: BLE001
                pass
        return ref

    def _safe_send_gripper(
        self,
        side: str,
        position_pct: float,
        blocking: bool = False,
    ):
        if self._controller is None or self._no_gripper:
            return None
        ref = self._controller.send_gripper(side, float(position_pct))
        if blocking and ref is not None and hasattr(ref, "wait"):
            try:
                ref.wait()
            except Exception:  # noqa: BLE001
                pass
        return ref


# ─────────────────────── JointDeltaDispatcher ───────────────────


class JointDeltaDispatcher(JointStateDispatcher):
    """Joint-space dispatcher with **delta** action semantics.

    Per design doc r1pro6op47.md §3.6: this is a sub-mode of joint
    mode where the policy outputs **per-step joint increments** in
    ``[-1, 1]^7`` instead of absolute targets.  The gripper dim
    remains an **absolute** pct (asymmetric, as the design calls
    out).  The reverse mapping is::

        delta_rad = clip(action[:7], -1, 1) * joint_delta_scale
        q_target = clip(q_current + delta_rad, q_min, q_max)
        gripper_pct = GripperMixer.action11_to_pct(action[7])  # unchanged

    All other behaviour -- ``get_required_topics`` (still publishes to
    ``/motion_target/target_joint_state_arm_*``), ``reset_to_safe_pose``
    (still issues absolute ``home_q``), ``verify_topology``,
    ``_safe_send_arm_joints``, ``_safe_send_gripper``, ``split`` -- is
    inherited verbatim from :class:`JointStateDispatcher`.

    The only state-dependent piece is :meth:`dispatch` which reads
    the *current* feedback qpos every call (NEVER cached) so cumulative
    drift cannot build up.

    Args:
        controller / use_right_arm / use_left_arm / no_gripper /
        gripper_mixer / q_min_right / q_max_right / q_min_left /
        q_max_left / q_vel_max / home_q_right / home_q_left:
            Same as :class:`JointStateDispatcher`.
        joint_delta_scale_right: 7-D rad/step per-joint cap on the
            right arm.  Defaults to
            :attr:`DEFAULT_JOINT_DELTA_SCALE` ``= [0.10, 0.10, 0.10,
            0.10, 0.20, 0.20, 0.20]`` rad/step.  Choose roughly
            ``arm_qvel_max[i] * dt_step`` for a 10 Hz step rate.
        joint_delta_scale_left: 7-D for the left arm.  Defaults to
            the same vector as ``joint_delta_scale_right``.
    """

    mode = "joint_delta"

    DEFAULT_JOINT_DELTA_SCALE = (0.10, 0.10, 0.10, 0.10, 0.20, 0.20, 0.20)

    def __init__(
        self,
        *,
        controller,
        use_right_arm: bool,
        use_left_arm: bool,
        no_gripper: bool,
        gripper_mixer: GripperMixer,
        q_min_right: np.ndarray,
        q_max_right: np.ndarray,
        q_min_left: Optional[np.ndarray] = None,
        q_max_left: Optional[np.ndarray] = None,
        q_vel_max: Optional[np.ndarray] = None,
        home_q_right: Optional[np.ndarray] = None,
        home_q_left: Optional[np.ndarray] = None,
        joint_delta_scale_right: Optional[np.ndarray] = None,
        joint_delta_scale_left: Optional[np.ndarray] = None,
    ):
        super().__init__(
            controller=controller,
            use_right_arm=use_right_arm,
            use_left_arm=use_left_arm,
            no_gripper=no_gripper,
            gripper_mixer=gripper_mixer,
            q_min_right=q_min_right,
            q_max_right=q_max_right,
            q_min_left=q_min_left,
            q_max_left=q_max_left,
            q_vel_max=q_vel_max,
            home_q_right=home_q_right,
            home_q_left=home_q_left,
        )
        # Right arm delta_scale: provided or default.
        scale_r = (
            joint_delta_scale_right
            if joint_delta_scale_right is not None
            else self.DEFAULT_JOINT_DELTA_SCALE
        )
        self._joint_delta_scale_right = self._as7(
            scale_r,
            "joint_delta_scale_right",
        )
        if not np.all(self._joint_delta_scale_right > 0):
            raise ValueError(
                "joint_delta_scale_right must be element-wise > 0 (rad/step). "
                f"Got {self._joint_delta_scale_right.tolist()}."
            )
        # Left arm delta_scale: provided, or copy right.
        scale_l = (
            joint_delta_scale_left
            if joint_delta_scale_left is not None
            else self._joint_delta_scale_right
        )
        self._joint_delta_scale_left = self._as7(
            scale_l,
            "joint_delta_scale_left",
        )
        if not np.all(self._joint_delta_scale_left > 0):
            raise ValueError(
                "joint_delta_scale_left must be element-wise > 0 (rad/step). "
                f"Got {self._joint_delta_scale_left.tolist()}."
            )

    # ── helpers (in addition to those inherited) ───────────────

    def _delta_scale(self, side: str) -> np.ndarray:
        if side == "right":
            return self._joint_delta_scale_right
        return self._joint_delta_scale_left

    def _compute_q_target(
        self,
        side: str,
        q_current: np.ndarray,
        a7: np.ndarray,
    ) -> np.ndarray:
        """Per design doc §3.6.5: ``q = clip(q_current + a*scale, qmin, qmax)``.

        ``q_current`` MUST be the latest feedback qpos (read fresh every
        dispatch), not a cached previous-step target -- otherwise
        mobiman tracking error compounds into drift.
        """
        a = np.clip(
            np.asarray(a7, dtype=np.float32).reshape(-1)[:7],
            -1.0,
            1.0,
        )
        if a.size != 7:
            raise ValueError(f"_compute_q_target needs 7 dims, got {a.size}")
        scale = self._delta_scale(side)
        cur = np.asarray(q_current, dtype=np.float32).reshape(-1)[:7]
        if cur.size != 7:
            raise ValueError(
                f"q_current must be 7-D, got {cur.size}; "
                "feedback callback wired to the controller?"
            )
        delta_rad = a * scale
        q_target = cur + delta_rad
        q_min, q_max = self._q_bounds(side)
        return np.clip(q_target, q_min, q_max).astype(np.float32)

    # ── core API: only dispatch differs from JointStateDispatcher ──

    def dispatch(
        self,
        safe_action: np.ndarray,
        state: GalaxeaR1ProRobotState,
    ) -> DispatchResult:
        d = self.split(safe_action)
        commands: dict = {}
        refs: list = []
        for side in ("right", "left"):
            if side not in d:
                continue
            slc = d[side]
            cur_q = np.asarray(
                state.get_arm_qpos(side),
                dtype=np.float32,
            ).reshape(-1)[:7]
            q_target = self._compute_q_target(side, cur_q, slc[:7])
            cmd: dict = {
                "q_current": cur_q.tolist(),
                "delta_rad": (q_target - cur_q).tolist(),
                "q_target": q_target.tolist(),
            }
            ref = self._safe_send_arm_joints(side, q_target)
            if ref is not None:
                refs.append(ref)
            if not self._no_gripper and slc.size >= 8:
                # Gripper remains absolute (asymmetric per §3.6.1).
                pct = self._unnormalize_gripper(float(slc[7]))
                cmd["gripper_pct"] = pct
                ref_g = self._safe_send_gripper(side, pct)
                if ref_g is not None:
                    refs.append(ref_g)
            commands[side] = cmd
        return DispatchResult(self.mode, commands, refs)


# ─────────────────────── EePoseDispatcher ───────────────────────


class EePoseDispatcher(ActionDispatcher):
    """Cartesian-space dispatcher: ``[xyz + quat]`` per arm + gripper.

    Per arm action layout:

        ``[x, y, z, qx, qy, qz, qw, gripper]``  (8-D with gripper)
        ``[x, y, z, qx, qy, qz, qw]``            (7-D no gripper)

    Position xyz is linearly mapped from ``[-1, 1]`` onto an absolute
    workspace box ``[ee_min, ee_max]`` (in the controller's reference
    frame, typically ``torso_link4`` per Galaxea convention).

    Quaternion is treated as raw ``[qx, qy, qz, qw]``, L2-normalised
    to a unit quat, and canonicalised into ``W >= 0`` form to avoid
    the well-known ``q <-> -q`` representation ambiguity.

    ROS 2 publishes:

    * ``/motion_target/target_pose_arm_{side}``
      (``geometry_msgs/PoseStamped``)
    * ``/motion_target/target_position_gripper_{side}`` (when gripper)

    Args:
        controller: A :class:`GalaxeaR1ProController` handle.
        use_right_arm / use_left_arm / no_gripper: Stage flags.
        gripper_mixer: For [-1, 1] <-> physical pct mapping.
        ee_min_right / ee_max_right / ee_min_left / ee_max_left:
            3-D xyz workspace box in metres (per arm).  Required for
            any enabled arm.
        home_pose_right / home_pose_left: 7-D xyzquat (qx qy qz qw)
            published by :meth:`reset_to_safe_pose`.  Default uses the
            workspace centre with identity orientation.
    """

    mode = "ee"

    def __init__(
        self,
        *,
        controller,
        use_right_arm: bool,
        use_left_arm: bool,
        no_gripper: bool,
        gripper_mixer: GripperMixer,
        ee_min_right: np.ndarray,
        ee_max_right: np.ndarray,
        ee_min_left: Optional[np.ndarray] = None,
        ee_max_left: Optional[np.ndarray] = None,
        home_pose_right: Optional[np.ndarray] = None,
        home_pose_left: Optional[np.ndarray] = None,
    ):
        super().__init__(
            use_right_arm=use_right_arm,
            use_left_arm=use_left_arm,
            no_gripper=no_gripper,
            gripper_mixer=gripper_mixer,
            controller=controller,
        )
        self._ee_min_right = self._as3(ee_min_right, "ee_min_right")
        self._ee_max_right = self._as3(ee_max_right, "ee_max_right")
        self._validate_box(
            self._ee_min_right,
            self._ee_max_right,
            "right arm",
        )
        if use_left_arm:
            if ee_min_left is None or ee_max_left is None:
                raise ValueError(
                    "use_left_arm=True but ee_min_left / ee_max_left missing"
                )
            self._ee_min_left = self._as3(ee_min_left, "ee_min_left")
            self._ee_max_left = self._as3(ee_max_left, "ee_max_left")
            self._validate_box(
                self._ee_min_left,
                self._ee_max_left,
                "left arm",
            )
        else:
            self._ee_min_left = np.zeros(3, dtype=np.float32)
            self._ee_max_left = np.zeros(3, dtype=np.float32)

        self._home_pose_right = self._home_default(
            home_pose_right,
            self._ee_min_right,
            self._ee_max_right,
        )
        self._home_pose_left = self._home_default(
            home_pose_left,
            self._ee_min_left,
            self._ee_max_left,
        )

    # ── helpers ────────────────────────────────────────────────

    @staticmethod
    def _as3(arr, name: str) -> np.ndarray:
        v = np.asarray(arr, dtype=np.float32).reshape(-1)
        if v.size != 3:
            raise ValueError(f"{name} must have length 3, got {v.size}")
        return v

    @staticmethod
    def _validate_box(lo: np.ndarray, hi: np.ndarray, label: str):
        if not np.all(hi > lo):
            raise ValueError(
                f"{label}: ee_max must be > ee_min element-wise. "
                f"ee_min={lo.tolist()}, ee_max={hi.tolist()}"
            )

    @staticmethod
    def _home_default(
        explicit: Optional[np.ndarray],
        lo: np.ndarray,
        hi: np.ndarray,
    ) -> np.ndarray:
        if explicit is not None:
            v = np.asarray(explicit, dtype=np.float32).reshape(-1)[:7]
            if v.size != 7:
                raise ValueError(f"home_pose must be 7-D xyzquat, got {v.size}")
            return v.astype(np.float32)
        # Default: workspace centre with identity orientation.
        centre = ((lo + hi) * 0.5).astype(np.float32)
        return np.concatenate(
            [centre, np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)],
        )

    @property
    def per_arm_dim(self) -> int:
        return 7 if self._no_gripper else 8

    def _ee_bounds(self, side: str):
        if side == "right":
            return self._ee_min_right, self._ee_max_right
        return self._ee_min_left, self._ee_max_left

    def _unnormalize_xyz(self, side: str, a3: np.ndarray) -> np.ndarray:
        a = np.clip(np.asarray(a3, dtype=np.float32).reshape(-1)[:3], -1.0, 1.0)
        lo, hi = self._ee_bounds(side)
        return (lo + (a + 1.0) * 0.5 * (hi - lo)).astype(np.float32)

    @staticmethod
    def _normalize_quat(a4: np.ndarray) -> np.ndarray:
        """L2-normalise to unit quat; canonicalise W >= 0; degenerate
        zero-vector input falls back to identity ``[0, 0, 0, 1]``."""
        q = np.asarray(a4, dtype=np.float32).reshape(-1)[:4]
        if q.size != 4:
            raise ValueError(f"quat must be 4-D, got {q.size}")
        n = float(np.linalg.norm(q))
        if n < 1e-8:
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        q = (q / n).astype(np.float32)
        if q[3] < 0:
            q = -q
        return q.astype(np.float32)

    def _unnormalize_gripper(self, a: float) -> float:
        return float(self._gripper_mixer.action11_to_pct(a))

    # ── core API ───────────────────────────────────────────────

    def dispatch(
        self,
        safe_action: np.ndarray,
        state: GalaxeaR1ProRobotState,
    ) -> DispatchResult:
        d = self.split(safe_action)
        commands: dict = {}
        refs: list = []
        for side in ("right", "left"):
            if side not in d:
                continue
            slc = d[side]
            xyz = self._unnormalize_xyz(side, slc[:3])
            quat = self._normalize_quat(slc[3:7])
            pose = np.concatenate([xyz, quat]).astype(np.float32)
            cmd: dict = {"pose": pose.tolist()}
            ref = self._safe_send_arm_pose(side, pose)
            if ref is not None:
                refs.append(ref)
            if not self._no_gripper and slc.size >= 8:
                pct = self._unnormalize_gripper(float(slc[7]))
                cmd["gripper_pct"] = pct
                ref_g = self._safe_send_gripper(side, pct)
                if ref_g is not None:
                    refs.append(ref_g)
            commands[side] = cmd
        return DispatchResult(self.mode, commands, refs)

    def reset_to_safe_pose(self, state: GalaxeaR1ProRobotState) -> None:
        if self._controller is None:
            return
        if self._use_right_arm:
            self._safe_send_arm_pose(
                "right",
                self._home_pose_right,
                blocking=True,
            )
            if not self._no_gripper:
                self._safe_send_gripper(
                    "right",
                    self._gripper_mixer.gmin_pct,
                    blocking=True,
                )
        if self._use_left_arm:
            self._safe_send_arm_pose(
                "left",
                self._home_pose_left,
                blocking=True,
            )
            if not self._no_gripper:
                self._safe_send_gripper(
                    "left",
                    self._gripper_mixer.gmin_pct,
                    blocking=True,
                )

    def get_required_topics(self) -> list[str]:
        topics: list[str] = []
        if self._use_right_arm:
            topics.append("/motion_target/target_pose_arm_right")
            if not self._no_gripper:
                topics.append("/motion_target/target_position_gripper_right")
        if self._use_left_arm:
            topics.append("/motion_target/target_pose_arm_left")
            if not self._no_gripper:
                topics.append("/motion_target/target_position_gripper_left")
        return topics

    # ── private send helpers ────────────────────────────────────

    def _safe_send_arm_pose(
        self,
        side: str,
        pose7: np.ndarray,
        blocking: bool = False,
    ):
        if self._controller is None:
            return None
        ref = self._controller.send_arm_pose(side, pose7.astype(np.float32))
        if blocking and ref is not None and hasattr(ref, "wait"):
            try:
                ref.wait()
            except Exception:  # noqa: BLE001
                pass
        return ref

    def _safe_send_gripper(
        self,
        side: str,
        position_pct: float,
        blocking: bool = False,
    ):
        if self._controller is None or self._no_gripper:
            return None
        ref = self._controller.send_gripper(side, float(position_pct))
        if blocking and ref is not None and hasattr(ref, "wait"):
            try:
                ref.wait()
            except Exception:  # noqa: BLE001
                pass
        return ref


# ───────────────────────── Factory ──────────────────────────────


def build_action_dispatcher(
    *,
    use_joint_mode: bool,
    use_right_arm: bool,
    use_left_arm: bool,
    no_gripper: bool,
    gripper_mixer: GripperMixer,
    controller=None,
    # joint mode
    q_min_right: Optional[np.ndarray] = None,
    q_max_right: Optional[np.ndarray] = None,
    q_min_left: Optional[np.ndarray] = None,
    q_max_left: Optional[np.ndarray] = None,
    q_vel_max: Optional[np.ndarray] = None,
    home_q_right: Optional[np.ndarray] = None,
    home_q_left: Optional[np.ndarray] = None,
    # joint sub-mode (per r1pro6op47.md §3.6)
    joint_delta_mode: bool = False,
    joint_delta_scale_right: Optional[np.ndarray] = None,
    joint_delta_scale_left: Optional[np.ndarray] = None,
    # ee mode
    ee_min_right: Optional[np.ndarray] = None,
    ee_max_right: Optional[np.ndarray] = None,
    ee_min_left: Optional[np.ndarray] = None,
    ee_max_left: Optional[np.ndarray] = None,
    home_pose_right: Optional[np.ndarray] = None,
    home_pose_left: Optional[np.ndarray] = None,
) -> ActionDispatcher:
    """Construct the dispatcher for the configured action mode.

    Single source of truth for ``(use_joint_mode, joint_delta_mode)`` ->
    dispatcher type mapping.  Per design doc r1pro6op47.md §3.5.

    Three-way branch:

    * ``use_joint_mode=True, joint_delta_mode=False`` (default for joint)
      -> :class:`JointStateDispatcher` (absolute joint targets)
    * ``use_joint_mode=True, joint_delta_mode=True``
      -> :class:`JointDeltaDispatcher` (delta joint targets, abs gripper,
      asymmetric per §3.6)
    * ``use_joint_mode=False`` -> :class:`EePoseDispatcher`
      (``joint_delta_mode`` is ignored)
    """
    if use_joint_mode:
        if q_min_right is None or q_max_right is None:
            raise ValueError(
                "joint mode requires q_min_right / q_max_right (parsed from "
                "URDF or supplied explicitly via SafetyConfig.right_arm_q_*)."
            )
        joint_kw = {
            "controller": controller,
            "use_right_arm": use_right_arm,
            "use_left_arm": use_left_arm,
            "no_gripper": no_gripper,
            "gripper_mixer": gripper_mixer,
            "q_min_right": q_min_right,
            "q_max_right": q_max_right,
            "q_min_left": q_min_left,
            "q_max_left": q_max_left,
            "q_vel_max": q_vel_max,
            "home_q_right": home_q_right,
            "home_q_left": home_q_left,
        }
        if joint_delta_mode:
            return JointDeltaDispatcher(
                joint_delta_scale_right=joint_delta_scale_right,
                joint_delta_scale_left=joint_delta_scale_left,
                **joint_kw,
            )
        return JointStateDispatcher(**joint_kw)
    return EePoseDispatcher(
        controller=controller,
        use_right_arm=use_right_arm,
        use_left_arm=use_left_arm,
        no_gripper=no_gripper,
        gripper_mixer=gripper_mixer,
        ee_min_right=ee_min_right,
        ee_max_right=ee_max_right,
        ee_min_left=ee_min_left,
        ee_max_left=ee_max_left,
        home_pose_right=home_pose_right,
        home_pose_left=home_pose_left,
    )
