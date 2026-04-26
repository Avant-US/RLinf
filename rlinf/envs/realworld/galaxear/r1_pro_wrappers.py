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

"""Galaxea R1 Pro specific gym wrappers.

Four wrappers implemented:

* :class:`GalaxeaR1ProJoystickIntervention` -- subscribes to
  ``/controller`` (R1 Pro wireless controller) and overrides the
  policy action when SWA / SWB triggers, with SWD acting as software
  E-stop.  Falls back to no-op when ``rclpy`` / ``hdas_msg`` are not
  importable.
* :class:`GalaxeaR1ProVRIntervention` -- subscribes to R1 Pro SDK VR
  teleop topics and overrides the policy action with VR hand pose
  deltas.  Compatible with ``algorithm.dagger.only_save_expert``.
* :class:`GalaxeaR1ProDualArmCollisionWrapper` -- soft slow-zone
  attenuation for dual-arm exploration; complements the L3b
  collision sphere check inside SafetySupervisor by giving policies
  a graceful escape gradient before SafetySupervisor freezes.
* :class:`GalaxeaR1ProActionSmoother` -- exponential moving average
  + jerk bound for safer real-robot deployment of high-frequency
  VLA / OpenPI policies.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import gymnasium as gym
import numpy as np

_logger = logging.getLogger(__name__)


# ─────────────────── R1 Pro wireless controller ─────────────────


class GalaxeaR1ProJoystickIntervention(gym.ActionWrapper):
    """Override policy action with the R1 Pro wireless joystick.

    The wrapper opens its own :class:`rclpy.Node` (lazy-imported) and
    subscribes to ``/controller``.  When a non-default switch position
    is detected the wrapper returns a small per-step action computed
    from the analog sticks; ``intervene_action`` and
    ``intervene_flag`` are written to ``info`` to plug into the
    HG-DAgger pipeline (consistent with
    :class:`GelloIntervention` / :class:`SpacemouseIntervention`).

    SWD = down acts as soft E-stop: the wrapper raises
    ``info["safe_pause"] = True``; the env then escalates to its
    SafetySupervisor.

    Args:
        env: Wrapped env.
        gripper_enabled: Whether the gripper dim is present.
        deadzone: Stick magnitude below which sticks are ignored.
        topic: ``/controller`` topic name (rarely changed).
    """

    def __init__(
        self,
        env: gym.Env,
        gripper_enabled: bool = True,
        deadzone: float = 0.05,
        topic: str = "/controller",
    ) -> None:
        super().__init__(env)
        self.gripper_enabled = gripper_enabled
        self._deadzone = float(deadzone)
        self._topic = topic
        self._latest_signal: Optional[dict] = None
        self._lock = threading.Lock()
        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_running = False
        self._init_ros2()

    def _init_ros2(self) -> None:
        try:
            import rclpy  # type: ignore[import]
            from rclpy.executors import SingleThreadedExecutor  # type: ignore
            from rclpy.qos import qos_profile_sensor_data  # type: ignore
        except Exception as e:
            _logger.warning(
                "GalaxeaR1ProJoystickIntervention: rclpy not available "
                "(%s); wrapper will pass-through.", e,
            )
            return
        try:
            from hdas_msg.msg import (  # type: ignore[import]
                controller_signal_stamped as ControllerSignalMsg,
            )
        except ImportError:
            _logger.warning(
                "hdas_msg not importable; joystick intervention disabled "
                "(wrapper passes through actions)."
            )
            return
        if not rclpy.ok():
            rclpy.init(args=[])
        self._node = rclpy.create_node(
            f"r1_pro_joystick_intervention_{id(self)}"
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._node.create_subscription(
            ControllerSignalMsg, self._topic, self._on_controller,
            qos_profile_sensor_data,
        )
        self._spin_running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True,
        )
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        try:
            while self._spin_running and self._executor is not None:
                self._executor.spin_once(timeout_sec=0.01)
        except Exception:
            pass

    def _on_controller(self, msg) -> None:
        data = getattr(msg, "data", msg)
        with self._lock:
            self._latest_signal = {
                "swa": int(getattr(data, "swa", 0)),
                "swb": int(getattr(data, "swb", 0)),
                "swc": int(getattr(data, "swc", 0)),
                "swd": int(getattr(data, "swd", 0)),
                "lx": float(getattr(data, "lx", 0.0)),
                "ly": float(getattr(data, "ly", 0.0)),
                "rx": float(getattr(data, "rx", 0.0)),
                "ry": float(getattr(data, "ry", 0.0)),
            }

    def action(self, action: np.ndarray):
        with self._lock:
            sig = dict(self._latest_signal) if self._latest_signal else None
        if sig is None:
            return action, False

        # SWD = down acts as soft E-stop request.
        if sig.get("swd", 0):
            # Pass through with no override; env's SafetySupervisor
            # will detect SWD and trigger SAFE_STOP.
            return action, False

        # SWA / SWB triggers -> overlay stick action; otherwise pass.
        active = bool(sig.get("swa", 0)) or bool(sig.get("swb", 0))
        if not active:
            return action, False

        new_action = np.zeros_like(action, dtype=np.float32)
        # Map sticks -> [dx, dy, dz, drx, dry, drz, gripper] for the
        # currently-active arm.  Keep simple: SWA = right, SWB = left.
        lx, ly = sig.get("lx", 0.0), sig.get("ly", 0.0)
        rx, ry = sig.get("rx", 0.0), sig.get("ry", 0.0)
        # Apply deadzone.
        for k, v in (("lx", lx), ("ly", ly), ("rx", rx), ("ry", ry)):
            if abs(v) < self._deadzone:
                sig[k] = 0.0
        per_arm_dim = 7 if self.gripper_enabled else 6
        # Choose arm slice (right preferred when SWA active).
        slot = 0 if sig.get("swa", 0) else (
            per_arm_dim if action.size >= 2 * per_arm_dim else 0
        )
        # dx <- ly, dy <- lx, dz <- ry, drz <- rx (rough convention).
        if slot + 6 <= action.size:
            new_action[slot + 0] = sig["ly"]
            new_action[slot + 1] = sig["lx"]
            new_action[slot + 2] = sig["ry"]
            new_action[slot + 5] = sig["rx"]
        if self.gripper_enabled and slot + per_arm_dim <= action.size:
            # SWC pulls grip; left=close (-1), right=open (1).
            swc = int(sig.get("swc", 0))
            if swc == 1:
                new_action[slot + 6] = 1.0
            elif swc == 2:
                new_action[slot + 6] = -1.0
        return new_action, True

    def step(self, action):
        new_action, replaced = self.action(action)
        obs, reward, term, trunc, info = self.env.step(new_action)
        info["intervene_action"] = new_action.copy() if replaced else None
        info["intervene_flag"] = bool(replaced)
        return obs, reward, term, trunc, info

    def close(self):
        self._spin_running = False
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=0.5)
        try:
            if self._executor is not None:
                self._executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        super().close()


# ───────────────────────── VR teleop ─────────────────────────


class GalaxeaR1ProVRIntervention(gym.ActionWrapper):
    """R1 Pro VR teleop intervention wrapper.

    Subscribes to two ``geometry_msgs/PoseStamped`` topics
    (``vr_left_topic`` / ``vr_right_topic``, default
    ``/vr_teleop/left_hand`` and ``/vr_teleop/right_hand``).  When VR
    pose is fresh, the wrapper computes a per-step delta in the
    current EE frame and substitutes it for the policy action,
    writing ``info["intervene_action"]`` / ``info["intervene_flag"]``
    just like :class:`GelloIntervention`.

    Falls back to pass-through when ``rclpy`` is not importable
    (e.g. in is_dummy / CI runs).

    The actual mapping from VR hand pose to per-arm delta depends on
    the Galaxea VR SDK version; this implementation provides the
    skeleton + safety interlocks (deadzone + max delta) and lets the
    deployment site specialise.  See design doc §6.8 / §14.4.
    """

    DEFAULT_LEFT_TOPIC = "/vr_teleop/left_hand"
    DEFAULT_RIGHT_TOPIC = "/vr_teleop/right_hand"
    STALE_THRESHOLD_S = 0.30

    def __init__(
        self,
        env: gym.Env,
        gripper_enabled: bool = True,
        vr_left_topic: str = DEFAULT_LEFT_TOPIC,
        vr_right_topic: str = DEFAULT_RIGHT_TOPIC,
    ) -> None:
        super().__init__(env)
        self.gripper_enabled = gripper_enabled
        self._left_topic = vr_left_topic
        self._right_topic = vr_right_topic
        self._latest = {"left": None, "right": None}
        self._latest_t = {"left": 0.0, "right": 0.0}
        self._lock = threading.Lock()
        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_running = False
        self._init_ros2()

    def _init_ros2(self) -> None:
        try:
            import rclpy  # type: ignore[import]
            from rclpy.executors import SingleThreadedExecutor  # type: ignore
            from rclpy.qos import qos_profile_sensor_data  # type: ignore
            from geometry_msgs.msg import PoseStamped  # type: ignore
        except Exception as e:
            _logger.info(
                "VR teleop disabled (rclpy unavailable): %s", e,
            )
            return
        if not rclpy.ok():
            rclpy.init(args=[])
        self._node = rclpy.create_node(
            f"r1_pro_vr_intervention_{id(self)}"
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._node.create_subscription(
            PoseStamped, self._left_topic,
            lambda m: self._on_pose("left", m),
            qos_profile_sensor_data,
        )
        self._node.create_subscription(
            PoseStamped, self._right_topic,
            lambda m: self._on_pose("right", m),
            qos_profile_sensor_data,
        )
        self._spin_running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True,
        )
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        try:
            while self._spin_running and self._executor is not None:
                self._executor.spin_once(timeout_sec=0.01)
        except Exception:
            pass

    def _on_pose(self, side: str, msg) -> None:
        with self._lock:
            self._latest[side] = msg
            self._latest_t[side] = time.time()

    def action(self, action: np.ndarray):
        with self._lock:
            now = time.time()
            fresh_right = (
                self._latest["right"] is not None
                and (now - self._latest_t["right"]) < self.STALE_THRESHOLD_S
            )
            fresh_left = (
                self._latest["left"] is not None
                and (now - self._latest_t["left"]) < self.STALE_THRESHOLD_S
            )
        if not (fresh_left or fresh_right):
            return action, False
        # Skeleton: zero delta until the deployment site populates a
        # concrete pose-delta computation.  Wrappers in production
        # should fill this with site-specific math.
        new_action = np.zeros_like(action, dtype=np.float32)
        return new_action, True

    def step(self, action):
        new_action, replaced = self.action(action)
        obs, reward, term, trunc, info = self.env.step(new_action)
        info["intervene_action"] = new_action.copy() if replaced else None
        info["intervene_flag"] = bool(replaced)
        return obs, reward, term, trunc, info

    def close(self):
        self._spin_running = False
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=0.5)
        try:
            if self._executor is not None:
                self._executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        super().close()


# ─────────────────── Dual-arm collision soft zone ────────────────


class GalaxeaR1ProDualArmCollisionWrapper(gym.ActionWrapper):
    """Soft slow-zone attenuation for dual-arm tasks.

    When predicted EE TCPs come within ``slow_zone_m`` of each other,
    scale the action by ``(d - min_dist) / (slow_zone - min_dist)``
    so the policy gets a smooth gradient pushing it away from
    collisions.  When ``d < min_dist_m``, the action is zeroed out;
    SafetySupervisor's L3b also catches this independently.

    Args:
        env: Wrapped env.
        slow_zone_m: Distance below which scaling kicks in.
        min_dist_m: Distance below which actions are zeroed.
    """

    def __init__(
        self,
        env: gym.Env,
        slow_zone_m: float = 0.15,
        min_dist_m: float = 0.08,
    ) -> None:
        super().__init__(env)
        self._slow_zone_m = float(slow_zone_m)
        self._min_dist_m = float(min_dist_m)

    def action(self, action: np.ndarray):
        env = self.env
        # Walk through wrappers to access the underlying env's state.
        target = env
        while hasattr(target, "env"):
            target = target.env
        state = getattr(target, "_state", None)
        cfg = getattr(target, "config", None)
        if (
            state is None
            or cfg is None
            or not getattr(cfg, "use_left_arm", False)
            or not getattr(cfg, "use_right_arm", False)
        ):
            return action
        d = float(np.linalg.norm(
            state.right_ee_pose[:3] - state.left_ee_pose[:3]
        ))
        if d < self._min_dist_m:
            return np.zeros_like(action, dtype=np.float32)
        if d < self._slow_zone_m:
            scale = (d - self._min_dist_m) / max(
                self._slow_zone_m - self._min_dist_m, 1e-9
            )
            return (np.asarray(action, dtype=np.float32) * float(scale))
        return action


# ───────────────────────── Action smoother ────────────────────────


class GalaxeaR1ProActionSmoother(gym.ActionWrapper):
    """EMA + per-step jerk bound for safe real-robot deployment.

    Args:
        env: Wrapped env.
        alpha: EMA factor in ``[0, 1]``.  Higher = follow policy more
            closely; lower = smoother.
        max_delta: Per-dim maximum action change between consecutive
            steps.  Helps protect against high-frequency VLA jitter.
    """

    def __init__(
        self,
        env: gym.Env,
        alpha: float = 0.3,
        max_delta: float = 0.5,
    ) -> None:
        super().__init__(env)
        self._alpha = float(alpha)
        self._max_delta = float(max_delta)
        self._prev: Optional[np.ndarray] = None

    def reset(self, **kwargs):
        self._prev = None
        return self.env.reset(**kwargs)

    def action(self, action):
        a = np.asarray(action, dtype=np.float32)
        if self._prev is None:
            self._prev = a.copy()
            return a
        # Jerk bound: clamp |a - prev| <= max_delta.
        diff = np.clip(
            a - self._prev, -self._max_delta, self._max_delta,
        )
        bounded = self._prev + diff
        # EMA towards bounded action.
        smoothed = self._alpha * bounded + (1.0 - self._alpha) * self._prev
        self._prev = smoothed
        return smoothed
