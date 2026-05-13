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
"""Shared runtime for R1 Pro M1 Orin local single-process RLinf SAC.

Reuses RLinf components without modifying them:

  * ``rlinf.envs.realworld.galaxear.tasks.r1_pro_single_arm_reach_joint``
  * ``rlinf.envs.realworld.galaxear.r1_pro_safety``
  * ``rlinf.envs.realworld.galaxear.r1_pro_action_dispatcher``
  * ``rlinf.envs.realworld.galaxear.r1_pro_robot_state``
  * ``rlinf.envs.realworld.galaxear.r1_pro_controller``
  * ``rlinf.models.embodiment.mlp_policy.mlp_policy``

Replaces only the distributed scheduling layer (Ray WorkerGroup, FSDP,
torch.distributed) with a local rclpy controller, since NVIDIA Jetson
PyTorch ships with ``torch.distributed`` disabled.

See ``bt/docs/rwRL/r1pro6op47_reach_joint3_2.md`` for the design.
"""
from __future__ import annotations

import os
import random
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch


# ─── Jetson torch.distributed import shim ──────────────────────────
def install_jetson_torch_import_shim() -> None:
    """Patch import-time symbols absent in distributed-disabled Jetson torch.

    Several RLinf modules type-annotate their APIs with ``dist.Work`` and
    ``torch.Event``. On NVIDIA Jetson PyTorch (e.g. ``2.4.0a0+...nv24.05``)
    these symbols do not exist because the wheel is built without
    ``USE_DISTRIBUTED``. The shim only adds *unused* stubs so import-time
    annotation evaluation succeeds; we never actually call any collective.
    """
    import torch.distributed as dist

    if not hasattr(dist, "Work"):

        class _StubWork:
            def wait(self, *args, **kwargs):  # noqa: ANN001 — RLinf signature
                raise RuntimeError(
                    "torch.distributed is disabled on this Jetson wheel."
                )

            def is_completed(self) -> bool:
                return False

        dist.Work = _StubWork  # type: ignore[attr-defined]

    if not hasattr(torch, "Event"):
        torch.Event = (  # type: ignore[attr-defined]
            torch.cuda.Event if torch.cuda.is_available() else object
        )


install_jetson_torch_import_shim()

# ----- Imports that touch ``dist.Work`` annotations go below the shim ------
from rlinf.envs.realworld.galaxear.r1_pro_controller import (  # noqa: E402
    GalaxeaR1ProController,
)
from rlinf.envs.realworld.galaxear.r1_pro_robot_state import (  # noqa: E402
    GalaxeaR1ProRobotState,
)
from rlinf.envs.realworld.galaxear.r1_pro_safety import (  # noqa: E402
    SafetyConfig,
    build_safety_config,
)
from rlinf.envs.realworld.galaxear.tasks.r1_pro_single_arm_reach_joint import (  # noqa: E402
    GalaxeaR1ProSingleArmReachJointEnv,
)
from rlinf.models.embodiment.base_policy import ForwardType  # noqa: E402
from rlinf.models.embodiment.mlp_policy.mlp_policy import MLPPolicy  # noqa: E402

__all__ = [
    "ForwardType",
    "MLPPolicy",
    "ReplayBuffer",
    "SafetyConfig",
    "SafetyConfigActionProjector",
    "build_env",
    "build_projector",
    "choose_device",
    "flatten_obs",
    "get",
    "install_jetson_torch_import_shim",
    "load_yaml",
    "preflight",
    "resolve_ros_env",
    "safe_move_to_q",
    "safe_step",
    "set_seed",
]


# ─── Config helpers ────────────────────────────────────────────────
def load_yaml(path: str | Path) -> dict[str, Any]:
    """Load a YAML file as a plain dict."""
    import yaml

    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def get(d: dict[str, Any], dotted: str, default: Any = None) -> Any:
    """Read ``a.b.c`` from a nested dict; return ``default`` on miss."""
    cur: Any = d
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def set_seed(seed: int) -> None:
    """Seed all stochastic sources used by the runner."""
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def resolve_ros_env(cfg: dict[str, Any]) -> dict[str, Any]:
    """Reconcile ROS env vars with cfg, with shell env taking priority.

    Why this exists
    ---------------
    Two layers control the DDS domain RLinf joins:

      L1.  ``os.environ["ROS_DOMAIN_ID"]`` — read by ``rclpy.init()``.
      L2.  ``cfg.env.override_cfg.ros_domain_id`` — RLinf env passes this
           into ``GalaxeaR1ProController.launch_controller(...)``, and
           our :class:`LocalR1ProController` re-applies it to ``os.environ``
           **before** ``rclpy.init()`` runs.

    If L1 and L2 disagree, L2 silently wins and the script joins a
    different domain than the operator's shell — exactly the failure
    mode where ``ros2 topic list`` finds the topic but the Python
    process cannot.  This helper enforces a single rule:

        shell env  >  cfg fallback

    and writes the chosen value back into both ``os.environ`` and
    ``cfg.env.override_cfg`` so L1 and L2 are guaranteed to agree.

    Returns
    -------
    The effective values, also printed for the operator log.
    """
    overrides = cfg.setdefault("env", {}).setdefault("override_cfg", {})

    cfg_domain = get(cfg, "runtime.ros_domain_id")
    if cfg_domain is None:
        cfg_domain = overrides.get("ros_domain_id")
    env_domain_raw = os.environ.get("ROS_DOMAIN_ID", None)
    eff_domain = (
        int(env_domain_raw) if env_domain_raw not in (None, "") else int(cfg_domain)
    )
    os.environ["ROS_DOMAIN_ID"] = str(eff_domain)
    overrides["ros_domain_id"] = eff_domain

    cfg_rmw = get(cfg, "runtime.rmw_implementation")
    env_rmw = os.environ.get("RMW_IMPLEMENTATION", None)
    eff_rmw = env_rmw if env_rmw else (str(cfg_rmw) if cfg_rmw else None)
    if eff_rmw:
        os.environ["RMW_IMPLEMENTATION"] = eff_rmw

    cfg_localhost = overrides.get("ros_localhost_only")
    env_localhost_raw = os.environ.get("ROS_LOCALHOST_ONLY", None)
    if env_localhost_raw is not None:
        eff_localhost = env_localhost_raw.strip() in ("1", "true", "True")
    else:
        eff_localhost = bool(cfg_localhost)
    os.environ["ROS_LOCALHOST_ONLY"] = "1" if eff_localhost else "0"
    overrides["ros_localhost_only"] = eff_localhost

    print(
        "[ROS] effective env: "
        f"ROS_DOMAIN_ID={eff_domain} "
        f"RMW_IMPLEMENTATION={eff_rmw or '(unset)'} "
        f"ROS_LOCALHOST_ONLY={'1' if eff_localhost else '0'} "
        f"(shell-domain={env_domain_raw!r}, cfg-domain={cfg_domain!r})"
    )
    return {
        "ros_domain_id": eff_domain,
        "rmw_implementation": eff_rmw,
        "ros_localhost_only": eff_localhost,
    }


def choose_device(cfg: dict[str, Any]) -> torch.device:
    """Resolve ``cfg.device`` (auto / cuda / cpu) into a torch.device."""
    want = str(get(cfg, "device", "auto"))
    if want == "cuda":
        return torch.device("cuda")
    if want == "cpu":
        return torch.device("cpu")
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


# ─── Local Controller (bypass Ray WorkerGroup) ─────────────────────
class LocalRef:
    """Mimics RLinf RPC futures: ``handle.method().wait()[0]`` returns value."""

    def __init__(self, value: Any = None) -> None:
        self._value = value

    def wait(self) -> list[Any]:
        return [self._value]


class LocalR1ProController:
    """Local rclpy implementation of the GalaxeaR1ProController API subset.

    Implements the methods that RLinf ``GalaxeaR1ProEnv`` calls on its
    ``self._controller`` handle (``get_state``, ``is_robot_up``,
    ``send_arm_joints``, ``apply_brake``, ``get_subscription_count``)
    directly via rclpy, so we can run end-to-end without a Ray cluster.
    """

    DEFAULT_JOINT_NAMES = {
        "right": [f"arm_right_j{i + 1}" for i in range(7)],
        "left": [f"arm_left_j{i + 1}" for i in range(7)],
    }

    def __init__(
        self,
        *,
        ros_domain_id: int,
        ros_localhost_only: bool,
        use_right_arm: bool,
        use_left_arm: bool,
        **_: Any,
    ) -> None:
        os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
        os.environ["ROS_LOCALHOST_ONLY"] = "1" if ros_localhost_only else "0"

        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import (
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool

        self.rclpy = rclpy
        self.JointState = JointState
        self.Bool = Bool
        self.PoseStamped = PoseStamped

        if not rclpy.ok():
            rclpy.init(args=[])
        self.node = rclpy.create_node(
            f"rlinf_local_r1pro_ctrl_{os.getpid()}"
        )
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state = GalaxeaR1ProRobotState()
        self.lock = threading.RLock()
        self.first_seen: dict[str, float] = {}
        self.pubs: dict[str, Any] = {}

        if use_right_arm:
            self.pubs["target_joint_state_arm_right"] = (
                self.node.create_publisher(
                    JointState,
                    "/motion_target/target_joint_state_arm_right",
                    reliable,
                )
            )
            self.node.create_subscription(
                JointState,
                "/hdas/feedback_arm_right",
                self._on_arm_right,
                qos_profile_sensor_data,
            )
            self.node.create_subscription(
                PoseStamped,
                "/motion_control/pose_ee_arm_right",
                self._on_pose_right,
                qos_profile_sensor_data,
            )
        if use_left_arm:
            self.pubs["target_joint_state_arm_left"] = (
                self.node.create_publisher(
                    JointState,
                    "/motion_target/target_joint_state_arm_left",
                    reliable,
                )
            )
        self.pubs["brake_mode"] = self.node.create_publisher(
            Bool, "/motion_target/brake_mode", reliable
        )

        # Optional Galaxea SDK message types (BMS / status) — best-effort.
        try:
            from hdas_msg.msg import (  # noqa: WPS433
                Bms,
                ControllerSignalStamped,
                FeedbackStatus,
            )

            self.node.create_subscription(
                Bms, "/hdas/bms", self._on_bms, qos_profile_sensor_data
            )
            self.node.create_subscription(
                ControllerSignalStamped,
                "/controller",
                self._on_controller_signal,
                qos_profile_sensor_data,
            )
            self.node.create_subscription(
                FeedbackStatus,
                "/hdas/feedback_status_arm_right",
                lambda msg: self._on_status(msg, "right"),
                qos_profile_sensor_data,
            )
        except ImportError:
            # Galaxea SDK not sourced — fine for dummy mode and CI.
            pass

        self.running = True
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

    # ── ROS spin / subscriptions ──────────────────────────────────
    def _spin(self) -> None:
        while self.running and self.rclpy.ok():
            self.executor.spin_once(timeout_sec=0.05)
            self._update_feedback_age()

    def _stamp(self, key: str) -> None:
        self.first_seen[key] = time.time()

    def _update_feedback_age(self) -> None:
        now = time.time()
        with self.lock:
            for key, t0 in self.first_seen.items():
                self.state.feedback_age_ms[key] = (now - t0) * 1000.0
            self.state.is_alive = any(
                age < 1500.0 for age in self.state.feedback_age_ms.values()
            )

    def _on_arm_right(self, msg) -> None:  # noqa: ANN001 — ROS callback
        self._stamp("arm_right")
        with self.lock:
            if msg.position:
                self.state.right_arm_qpos = np.asarray(
                    msg.position[:7], dtype=np.float32
                )
            if msg.velocity:
                self.state.right_arm_qvel = np.asarray(
                    msg.velocity[:7], dtype=np.float32
                )
            if msg.effort:
                self.state.right_arm_qtau = np.asarray(
                    msg.effort[:7], dtype=np.float32
                )

    def _on_pose_right(self, msg) -> None:  # noqa: ANN001 — ROS callback
        self._stamp("pose_ee_arm_right")
        p = msg.pose.position
        q = msg.pose.orientation
        with self.lock:
            self.state.right_ee_pose = np.asarray(
                [p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=np.float32
            )

    def _on_bms(self, msg) -> None:  # noqa: ANN001 — ROS callback
        with self.lock:
            self.state.bms["capital_pct"] = float(
                getattr(msg, "capital", getattr(msg, "capital_pct", 100.0))
            )

    def _on_controller_signal(self, msg) -> None:  # noqa: ANN001
        data = getattr(msg, "data", msg)
        with self.lock:
            self._stamp("controller")
            self.state.controller_signal = {
                "mode": int(getattr(data, "mode", 0)),
                "swa": int(getattr(data, "swa", 0)),
                "swb": int(getattr(data, "swb", 0)),
                "swc": int(getattr(data, "swc", 0)),
                "swd": int(getattr(data, "swd", 0)),
            }

    def _on_status(self, msg, side: str) -> None:  # noqa: ANN001
        with self.lock:
            self.state.status_errors[side] = list(
                getattr(msg, "errors", []) or []
            )

    # ── GalaxeaR1ProController API used by RLinf env ──────────────
    def get_state(self) -> GalaxeaR1ProRobotState:
        with self.lock:
            return self.state.copy()

    def is_robot_up(self) -> bool:
        return bool(self.get_state().is_alive)

    def send_arm_joints(
        self, side: str, qpos: list, qvel_max: list | None = None
    ) -> None:
        topic = f"target_joint_state_arm_{side}"
        if topic not in self.pubs:
            raise ValueError(f"{topic} publisher not enabled")
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(self.DEFAULT_JOINT_NAMES.get(side, []))
        msg.position = [float(x) for x in list(qpos)[:7]]
        msg.velocity = [
            float(x)
            for x in (
                qvel_max or [0.3, 0.3, 0.3, 0.3, 0.6, 0.6, 0.6]
            )[:7]
        ]
        self.pubs[topic].publish(msg)

    def apply_brake(self, on: bool) -> None:
        msg = self.Bool()
        msg.data = bool(on)
        self.pubs["brake_mode"].publish(msg)

    def get_subscription_count(self, topic: str) -> int:
        for pub in self.pubs.values():
            if getattr(pub, "topic_name", "") == topic:
                return int(pub.get_subscription_count())
        return 0

    def list_nodes(self) -> list[str]:
        """Return ``"<ns>/<name>"`` strings for every node ROS 2 discovery sees."""
        try:
            return [
                ((ns or "/").rstrip("/") + "/" + name)
                for name, ns in self.node.get_node_names_and_namespaces()
            ]
        except Exception:  # noqa: BLE001 — defensive RPC call
            return []

    def list_topics(self, contains: str | None = None) -> list[str]:
        """Return topic names visible to discovery; optionally substring-filtered."""
        try:
            topics = [t for t, _ in self.node.get_topic_names_and_types()]
        except Exception:  # noqa: BLE001 — defensive RPC call
            return []
        if contains:
            topics = [t for t in topics if contains in t]
        return sorted(topics)

    def shutdown(self) -> None:
        self.running = False
        time.sleep(0.1)
        self.executor.remove_node(self.node)
        self.node.destroy_node()


class LocalControllerRpcShim:
    """Wraps LocalR1ProController in the .wait()[0] RPC pattern RLinf expects."""

    def __init__(self, ctrl: LocalR1ProController) -> None:
        self.controller = ctrl

    def get_state(self) -> LocalRef:
        return LocalRef(self.controller.get_state())

    def is_robot_up(self) -> LocalRef:
        return LocalRef(self.controller.is_robot_up())

    def send_arm_joints(self, *args: Any, **kwargs: Any) -> LocalRef:
        return LocalRef(self.controller.send_arm_joints(*args, **kwargs))

    def apply_brake(self, *args: Any, **kwargs: Any) -> LocalRef:
        return LocalRef(self.controller.apply_brake(*args, **kwargs))

    def get_subscription_count(self, topic: str) -> int:
        return self.controller.get_subscription_count(topic)

    def list_nodes(self) -> list[str]:
        return self.controller.list_nodes()

    def list_topics(self, contains: str | None = None) -> list[str]:
        return self.controller.list_topics(contains)

    def shutdown(self) -> None:
        self.controller.shutdown()


# ─── SafetyConfigActionProjector ──────────────────────────────────
class SafetyConfigActionProjector:
    """Project policy actions into SafetyConfig-safe absolute joint targets.

    Two-stage projection (per guide §0.2 and §2):

    1. Position clip: keep ``q`` away from hard limits by ``margin``.
    2. Step clip: limit ``|q_safe - q_current|`` to ``v_max * dt * step_scale``
       per joint, so a single env step never asks for a fast move.

    The output is a normalized action in :math:`[-1, 1]^7` ready for the
    RLinf ``JointStateDispatcher`` (which itself does an extra schema check).
    """

    def __init__(
        self,
        safety_cfg: SafetyConfig,
        step_scale: float,
        margin: float | None = None,
    ) -> None:
        self.cfg = safety_cfg
        self.q_min = np.asarray(
            safety_cfg.right_arm_q_min, dtype=np.float32
        ).reshape(7)
        self.q_max = np.asarray(
            safety_cfg.right_arm_q_max, dtype=np.float32
        ).reshape(7)
        self.margin = float(
            safety_cfg.l2_critical_margin_rad if margin is None else margin
        )
        self.safe_lo = self.q_min + self.margin
        self.safe_hi = self.q_max - self.margin
        self.step_cap = (
            np.asarray(safety_cfg.arm_qvel_max, dtype=np.float32).reshape(7)
            * float(safety_cfg.dt_step)
            * float(step_scale)
        )
        if not np.all(self.safe_hi > self.safe_lo):
            raise ValueError(
                "SafetyConfig margin leaves no valid joint range; "
                f"safe_lo={self.safe_lo.tolist()}, "
                f"safe_hi={self.safe_hi.tolist()}"
            )
        if not np.all(self.step_cap > 0):
            raise ValueError(
                f"step_cap must be positive; got {self.step_cap.tolist()}"
            )

    def normalize(self, q_abs: np.ndarray) -> np.ndarray:
        q = np.asarray(q_abs, dtype=np.float32).reshape(7)
        return np.clip(
            2.0 * (q - self.q_min) / (self.q_max - self.q_min) - 1.0,
            -1.0,
            1.0,
        ).astype(np.float32)

    def unnormalize(self, action: np.ndarray) -> np.ndarray:
        a = np.clip(np.asarray(action, dtype=np.float32).reshape(7), -1.0, 1.0)
        return (
            self.q_min + (a + 1.0) * 0.5 * (self.q_max - self.q_min)
        ).astype(np.float32)

    def assert_inside(self, name: str, q_abs: np.ndarray) -> None:
        q = np.asarray(q_abs, dtype=np.float32).reshape(7)
        if not np.all((q >= self.safe_lo) & (q <= self.safe_hi)):
            raise RuntimeError(
                f"{name} outside SafetyConfig safe range: "
                f"q={q.tolist()}, safe_lo={self.safe_lo.tolist()}, "
                f"safe_hi={self.safe_hi.tolist()}"
            )

    def project(
        self, policy_action: np.ndarray, q_current: np.ndarray
    ) -> tuple[np.ndarray, dict[str, Any]]:
        q_current = np.asarray(q_current, dtype=np.float32).reshape(7)
        self.assert_inside("q_current", q_current)
        q_desired = np.clip(
            self.unnormalize(policy_action), self.safe_lo, self.safe_hi
        )
        q_safe = np.clip(
            q_desired,
            np.maximum(q_current - self.step_cap, self.safe_lo),
            np.minimum(q_current + self.step_cap, self.safe_hi),
        )
        return self.normalize(q_safe), {
            "q_desired": q_desired.tolist(),
            "q_safe": q_safe.tolist(),
            "delta": (q_safe - q_current).tolist(),
            "step_cap": self.step_cap.tolist(),
        }


# ─── Replay Buffer (single-process, npz persistence) ──────────────
class ReplayBuffer:
    """Minimal replay buffer with ``save``/``load`` for ``(s, a, r, s', d)``."""

    def __init__(self, obs_dim: int, action_dim: int, size: int) -> None:
        self.obs = np.zeros((size, obs_dim), dtype=np.float32)
        self.act = np.zeros((size, action_dim), dtype=np.float32)
        self.rew = np.zeros((size, 1), dtype=np.float32)
        self.next_obs = np.zeros((size, obs_dim), dtype=np.float32)
        self.done = np.zeros((size, 1), dtype=np.float32)
        self.size = int(size)
        self.ptr = 0
        self.count = 0

    def add(
        self,
        obs: np.ndarray,
        action: np.ndarray,
        reward: float,
        next_obs: np.ndarray,
        done: bool,
    ) -> None:
        i = self.ptr
        self.obs[i] = obs
        self.act[i] = action
        self.rew[i] = float(reward)
        self.next_obs[i] = next_obs
        self.done[i] = float(done)
        self.ptr = (self.ptr + 1) % self.size
        self.count = min(self.count + 1, self.size)

    def sample(
        self, batch_size: int, device: torch.device
    ) -> dict[str, torch.Tensor]:
        idx = np.random.randint(0, self.count, size=batch_size)
        return {
            "obs": torch.as_tensor(self.obs[idx], device=device),
            "act": torch.as_tensor(self.act[idx], device=device),
            "rew": torch.as_tensor(self.rew[idx], device=device),
            "next_obs": torch.as_tensor(self.next_obs[idx], device=device),
            "done": torch.as_tensor(self.done[idx], device=device),
        }

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            obs=self.obs[: self.count],
            act=self.act[: self.count],
            rew=self.rew[: self.count],
            next_obs=self.next_obs[: self.count],
            done=self.done[: self.count],
        )
        print(f"[BUFFER] saved {self.count} transitions -> {path}")

    def load(self, path: Path) -> int:
        data = np.load(path)
        n = min(len(data["obs"]), self.size)
        self.obs[:n] = data["obs"][:n]
        self.act[:n] = data["act"][:n]
        self.rew[:n] = data["rew"][:n]
        self.next_obs[:n] = data["next_obs"][:n]
        self.done[:n] = data["done"][:n]
        self.count = n
        self.ptr = n % self.size
        print(f"[BUFFER] loaded {n} transitions <- {path}")
        return n


# ─── Env builder + RLinf monkey-patches (runtime only, no source edit) ──
_PATCHED = False


def _patch_rlinf_for_local_controller() -> None:
    """Make RLinf env construct a local rclpy controller instead of Ray.

    Replaces ``GalaxeaR1ProController.launch_controller`` with a factory that
    returns ``LocalControllerRpcShim`` and disables the env's one-shot
    ``_reset_to_safe_pose`` (we use ``safe_move_to_q`` for segmented resets).

    This is a *runtime* monkey-patch on attribute references — no RLinf
    source files are modified.
    """
    global _PATCHED
    if _PATCHED:
        return

    def _launch_local_controller(**kwargs: Any) -> LocalControllerRpcShim:
        return LocalControllerRpcShim(LocalR1ProController(**kwargs))

    GalaxeaR1ProController.launch_controller = staticmethod(  # type: ignore[assignment]
        _launch_local_controller
    )
    from rlinf.envs.realworld.galaxear import r1_pro_env  # noqa: WPS433

    r1_pro_env.GalaxeaR1ProEnv._reset_to_safe_pose = (  # type: ignore[assignment]
        lambda self: None
    )
    _PATCHED = True


def build_env(
    cfg: dict[str, Any],
    start_q: list | np.ndarray | None = None,
    target_q: list | np.ndarray | None = None,
) -> GalaxeaR1ProSingleArmReachJointEnv:
    """Construct the RLinf joint-reach env with optional pair override.

    ``start_q`` overrides ``home_q_right`` (used by safe_move_to_q for reset).
    ``target_q`` overrides ``target_q_right`` (used by the M1 task reward).
    """
    _patch_rlinf_for_local_controller()
    override = dict(get(cfg, "env.override_cfg") or {})
    if start_q is not None:
        override["home_q_right"] = list(np.asarray(start_q).reshape(-1))
    if target_q is not None:
        override["target_q_right"] = list(np.asarray(target_q).reshape(-1))
    return GalaxeaR1ProSingleArmReachJointEnv(
        override_cfg=override,
        worker_info=None,
        hardware_info=None,
        env_idx=0,
    )


def build_projector(cfg: dict[str, Any]) -> SafetyConfigActionProjector:
    """Build a SafetyConfigActionProjector from cfg + RLinf SafetyConfig."""
    safety_cfg = build_safety_config(
        dict(get(cfg, "env.override_cfg.safety_cfg") or {})
    )
    margin_value = get(cfg, "runtime.safety_margin_rad", None)
    margin = None if margin_value is None else float(margin_value)
    return SafetyConfigActionProjector(
        safety_cfg=safety_cfg,
        step_scale=float(get(cfg, "runtime.safety_step_scale")),
        margin=margin,
    )


# ─── Obs / safe-step helpers ──────────────────────────────────────
def flatten_obs(obs: dict) -> np.ndarray:
    """Flatten ``obs["state"]`` (RLinf returns dict-of-arrays) into 1D float32.

    Sorted-key concatenation gives a stable layout across calls. With
    ``use_right_arm=True, no_gripper=True, cameras=[]`` this yields
    ``right_arm_qpos(7) + right_ee_pose(7) = 14``, matching ``sac.obs_dim``.
    """
    state = obs["state"]
    parts = [
        np.asarray(state[k], dtype=np.float32).reshape(-1)
        for k in sorted(state)
    ]
    return np.concatenate(parts).astype(np.float32)


def safe_step(
    env: GalaxeaR1ProSingleArmReachJointEnv,
    proj: SafetyConfigActionProjector,
    policy_action: np.ndarray,
):
    """Project the action with SafetyConfig limits, then call ``env.step``."""
    st = env._controller.get_state().wait()[0]
    q_current = np.asarray(st.right_arm_qpos, dtype=np.float32)
    safe_action, proj_info = proj.project(policy_action, q_current)
    obs, reward, terminated, truncated, info = env.step(safe_action)
    info = dict(info)
    info["projector"] = proj_info
    return obs, reward, terminated, truncated, info, safe_action


def safe_move_to_q(
    env: GalaxeaR1ProSingleArmReachJointEnv,
    proj: SafetyConfigActionProjector,
    q_goal: np.ndarray,
    cfg: dict[str, Any],
) -> np.ndarray:
    """Drive the arm to ``q_goal`` in joint mode, segmented by ``step_cap``.

    Each iteration sends ``normalize(q_goal)`` through the projector, which
    clips to one ``step_cap`` worth of motion. Stops when ``||q - q_goal|| <
    safe_reset_tolerance_rad`` or after ``safe_reset_max_steps``.
    """
    proj.assert_inside("q_goal", q_goal)
    tol = float(get(cfg, "train.safe_reset_tolerance_rad"))
    max_steps = int(get(cfg, "train.safe_reset_max_steps"))
    obs = env._get_observation()
    for _ in range(max_steps):
        st = env._controller.get_state().wait()[0]
        cur = np.asarray(st.right_arm_qpos, dtype=np.float32)
        if float(np.linalg.norm(q_goal - cur)) <= tol:
            return flatten_obs(obs)
        raw_action = proj.normalize(q_goal)
        obs, _, _, _, info, _ = safe_step(env, proj, raw_action)
        if info.get("safe_pause"):
            raise RuntimeError(f"safe_pause during reset: {info}")
    raise RuntimeError(
        f"safe_move_to_q did not converge to {np.asarray(q_goal).tolist()}"
    )


def preflight(
    env: GalaxeaR1ProSingleArmReachJointEnv,
    proj: SafetyConfigActionProjector,
    start_q: list | np.ndarray,
    target_q: list | np.ndarray,
    *,
    target_topic: str = "/motion_target/target_joint_state_arm_right",
    sub_wait_timeout_sec: float = 15.0,
    sub_poll_interval_sec: float = 0.5,
) -> None:
    """Validate joint ranges and wait for mobiman to subscribe to the target topic.

    On timeout (no subscriber), prints a rich diagnostic block (ROS env,
    visible nodes, related topics, feedback freshness) so the engineer can
    immediately tell whether mobiman is not running, or running on a
    different ROS_DOMAIN_ID / RMW_IMPLEMENTATION than this process.
    """
    st = env._controller.get_state().wait()[0]
    q_current = np.asarray(st.right_arm_qpos, dtype=np.float32)
    sq = np.asarray(start_q, dtype=np.float32)
    tq = np.asarray(target_q, dtype=np.float32)
    proj.assert_inside("current_q", q_current)
    proj.assert_inside("start_q", sq)
    proj.assert_inside("target_q", tq)

    ros_domain = os.environ.get("ROS_DOMAIN_ID", "(unset, default 0)")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "(unset)")
    localhost_only = os.environ.get("ROS_LOCALHOST_ONLY", "(unset)")

    print(f"[PREFLIGHT] current_q          : {q_current.tolist()}")
    print(f"[PREFLIGHT] start_q            : {sq.tolist()}")
    print(f"[PREFLIGHT] target_q           : {tq.tolist()}")
    print(f"[PREFLIGHT] safe_lo            : {proj.safe_lo.tolist()}")
    print(f"[PREFLIGHT] safe_hi            : {proj.safe_hi.tolist()}")
    print(f"[PREFLIGHT] step_cap           : {proj.step_cap.tolist()}")
    print(f"[PREFLIGHT] ROS_DOMAIN_ID      : {ros_domain}")
    print(f"[PREFLIGHT] RMW_IMPLEMENTATION : {rmw}")
    print(f"[PREFLIGHT] ROS_LOCALHOST_ONLY : {localhost_only}")
    print(
        f"[PREFLIGHT] waiting up to {sub_wait_timeout_sec:.1f}s for a "
        f"subscriber on {target_topic} ..."
    )

    deadline = time.time() + float(sub_wait_timeout_sec)
    sub_count = 0
    while time.time() < deadline:
        sub_count = env._controller.get_subscription_count(target_topic)
        if sub_count > 0:
            break
        time.sleep(float(sub_poll_interval_sec))

    if sub_count > 0:
        print(
            f"[PREFLIGHT] subscribers on {target_topic}: {sub_count} ✓"
        )
        return

    # Timed out — gather diagnostics to make the failure actionable.
    ctrl = env._controller
    nodes: list[str] = []
    motion_topics: list[str] = []
    hdas_topics: list[str] = []
    if hasattr(ctrl, "list_nodes"):
        try:
            nodes = ctrl.list_nodes()
        except Exception:  # noqa: BLE001
            nodes = []
    if hasattr(ctrl, "list_topics"):
        try:
            motion_topics = ctrl.list_topics(contains="/motion_target")
            hdas_topics = ctrl.list_topics(contains="/hdas")
        except Exception:  # noqa: BLE001
            pass
    feedback_age_ms = getattr(st, "feedback_age_ms", {}) or {}
    feedback_lines = (
        ", ".join(f"{k}={v:.0f}ms" for k, v in feedback_age_ms.items())
        or "<none received yet>"
    )

    diag_lines = [
        "",
        f"  No subscriber appeared on {target_topic} within "
        f"{sub_wait_timeout_sec:.1f}s.",
        "  Diagnostics:",
        f"    ROS_DOMAIN_ID      = {ros_domain}",
        f"    RMW_IMPLEMENTATION = {rmw}",
        f"    ROS_LOCALHOST_ONLY = {localhost_only}",
        f"    visible nodes      = {nodes if nodes else '<none>'}",
        f"    /motion_target/*   = {motion_topics if motion_topics else '<none>'}",
        f"    /hdas/*            = {hdas_topics if hdas_topics else '<none>'}",
        f"    feedback freshness = {feedback_lines}",
        "  Likely causes (in order of frequency):",
        "    1. mobiman joint tracker is not running.  Launch it on the",
        "       robot in another terminal with the SAME ROS_DOMAIN_ID and",
        "       RMW_IMPLEMENTATION as printed above:",
        "         source /home/nvidia/galaxea/install/setup.bash",
        f"         export ROS_DOMAIN_ID={ros_domain}",
        f"         export RMW_IMPLEMENTATION={rmw}",
        "         <galaxea mobiman launch command for joint mode>",
        "    2. mobiman is running, but on a different ROS_DOMAIN_ID or RMW.",
        "       Verify with (in a fresh shell, with same env exported):",
        f"         ROS_DOMAIN_ID={ros_domain} RMW_IMPLEMENTATION={rmw} \\",
        f"           ros2 topic info {target_topic} -v",
        "    3. ROS_LOCALHOST_ONLY=1 with mobiman on another machine; set 0.",
        "",
    ]
    raise RuntimeError("\n".join(diag_lines))
