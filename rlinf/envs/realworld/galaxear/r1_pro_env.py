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

"""Galaxea R1 Pro real-world environment.

Mirrors :class:`rlinf.envs.realworld.franka.franka_env.FrankaEnv`'s
constructor signature so that
:meth:`rlinf.envs.realworld.realworld_env.RealWorldEnv._create_env`
can dispatch via ``gym.make(init_params.id, override_cfg=...,
worker_info=..., hardware_info=..., env_idx=...)``.

Naming:
    * file: ``r1_pro_env.py``
    * class prefix: ``GalaxeaR1Pro*``

See design doc §6.6 for full rationale.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Optional

import gymnasium as gym
import numpy as np

from rlinf.scheduler import GalaxeaR1ProHWInfo, WorkerInfo
from rlinf.utils.logging import get_logger

from .r1_pro_action_schema import ActionSchema, build_action_schema
from .r1_pro_camera_mux import CameraMuxConfig, GalaxeaR1ProCameraMux
from .r1_pro_robot_state import GalaxeaR1ProRobotState
from .r1_pro_safety import (
    GalaxeaR1ProSafetySupervisor,
    SafetyInfo,
    build_safety_config,
)


@dataclass
class GalaxeaR1ProRobotConfig:
    """Per-instance R1 Pro env configuration.

    Most fields mirror :class:`FrankaRobotConfig` with dual-arm /
    torso / chassis extensions.  Cameras live in :class:`CameraSpec`
    inside :class:`GalaxeaR1ProConfig`; this dataclass also accepts a
    list of dicts via ``cameras`` so that ``override_cfg`` can fully
    drive the env without going through the hardware registry.
    """

    # ── Stage flags ─────────────────────────────────────────────
    use_left_arm: bool = False
    use_right_arm: bool = True
    use_torso: bool = False
    use_chassis: bool = False
    no_gripper: bool = False

    # ── Hardware / connection ───────────────────────────────────
    is_dummy: bool = False
    robot_ip: Optional[str] = None
    ros_domain_id: int = 72
    ros_localhost_only: bool = False
    galaxea_install_path: str = "~/galaxea/install"
    mobiman_launch_mode: str = "pose"

    # ── Cameras ─────────────────────────────────────────────────
    cameras: list = field(default_factory=list)
    image_size: tuple = (128, 128)
    soft_sync_window_ms: int = 33
    align_strategy: str = "latest"
    fallback_to_ros2_on_usb_drop: bool = True

    # ── Control ─────────────────────────────────────────────────
    step_frequency: float = 10.0
    action_scale: list = field(default_factory=lambda: [0.05, 0.10, 1.0])
    binary_gripper_threshold: float = 0.5
    enable_camera_player: bool = False

    # ── Targets / resets ────────────────────────────────────────
    target_ee_pose_right: list = field(default_factory=lambda: [
        0.45, -0.10, 0.30, -3.14, 0.0, 0.0,
    ])
    target_ee_pose_left: Optional[list] = None
    reset_ee_pose_right: list = field(default_factory=lambda: [
        0.35, -0.10, 0.45, -3.14, 0.0, 0.0,
    ])
    reset_ee_pose_left: Optional[list] = None
    joint_reset_qpos_right: list = field(default_factory=lambda: [
        0.0, 0.3, 0.0, -1.8, 0.0, 2.1, 0.0,
    ])
    joint_reset_qpos_left: Optional[list] = None
    max_num_steps: int = 200
    success_hold_steps: int = 5

    # ── Reward ──────────────────────────────────────────────────
    use_dense_reward: bool = False
    reward_scale: float = 1.0
    reward_threshold: list = field(default_factory=lambda: [
        0.02, 0.02, 0.02, 0.20, 0.20, 0.20,
    ])
    enable_gripper_penalty: bool = True
    gripper_penalty: float = 0.1

    # ── Reward model worker (re-uses Franka path) ───────────────
    use_reward_model: bool = False
    reward_worker_cfg: Optional[dict] = None
    reward_worker_hardware_rank: Optional[int] = None
    reward_worker_node_rank: Optional[int] = None
    reward_worker_node_group: Optional[str] = None
    reward_image_key: Optional[str] = None

    # ── Safety overrides ────────────────────────────────────────
    safety_cfg: Optional[dict] = None

    # ── Misc ────────────────────────────────────────────────────
    task_description: str = ""

    def __post_init__(self) -> None:
        if isinstance(self.image_size, list):
            self.image_size = tuple(self.image_size)
        # No numpy coercion here -- keep dataclass picklable; numpy
        # arrays are constructed at use site (env.step / safety).


class GalaxeaR1ProEnv(gym.Env):
    """Galaxea R1 Pro :class:`gym.Env` implementation.

    Constructor signature exactly matches
    :class:`FrankaEnv.__init__(override_cfg, worker_info, hardware_info, env_idx)`
    so :meth:`RealWorldEnv._create_env` works without modification.
    """

    CONFIG_CLS: type[GalaxeaR1ProRobotConfig] = GalaxeaR1ProRobotConfig
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(
        self,
        override_cfg: dict[str, Any],
        worker_info: Optional[WorkerInfo] = None,
        hardware_info: Optional[GalaxeaR1ProHWInfo] = None,
        env_idx: int = 0,
    ) -> None:
        super().__init__()
        self.config: GalaxeaR1ProRobotConfig = self.CONFIG_CLS(
            **(override_cfg or {})
        )
        self._task_description: str = self.config.task_description
        self._logger = get_logger()
        self._worker_info = worker_info
        self._hardware_info = hardware_info
        self._env_idx = int(env_idx)

        self.node_rank: int = 0
        self.env_worker_rank: int = 0
        if worker_info is not None:
            self.node_rank = getattr(worker_info, "cluster_node_rank", 0)
            self.env_worker_rank = getattr(worker_info, "rank", 0)

        # Internal state
        self._state = GalaxeaR1ProRobotState()
        self._num_steps = 0
        self._success_hold_counter = 0

        # Lazy-built sub-components
        self._controller = None
        self._camera_mux: Optional[GalaxeaR1ProCameraMux] = None
        self._safety: Optional[GalaxeaR1ProSafetySupervisor] = None
        self._reward_worker = None
        self._action_schema: ActionSchema = build_action_schema(self.config)

        # Init action / observation spaces *before* talking to hardware
        # so dummy mode also exposes correct spaces.
        self._init_action_obs_spaces()

        # Always install the safety supervisor; cheap and stage-aware.
        self._install_safety_supervisor()

        if not self.config.is_dummy:
            try:
                self._setup_hardware()
            except Exception as e:
                self._logger.error(
                    "GalaxeaR1ProEnv hardware setup failed: %s", e,
                )
                raise

        if self.config.use_reward_model:
            self._setup_reward_worker()

        if not self.config.is_dummy:
            self._wait_robot_ready(timeout=30.0)
            try:
                self._reset_to_safe_pose()
            except Exception as e:  # pragma: no cover - hardware path
                self._logger.warning(
                    "Initial reset_to_safe_pose failed (continuing): %s", e,
                )

    # ── Public properties ───────────────────────────────────────

    @property
    def task_description(self) -> str:
        return self._task_description

    def get_action_scale(self) -> np.ndarray:
        return np.asarray(self.config.action_scale, dtype=np.float32)

    def get_tcp_pose(self) -> np.ndarray:
        """Return the right (or left) arm EE pose ``[xyz + quat]``."""
        if self.config.use_right_arm:
            return self._state.right_ee_pose.copy()
        return self._state.left_ee_pose.copy()

    # ── Spaces ──────────────────────────────────────────────────

    def _init_action_obs_spaces(self) -> None:
        action_dim = self._action_schema.action_dim
        self.action_space = gym.spaces.Box(
            low=-np.ones(action_dim, dtype=np.float32),
            high=np.ones(action_dim, dtype=np.float32),
            dtype=np.float32,
        )

        state_spaces: dict = {}
        if self.config.use_right_arm:
            state_spaces["right_arm_qpos"] = gym.spaces.Box(
                -np.inf, np.inf, shape=(7,), dtype=np.float32,
            )
            state_spaces["right_ee_pose"] = gym.spaces.Box(
                -np.inf, np.inf, shape=(7,), dtype=np.float32,
            )
            if not self.config.no_gripper:
                state_spaces["right_gripper_pos"] = gym.spaces.Box(
                    0.0, 1.0, shape=(1,), dtype=np.float32,
                )
        if self.config.use_left_arm:
            state_spaces["left_arm_qpos"] = gym.spaces.Box(
                -np.inf, np.inf, shape=(7,), dtype=np.float32,
            )
            state_spaces["left_ee_pose"] = gym.spaces.Box(
                -np.inf, np.inf, shape=(7,), dtype=np.float32,
            )
            if not self.config.no_gripper:
                state_spaces["left_gripper_pos"] = gym.spaces.Box(
                    0.0, 1.0, shape=(1,), dtype=np.float32,
                )
        if self.config.use_torso:
            state_spaces["torso_qpos"] = gym.spaces.Box(
                -np.inf, np.inf, shape=(4,), dtype=np.float32,
            )
        if self.config.use_chassis:
            state_spaces["chassis_state"] = gym.spaces.Box(
                -np.inf, np.inf, shape=(6,), dtype=np.float32,
            )

        h, w = self.config.image_size
        frame_spaces: dict = {}
        for spec in self.config.cameras:
            name = spec["name"] if isinstance(spec, dict) else spec.name
            enable_depth = (
                spec["enable_depth"] if isinstance(spec, dict)
                else spec.enable_depth
            )
            channels = 4 if enable_depth else 3
            dtype = np.uint16 if enable_depth else np.uint8
            high = 65535 if dtype == np.uint16 else 255
            frame_spaces[name] = gym.spaces.Box(
                0, high, shape=(h, w, channels), dtype=dtype,
            )

        self.observation_space = gym.spaces.Dict(
            {
                "state": gym.spaces.Dict(state_spaces),
                "frames": gym.spaces.Dict(frame_spaces),
            }
        )

    # ── Hardware setup ──────────────────────────────────────────

    def _setup_hardware(self) -> None:
        """Bring up controller worker + camera mux."""
        from .r1_pro_controller import GalaxeaR1ProController

        cfg = self.config

        controller_node_rank: Optional[int] = None
        camera_node_rank: Optional[int] = None
        hw_cfg = (
            self._hardware_info.config if self._hardware_info is not None
            else None
        )
        if hw_cfg is not None:
            controller_node_rank = getattr(hw_cfg, "controller_node_rank", None)
            camera_node_rank = getattr(hw_cfg, "camera_node_rank", None)
        if controller_node_rank is None:
            controller_node_rank = self.node_rank
        if camera_node_rank is None:
            camera_node_rank = self.node_rank

        self._controller = GalaxeaR1ProController.launch_controller(
            env_idx=self._env_idx,
            node_rank=int(controller_node_rank),
            worker_rank=int(self.env_worker_rank),
            ros_domain_id=cfg.ros_domain_id,
            ros_localhost_only=cfg.ros_localhost_only,
            galaxea_install_path=cfg.galaxea_install_path,
            use_left_arm=cfg.use_left_arm,
            use_right_arm=cfg.use_right_arm,
            use_torso=cfg.use_torso,
            use_chassis=cfg.use_chassis,
            mobiman_launch_mode=cfg.mobiman_launch_mode,
            is_dummy=False,
        )

        # Build the camera mux on this node (where USB cables are).
        mux_cfg = CameraMuxConfig(
            cameras=list(cfg.cameras),
            image_size=cfg.image_size,
            soft_sync_window_ms=cfg.soft_sync_window_ms,
            align_strategy=cfg.align_strategy,
            fallback_to_ros2_on_usb_drop=cfg.fallback_to_ros2_on_usb_drop,
            is_dummy=False,
        )
        self._camera_mux = GalaxeaR1ProCameraMux(mux_cfg)

    def _install_safety_supervisor(self) -> None:
        cfg_dict = dict(self.config.safety_cfg or {})
        self._safety = GalaxeaR1ProSafetySupervisor(
            build_safety_config(cfg_dict),
        )

    def _setup_reward_worker(self) -> None:
        from rlinf.workers.reward.reward_worker import EmbodiedRewardWorker

        if self.config.reward_worker_cfg is None:
            raise ValueError(
                "use_reward_model=True but reward_worker_cfg is missing"
            )
        node_rank = self.config.reward_worker_node_rank
        if node_rank is None:
            node_rank = self.node_rank
        self._reward_worker = EmbodiedRewardWorker.launch_for_realworld(
            reward_cfg=self.config.reward_worker_cfg,
            node_rank=node_rank,
            node_group_label=self.config.reward_worker_node_group,
            hardware_rank=self.config.reward_worker_hardware_rank,
            env_idx=self._env_idx,
            worker_rank=self.env_worker_rank,
        )
        self._reward_worker.init_worker().wait()

    def _wait_robot_ready(self, timeout: float = 30.0) -> None:
        if self._controller is None:
            return
        start = time.time()
        while time.time() - start < timeout:
            try:
                if self._controller.is_robot_up().wait()[0]:
                    return
            except Exception as e:
                self._logger.debug("is_robot_up RPC failed: %s", e)
            time.sleep(0.5)
        self._logger.warning(
            "Timed out waiting for R1 Pro feedback topics; continuing."
        )

    # ── step / reset ────────────────────────────────────────────

    def step(self, action: np.ndarray):
        t0 = time.time()
        action = np.asarray(action, dtype=np.float32).reshape(-1)

        # Refresh state for safety prediction.
        if not self.config.is_dummy and self._controller is not None:
            try:
                self._state = self._controller.get_state().wait()[0]
            except Exception as e:
                self._logger.warning("get_state RPC failed: %s", e)

        # Safety pipeline.
        sinfo: SafetyInfo = self._safety.validate(
            action, self._state, self._action_schema,
        )
        safe_action = sinfo.safe_action

        # Execute (only when not dummy and not e-stop).
        if not self.config.is_dummy and self._controller is not None:
            try:
                if sinfo.emergency_stop:
                    self._controller.apply_brake(True).wait()
                elif sinfo.safe_stop:
                    # SAFE_STOP -> hold pose / brake; do not publish action.
                    self._controller.apply_brake(True).wait()
                else:
                    self._dispatch_action(safe_action)
            except Exception as e:
                self._logger.warning("controller dispatch failed: %s", e)

        # Maintain step frequency.
        elapsed = time.time() - t0
        sleep_for = (1.0 / max(self.config.step_frequency, 1e-3)) - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)

        # Refresh observation.
        if not self.config.is_dummy and self._controller is not None:
            try:
                self._state = self._controller.get_state().wait()[0]
            except Exception:
                pass
        obs = self._get_observation()

        reward = self._calc_step_reward(obs, sinfo)
        self._num_steps += 1
        terminated = (reward >= 1.0) and (
            self._success_hold_counter >= int(self.config.success_hold_steps)
        )
        truncated = self._num_steps >= int(self.config.max_num_steps)
        info = {
            "safety_info": dict(sinfo.metrics),
            "safety_reasons": list(sinfo.reason),
            "step_count": int(self._num_steps),
        }
        if sinfo.safe_stop or sinfo.emergency_stop:
            info["safe_pause"] = True
            truncated = True
        return obs, float(reward * self.config.reward_scale), terminated, truncated, info

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._num_steps = 0
        self._success_hold_counter = 0
        if self.config.is_dummy:
            return self._get_observation(), {}
        try:
            self._reset_to_safe_pose()
        except Exception as e:
            self._logger.warning("reset_to_safe_pose failed: %s", e)
        if self._controller is not None:
            try:
                self._state = self._controller.get_state().wait()[0]
            except Exception:
                pass
        return self._get_observation(), {}

    # ── Action dispatch ────────────────────────────────────────

    def _dispatch_action(self, safe_action: np.ndarray) -> None:
        from scipy.spatial.transform import Rotation as R

        d = self._action_schema.split(safe_action)
        # Right arm
        if "right_xyz" in d:
            target = self._action_schema.predict_arm_ee_pose(
                "right", safe_action, self._state,
            )
            quat = R.from_euler("xyz", target[3:]).as_quat()
            pose = np.concatenate([target[:3], quat]).astype(np.float32)
            self._controller.send_arm_pose("right", pose).wait()
            if not self.config.no_gripper and "right_gripper" in d:
                pct = float(np.clip((d["right_gripper"] + 1.0) * 50.0, 0.0, 100.0))
                self._controller.send_gripper("right", pct).wait()
        # Left arm
        if "left_xyz" in d:
            target = self._action_schema.predict_arm_ee_pose(
                "left", safe_action, self._state,
            )
            quat = R.from_euler("xyz", target[3:]).as_quat()
            pose = np.concatenate([target[:3], quat]).astype(np.float32)
            self._controller.send_arm_pose("left", pose).wait()
            if not self.config.no_gripper and "left_gripper" in d:
                pct = float(np.clip((d["left_gripper"] + 1.0) * 50.0, 0.0, 100.0))
                self._controller.send_gripper("left", pct).wait()
        # Torso / chassis (already capped by L4 in safety)
        if "torso_twist" in d:
            self._controller.send_torso_twist(d["torso_twist"]).wait()
        if "chassis_twist" in d:
            self._controller.send_chassis_twist(d["chassis_twist"]).wait()

    # ── Reset choreography ────────────────────────────────────

    def _reset_to_safe_pose(self) -> None:
        """Reset arms / torso / chassis to a safe configuration.

        Steps (per design doc §6.6 / improvement 9):
        1. BMS check; below threshold -> SAFE_PAUSE before motion.
        2. Joint tracker reset for each enabled arm.
        3. Optional EE-pose alignment via mobiman relaxed_ik.
        4. Brake chassis to zero twist.
        """
        cfg = self.config
        if self._controller is None:
            return
        try:
            self._state = self._controller.get_state().wait()[0]
        except Exception:
            pass

        bms_pct = float(self._state.bms.get("capital_pct", 100.0))
        if (
            self._safety is not None
            and bms_pct < self._safety.cfg.bms_low_battery_threshold_pct
        ):
            self._logger.warning(
                "BMS=%.1f%% below threshold; skipping reset motion.",
                bms_pct,
            )
            return

        if cfg.use_right_arm and cfg.joint_reset_qpos_right:
            self._safe_go_to_rest("right", cfg.joint_reset_qpos_right)
        if cfg.use_left_arm and cfg.joint_reset_qpos_left:
            self._safe_go_to_rest("left", cfg.joint_reset_qpos_left)

        if cfg.use_right_arm and cfg.reset_ee_pose_right:
            try:
                self._send_pose("right", cfg.reset_ee_pose_right)
            except Exception as e:
                self._logger.warning(
                    "reset right EE pose failed: %s", e,
                )
        if cfg.use_left_arm and cfg.reset_ee_pose_left:
            try:
                self._send_pose("left", cfg.reset_ee_pose_left)
            except Exception as e:
                self._logger.warning(
                    "reset left EE pose failed: %s", e,
                )

        if cfg.use_chassis:
            try:
                self._controller.apply_brake(True).wait()
                self._controller.send_chassis_twist(np.zeros(3, dtype=np.float32)).wait()
            except Exception:
                pass

    def _safe_go_to_rest(self, side: str, qpos: list) -> None:
        ok = self._controller.go_to_rest(side, list(qpos), 5.0).wait()[0]
        if not ok:
            self._logger.warning(
                "go_to_rest(%s) timed out; trying clear_errors + retry.",
                side,
            )
            try:
                self._controller.clear_errors(side).wait()
            except Exception:
                pass
            ok2 = self._controller.go_to_rest(side, list(qpos), 5.0).wait()[0]
            if not ok2:
                raise RuntimeError(
                    f"Reset failed for {side} arm twice; entering FATAL"
                )

    def _send_pose(self, side: str, ee_xyzrpy: list) -> None:
        from scipy.spatial.transform import Rotation as R
        ee = np.asarray(ee_xyzrpy, dtype=np.float32).reshape(-1)[:6]
        quat = R.from_euler("xyz", ee[3:]).as_quat()
        pose = np.concatenate([ee[:3], quat]).astype(np.float32)
        self._controller.send_arm_pose(side, pose).wait()

    # ── Observation / reward ─────────────────────────────────

    def _get_observation(self) -> dict:
        if self.config.is_dummy:
            return self._dummy_observation()
        frames = (
            self._camera_mux.get_frames()
            if self._camera_mux is not None else {}
        )
        # Fill gaps with zero matrices to preserve shape contract.
        h, w = self.config.image_size
        for spec in self.config.cameras:
            name = spec["name"] if isinstance(spec, dict) else spec.name
            if name not in frames:
                channels = 4 if (
                    spec["enable_depth"] if isinstance(spec, dict)
                    else spec.enable_depth
                ) else 3
                dtype = np.uint16 if channels == 4 else np.uint8
                frames[name] = np.zeros((h, w, channels), dtype=dtype)
        return {
            "state": self._build_state_dict(),
            "frames": frames,
        }

    def _build_state_dict(self) -> dict:
        cfg = self.config
        st = self._state
        d: dict = {}
        if cfg.use_right_arm:
            d["right_arm_qpos"] = st.right_arm_qpos.copy()
            d["right_ee_pose"] = st.right_ee_pose.copy()
            if not cfg.no_gripper:
                d["right_gripper_pos"] = np.array(
                    [st.right_gripper_pos / 100.0], dtype=np.float32,
                )
        if cfg.use_left_arm:
            d["left_arm_qpos"] = st.left_arm_qpos.copy()
            d["left_ee_pose"] = st.left_ee_pose.copy()
            if not cfg.no_gripper:
                d["left_gripper_pos"] = np.array(
                    [st.left_gripper_pos / 100.0], dtype=np.float32,
                )
        if cfg.use_torso:
            d["torso_qpos"] = st.torso_qpos.copy()
        if cfg.use_chassis:
            d["chassis_state"] = np.concatenate(
                [st.chassis_qpos, st.chassis_qvel],
            ).astype(np.float32)
        return d

    def _dummy_observation(self) -> dict:
        sample_state: dict = {}
        for k, box in self.observation_space["state"].spaces.items():
            sample_state[k] = np.zeros(box.shape, dtype=box.dtype)
        sample_frames: dict = {}
        for k, box in self.observation_space["frames"].spaces.items():
            sample_frames[k] = np.zeros(box.shape, dtype=box.dtype)
        return {"state": sample_state, "frames": sample_frames}

    def _calc_step_reward(self, obs: dict, sinfo: SafetyInfo) -> float:
        """Default geometric reward: AND of per-arm distance to target.

        Subclasses (tasks) may override this entirely.
        """
        if self.config.use_reward_model:
            return self._compute_reward_model(obs)
        if self.config.is_dummy:
            return 0.0

        cfg = self.config
        deltas: list[np.ndarray] = []
        success = True
        if cfg.use_right_arm and cfg.target_ee_pose_right:
            delta = self._delta_to_target(
                self._state.right_ee_pose,
                np.asarray(cfg.target_ee_pose_right, dtype=np.float32),
            )
            deltas.append(delta)
            thresh = np.asarray(cfg.reward_threshold, dtype=np.float32)
            success = success and bool(np.all(delta[:3] <= thresh[:3]))
        if cfg.use_left_arm and cfg.target_ee_pose_left:
            delta = self._delta_to_target(
                self._state.left_ee_pose,
                np.asarray(cfg.target_ee_pose_left, dtype=np.float32),
            )
            deltas.append(delta)
            thresh = np.asarray(cfg.reward_threshold, dtype=np.float32)
            success = success and bool(np.all(delta[:3] <= thresh[:3]))
        if not deltas:
            return 0.0
        if success:
            self._success_hold_counter += 1
            return 1.0
        self._success_hold_counter = 0
        if cfg.use_dense_reward:
            d = float(np.mean(
                [np.sum(np.square(x[:3])) for x in deltas]
            ))
            return float(np.exp(-500.0 * d))
        return 0.0

    def _delta_to_target(
        self, ee_pose_xyzquat: np.ndarray, target_xyzrpy: np.ndarray,
    ) -> np.ndarray:
        from scipy.spatial.transform import Rotation as R
        cur_xyz = np.asarray(ee_pose_xyzquat[:3], dtype=np.float32)
        cur_eul = np.abs(
            R.from_quat(np.asarray(ee_pose_xyzquat[3:], dtype=np.float64)).as_euler("xyz")
        ).astype(np.float32)
        cur = np.concatenate([cur_xyz, cur_eul])
        return np.abs(cur - target_xyzrpy.astype(np.float32))

    def _compute_reward_model(self, obs: dict) -> float:
        if self._reward_worker is None:
            raise RuntimeError("Reward worker not initialized.")
        frames = obs.get("frames", {})
        if not frames:
            raise ValueError("No frames available for reward model inference.")
        key = self.config.reward_image_key or sorted(frames.keys())[0]
        if key not in frames:
            raise KeyError(
                f"reward_image_key {key!r} missing; have {list(frames.keys())}"
            )
        batch = np.expand_dims(frames[key], axis=0)
        out = self._reward_worker.compute_image_rewards(batch).wait()[0]
        if hasattr(out, "detach"):
            out = out.detach().cpu().numpy()
        r = float(np.asarray(out).reshape(-1)[0])
        if r >= 1.0:
            self._success_hold_counter += 1
        else:
            self._success_hold_counter = 0
        return r

    def close(self) -> None:
        try:
            if self._camera_mux is not None:
                self._camera_mux.close()
        except Exception:
            pass
        try:
            if self._controller is not None:
                self._controller.shutdown().wait()
        except Exception:
            pass
