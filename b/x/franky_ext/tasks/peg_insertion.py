"""Franky-backed PegInsertion env with calibrated safety defaults."""

from __future__ import annotations

import copy
import logging
from dataclasses import dataclass, field

import gymnasium as gym
import numpy as np

from franky_ext.franky_single_franka_env import FrankySingleFrankaEnvMixin
from rlinf.envs.realworld.franka.franka_env import FrankaEnv
from rlinf.envs.realworld.franka.tasks.peg_insertion_env import (
    PegInsertionConfig,
    PegInsertionEnv,
)

_log = logging.getLogger(__name__)


@dataclass
class FrankyPegInsertionEnvConfig(PegInsertionConfig):
    """Peg insertion config with Franky smoke / safety knobs."""

    safe_smoke_hold: bool = False
    reset_z_offset: float = 0.0
    safety_box_half_width_m: float = 0.05

    # Absolute 6-D reset pose [x, y, z, roll, pitch, yaw]. When set, the reset
    # pose no longer derives from target_ee_pose, and the safety box is
    # re-anchored on it. Use this when the demo start pose (e.g. above the
    # object to grasp) differs from the task target pose (e.g. the socket).
    reset_ee_pose_override: list[float] | None = None
    # When False the target-pose reward/termination is skipped entirely and the
    # reward must come from a teleop/keyboard wrapper.
    use_pose_reward: bool = True

    camera_image_size: int = 224
    invert_gripper_action: bool = True
    invert_gripper_width_obs: bool = True
    gripper_max_width: float = 0.08
    gripper_width_obs_range: tuple[float, float] = field(
        default_factory=lambda: (0.01, 0.09)
    )

    def __post_init__(self):
        half = float(self.safety_box_half_width_m)
        self.clip_x_range = half
        self.clip_y_range = half
        self.clip_z_range_low = half
        self.clip_z_range_high = half
        self.random_xy_range = min(float(self.random_xy_range), half)
        super().__post_init__()
        if self.reset_ee_pose_override is not None:
            pose = np.asarray(self.reset_ee_pose_override, dtype=float)
            if pose.shape != (6,):
                raise ValueError(
                    "reset_ee_pose_override must be 6-D [x, y, z, roll, pitch, yaw], "
                    f"got shape {pose.shape}"
                )
            self.reset_ee_pose = pose
            self._anchor_safety_box(pose)
        else:
            # Reset = target + z offset (default 0 = same as target)
            self.reset_ee_pose = self.target_ee_pose + np.array(
                [0.0, 0.0, float(self.reset_z_offset), 0.0, 0.0, 0.0]
            )

    def _anchor_safety_box(self, center: np.ndarray):
        """Re-anchor the safety box on ``center`` instead of ``target_ee_pose``."""
        self.ee_pose_limit_min = np.array(
            [
                center[0] - self.clip_x_range,
                center[1] - self.clip_y_range,
                center[2] - self.clip_z_range_low,
                center[3] - 0.01,
                center[4] - 0.01,
                center[5] - self.clip_rz_range,
            ]
        )
        self.ee_pose_limit_max = np.array(
            [
                center[0] + self.clip_x_range,
                center[1] + self.clip_y_range,
                center[2] + self.clip_z_range_high,
                center[3] + 0.01,
                center[4] + 0.01,
                center[5] + self.clip_rz_range,
            ]
        )


class FrankyPegInsertionEnv(FrankySingleFrankaEnvMixin, PegInsertionEnv):
    """Peg insertion on Franky with 5 cm lift / safety box defaults."""

    CONFIG_CLS = FrankyPegInsertionEnvConfig

    def _init_action_obs_spaces(self):
        super()._init_action_obs_spaces()
        sz = self.config.camera_image_size
        if sz != 128:
            self.observation_space["frames"] = gym.spaces.Dict(
                {
                    name: gym.spaces.Box(0, 255, shape=(sz, sz, 3), dtype=np.uint8)
                    for name in self.observation_space["frames"].spaces
                }
            )
            self._base_observation_space = copy.deepcopy(self.observation_space)

    def step(self, action):
        if self.config.invert_gripper_action and action.shape[-1] > 6:
            action = np.array(action, copy=True)
            action[..., 6] = -action[..., 6]
        if not self.config.is_dummy and self._num_steps % 10 == 0:
            tcp_before = self._franka_state.tcp_pose[:3].copy()
            clipped = np.clip(action, self.action_space.low, self.action_space.high)
            if self.config.use_absolute_action:
                _log.warning(
                    "[ActionDebug] step=%d ABSOLUTE raw_act=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f] "
                    "tcp=[%.4f,%.4f,%.4f]",
                    self._num_steps,
                    *action[:7],
                    *tcp_before,
                )
            else:
                scaled_xyz = clipped[:3] * self.config.action_scale[0]
                _log.warning(
                    "[ActionDebug] step=%d raw_act=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f] "
                    "clipped=[%.4f,%.4f,%.4f] scaled_xyz_m=[%.5f,%.5f,%.5f] "
                    "tcp=[%.4f,%.4f,%.4f]",
                    self._num_steps,
                    *action[:7],
                    *clipped[:3],
                    *scaled_xyz,
                    *tcp_before,
                )
        return super().step(action)

    def _calc_step_reward(self, observation, is_gripper_action_effective=False):
        if not self.config.use_pose_reward:
            self._success_hold_counter = 0
            return 0.0
        return super()._calc_step_reward(observation, is_gripper_action_effective)

    def _get_observation(self):
        obs = super()._get_observation()
        if (
            self.config.invert_gripper_width_obs
            and not self.config.is_dummy
            and "state" in obs
            and "gripper_position" in obs["state"]
        ):
            raw = float(obs["state"]["gripper_position"][0])
            openness = np.clip(raw / self.config.gripper_max_width, 0.0, 1.0)
            closed_w, open_w = self.config.gripper_width_obs_range
            inverted = closed_w + (1.0 - openness) * (open_w - closed_w)
            obs["state"]["gripper_position"] = np.array(
                [inverted], dtype=np.float32
            )
        return obs

    def go_to_rest(self, joint_reset=False):
        """Open gripper, then go to rest."""
        try:
            self._controller.open_gripper().wait()
        except Exception:
            pass
        self._franka_state = self._controller.get_state().wait()[0]
        self._move_action(self._franka_state.tcp_pose)
        self._franka_state = self._controller.get_state().wait()[0]
        z_offset = float(self.config.reset_z_offset)
        if z_offset > 0:
            reset_pose = copy.deepcopy(self._franka_state.tcp_pose)
            reset_pose[2] += z_offset
            self._interpolate_move(reset_pose, timeout=1)
        FrankaEnv.go_to_rest(self, joint_reset)
