"""Single-arm Franka env using FrankyControllerExtended (local extension)."""

from __future__ import annotations

import copy
import os
from typing import Any

import numpy as np

from rlinf.envs.realworld.franka.franka_env import FrankaEnv, FrankaRobotConfig
from rlinf.envs.realworld.franka.end_effectors.base import normalize_end_effector_type
from rlinf.scheduler import FrankaHWInfo

from franky_ext.controller_extended import FrankyControllerExtended


def _skip_camera() -> bool:
    return os.environ.get("RLINF_SKIP_CAMERA", "0").lower() in ("1", "true", "yes")


class FrankySingleFrankaEnvMixin:
    """Replace ROS FrankaController with FrankyControllerExtended."""

    def _setup_hardware(self):
        assert self.env_idx >= 0, "env_idx must be set for FrankaEnv."
        assert isinstance(self.hardware_info, FrankaHWInfo), (
            f"hardware_info must be FrankaHWInfo, got {type(self.hardware_info)}."
        )
        if self.config.robot_ip is None:
            self.config.robot_ip = self.hardware_info.config.robot_ip
        if self.config.camera_serials is None:
            self.config.camera_serials = self.hardware_info.config.camera_serials
        if self.config.camera_type is None:
            self.config.camera_type = getattr(
                self.hardware_info.config, "camera_type", "realsense"
            )
        if self.config.gripper_type is None:
            self.config.gripper_type = getattr(
                self.hardware_info.config, "gripper_type", "franka"
            )
        if self.config.gripper_connection is None:
            self.config.gripper_connection = getattr(
                self.hardware_info.config, "gripper_connection", None
            )
        self.config.end_effector_type = normalize_end_effector_type(
            self.config.end_effector_type,
            self.config.gripper_type,
        ).value

        controller_node_rank = getattr(
            self.hardware_info.config, "controller_node_rank", None
        )
        if controller_node_rank is None:
            controller_node_rank = self.node_rank

        self._controller = FrankyControllerExtended.launch_controller(
            robot_ip=self.config.robot_ip,
            env_idx=self.env_idx,
            node_rank=controller_node_rank,
            worker_rank=self.env_worker_rank,
            gripper_type=self.config.gripper_type or "franka",
            gripper_connection=self.config.gripper_connection,
        )

    def _open_cameras(self):
        if _skip_camera():
            self._cameras = []
            return
        super()._open_cameras()  # type: ignore[misc]

    def _get_camera_frames(self) -> dict[str, np.ndarray]:
        if _skip_camera():
            frames = {}
            for name, space in self.observation_space["frames"].spaces.items():
                h, w, c = space.shape
                frames[name] = np.zeros((h, w, c), dtype=np.uint8)
            return frames
        return super()._get_camera_frames()  # type: ignore[misc]


class FrankySingleFrankaEnvConfig(FrankaRobotConfig):
    pass


class FrankySingleFrankaEnv(FrankySingleFrankaEnvMixin, FrankaEnv):
    CONFIG_CLS = FrankySingleFrankaEnvConfig
