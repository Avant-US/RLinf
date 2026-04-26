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

"""Robot-state container for Galaxea R1 Pro.

Mirrors :class:`rlinf.envs.realworld.franka.franka_robot_state.FrankaRobotState`
but covers all 26 DoF (dual A2 arms x 7, dual G1 grippers, 4 DoF torso,
6 DoF chassis), plus IMU x 2, BMS, controller signal (SWA-D) and
per-arm error codes.

This dataclass is the data contract crossing the Ray RPC boundary
between :class:`GalaxeaR1ProController` (Orin) and
:class:`GalaxeaR1ProEnv` (GPU server).  All fields are plain numpy
arrays / dicts so cloudpickle can serialise them with negligible
overhead.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np


def _zeros7():
    return np.zeros(7, dtype=np.float32)


def _zeros4():
    return np.zeros(4, dtype=np.float32)


def _zeros3():
    return np.zeros(3, dtype=np.float32)


def _identity_quat():
    return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)


def _zero_xyz_quat():
    return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)


@dataclass
class GalaxeaR1ProRobotState:
    """Full robot snapshot: arms, grippers, torso, chassis, sensors.

    Layout matches the design doc (§6.3):

    * Arm joint state: ``q``, ``dq``, ``tau`` for left and right (each (7,))
    * EE pose: ``[x, y, z, qx, qy, qz, qw]`` (7-D, frame ``torso_link4``)
    * Gripper: ``pos`` (0-100 mm) and ``vel``
    * Torso: 4-DoF ``q`` and ``dq``
    * Chassis: 3-DoF ``q`` (steering angle) and ``dq`` (drive velocity)
    * IMU: dict with ``orient`` (xyzw quat), ``ang_vel`` (rad/s), ``lin_acc`` (m/s^2)
    * BMS: dict with ``voltage``, ``current``, ``capital_pct``, ``temperature``
    * Controller signal: dict with ``swa``/``swb``/``swc``/``swd`` switches
      (typically int 0/1/2 for 3-position toggle) and ``mode`` int
    * Per-arm ``status_errors``: dict ``side -> list[int]`` populated
      from ``/hdas/feedback_status_arm_*``
    * ``feedback_age_ms``: dict ``source_name -> ms`` populated by
      controller; used by SafetySupervisor as L5 watchdog
    * ``is_alive``: rolled-up health flag.  False -> SOFT_HOLD
    """

    # ── Arms ────────────────────────────────────────────────────
    left_arm_qpos: np.ndarray = field(default_factory=_zeros7)
    left_arm_qvel: np.ndarray = field(default_factory=_zeros7)
    left_arm_qtau: np.ndarray = field(default_factory=_zeros7)
    right_arm_qpos: np.ndarray = field(default_factory=_zeros7)
    right_arm_qvel: np.ndarray = field(default_factory=_zeros7)
    right_arm_qtau: np.ndarray = field(default_factory=_zeros7)

    # ── Grippers (G1: position 0-100 mm) ────────────────────────
    left_gripper_pos: float = 0.0
    left_gripper_vel: float = 0.0
    right_gripper_pos: float = 0.0
    right_gripper_vel: float = 0.0

    # ── EE poses (xyz + quat xyzw, frame torso_link4) ───────────
    left_ee_pose: np.ndarray = field(default_factory=_zero_xyz_quat)
    right_ee_pose: np.ndarray = field(default_factory=_zero_xyz_quat)

    # ── Torso (T1-T4: 4 DoF) ────────────────────────────────────
    torso_qpos: np.ndarray = field(default_factory=_zeros4)
    torso_qvel: np.ndarray = field(default_factory=_zeros4)

    # ── Chassis (3 steering wheels: angle + drive vel) ──────────
    chassis_qpos: np.ndarray = field(default_factory=_zeros3)
    chassis_qvel: np.ndarray = field(default_factory=_zeros3)

    # ── Floating base pose from mobiman eepose pub ──────────────
    floating_base_pose: np.ndarray = field(default_factory=_zero_xyz_quat)

    # ── IMU (torso & chassis) ───────────────────────────────────
    imu_torso: dict = field(
        default_factory=lambda: {
            "orient": _identity_quat(),
            "ang_vel": _zeros3(),
            "lin_acc": _zeros3(),
        }
    )
    imu_chassis: dict = field(
        default_factory=lambda: {
            "orient": _identity_quat(),
            "ang_vel": _zeros3(),
            "lin_acc": _zeros3(),
        }
    )

    # ── BMS / controller / errors / watchdog ────────────────────
    bms: dict = field(
        default_factory=lambda: {
            "voltage": 0.0,
            "current": 0.0,
            "capital_pct": 100.0,
            "temperature": 25.0,
        }
    )
    controller_signal: dict = field(
        default_factory=lambda: {
            "swa": 0, "swb": 0, "swc": 0, "swd": 0, "mode": 0,
        }
    )
    status_errors: Dict[str, List[int]] = field(default_factory=dict)

    feedback_age_ms: Dict[str, float] = field(default_factory=dict)
    is_alive: bool = False

    # ── Helpers ──────────────────────────────────────────────────

    def get_arm_qpos(self, side: str) -> np.ndarray:
        return self.right_arm_qpos if side == "right" else self.left_arm_qpos

    def get_arm_qvel(self, side: str) -> np.ndarray:
        return self.right_arm_qvel if side == "right" else self.left_arm_qvel

    def get_ee_pose(self, side: str) -> np.ndarray:
        return self.right_ee_pose if side == "right" else self.left_ee_pose

    def get_gripper_pos(self, side: str) -> float:
        return float(
            self.right_gripper_pos if side == "right" else self.left_gripper_pos
        )

    def get_state_vector(
        self,
        use_left_arm: bool = False,
        use_right_arm: bool = True,
        use_torso: bool = False,
        use_chassis: bool = False,
        include_grippers: bool = True,
        include_ee: bool = True,
    ) -> np.ndarray:
        """Flatten subset of state into a 1-D vector.

        Order is deterministic: right_qpos -> right_ee -> right_gripper
        -> left_qpos -> left_ee -> left_gripper -> torso -> chassis.
        """
        parts: list[np.ndarray] = []
        if use_right_arm:
            parts.append(self.right_arm_qpos.astype(np.float32))
            if include_ee:
                parts.append(self.right_ee_pose.astype(np.float32))
            if include_grippers:
                parts.append(np.array(
                    [self.right_gripper_pos / 100.0], dtype=np.float32,
                ))
        if use_left_arm:
            parts.append(self.left_arm_qpos.astype(np.float32))
            if include_ee:
                parts.append(self.left_ee_pose.astype(np.float32))
            if include_grippers:
                parts.append(np.array(
                    [self.left_gripper_pos / 100.0], dtype=np.float32,
                ))
        if use_torso:
            parts.append(self.torso_qpos.astype(np.float32))
        if use_chassis:
            parts.append(np.concatenate(
                [self.chassis_qpos, self.chassis_qvel],
            ).astype(np.float32))
        if not parts:
            return np.zeros(0, dtype=np.float32)
        return np.concatenate(parts).astype(np.float32)

    def to_dict(self) -> dict:
        """Return a *picklable* dict suitable as observation['state']."""
        return {
            "left_arm_qpos": self.left_arm_qpos.copy(),
            "left_arm_qvel": self.left_arm_qvel.copy(),
            "right_arm_qpos": self.right_arm_qpos.copy(),
            "right_arm_qvel": self.right_arm_qvel.copy(),
            "left_gripper_pos": np.array([self.left_gripper_pos], dtype=np.float32),
            "right_gripper_pos": np.array([self.right_gripper_pos], dtype=np.float32),
            "left_ee_pose": self.left_ee_pose.copy(),
            "right_ee_pose": self.right_ee_pose.copy(),
            "torso_qpos": self.torso_qpos.copy(),
            "chassis_qpos": self.chassis_qpos.copy(),
            "chassis_qvel": self.chassis_qvel.copy(),
            "floating_base_pose": self.floating_base_pose.copy(),
        }

    def copy(self) -> "GalaxeaR1ProRobotState":
        return copy.deepcopy(self)
