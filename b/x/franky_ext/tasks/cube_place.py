"""Franky-backed cube-touch env: gripper stays closed, success is TCP at mark."""

from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np

from franky_ext.franky_single_franka_env import FrankySingleFrankaEnvMixin
from rlinf.envs.realworld.franka.franka_env import FrankaEnv
from rlinf.envs.realworld.franka.tasks.peg_insertion_env import (
    PegInsertionConfig,
    PegInsertionEnv,
)


@dataclass
class CubePlaceConfig(PegInsertionConfig):
    """Defaults from dmo_place_1.md §5: closed-gripper touch, not release."""

    task_description: str = "touch the marked place with a grasped cube"
    clip_x_range: float = 0.05
    clip_y_range: float = 0.05
    clip_z_range_low: float = 0.005
    clip_z_range_high: float = 0.08
    random_xy_range: float = 0.03
    random_rz_range: float = 0.35
    clip_rz_range: float = 0.35
    safe_smoke_hold: bool = False
    # Same as PegInsertionEnv.go_to_rest: lift relative to *current* TCP, not target.
    reset_z_lift_m: float = 0.10


class FrankyCubePlaceEnv(FrankySingleFrankaEnvMixin, PegInsertionEnv):
    """Closed-gripper cube touch on Franky (PegInsertion-style lift, not open)."""

    CONFIG_CLS = CubePlaceConfig

    def go_to_rest(self, joint_reset=False):
        """PegInsertionEnv / charger sequence, gripper stays closed.

        1. Close gripper (``-1``).
        2. ``_move_action(current)`` so cartesian impedance tracks *here*
           (ROS publishes equilibrium; franky starts the tracker after
           ``reconfigure_compliance_params`` stopped it).
        3. Lift ``reset_z_lift_m`` (default 0.10 m) along **current** TCP z,
           same pose — this is not ``target + clip_z_range_high``.
        4. ``FrankaEnv.go_to_rest`` interpolates to ``reset_ee_pose``
           (target + ``clip_z_range_high``).
        """
        self._end_effector_action(np.array([-1.0]))
        self._franka_state = self._controller.get_state().wait()[0]
        self._move_action(self._franka_state.tcp_pose)
        self._franka_state = self._controller.get_state().wait()[0]
        before = np.asarray(self._franka_state.tcp_pose[:3], dtype=np.float64)
        reset_pose = copy.deepcopy(self._franka_state.tcp_pose)
        z_lift = float(getattr(self.config, "reset_z_lift_m", 0.10))
        reset_pose[2] += z_lift
        self._logger.info(
            "cube_place go_to_rest: current %s +z=%.3f -> %s; then rest target+clip_z_high",
            np.round(before, 4).tolist(),
            z_lift,
            np.round(reset_pose[:3], 4).tolist(),
        )
        self._interpolate_move(reset_pose, timeout=1)
        FrankaEnv.go_to_rest(self, joint_reset)
        self._franka_state = self._controller.get_state().wait()[0]
        after = np.asarray(self._franka_state.tcp_pose[:3], dtype=np.float64)
        self._logger.info(
            "cube_place go_to_rest done: tcp_xyz=%s dz=%.4f",
            np.round(after, 4).tolist(),
            float(after[2] - before[2]),
        )
