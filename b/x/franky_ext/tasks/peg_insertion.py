"""Franky-backed PegInsertion env with calibrated safety defaults."""

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
class FrankyPegInsertionEnvConfig(PegInsertionConfig):
    """Peg insertion config with Franky smoke / safety knobs."""

    safe_smoke_hold: bool = False
    reset_z_lift_m: float = 0.05
    safety_box_half_width_m: float = 0.05

    def __post_init__(self):
        half = float(self.safety_box_half_width_m)
        z_hover = float(self.reset_z_lift_m)
        self.clip_x_range = half
        self.clip_y_range = half
        self.clip_z_range_low = 0.0
        self.clip_z_range_high = z_hover
        self.random_xy_range = min(float(self.random_xy_range), half)
        super().__post_init__()


class FrankyPegInsertionEnv(FrankySingleFrankaEnvMixin, PegInsertionEnv):
    """Peg insertion on Franky with 5 cm lift / safety box defaults."""

    CONFIG_CLS = FrankyPegInsertionEnvConfig

    def go_to_rest(self, joint_reset=False):
        """Lift by ``reset_z_lift_m`` (default 5 cm) before Peg rest interpolation."""
        z_lift = float(getattr(self.config, "reset_z_lift_m", 0.05))
        self._end_effector_action(np.array([-1.0]))
        self._franka_state = self._controller.get_state().wait()[0]
        self._move_action(self._franka_state.tcp_pose)
        self._franka_state = self._controller.get_state().wait()[0]
        reset_pose = copy.deepcopy(self._franka_state.tcp_pose)
        reset_pose[2] += z_lift
        self._interpolate_move(reset_pose, timeout=1)
        FrankaEnv.go_to_rest(self, joint_reset)
