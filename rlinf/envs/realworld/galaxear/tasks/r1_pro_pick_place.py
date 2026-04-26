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

"""M1 main task: right-arm tabletop pick-place with phased reward.

Four-phase reward shaping:
* ``approach`` -- minimise distance from EE to ``pick_position``.
* ``grasp`` -- close the gripper at the pick site.
* ``lift`` -- raise to ``place_position[2]``.
* ``place`` -- minimise distance from EE to ``place_position`` and
  open the gripper.

Subclasses :class:`GalaxeaR1ProEnv` and overrides ``_calc_step_reward``
+ ``reset`` to track the active phase.

Phase progression is *strictly monotone*: once advanced, the phase
does not regress within an episode.  Phase advancement awards a +1
shaped bonus; final ``place`` success returns 1.0 to terminate.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

from rlinf.envs.realworld.galaxear.r1_pro_env import (
    GalaxeaR1ProEnv,
    GalaxeaR1ProRobotConfig,
)


@dataclass
class GalaxeaR1ProPickPlaceConfig(GalaxeaR1ProRobotConfig):
    """Pick-place specific config."""

    pick_position: list = field(default_factory=lambda: [0.45, -0.10, 0.18])
    place_position: list = field(default_factory=lambda: [0.45, 0.10, 0.30])
    object_width_m: float = 0.05
    approach_threshold_m: float = 0.04
    grasp_close_threshold_pct: float = 30.0
    lift_height_m: float = 0.05
    place_threshold_m: float = 0.04


class GalaxeaR1ProPickPlaceEnv(GalaxeaR1ProEnv):
    """Pick-and-place task with 4-phase shaped reward."""

    CONFIG_CLS: type[GalaxeaR1ProPickPlaceConfig] = (
        GalaxeaR1ProPickPlaceConfig
    )
    PHASES = ("approach", "grasp", "lift", "place")

    def __init__(
        self,
        override_cfg: dict[str, Any],
        worker_info: Optional[Any] = None,
        hardware_info: Optional[Any] = None,
        env_idx: int = 0,
    ) -> None:
        super().__init__(override_cfg, worker_info, hardware_info, env_idx)
        self._phase: str = "approach"

    @property
    def task_description(self) -> str:
        return (
            "Pick the cube from `pick_position` and place it at "
            "`place_position` with the right arm."
        )

    def reset(self, *, seed=None, options=None):
        self._phase = "approach"
        return super().reset(seed=seed, options=options)

    # Ordered phase index for monotonicity.
    def _phase_index(self, name: str) -> int:
        try:
            return self.PHASES.index(name)
        except ValueError:
            return -1

    def _advance_to(self, phase: str) -> bool:
        if self._phase_index(phase) > self._phase_index(self._phase):
            self._phase = phase
            return True
        return False

    def _calc_step_reward(self, obs, sinfo) -> float:
        if self.config.is_dummy:
            # In dummy mode return a small constant so the run completes.
            return 0.0
        cfg: GalaxeaR1ProPickPlaceConfig = self.config  # type: ignore[assignment]
        if self.config.use_reward_model:
            return self._compute_reward_model(obs)

        st = self._state
        ee_xyz = st.right_ee_pose[:3]
        gripper_pct = float(st.right_gripper_pos)
        pick = np.asarray(cfg.pick_position, dtype=np.float32)
        place = np.asarray(cfg.place_position, dtype=np.float32)

        reward = 0.0
        if self._phase == "approach":
            d = float(np.linalg.norm(ee_xyz - pick))
            if d < cfg.approach_threshold_m:
                self._advance_to("grasp")
                reward = 1.0
            elif cfg.use_dense_reward:
                reward = float(np.exp(-50.0 * d * d) * 0.5)

        elif self._phase == "grasp":
            if gripper_pct < cfg.grasp_close_threshold_pct:
                self._advance_to("lift")
                reward = 2.0

        elif self._phase == "lift":
            if ee_xyz[2] >= (pick[2] + cfg.lift_height_m):
                self._advance_to("place")
                reward = 3.0
            elif cfg.use_dense_reward:
                lift_progress = float(
                    (ee_xyz[2] - pick[2]) / max(cfg.lift_height_m, 1e-6)
                )
                reward = float(np.clip(lift_progress, 0.0, 1.0)) * 0.5

        elif self._phase == "place":
            d = float(np.linalg.norm(ee_xyz - place))
            close_enough = d < cfg.place_threshold_m
            opened = gripper_pct > cfg.grasp_close_threshold_pct
            if close_enough and opened:
                self._success_hold_counter += 1
                return 1.0
            self._success_hold_counter = 0
            if cfg.use_dense_reward:
                reward = float(np.exp(-50.0 * d * d) * 0.8)

        return reward * float(cfg.reward_scale)
