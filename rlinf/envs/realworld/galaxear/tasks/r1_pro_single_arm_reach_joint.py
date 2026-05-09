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

"""M1 task: right-arm reach to a fixed JOINT target (joint mode).

Per design doc r1pro6op47.md §10.2, this is the very first real-robot
RL task we recommend running on R1 Pro.  It deliberately:

* Uses the **joint** action mode so the policy outputs 7 absolute
  joint angles directly to ``/motion_target/target_joint_state_arm_*``
  (no IK chain dependency, no relaxed_ik bring-up risk).
* Has **no gripper** and **no camera** (proprio-only); reward is
  purely a function of ``q - q_target``.
* Has **no left arm**, **no torso**, **no chassis**.

That triple-zero dependency profile makes it the perfect smoke-test
for "is the entire stack actually wired correctly?".
"""

from __future__ import annotations

from typing import Any

import numpy as np

from rlinf.envs.realworld.galaxear.r1_pro_env import (
    GalaxeaR1ProEnv,
)


class GalaxeaR1ProSingleArmReachJointEnv(GalaxeaR1ProEnv):
    """Right-arm reach to ``target_q_right`` in joint space.

    Reward: ``r = -||q_current - q_target||_2`` plus a sparse +1.0
    bonus when the L2 distance falls below ``joint_tolerance_rad``
    (default 0.05 rad).
    """

    DEFAULT_TARGET_Q_RIGHT = (
        0.5,
        0.5,
        0.0,
        -1.2,
        0.0,
        1.5,
        0.0,
    )
    DEFAULT_JOINT_TOLERANCE_RAD = 0.05

    def __init__(
        self,
        override_cfg: dict[str, Any],
        worker_info=None,
        hardware_info=None,
        env_idx: int = 0,
    ) -> None:
        # Force the config into the new dispatcher path: joint mode,
        # right arm only, no gripper.  Users may still override
        # ``target_q_right`` and ``joint_tolerance_rad`` from YAML.
        cfg = dict(override_cfg or {})
        cfg.setdefault("use_joint_mode", True)
        cfg.setdefault("use_right_arm", True)
        cfg.setdefault("use_left_arm", False)
        cfg.setdefault("no_gripper", True)
        cfg.setdefault("use_torso", False)
        cfg.setdefault("use_chassis", False)
        # Pull task-specific keys out so GalaxeaR1ProRobotConfig doesn't
        # complain about unknown fields.
        self._target_q_right = np.asarray(
            cfg.pop("target_q_right", self.DEFAULT_TARGET_Q_RIGHT),
            dtype=np.float32,
        ).reshape(7)
        self._joint_tolerance = float(
            cfg.pop("joint_tolerance_rad", self.DEFAULT_JOINT_TOLERANCE_RAD)
        )
        super().__init__(
            override_cfg=cfg,
            worker_info=worker_info,
            hardware_info=hardware_info,
            env_idx=env_idx,
        )

    @property
    def task_description(self) -> str:
        return (
            "Move the right arm to the configured target joint angles "
            "and hold for ``success_hold_steps`` steps.  Joint-mode "
            "M1 bring-up task per r1pro6op47.md §10.2."
        )

    def _calc_step_reward(self, obs, sinfo) -> float:
        """Override the legacy EE-distance reward with a joint-distance
        reward.  Reward is dense and bounded:

        ``r = -||q - q_target||_2 / sqrt(7)`` so it lies in
        ``[-pi*sqrt(7), 0]``-ish; plus +1.0 when ``||.|| <
        joint_tolerance_rad``.
        """
        q = np.asarray(self._state.right_arm_qpos, dtype=np.float32).reshape(7)
        diff = q - self._target_q_right
        l2 = float(np.linalg.norm(diff))
        # Dense shaping (negative -> positive as we get closer).
        dense = -l2 / np.sqrt(7.0)
        # Sparse success bonus.
        bonus = 1.0 if l2 < self._joint_tolerance else 0.0
        if bonus > 0.0:
            self._success_hold_counter = self._success_hold_counter + 1
        else:
            self._success_hold_counter = 0
        return float(dense + bonus)
