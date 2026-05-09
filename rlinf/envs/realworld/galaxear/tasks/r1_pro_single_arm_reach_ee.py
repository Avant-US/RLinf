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

"""M2 task: right-arm reach to a fixed EE pose target (ee+quat mode).

Per design doc r1pro6op47.md §10.3, this is the M2 follow-up to
``GalaxeaR1ProSingleArmReachJointEnv``.  It uses the new EE+quaternion
dispatcher path (no RPY) and depends on mobiman ``relaxed_ik`` being
launched.

Reward: ``r = -dist(xyz, xyz_target) - 0.5 * quat_distance + 1.0 on success``.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from rlinf.envs.realworld.galaxear.r1_pro_env import GalaxeaR1ProEnv


class GalaxeaR1ProSingleArmReachEeEnv(GalaxeaR1ProEnv):
    """Right-arm reach to ``target_pose_right`` ([x y z qx qy qz qw])."""

    DEFAULT_TARGET_POSE_RIGHT = (
        0.45,
        -0.10,
        0.30,
        0.0,
        0.0,
        0.0,
        1.0,
    )
    DEFAULT_XYZ_TOLERANCE_M = 0.03
    DEFAULT_QUAT_DOT_THRESHOLD = 0.95  # cos(angle/2) -> ~36 deg

    def __init__(
        self,
        override_cfg: dict[str, Any],
        worker_info=None,
        hardware_info=None,
        env_idx: int = 0,
    ) -> None:
        cfg = dict(override_cfg or {})
        cfg.setdefault("use_joint_mode", False)
        cfg.setdefault("use_new_dispatcher", True)
        cfg.setdefault("use_right_arm", True)
        cfg.setdefault("use_left_arm", False)
        cfg.setdefault("no_gripper", True)
        cfg.setdefault("use_torso", False)
        cfg.setdefault("use_chassis", False)
        self._target_pose_right = np.asarray(
            cfg.pop("target_pose_right", self.DEFAULT_TARGET_POSE_RIGHT),
            dtype=np.float32,
        ).reshape(7)
        self._xyz_tolerance = float(
            cfg.pop("xyz_tolerance_m", self.DEFAULT_XYZ_TOLERANCE_M)
        )
        self._quat_dot_threshold = float(
            cfg.pop("quat_dot_threshold", self.DEFAULT_QUAT_DOT_THRESHOLD)
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
            "Move the right end-effector to the configured target pose "
            "(xyz + quat) and hold for ``success_hold_steps`` steps.  "
            "EE+quat mode M2 task per r1pro6op47.md §10.3."
        )

    def _calc_step_reward(self, obs, sinfo) -> float:
        ee = np.asarray(self._state.right_ee_pose, dtype=np.float32).reshape(7)
        cur_xyz = ee[:3]
        cur_quat = ee[3:]
        d_xyz = float(np.linalg.norm(cur_xyz - self._target_pose_right[:3]))
        # Quaternion similarity: |dot| in [0, 1], 1 means identical
        # rotation (handles q vs -q symmetry).
        q_dot = float(np.abs(np.dot(cur_quat, self._target_pose_right[3:])))
        dense = -d_xyz - 0.5 * (1.0 - q_dot)
        success = d_xyz < self._xyz_tolerance and q_dot > self._quat_dot_threshold
        bonus = 1.0 if success else 0.0
        if bonus > 0.0:
            self._success_hold_counter += 1
        else:
            self._success_hold_counter = 0
        return float(dense + bonus)
