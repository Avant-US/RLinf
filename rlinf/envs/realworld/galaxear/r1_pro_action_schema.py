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

"""Action-schema abstraction for stage-dependent action vectors.

Per the design doc (§8), the policy action dimension changes by stage:

* M1 single arm:              7  = ``[dx dy dz drx dry drz gripper]``
* M2 dual arm:                14 = right(7) + left(7)
* M3 whole body (+ torso):    18 = M2 + ``[v_x v_z w_pitch w_yaw]``
* M4 mobile manipulation:     21 = M3 + ``[v_x v_y w_z]`` (chassis)

The :class:`ActionSchema` hides this from :class:`GalaxeaR1ProEnv`
and :class:`GalaxeaR1ProSafetySupervisor`: both consume a flat
normalised vector in ``[-1, 1]^D`` and let the schema do the
splitting / scaling / EE-target prediction / dispatch to controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import numpy as np

if TYPE_CHECKING:
    from .r1_pro_robot_state import GalaxeaR1ProRobotState


@dataclass
class ActionSchema:
    """Stage-dependent action layout + execution.

    Attributes:
        has_left_arm / has_right_arm: Stage flags.
        has_torso / has_chassis: Stage flags.
        no_gripper: When True, the gripper dim is dropped (per arm).
        action_scale: ``[pos_scale, ori_scale, gripper_scale]`` the
            policy output is multiplied by before being applied as a
            delta on top of the current EE pose.
        use_joint_mode: Reserved; ``True`` switches arm publishing
            from mobiman pose mode to joint tracker.  Not yet exposed
            beyond the controller skeleton.
    """

    has_left_arm: bool
    has_right_arm: bool
    has_torso: bool
    has_chassis: bool
    no_gripper: bool
    action_scale: np.ndarray
    use_joint_mode: bool = False

    @property
    def per_arm_dim(self) -> int:
        return 6 if self.no_gripper else 7

    @property
    def action_dim(self) -> int:
        d = 0
        if self.has_right_arm:
            d += self.per_arm_dim
        if self.has_left_arm:
            d += self.per_arm_dim
        if self.has_torso:
            d += 4
        if self.has_chassis:
            d += 3
        return max(d, 1)

    # ── Splitting & prediction ──────────────────────────────────

    def split(self, action: np.ndarray) -> dict:
        """Split a flat action vector into named per-component slices.

        Returns a dict that may contain any of:
        ``right_xyz`` (3,), ``right_rpy`` (3,), ``right_gripper`` (float),
        ``left_xyz`` (3,), ``left_rpy`` (3,), ``left_gripper`` (float),
        ``torso_twist`` (4,), ``chassis_twist`` (3,).
        """
        out: dict = {}
        idx = 0
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if self.has_right_arm:
            out["right_xyz"] = action[idx : idx + 3]
            out["right_rpy"] = action[idx + 3 : idx + 6]
            idx += 6
            if not self.no_gripper:
                out["right_gripper"] = float(action[idx])
                idx += 1
        if self.has_left_arm:
            out["left_xyz"] = action[idx : idx + 3]
            out["left_rpy"] = action[idx + 3 : idx + 6]
            idx += 6
            if not self.no_gripper:
                out["left_gripper"] = float(action[idx])
                idx += 1
        if self.has_torso:
            out["torso_twist"] = action[idx : idx + 4]
            idx += 4
        if self.has_chassis:
            out["chassis_twist"] = action[idx : idx + 3]
            idx += 3
        return out

    # ── EE prediction (used by SafetySupervisor for L3 box clip) ─

    def predict_arm_ee_pose(
        self,
        side: str,
        action: np.ndarray,
        state: "GalaxeaR1ProRobotState",
    ) -> Optional[np.ndarray]:
        """Predict the EE target ``[x y z roll pitch yaw]`` after step.

        Returns ``None`` when the corresponding arm is not enabled.
        """
        from scipy.spatial.transform import Rotation as R

        d = self.split(action)
        key_xyz = f"{side}_xyz"
        key_rpy = f"{side}_rpy"
        if key_xyz not in d:
            return None
        ee = state.get_ee_pose(side)
        cur_xyz = ee[:3].astype(np.float32)
        cur_eul = R.from_quat(np.asarray(ee[3:], dtype=np.float64)).as_euler("xyz")
        nxt_xyz = cur_xyz + d[key_xyz] * float(self.action_scale[0])
        nxt_eul = cur_eul + d[key_rpy] * float(self.action_scale[1])
        return np.concatenate([nxt_xyz, nxt_eul]).astype(np.float32)

    @property
    def has_dual_arms(self) -> bool:
        return self.has_left_arm and self.has_right_arm


def build_action_schema(cfg) -> ActionSchema:
    """Construct the schema from a :class:`GalaxeaR1ProRobotConfig`."""
    action_scale = np.asarray(getattr(cfg, "action_scale", [0.05, 0.10, 1.0]),
                              dtype=np.float32).reshape(-1)
    if action_scale.size < 3:
        action_scale = np.array([0.05, 0.10, 1.0], dtype=np.float32)
    return ActionSchema(
        has_left_arm=bool(cfg.use_left_arm),
        has_right_arm=bool(cfg.use_right_arm),
        has_torso=bool(getattr(cfg, "use_torso", False)),
        has_chassis=bool(getattr(cfg, "use_chassis", False)),
        no_gripper=bool(getattr(cfg, "no_gripper", False)),
        action_scale=action_scale,
        use_joint_mode=getattr(cfg, "mobiman_launch_mode", "pose") == "joint",
    )
