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

    # ── Joint prediction (used by SafetySupervisor L2 per-joint check) ──

    def predict_arm_qpos(
        self,
        side: str,
        action: np.ndarray,
        state: "GalaxeaR1ProRobotState",
        *,
        inv_jacobian_fn=None,
    ) -> Optional[np.ndarray]:
        """Predict the joint positions for *side* arm after this step.

        Strategy (in order of preference):

        1. ``inv_jacobian_fn`` provided -> use IK to map the predicted
           Cartesian EE delta to a joint delta (precise, requires
           URDF + an IK library; future hook).
        2. ``self.use_joint_mode == True`` -> action *is* a normalised
           joint delta; just add it to the current qpos.
        3. otherwise (Cartesian EE delta mode, the default) -> return
           ``state.get_arm_qpos(side)`` *unchanged*.  L2a is then
           effectively a no-op for arms in EE mode -- the actual
           protection comes from L2b (post-hoc qpos freeze) and L2c
           (qvel feedback watchdog).  This is intentional and keeps
           the L2a hook future-proof (drop in an IK-enabled
           ``inv_jacobian_fn`` to start using it without touching
           the supervisor).

        Args:
            side: ``"right"`` or ``"left"``.
            action: Normalised flat action vector ``[-1, 1]^D``.
            state: Latest robot snapshot.
            inv_jacobian_fn: Optional callable
                ``fn(*, side, cur_q, d_xyz, d_rpy) -> np.ndarray(7,)``
                returning a per-joint delta (rad).

        Returns:
            ``np.ndarray`` of shape ``(7,)`` with predicted joint
            angles, or ``None`` when the corresponding arm is not
            enabled in the schema.
        """
        d = self.split(action)
        key_xyz = f"{side}_xyz"
        if key_xyz not in d:
            return None
        cur_q = np.asarray(
            state.get_arm_qpos(side), dtype=np.float32,
        ).reshape(-1)[:7]
        if cur_q.size != 7:
            return None

        # Branch 1: external IK predictor supplied.
        if inv_jacobian_fn is not None:
            try:
                delta_q = inv_jacobian_fn(
                    side=side,
                    cur_q=cur_q,
                    d_xyz=np.asarray(d[key_xyz], dtype=np.float32)
                    * float(self.action_scale[0]),
                    d_rpy=np.asarray(d[f"{side}_rpy"], dtype=np.float32)
                    * float(self.action_scale[1]),
                )
                delta_q = np.asarray(delta_q, dtype=np.float32).reshape(-1)[:7]
                if delta_q.size == 7:
                    return (cur_q + delta_q).astype(np.float32)
            except Exception:
                pass  # fall through to other branches

        # Branch 2: action is already in joint space.
        if self.use_joint_mode:
            xyz = np.asarray(d[key_xyz], dtype=np.float32).reshape(-1)[:3]
            rpy = np.asarray(d[f"{side}_rpy"], dtype=np.float32).reshape(-1)[:3]
            # In a joint-tracker stage the per-arm 6 dims are the first 6
            # joint deltas; the schema uses xyz+rpy slots as a transport
            # for those joint deltas (bring-up convention; will be
            # generalised when real joint-mode actions land).
            joint_delta6 = np.concatenate([xyz, rpy])
            scale = float(self.action_scale[0])
            new_q = cur_q.copy()
            new_q[:6] = cur_q[:6] + joint_delta6 * scale
            return new_q.astype(np.float32)

        # Branch 3: Cartesian mode -> return current qpos as a "no-op"
        # predictor; L2a will be a no-op and L2b/L2c will do the heavy
        # lifting via reactive feedback checks.
        return cur_q.astype(np.float32)

    # ── Action rewriters (used by SafetySupervisor L2 to scale-shrink) ──

    def rewrite_action_arm(
        self,
        action: np.ndarray,
        side: str,
        scale_factor: float,
    ) -> np.ndarray:
        """Multiply the per-arm xyz+rpy slice of *action* by *scale_factor*.

        Used by L2a / L2b / L2c to softly shrink the policy's intent
        without changing direction.  ``scale_factor=0.0`` freezes the
        arm in place.  The gripper dim (if any) is *not* scaled --
        gripper has its own L2 sub-check via :meth:`set_gripper_action`.

        Returns a new (modified) action vector.  Operates on a copy
        so callers may keep the raw_action snapshot intact.
        """
        a = np.asarray(action, dtype=np.float32).reshape(-1).copy()
        s = float(np.clip(scale_factor, 0.0, 1.0))
        idx = 0
        if self.has_right_arm:
            if side == "right":
                a[idx : idx + 6] *= s
                return a
            idx += self.per_arm_dim
        if self.has_left_arm and side == "left":
            a[idx : idx + 6] *= s
        return a

    def rewrite_action_torso(
        self,
        action: np.ndarray,
        scale_factor: float,
    ) -> np.ndarray:
        """Scale the torso 4-D twist slice.  No-op when ``has_torso`` is False."""
        a = np.asarray(action, dtype=np.float32).reshape(-1).copy()
        if not self.has_torso:
            return a
        s = float(np.clip(scale_factor, 0.0, 1.0))
        idx = 0
        if self.has_right_arm:
            idx += self.per_arm_dim
        if self.has_left_arm:
            idx += self.per_arm_dim
        a[idx : idx + 4] *= s
        return a

    def rewrite_action_chassis(
        self,
        action: np.ndarray,
        scale_factor: float,
    ) -> np.ndarray:
        """Scale the chassis 3-D twist slice.  No-op when ``has_chassis`` is False."""
        a = np.asarray(action, dtype=np.float32).reshape(-1).copy()
        if not self.has_chassis:
            return a
        s = float(np.clip(scale_factor, 0.0, 1.0))
        idx = 0
        if self.has_right_arm:
            idx += self.per_arm_dim
        if self.has_left_arm:
            idx += self.per_arm_dim
        if self.has_torso:
            idx += 4
        a[idx : idx + 3] *= s
        return a

    def rewrite_action_chassis_set(
        self,
        action: np.ndarray,
        new_twist3: np.ndarray,
    ) -> np.ndarray:
        """Replace the chassis 3-D twist slice with absolute *new_twist3*.

        Used by L2d when the dead-zone or per-axis cap demands a
        non-proportional change.  No-op when ``has_chassis`` is False.
        """
        a = np.asarray(action, dtype=np.float32).reshape(-1).copy()
        if not self.has_chassis:
            return a
        v = np.asarray(new_twist3, dtype=np.float32).reshape(-1)[:3]
        if v.size < 3:
            v = np.concatenate(
                [v, np.zeros(3 - v.size, dtype=np.float32)],
            )
        idx = 0
        if self.has_right_arm:
            idx += self.per_arm_dim
        if self.has_left_arm:
            idx += self.per_arm_dim
        if self.has_torso:
            idx += 4
        a[idx : idx + 3] = v.astype(np.float32)
        return a

    def set_gripper_action(
        self,
        action: np.ndarray,
        side: str,
        new_normalised_value: float,
    ) -> np.ndarray:
        """Replace the gripper dim of *action* with *new_normalised_value*.

        ``new_normalised_value`` is the post-rate-limit / post-clamp
        action value (in ``[-1, 1]``).  No-op when ``no_gripper`` is
        True or the corresponding arm is not enabled.

        Returns a new (modified) action vector.
        """
        a = np.asarray(action, dtype=np.float32).reshape(-1).copy()
        if self.no_gripper:
            return a
        idx = 0
        if self.has_right_arm:
            if side == "right":
                a[idx + 6] = float(new_normalised_value)
                return a
            idx += self.per_arm_dim
        if self.has_left_arm and side == "left":
            a[idx + 6] = float(new_normalised_value)
        return a

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
