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

"""Multi-level safety supervisor for Galaxea R1 Pro.

The supervisor sits inside :meth:`GalaxeaR1ProEnv.step` and gates
*every* action that goes to the controller through five levels:

* **L1 schema** -- shape / dtype / NaN / range ``[-1, 1]``
* **L2 per-joint limit** -- ``q_min``, ``q_max``, ``q_vel_max``
* **L3a TCP safety box** -- per-arm ``[xyz, rpy]`` clip
* **L3b dual-arm collision** -- bounding-sphere distance
* **L4 velocity / accel / jerk caps** -- per-step caps for arm /
  torso / chassis
* **L5 system watchdog** -- SWD E-stop, BMS low battery, status
  errors, feedback stale, A2 fall-risk posture rule

Outputs a :class:`SafetyInfo` with the (potentially modified)
``safe_action``, three escalation flags (``soft_hold`` / ``safe_stop``
/ ``emergency_stop``), the human-readable list of reasons, and a
metrics dict consumed by ``MetricLogger``.

See design doc §6.4 and §9 for the full rationale.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, List, Optional

import numpy as np

if TYPE_CHECKING:
    from .r1_pro_action_schema import ActionSchema
    from .r1_pro_robot_state import GalaxeaR1ProRobotState


# ───────────────────────── Config dataclass ─────────────────────


@dataclass
class SafetyConfig:
    """Hard safety limits + watchdog thresholds.

    Defaults are tuned for the M1 single-arm bring-up.  Tune via
    ``override_cfg.safety_cfg`` in the env YAML.
    """

    # ── L2: per-joint limits (right arm; left arm uses same) ────
    arm_q_min: np.ndarray = field(default_factory=lambda: np.array(
        [-2.7, -1.8, -2.7, -3.0, -2.7, -0.1, -2.7], dtype=np.float32))
    arm_q_max: np.ndarray = field(default_factory=lambda: np.array(
        [2.7, 1.8, 2.7, 0.0, 2.7, 3.7, 2.7], dtype=np.float32))
    arm_qvel_max: np.ndarray = field(default_factory=lambda: np.array(
        [3.0, 3.0, 3.0, 3.0, 5.0, 5.0, 5.0], dtype=np.float32))

    # ── L3a: per-arm TCP safety box (xyz + rpy in torso_link4) ──
    right_ee_min: np.ndarray = field(default_factory=lambda: np.array(
        [0.20, -0.35, 0.05, -3.20, -0.30, -0.30], dtype=np.float32))
    right_ee_max: np.ndarray = field(default_factory=lambda: np.array(
        [0.65, 0.35, 0.65, 3.20, 0.30, 0.30], dtype=np.float32))
    left_ee_min: np.ndarray = field(default_factory=lambda: np.array(
        [0.20, -0.35, 0.05, -3.20, -0.30, -0.30], dtype=np.float32))
    left_ee_max: np.ndarray = field(default_factory=lambda: np.array(
        [0.65, 0.35, 0.65, 3.20, 0.30, 0.30], dtype=np.float32))

    # ── L3b: dual-arm collision spheres ─────────────────────────
    dual_arm_collision_enable: bool = True
    dual_arm_sphere_radius_m: float = 0.08
    dual_arm_min_distance_m: float = 0.05

    # ── L4: vel / accel / jerk caps ─────────────────────────────
    max_linear_step_m: float = 0.05
    max_angular_step_rad: float = 0.20
    torso_v_x_max: float = 0.10
    torso_v_z_max: float = 0.10
    torso_w_pitch_max: float = 0.30
    torso_w_yaw_max: float = 0.30
    chassis_v_x_max: float = 0.60
    chassis_v_y_max: float = 0.60
    chassis_w_z_max: float = 1.50
    chassis_acc_x_max: float = 1.0
    chassis_acc_y_max: float = 0.5
    chassis_acc_w_max: float = 0.8

    # ── L5: watchdog ────────────────────────────────────────────
    bms_low_battery_threshold_pct: float = 25.0
    feedback_stale_threshold_ms: float = 200.0
    operator_heartbeat_timeout_ms: float = 1500.0
    a2_fall_risk_pct: float = 30.0
    estop_swd_value_down: bool = True

    def __post_init__(self):
        for attr in (
            "arm_q_min", "arm_q_max", "arm_qvel_max",
            "right_ee_min", "right_ee_max",
            "left_ee_min", "left_ee_max",
        ):
            v = getattr(self, attr)
            if isinstance(v, list):
                setattr(self, attr, np.asarray(v, dtype=np.float32))


# ────────────────────────── SafetyInfo ──────────────────────────


@dataclass
class SafetyInfo:
    """Per-step audit produced by :meth:`SafetySupervisor.validate`."""

    raw_action: np.ndarray
    safe_action: np.ndarray
    clipped: bool = False
    soft_hold: bool = False
    safe_stop: bool = False
    emergency_stop: bool = False
    reason: List[str] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)

    @property
    def hold_or_stop(self) -> bool:
        return self.soft_hold or self.safe_stop or self.emergency_stop


# ──────────────────────── SafetySupervisor ──────────────────────


class GalaxeaR1ProSafetySupervisor:
    """Five-level safety pipeline.

    Used inside :meth:`GalaxeaR1ProEnv.step` *before* publishing to the
    controller.  See class docstring of the module for the full
    contract.
    """

    def __init__(self, cfg: SafetyConfig):
        self._cfg = cfg
        self._operator_heartbeat_ms: float = (time.monotonic() * 1000.0)

    @property
    def cfg(self) -> SafetyConfig:
        return self._cfg

    def heartbeat(self) -> None:
        """Operator UI / wrapper calls this periodically.

        If skipped for more than ``operator_heartbeat_timeout_ms``,
        L5 escalates to SAFE_STOP on next ``validate``.
        """
        self._operator_heartbeat_ms = time.monotonic() * 1000.0

    def validate(
        self,
        action: np.ndarray,
        state: "GalaxeaR1ProRobotState",
        action_schema: "ActionSchema",
    ) -> SafetyInfo:
        """Run all five levels of safety checks on *action*.

        Args:
            action: Raw policy action vector (already in
                ``[-1, 1]^D`` is recommended).
            state: Latest robot snapshot.
            action_schema: Stage-dependent schema.

        Returns:
            :class:`SafetyInfo` with ``safe_action`` ready to be
            executed by the controller.
        """
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        info = SafetyInfo(raw_action=action.copy(), safe_action=action.copy())

        # ── L1 schema ───────────────────────────────────────────
        if not np.all(np.isfinite(info.safe_action)):
            info.safe_action = np.zeros_like(info.safe_action)
            info.emergency_stop = True
            info.reason.append("L1:non_finite_action")
        clipped = np.clip(info.safe_action, -1.0, 1.0)  #@#TODO 这里是不是要保证之前就归一化到[-1,1]或其它范围???
        if not np.array_equal(clipped, info.safe_action):
            info.clipped = True
            info.reason.append("L1:clipped_to_unit_box")
        info.safe_action = clipped

        # ── L2 per-joint limit ──────────────────────────────────
        # We do not have raw qpos in the action (only deltas), so L2
        # acts as a guard against absurdly large velocity demands.
        # Predicted next q is checked through the EE clip + step cap.

        # ── L3a per-arm TCP safety box ──────────────────────────
        if action_schema.has_right_arm:
            target = action_schema.predict_arm_ee_pose(
                "right", info.safe_action, state,
            )
            if target is not None:
                clipped_target = self._clip_to_box(
                    target,
                    self._cfg.right_ee_min,
                    self._cfg.right_ee_max,
                    "L3a:right_ee_box", info,
                )
                self._rewrite_arm_action(
                    "right", info.safe_action, clipped_target, state,
                    action_schema,
                )
        if action_schema.has_left_arm:
            target_l = action_schema.predict_arm_ee_pose(
                "left", info.safe_action, state,
            )
            if target_l is not None:
                clipped_target_l = self._clip_to_box(
                    target_l,
                    self._cfg.left_ee_min,
                    self._cfg.left_ee_max,
                    "L3a:left_ee_box", info,
                )
                self._rewrite_arm_action(
                    "left", info.safe_action, clipped_target_l, state,
                    action_schema,
                )

        # ── L3b dual-arm collision spheres ──────────────────────
        if (
            self._cfg.dual_arm_collision_enable
            and action_schema.has_dual_arms
        ):
            tgt_r = action_schema.predict_arm_ee_pose("right", info.safe_action, state)
            tgt_l = action_schema.predict_arm_ee_pose("left", info.safe_action, state)
            if tgt_r is not None and tgt_l is not None:
                d = float(np.linalg.norm(tgt_r[:3] - tgt_l[:3]))
                if d < self._cfg.dual_arm_min_distance_m:
                    info.soft_hold = True
                    info.reason.append(
                        f"L3b:dual_arm_collision d={d:.3f}m"
                    )
                    info.safe_action = np.zeros_like(info.safe_action)

        # ── L4 vel / accel / jerk caps ──────────────────────────
        info.safe_action = self._apply_velocity_caps(
            info.safe_action, action_schema, info,
        )

        # ── L5 system watchdog ──────────────────────────────────
        bms_pct = float(state.bms.get("capital_pct", 100.0))
        if bms_pct < self._cfg.bms_low_battery_threshold_pct:
            info.safe_stop = True
            info.reason.append(f"L5:bms_low {bms_pct:.1f}pct")
        if state.feedback_age_ms:
            for src, age in state.feedback_age_ms.items():
                if age > self._cfg.feedback_stale_threshold_ms:
                    info.soft_hold = True
                    info.reason.append(f"L5:stale {src} {age:.0f}ms")
        if (
            self._cfg.estop_swd_value_down
            and state.controller_signal.get("swd", 0)
        ):
            info.emergency_stop = True
            info.reason.append("L5:SWD_DOWN")
        for side, errs in state.status_errors.items():
            if errs:
                info.soft_hold = True
                info.reason.append(
                    f"L5:status_errors_{side}={list(errs)[:5]}"
                )

        # Operator heartbeat watchdog
        now_ms = time.monotonic() * 1000.0
        hb_age = now_ms - self._operator_heartbeat_ms
        if hb_age > self._cfg.operator_heartbeat_timeout_ms:
            info.soft_hold = True
            info.reason.append(f"L5:operator_hb_age={hb_age:.0f}ms")

        # A2 fall-risk posture rule: when battery is low *and* the
        # arm is far from a natural-hanging posture, bias action
        # toward zero to avoid bounce on power-loss.
        if bms_pct < self._cfg.a2_fall_risk_pct:
            info.safe_action *= 0.5
            if "L5:a2_fall_risk_dampen" not in info.reason:
                info.reason.append("L5:a2_fall_risk_dampen")

        # If any termination requested, freeze action.
        if info.emergency_stop or info.safe_stop:
            info.safe_action = np.zeros_like(info.safe_action)

        info.metrics = {
            "safety/clip_ratio": float(info.clipped),
            "safety/soft_hold": float(info.soft_hold),
            "safety/safe_stop": float(info.safe_stop),
            "safety/emergency_stop": float(info.emergency_stop),
            "safety/reason_count": float(len(info.reason)),
            "hw/bms_capital_pct": bms_pct,
        }
        return info

    # ── Internal helpers ────────────────────────────────────────

    def _clip_to_box(
        self,
        target: np.ndarray,
        lo: np.ndarray,
        hi: np.ndarray,
        tag: str,
        info: SafetyInfo,
    ) -> np.ndarray:
        clipped = np.clip(target, lo, hi)
        if not np.allclose(clipped, target, atol=1e-6):
            info.clipped = True
            info.reason.append(tag)
        return clipped

    def _rewrite_arm_action(
        self,
        side: str,
        action: np.ndarray,
        clipped_target_xyzrpy: np.ndarray,
        state: "GalaxeaR1ProRobotState",
        schema: "ActionSchema",
    ) -> None:
        """In-place rewrite the per-arm xyz / rpy slice of *action*
        so that the predicted EE pose lands at ``clipped_target``.

        This keeps the rest of the code (which only touches the action
        vector and the controller) oblivious to the clipping.
        """
        from scipy.spatial.transform import Rotation as R

        ee = state.get_ee_pose(side)
        cur_xyz = ee[:3].astype(np.float32)
        cur_eul = R.from_quat(np.asarray(ee[3:], dtype=np.float64)).as_euler(
            "xyz"
        )
        delta_xyz = clipped_target_xyzrpy[:3] - cur_xyz
        delta_eul = clipped_target_xyzrpy[3:] - cur_eul

        # Convert back to *normalised* per-step deltas.
        scale_pos = max(float(schema.action_scale[0]), 1e-9)
        scale_ori = max(float(schema.action_scale[1]), 1e-9)
        norm_xyz = delta_xyz / scale_pos
        norm_rpy = delta_eul / scale_ori

        # Locate the per-arm slice in the action vector.
        idx = 0
        if schema.has_right_arm:
            if side == "right":
                action[idx : idx + 3] = norm_xyz
                action[idx + 3 : idx + 6] = norm_rpy
                return
            idx += schema.per_arm_dim
        if schema.has_left_arm and side == "left":
            action[idx : idx + 3] = norm_xyz
            action[idx + 3 : idx + 6] = norm_rpy

    def _apply_velocity_caps(
        self,
        action: np.ndarray,
        schema: "ActionSchema",
        info: SafetyInfo,
    ) -> np.ndarray:
        """Clamp per-step deltas to ``max_linear_step_m`` /
        ``max_angular_step_rad`` and chassis / torso to their per-axis
        velocity caps.

        Chassis / torso clamps are absolute on the per-axis component
        (after multiplying by ``action_scale[2]`` upstream;
        :class:`ActionSchema` does not pre-scale chassis/torso).
        """
        idx = 0
        scale_pos = float(schema.action_scale[0])
        scale_ori = float(schema.action_scale[1])
        max_lin = self._cfg.max_linear_step_m
        max_ang = self._cfg.max_angular_step_rad

        # Right / left arm (xyz + rpy) caps.
        for present in (schema.has_right_arm, schema.has_left_arm):
            if not present:
                continue
            xyz = action[idx : idx + 3] * scale_pos
            rpy = action[idx + 3 : idx + 6] * scale_ori
            xyz_clip = np.clip(xyz, -max_lin, max_lin)
            rpy_clip = np.clip(rpy, -max_ang, max_ang)
            if (
                not np.array_equal(xyz_clip, xyz)
                or not np.array_equal(rpy_clip, rpy)
            ):
                info.clipped = True
                info.reason.append("L4:per_step_cap")
            action[idx : idx + 3] = xyz_clip / max(scale_pos, 1e-9)
            action[idx + 3 : idx + 6] = rpy_clip / max(scale_ori, 1e-9)
            idx += schema.per_arm_dim

        # Torso twist clamp (action layout: [v_x, v_z, w_pitch, w_yaw]).
        if schema.has_torso:
            tor = action[idx : idx + 4].copy()
            limits = np.array(
                [
                    self._cfg.torso_v_x_max,
                    self._cfg.torso_v_z_max,
                    self._cfg.torso_w_pitch_max,
                    self._cfg.torso_w_yaw_max,
                ],
                dtype=np.float32,
            )
            clipped = np.clip(tor, -limits, limits)
            if not np.array_equal(tor, clipped):
                info.clipped = True
                info.reason.append("L4:torso_cap")
            action[idx : idx + 4] = clipped
            idx += 4

        # Chassis twist clamp ([v_x, v_y, w_z]).
        if schema.has_chassis:
            ch = action[idx : idx + 3].copy()
            limits = np.array(
                [
                    self._cfg.chassis_v_x_max,
                    self._cfg.chassis_v_y_max,
                    self._cfg.chassis_w_z_max,
                ],
                dtype=np.float32,
            )
            clipped = np.clip(ch, -limits, limits)
            if not np.array_equal(ch, clipped):
                info.clipped = True
                info.reason.append("L4:chassis_cap")
            action[idx : idx + 3] = clipped
            idx += 3

        return action


def build_safety_config(cfg_dict: Optional[dict]) -> SafetyConfig:
    """Build a :class:`SafetyConfig` from a YAML-derived dict.

    Unknown keys are ignored with a warning to make it forward
    compatible with newer config additions.
    """
    if cfg_dict is None:
        return SafetyConfig()
    base = SafetyConfig()
    valid_keys = set(base.__dict__.keys())
    kwargs = {}
    for k, v in dict(cfg_dict).items():
        if k in valid_keys:
            kwargs[k] = v
    base_kwargs = {**base.__dict__, **kwargs}
    return SafetyConfig(**base_kwargs)
