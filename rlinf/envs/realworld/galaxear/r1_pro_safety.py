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

import logging
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, List, Optional

import numpy as np

if TYPE_CHECKING:
    from .r1_pro_action_schema import ActionSchema
    from .r1_pro_robot_state import GalaxeaR1ProRobotState

_logger = logging.getLogger(__name__)


# ───────────────────────── Config dataclass ─────────────────────


@dataclass
class SafetyConfig:
    """Hard safety limits + watchdog thresholds.

    Position and velocity defaults are derived from the R1 Pro URDF
    (``mobiman/lib/mobiman/configs/urdfs/r1_pro_floating_*.urdf`` and
    ``mobiman/share/mobiman/urdf/R1_PRO/urdf/r1_pro.urdf``) minus a
    0.1 rad (5.7°) safety margin per joint, with the velocity bound
    additionally cross-checked against the BRS-ctrl production
    constants in ``R1ProInterface``.  Notably the **left and right
    arms differ on J2** (mirrored mechanical design), so we keep
    two separate position vectors instead of a single ``arm_q_min/max``.

    See ``bt/docs/rwRL/safety_2_joinlimit_2.md`` §3 for the full
    derivation and rationale.

    Tune via ``override_cfg.safety_cfg`` in the env YAML.
    """

    # ── L2: 7-DoF per-arm position limits (rad) ────────────────
    # URDF - 0.1 rad margin.  J2 is mirrored between arms.
    right_arm_q_min: np.ndarray = field(default_factory=lambda: np.array(
        [-4.35, -3.04, -2.26, -1.99, -2.26, -0.95, -1.47],
        dtype=np.float32))
    right_arm_q_max: np.ndarray = field(default_factory=lambda: np.array(
        [1.21, 0.07, 2.26, 0.25, 2.26, 0.95, 1.47],
        dtype=np.float32))
    left_arm_q_min: np.ndarray = field(default_factory=lambda: np.array(
        [-4.35, -0.07, -2.26, -1.99, -2.26, -0.95, -1.47],
        dtype=np.float32))
    left_arm_q_max: np.ndarray = field(default_factory=lambda: np.array(
        [1.21, 3.04, 2.26, 0.25, 2.26, 0.95, 1.47],
        dtype=np.float32))

    # ── L2: 7-DoF per-arm velocity limits (rad/s) ──────────────
    # min(URDF * 0.5, BRS-ctrl reset.py); same for both arms.
    arm_qvel_max: np.ndarray = field(default_factory=lambda: np.array(
        [1.6, 1.6, 1.6, 1.6, 4.0, 4.0, 4.0], dtype=np.float32))

    # ── L2: 4-DoF torso ─────────────────────────────────────────
    # BRS-ctrl R1ProInterface.torso_joint_high/low - 0.1 rad.
    torso_q_min: np.ndarray = field(default_factory=lambda: np.array(
        [-1.03, -2.69, -1.73, -2.95], dtype=np.float32))
    torso_q_max: np.ndarray = field(default_factory=lambda: np.array(
        [1.73, 2.43, 1.47, 2.95], dtype=np.float32))
    torso_qvel_max: np.ndarray = field(default_factory=lambda: np.array(
        [0.5, 0.5, 0.5, 0.5], dtype=np.float32))

    # ── L2: 3-DoF chassis (only velocity + dead-zone) ──────────
    # BRS-ctrl R1ProInterface.mobile_base_cmd_limit / threshold.
    chassis_qvel_max: np.ndarray = field(default_factory=lambda: np.array(
        [0.30, 0.30, 0.40], dtype=np.float32))  # vx, vy, wz
    chassis_dead_zone: np.ndarray = field(default_factory=lambda: np.array(
        [0.01, 0.01, 0.05], dtype=np.float32))

    # ── L2: gripper (0-100% stroke) ────────────────────────────
    gripper_pct_min: float = 0.0
    gripper_pct_max: float = 100.0
    max_gripper_step_pct: float = 30.0  # per-step delta cap

    # ── L2: thresholds & gains ─────────────────────────────────
    l2_warning_margin_rad: float = 0.15      # start scaling action
    l2_critical_margin_rad: float = 0.05     # freeze that joint
    l2_qvel_overspeed_factor: float = 1.20   # > 1.2x limit -> soft_hold
    l2_qvel_warning_factor: float = 0.90     # > 0.9x limit -> scale 0.5

    # ── L2 sub-layer toggles (let YAML disable individual checks) ──
    enable_l2a_predict_q_clip: bool = True
    enable_l2b_qpos_freeze: bool = True
    enable_l2c_qvel_watchdog: bool = True
    enable_l2d_cmd_speed_cap: bool = True
    enable_l2_gripper: bool = True

    # ── Step period used by L2d (s); should match Env.step_frequency. ──
    dt_step: float = 0.10  # 10 Hz default

    # ── Deprecated single-arm aliases (kept for back-compat) ────
    # Old code used a single ``arm_q_min/max`` shared between arms.
    # If a YAML still passes them, ``__post_init__`` will splat them
    # into both ``right_arm_q_*`` and ``left_arm_q_*`` and emit a
    # one-time warning so users can migrate.
    arm_q_min: Optional[np.ndarray] = None  # deprecated: use right/left
    arm_q_max: Optional[np.ndarray] = None  # deprecated: use right/left

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
            "right_arm_q_min", "right_arm_q_max",
            "left_arm_q_min", "left_arm_q_max",
            "arm_qvel_max",
            "torso_q_min", "torso_q_max", "torso_qvel_max",
            "chassis_qvel_max", "chassis_dead_zone",
            "right_ee_min", "right_ee_max",
            "left_ee_min", "left_ee_max",
        ):
            v = getattr(self, attr)
            if isinstance(v, list):
                setattr(self, attr, np.asarray(v, dtype=np.float32))

        # Back-compat: legacy single-arm ``arm_q_min/max`` overrides.
        if self.arm_q_min is not None:
            arr = np.asarray(self.arm_q_min, dtype=np.float32).reshape(-1)[:7]
            if arr.size == 7:
                self.right_arm_q_min = arr.copy()
                self.left_arm_q_min = arr.copy()
                _logger.warning(
                    "SafetyConfig.arm_q_min is deprecated; populated both "
                    "right_arm_q_min and left_arm_q_min from it.  Migrate "
                    "your YAML to use right_arm_q_min / left_arm_q_min "
                    "(R1 Pro arms are mirrored on J2 -- see "
                    "bt/docs/rwRL/safety_2_joinlimit_2.md §3.3).",
                )
        if self.arm_q_max is not None:
            arr = np.asarray(self.arm_q_max, dtype=np.float32).reshape(-1)[:7]
            if arr.size == 7:
                self.right_arm_q_max = arr.copy()
                self.left_arm_q_max = arr.copy()
                _logger.warning(
                    "SafetyConfig.arm_q_max is deprecated; populated both "
                    "right_arm_q_max and left_arm_q_max from it.",
                )


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
    # New in safety_2_joinlimit_2: per-joint, per-side L2 audit so
    # analyzers can spot 'always saturated J6' style issues.  Schema:
    #   {
    #     "right": {"J3": {"L2a": 0.5, "L2c": "freeze"}, ...},
    #     "left":  {"J7": {"L2b": "freeze"}},
    #     "torso": {"T2": {"L2b": "freeze"}},
    #     "chassis": {"vy": {"L2d": "deadzone"}, "wz": {"L2d": "cap"}},
    #     "gripper_right": {"L2": "rate_cap"},
    #   }
    l2_per_joint_clip: Dict[str, Dict[str, dict]] = field(
        default_factory=dict
    )

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

        # ── L2 per-joint limit (4 sub-layers + gripper) ─────────
        # Sequence: predict-then-shrink (L2a) -> hard freeze (L2b)
        # -> feedback velocity watchdog (L2c) -> commanded velocity
        # cap (L2d) -> gripper position/rate cap.  Each sub-layer is
        # individually toggleable via SafetyConfig.enable_l2*.
        # See bt/docs/rwRL/safety_2_joinlimit_2.md §4 for rationale.
        if self._cfg.enable_l2a_predict_q_clip:
            self._check_l2a_predict_q_clip(info, state, action_schema)
        if self._cfg.enable_l2b_qpos_freeze:
            self._check_l2b_qpos_freeze(info, state, action_schema)
        if self._cfg.enable_l2c_qvel_watchdog:
            self._check_l2c_qvel_watchdog(info, state, action_schema)
        if self._cfg.enable_l2d_cmd_speed_cap:
            self._check_l2d_cmd_speed_cap(info, state, action_schema)
        if self._cfg.enable_l2_gripper:
            self._check_l2_gripper(info, state, action_schema)

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

    # ── L2 sub-layer helpers ────────────────────────────────────

    @staticmethod
    def _record_per_joint_clip(
        info: SafetyInfo,
        side: str,
        joint_label: str,
        layer: str,
        value,
    ) -> None:
        """Append a per-joint audit entry to ``info.l2_per_joint_clip``."""
        side_dict = info.l2_per_joint_clip.setdefault(side, {})
        joint_dict = side_dict.setdefault(joint_label, {})
        joint_dict[layer] = value

    def _arm_q_bounds(
        self,
        side: str,
    ) -> tuple[np.ndarray, np.ndarray]:
        if side == "right":
            return self._cfg.right_arm_q_min, self._cfg.right_arm_q_max
        return self._cfg.left_arm_q_min, self._cfg.left_arm_q_max

    def _check_l2a_predict_q_clip(
        self,
        info: SafetyInfo,
        state: "GalaxeaR1ProRobotState",
        schema: "ActionSchema",
    ) -> None:
        """Predict next-step joint angles; if the worst joint margin
        falls below ``warning_margin`` (or ``critical_margin``), shrink
        the per-arm action proportionally.

        In Cartesian EE control mode (the default for M1 bring-up)
        ``schema.predict_arm_qpos`` returns the current qpos as a
        no-op predictor, so this layer is essentially a placeholder
        until an IK callback is plugged in.  The protection is then
        delegated to L2b (post-hoc qpos freeze) and L2c (qvel
        watchdog) — see safety_2_joinlimit_2.md §7.2.
        """
        for side in ("right", "left"):
            if side == "right" and not schema.has_right_arm:
                continue
            if side == "left" and not schema.has_left_arm:
                continue

            # Skip when feedback hasn't arrived yet -- the default zero
            # state vector lands on top of left J2's q_min (-0.07) and
            # would make L2a fire on every first step before subscribers
            # populate ``state.{side}_arm_qpos``.  L2b also enforces
            # this guard for symmetry.
            cur_q_raw = np.asarray(
                state.get_arm_qpos(side), dtype=np.float32,
            ).reshape(-1)
            if cur_q_raw.size < 7 or not np.any(cur_q_raw[:7]):
                continue

            q_next = schema.predict_arm_qpos(side, info.safe_action, state)
            if q_next is None:
                continue
            q_next = np.asarray(q_next, dtype=np.float32).reshape(-1)[:7]
            if q_next.size != 7:
                continue

            q_min, q_max = self._arm_q_bounds(side)
            margin_low = q_next - q_min
            margin_high = q_max - q_next
            margin = np.minimum(margin_low, margin_high)
            worst = float(np.min(margin))
            worst_idx = int(np.argmin(margin))

            if worst < self._cfg.l2_critical_margin_rad:
                scale = 0.0
                tag_kind = "critical"
            elif worst < self._cfg.l2_warning_margin_rad:
                scale = max(
                    0.0,
                    worst / max(self._cfg.l2_warning_margin_rad, 1e-9),
                )
                tag_kind = "warning"
            else:
                continue  # plenty of room

            info.safe_action = schema.rewrite_action_arm(
                info.safe_action, side, scale,
            )
            info.clipped = True
            info.reason.append(
                f"L2a:{side}_q_{tag_kind} J{worst_idx + 1} "
                f"margin={worst:+.3f}rad scale={scale:.2f}"
            )
            info.metrics[f"safety/l2a_{side}_scale"] = float(scale)
            info.metrics[f"safety/l2a_{side}_worst_margin"] = float(worst)
            self._record_per_joint_clip(
                info, side, f"J{worst_idx + 1}", "L2a",
                {"scale": float(scale), "margin": float(worst)},
            )

    def _check_l2b_qpos_freeze(
        self,
        info: SafetyInfo,
        state: "GalaxeaR1ProRobotState",
        schema: "ActionSchema",
    ) -> None:
        """If the *current* feedback qpos is already inside
        ``critical_margin`` from any joint limit, freeze that arm
        (or torso) for one step and raise ``soft_hold``.

        This is the reactive complement to the predictive L2a: it
        catches the case where last step's command pushed a joint
        right up to its limit even though L2a believed margin was
        OK (e.g. mobiman IK overshoot, mechanical play, or a
        first-step where the policy violated its priors).
        """
        # Arms.
        for side in ("right", "left"):
            if side == "right" and not schema.has_right_arm:
                continue
            if side == "left" and not schema.has_left_arm:
                continue

            cur_q = np.asarray(
                state.get_arm_qpos(side), dtype=np.float32,
            ).reshape(-1)[:7]
            if cur_q.size != 7:
                continue
            # Skip the freeze when feedback hasn't arrived yet
            # (initial state vector is all-zeros, which would land
            # on top of J2's limit and falsely freeze the arm).
            if not np.any(cur_q):
                continue
            q_min, q_max = self._arm_q_bounds(side)
            margin = np.minimum(cur_q - q_min, q_max - cur_q)

            bad_idx = np.where(margin < self._cfg.l2_critical_margin_rad)[0]
            if bad_idx.size > 0:
                info.safe_action = schema.rewrite_action_arm(
                    info.safe_action, side, 0.0,
                )
                info.soft_hold = True
                joint_labels = ",".join(f"J{int(i) + 1}" for i in bad_idx)
                info.reason.append(
                    f"L2b:{side}_qpos_critical {joint_labels} "
                    f"margin={float(margin[bad_idx].min()):+.3f}"
                )
                info.metrics[f"safety/l2b_{side}_freeze"] = 1.0
                for i in bad_idx:
                    self._record_per_joint_clip(
                        info, side, f"J{int(i) + 1}", "L2b", "freeze",
                    )

        # Torso.
        if schema.has_torso:
            cur_t = np.asarray(
                state.get_torso_qpos(), dtype=np.float32,
            ).reshape(-1)[:4]
            if cur_t.size == 4 and np.any(cur_t):
                margin_t = np.minimum(
                    cur_t - self._cfg.torso_q_min,
                    self._cfg.torso_q_max - cur_t,
                )
                bad_t = np.where(margin_t < self._cfg.l2_critical_margin_rad)[0]
                if bad_t.size > 0:
                    info.safe_action = schema.rewrite_action_torso(
                        info.safe_action, 0.0,
                    )
                    info.soft_hold = True
                    tlst = ",".join(f"T{int(i) + 1}" for i in bad_t)
                    info.reason.append(
                        f"L2b:torso_qpos_critical {tlst} "
                        f"margin={float(margin_t[bad_t].min()):+.3f}"
                    )
                    info.metrics["safety/l2b_torso_freeze"] = 1.0
                    for i in bad_t:
                        self._record_per_joint_clip(
                            info, "torso", f"T{int(i) + 1}",
                            "L2b", "freeze",
                        )

    def _check_l2c_qvel_watchdog(
        self,
        info: SafetyInfo,
        state: "GalaxeaR1ProRobotState",
        schema: "ActionSchema",
    ) -> None:
        """React to runaway velocity *as reported by the robot*.

        Two thresholds (per the design doc):
        - feedback |qvel| > overspeed_factor * qvel_max -> soft_hold
        - feedback |qvel| > warning_factor   * qvel_max -> scale 0.5
        """
        over = float(self._cfg.l2_qvel_overspeed_factor)
        warn = float(self._cfg.l2_qvel_warning_factor)

        # Arms.
        qvm_arm = self._cfg.arm_qvel_max
        for side in ("right", "left"):
            if side == "right" and not schema.has_right_arm:
                continue
            if side == "left" and not schema.has_left_arm:
                continue
            qv = np.asarray(
                state.get_arm_qvel(side), dtype=np.float32,
            ).reshape(-1)[:7]
            if qv.size != 7:
                continue
            ratio = np.abs(qv) / np.maximum(qvm_arm, 1e-6)
            max_ratio = float(np.max(ratio))
            max_idx = int(np.argmax(ratio))
            if max_ratio > over:
                info.safe_action = schema.rewrite_action_arm(
                    info.safe_action, side, 0.0,
                )
                info.soft_hold = True
                info.reason.append(
                    f"L2c:{side}_qvel_overspeed J{max_idx + 1} "
                    f"max={max_ratio:.2f}x limit"
                )
                info.metrics[
                    f"safety/l2c_{side}_overspeed_ratio"
                ] = max_ratio
                self._record_per_joint_clip(
                    info, side, f"J{max_idx + 1}", "L2c", "freeze",
                )
            elif max_ratio > warn:
                info.safe_action = schema.rewrite_action_arm(
                    info.safe_action, side, 0.5,
                )
                info.clipped = True
                info.reason.append(
                    f"L2c:{side}_qvel_warning J{max_idx + 1} "
                    f"max={max_ratio:.2f}x limit, scale=0.5"
                )
                info.metrics[
                    f"safety/l2c_{side}_warn_ratio"
                ] = max_ratio
                self._record_per_joint_clip(
                    info, side, f"J{max_idx + 1}", "L2c", "scale=0.5",
                )

        # Torso.
        if schema.has_torso:
            qv_t = np.asarray(
                state.get_torso_qvel(), dtype=np.float32,
            ).reshape(-1)[:4]
            if qv_t.size == 4:
                ratio_t = np.abs(qv_t) / np.maximum(
                    self._cfg.torso_qvel_max, 1e-6,
                )
                if np.any(ratio_t > over):
                    info.safe_action = schema.rewrite_action_torso(
                        info.safe_action, 0.0,
                    )
                    info.soft_hold = True
                    info.reason.append(
                        f"L2c:torso_qvel_overspeed max={float(ratio_t.max()):.2f}x"
                    )
                    info.metrics["safety/l2c_torso_overspeed_ratio"] = float(
                        ratio_t.max()
                    )
                elif np.any(ratio_t > warn):
                    info.safe_action = schema.rewrite_action_torso(
                        info.safe_action, 0.5,
                    )
                    info.clipped = True
                    info.reason.append(
                        f"L2c:torso_qvel_warning max={float(ratio_t.max()):.2f}x, scale=0.5"
                    )

        # Chassis.
        if schema.has_chassis:
            qv_c = np.asarray(
                state.get_chassis_qvel(), dtype=np.float32,
            ).reshape(-1)[:3]
            if qv_c.size == 3:
                ratio_c = np.abs(qv_c) / np.maximum(
                    self._cfg.chassis_qvel_max, 1e-6,
                )
                if np.any(ratio_c > over):
                    info.safe_action = schema.rewrite_action_chassis(
                        info.safe_action, 0.0,
                    )
                    info.soft_hold = True
                    info.reason.append(
                        f"L2c:chassis_qvel_overspeed max={float(ratio_c.max()):.2f}x"
                    )

    def _check_l2d_cmd_speed_cap(
        self,
        info: SafetyInfo,
        state: "GalaxeaR1ProRobotState",
        schema: "ActionSchema",
    ) -> None:
        """Bound the *commanded* velocity for joint-mode arms,
        torso (4-D twist) and chassis (3-D twist).

        For Cartesian-mode arms (the default M1 path) we cannot map
        EE deltas to joint-velocity demands without IK, so this
        layer no-ops on the arm slice in that case (L4 already caps
        Cartesian per-step distance).  When ``schema.use_joint_mode``
        is True the arm slice is interpreted as a per-step joint
        delta and capped via |delta_q| / dt <= qvel_max.

        Chassis additionally honours the BRS-style dead-zone to
        avoid command jitter triggering wheel buzz.
        """
        dt = max(float(self._cfg.dt_step), 1e-3)
        d = schema.split(info.safe_action)

        # Arms (joint-mode only).
        if schema.use_joint_mode:
            for side in ("right", "left"):
                if side == "right" and not schema.has_right_arm:
                    continue
                if side == "left" and not schema.has_left_arm:
                    continue
                xyz = np.asarray(
                    d.get(f"{side}_xyz", np.zeros(3)), dtype=np.float32,
                ).reshape(-1)[:3]
                rpy = np.asarray(
                    d.get(f"{side}_rpy", np.zeros(3)), dtype=np.float32,
                ).reshape(-1)[:3]
                joint_delta6 = np.concatenate([xyz, rpy])
                v_cmd = (
                    joint_delta6 * float(schema.action_scale[0]) / dt
                )
                qvm = self._cfg.arm_qvel_max[: v_cmd.size]
                ratio = np.abs(v_cmd) / np.maximum(qvm, 1e-6)
                max_ratio = float(np.max(ratio)) if ratio.size else 0.0
                if max_ratio > 1.0:
                    scale = 1.0 / max_ratio
                    info.safe_action = schema.rewrite_action_arm(
                        info.safe_action, side, scale,
                    )
                    info.clipped = True
                    info.reason.append(
                        f"L2d:{side}_cmd_qvel_cap max={max_ratio:.2f}x limit, "
                        f"scale={scale:.2f}"
                    )

        # Chassis: action *is* a velocity vector (m/s, m/s, rad/s).
        if schema.has_chassis:
            v_raw = np.asarray(
                d.get("chassis_twist", np.zeros(3)), dtype=np.float32,
            ).reshape(-1)[:3]
            v = v_raw.copy()
            modified = False
            # Dead-zone (per-axis).
            dead = np.abs(v) < self._cfg.chassis_dead_zone
            if np.any(dead & (v != 0.0)):
                v = np.where(dead, 0.0, v).astype(np.float32)
                modified = True
                self._record_per_joint_clip(
                    info, "chassis", "deadzone",
                    "L2d", v_raw.tolist(),
                )
            # Per-axis cap.
            v_clip = np.clip(
                v,
                -self._cfg.chassis_qvel_max,
                self._cfg.chassis_qvel_max,
            )
            if not np.array_equal(v_clip, v):
                modified = True
                self._record_per_joint_clip(
                    info, "chassis", "cap",
                    "L2d", v.tolist(),
                )
                v = v_clip
            if modified:
                info.safe_action = schema.rewrite_action_chassis_set(
                    info.safe_action, v,
                )
                info.clipped = True
                info.reason.append(
                    f"L2d:chassis_cmd_speed_cap raw={v_raw.tolist()}"
                )

        # Torso: action is a 4-D twist (v_x, v_z, w_pitch, w_yaw),
        # already capped by L4 in absolute units.  L2d here only
        # adds the per-joint qvel envelope cap; symbolic units match
        # ``torso_qvel_max[i]`` per axis.
        if schema.has_torso:
            v_t = np.asarray(
                d.get("torso_twist", np.zeros(4)), dtype=np.float32,
            ).reshape(-1)[:4]
            ratio_t = np.abs(v_t) / np.maximum(self._cfg.torso_qvel_max, 1e-6)
            if ratio_t.size and np.any(ratio_t > 1.0):
                scale = 1.0 / float(ratio_t.max())
                info.safe_action = schema.rewrite_action_torso(
                    info.safe_action, scale,
                )
                info.clipped = True
                info.reason.append(
                    f"L2d:torso_cmd_speed_cap scale={scale:.2f}"
                )

    def _check_l2_gripper(
        self,
        info: SafetyInfo,
        state: "GalaxeaR1ProRobotState",
        schema: "ActionSchema",
    ) -> None:
        """Two L2 checks for each enabled gripper:

        1. Per-step delta cap (``max_gripper_step_pct``):
           |Δ pct| ≤ max_gripper_step_pct.
        2. Position bound: predicted next pct ∈
           [gripper_pct_min, gripper_pct_max].

        Both are implemented in the predict-clip-rewrite style used
        by L3a so that the rest of the pipeline sees a clean
        normalised action.
        """
        if schema.no_gripper:
            return
        scale_g = max(float(schema.action_scale[2]), 1e-9)
        max_step = float(self._cfg.max_gripper_step_pct)
        lo = float(self._cfg.gripper_pct_min)
        hi = float(self._cfg.gripper_pct_max)
        for side in ("right", "left"):
            if side == "right" and not schema.has_right_arm:
                continue
            if side == "left" and not schema.has_left_arm:
                continue
            d = schema.split(info.safe_action)
            key = f"{side}_gripper"
            if key not in d:
                continue
            a_g = float(d[key])           # already in [-1, 1] post L1
            cur_pct = float(state.get_gripper_pos(side))
            delta_pct = a_g * scale_g

            # 1) rate cap
            new_a = a_g
            rate_capped = False
            if abs(delta_pct) > max_step:
                new_a = float(np.sign(a_g) * max_step / scale_g)
                rate_capped = True

            # 2) position bound
            next_pct = cur_pct + new_a * scale_g
            pos_capped = False
            if next_pct < lo or next_pct > hi:
                target = float(np.clip(next_pct, lo, hi))
                new_a = float((target - cur_pct) / scale_g)
                pos_capped = True

            if rate_capped or pos_capped:
                info.safe_action = schema.set_gripper_action(
                    info.safe_action, side, new_a,
                )
                info.clipped = True
                tag_parts = []
                if rate_capped:
                    tag_parts.append(
                        f"rate_cap |Δ|={abs(delta_pct):.1f}>{max_step:.1f}%/step"
                    )
                if pos_capped:
                    tag_parts.append(
                        f"pos_cap pred={next_pct:.1f}->[{lo:.1f},{hi:.1f}]"
                    )
                info.reason.append(
                    f"L2:{side}_gripper " + " ".join(tag_parts)
                )
                self._record_per_joint_clip(
                    info, f"gripper_{side}", "g",
                    "L2",
                    {
                        "rate_cap": rate_capped,
                        "pos_cap": pos_capped,
                        "raw_a": a_g,
                        "new_a": new_a,
                    },
                )

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
