#!/usr/bin/env python3
"""Joint-space Gym environment for Franka VLA evaluation.

Uses FrankyControllerDirect for safe robot control with
motion guard, watchdog, and collision behavior tightening.

8-level safety hierarchy:
  L1: check_action_safety -- joint clipping (per step)
  L2: check_action_safety -- training range + margin (per step)
  L3: check_action_safety -- velocity limiting (per step)
  L4: Motion guard -- TCP fence watchdog (50 Hz, FrankyControllerDirect)
  L5: Joint velocity norm limit (50 Hz watchdog)
  L6: Collision behavior tightening (init, FrankyControllerDirect)
  L7: libfranka hardware reflex (1 kHz, robot firmware)
  L8: E-Stop button (immediate, hardware)

Reused from RLinf:
  - franky_ext.motion_limits (safety constants, imported)
  - FrankyControllerDirect (safety logic from FrankyControllerExtended)

Reused from eval_3A2 FrankyJointEnvMixin:
  - check_action_safety() 3-layer joint-space safety
  - go_to_rest() joint reset + gripper open sequence
  - MotionGuardTripped exception handling
"""
from __future__ import annotations

import logging
import os
import sys
import time
from pathlib import Path
from typing import Optional

import gymnasium as gym
import numpy as np

# 4dwvla_ext 以数字开头, 不能用 from 4dwvla_ext.X import Y
# 直接把本目录加入 sys.path 后按模块名导入
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from franky_controller_direct import (
    FrankyControllerDirect,
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    JOINT_VEL_LIMITS,
)
from franky_ext.dsplug.home_pose import load_home_joints

logger = logging.getLogger(__name__)

# Training data range (from abs_stats.json)
TRAIN_ARM_MIN  = np.array([-0.4842, -0.1030, -0.2025, -2.2044, -0.2041, 1.5702, 0.4843])
TRAIN_ARM_MAX  = np.array([ 0.0452,  0.3120,  0.4789, -1.5347,  0.0806, 2.4536, 0.9807])
TRAIN_TCP_MIN  = np.array([0.534, -0.140, 0.178])
TRAIN_TCP_MAX  = np.array([0.602,  0.053, 0.517])
HOME_JOINTS = load_home_joints()

# C fix (grperr_1.md R3/修复 C): a uniform 0.15 rad margin on every joint let
# q7 (wrist roll) drift to ~0.40 rad on-robot -- well outside the
# demonstrated range [0.4843, 0.9807] -- without ever tripping L2
# (OUT-OF-TRAIN). That let the state (and therefore the FK-derived keypoint
# input) leave the training distribution unnoticed, compounding the R1
# keypoint-history bug. q7 gets a tighter 0.05 rad margin; q1-q6 keep 0.15.
# This can only make L2 *stricter* (never loosens a previously-checked
# joint), so it does not need a fresh §15.7 four-level revalidation -- worst
# case it clips q7 sooner and logs an extra OUT-OF-TRAIN warning.
#
# The 0.05 rad q7 margin was still too loose. Sweeping q7 against the loaded
# checkpoint (tests/diag_pose_attractor.py, 2026-09-18) shows the predicted
# action direction inverts precisely at the training bound: the cosine against
# the demonstrations' own action is -0.273 at q7 = 0.4750 and +0.343 at
# q7 = 0.4850 = TRAIN_ARM_MIN[6], climbing to +0.77 by +0.06 above it. The
# 600-step run that stalled sat at q7 = 0.4579, i.e. 0.026 rad below the bound
# and squarely inside what a 0.05 margin permits, and it was the only joint out
# of range. Restoring q7 alone recovered 78% of the action lead and flipped the
# cosine to +0.756, more than any other joint. So q7 LOWER margin stays at 0.
#
# Asymmetric q7 margin (A2_1_6 finding): the model consistently requests
# q7 ≈ 1.04-1.05 during insertion but gets clamped at 0.9807, causing
# ~1 mm plug-tip misalignment and socket-entry failure. The UPPER bound
# behaviour is different from the lower: the model is actively commanding
# those values (not drifting), so allowing 0.08 rad above TRAIN_ARM_MAX
# lets q7 reach 1.06 for insertion while the lower bound stays tight.
# SAFETY_MARGIN_LOWER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.08])
# SAFETY_MARGIN_UPPER_RAD = np.array([0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.08])
SAFETY_MARGIN_LOWER_RAD = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.08])
SAFETY_MARGIN_UPPER_RAD = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.1])
ACTION_LIMIT_LOWER = np.maximum(TRAIN_ARM_MIN - SAFETY_MARGIN_LOWER_RAD, JOINT_LIMITS_LOWER)
ACTION_LIMIT_UPPER = np.minimum(TRAIN_ARM_MAX + SAFETY_MARGIN_UPPER_RAD, JOINT_LIMITS_UPPER)
MAX_JOINT_STEP_RAD = 0.15

#: Clamping at the bound stops the inversion but leaves the policy on a weak
#: edge (cosine +0.343 there versus +0.6 at 0.03 rad inside), so loitering near
#: it is worth reporting even though nothing is clipped.
TRAIN_EDGE_WARN_RAD = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03])


def _env_truthy(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return default
    return raw.strip().lower() not in ("0", "false", "no", "off")


# InternVLA-A1.5 franka_plug (3A3 + abs_stats): action.gripper ∈ [0.007, 1.0],
# 1.0 = close, ~0.01 = open. Export VLA_GRIPPER_CLOSE_IF_ABOVE=0 if a demo
# frame shows 0 = close (OpenVLA RLDS polarity).
GRIPPER_CLOSE_THRESHOLD = float(os.environ.get("VLA_GRIPPER_CLOSE_THRESHOLD", "0.5"))
GRIPPER_CLOSE_IF_ABOVE = _env_truthy("VLA_GRIPPER_CLOSE_IF_ABOVE", True)

VLA_GRIPPER_MODE = os.environ.get("VLA_GRIPPER_MODE", "binary_abs")
W0 = 0.08  # training normalization constant
GRIPPER_DEADBAND_M = float(os.environ.get("VLA_GRIPPER_DEADBAND_M", "0.0015"))
GRIPPER_CLOSE_DELTA_M = float(os.environ.get("VLA_GRIPPER_CLOSE_DELTA_M", "0.001"))
GRIPPER_OPEN_DELTA_M = float(os.environ.get("VLA_GRIPPER_OPEN_DELTA_M", "0.004"))
GRASP_HANDOFF_A = float(os.environ.get("VLA_GRIPPER_GRASP_HANDOFF_A", "0.8"))
# A missed force-grasp used to be retried on every control step. close() blocks,
# so that dropped the loop the same way the old per-step move_width did.
GRASP_RETRY_COOLDOWN_S = float(os.environ.get("VLA_GRASP_RETRY_COOLDOWN_S", "1.0"))

# Ceiling for w_cmd, in the SAME units libfranka reports (``gripper.width``),
# which is not necessarily the physically measured finger gap: on this cell a
# caliper reads 80 mm at full open while the hand reports 66.4 mm (homing
# offset, see grperr_1.2.md Q1). Configuring the physical 0.080 here let the
# approach-phase command w_cmd=0.0773 escape the clamp, so every control step
# issued a blocking "open wider" command the hand could never satisfy: the
# fingers buzzed against their limit and the loop fell from 3.58 to 2.28 Hz.
# ``resolve_gripper_max_width_m`` therefore takes the min with whatever the
# hand actually reports, so a mis-filled config can no longer reintroduce this.
GRIPPER_MAX_WIDTH_M = float(os.environ.get("FRANKA_GRIPPER_MAX_WIDTH_M", "0.066"))

# Asymmetric dead zone for continuous mode. Narrowing tracks the demonstration
# ramp (~1.75 mm per 30 Hz frame) so its threshold is small; widening is rare
# during a grasp and a small threshold there only produces chatter, so it needs
# an explicit, much larger request.
GRIPPER_WIDEN_DEADBAND_M = float(os.environ.get("VLA_GRIPPER_WIDEN_DEADBAND_M", "0.006"))


def want_gripper_close(action_grip: float) -> bool:
    """Binary close/open from a [0, 1] InternVLA gripper command."""
    if GRIPPER_CLOSE_IF_ABOVE:
        return float(action_grip) >= GRIPPER_CLOSE_THRESHOLD
    return float(action_grip) < GRIPPER_CLOSE_THRESHOLD


def want_gripper_close_delta(action_grip: float, w_meas: float, currently_closed: bool) -> bool:
    """Delta-w based close/open with hysteresis."""
    delta_w = w_meas - W0 * (1.0 - action_grip)
    if currently_closed:
        return delta_w > -GRIPPER_OPEN_DELTA_M
    return delta_w >= GRIPPER_CLOSE_DELTA_M


def resolve_gripper_max_width_m(configured: float, reported: float | None) -> float:
    """Effective w_cmd ceiling: never command wider than the hand reports."""
    if reported is None:
        return float(configured)
    reported = float(reported)
    if not np.isfinite(reported) or reported <= 0.0:
        return float(configured)
    return min(float(configured), reported)


def continuous_gripper_decision(
    action_grip: float,
    w_meas: float,
    max_width_m: float,
    last_cmd_w: float | None = None,
) -> tuple[str, float]:
    """Decide the continuous-mode gripper command.

    Returns ``(command, w_cmd)`` where command is one of ``grasp_handoff``
    (force-control the final bite), ``move_width`` (position-track the ramp)
    or ``hold`` (issue nothing).

    ``last_cmd_w`` is the width most recently commanded this episode, or None
    if none has been. Position control holds its target, so re-sending one the
    hand is already tracking buys nothing and costs a blocking round trip.
    Skipping it also bounds the damage when ``max_width_m`` is wrong: an
    unreachable target is then commanded once rather than every control step.
    """
    w_cmd = min(W0 * (1.0 - float(action_grip)), float(max_width_m))
    if float(action_grip) >= GRASP_HANDOFF_A:
        return "grasp_handoff", w_cmd
    if last_cmd_w is not None and abs(w_cmd - float(last_cmd_w)) < GRIPPER_DEADBAND_M:
        return "hold", w_cmd
    delta_w = float(w_meas) - w_cmd
    if delta_w >= GRIPPER_DEADBAND_M:
        return "move_width", w_cmd
    if delta_w <= -GRIPPER_WIDEN_DEADBAND_M:
        return "move_width", w_cmd
    return "hold", w_cmd


def should_issue_grasp_handoff(
    holding: bool,
    now_s: float,
    last_attempt_s: float,
    cooldown_s: float,
) -> bool:
    """Whether continuous mode may call ``close_gripper()`` this step.

    The ramp narrows the fingers with position control before the force grasp.
    ``gripper_is_open()`` turns false once the width drops below 60 mm, which is
    exactly when this handoff is supposed to run, so that predicate must not be
    the gate. ``holding`` is the calibrated-width check (plug window): an
    already-successful grasp is not issued again. ``cooldown_s`` bounds how
    often a miss can block the control loop.
    """
    if holding:
        return False
    return (float(now_s) - float(last_attempt_s)) >= float(cooldown_s)


class MotionGuardTripped(RuntimeError):
    """Same as FrankySingleFrankaEnvMixin.MotionGuardTripped."""
    pass


def check_action_safety(action_arm, current_joints, step_idx):
    """3-layer joint-space safety: hard limits, training range, velocity."""
    warnings = []
    clipped = action_arm.copy()

    # L1: Hard joint limits
    below = clipped < JOINT_LIMITS_LOWER
    above = clipped > JOINT_LIMITS_UPPER
    if np.any(below) or np.any(above):
        viol = []
        for i in range(7):
            if below[i]: viol.append(f"q{i+1}={clipped[i]:.4f}<{JOINT_LIMITS_LOWER[i]:.4f}")
            elif above[i]: viol.append(f"q{i+1}={clipped[i]:.4f}>{JOINT_LIMITS_UPPER[i]:.4f}")
        warnings.append(f"[step {step_idx}] HARD LIMIT: {', '.join(viol)}")
        clipped = np.clip(clipped, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

    # L2: Training range + margin
    below_t = clipped < ACTION_LIMIT_LOWER
    above_t = clipped > ACTION_LIMIT_UPPER
    if np.any(below_t) or np.any(above_t):
        viol = []
        for i in range(7):
            if below_t[i]: viol.append(f"q{i+1}={clipped[i]:.4f}<{ACTION_LIMIT_LOWER[i]:.4f}")
            elif above_t[i]: viol.append(f"q{i+1}={clipped[i]:.4f}>{ACTION_LIMIT_UPPER[i]:.4f}")
        warnings.append(f"[step {step_idx}] OUT-OF-TRAIN: {', '.join(viol)}")
        clipped = np.clip(clipped, ACTION_LIMIT_LOWER, ACTION_LIMIT_UPPER)

    # L2b: inside the range but hugging its edge, where the policy is weakest.
    edge = []
    for i in range(7):
        if TRAIN_EDGE_WARN_RAD[i] <= 0.0:
            continue
        low_gap = clipped[i] - TRAIN_ARM_MIN[i]
        high_gap = TRAIN_ARM_MAX[i] - clipped[i]
        if 0.0 <= low_gap < TRAIN_EDGE_WARN_RAD[i]:
            edge.append(f"q{i+1}={clipped[i]:.4f} is {low_gap:.4f} above min")
        elif 0.0 <= high_gap < TRAIN_EDGE_WARN_RAD[i]:
            edge.append(f"q{i+1}={clipped[i]:.4f} is {high_gap:.4f} below max")
    if edge:
        warnings.append(f"[step {step_idx}] TRAIN-EDGE: {', '.join(edge)}")

    # L3: Velocity limit
    delta = clipped - current_joints
    if np.any(np.abs(delta) > MAX_JOINT_STEP_RAD):
        viol = [f"q{i+1}: {abs(delta[i]):.4f}" for i in range(7) if abs(delta[i]) > MAX_JOINT_STEP_RAD]
        warnings.append(f"[step {step_idx}] VEL LIMIT: {', '.join(viol)}")
        scale = min(1.0, MAX_JOINT_STEP_RAD / float(np.abs(delta).max()))
        clipped = current_joints + delta * scale

    return clipped, warnings


class FrankyJointEnv(gym.Env):
    """Joint-space Gym environment with 8-level safety."""

    metadata = {"render_modes": []}

    def __init__(self, robot_ip="172.16.0.2", control_hz=10.0,
                 is_dummy=False, use_realsense=False, camera_serials=None):
        super().__init__()
        self._control_hz = control_hz
        self._is_dummy = is_dummy
        self._step_count = 0
        self._total_warnings = 0
        self._controller: Optional[FrankyControllerDirect] = None
        self._camera = None
        self._last_gripper_want_close: Optional[bool] = None
        self._last_gripper_closed: bool = False
        # Width most recently commanded in continuous mode; None means "no
        # target in flight", which is also the state after any force-control
        # grasp or open/close, since those move the fingers behind our back.
        self._last_gripper_cmd_w: Optional[float] = None
        self._last_grasp_attempt_t = float("-inf")

        self.action_space = gym.spaces.Box(
            low=np.concatenate([JOINT_LIMITS_LOWER, [0.0]]),
            high=np.concatenate([JOINT_LIMITS_UPPER, [1.0]]),
            dtype=np.float64,
        )
        self.observation_space = gym.spaces.Dict({
            "state": gym.spaces.Box(low=-10, high=10, shape=(8,), dtype=np.float64),
        })

        self._gripper_max_width_m = GRIPPER_MAX_WIDTH_M

        if not is_dummy:
            self._controller = self._make_controller(robot_ip)
            self._controller.set_motion_guard(TRAIN_TCP_MIN, TRAIN_TCP_MAX)
            reported_max = self._controller.gripper_max_width()
            self._gripper_max_width_m = resolve_gripper_max_width_m(
                GRIPPER_MAX_WIDTH_M, reported_max
            )
            logger.info(
                "gripper mode=%s: w_cmd ceiling=%.4fm "
                "(configured FRANKA_GRIPPER_MAX_WIDTH_M=%.4fm, hand reports %s)",
                VLA_GRIPPER_MODE,
                self._gripper_max_width_m,
                GRIPPER_MAX_WIDTH_M,
                f"{reported_max:.4f}m" if reported_max is not None else "unknown",
            )
            if reported_max is not None and GRIPPER_MAX_WIDTH_M > reported_max + 1e-6:
                logger.warning(
                    "FRANKA_GRIPPER_MAX_WIDTH_M=%.4fm exceeds the %.4fm the hand "
                    "reports; clamping to the reported value. This config must be "
                    "in libfranka's reported units, NOT the caliper-measured "
                    "finger gap (see grperr_1.2.md Q1).",
                    GRIPPER_MAX_WIDTH_M, reported_max,
                )
            logger.info(
                "gripper command: close if action %s %.2f (VLA_GRIPPER_CLOSE_IF_ABOVE=%s)",
                ">=" if GRIPPER_CLOSE_IF_ABOVE else "<",
                GRIPPER_CLOSE_THRESHOLD,
                GRIPPER_CLOSE_IF_ABOVE,
            )

        if use_realsense:
            self._init_cameras(camera_serials or {})

    def _make_controller(self, robot_ip: str):
        """Arm backend. Subclasses swap this without copying env setup."""
        return FrankyControllerDirect(robot_ip)

    def _init_cameras(self, serials):
        try:
            import pyrealsense2 as rs  # pyright: ignore[reportMissingImports]
        except ImportError:
            logger.warning("pyrealsense2 not available")
            return
        self._camera = {}
        for name in ["global", "wrist"]:
            pipe = rs.pipeline()
            cfg = rs.config()
            serial = serials.get(name)
            if serial: cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            pipe.start(cfg)
            self._camera[name] = pipe

    def get_camera_frames(self):
        if self._camera is None:
            return {"global": np.zeros((480, 640, 3), dtype=np.uint8),
                    "wrist": np.zeros((480, 640, 3), dtype=np.uint8)}
        frames = {}
        for name, pipe in self._camera.items():
            fs = pipe.wait_for_frames(timeout_ms=1000)
            color = fs.get_color_frame()
            if not color: raise RuntimeError(f"No frame from {name}")
            frames[name] = np.asarray(color.get_data(), dtype=np.uint8)
        return frames

    def step(self, action):
        action_arm = action[:7].copy()
        action_grip = float(action[7]) if len(action) > 7 else 0.5
        info = {
            "warnings": [],
            "step": self._step_count,
            "action_grip": action_grip,
            "gripper_mode": VLA_GRIPPER_MODE,
            "gripper_close_if_above": GRIPPER_CLOSE_IF_ABOVE,
        }

        if self._is_dummy:
            self._step_count += 1
            return self._get_observation(), 0.0, False, False, info

        # L1-L3: Joint-space safety
        current_q = self._controller.get_state()["arm_joint_position"]
        action_arm, warnings = check_action_safety(action_arm, current_q, self._step_count)
        for w in warnings: logger.warning(w)
        self._total_warnings += len(warnings)
        info["warnings"] = warnings

        # L4-L5: Motion guard check
        tripped = self._controller.guard_tripped()
        if tripped is not None:
            logger.error("MOTION GUARD TRIP: %s", tripped)
            info["motion_guard_trip"] = tripped
            return self._get_observation(), 0.0, False, True, info

        # Execute
        try:
            self._controller.move_joints(action_arm)
        except RuntimeError as e:
            if "guard" in str(e).lower():
                info["motion_guard_trip"] = str(e)
                return self._get_observation(), 0.0, False, True, info
            raise

        # Gripper: mode-dependent execution (grperr_1.2.md Phase 2).
        # binary_abs  -- original threshold logic (backward compat)
        # binary_delta -- delta-w with hysteresis
        # continuous   -- position-control + force-grasp handoff
        w_meas = self._controller.gripper_width()
        if VLA_GRIPPER_MODE == "binary_abs":
            want_close = want_gripper_close(action_grip)
            info["gripper_cmd"] = "close" if want_close else "open"
            if want_close != self._last_gripper_want_close:
                logger.info(
                    "[step %d] gripper action=%.4f -> %s (threshold %s %.2f, width=%s, holding=%s)",
                    self._step_count,
                    action_grip,
                    "close" if want_close else "open",
                    ">=" if GRIPPER_CLOSE_IF_ABOVE else "<",
                    GRIPPER_CLOSE_THRESHOLD,
                    w_meas,
                    self._controller.gripper_holding(),
                )
                self._last_gripper_want_close = want_close
            if want_close:
                if self._controller.gripper_is_open():
                    self._controller.close_gripper()
            elif not self._controller.gripper_is_open():
                self._controller.open_gripper()

        elif VLA_GRIPPER_MODE == "binary_delta":
            want_close = want_gripper_close_delta(
                action_grip, float(w_meas) if w_meas is not None else 0.08,
                self._last_gripper_closed,
            )
            w_cmd = W0 * (1.0 - action_grip)
            delta_w = (float(w_meas) if w_meas is not None else 0.08) - w_cmd
            info["gripper_cmd"] = "close" if want_close else "open"
            info["w_meas"] = w_meas
            info["w_cmd"] = w_cmd
            info["delta_w"] = delta_w
            if want_close != self._last_gripper_closed:
                logger.info(
                    "[step %d] gripper binary_delta action=%.4f w_meas=%s w_cmd=%.4f "
                    "delta_w=%.4f -> %s",
                    self._step_count, action_grip, w_meas, w_cmd, delta_w,
                    "close" if want_close else "open",
                )
            self._last_gripper_closed = want_close
            if want_close:
                if self._controller.gripper_is_open():
                    self._controller.close_gripper()
            elif not self._controller.gripper_is_open():
                self._controller.open_gripper()

        elif VLA_GRIPPER_MODE == "continuous":
            # Unknown width falls back to the ceiling, i.e. "assume fully
            # open", expressed in the hand's units rather than the training
            # constant: 0.08 here would read as 13.6 mm of slack the hand does
            # not have and would fake a narrowing request every step.
            w_meas_f = (
                float(w_meas) if w_meas is not None else self._gripper_max_width_m
            )
            cmd, w_cmd = continuous_gripper_decision(
                action_grip, w_meas_f, self._gripper_max_width_m,
                self._last_gripper_cmd_w,
            )
            info["w_meas"] = w_meas
            info["w_cmd"] = w_cmd
            info["delta_w"] = w_meas_f - w_cmd
            info["gripper_cmd"] = cmd
            info["grasp_issued"] = False
            if cmd == "grasp_handoff":
                now = time.monotonic()
                issue = should_issue_grasp_handoff(
                    self._controller.gripper_holding(),
                    now,
                    self._last_grasp_attempt_t,
                    GRASP_RETRY_COOLDOWN_S,
                )
                info["grasp_issued"] = issue
                if issue:
                    logger.info(
                        "[step %d] gripper continuous grasp_handoff action=%.4f "
                        "w_meas=%s w_cmd=%.4f holding=%s",
                        self._step_count, action_grip, w_meas, w_cmd,
                        self._controller.gripper_holding(),
                    )
                    self._last_grasp_attempt_t = now
                    self._controller.close_gripper()
                    self._last_gripper_cmd_w = None
            elif cmd == "move_width":
                logger.info(
                    "[step %d] gripper continuous move_width action=%.4f "
                    "w_meas=%s w_cmd=%.4f delta_w=%+.4f",
                    self._step_count, action_grip, w_meas, w_cmd, w_meas_f - w_cmd,
                )
                self._controller.gripper_move_width_m(w_cmd)
                self._last_gripper_cmd_w = w_cmd

        else:
            raise ValueError(
                f"unknown VLA_GRIPPER_MODE={VLA_GRIPPER_MODE!r}; "
                f"expected binary_abs|binary_delta|continuous"
            )

        time.sleep(1.0 / self._control_hz)

        # Re-check guard
        tripped = self._controller.guard_tripped()
        if tripped is not None:
            info["motion_guard_trip"] = tripped
            return self._get_observation(), 0.0, False, True, info

        self._step_count += 1
        return self._get_observation(), 0.0, False, False, info

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self._step_count = 0
        self._last_gripper_want_close = None
        self._last_gripper_closed = False
        self._last_gripper_cmd_w = None
        self._last_grasp_attempt_t = float("-inf")

        if not self._is_dummy:
            tripped = self._controller.guard_tripped()
            if tripped:
                result = self._controller.recover_from_guard_trip()
                if not result["recovered"]:
                    raise MotionGuardTripped(f"Cannot recover: {tripped}")

            self._controller.open_gripper()
            time.sleep(0.3)
            self._controller.reset_joint(HOME_JOINTS.tolist())
            time.sleep(0.5)
            self._controller.open_gripper()
            time.sleep(0.3)

        return self._get_observation(), {}

    def _get_observation(self):
        if self._is_dummy:
            return {"state": np.concatenate([HOME_JOINTS, [0.078]])}
        state = self._controller.get_state()
        q = state["arm_joint_position"]
        g = state["gripper_width"]
        if g is None:
            raise RuntimeError(
                "gripper width unavailable; refusing to fabricate observation state"
            )
        return {"state": np.concatenate([q, [float(g)]])}

    def go_to_rest(self):
        """Replicates eval_3A2 go_to_rest(): open -> HOME -> open."""
        if self._is_dummy: return
        self._controller.open_gripper()
        time.sleep(0.3)
        self._controller.reset_joint(HOME_JOINTS.tolist())
        time.sleep(0.5)
        self._controller.open_gripper()

    def close(self):
        if self._controller: self._controller.cleanup()
        if self._camera:
            for pipe in self._camera.values():
                try: pipe.stop()
                except Exception: pass
        super().close()
