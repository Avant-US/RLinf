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
import sys
import time
from pathlib import Path
from typing import Optional

import gymnasium as gym
import numpy as np

# 4dwvla_ext 以数字开头, 不能用 from 4dwvla_ext.X import Y
# 直接把本目录加入 sys.path 后按模块名导入
sys.path.insert(0, str(Path(__file__).resolve().parent))

from franky_controller_direct import (
    FrankyControllerDirect,
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    JOINT_VEL_LIMITS,
)

logger = logging.getLogger(__name__)

# Training data range (from abs_stats.json)
TRAIN_ARM_MIN  = np.array([-0.4842, -0.1030, -0.2025, -2.2044, -0.2041, 1.5702, 0.4843])
TRAIN_ARM_MAX  = np.array([ 0.0452,  0.3120,  0.4789, -1.5347,  0.0806, 2.4536, 0.9807])
TRAIN_ARM_MEAN = np.array([-0.2406,  0.1457,  0.1872, -2.0600, -0.0553, 2.2011, 0.6998])
TRAIN_TCP_MIN  = np.array([0.534, -0.140, 0.178])
TRAIN_TCP_MAX  = np.array([0.602,  0.053, 0.517])
HOME_JOINTS = TRAIN_ARM_MEAN.copy()

SAFETY_MARGIN_RAD = 0.15
ACTION_LIMIT_LOWER = np.maximum(TRAIN_ARM_MIN - SAFETY_MARGIN_RAD, JOINT_LIMITS_LOWER)
ACTION_LIMIT_UPPER = np.minimum(TRAIN_ARM_MAX + SAFETY_MARGIN_RAD, JOINT_LIMITS_UPPER)
MAX_JOINT_STEP_RAD = 0.15
GRIPPER_CLOSE_THRESHOLD = 0.5


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

        self.action_space = gym.spaces.Box(
            low=np.concatenate([JOINT_LIMITS_LOWER, [0.0]]),
            high=np.concatenate([JOINT_LIMITS_UPPER, [1.0]]),
            dtype=np.float64,
        )
        self.observation_space = gym.spaces.Dict({
            "state": gym.spaces.Box(low=-10, high=10, shape=(8,), dtype=np.float64),
        })

        if not is_dummy:
            self._controller = FrankyControllerDirect(robot_ip)
            self._controller.set_motion_guard(TRAIN_TCP_MIN, TRAIN_TCP_MAX)

        if use_realsense:
            self._init_cameras(camera_serials or {})

    def _init_cameras(self, serials):
        try:
            import pyrealsense2 as rs
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
        info = {"warnings": [], "step": self._step_count}

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

        # Gripper
        if action_grip >= GRIPPER_CLOSE_THRESHOLD:
            self._controller.close_gripper()
        else:
            self._controller.open_gripper()

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
            return {"state": np.concatenate([HOME_JOINTS, [0.04]])}
        state = self._controller.get_state()
        q = state["arm_joint_position"]
        g = state["gripper_width"] or 0.04
        return {"state": np.concatenate([q, [g]])}

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
