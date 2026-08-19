"""Franka parallel-jaw gripper via libfranka / franky (no ROS)."""

from __future__ import annotations

import os

from rlinf.envs.realworld.common.gripper.base_gripper import BaseGripper
from rlinf.utils.logging import get_logger

_MAX_WIDTH_M = 0.09
_CLOSE_WIDTH_M = 0.01
# libfranka grasp() keeps applying ``force`` after contact. 130 N was copied
# from ROS FrankaGripper and will crush a small cube (and pinch a finger).
# 20 N is enough to hold a light cube; cap at 40 N unless explicitly raised.
_DEFAULT_GRASP_FORCE_N = 20.0
_MAX_GRASP_FORCE_N = 40.0
_MAX_CLOSE_SPEED_M_S = 0.08
_OPEN_WIDTH_M = 0.06


def _is_command_failed(exc: BaseException) -> bool:
    text = str(exc)
    return "Command failed" in text or type(exc).__name__ == "CommandException"


def _grasp_force_n(requested: float | None) -> float:
    raw = os.environ.get("FRANKA_GRASP_FORCE")
    if requested is None:
        value = float(raw) if raw else _DEFAULT_GRASP_FORCE_N
    else:
        value = float(requested)
    return min(max(value, 5.0), _MAX_GRASP_FORCE_N)


def _close_speed_m_s(requested: float) -> float:
    return min(max(float(requested), 0.01), _MAX_CLOSE_SPEED_M_S)


class FrankaLibfrankaGripper(BaseGripper):
    """Franka Hand via franky.Gripper (libfranka, no ROS)."""

    def __init__(self, robot_ip: str):
        import franky

        self._logger = get_logger()
        self._gripper = franky.Gripper(robot_ip)
        self._is_open_flag = not self._hardware_holding()
        max_w = getattr(self._gripper, "max_width", None)
        self._logger.info(
            "FrankaLibfrankaGripper connected (max_width=%s, grasp_force=%.1fN, "
            "holding=%s width=%s)",
            f"{max_w:.3f}m" if max_w is not None else "unknown",
            _grasp_force_n(None),
            self._hardware_holding(),
            f"{self._width_m():.4f}m" if self._width_m() is not None else "unknown",
        )

    def _width_m(self) -> float | None:
        try:
            return float(self._gripper.width)
        except Exception:
            return None

    def _hardware_holding(self) -> bool:
        """True if fingers already have an object (or a prior grasp)."""
        grasped = getattr(self._gripper, "is_grasped", None)
        try:
            if callable(grasped):
                if bool(grasped()):
                    return True
            elif grasped is not None and bool(grasped):
                return True
        except Exception:
            pass
        width = self._width_m()
        if width is None:
            return False
        return 0.004 <= width <= 0.055

    def open(self, speed: float = 0.3) -> None:
        self._gripper.move(_MAX_WIDTH_M, speed)
        self._is_open_flag = True

    def close(self, speed: float = 0.05, force: float | None = None) -> None:
        if self._hardware_holding():
            self._logger.info(
                "gripper already holding (width=%s); skip grasp",
                f"{self._width_m():.4f}m" if self._width_m() is not None else "unknown",
            )
            self._is_open_flag = False
            return
        speed_m_s = _close_speed_m_s(speed)
        force_n = _grasp_force_n(force)
        self._logger.info(
            "gripper grasp: width=%.3fm speed=%.3fm/s force=%.1fN (held after contact)",
            _CLOSE_WIDTH_M,
            speed_m_s,
            force_n,
        )
        try:
            self._gripper.grasp(
                width=_CLOSE_WIDTH_M,
                speed=speed_m_s,
                force=force_n,
                epsilon_inner=0.05,
                epsilon_outer=0.05,
            )
        except Exception as exc:
            if _is_command_failed(exc) and self._hardware_holding():
                self._logger.warning(
                    "grasp Command failed but fingers still holding; continue. err=%s",
                    exc,
                )
            else:
                raise
        self._is_open_flag = False

    def move(self, position: float, speed: float = 0.3) -> None:
        width = float(position / (255 * 10))
        width = max(0.0, min(width, _MAX_WIDTH_M))
        self._gripper.move(width, speed)

    def stop(self) -> None:
        stop_fn = getattr(self._gripper, "stop", None)
        if not callable(stop_fn):
            raise RuntimeError("franky.Gripper has no stop()")
        stop_fn()

    @property
    def position(self) -> float:
        return float(self._gripper.width)

    @property
    def is_open(self) -> bool:
        if self._hardware_holding():
            return False
        width = self._width_m()
        if width is not None:
            return width >= _OPEN_WIDTH_M
        return self._is_open_flag

    def is_ready(self) -> bool:
        try:
            _ = self._gripper.width
            return True
        except Exception:
            return False

    def cleanup(self) -> None:
        pass
