"""Franka parallel-jaw gripper via libfranka / franky (no ROS)."""

from __future__ import annotations

from rlinf.envs.realworld.common.gripper.base_gripper import BaseGripper
from rlinf.utils.logging import get_logger

_MAX_WIDTH_M = 0.09
_CLOSE_WIDTH_M = 0.01
_DEFAULT_GRASP_FORCE = 130.0


class FrankaLibfrankaGripper(BaseGripper):
    """Franka Hand via franky.Gripper (libfranka, no ROS)."""

    def __init__(self, robot_ip: str):
        import franky

        self._logger = get_logger()
        self._gripper = franky.Gripper(robot_ip)
        self._is_open_flag = True
        max_w = getattr(self._gripper, "max_width", None)
        self._logger.info(
            "FrankaLibfrankaGripper connected (max_width=%s)",
            f"{max_w:.3f}m" if max_w is not None else "unknown",
        )

    def open(self, speed: float = 0.3) -> None:
        self._gripper.move(_MAX_WIDTH_M, speed)
        self._is_open_flag = True

    def close(self, speed: float = 0.3, force: float = _DEFAULT_GRASP_FORCE) -> None:
        self._gripper.grasp(
            width=_CLOSE_WIDTH_M,
            speed=speed,
            force=force,
            epsilon_inner=0.05,
            epsilon_outer=0.05,
        )
        self._is_open_flag = False

    def move(self, position: float, speed: float = 0.3) -> None:
        width = float(position / (255 * 10))
        width = max(0.0, min(width, _MAX_WIDTH_M))
        self._gripper.move(width, speed)

    @property
    def position(self) -> float:
        return float(self._gripper.width)

    @property
    def is_open(self) -> bool:
        return self._is_open_flag

    def is_ready(self) -> bool:
        try:
            _ = self._gripper.width
            return True
        except Exception:
            return False

    def cleanup(self) -> None:
        pass
