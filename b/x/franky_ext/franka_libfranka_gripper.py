"""Franka parallel-jaw gripper via libfranka / franky (no ROS).

Three properties this file has to get right, each learned the hard way:

**Grasp force is a hold force, not a close force.** libfranka's ``grasp`` keeps
applying ``force`` after contact. 130 N (copied from ROS ``FrankaGripper``) will
crush a small cube and pinch a finger (LOG-008), so the default is 20 N with a
hard 40 N cap.

**"Am I holding something" must be decided by measured width, never by
``is_grasped``.** libfranka reports ``is_grasped`` when the final width lands in
``[width - epsilon_inner, width + epsilon_outer]``. With the previous
``width=0.01, epsilon=0.05`` that window was ``[-0.04, 0.06]`` -- and the hand's
entire travel is 0 to 0.08 m, so **every** terminal width under 6 cm satisfied it,
including 0.000 with the fingers closed on air. A false ``is_grasped`` then
poisoned everything downstream: ``close()`` took its ``skip grasp`` branch
forever, ``is_open`` stayed ``False`` forever, and the smoke script's
"cube must be present" gate passed with an empty hand -- after which the arm runs
a hover-and-touch geometry calibrated for a cube that is not there, descending
until the fingertips rather than the cube reach the mark. So the grasp is now
commanded at the *calibrated cube width* with a tight window, and the result is
verified by re-reading the width.

**Every call here is a blocking libfranka round trip inside a Ray actor.** With no
timeout, an unresponsive hand blocks ``self._controller.close_gripper().wait()``
forever: ``reset`` hangs holding FCI with the arm live under impedance and no way
out short of killing the actor. All commands therefore run through a bounded
executor.
"""

from __future__ import annotations

import os
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as _FutureTimeout
from typing import Callable

from rlinf.envs.realworld.common.gripper.base_gripper import BaseGripper
from rlinf.utils.logging import get_logger

# cube_width_m / hold_tolerance_m live in motion_limits.py (round-2 audit finding
# 5) so tcp_probe.py's pre-Ray gate can decide "holding" from the same window
# without importing this module or duplicating the defaults in a string literal.
# Re-imported here so this module's own behaviour is unchanged.
from franky_ext.motion_limits import cube_width_m, env_float, hold_tolerance_m  # noqa: F401

_MAX_WIDTH_M = 0.09
_OPEN_WIDTH_M = 0.06

# libfranka grasp() keeps applying ``force`` after contact. 130 N was copied from
# ROS FrankaGripper and will crush a small cube (and pinch a finger).
_DEFAULT_GRASP_FORCE_N = 20.0
_MAX_GRASP_FORCE_N = 40.0
_GRASP_FORCE_RANGE = (5.0, _MAX_GRASP_FORCE_N)
_MAX_CLOSE_SPEED_M_S = 0.08

#: Bound on any single blocking gripper command, seconds. A grasp at the capped
#: 0.08 m/s crosses the full 8 cm of travel in ~1 s.
_GRIPPER_TIMEOUT_S = 6.0


def _grasp_force_n(requested: float | None) -> float:
    """Effective grasp force, clamped to ``[5, 40]`` N.

    Uses the tolerant reader: a malformed ``FRANKA_GRASP_FORCE`` (``"20N"``,
    ``"twenty"``, a stray space) used to raise ``ValueError`` from
    ``__init__``'s own log line, killing the gripper constructor, the controller
    constructor and the whole Ray actor with a traceback pointing at actor
    construction rather than at the typo.
    """
    if requested is None:
        return env_float(
            "FRANKA_GRASP_FORCE", _DEFAULT_GRASP_FORCE_N, _GRASP_FORCE_RANGE
        )
    return float(
        min(max(float(requested), _GRASP_FORCE_RANGE[0]), _GRASP_FORCE_RANGE[1])
    )


def _close_speed_m_s(requested: float) -> float:
    return min(max(float(requested), 0.01), _MAX_CLOSE_SPEED_M_S)


def _is_command_failed(exc: BaseException) -> bool:
    text = str(exc)
    return "Command failed" in text or type(exc).__name__ == "CommandException"


class FrankaLibfrankaGripper(BaseGripper):
    """Franka Hand via franky.Gripper (libfranka, no ROS)."""

    def __init__(self, robot_ip: str):
        import franky

        self._logger = get_logger()
        self._gripper = franky.Gripper(robot_ip)
        # One worker: a timed-out command stays in flight, so the next command
        # queues behind it and times out too. That degrades to bounded repeated
        # failures instead of an unbounded hang, which is the point.
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="franka-hand")
        self._is_open_flag = not self._hardware_holding()
        force = _grasp_force_n(None)
        if force > _DEFAULT_GRASP_FORCE_N:
            self._logger.warning(
                "grasp force %.1fN exceeds the %.1fN default; a light cube will be "
                "crushed and a finger can be pinched. Set FRANKA_GRASP_FORCE lower.",
                force,
                _DEFAULT_GRASP_FORCE_N,
            )
        max_w = getattr(self._gripper, "max_width", None)
        # LOG-034 (T17): logged explicitly, in THIS process's actual environment,
        # because the controller actor's env_configs.env_vars can silently fail to
        # arrive here (wrong node_group_label on launch) while everything upstream
        # -- the YAML, the printed Hydra config -- still shows the intended value.
        # "raw_env=None" means this process never saw the var at all and is
        # running on the hardcoded default below, whatever the YAML says.
        self._logger.info(
            "FrankaLibfrankaGripper connected (max_width=%s, grasp_force=%.1fN, "
            "cube_width=%.4fm +/-%.4fm [raw_env FRANKA_CUBE_WIDTH_M=%s "
            "FRANKA_HOLD_TOL_M=%s], holding=%s width=%s)",
            f"{max_w:.3f}m" if max_w is not None else "unknown",
            force,
            cube_width_m(),
            hold_tolerance_m(),
            os.environ.get("FRANKA_CUBE_WIDTH_M"),
            os.environ.get("FRANKA_HOLD_TOL_M"),
            self._hardware_holding(),
            f"{self._width_m():.4f}m" if self._width_m() is not None else "unknown",
        )

    # ------------------------------------------------------------------
    # Bounded blocking calls
    # ------------------------------------------------------------------

    def _call(self, label: str, fn: Callable, *args, **kwargs):
        """Run a blocking libfranka gripper command with a deadline."""
        future = self._pool.submit(fn, *args, **kwargs)
        try:
            return future.result(timeout=_GRIPPER_TIMEOUT_S)
        except _FutureTimeout:
            self._logger.error(
                "gripper %s did not finish in %.1fs; attempting stop()",
                label,
                _GRIPPER_TIMEOUT_S,
            )
            try:
                self._gripper.stop()
            except Exception as exc:  # noqa: BLE001 - best effort on a hung hand
                self._logger.error("gripper stop() also failed: %s", exc)
            raise RuntimeError(
                f"gripper {label} did not finish in {_GRIPPER_TIMEOUT_S:.1f}s "
                "(hand unresponsive). Not retrying: the arm is live under "
                "impedance while this blocks."
            ) from None

    def _width_m(self) -> float | None:
        try:
            return float(self._gripper.width)
        except Exception:
            return None

    def _hardware_holding(self) -> bool:
        """True when the measured width matches the calibrated cube.

        Deliberately does **not** consult ``is_grasped``: see the module
        docstring -- with any usable epsilon that flag cannot distinguish a cube
        from an empty hand closed on air, and a false positive here silently
        disables both ``close()`` and every "is the cube present" gate built on
        top of it.
        """
        width = self._width_m()
        if width is None:
            return False
        return abs(width - cube_width_m()) <= hold_tolerance_m()

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def open(self, speed: float = 0.3) -> None:
        """Open fully. Warns first if that means dropping a held object.

        The speed is clamped: upstream ``FrankyController.open_gripper`` asks for
        ``speed=1.0`` against a hand whose fingers top out near 0.1 m/s, which
        either gets clamped by libfranka or flings a held cube.
        """
        if self._hardware_holding():
            self._logger.warning(
                "open() while holding (width=%.4fm): the object WILL drop",
                self._width_m() or 0.0,
            )
        self._call("open", self._gripper.move, _MAX_WIDTH_M, _close_speed_m_s(speed))
        self._is_open_flag = True

    def close(self, speed: float = 0.05, force: float | None = None) -> None:
        """Grasp the calibrated cube gently, then verify by measured width.

        Skips entirely when already holding: libfranka fails a ``grasp`` issued
        while a grasp is being maintained (LOG-010).
        """
        if self._hardware_holding():
            self._logger.info(
                "gripper already holding (width=%.4fm); skip grasp",
                self._width_m() or 0.0,
            )
            self._is_open_flag = False
            return

        speed_m_s = _close_speed_m_s(speed)
        force_n = _grasp_force_n(force)
        target_w = cube_width_m()
        tol = hold_tolerance_m()
        self._logger.info(
            "gripper grasp: width=%.4fm +/-%.4fm speed=%.3fm/s force=%.1fN "
            "(force held after contact)",
            target_w,
            tol,
            speed_m_s,
            force_n,
        )
        try:
            self._call(
                "grasp",
                self._gripper.grasp,
                width=target_w,
                speed=speed_m_s,
                force=force_n,
                epsilon_inner=tol,
                epsilon_outer=tol,
            )
        except RuntimeError:
            raise  # timeout from _call: already logged, do not paper over it
        except Exception as exc:
            # Narrow swallow: continue ONLY if the measured width says a cube of
            # the calibrated size is actually between the fingers. The old version
            # accepted any width in [0.004, 0.055], so a cube that slipped out
            # mid-close and left the fingers anywhere in that range was reported
            # as held.
            width = self._width_m()
            if _is_command_failed(exc) and self._hardware_holding():
                self._logger.warning(
                    "grasp reported failure but width=%.4fm matches the cube; "
                    "continuing. err=%s",
                    width if width is not None else -1.0,
                    exc,
                )
            else:
                raise

        width = self._width_m()
        if width is None or abs(width - target_w) > tol:
            raise RuntimeError(
                f"grasp did not capture the cube: measured width="
                f"{'unknown' if width is None else f'{width:.4f}m'}, expected "
                f"{target_w:.4f}m +/-{tol:.4f}m. Either the hand is empty, or the "
                f"cube is a different size -- set FRANKA_CUBE_WIDTH_M to the "
                f"measured width and redo the H1 calibration."
            )
        self._is_open_flag = False

    def move(self, position: float, speed: float = 0.3) -> None:
        """Move to a width given as ``BaseGripper``'s 0-255 integer.

        The arithmetic ``position / (255 * 10)`` is inherited verbatim from
        upstream ``franka_gripper.py`` and is **inverted relative to the Robotiq
        implementation of the same interface**, where ``0 = open, 255 = closed``.
        Here 255 maps to 0.100 m, clamped to 0.09 -- i.e. fully open.

        That discrepancy is deliberately *not* silently flipped: two ``b/x``
        scripts already call ``move_gripper`` with the current sense, and changing
        what a number means without changing the callers is how a "fix" drops a
        cube. Instead the dangerous direction is refused outright, so whichever
        convention the caller believed, it cannot open onto the mark.
        """
        width = float(position) / (255.0 * 10.0)
        width = max(0.0, min(width, _MAX_WIDTH_M))
        held = self._width_m()
        if self._hardware_holding() and held is not None and width > held + 0.002:
            raise RuntimeError(
                f"refusing to widen the gripper to {width:.4f}m (from "
                f"{held:.4f}m) while holding the cube: it would drop. Note "
                f"position={position:g} means OPEN in this implementation "
                "(0=closed, 255=open), the opposite of the Robotiq one."
            )
        self._logger.info(
            "gripper move: position=%g -> width=%.4fm speed=%.3fm/s",
            float(position),
            width,
            _close_speed_m_s(speed),
        )
        self._call("move", self._gripper.move, width, _close_speed_m_s(speed))

    def stop(self) -> None:
        stop_fn = getattr(self._gripper, "stop", None)
        if not callable(stop_fn):
            raise RuntimeError("franky.Gripper has no stop()")
        # Not via _call: stop() is the escape hatch used *by* the timeout path.
        stop_fn()

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

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
        """Deliberately does NOT open the gripper.

        The Franka Hand maintains its grasp force mechanically after the control
        connection drops, so a process killed mid-episode leaves the cube held
        rather than dropping it onto the mark. That is the correct behaviour for a
        task whose whole premise is "the gripper never opens" -- do not "fix" this
        into an ``open()``.
        """
        try:
            self._pool.shutdown(wait=False)
        except Exception:  # noqa: BLE001 - teardown must not raise
            pass
