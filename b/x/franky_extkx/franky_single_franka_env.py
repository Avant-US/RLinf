"""Single-arm Franka env using FrankyControllerExtended (local extension).

Three jobs beyond swapping the controller:

**Speed-cap interpolated moves.** ``FrankaEnv._interpolate_move(pose, timeout=t)``
emits ``t * step_frequency`` waypoints over ``t`` seconds, so ``t`` *is* the
inverse of the commanded speed -- and upstream hardcodes ``t`` (1.0 or 1.5 s)
regardless of how far the move is. A 10 cm lift at ``timeout=1`` with
``step_frequency=10`` is a 10 cm/s command, ten times the only speed ever
validated on this arm, and that saturated the impedance error clip for the whole
lift (LOG-019 R1). Here the duration is derived from the displacement instead, so
the *speed* is the constant. A caller-supplied ``timeout`` is honoured only when
it is slower.

**Arm the motion guard.** The controller enforces a geometric fence on the
measured TCP, but only the env knows the safety box, so the env installs it once
the hardware exists -- before ``FrankaEnv.__init__`` performs its own
``_interpolate_move(self._reset_pose)``, which is otherwise the very first and
completely unbounded motion of a training run.

**Expose the per-waypoint ``recover_from_errors`` knob.** ``_move_action`` calls
``_clear_error()`` on every waypoint, i.e. ``automaticErrorRecovery`` at 10 Hz
inside a live 1 kHz torque motion (LOG-019 R5). Whether that is harmful is *not*
settled: LOG-013 proposed it, and LOG-014's refutation is void because that run
happened in ``RobotMode.UserStopped`` where nothing could move anyway. So the
default here stays at upstream behaviour and the knob exists to settle it by
measurement (``diag_franky_motion.py --test-recover-loop``), not by guesswork.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from rlinf.envs.realworld.franka.franka_env import FrankaEnv, FrankaRobotConfig
from rlinf.envs.realworld.franka.franky_controller import JOINT_VEL_LIMITS
from rlinf.envs.realworld.franka.end_effectors.base import normalize_end_effector_type
from rlinf.scheduler import FrankaHWInfo

from franky_ext.controller_extended import FrankyControllerExtended
from franky_ext.motion_limits import (
    INTERP_DURATION_S_RANGE,
    MAX_INTERP_ANGLE_RAD,
    MAX_INTERP_DISTANCE_M,
    env_float,
    guard_recovery_budget,
    interp_duration_s,
    interp_speed_m_s,
    jacobian_conditioning,
    joint_demand_scale,
    joint_vel_demand_fraction,
    max_action_scale_rot,
    max_action_scale_xyz,
    orientation_fence_rad,
    quat_angle_rad,
    sigma_min_warn,
    step_rot_speed_rad_s,
    step_speed_m_s,
)

#: Absolute ``+z`` fence headroom above the safety-box top, metres. Unlike the
#: ``extra_z_up`` term this does not move when an operator tunes
#: ``reset_z_lift_m``, so it is a fixed ceiling on where the arm can end up.
ABS_Z_CEILING_ABOVE_BOX_M = 0.10

#: How many times ``reset()`` may recover from a trip and try again within one
#: call. A trip during reset is far more serious than one during ``step()``: the
#: arm cannot get back to the rest pose, so retrying forever would just brake
#: against the same obstruction. Two attempts distinguishes "a transient caught
#: us mid-reset" from "this configuration cannot be reset out of", which is the
#: LOG-040 §40.1 case that needs a human with the enabling device.
RESET_TRIP_ATTEMPTS = 2

#: Banner used to make an operator-actionable line findable in a training log
#: that is otherwise a wall of per-epoch metrics tables.
_BANNER = "*" * 12


class MotionGuardTripped(RuntimeError):
    """A latched motion-guard trip, surfaced env-side.

    A distinct type so the recovery path in ``step``/``reset`` can catch *this*
    and nothing else. Catching bare ``RuntimeError`` there would also swallow the
    refusals raised a few lines away in ``_interpolate_move`` and
    ``_stretch_interp_for_joint_demand`` -- which must stay fatal, because they
    mean the arm is somewhere this task cannot safely drive it out of, and
    "recover and try again" would just re-refuse in a loop.

    Subclasses ``RuntimeError`` so every existing ``except RuntimeError`` around
    this code (the smoke scripts' teardown, mainly) behaves exactly as before.
    """


def _skip_camera() -> bool:
    return os.environ.get("RLINF_SKIP_CAMERA", "0").lower() in ("1", "true", "yes")


class FrankySingleFrankaEnvMixin:
    """Replace ROS FrankaController with FrankyControllerExtended."""

    def __init__(self, *args: Any, **kwargs: Any):
        # LOG-034/LOG-035 (T17): captured BEFORE super().__init__, which is what
        # calls _setup_hardware -> launch_controller. The FIRST attempt at this
        # fix read `kwargs["worker_info"].node_group_label` -- WRONG on two
        # counts, confirmed by LOG-035's real-machine retest still showing
        # node_group_label='cluster' on the controller placement: (1)
        # `worker_info` is FrankaEnv.__init__'s 2nd *positional* parameter, not
        # a kwarg, so `kwargs.get("worker_info")` was always None; (2) it is a
        # `WorkerInfo` dataclass (rlinf/scheduler/manager/worker_manager.py),
        # which has no `node_group_label` field at all -- unlike `Placement`
        # (rlinf/scheduler/placement/placement.py), which does. The actual
        # source of truth for "which node group was THIS worker process
        # launched into" is the `NODE_GROUP_LABEL` env var that
        # `WorkerGroup._launch_worker` sets on the worker's OS environment
        # (worker_group.py) -- the same one `Worker._init_node_group`
        # (scheduler/worker/worker.py) reads for its own `self._node_group`.
        # Reading os.environ here needs no positional/keyword guessing and no
        # dataclass shape assumptions.
        self._worker_node_group_label = os.environ.get("NODE_GROUP_LABEL") or None
        # Counted for the life of the env, not per episode: the budget is meant
        # to bound how much a run may lean on recovery in total (LOG-040 T21).
        self._guard_recoveries_used = 0
        self._in_franka_env_init = True
        try:
            super().__init__(*args, **kwargs)  # type: ignore[misc]
        finally:
            self._in_franka_env_init = False

    # ------------------------------------------------------------------
    # Motion authority
    # ------------------------------------------------------------------

    def arm_motion_guard(self) -> None:
        """Install the measured-TCP fence on the controller.

        ``+z`` gets ``reset_z_lift_m`` of extra headroom because ``go_to_rest``
        legitimately lifts above the box top before descending to hover, and
        upstream ``_interpolate_move`` does not clip to ``ee_pose_limit`` (only
        ``step()`` clips). That one excursion is allowed; nothing more is.

        On hardware an unusable box is a **refusal**, not a warning. The first
        version of this method logged "motion guard NOT armed" and let
        construction continue -- which is the same failure class as LOG-019
        itself: the mechanism that was supposed to bound the motion opts out, and
        the run proceeds. It is precisely the config shape most likely to be
        wrong (all zeros, e.g. any config that composes ``FrankaRobotConfig``
        without PegInsertion's ``__post_init__``), and it is followed immediately
        by ``FrankaEnv.__init__``'s own unbounded ``_interpolate_move``.
        """
        lim_min = np.asarray(self.config.ee_pose_limit_min, dtype=np.float64).reshape(-1)
        lim_max = np.asarray(self.config.ee_pose_limit_max, dtype=np.float64).reshape(-1)
        unusable = None
        if lim_min.size < 3 or lim_max.size < 3:
            unusable = f"ee_pose_limit is not a 6-vector (min={lim_min}, max={lim_max})"
        elif np.allclose(lim_min[:3], 0.0) and np.allclose(lim_max[:3], 0.0):
            unusable = (
                "ee_pose_limit is all zeros, so there is no safety box to fence. "
                "Set target_ee_pose and the clip_* ranges (or ee_pose_limit_min/max) "
                "in the config."
            )
        if unusable is not None:
            if not self.config.is_dummy:
                raise RuntimeError(
                    f"refusing to run on hardware without a motion guard: {unusable}"
                )
            self._logger.warning("motion guard NOT armed (dummy): %s", unusable)
            return

        lift = float(getattr(self.config, "reset_z_lift_m", 0.0) or 0.0)
        # Orientation is fenced by angle from the task target, not by comparing
        # euler to ee_pose_limit[3:]: this task's roll is ~-3.116 rad, a hair from
        # -pi, so euler comparison would abort on wrap-around. The three half
        # widths are NOT equal -- PegInsertionConfig.__post_init__ uses +/-0.01 on
        # roll and pitch and +/-clip_rz_range only on yaw -- so they are passed
        # separately rather than assuming clip_rz on all three.
        target_quat = None
        max_orient = None
        target = np.asarray(self.config.target_ee_pose, dtype=np.float64).reshape(-1)
        if target.size >= 6 and lim_min.size >= 6 and lim_max.size >= 6:
            halves = [
                float(abs(lim_max[3 + i] - lim_min[3 + i]) / 2.0) for i in range(3)
            ]
            if max(halves) > 0:
                target_quat = R.from_euler("xyz", target[3:6]).as_quat().tolist()
                max_orient = orientation_fence_rad(*halves)
        if max_orient is None and not self.config.is_dummy:
            raise RuntimeError(
                "refusing to run on hardware without an orientation fence: "
                f"target_ee_pose={target.tolist()} and ee_pose_limit rpy widths are "
                "degenerate. Set clip_rz_range > 0."
            )

        self._controller.set_motion_guard(
            lim_min[:3].tolist(),
            lim_max[:3].tolist(),
            extra_z_up=lift,
            z_ceiling=self._absolute_z_ceiling(lim_max),
            target_quat=target_quat,
            max_orient_err=max_orient,
        ).wait()
        health = self._controller.motion_health().wait()[0]
        if not health.get("guard_enabled"):
            raise RuntimeError(f"motion guard did not arm: {health}")
        self._logger.info(
            "motion guard confirmed: xyz in [%s, %s] orient<=%s",
            health.get("guard_min_xyz"),
            health.get("guard_max_xyz"),
            health.get("guard_max_orient_err"),
        )

    def _absolute_z_ceiling(self, lim_max: np.ndarray) -> float:
        """Absolute ``+z`` ceiling, independent of ``reset_z_lift_m``.

        ``extra_z_up`` moves the fence top whenever an operator tunes the lift, so
        the fence alone has no fixed upper bound on where the arm may end up. This
        adds one, the way the diagnostic script's ``Guards.z_ceiling`` does -- it is
        what would have stopped LOG-019 at a few centimetres instead of 36.
        """
        default = float(lim_max[2]) + ABS_Z_CEILING_ABOVE_BOX_M
        return env_float(
            "RLINF_CUBE_Z_CEILING", default, (float(lim_max[2]), float(lim_max[2]) + 0.40)
        )

    def _interpolate_move(self, pose: np.ndarray, timeout: float = 1.5):
        if (
            getattr(self.config, "safe_smoke_hold", False)
            and getattr(self, "_in_franka_env_init", False)
        ):
            self._logger.info("safe_smoke_hold: skip __init__ _interpolate_move")
            return
        if self.config.is_dummy:
            return super()._interpolate_move(pose, timeout=timeout)  # type: ignore[misc]

        self._franka_state = self._controller.get_state().wait()[0]
        current = np.asarray(self._franka_state.tcp_pose, dtype=np.float64)
        target = np.asarray(pose, dtype=np.float64)
        dist = float(np.linalg.norm(target[:3] - current[:3]))
        if dist > MAX_INTERP_DISTANCE_M:
            raise RuntimeError(
                f"refusing to interpolate {dist:.3f}m under cartesian impedance "
                f"(cap {MAX_INTERP_DISTANCE_M:.2f}m): from "
                f"{np.round(current[:3], 4).tolist()} to "
                f"{np.round(target[:3], 4).tolist()}. Either the target geometry is "
                "wrong, or the arm is not where this task expects it. Guide it back "
                "near the mark with the enabling device -- RLinf's own franky env "
                "uses a blocking reset_joint for large displacements, never a 10 Hz "
                "impedance sweep."
            )
        # Rotation needs the same refusal, and it is reachable: a calibration
        # written from a quaternion's first three components instead of euler
        # angles puts the target orientation near identity rather than near
        # gripper-down. The safety box and the orientation fence both re-centre on
        # that target, so no guard fires -- and reset then commands a ~180 deg wrist
        # flip with a cube in the gripper.
        if current.size >= 7 and target.size >= 7:
            ang = quat_angle_rad(current[3:7], target[3:7])
            if ang > MAX_INTERP_ANGLE_RAD:
                raise RuntimeError(
                    f"refusing to rotate {ang:.3f}rad under cartesian impedance "
                    f"(cap {MAX_INTERP_ANGLE_RAD:.2f}rad). The target orientation is "
                    f"{ang * 180.0 / np.pi:.0f} deg from the current one -- check that "
                    "target_ee_pose holds euler xyz angles and not a quaternion."
                )
        capped = interp_duration_s(current, target, requested=timeout)
        capped = self._stretch_interp_for_joint_demand(current, target, capped)
        if capped > timeout + 1e-6:
            self._logger.info(
                "interpolate_move: %.4fm in %.2fs would be %.1fcm/s; "
                "stretching to %.2fs to respect the %.1fcm/s cap",
                dist,
                timeout,
                dist / max(timeout, 1e-6) * 100.0,
                capped,
                interp_speed_m_s() * 100.0,
            )
        else:
            self._logger.info(
                "interpolate_move: %.4fm in %.2fs (%.1fcm/s)",
                dist,
                capped,
                dist / max(capped, 1e-6) * 100.0,
            )
        # Mark interpolation scope so _move_action skips the step-path slew clamp:
        # upstream does not refresh self._franka_state inside its waypoint loop, so
        # the clamp's reference pose would be stale for the whole call. See
        # _clamp_step_slew's docstring.
        self._in_interpolate = True
        try:
            result = super()._interpolate_move(pose, timeout=capped)  # type: ignore[misc]
        finally:
            self._in_interpolate = False
        # Poll once more after the last waypoint. The per-waypoint poll in
        # _move_action cannot see a watchdog trip that happens during the final
        # ``time.sleep`` or the trailing ``get_state``, and without this the reason
        # would only surface on the *next* commanded move -- which for the last
        # interpolation of a reset is after the hover gate has already reported a
        # geometry failure instead of the trip that caused it.
        self._raise_if_guard_tripped()
        return result

    def _stretch_interp_for_joint_demand(
        self, current: np.ndarray, target: np.ndarray, capped: float
    ) -> float:
        """Slow an interpolated move until its joint demand fits the budget.

        The interpolation path needs the same joint-space bound as ``step()``
        (measured at the LOG-036 trip pose: 2 cm/s three-axis translation, the
        *interpolation* speed, still demands 1.73x this budget) but it cannot use
        the same mechanism. ``_clamp_step_slew`` bounds a **position**, and
        ``_move_action`` skips it here for a reason its docstring records: upstream
        reads ``_franka_state`` once before the waypoint loop, so a position clamp
        pins the effective target at ``start + budget`` and the arm stalls short.

        A **duration** has neither problem. Joint demand is inversely proportional
        to duration, so stretching by ``1/scale`` is exact, needs no per-waypoint
        state, and lands the demand on the budget -- the arm still reaches the
        target, it just takes longer to get there.

        Evaluated at the *start* Jacobian only. The conditioning changes along the
        path, so this bounds the demand where the move begins, not everywhere; the
        watchdog's ``|dq|`` gate is what covers the rest of the trajectory. Saying
        so explicitly because assuming otherwise is how T19 was missed the first
        time.
        """
        state = getattr(self, "_franka_state", None)
        jacobian = getattr(state, "arm_jacobian", None) if state is not None else None
        if jacobian is None:
            return capped
        d_xyz = np.asarray(target[:3], dtype=np.float64) - np.asarray(
            current[:3], dtype=np.float64
        )
        if current.size >= 7 and target.size >= 7:
            d_rot = (
                R.from_quat(np.asarray(target[3:7], dtype=np.float64))
                * R.from_quat(np.asarray(current[3:7], dtype=np.float64)).inv()
            ).as_rotvec()
        else:
            d_rot = np.zeros(3)

        # Passing the WHOLE displacement with ``capped`` as dt gives the average
        # twist of the move directly, which is what the waypoints will realise --
        # no waypoint-count rounding to get wrong.
        scale, ratio, joint = joint_demand_scale(
            jacobian, d_xyz, d_rot, capped, JOINT_VEL_LIMITS
        )
        if scale >= 1.0:
            return capped
        stretched = capped / scale
        _lo, hi = INTERP_DURATION_S_RANGE
        if stretched > hi:
            raise RuntimeError(
                f"refusing to interpolate from this configuration: the move demands "
                f"{ratio:.1f}x joint j{joint}'s velocity budget "
                f"({joint_vel_demand_fraction():.2f} of "
                f"{JOINT_VEL_LIMITS[joint - 1]:.3f} rad/s), and slowing it enough "
                f"would take {stretched:.1f}s > the {hi:.0f}s cap. The arm is in an "
                "ill-conditioned (near-singular) configuration where cartesian "
                "impedance cannot move it safely -- guide it back near the mark with "
                "the enabling device. See dmo_place_2 T19 / LOG-037."
            )
        self._logger.warning(
            "interpolate_move: joint demand j%d %.2fx budget at %.2fs; "
            "stretching to %.2fs (ill-conditioned pose, LOG-037 T19)",
            joint,
            ratio,
            capped,
            stretched,
        )
        return stretched

    def _raise_if_guard_tripped(self) -> None:
        """Turn a latched controller-side trip into an env-side exception.

        A ``RuntimeError`` raised inside a Worker method never reaches the caller:
        ``WorkerGroupFuncResult`` catches it, prints it, signals the main thread and
        exits, and the handler ``ray.kill``s every actor -- so the smoke script's
        ``try/finally`` teardown cannot run for the case it exists for, and nothing
        env-side ever sees the reason. The controller therefore only *brakes and
        latches* -- no guard path there raises, including on an already-latched trip
        (LOG-023 finding 1: that raise fired from inside ``move_arm`` and killed the
        driver before this poll could run) -- and this converts the latch into a
        normal Python exception here, where ``finally`` blocks still work.
        """
        if self.config.is_dummy:
            return
        reason = self._controller.guard_tripped().wait()[0]
        if reason:
            raise MotionGuardTripped(
                f"motion guard tripped and the arm was braked: {reason}. "
                "Clear the fault in Desk, re-check the start pose, and see "
                "dmo_place_2 §6 before commanding motion again."
            )

    # ------------------------------------------------------------------
    # Trip recovery (T21) and joint-configuration self-check (T22), LOG-040
    # ------------------------------------------------------------------

    def _recover_from_trip(self, exc: MotionGuardTripped, context: str) -> bool:
        """Try to clear a trip so the episode can end normally instead of the run.

        LOG-040's T21. A trip is two separate questions that had been answered as
        one: *is the arm safe* (yes -- the controller braked before latching, and
        LOG-040's ``[watchdog:dq]`` trip left it ``Idle`` with ``|dq|=0.0016``)
        and *should training stop* (previously always yes, because the exception
        propagated out of the Ray actor and took ``ActorGroup`` and
        ``RolloutGroup`` with it). An exploring policy near a workspace edge will
        trip occasionally; ending the whole run each time discards every unsaved
        epoch for an event the safety layer handled correctly.

        The budget is what keeps this from becoming "the guard opts out": each
        recovery is counted for the life of the env, and once
        :func:`guard_recovery_budget` is spent the exception is re-raised and the
        run ends the old way. ``RLINF_CUBE_GUARD_RECOVERY_BUDGET=0`` disables
        recovery entirely.

        Returns:
            Whether the caller may carry on. ``False`` means "re-raise"; it does
            not mean the arm is unsafe (it was braked either way).
        """
        budget = guard_recovery_budget()
        used = getattr(self, "_guard_recoveries_used", 0)
        self._logger.error(
            "\n%s MOTION GUARD TRIP during %s (recovery %d of %d) %s\n%s",
            _BANNER,
            context,
            used + 1,
            budget,
            _BANNER,
            exc,
        )
        if used >= budget:
            self._logger.error(
                "%s guard recovery budget exhausted (%d used); letting the run end. "
                "This is either a workspace problem (the policy keeps reaching a "
                "configuration the arm cannot be driven in -- see dmo_place_2 §7 "
                "T19/T20 and consider shrinking clip_x_range) or a sign the trip "
                "threshold is wrong for this task. Raise "
                "RLINF_CUBE_GUARD_RECOVERY_BUDGET only once you know which. %s",
                _BANNER,
                used,
                _BANNER,
            )
            return False

        report = self._controller.recover_from_guard_trip().wait()[0]
        self._guard_recoveries_used = used + 1
        if not report.get("recovered"):
            self._logger.error(
                "%s guard recovery REFUSED: %s -- the arm stays braked and the run "
                "ends. MANUAL ACTION: clear the fault in Desk, then check the joint "
                "configuration with 'diag_franky_jacobian.py --live' before "
                "restarting (dmo_place_2 §S3.12). %s",
                _BANNER,
                report.get("refusal"),
                _BANNER,
            )
            return False
        self._logger.warning(
            "%s guard trip recovered (%d of %d used): mode=%s has_errors=%s "
            "|dq|=%.4f. The episode ends here; the next reset re-approaches the "
            "hover pose and rebuilds the tracker. %s",
            _BANNER,
            self._guard_recoveries_used,
            budget,
            report.get("robot_mode"),
            report.get("has_errors"),
            float(report.get("joint_speed") or 0.0),
            _BANNER,
        )
        # The trip pose is the one worth measuring: LOG-040's drift was only
        # noticed because a reset refused several minutes later.
        self._warn_if_ill_conditioned("after a guard trip")
        return True

    def _warn_if_ill_conditioned(self, when: str) -> Optional[dict]:
        """Warn when the arm's *joint* configuration is bad, however legal its TCP.

        LOG-040's T22. Every other check in this file is Cartesian -- the safety
        box, the orientation fence, the reach report -- and a 7-axis arm has one
        degree of freedom that none of them can see. Cartesian impedance places
        no bound on that redundancy either, so the elbow drifts with the
        null-space torques; LOG-040 found ``sigma_min=0.0009`` (150x worse than
        the hover pose) with the TCP sitting well inside the box, and nothing had
        anything to object to until a reset could not be performed.

        Costs one SVD of a 6x7 matrix on a Jacobian the controller already
        returned with the state, so it is affordable on the reset path where it
        is actionable. It only ever *warns*: the arm at this moment is not doing
        anything dangerous, and the operator, not the software, is the one who
        can fix it (guide the elbow back -- see LOG-040 §40.2).
        """
        if self.config.is_dummy:
            return None
        state = self._safe_state()
        jacobian = getattr(state, "arm_jacobian", None) if state is not None else None
        cond = jacobian_conditioning(jacobian)
        if cond is None:
            return None
        threshold = sigma_min_warn()
        q = getattr(state, "arm_joint_position", None)
        if cond["sigma_min"] >= threshold:
            self._logger.info(
                "joint configuration OK %s: sigma_min=%.4f (>= %.4f), cond=%.1f",
                when,
                cond["sigma_min"],
                threshold,
                cond["cond"],
            )
            return cond
        self._logger.warning(
            "\n%s ILL-CONDITIONED JOINT CONFIGURATION %s %s\n"
            "sigma_min=%.4f < %.4f (cond=%.1f, manipulability=%.6f)\n"
            "q=%s\n"
            "The TCP pose may be perfectly legal -- this is about the ELBOW, which "
            "cartesian impedance does not control. Steps here get shrunk hard by "
            "the joint-demand clamp (slow but safe); below roughly sigma_min=0.006 "
            "reset itself starts refusing (LOG-040 §40.1).\n"
            "MANUAL ACTION: guide the arm back with the enabling device, aiming the "
            "elbow at the known-good q=[0.23, 0.55, -0.29, -1.30, 0.18, 1.80, 1.05] "
            "(sigma_min~0.11); joints 2 and 4 dominate. Verify with "
            "'diag_franky_jacobian.py --live'. See dmo_place_2 §S3.12 / §7 T22.\n%s",
            when,
            _BANNER,
            _BANNER,
            cond["sigma_min"],
            threshold,
            cond["cond"],
            cond["manipulability"],
            "unknown" if q is None else np.round(np.asarray(q, dtype=np.float64), 4).tolist(),
            _BANNER,
        )
        return cond

    def _safe_state(self):
        """Read controller state for a diagnostic, returning ``None`` on failure.

        The self-check is advisory, so it must not be the reason a reset fails --
        particularly not right after a trip, where the interesting failure is the
        one that already happened.
        """
        try:
            state = self._controller.get_state().wait()[0]
        except Exception as exc:  # noqa: BLE001 - advisory check, never fatal
            self._logger.warning(
                "joint-configuration self-check skipped: could not read state "
                "(%s: %s)",
                type(exc).__name__,
                exc,
            )
            return None
        self._franka_state = state
        return state

    def step(self, action):
        """Run one step, turning a guard trip into a truncated episode (T21).

        Reward is **0.0** and success accounting is skipped, deliberately. The
        commanded step did not execute -- the arm braked partway through it -- so
        scoring the pose it stopped in would credit an action that was never
        carried out. Equally deliberately there is no invented penalty: changing
        the reward function in an error path is how a reward function stops
        meaning what its definition says, and the trip is already visible to
        anyone looking through ``info``.

        ``truncated`` rather than ``terminated`` because the episode was cut
        short by the apparatus, not finished by the task -- which is also what
        makes the SAC bootstrap treat the final value correctly.
        """
        try:
            return super().step(action)  # type: ignore[misc]
        except MotionGuardTripped as exc:
            if not self._recover_from_trip(exc, "step()"):
                raise
            self._num_steps += 1
            self._safe_state()
            observation = self._get_observation()
            return observation, 0.0, False, True, {"motion_guard_trip": str(exc)}

    def reset(self, *args: Any, **kwargs: Any):
        """Reset, self-checking the joint configuration and retrying past a trip.

        The self-check runs *before* the moves, because that is when its advice
        is actionable: an operator who sees the warning here can guide the elbow
        back before ``go_to_rest`` spends 20 seconds creeping toward a pose it
        may not reach, or refuses outright.

        Retries are bounded by :data:`RESET_TRIP_ATTEMPTS` and each one costs a
        slot of the same recovery budget ``step()`` draws on, so a reset that
        keeps tripping cannot quietly consume a long run's worth of recoveries.
        """
        self._warn_if_ill_conditioned("before reset")
        last: Optional[MotionGuardTripped] = None
        for attempt in range(1, RESET_TRIP_ATTEMPTS + 1):
            try:
                return super().reset(*args, **kwargs)  # type: ignore[misc]
            except MotionGuardTripped as exc:
                last = exc
                if not self._recover_from_trip(
                    exc, f"reset() attempt {attempt}/{RESET_TRIP_ATTEMPTS}"
                ):
                    raise
                self._logger.warning(
                    "%s retrying reset after a recovered trip (attempt %d of %d) %s",
                    _BANNER,
                    attempt + 1,
                    RESET_TRIP_ATTEMPTS,
                    _BANNER,
                )
        assert last is not None  # only reachable via the except branch
        raise MotionGuardTripped(
            f"reset tripped the motion guard {RESET_TRIP_ATTEMPTS} times in a row "
            f"and cannot bring the arm back to the rest pose: {last}. This is the "
            "LOG-040 §40.1 case -- guide the arm back with the enabling device and "
            "verify with 'diag_franky_jacobian.py --live' (dmo_place_2 §S3.12)."
        )

    def _move_action(self, position: np.ndarray):
        """Speed-clamp, mark waypoint scope, and surface a latched guard trip.

        The slew clamp is a backstop for the ``step()`` path, which does **not** go
        through ``_interpolate_move`` and therefore never saw the interpolation
        speed cap: ``FrankaEnv.step`` adds ``action * action_scale[0]`` to the
        measured pose and calls this directly. ``CubePlaceConfig`` clamps
        ``action_scale[0]`` for that reason, but clamping here as well covers any
        caller that bypasses ``action_scale``.
        """
        self._in_waypoint_move = True
        try:
            if not self.config.is_dummy and not getattr(self, "_in_interpolate", False):
                position = self._clamp_step_slew(np.asarray(position, dtype=np.float64))
            result = super()._move_action(position)  # type: ignore[misc]
        finally:
            self._in_waypoint_move = False
        self._raise_if_guard_tripped()
        return result

    def _clamp_step_slew(self, position: np.ndarray) -> np.ndarray:
        """Limit one command's translation AND rotation to the per-cycle budget.

        **Only valid on the ``step()`` path**, and ``_move_action`` skips it during
        interpolation for a concrete reason: upstream ``_interpolate_move`` reads
        ``self._franka_state`` **once**, before the waypoint loop
        (``franka_env.py:856``), and never refreshes it inside the loop. Waypoint
        *k* is ``start + k*delta``, so ``|waypoint_k - stale_state|`` grows linearly
        and this clamp would pin the effective target at ``start + budget`` for the
        whole call -- which is exactly what it did: each ``_interpolate_move``
        advanced the arm ~6.5 mm and no further, so ``reset`` stalled 4.2 cm short
        of hover while every individual number in the log looked correct.

        It is also redundant there for translation: ``interp_duration_s`` makes the
        waypoint spacing ``interp_speed / step_frequency`` = 2 mm, well under the
        8.7 mm default budget. On the ``step()`` path ``_franka_state`` *is* fresh
        (``FrankaEnv.step`` recomputes its target from it every cycle), so the
        reference pose is right and the clamp is a pure backstop for a caller that
        bypasses ``action_scale``.

        The rotation half is not redundant anywhere: LOG-034's WATCHDOG trip was a
        ``next_position[3:]`` commanded via ``action_scale[1]`` with no clamp on
        this path at all (only ``CubePlaceConfig.__post_init__`` now clamps the
        *scale*, which a caller bypassing ``action_scale`` -- or an as-yet-untuned
        cap -- would not catch). Clamping the per-cycle rotation here mirrors the
        translation backstop and is a second, independent line of defence.
        """
        state = getattr(self, "_franka_state", None)
        if state is None:
            return position
        here = np.asarray(state.tcp_pose[:3], dtype=np.float64)
        delta = np.asarray(position[:3], dtype=np.float64) - here
        dist = float(np.linalg.norm(delta))
        budget = max_action_scale_xyz(self.config.step_frequency) * np.sqrt(3.0)
        out = np.array(position, dtype=np.float64, copy=True)
        notes = []
        if dist > budget and dist > 1e-12:
            out[:3] = here + delta * (budget / dist)
            notes.append(f"xyz {dist:.4f}m -> {budget:.4f}m ({step_speed_m_s() * 100:.1f}cm/s cap)")

        if out.size >= 7 and len(state.tcp_pose) >= 7:
            here_quat = np.asarray(state.tcp_pose[3:7], dtype=np.float64)
            target_quat = np.asarray(position[3:7], dtype=np.float64)
            ang = quat_angle_rad(here_quat, target_quat)
            rot_budget = max_action_scale_rot(self.config.step_frequency) * np.sqrt(3.0)
            if ang > rot_budget and ang > 1e-9:
                slerp = Slerp([0.0, 1.0], R.from_quat(np.stack([here_quat, target_quat])))
                out[3:7] = slerp(rot_budget / ang).as_quat()
                notes.append(
                    f"rot {ang:.4f}rad -> {rot_budget:.4f}rad "
                    f"({step_rot_speed_rad_s():.2f}rad/s cap)"
                )

        out, dq_note = self._clamp_joint_demand(here, out, state)
        if dq_note:
            notes.append(dq_note)

        if not notes:
            return position
        self._logger.warning(
            "step slew clamped: %s at %.0fHz",
            "; ".join(notes),
            float(self.config.step_frequency),
        )
        return out

    def _clamp_joint_demand(
        self, here: np.ndarray, out: np.ndarray, state: Any
    ) -> tuple[np.ndarray, str]:
        """Shrink the step until the joint velocity it *demands* fits the budget.

        The third and last clamp on this path, and the only one that is not
        Cartesian. LOG-037 measured why the other two are insufficient: the
        Jacobian's conditioning varied 14.6x between the hover pose and the pose
        training tripped at, so a command that is fully compliant with both
        Cartesian caps demanded 0.81 rad/s at one and 11.77 rad/s at the other.
        Neither ``step_speed_m_s`` nor ``step_rot_speed_rad_s`` can see that
        difference -- they do not know where the arm is.

        Runs last, on the already-Cartesian-clamped ``out``, so the two bounds
        compose: whichever binds harder wins, and the joint bound is evaluated on
        a step that is already inside the Cartesian one.

        Uses ``state.arm_jacobian``, which the controller returns with every
        ``get_state()`` and which ``FrankaEnv.step`` refreshes each cycle, so this
        costs one 6x7 pseudo-inverse and no extra round-trip.
        """
        jacobian = getattr(state, "arm_jacobian", None)
        if jacobian is None:
            return out, ""
        hz = float(self.config.step_frequency) or 10.0
        dt = 1.0 / max(hz, 1e-6)

        d_xyz = np.asarray(out[:3], dtype=np.float64) - here
        if out.size >= 7 and len(state.tcp_pose) >= 7:
            here_quat = np.asarray(state.tcp_pose[3:7], dtype=np.float64)
            target_quat = np.asarray(out[3:7], dtype=np.float64)
            d_rot = (
                R.from_quat(target_quat) * R.from_quat(here_quat).inv()
            ).as_rotvec()
        else:
            here_quat = None
            target_quat = None
            d_rot = np.zeros(3)

        scale, worst_ratio, worst_joint = joint_demand_scale(
            jacobian, d_xyz, d_rot, dt, JOINT_VEL_LIMITS
        )
        if scale >= 1.0:
            return out, ""

        # dq_req is linear in the step, so scaling the step scales the demand by
        # exactly the same factor -- no iteration needed.
        clamped = np.array(out, dtype=np.float64, copy=True)
        clamped[:3] = here + d_xyz * scale
        if here_quat is not None and target_quat is not None:
            ang = quat_angle_rad(here_quat, target_quat)
            if ang > 1e-9:
                slerp = Slerp([0.0, 1.0], R.from_quat(np.stack([here_quat, target_quat])))
                clamped[3:7] = slerp(scale).as_quat()
        note = (
            f"joint demand j{worst_joint} {worst_ratio:.2f}x budget "
            f"({joint_vel_demand_fraction():.2f} of "
            f"{JOINT_VEL_LIMITS[worst_joint - 1]:.3f}rad/s) -> step x{scale:.3f} "
            "(ill-conditioned pose, see LOG-037 T19)"
        )
        return clamped, note

    def _clear_error(self):
        """Honour ``clear_error_per_waypoint`` while keeping upstream as default.

        ``_move_action`` calls this on every waypoint / every ``step``. Setting
        ``clear_error_per_waypoint: False`` suppresses only those calls; the
        explicit ``_clear_error()`` at the end of ``FrankaEnv.reset`` still runs,
        so latched faults are still cleared once per episode.

        Default ``True`` = unchanged upstream behaviour. See the module docstring
        for why this is a knob and not a fix.
        """
        if getattr(self, "_in_waypoint_move", False) and not getattr(
            self.config, "clear_error_per_waypoint", True
        ):
            return None
        return super()._clear_error()  # type: ignore[misc]

    # ------------------------------------------------------------------
    # Hardware
    # ------------------------------------------------------------------

    def _setup_hardware(self):
        assert self.env_idx >= 0, "env_idx must be set for FrankaEnv."
        assert isinstance(self.hardware_info, FrankaHWInfo), (
            f"hardware_info must be FrankaHWInfo, got {type(self.hardware_info)}."
        )
        if self.config.robot_ip is None:
            self.config.robot_ip = self.hardware_info.config.robot_ip
        if self.config.camera_serials is None:
            self.config.camera_serials = self.hardware_info.config.camera_serials
        if self.config.camera_type is None:
            self.config.camera_type = getattr(
                self.hardware_info.config, "camera_type", "realsense"
            )
        if self.config.gripper_type is None:
            self.config.gripper_type = getattr(
                self.hardware_info.config, "gripper_type", "franka"
            )
        if self.config.gripper_connection is None:
            self.config.gripper_connection = getattr(
                self.hardware_info.config, "gripper_connection", None
            )
        self.config.end_effector_type = normalize_end_effector_type(
            self.config.end_effector_type,
            self.config.gripper_type,
        ).value

        controller_node_rank = getattr(
            self.hardware_info.config, "controller_node_rank", None
        )
        if controller_node_rank is None:
            controller_node_rank = self.node_rank

        # LOG-034 (T17): only trust this env's own node_group_label when the
        # controller is co-located on the SAME cluster node -- if an operator
        # explicitly set controller_node_rank to a different node, this env's
        # label may not even be a valid group for that node, and launch_controller
        # falls back to the old (env-var-blind) behaviour with a logged warning
        # rather than guessing.
        controller_node_group_label = (
            self._worker_node_group_label
            if controller_node_rank == self.node_rank
            else None
        )
        self._controller = FrankyControllerExtended.launch_controller(
            robot_ip=self.config.robot_ip,
            env_idx=self.env_idx,
            node_rank=controller_node_rank,
            worker_rank=self.env_worker_rank,
            gripper_type=self.config.gripper_type or "franka",
            gripper_connection=self.config.gripper_connection,
            node_group_label=controller_node_group_label,
        )
        # Arm the fence before FrankaEnv.__init__ runs its own
        # _interpolate_move(self._reset_pose) -- otherwise the first motion of a
        # training run is completely unbounded.
        self.arm_motion_guard()

    def _open_cameras(self):
        if _skip_camera():
            self._cameras = []
            return
        super()._open_cameras()  # type: ignore[misc]

    def _get_camera_frames(self) -> dict[str, np.ndarray]:
        if _skip_camera():
            frames = {}
            for name, space in self.observation_space["frames"].spaces.items():
                h, w, c = space.shape
                frames[name] = np.zeros((h, w, c), dtype=np.uint8)
            return frames
        return super()._get_camera_frames()  # type: ignore[misc]

    def close(self):
        """Stop the impedance tracker deliberately before shutting down.

        ``FrankaEnv.close`` stops the video player and the cameras and never
        touches the controller, so even a *successful* run used to end with the
        cartesian impedance tracker still commanding torque at its last target
        until the process exited and libfranka's comms watchdog forced a stop.
        ``Cluster._shutdown_ray_at_exit`` calls ``os._exit(0)`` on the happy path,
        so atexit hooks do not save us either.

        ``cleanup()`` stops both trackers and joins the motion; it does **not**
        open the gripper (``FrankaLibfrankaGripper.cleanup`` is deliberately a
        no-op), so a held cube stays held.
        """
        try:
            controller = getattr(self, "_controller", None)
            if controller is not None and not self.config.is_dummy:
                controller.freeze_at_current().wait()
                controller.cleanup().wait()
                self._logger.info("controller tracker stopped on close()")
        except Exception as exc:  # noqa: BLE001 - teardown must not mask a failure
            self._logger.error(
                "controller teardown on close() failed: %s: %s",
                type(exc).__name__,
                exc,
            )
        finally:
            super().close()  # type: ignore[misc]


@dataclass
class FrankySingleFrankaEnvConfig(FrankaRobotConfig):
    """Config for franky single-arm env; safe_smoke_hold skips only __init__ interpolate."""

    safe_smoke_hold: bool = False
    #: Call ``clear_errors()`` on every interpolation waypoint (upstream default).
    #: See the module docstring: pending measurement, not a settled question.
    clear_error_per_waypoint: bool = True


class FrankySingleFrankaEnv(FrankySingleFrankaEnvMixin, FrankaEnv):
    CONFIG_CLS = FrankySingleFrankaEnvConfig
