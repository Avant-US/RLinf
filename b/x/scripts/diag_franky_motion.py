#!/usr/bin/env python3
"""Which franky motion primitive actually moves this arm, and how safely?

No Ray, no ``FrankaEnv``, no wrappers -- this talks to ``franky`` directly so the
control link can be judged on its own.

Two incidents shaped this script; read both before changing it.

**LOG-017 (silent stall).** ``CartesianImpedanceTracker.__init__`` starts an
*asynchronous* ``robot.move(motion, asynchronous=True)``. If that control thread
dies, ``set_target`` silently writes to a dead reference handle and the arm never
moves (``dz=0.0000``). The stored exception only surfaces on ``stop()`` /
``join_motion()``. Root cause that time: the hardware user-stop was pressed, so
``robot_mode`` was ``UserStopped`` -- hence ``--probe`` and the mode gate.

**LOG-019 (runaway).** A ``reset`` commanded a 10 cm lift and the arm reached
26 cm *above* the highest commanded target. The previous version of this script
could not have caught it, for two reasons that are both fixed here:

* it called ``tracker.stop()`` immediately after the last waypoint, so
  post-ramp **overshoot** -- the entire quantity of interest -- was invisible;
* it hardcoded ``K_t = 2000`` with ``clip = 0.05``, i.e. it reproduced the very
  100 N force ceiling that caused the incident, and called that "the same
  parameters as the env".

So now: clips are derived from stiffness via
:mod:`franky_ext.motion_limits` (constant *force* ceiling, not constant clip),
every motion test samples continuously through a settle window, and the script
carries its own abort guards -- overshoot, absolute z ceiling, tracking lag and
tracker death all brake the arm rather than waiting for a human.

Braking, when a guard fires, splits by violation kind -- mirroring
``FrankyControllerExtended._brake`` (round-2 audit finding 1, which caught this
script still using the single order LOG-021 retracted from the controller):

* ``fence`` (z-ceiling / z-floor / overshoot) -- the arm has gone *past* the
  target, so the impedance spring is already pulling it back. ``stop()`` runs
  **first**; ``set_target(measured)`` would zero that restoring force and is not
  called for this kind.
* ``lag`` -- the *target* is the problem (running away from the arm), so
  ``set_target(measured)`` removes it first, then the tracker is stopped.

Either way the dwell is a bounded poll of ``|dq|``, not a fixed sleep -- a fixed
0.25 s sleep at LOG-019's ~26 cm/s excursion rate is 6.5 cm of extra travel before
the step that actually decelerates.

Tests
-----
``--probe``                 read-only: mode / errors / TCP / joints / success rate
``--test-hold``             tracker at the *current* pose; liveness + gravity sag
``--test-impedance``        ramp z by ``--dz``, then hold and watch for overshoot
``--test-waypoints``        replay the ``_interpolate_move`` regime: absolute
                            waypoints at ``--waypoint-hz``, i.e. what ``reset``
                            actually commands. This is the LOG-019 reproduction.
``--test-recover-loop``     same as ``--test-waypoints`` but calls
                            ``recover_from_errors()`` between waypoints, the way
                            ``FrankaEnv._move_action`` does (settles LOG-019 R5)
``--test-cartesian-motion`` blocking ``robot.move(CartesianMotion)``
``--test-rotation``         ramp orientation by ``--drx/--dry/--drz`` (euler xyz,
                            rad) about the CURRENT pose, xyz held fixed; then hold
                            and watch. The rotation analogue of ``--test-impedance``.
``--test-rotation-waypoints`` same rotation, but as a 10 Hz zero-order-hold
                            staircase (``--rot-waypoint-hz``) -- this is what
                            ``FrankaEnv.step`` actually looks like one cycle at a
                            time (closed-loop re-commanding from the fresh measured
                            pose every cycle looks, worst-case, like a sustained
                            per-cycle increment). **This is the 2.4b-rot ladder**
                            LOG-034 asked for: it is the only way to measure whether
                            ``motion_limits.STEP_ROT_SPEED_RAD_S_DEFAULT`` is safe
                            near a reach boundary, rather than assumed from a ratio.

Run inside the franky container after
``source b/x/configs/setup_before_ray_5090.sh``, with nothing else holding FCI.
Motion tests need ``--yes-move``; stand by the e-stop with the user-stop **up**.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from franky_ext.motion_limits import (  # noqa: E402
    cartesian_collision_thresholds,
    clip_shortfall,
    describe_authority,
    error_clips_for_stiffness,
    guard_max_lag_m,
    max_action_scale_rot,
    quat_angle_rad,
    step_rot_speed_rad_s,
)
from franky_ext.tcp_probe import describe_robot_mode  # noqa: E402

# Read-only import of upstream constants (JOINT_LIMITS_*, _TORQUE_THRESHOLD): this
# is an import, not an edit -- rlinf/ is not modified. Needed so 2.4b runs under
# the SAME hardware reflex and soft joint-limit repulsion as the env path (round-2
# audit finding 3): without them, the one rung deliberately designed to provoke a
# runaway had no 1 kHz hardware bound and no joint protection, only the 20 ms
# Python guards below -- while gate 2.4-0 requires the tightened thresholds in
# every log involving motion.
from rlinf.envs.realworld.franka import franky_controller as _fc  # noqa: E402

DEFAULT_IP = "172.16.0.2"

# Matches what the env actually builds after ``reconfigure_compliance_params``
# pushes PegInsertion's compliance_param in: K_t=2000, K_r=150, tc from franky's
# own default (the ROS ``translational_damping`` -> ``gains_time_constant`` map
# was removed, see LOG-020 F2).
K_TRANS = 2000.0
K_ROT = 150.0
MAX_DELTA_TAU = 0.3
GAINS_TC = 0.1
NULLSPACE_STIFFNESS = 5.0
DYNAMICS_FACTOR = 0.2

# Soft joint-limit repulsion parameters, identical to
# FrankyControllerExtended._ensure_cart_tracking_motion -- kept in sync by reading
# the same fc.JOINT_LIMITS_LOWER/UPPER rather than duplicating the numbers.
_JOINT_LIMIT_ACTIVATION_RAD = 0.10
_JOINT_LIMIT_STIFFNESS = 4.0
_JOINT_LIMIT_DAMPING = 1.0
_JOINT_LIMIT_MAX_TORQUE = 5.0

#: State sampling period during motion tests. The LOG-019 event lasted ~3 s, so
#: 20 ms gives ~150 samples -- enough to see a peak, not just an endpoint.
SAMPLE_DT = 0.02


def _fmt(v) -> str:
    return "[" + ", ".join(f"{float(x):.4f}" for x in np.asarray(v).reshape(-1)) + "]"


def _get(obj, name, default=None):
    """Read an optional franky state field without assuming the version has it."""
    try:
        value = getattr(obj, name)
    except Exception:
        return default
    return default if value is None else value


def _tcp(robot) -> np.ndarray:
    return np.asarray(robot.current_pose.end_effector_pose.translation, dtype=np.float64)


def _quat(robot) -> np.ndarray:
    return np.asarray(robot.current_pose.end_effector_pose.quaternion, dtype=np.float64)


def _ext_force_norm(robot) -> Optional[float]:
    state = _get(robot, "state")
    if state is None:
        return None
    wrench = _get(state, "O_F_ext_hat_K")
    if wrench is None:
        return None
    try:
        return float(np.linalg.norm(np.asarray(wrench, dtype=np.float64).reshape(-1)[:3]))
    except Exception:
        return None


def _joint_vel_norm(robot) -> Optional[float]:
    dq = _get(robot, "current_joint_velocities")
    if dq is None:
        state = _get(robot, "state")
        dq = _get(state, "dq") if state is not None else None
    if dq is None:
        return None
    try:
        return float(np.linalg.norm(np.asarray(dq, dtype=np.float64).reshape(-1)))
    except Exception:
        return None


def _success_rate(robot) -> Optional[float]:
    state = _get(robot, "state")
    if state is None:
        return None
    rate = _get(state, "control_command_success_rate")
    try:
        return None if rate is None else float(rate)
    except Exception:
        return None


def _report(robot, label: str) -> None:
    print(
        f"{label}: has_errors={robot.has_errors} "
        f"is_in_control={robot.is_in_control} "
        f"signal={robot.current_control_signal_type} "
        f"cmd_success_rate={_success_rate(robot)}"
    )


def probe(robot) -> str:
    """Print state and return ``robot_mode``. Motion only runs in ``Idle``."""
    _report(robot, "probe")
    mode = str(robot.state.robot_mode)
    print(f"probe robot_mode={mode}")
    print(f"probe tcp xyz={_fmt(_tcp(robot))} quat={_fmt(_quat(robot))}")
    print(f"probe q={_fmt(robot.current_joint_positions)}")
    fext = _ext_force_norm(robot)
    dqn = _joint_vel_norm(robot)
    print(
        f"probe |F_ext|={'n/a' if fext is None else f'{fext:.2f}N'} "
        f"|dq|={'n/a' if dqn is None else f'{dqn:.4f}rad/s'}"
    )
    if mode != "RobotMode.Idle":
        print(f"probe WARNING: {describe_robot_mode(mode)}")
    return mode


# --------------------------------------------------------------------------
# Guarded tracker runner
# --------------------------------------------------------------------------


@dataclass
class Guards:
    """Abort thresholds for a single motion test.

    ``z_ceiling`` is absolute and is the last line of defence: it is what would
    have stopped LOG-019 at +5 cm instead of +36 cm.
    """

    z_ceiling: float
    z_floor: float
    max_overshoot_m: float
    max_lag_m: float


@dataclass
class Trace:
    """What a motion test measured, so a human can judge it without the arm."""

    start_xyz: np.ndarray
    samples: int = 0
    peak_dz: float = 0.0
    min_dz: float = 0.0
    peak_overshoot: float = 0.0
    peak_lag: float = 0.0
    peak_fext: float = 0.0
    peak_dq: float = 0.0
    final_dz: float = 0.0
    settle_drift: float = 0.0
    alive: bool = True
    abort_reason: str = ""
    stop_cause: str = ""
    notes: list[str] = field(default_factory=list)

    def summary(self) -> str:
        return (
            f"samples={self.samples} final_dz={self.final_dz:+.4f} "
            f"peak_dz={self.peak_dz:+.4f} min_dz={self.min_dz:+.4f} "
            f"peak_overshoot={self.peak_overshoot:+.4f} peak_lag={self.peak_lag:.4f} "
            f"settle_drift={self.settle_drift:+.4f} "
            f"peak|F_ext|={self.peak_fext:.1f}N peak|dq|={self.peak_dq:.3f}rad/s "
            f"alive={self.alive}"
        )


def _make_tracker(robot, franky, *, k_trans: float, k_rot: float):
    """Build the tracker with clips *derived from* stiffness (constant force).

    Passes the same soft joint-limit repulsion kwargs as
    ``FrankyControllerExtended._ensure_cart_tracking_motion``, with the same
    ``TypeError`` fallback for an older franky that does not accept them --
    otherwise the rung built to provoke a runaway would run with LESS joint
    protection than the env path it is meant to validate (round-2 audit
    finding 3).
    """
    trans_clip, rot_clip = error_clips_for_stiffness(k_trans, k_rot)
    nullspace_target = np.asarray(robot.current_joint_positions, dtype=np.float64).copy()
    kwargs = dict(
        translational_stiffness=k_trans,
        rotational_stiffness=k_rot,
        nullspace_target=nullspace_target,
        nullspace_stiffness=NULLSPACE_STIFFNESS,
        translational_error_clip=np.full(3, trans_clip),
        rotational_error_clip=np.full(3, rot_clip),
        max_delta_tau=MAX_DELTA_TAU,
        gains_time_constant=GAINS_TC,
    )
    limit_kwargs = dict(
        lower_joint_limits=_fc.JOINT_LIMITS_LOWER,
        upper_joint_limits=_fc.JOINT_LIMITS_UPPER,
        joint_limit_activation_distance=_JOINT_LIMIT_ACTIVATION_RAD,
        joint_limit_stiffness=_JOINT_LIMIT_STIFFNESS,
        joint_limit_damping=_JOINT_LIMIT_DAMPING,
        joint_limit_max_torque=_JOINT_LIMIT_MAX_TORQUE,
    )
    joint_repulsion = True
    try:
        tracker = franky.CartesianImpedanceTracker(robot, **kwargs, **limit_kwargs)
    except TypeError as exc:
        joint_repulsion = False
        print(f"  !! franky rejected joint-limit kwargs ({exc}); no joint repulsion")
        tracker = franky.CartesianImpedanceTracker(robot, **kwargs)
    print(
        f"tracker created: is_running={tracker.is_running} joint_repulsion={joint_repulsion}  "
        f"{describe_authority(k_trans, k_rot)}"
    )
    return tracker


def _joint_speed(robot) -> float:
    dqn = _joint_vel_norm(robot)
    return dqn if dqn is not None else float("nan")


def _brake(tracker, robot, franky, kind: str, reason: str, trace: Trace) -> None:
    """Decelerate the arm, ordered by violation kind. Mirrors
    ``FrankyControllerExtended._brake`` -- round-2 audit finding 1 caught this
    function still using the single order LOG-021 retracted from the controller,
    which for an overshoot removes the only force decelerating the arm and then
    waits a fixed 0.25 s (6.5 cm of extra travel at LOG-019's excursion rate)
    before the step that actually stops it.

    ``fence`` (z-ceiling / z-floor / overshoot): the arm has gone PAST the target,
    so the impedance spring is already restoring it -- ``stop()`` first, and do
    NOT ``set_target`` (that would zero the restoring force).

    ``lag``: the TARGET is the problem -- ``set_target(measured)`` first to remove
    it, then stop.

    Both branches replace the fixed sleep with a bounded poll of ``|dq|``.
    """
    trace.alive = False
    trace.abort_reason = reason
    print(f"  !! ABORT [{kind}]: {reason}")
    v0 = _joint_speed(robot)

    if kind == "lag":
        try:
            tracker.set_target(franky.Affine(_tcp(robot), _quat(robot)))
            print("  brake: target <- measured pose (impedance decelerates)")
        except Exception as exc:
            print(f"  brake set_target failed: {type(exc).__name__}: {exc}")
        deadline = time.perf_counter() + 0.25
        while time.perf_counter() < deadline:
            if _joint_speed(robot) < 0.02:
                break
            time.sleep(0.005)
        try:
            trace.stop_cause = "clean"
            tracker.stop()
        except Exception as exc:
            trace.stop_cause = f"{type(exc).__name__}: {exc}"
    else:
        try:
            trace.stop_cause = "clean"
            tracker.stop()
            print("  brake: stop() first (spring is still restoring; not zeroing it)")
        except Exception as exc:
            trace.stop_cause = f"{type(exc).__name__}: {exc}"
        deadline = time.perf_counter() + 0.25
        while time.perf_counter() < deadline:
            if _joint_speed(robot) < 0.02:
                break
            time.sleep(0.005)

    print(f"  brake: |dq| {v0:.4f} -> {_joint_speed(robot):.4f} rad/s, stop={trace.stop_cause}")


def run_tracker_motion(
    robot,
    franky,
    *,
    label: str,
    target_at: Callable[[float], np.ndarray],
    duration_s: float,
    settle_s: float,
    guards: Guards,
    k_trans: float = K_TRANS,
    k_rot: float = K_ROT,
    target_hz: Optional[float] = None,
    between_targets: Optional[Callable[[], None]] = None,
    verbose_every: float = 0.25,
) -> Trace:
    """Drive the impedance tracker along ``target_at`` and measure everything.

    Args:
        target_at: ``t -> xyz`` commanded target, ``t`` in seconds from start.
            Clamped to the final value once ``t > duration_s`` so the settle
            window holds the endpoint (this is where overshoot shows up).
        duration_s: length of the commanded motion.
        settle_s: extra time to keep the tracker alive, target held at the
            endpoint, purely to observe. **Do not remove this** -- the whole
            LOG-019 quantity lives in this window.
        guards: abort thresholds.
        target_hz: if set, ``set_target`` is called at this rate instead of every
            sample, reproducing the 10 Hz zero-order-hold staircase that
            ``FrankaEnv._interpolate_move`` actually emits.
        between_targets: optional hook run right before each ``set_target``,
            used by ``--test-recover-loop`` to inject
            ``robot.recover_from_errors()`` the way ``_move_action`` does.
    """
    print(f"\n=== {label} ===")
    robot.recover_from_errors()
    start_xyz = _tcp(robot)
    quat = _quat(robot)
    trace = Trace(start_xyz=start_xyz)
    print(
        f"start xyz={_fmt(start_xyz)}  guards: z in "
        f"[{guards.z_floor:.4f}, {guards.z_ceiling:.4f}] "
        f"overshoot<={guards.max_overshoot_m:.3f} lag<={guards.max_lag_m:.3f}"
    )

    tracker = _make_tracker(robot, franky, k_trans=k_trans, k_rot=k_rot)
    final_target = np.asarray(target_at(duration_s), dtype=np.float64)

    t0 = time.perf_counter()
    total = duration_s + max(0.0, settle_s)
    next_target_t = 0.0
    next_print_t = 0.0
    target_period = (1.0 / target_hz) if target_hz else 0.0
    commanded = start_xyz.copy()

    while True:
        t = time.perf_counter() - t0
        if t > total:
            break

        if t >= next_target_t:
            commanded = np.asarray(
                target_at(min(t, duration_s)), dtype=np.float64
            )
            if between_targets is not None:
                try:
                    between_targets()
                except Exception as exc:
                    trace.notes.append(f"between_targets raised: {exc}")
            try:
                tracker.set_target(franky.Affine(commanded, quat))
            except Exception as exc:
                trace.alive = False
                trace.abort_reason = f"set_target raised: {type(exc).__name__}: {exc}"
                print(f"  !! {trace.abort_reason}")
                break
            next_target_t = t + target_period if target_period else t

        time.sleep(SAMPLE_DT)
        live = _tcp(robot)
        trace.samples += 1
        dz = float(live[2] - start_xyz[2])
        trace.peak_dz = max(trace.peak_dz, dz)
        trace.min_dz = min(trace.min_dz, dz)
        lag = float(np.linalg.norm(live - commanded))
        trace.peak_lag = max(trace.peak_lag, lag)
        # Overshoot: distance past the *final* target along the motion direction.
        motion = final_target - start_xyz
        norm = float(np.linalg.norm(motion))
        if norm > 1e-9:
            along = float(np.dot(live - final_target, motion / norm))
            trace.peak_overshoot = max(trace.peak_overshoot, along)
        else:
            trace.peak_overshoot = max(
                trace.peak_overshoot, float(np.linalg.norm(live - final_target))
            )
        fext = _ext_force_norm(robot)
        if fext is not None:
            trace.peak_fext = max(trace.peak_fext, fext)
        dqn = _joint_vel_norm(robot)
        if dqn is not None:
            trace.peak_dq = max(trace.peak_dq, dqn)

        if t >= next_print_t:
            next_print_t = t + verbose_every
            phase = "move" if t <= duration_s else "settle"
            print(
                f"  t={t:5.2f}s {phase:6s} cmd_z={commanded[2]:.4f} "
                f"live={_fmt(live)} dz={dz:+.4f} lag={lag:.4f} "
                f"over={trace.peak_overshoot:+.4f} "
                f"F={'n/a' if fext is None else f'{fext:5.1f}N'} "
                f"run={tracker.is_running}"
            )

        if not tracker.is_running:
            trace.alive = False
            trace.abort_reason = "tracker died (async control thread gone)"
            print(f"  !! {trace.abort_reason} at t={t:.2f}s live={_fmt(live)}")
            break
        if live[2] > guards.z_ceiling:
            _brake(tracker, robot, franky, "fence", f"z={live[2]:.4f} > ceiling {guards.z_ceiling:.4f}", trace)
            break
        if live[2] < guards.z_floor:
            _brake(tracker, robot, franky, "fence", f"z={live[2]:.4f} < floor {guards.z_floor:.4f}", trace)
            break
        if trace.peak_overshoot > guards.max_overshoot_m:
            _brake(
                tracker, robot, franky, "fence",
                f"overshoot {trace.peak_overshoot:.4f} > {guards.max_overshoot_m:.4f}",
                trace,
            )
            break
        if lag > guards.max_lag_m:
            _brake(
                tracker, robot, franky, "lag",
                f"lag {lag:.4f} > {guards.max_lag_m:.4f} (not tracking)",
                trace,
            )
            break

    end_of_move = _tcp(robot)
    # _brake() above may already have stopped the tracker (fence kind stops
    # first). Only stop here if that did not already happen, and only report a
    # cause if one is not already recorded, so an abort's real stop_cause is not
    # overwritten by a second, redundant stop() call.
    if not trace.stop_cause:
        try:
            tracker.stop()
            trace.stop_cause = "clean"
            print("tracker.stop() clean")
        except Exception as exc:
            # stop() -> join_motion() re-raises whatever killed the control thread.
            trace.stop_cause = f"{type(exc).__name__}: {exc}"
            trace.alive = False
            print(f"tracker.stop() surfaced: {trace.stop_cause}")

    time.sleep(0.2)
    final = _tcp(robot)
    trace.final_dz = float(final[2] - start_xyz[2])
    trace.settle_drift = float(final[2] - end_of_move[2])
    _report(robot, "after")
    print(f"{label}: {trace.summary()}")
    if trace.notes:
        for note in trace.notes:
            print(f"  note: {note}")
    return trace


# --------------------------------------------------------------------------
# Rotation ladder (LOG-034 / T16): the 2.4b-rot counterpart of run_tracker_motion.
#
# Kept as a SEPARATE loop rather than generalising run_tracker_motion's target to
# a 7-vector, on purpose: the translation ladder is already validated (2.4b) and
# every existing test (--test-hold/--test-impedance/--test-waypoints) depends on
# its exact fence/overshoot/lag semantics. Reusing it for rotation would mean
# either changing what "overshoot" and "lag" mean (they are xyz-only today) under
# a script whose whole job is to be trusted, or bolting rotation fields onto the
# same Guards/Trace with half of them meaningless for either mode. A parallel
# loop is more code but nothing here can silently change the meaning of a number
# 2.4b already reported.
# --------------------------------------------------------------------------


@dataclass
class RotGuards:
    """Abort thresholds for a rotation-only motion test.

    ``max_dq`` is deliberately tighter than anything the translation ladder uses:
    this exercises an authority (``action_scale[1]``) that has NEVER been
    measured on this arm, on a task whose safety box corners were logged as
    NEAR-SINGULAR (92-98% of max reach) in LOG-034 -- exactly where a cartesian
    rotation command demands a disproportionate joint velocity. The incident's
    ``|dq|`` reached 2.69 rad/s before the brake; this default aborts more than
    an order of magnitude earlier, on the first rung, so a genuinely dangerous
    rotation authority is caught while displacement is still tiny.
    """

    max_xyz_drift_m: float
    max_ang_overshoot_rad: float
    max_ang_lag_rad: float
    max_dq_rad_s: float


@dataclass
class RotTrace:
    """What a rotation motion test measured."""

    start_xyz: np.ndarray
    start_quat: np.ndarray
    samples: int = 0
    peak_ang: float = 0.0
    peak_ang_overshoot: float = 0.0
    peak_ang_lag: float = 0.0
    peak_xyz_drift: float = 0.0
    peak_fext: float = 0.0
    peak_dq: float = 0.0
    final_ang: float = 0.0
    settle_ang_drift: float = 0.0
    alive: bool = True
    abort_reason: str = ""
    stop_cause: str = ""
    notes: list[str] = field(default_factory=list)

    def summary(self) -> str:
        return (
            f"samples={self.samples} final_ang={self.final_ang:+.4f}rad "
            f"peak_ang={self.peak_ang:+.4f}rad "
            f"peak_ang_overshoot={self.peak_ang_overshoot:+.4f}rad "
            f"peak_ang_lag={self.peak_ang_lag:.4f}rad "
            f"peak_xyz_drift={self.peak_xyz_drift:.4f}m "
            f"settle_ang_drift={self.settle_ang_drift:+.4f}rad "
            f"peak|F_ext|={self.peak_fext:.1f}N peak|dq|={self.peak_dq:.3f}rad/s "
            f"alive={self.alive}"
        )


def run_tracker_rotation(
    robot,
    franky,
    *,
    label: str,
    quat_at: Callable[[float], np.ndarray],
    duration_s: float,
    settle_s: float,
    guards: RotGuards,
    k_trans: float = K_TRANS,
    k_rot: float = K_ROT,
    target_hz: Optional[float] = None,
    verbose_every: float = 0.25,
) -> RotTrace:
    """Drive the impedance tracker's ORIENTATION along ``quat_at``, xyz held fixed.

    Mirrors :func:`run_tracker_motion`'s structure and brake semantics (same
    ``_brake`` helper, same fence-vs-lag ordering, same bounded poll instead of a
    fixed sleep) but measures angle instead of z-displacement, and adds a raw
    ``|dq|`` circuit breaker (:attr:`RotGuards.max_dq_rad_s`) as an EXTRA line of
    defence specific to this never-before-exercised authority.

    Args:
        quat_at: ``t -> xyzw quaternion`` commanded target, ``t`` in seconds from
            start. Clamped to the final value once ``t > duration_s``.
        target_hz: if set, reproduces the 10 Hz zero-order-hold ``step()`` uses,
            instead of a per-sample smooth ramp.
    """
    print(f"\n=== {label} ===")
    robot.recover_from_errors()
    start_xyz = _tcp(robot)
    start_quat = _quat(robot)
    trace = RotTrace(start_xyz=start_xyz, start_quat=start_quat)
    print(
        f"start xyz={_fmt(start_xyz)} quat={_fmt(start_quat)}  guards: "
        f"xyz_drift<={guards.max_xyz_drift_m:.3f} "
        f"ang_overshoot<={guards.max_ang_overshoot_rad:.3f} "
        f"ang_lag<={guards.max_ang_lag_rad:.3f} |dq|<={guards.max_dq_rad_s:.3f}"
    )

    tracker = _make_tracker(robot, franky, k_trans=k_trans, k_rot=k_rot)
    final_quat = np.asarray(quat_at(duration_s), dtype=np.float64)
    # Commanded amplitude (start -> final), used to turn the *unsigned* angle
    # travelled so far into a *signed* overshoot -- mirrors the translation
    # side's `along = dot(live - final_target, motion_direction)`, which is
    # negative while still approaching the target and only positive once the
    # live pose has gone past it. Rotation has no natural "direction" vector,
    # but for a monotonic single-axis ramp the travelled angle from start is
    # a faithful 1-D stand-in, so `ang - target_ang` plays the same role.
    target_ang = float(quat_angle_rad(final_quat, start_quat))

    t0 = time.perf_counter()
    total = duration_s + max(0.0, settle_s)
    next_target_t = 0.0
    next_print_t = 0.0
    target_period = (1.0 / target_hz) if target_hz else 0.0
    commanded_quat = start_quat.copy()

    while True:
        t = time.perf_counter() - t0
        if t > total:
            break

        if t >= next_target_t:
            commanded_quat = np.asarray(
                quat_at(min(t, duration_s)), dtype=np.float64
            )
            try:
                tracker.set_target(franky.Affine(start_xyz, commanded_quat))
            except Exception as exc:
                trace.alive = False
                trace.abort_reason = f"set_target raised: {type(exc).__name__}: {exc}"
                print(f"  !! {trace.abort_reason}")
                break
            next_target_t = t + target_period if target_period else t

        time.sleep(SAMPLE_DT)
        live_xyz = _tcp(robot)
        live_quat = _quat(robot)
        trace.samples += 1

        xyz_drift = float(np.linalg.norm(live_xyz - start_xyz))
        trace.peak_xyz_drift = max(trace.peak_xyz_drift, xyz_drift)

        ang = quat_angle_rad(live_quat, start_quat)
        trace.peak_ang = max(trace.peak_ang, ang)
        ang_lag = quat_angle_rad(live_quat, commanded_quat)
        trace.peak_ang_lag = max(trace.peak_ang_lag, ang_lag)
        ang_overshoot = ang - target_ang
        trace.peak_ang_overshoot = max(trace.peak_ang_overshoot, ang_overshoot)

        fext = _ext_force_norm(robot)
        if fext is not None:
            trace.peak_fext = max(trace.peak_fext, fext)
        dqn = _joint_vel_norm(robot)
        if dqn is not None:
            trace.peak_dq = max(trace.peak_dq, dqn)

        if t >= next_print_t:
            next_print_t = t + verbose_every
            phase = "move" if t <= duration_s else "settle"
            print(
                f"  t={t:5.2f}s {phase:6s} ang={ang:+.4f}rad "
                f"ang_lag={ang_lag:.4f} xyz_drift={xyz_drift:.4f} "
                f"F={'n/a' if fext is None else f'{fext:5.1f}N'} "
                f"|dq|={'n/a' if dqn is None else f'{dqn:.3f}'} "
                f"run={tracker.is_running}"
            )

        if not tracker.is_running:
            trace.alive = False
            trace.abort_reason = "tracker died (async control thread gone)"
            print(f"  !! {trace.abort_reason} at t={t:.2f}s")
            break
        if dqn is not None and dqn > guards.max_dq_rad_s:
            _brake(
                tracker, robot, franky, "lag",
                f"|dq|={dqn:.3f}rad/s > {guards.max_dq_rad_s:.3f}rad/s "
                "(rotation ladder circuit breaker)",
                trace,
            )
            break
        if xyz_drift > guards.max_xyz_drift_m:
            _brake(
                tracker, robot, franky, "fence",
                f"xyz drifted {xyz_drift:.4f}m > {guards.max_xyz_drift_m:.4f}m "
                "while only orientation was commanded",
                trace,
            )
            break
        if trace.peak_ang_overshoot > guards.max_ang_overshoot_rad:
            _brake(
                tracker, robot, franky, "fence",
                f"orientation overshoot {trace.peak_ang_overshoot:.4f}rad > "
                f"{guards.max_ang_overshoot_rad:.4f}rad",
                trace,
            )
            break
        if ang_lag > guards.max_ang_lag_rad:
            _brake(
                tracker, robot, franky, "lag",
                f"orientation lag {ang_lag:.4f}rad > {guards.max_ang_lag_rad:.4f}rad "
                "(not tracking)",
                trace,
            )
            break

    if not trace.stop_cause:
        try:
            tracker.stop()
            trace.stop_cause = "clean"
            print("tracker.stop() clean")
        except Exception as exc:
            trace.stop_cause = f"{type(exc).__name__}: {exc}"
            trace.alive = False
            print(f"tracker.stop() surfaced: {trace.stop_cause}")

    end_ang = quat_angle_rad(_quat(robot), start_quat)
    time.sleep(0.2)
    final_quat_live = _quat(robot)
    trace.final_ang = quat_angle_rad(final_quat_live, start_quat)
    trace.settle_ang_drift = float(trace.final_ang - end_ang)
    _report(robot, "after")
    print(f"{label}: {trace.summary()}")
    if trace.notes:
        for note in trace.notes:
            print(f"  note: {note}")
    return trace


def _rot_guards_for(args) -> RotGuards:
    return RotGuards(
        max_xyz_drift_m=float(args.max_xyz_drift),
        max_ang_overshoot_rad=float(args.max_ang_overshoot),
        max_ang_lag_rad=float(args.max_ang_lag),
        max_dq_rad_s=float(args.max_dq),
    )


def test_rotation(robot, franky, args) -> bool:
    """Ramp orientation by --drx/--dry/--drz (euler xyz, rad), then hold+watch."""
    start_quat = _quat(robot).copy()
    drpy = np.array([args.drx, args.dry, args.drz], dtype=np.float64)
    ang_total = float(np.linalg.norm(drpy))
    duration = args.seconds
    if args.rot_ramp_speed:
        duration = ang_total / float(args.rot_ramp_speed) if ang_total > 0 else args.seconds
        print(f"rot-ramp-speed {args.rot_ramp_speed:.3f}rad/s -> duration {duration:.2f}s")

    def quat_at(t: float) -> np.ndarray:
        frac = min(1.0, max(0.0, t / duration)) if duration > 0 else 1.0
        delta = SciRotation.from_euler("xyz", drpy * frac)
        return (delta * SciRotation.from_quat(start_quat)).as_quat()

    trace = run_tracker_rotation(
        robot,
        franky,
        label=(
            f"rotation ramp drpy={drpy.tolist()}rad over {duration:.2f}s "
            f"({ang_total / max(duration, 1e-6):.3f}rad/s) + {args.settle_seconds:.1f}s settle"
        ),
        quat_at=quat_at,
        duration_s=duration,
        settle_s=args.settle_seconds,
        guards=_rot_guards_for(args),
    )
    moved = abs(trace.final_ang) > 0.01
    ok = trace.alive and moved and trace.peak_ang_overshoot <= args.max_ang_overshoot
    print(
        f"rotation verdict: moved={moved} ({trace.final_ang:+.4f}rad of "
        f"{ang_total:+.4f}rad) overshoot={trace.peak_ang_overshoot:+.4f}rad "
        f"alive={trace.alive}"
    )
    return ok


def test_rotation_waypoints(robot, franky, args) -> bool:
    """10 Hz zero-order-hold rotation staircase -- the 2.4b-rot reproduction of
    what ``FrankaEnv.step`` commands one cycle at a time (closed-loop
    re-commanding from a fresh measured pose every cycle looks, worst case, like
    a sustained per-cycle increment of ``action_scale[1]``).

    This is the ONE test that can confirm or refute
    ``motion_limits.STEP_ROT_SPEED_RAD_S_DEFAULT``: it drives the exact rung
    shape (absolute waypoints at ``step_frequency``) that a saturated policy
    action would produce, and reports the same ``|dq|`` LOG-034 used to diagnose
    the incident.
    """
    start_quat = _quat(robot).copy()
    drpy = np.array([args.drx, args.dry, args.drz], dtype=np.float64)
    hz = args.rot_waypoint_hz
    ang_total = float(np.linalg.norm(drpy))
    duration = args.seconds if not args.rot_ramp_speed else ang_total / float(args.rot_ramp_speed)
    n = max(1, int(round(duration * hz)))

    def quat_at(t: float) -> np.ndarray:
        idx = min(n, int(np.floor(t * hz)) + 1)
        delta = SciRotation.from_euler("xyz", drpy * (idx / n))
        return (delta * SciRotation.from_quat(start_quat)).as_quat()

    trace = run_tracker_rotation(
        robot,
        franky,
        label=(
            f"step() rotation replay: {n} waypoints @ {hz:.0f}Hz, "
            f"drpy={drpy.tolist()}rad in {duration:.2f}s "
            f"({ang_total / max(duration, 1e-6):.3f}rad/s, i.e. action_scale[1]="
            f"{ang_total / max(n, 1):.4f}rad/step at {hz:.0f}Hz)"
        ),
        quat_at=quat_at,
        duration_s=duration,
        settle_s=args.settle_seconds,
        guards=_rot_guards_for(args),
        target_hz=hz,
    )
    moved = abs(trace.final_ang) > 0.01
    ok = trace.alive and moved and trace.peak_ang_overshoot <= args.max_ang_overshoot
    print(
        f"step()-rotation-replay verdict: moved={moved} "
        f"overshoot={trace.peak_ang_overshoot:+.4f}rad peak|dq|={trace.peak_dq:.3f}rad/s "
        f"alive={trace.alive} stop={trace.stop_cause}"
    )
    per_step_rad = ang_total / max(n, 1)
    cap = max_action_scale_rot(hz)
    verdict = "within" if per_step_rad <= cap + 1e-9 else "ABOVE"
    print(
        f"  cf. motion_limits cap: this rung commands {per_step_rad:.4f}rad/step "
        f"at {hz:.0f}Hz, {verdict} the current max_action_scale_rot={cap:.4f}rad/step "
        f"({step_rot_speed_rad_s():.2f}rad/s) -- adjust RLINF_CUBE_STEP_ROT_SPEED "
        "once this rung's peak|dq| and ang_lag are judged safe (or not)."
    )
    return ok


# --------------------------------------------------------------------------
# Individual tests
# --------------------------------------------------------------------------


def _guards_for(
    robot, *, dz: float, args, extra_ceiling: float = 0.05
) -> Guards:
    """Guards anchored on the pose the arm is in *now*."""
    z0 = float(_tcp(robot)[2])
    ceiling = args.z_ceiling if args.z_ceiling is not None else z0 + max(dz, 0.0) + extra_ceiling
    floor = args.z_floor if args.z_floor is not None else z0 + min(dz, 0.0) - extra_ceiling
    return Guards(
        z_ceiling=float(ceiling),
        z_floor=float(floor),
        max_overshoot_m=float(args.max_overshoot),
        max_lag_m=float(args.max_lag),
    )


def test_hold(robot, franky, args) -> bool:
    """Target = current pose. Is the async torque motion alive, and does it sag?"""
    start = _tcp(robot).copy()
    trace = run_tracker_motion(
        robot,
        franky,
        label=f"impedance hold (zero displacement) {args.seconds:.1f}s",
        target_at=lambda _t: start,
        duration_s=args.seconds,
        settle_s=0.0,
        guards=_guards_for(robot, dz=0.0, args=args, extra_ceiling=0.03),
        verbose_every=0.5,
    )
    ok = trace.alive and trace.stop_cause == "clean"
    print(f"hold verdict: alive={trace.alive} sag={trace.final_dz:+.4f}m")
    return ok


def test_impedance(robot, franky, args) -> bool:
    """Ramp z, then **hold and keep watching** -- overshoot lives in the settle."""
    start = _tcp(robot).copy()
    dz = args.dz
    duration = args.seconds
    if args.ramp_speed:
        duration = abs(dz) / float(args.ramp_speed)
        print(f"ramp-speed {args.ramp_speed:.3f}m/s -> duration {duration:.2f}s")

    def target_at(t: float) -> np.ndarray:
        frac = min(1.0, max(0.0, t / duration)) if duration > 0 else 1.0
        return start + np.array([0.0, 0.0, dz * frac])

    trace = run_tracker_motion(
        robot,
        franky,
        label=(
            f"impedance ramp z {dz:+.3f}m over {duration:.2f}s "
            f"({abs(dz) / max(duration, 1e-6) * 100:.1f}cm/s) + {args.settle_seconds:.1f}s settle"
        ),
        target_at=target_at,
        duration_s=duration,
        settle_s=args.settle_seconds,
        guards=_guards_for(robot, dz=dz, args=args),
    )
    moved = abs(trace.final_dz) > 0.005
    ok = trace.alive and moved and trace.peak_overshoot <= args.max_overshoot
    print(
        f"impedance verdict: moved={moved} ({trace.final_dz:+.4f} of {dz:+.4f}) "
        f"overshoot={trace.peak_overshoot:+.4f} alive={trace.alive}"
    )
    return ok


def test_waypoints(robot, franky, args, *, with_recover: bool = False) -> bool:
    """Replay what ``FrankaEnv._interpolate_move`` actually commands.

    ``_interpolate_move(pose, timeout=t)`` emits ``t * step_frequency`` absolute
    waypoints from a pose sampled **once** at the start, on a ``time.sleep``
    timer, with no feedback. That open-loop staircase -- not a smooth ramp -- is
    what ran during LOG-019, so it is what has to be reproduced to study it.

    With ``with_recover=True`` each waypoint is preceded by
    ``robot.recover_from_errors()``, exactly as ``_move_action`` -> ``_clear_error``
    does at 10 Hz inside the live torque motion. Comparing the two runs settles
    LOG-019 R5 (whose earlier "refutation" in LOG-014 was invalid, having been
    measured while the robot was in ``UserStopped``).
    """
    start = _tcp(robot).copy()
    dz = args.dz
    hz = args.waypoint_hz
    duration = args.seconds if not args.ramp_speed else abs(dz) / float(args.ramp_speed)
    n = max(1, int(round(duration * hz)))

    def target_at(t: float) -> np.ndarray:
        # Zero-order hold: which of the n waypoints are we on?
        idx = min(n, int(np.floor(t * hz)) + 1)
        return start + np.array([0.0, 0.0, dz * (idx / n)])

    hook = None
    if with_recover:
        def hook() -> None:  # noqa: D401
            robot.recover_from_errors()

    trace = run_tracker_motion(
        robot,
        franky,
        label=(
            f"interpolate_move replay: {n} waypoints @ {hz:.0f}Hz, "
            f"dz={dz:+.3f}m in {duration:.2f}s ({abs(dz) / max(duration, 1e-6) * 100:.1f}cm/s)"
            + (" WITH recover_from_errors per waypoint" if with_recover else "")
        ),
        target_at=target_at,
        duration_s=duration,
        settle_s=args.settle_seconds,
        guards=_guards_for(robot, dz=dz, args=args),
        target_hz=hz,
        between_targets=hook,
    )
    moved = abs(trace.final_dz) > 0.005
    ok = trace.alive and moved and trace.peak_overshoot <= args.max_overshoot
    print(
        f"waypoints verdict: moved={moved} overshoot={trace.peak_overshoot:+.4f} "
        f"alive={trace.alive} stop={trace.stop_cause}"
    )
    return ok


def test_cartesian_motion(robot, franky, args) -> bool:
    """Blocking ``robot.move(CartesianMotion)``.

    Kept for completeness, but note LOG-018: this reflexes with
    ``cartesian_motion_generator_joint_velocity_discontinuity`` if the arm still
    has velocity when it starts (its position motion generator requires a
    continuous initial joint velocity), and near a shoulder singularity (q2 ~ 0)
    its IK is ill-conditioned. The dwell below exists for the first reason --
    do not remove it and then conclude the primitive is broken.
    """
    dz = args.dz
    print(f"\n=== blocking CartesianMotion: z {dz:+.3f}m ===")
    robot.recover_from_errors()
    dwell = max(0.5, args.settle_seconds)
    print(f"dwell {dwell:.1f}s so initial joint velocity is ~0 (LOG-018)")
    time.sleep(dwell)
    dqn = _joint_vel_norm(robot)
    if dqn is not None and dqn > 0.02:
        print(f"refusing: |dq|={dqn:.4f}rad/s still moving; wait longer")
        return False

    start_xyz = _tcp(robot)
    quat = _quat(robot)
    target = start_xyz + np.array([0.0, 0.0, dz])
    motion = franky.CartesianMotion(
        franky.Affine(target, quat),
        reference_type=franky.ReferenceType.Absolute,
    )
    t0 = time.perf_counter()
    try:
        robot.move(motion)
        print(f"move() returned in {time.perf_counter() - t0:.2f}s")
    except Exception as exc:
        print(f"move() raised: {type(exc).__name__}: {exc}")
    _report(robot, "after move")
    total = float(_tcp(robot)[2] - start_xyz[2])
    print(f"CartesianMotion total dz={total:+.4f}m (target {dz:+.4f})")
    return abs(total) > 0.005


# --------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--robot-ip", default=DEFAULT_IP)
    parser.add_argument("--probe", action="store_true", help="read-only, no motion")
    parser.add_argument("--test-hold", action="store_true")
    parser.add_argument("--test-impedance", action="store_true")
    parser.add_argument(
        "--test-waypoints",
        action="store_true",
        help="replay the open-loop _interpolate_move staircase (LOG-019 regime)",
    )
    parser.add_argument(
        "--test-recover-loop",
        action="store_true",
        help="--test-waypoints plus recover_from_errors() per waypoint (LOG-019 R5)",
    )
    parser.add_argument("--test-cartesian-motion", action="store_true")
    parser.add_argument(
        "--test-rotation",
        action="store_true",
        help="ramp orientation by --drx/--dry/--drz, xyz fixed (2.4b-rot, LOG-034/T16)",
    )
    parser.add_argument(
        "--test-rotation-waypoints",
        action="store_true",
        help=(
            "10Hz ZOH rotation staircase -- reproduces FrankaEnv.step's per-cycle "
            "rotation command shape (2.4b-rot, LOG-034/T16)"
        ),
    )
    parser.add_argument(
        "--yes-move",
        action="store_true",
        help="required for any test that commands the arm",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="run motion tests even when robot_mode is not Idle",
    )
    parser.add_argument("--dz", type=float, default=0.03, help="lift height (m)")
    parser.add_argument("--seconds", type=float, default=3.0, help="commanded duration (s)")
    parser.add_argument(
        "--ramp-speed",
        type=float,
        default=None,
        help="derive duration from this speed (m/s) instead of --seconds",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=2.0,
        help="observe-only window after the ramp; where overshoot appears (s)",
    )
    parser.add_argument(
        "--waypoint-hz",
        type=float,
        default=10.0,
        help="set_target rate for --test-waypoints (FrankaEnv uses step_frequency=10)",
    )
    parser.add_argument(
        "--max-overshoot",
        type=float,
        default=0.02,
        help="abort if the TCP goes this far past the final target (m)",
    )
    parser.add_argument(
        "--max-lag",
        type=float,
        default=guard_max_lag_m(),
        help="abort if |measured - commanded| exceeds this (m)",
    )
    parser.add_argument(
        "--z-ceiling", type=float, default=None, help="absolute z abort ceiling (m)"
    )
    parser.add_argument(
        "--z-floor", type=float, default=None, help="absolute z abort floor (m)"
    )
    parser.add_argument(
        "--drx", type=float, default=0.0, help="rotation test: euler roll delta (rad)"
    )
    parser.add_argument(
        "--dry", type=float, default=0.0, help="rotation test: euler pitch delta (rad)"
    )
    parser.add_argument(
        "--drz", type=float, default=0.15, help="rotation test: euler yaw delta (rad)"
    )
    parser.add_argument(
        "--rot-ramp-speed",
        type=float,
        default=None,
        help="derive rotation duration from this speed (rad/s) instead of --seconds",
    )
    parser.add_argument(
        "--rot-waypoint-hz",
        type=float,
        default=10.0,
        help="set_target rate for --test-rotation-waypoints (matches step_frequency=10)",
    )
    parser.add_argument(
        "--max-xyz-drift",
        type=float,
        default=0.02,
        help="rotation tests: abort if xyz drifts this far while only orientation "
        "is commanded (m)",
    )
    parser.add_argument(
        "--max-ang-overshoot",
        type=float,
        default=0.05,
        help="rotation tests: abort if orientation goes this far past the final "
        "target (rad)",
    )
    parser.add_argument(
        "--max-ang-lag",
        type=float,
        default=0.15,
        help="rotation tests: abort if |measured-commanded| orientation angle "
        "exceeds this (rad)",
    )
    parser.add_argument(
        "--max-dq",
        type=float,
        default=1.0,
        help="rotation tests: abort the INSTANT |dq| exceeds this (rad/s) -- an "
        "extra circuit breaker for an authority never measured before (LOG-034's "
        "incident reached 2.69rad/s; this aborts an order of magnitude earlier)",
    )
    parser.add_argument(
        "--force-ceiling",
        type=float,
        default=None,
        help="per-axis commanded force ceiling (N); default 20, also capped by "
        "--force-norm-ceiling / sqrt(3). Writes RLINF_CUBE_FORCE_CEILING_N before "
        "the tracker is built, so `authority:` reflects the change",
    )
    parser.add_argument(
        "--force-norm-ceiling",
        type=float,
        default=None,
        help="worst-case (3-axis) commanded force ceiling (N); default 40. If "
        "--force-ceiling / sqrt(3) exceeds this, THIS is the one that actually "
        "binds -- e.g. --force-ceiling 35 alone still yields only 23.1N/axis",
    )
    parser.add_argument(
        "--torque-norm-ceiling",
        type=float,
        default=None,
        help="worst-case (3-axis) commanded torque ceiling (N.m); default 12 = the "
        "j5-j7 joint limit",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    wants_motion = bool(
        args.test_hold
        or args.test_impedance
        or args.test_waypoints
        or args.test_recover_loop
        or args.test_cartesian_motion
        or args.test_rotation
        or args.test_rotation_waypoints
    )
    if wants_motion and not args.yes_move:
        raise SystemExit("refusing to move without --yes-move (stand by the e-stop)")
    if not wants_motion:
        args.probe = True

    # Export BEFORE import franky / describe_authority, so the printed authority
    # and the tracker actually built reflect the override -- same ordering as
    # step_cube_place_robot.py.
    if args.force_ceiling is not None:
        os.environ["RLINF_CUBE_FORCE_CEILING_N"] = repr(float(args.force_ceiling))
    if args.force_norm_ceiling is not None:
        os.environ["RLINF_CUBE_FORCE_NORM_CEILING_N"] = repr(float(args.force_norm_ceiling))
    if args.torque_norm_ceiling is not None:
        os.environ["RLINF_CUBE_TORQUE_NORM_CEILING_NM"] = repr(
            float(args.torque_norm_ceiling)
        )

    import franky

    robot = franky.Robot(args.robot_ip)
    robot.relative_dynamics_factor = DYNAMICS_FACTOR
    print(f"connected to {args.robot_ip} (relative_dynamics_factor={DYNAMICS_FACTOR})")
    print(f"authority: {describe_authority(K_TRANS, K_ROT)}")
    for msg in clip_shortfall(K_TRANS, K_ROT):
        print(f"WARNING: {msg}")
    mode = probe(robot)

    if args.probe and not wants_motion:
        return 0
    if mode != "RobotMode.Idle" and not args.force:
        print("\nskipping motion tests (pass --force to try anyway)")
        return 1

    # Tighten libfranka's own 1 kHz reflex the same way
    # FrankyControllerExtended.__init__ does, so 2.4b runs under the SAME hardware
    # bound as the env path (round-2 audit finding 3) rather than upstream's
    # untouched 100 N / 25 N.m.
    force_thresholds = cartesian_collision_thresholds()
    try:
        robot.set_collision_behavior(_fc._TORQUE_THRESHOLD, force_thresholds)
        print(
            f"collision behavior tightened: cartesian force/torque "
            f"thresholds={[round(v, 2) for v in force_thresholds]} "
            f"(was {_fc._FORCE_THRESHOLD}), joint torque left at {_fc._TORQUE_THRESHOLD}"
        )
    except Exception as exc:  # noqa: BLE001 - must not block the diagnostic
        print(
            f"WARNING: could not tighten collision behavior ({type(exc).__name__}: "
            f"{exc}); libfranka's reflex stays at {_fc._FORCE_THRESHOLD}, far above "
            "the commanded ceiling"
        )

    results: dict[str, bool] = {}

    def _run(name: str, fn) -> bool:
        ok = fn()
        results[name] = ok
        if not ok:
            print(f"\naborting the remaining motion tests: {name} FAILED")
        return ok

    # Stop at the first failure rather than continuing to the next test: every
    # test's first action is robot.recover_from_errors(), so running the next one
    # after an abort clears the fault and commands the arm again immediately --
    # voiding the ladder's whole premise of "one rung at a time, back to
    # diag-probe between rungs" (round-2 audit finding 2).
    ok = True
    if ok and args.test_hold:
        ok = _run("impedance hold alive", lambda: test_hold(robot, franky, args))
    if ok and args.test_impedance:
        ok = _run("impedance ramp + settle", lambda: test_impedance(robot, franky, args))
    if ok and args.test_waypoints:
        ok = _run("interpolate_move replay", lambda: test_waypoints(robot, franky, args))
    if ok and args.test_recover_loop:
        ok = _run(
            "replay + per-waypoint recover",
            lambda: test_waypoints(robot, franky, args, with_recover=True),
        )
    if ok and args.test_cartesian_motion:
        ok = _run("blocking CartesianMotion", lambda: test_cartesian_motion(robot, franky, args))
    if ok and args.test_rotation:
        ok = _run("rotation ramp + settle", lambda: test_rotation(robot, franky, args))
    if ok and args.test_rotation_waypoints:
        ok = _run(
            "step() rotation replay",
            lambda: test_rotation_waypoints(robot, franky, args),
        )

    print("\n=== summary ===")
    for name, res in results.items():
        print(f"{name}: {'OK' if res else 'FAILED'}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
