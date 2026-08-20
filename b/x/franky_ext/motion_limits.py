"""Central motion-authority limits for the franky cube-place stack.

Why this module exists (LOG-019, 2026-08-19): a ``reset`` commanded a 10 cm lift
and the arm reached **26 cm above the highest commanded target** (+36 cm from its
start, +27 cm above the configured safety box). Nothing aborted; a human pressed
the hardware user-stop.

Two independent *defaults* had multiplied into an authority nobody decided on:

* ``PegInsertionConfig.compliance_param.translational_stiffness = 2000`` N/m
  (pushed into franky by ``reconfigure_compliance_params`` on every ``reset``)
* franky's own ``_CART_TRANS_ERROR_CLIP_M = 0.05`` m

  => per-axis spring-force ceiling ``2000 * 0.05 = 100 N``, where franky's own
  default *pairing* is ``500 * 0.05 = 25 N``.

and the commanded *speed* was 10 cm/s (``_interpolate_move(pose, timeout=1)``
splits a 10 cm move into ``timeout * step_frequency = 10`` waypoints at 10 Hz),
i.e. 10x the only speed ever validated on this arm.

So: every quantity that bounds *how hard*, *how fast* and *how far* the arm may
be commanded lives here, and the dangerous quantity is always a **product** --
``stiffness x error_clip`` is a force, ``displacement / duration`` is a speed.
Never set one of a pair from a config and the other from a library default.

What ``stiffness x clip`` does and does not bound
-------------------------------------------------
Read franky's ``cartesian_impedance_base.cpp`` before trusting any of this::

    error.head(3) = O_T_EE.translation() - target.translation();
    error.head(3) = error.head(3).cwiseMax(-clip).cwiseMin(clip);   // ELEMENTWISE
    wrench = -K * error - D * (measured_twist - desired_twist);     // D UNCLIPPED

Three consequences, all of which the first version of this module got wrong:

1. **The clip is elementwise, so the bound is per-axis, not on the norm.** A
   fully saturated three-axis error commands ``K * clip * sqrt(3)``. That is why
   there are two ceilings below (per-axis and norm) and the derivation honours
   whichever binds first.
2. **The damping term is never clipped.** franky's default damping is critical,
   ``D ~ 2*sqrt(K*Lambda)`` -- roughly 155 N.s/m at ``K_t = 2000`` for a ~3 kg
   task-space inertia. Since ``desired_twist`` is always zero here (upstream
   deliberately feeds no twist), that term always *opposes* motion and cannot
   drive a runaway -- but it does mean the peak force a hand would feel is not
   bounded by these numbers, and that raising stiffness raises damping as
   ``sqrt(K)`` no matter how the clip is derived. **Deriving the clip does not
   make K_t=2000 equivalent to K_t=500.**
3. Gravity and Coriolis are compensated by libfranka and by the controller, but
   **joint friction is not** (franky's friction feedforward defaults to
   disabled). Measured on this arm: 4.7 mm of lag at 1 cm/s with ``K_t = 2000``,
   i.e. ~9.4 N of spring force, of which only ~1.6 N is viscous -- so ~8 N is
   stiction and it does **not** scale with speed. Budget accordingly, and do not
   linearly extrapolate lag with velocity (LOG-021 corrects exactly that error).

All limits are overridable by environment variable for on-robot tuning, and every
override is clamped to a hard range that cannot be exceeded, so a typo in a YAML
or a shell export cannot recreate the LOG-019 authority.

Nothing here talks to the robot; it is pure arithmetic so the phase-1A dummy
regression covers it.
"""

from __future__ import annotations

import os
from typing import Optional, Sequence

import numpy as np

# --------------------------------------------------------------------------
# Hard ranges. Environment overrides are clamped into these; they are not
# themselves overridable.
# --------------------------------------------------------------------------

#: Per-axis commanded spring-force ceiling, i.e. ``translational_stiffness *
#: translational_error_clip`` on ONE axis. This is the quantity franky's knob
#: actually controls.
#:
#: 20 N: measured need on this arm is ~9.4 N at 1 cm/s and ~11 N at 2 cm/s
#: (mostly stiction, see the module docstring), so 20 N is ~1.8x headroom on the
#: axis that is moving. franky's own default pairing is 25 N.
FORCE_CEILING_N_DEFAULT = 20.0
FORCE_CEILING_N_RANGE = (5.0, 60.0)

#: Worst-case ceiling on the *norm* of the commanded spring force, i.e. all three
#: axes saturated at once. The derivation lowers the per-axis clip if
#: ``per_axis * sqrt(3)`` would exceed this.
FORCE_NORM_CEILING_N_DEFAULT = 40.0
FORCE_NORM_CEILING_N_RANGE = (10.0, 120.0)

#: Per-axis commanded torque ceiling, ``rotational_stiffness * rot_clip``.
#: 6 N.m, not 10: Franka j5-j7 are limited to roughly 12 N.m, and the worst-case
#: three-axis norm is ``per_axis * sqrt(3)``, so a 10 N.m per-axis ceiling would
#: command up to 17.3 N.m -- above the very limit it was chosen to respect.
TORQUE_CEILING_NM_DEFAULT = 6.0
TORQUE_CEILING_NM_RANGE = (1.0, 15.0)

#: Worst-case norm ceiling for commanded torque. 12 N.m = the j5-j7 joint limit.
TORQUE_NORM_CEILING_NM_DEFAULT = 12.0
TORQUE_NORM_CEILING_NM_RANGE = (4.0, 30.0)

#: Stiffness itself is clamped too: a REPL ``impedance 99999 9999`` or a config
#: typo must not be able to build a monster tracker even at a small clip.
STIFFNESS_TRANS_RANGE = (50.0, 3000.0)
STIFFNESS_ROT_RANGE = (5.0, 300.0)

#: Error clip bounds. Too small and the arm cannot overcome its own stiction;
#: too large and a lagging target turns into a catapult.
ERROR_CLIP_M_RANGE = (0.002, 0.05)
ERROR_CLIP_RAD_RANGE = (0.005, 0.30)

#: Speed cap for ``_interpolate_move``. The validated regime is 1 cm/s; 2 cm/s
#: keeps a modest margin while still finishing a 3 cm lift in under 2 s.
INTERP_SPEED_M_S_DEFAULT = 0.02
INTERP_SPEED_M_S_RANGE = (0.002, 0.06)

INTERP_SPEED_RAD_S_DEFAULT = 0.15
INTERP_SPEED_RAD_S_RANGE = (0.02, 0.60)

#: Speed cap for the ``step()`` path, m/s -- i.e. what the *policy* may command.
#:
#: The interpolation cap above does **not** cover this. ``FrankaEnv.step`` does
#: ``next_position[:3] += action[:3] * action_scale[0]`` and calls ``_move_action``
#: directly, so with PegInsertion's ``action_scale[0] = 0.02`` at
#: ``step_frequency = 10`` a full-scale action commands **20 cm/s** -- twice
#: LOG-019's 10 cm/s and 20x the validated 1 cm/s. Nothing downstream throttled it:
#: ``_CART_MAX_STEP_M = 0.10`` does not bite at 2 cm, and because ``step``
#: recomputes its target from the freshly measured pose each cycle the lag never
#: accumulates, so the guard's lag test cannot see it either.
#:
#: 5 cm/s, rather than the interpolation cap, because ``step`` is closed-loop
#: against the measured pose every cycle while ``_interpolate_move`` is open-loop.
#: It is also below the arm's achievable terminal velocity under the commanded
#: force ceiling (~13 cm/s against ~155 N.s/m of damping), so the action space now
#: corresponds to motion the arm can actually produce -- previously most of the
#: action range was physically unreachable, which is bad for the policy as well as
#: for safety. Phase 3 should revisit this with data.
STEP_SPEED_M_S_DEFAULT = 0.05
STEP_SPEED_M_S_RANGE = (0.002, 0.20)

#: Speed cap for the ``step()`` path's ROTATION, rad/s -- LOG-034's missing half
#: of :data:`STEP_SPEED_M_S_DEFAULT`. ``FrankaEnv.step`` does::
#:
#:     next_position[3:] = (R.from_euler("xyz", action[3:6] * action_scale[1])
#:                           * R.from_quat(current_quat)).as_quat()
#:
#: with PegInsertion's ``action_scale[1] = 0.1`` rad at ``step_frequency = 10``: a
#: full-scale action is a **1.0 rad/s per-axis** rotation command, worst-case
#: 3-axis ``1.73 rad/s`` -- and nothing on this path clamped it, exactly the same
#: shape of gap that ``STEP_SPEED_M_S_DEFAULT`` closed for translation. LOG-034's
#: WATCHDOG trip happened with the arm within 2-8% of its reach limit on two box
#: corners (``NEAR-SINGULAR``), where a cartesian-space rotation command demands a
#: disproportionate joint velocity; ``|dq|`` reached 2.69 rad/s before the brake.
#:
#: 0.3 rad/s, not (yet) a measured number: it is ``INTERP_SPEED_RAD_S_DEFAULT``
#: (0.15 rad/s, the only rotation speed ever exercised on this arm without
#: incident) doubled, mirroring the ~2.5x ratio between
#: ``STEP_SPEED_M_S_DEFAULT`` and ``INTERP_SPEED_M_S_DEFAULT`` for translation.
#: Revisit with ``diag_franky_motion.py --test-rotation-waypoints`` data (2.4b-rot)
#: before trusting it near a reach boundary the way 2.4b's translation ladder was
#: trusted only after being measured.
STEP_ROT_SPEED_RAD_S_DEFAULT = 0.3
STEP_ROT_SPEED_RAD_S_RANGE = (0.02, 1.0)

#: Minimum / maximum duration of a single ``_interpolate_move`` call, seconds.
INTERP_DURATION_S_RANGE = (0.6, 20.0)

#: Largest translation allowed through a 10 Hz impedance interpolation.
#:
#: Note the interaction with ``INTERP_DURATION_S_RANGE``: clamping the duration at
#: 20 s means anything past ``20 * 0.02 = 0.4 m`` would silently be commanded
#: *faster* than the speed cap again. Rather than paper over that with a bigger
#: duration cap, refuse: a third of a metre is already far more than this task's
#: safety box, and RLinf's own franky env (``DualFrankaEnv._go_to_rest``) does not
#: use cartesian impedance for large displacements at all -- it uses a blocking
#: ``reset_joint``.
MAX_INTERP_DISTANCE_M = 0.35

#: Largest rotation allowed through the same path, radians (~34 deg).
#:
#: Needed for the same reason as the translation cap, and reachable in practice:
#: if a calibration is written from a quaternion's first three components instead
#: of euler angles, the target orientation lands near identity instead of near
#: gripper-down, the safety box and the orientation fence both re-centre on it, so
#: *no guard fires* -- and reset commands a ~180 deg wrist flip with a cube in the
#: gripper. This cap is what refuses that.
MAX_INTERP_ANGLE_RAD = 0.6

#: How far outside ``ee_pose_limit`` the *measured* TCP may stray before the
#: motion guard aborts. This is the geometric fence that LOG-019 lacked.
GUARD_MARGIN_M_DEFAULT = 0.05
GUARD_MARGIN_M_RANGE = (0.01, 0.15)

#: Same, but on -z only. Deliberately much tighter than the general margin:
#: below the contact point there is a physical table, and "5 cm of slack" there
#: means authorising a 5 cm press into a solid surface.
GUARD_FLOOR_MARGIN_M_DEFAULT = 0.01
GUARD_FLOOR_MARGIN_M_RANGE = (0.002, 0.05)

#: How far the measured TCP may lag the commanded target before the guard treats
#: the controller as "not tracking" (dead, saturated, or unstable). Ordinary
#: impedance lag at the capped speed is a few millimetres; ``step()`` can legally
#: command ``action_scale[0] * sqrt(3) = 0.035`` m from a pose read one cycle
#: earlier, so this must stay comfortably above that.
GUARD_MAX_LAG_M_DEFAULT = 0.05
GUARD_MAX_LAG_M_RANGE = (0.02, 0.20)

#: Extra slack added on top of the task's own orientation box when fencing the
#: measured orientation. See :func:`orientation_fence_rad`.
GUARD_ORIENT_SLACK_RAD_DEFAULT = 0.20
GUARD_ORIENT_SLACK_RAD_RANGE = (0.05, 0.80)

#: Joint-speed norm at which the motion guard treats the arm as running away,
#: rad/s -- LOG-037's T19 backstop.
#:
#: This is defence in depth, not the fix: it cannot prevent anything, it can only
#: name the failure correctly. Both real trips (LOG-034, LOG-036) were *reported*
#: as ``lag`` because the joint speed was never a criterion -- the arm failed to
#: track, the position error grew, and the lag gate fired second-hand. LOG-036's
#: ``|dq|=2.03 rad/s`` was in the trip message as a diagnostic only.
#:
#: 1.2 rad/s sits between what is known-normal and what is known-bad, with a
#: gap on both sides (all values measured on this arm):
#:
#:   * 0.379 rad/s -- worst peak across the whole 2.4b-rot rotation ladder.
#:   * 0.81 rad/s -- what the *full* allowed twist demands at the hover pose
#:     (``diag_franky_jacobian.py``), i.e. legitimate full-speed commanding.
#:   * 2.03 / 2.69 rad/s -- LOG-036 / LOG-034 trips.
#:   * 2.075 rad/s -- Panda's own per-joint limit for joints 1-4, so this stays
#:     below the point where libfranka would start objecting on its own.
GUARD_MAX_DQ_RAD_S_DEFAULT = 1.2
GUARD_MAX_DQ_RAD_S_RANGE = (0.5, 3.0)

#: Fraction of Panda's per-joint velocity limits that one ``step()`` command may
#: *demand*, via ``dq_req = J^+ (dpose/dt)`` -- LOG-037's T19 fix.
#:
#: This is the one limit in this file that bounds the quantity the two incidents
#: actually failed on. Every other cap here is Cartesian, and LOG-037 measured
#: why that is not enough: the ratio between a Cartesian command and the joint
#: velocity it demands is set by the Jacobian's conditioning, which varied by
#: **14.6x** between the hover pose (``sigma_min=0.1115``) and the pose training
#: tripped at (``sigma_min=0.0062``). The same fully-compliant command demanded
#: 0.81 rad/s at one and 11.77 rad/s at the other -- 9.64 rad/s on joint 4 alone,
#: 4.6x that joint's own limit.
#:
#: 0.5 leaves the hover pose completely unclamped (its worst per-joint demand at
#: full commanded speed is 0.54 rad/s = 0.26 of the limit) while shrinking a step
#: at the LOG-036 trip pose to ~11% of its requested size. That asymmetry is the
#: point: the clamp should be invisible where the arm is well conditioned and
#: bite hard where it is not.
JOINT_VEL_DEMAND_FRACTION_DEFAULT = 0.5
JOINT_VEL_DEMAND_FRACTION_RANGE = (0.05, 1.0)

#: Smallest singular value of the 6x7 Jacobian below which the arm's *joint*
#: configuration is warned about, even though its TCP pose is perfectly legal --
#: LOG-040's T22.
#:
#: A 7-axis arm has one redundant degree of freedom, and cartesian impedance
#: control places no bound on it whatsoever: the elbow drifts wherever the
#: null-space torques leave it. LOG-040 found the arm at ``sigma_min=0.0009``
#: with the TCP sitting comfortably *inside* the safety box -- no fence, no
#: orientation gate and no reach warning had anything to object to, because none
#: of them look at joint configuration at all. The first thing that noticed was
#: the interpolation clamp refusing a reset it could not perform safely, which is
#: several steps too late to be useful to an operator.
#:
#: Measured anchors on this arm, all from ``diag_franky_jacobian.py``:
#:
#:   * 0.1115 / 0.1141 -- the hover pose (LOG-037) and the hand-guided recovery
#:     that followed LOG-040. Both known-good; nothing clamps here.
#:   * ~0.055 -- where the joint-demand clamp starts to bite, extrapolated from
#:     the hover pose's worst per-joint demand being 0.52 of its budget.
#:   * 0.0062 -- the LOG-036 trip pose.
#:   * 0.0009 -- LOG-040's drifted pose, which could not be reset out of.
#:
#: 0.04 sits below where clamping begins (so a warning means something is
#: genuinely unusual, not merely that the clamp is working) and well above both
#: known-bad poses. It is a *warning* threshold and nothing refuses on it, so
#: erring low costs only silence, never safety. T20's sweep of the safety box is
#: what will replace this estimate with a distribution.
SIGMA_MIN_WARN_DEFAULT = 0.04
SIGMA_MIN_WARN_RANGE = (0.001, 0.5)

#: How many motion-guard trips one env instance may auto-recover from before it
#: gives up and lets the exception end the run -- LOG-040's T21.
#:
#: The budget exists because "recover and carry on" has a failure mode that is
#: worse than crashing: an arm that trips every few steps would be ground against
#: the same bad configuration indefinitely, with the guard dutifully braking each
#: time and the logs scrolling past. A trip is a safety event; recovering from a
#: handful of them is pragmatism, recovering from an unbounded number is the
#: mechanism opting out.
#:
#: 10 is enough to survive the occasional trip an exploring policy will cause
#: near the workspace edges (LOG-040 saw one in the first two rollout epochs)
#: without letting a systematically broken setup run unattended. **Set it to 0 to
#: restore the pre-LOG-040 behaviour**, where any trip ends the run.
GUARD_RECOVERY_BUDGET_DEFAULT = 10
GUARD_RECOVERY_BUDGET_RANGE = (0.0, 100.0)

#: Nominal width of the grasped cube, metres. Measured on this setup during H1:
#: 0.0463 / 0.0464 / 0.0470 / 0.0365. It is a *calibration* value, not a constant --
#: change the cube and you must change this, which is why it is an environment
#: override rather than a literal buried in a branch.
#:
#: Lives here (not in ``franka_libfranka_gripper.py``) so ``tcp_probe.py``'s
#: pre-Ray gripper gate can decide "holding" from the SAME window the controller's
#: gripper uses, rather than trusting libfranka's ``is_grasped`` -- see
#: :func:`holding_from_width`.
CUBE_WIDTH_RANGE_M = (0.005, 0.070)
CUBE_WIDTH_M_DEFAULT = 0.046

#: Half-width of the "this is the cube" window around :data:`CUBE_WIDTH_M_DEFAULT`.
#: Wide enough for finger-pad compression and a slightly off-nominal cube, narrow
#: enough that an empty hand (~0.000) and a fully open hand (0.09) are both out.
HOLD_TOL_RANGE_M = (0.004, 0.025)
HOLD_TOL_M_DEFAULT = 0.012

#: Franka Emika Panda maximum reach, metres (datasheet, flange from shoulder).
PANDA_MAX_REACH_M = 0.855

#: Shoulder height above the base flange, metres -- reach is measured from here.
PANDA_SHOULDER_Z_M = 0.333

#: Fraction of maximum reach beyond which manipulability is poor enough that a
#: cartesian impedance controller commanding radial error produces large
#: null-space joint motion against only 5 N.m/rad of nullspace stiffness.
REACH_WARN_FRACTION = 0.88


def _env_float(name: str, default: float, bounds: tuple[float, float]) -> float:
    """Read ``name`` as a float, clamped into ``bounds``.

    A malformed value is ignored (falls back to ``default``) rather than raising,
    because these are read inside a Ray actor where a traceback at import time is
    hard to attribute to its cause.
    """
    raw = os.environ.get(name)
    value = default
    if raw is not None and raw.strip():
        try:
            value = float(raw)
        except ValueError:
            value = default
    low, high = bounds
    return float(min(max(value, low), high))


def env_float(name: str, default: float, bounds: tuple[float, float]) -> float:
    """Public alias for :func:`_env_float`.

    Other modules in ``b/x`` should read their environment overrides through this
    so they inherit the "clamp, and tolerate garbage" behaviour rather than
    raising from inside a Ray actor constructor.
    """
    return _env_float(name, default, bounds)


def force_ceiling_n() -> float:
    """Per-axis commanded spring-force ceiling, newtons."""
    return _env_float(
        "RLINF_CUBE_FORCE_CEILING_N", FORCE_CEILING_N_DEFAULT, FORCE_CEILING_N_RANGE
    )


def force_norm_ceiling_n() -> float:
    """Worst-case (three axes saturated) commanded spring-force norm, newtons."""
    return _env_float(
        "RLINF_CUBE_FORCE_NORM_CEILING_N",
        FORCE_NORM_CEILING_N_DEFAULT,
        FORCE_NORM_CEILING_N_RANGE,
    )


def torque_ceiling_nm() -> float:
    """Per-axis commanded torque ceiling, newton-metres."""
    return _env_float(
        "RLINF_CUBE_TORQUE_CEILING_NM",
        TORQUE_CEILING_NM_DEFAULT,
        TORQUE_CEILING_NM_RANGE,
    )


def torque_norm_ceiling_nm() -> float:
    """Worst-case commanded torque norm, newton-metres."""
    return _env_float(
        "RLINF_CUBE_TORQUE_NORM_CEILING_NM",
        TORQUE_NORM_CEILING_NM_DEFAULT,
        TORQUE_NORM_CEILING_NM_RANGE,
    )


def interp_speed_m_s() -> float:
    """Translational speed cap for interpolated (non-``step``) moves, m/s."""
    return _env_float(
        "RLINF_CUBE_INTERP_SPEED", INTERP_SPEED_M_S_DEFAULT, INTERP_SPEED_M_S_RANGE
    )


def step_speed_m_s() -> float:
    """Translational speed cap for the policy ``step()`` path, m/s."""
    return _env_float(
        "RLINF_CUBE_STEP_SPEED", STEP_SPEED_M_S_DEFAULT, STEP_SPEED_M_S_RANGE
    )


def max_action_scale_xyz(step_frequency: float) -> float:
    """Largest ``action_scale[0]`` that respects :func:`step_speed_m_s`, metres."""
    hz = float(step_frequency) if step_frequency else 10.0
    return float(step_speed_m_s() / max(hz, 1e-6))


def step_rot_speed_rad_s() -> float:
    """Rotational speed cap for the policy ``step()`` path, rad/s (LOG-034)."""
    return _env_float(
        "RLINF_CUBE_STEP_ROT_SPEED",
        STEP_ROT_SPEED_RAD_S_DEFAULT,
        STEP_ROT_SPEED_RAD_S_RANGE,
    )


def max_action_scale_rot(step_frequency: float) -> float:
    """Largest per-axis ``action_scale[1]`` that respects :func:`step_rot_speed_rad_s`, rad."""
    hz = float(step_frequency) if step_frequency else 10.0
    return float(step_rot_speed_rad_s() / max(hz, 1e-6))


def guard_max_dq_rad_s() -> float:
    """Joint-speed norm at which the motion guard aborts, rad/s (LOG-037)."""
    return _env_float(
        "RLINF_CUBE_GUARD_MAX_DQ",
        GUARD_MAX_DQ_RAD_S_DEFAULT,
        GUARD_MAX_DQ_RAD_S_RANGE,
    )


def joint_vel_demand_fraction() -> float:
    """Fraction of Panda's joint velocity limits one step may demand (LOG-037)."""
    return _env_float(
        "RLINF_CUBE_DQ_DEMAND_FRAC",
        JOINT_VEL_DEMAND_FRACTION_DEFAULT,
        JOINT_VEL_DEMAND_FRACTION_RANGE,
    )


def sigma_min_warn() -> float:
    """Jacobian ``sigma_min`` below which the joint configuration is flagged (LOG-040)."""
    return _env_float(
        "RLINF_CUBE_SIGMA_MIN_WARN",
        SIGMA_MIN_WARN_DEFAULT,
        SIGMA_MIN_WARN_RANGE,
    )


def guard_recovery_budget() -> int:
    """How many guard trips one env may auto-recover from; 0 disables (LOG-040)."""
    return int(
        _env_float(
            "RLINF_CUBE_GUARD_RECOVERY_BUDGET",
            float(GUARD_RECOVERY_BUDGET_DEFAULT),
            GUARD_RECOVERY_BUDGET_RANGE,
        )
    )


def jacobian_conditioning(jacobian) -> Optional[dict]:
    """Conditioning of a ``(6, 7)`` Jacobian: what a Cartesian command costs in joints.

    One SVD, shared by the offline diagnostic (``diag_franky_jacobian.py``) and
    the online self-check on the reset path, so the number an operator reads in
    a warning is computed the same way as the number they get when they go and
    measure it. Keeping two copies of this is how the two drift apart.

    ``sigma_min`` is the quantity of interest, not ``cond``: ``1 / sigma_min`` is
    the worst-case joint speed per unit of commanded twist, so it converts
    directly into "how much will the joint-demand clamp shrink my steps here".
    ``cond`` and ``manipulability`` come along because they are what the
    literature quotes and what the LOG entries recorded.

    Args:
        jacobian: zero/base-frame Jacobian, ``(6, 7)`` in normal use. Row subsets
            (``J[:3]``, ``J[3:]``) are accepted so the diagnostic can report the
            translation and rotation blocks separately -- this task caps the two
            independently, and a configuration can be well conditioned for one
            and not the other.

    Returns:
        ``{"sigma", "sigma_min", "sigma_max", "cond", "manipulability"}``, or
        ``None`` when there is no usable Jacobian (wrong shape, or the all-zero
        dataclass default that means "no data yet" -- same convention as
        :func:`joint_demand_scale`). ``cond`` is ``inf`` at an exact singularity.
    """
    J = np.asarray(jacobian, dtype=np.float64)
    if J.ndim != 2 or J.shape[1] != 7 or not np.any(np.abs(J) > 0.0):
        return None
    sigma = np.linalg.svd(J, compute_uv=False)
    s_min = float(sigma[-1])
    s_max = float(sigma[0])
    return {
        "sigma": sigma,
        "sigma_min": s_min,
        "sigma_max": s_max,
        "cond": float("inf") if s_min <= 1e-12 else s_max / s_min,
        "manipulability": float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0))),
    }


def joint_demand_scale(
    jacobian, delta_xyz, delta_rotvec, dt_s: float, joint_vel_limits
) -> tuple[float, float, int]:
    """How much to shrink one Cartesian step so its joint demand stays in budget.

    This is the T19 fix in one function: it converts the step the caller wants
    into the joint velocity that step *demands* at the arm's present
    configuration, and reports the factor that brings the worst joint back inside
    ``joint_vel_limits * joint_vel_demand_fraction()``.

    Why the demand and not the measurement: the arm is torque-controlled, so an
    impossible demand does not show up as a fast joint, it shows up as a pose
    error that grows until the lag gate fires (LOG-036). By the time ``|dq|`` is
    measurable the command has already been accepted. The Jacobian is the only
    thing that knows, *before* commanding, that this particular 5 cm/s is worth
    9.64 rad/s on joint 4 (LOG-037).

    Scaling is exact rather than iterative: ``dq_req = J^+ v`` is linear in ``v``,
    and ``v`` is linear in the step, so one division gives the factor. Shrinking
    (rather than refusing) is deliberate -- near an ill-conditioned pose the arm
    slows down and the episode continues, instead of tripping the guard and
    taking the whole training run with it.

    Args:
        jacobian: ``(6, 7)`` zero/base-frame Jacobian at the CURRENT pose.
        delta_xyz: commanded translation for this step, metres, base frame.
        delta_rotvec: commanded rotation for this step as a rotation vector,
            radians, base frame.
        dt_s: control period, seconds.
        joint_vel_limits: per-joint speed limits, rad/s (7).

    Returns:
        ``(scale, worst_ratio, worst_joint)``. ``scale`` is 1.0 when nothing is
        needed; ``worst_ratio`` is the worst per-joint demand as a multiple of its
        budget (>1 means clamping happened); ``worst_joint`` is 1-based, or 0 when
        the check could not run (no Jacobian data, degenerate dt).
    """
    J = np.asarray(jacobian, dtype=np.float64)
    if J.shape != (6, 7) or not np.any(np.abs(J) > 0.0):
        # A zeroed Jacobian is the dataclass default, i.e. "no data" (dummy env,
        # or a state that predates the first controller read). Silently skipping
        # is right: this is a backstop, and a dummy env has no joints to protect.
        return 1.0, 0.0, 0
    dt = float(dt_s)
    if not dt > 0.0:
        return 1.0, 0.0, 0

    twist = np.concatenate(
        [
            np.asarray(delta_xyz, dtype=np.float64).reshape(3) / dt,
            np.asarray(delta_rotvec, dtype=np.float64).reshape(3) / dt,
        ]
    )
    if not np.any(np.abs(twist) > 0.0):
        return 1.0, 0.0, 0

    dq_req = np.linalg.pinv(J) @ twist
    budget = np.asarray(joint_vel_limits, dtype=np.float64).reshape(7) * (
        joint_vel_demand_fraction()
    )
    ratios = np.abs(dq_req) / np.maximum(budget, 1e-9)
    worst_idx = int(np.argmax(ratios))
    worst_ratio = float(ratios[worst_idx])
    if worst_ratio <= 1.0:
        return 1.0, worst_ratio, worst_idx + 1
    return 1.0 / worst_ratio, worst_ratio, worst_idx + 1


def cartesian_collision_thresholds() -> list[float]:
    """Cartesian collision thresholds derived from the commanded wrench ceilings.

    Six values shaped for the ``force_thresholds`` argument of franky's
    ``set_collision_behavior(torque_thresholds, force_thresholds)``: three forces
    (N) then three torques (N.m).

    Why this matters more than any Python guard: upstream leaves libfranka's own
    reflex at ``_FORCE_THRESHOLD = [100, 100, 100, 25, 25, 25]``, five times above
    the commanded force ceiling, so the **hardware** bound could never fire first
    and every actual bound was Python -- a 50 Hz GIL-bound watchdog thread inside a
    Ray actor, supervising a 1 kHz torque loop. At the excursion rate LOG-019
    reached, one watchdog period is ~5 mm and a 100 ms GIL stall is ~2.6 cm. The
    reflex is the only bound that does not depend on Python being scheduled.

    Joint torque thresholds are deliberately **not** lowered here: they guard
    against joint-level collision, and tightening them invites nuisance reflexes
    from the arm's own dynamics. Upstream's j5-j7 value (11 N.m) already sits below
    the commanded torque norm ceiling.
    """
    f = force_norm_ceiling_n()
    t = torque_norm_ceiling_nm()
    return [f, f, f, t, t, t]


def interp_speed_rad_s() -> float:
    """Rotational speed cap for interpolated moves, rad/s."""
    return _env_float(
        "RLINF_CUBE_INTERP_SPEED_RAD",
        INTERP_SPEED_RAD_S_DEFAULT,
        INTERP_SPEED_RAD_S_RANGE,
    )


def guard_margin_m() -> float:
    """Allowed excursion of the measured TCP outside ``ee_pose_limit``, m."""
    return _env_float(
        "RLINF_CUBE_GUARD_MARGIN", GUARD_MARGIN_M_DEFAULT, GUARD_MARGIN_M_RANGE
    )


def guard_floor_margin_m() -> float:
    """Allowed excursion below the box floor, m. Tighter: the table is there."""
    return _env_float(
        "RLINF_CUBE_GUARD_FLOOR_MARGIN",
        GUARD_FLOOR_MARGIN_M_DEFAULT,
        GUARD_FLOOR_MARGIN_M_RANGE,
    )


def guard_max_lag_m() -> float:
    """Allowed |measured - commanded| before the guard calls it "not tracking", m."""
    return _env_float(
        "RLINF_CUBE_GUARD_MAX_LAG", GUARD_MAX_LAG_M_DEFAULT, GUARD_MAX_LAG_M_RANGE
    )


def clamp_stiffness(k_trans: float, k_rot: float) -> tuple[float, float]:
    """Clamp requested impedance stiffness into the hard ranges."""
    kt = float(
        min(max(float(k_trans), STIFFNESS_TRANS_RANGE[0]), STIFFNESS_TRANS_RANGE[1])
    )
    kr = float(min(max(float(k_rot), STIFFNESS_ROT_RANGE[0]), STIFFNESS_ROT_RANGE[1]))
    return kt, kr


def error_clips_for_stiffness(
    k_trans: float,
    k_rot: float,
    *,
    force_n: Optional[float] = None,
    torque_nm: Optional[float] = None,
) -> tuple[float, float]:
    """Derive ``(translational_error_clip, rotational_error_clip)`` from stiffness.

    The clip is *not* an independent tuning knob: ``stiffness * clip`` is the
    commanded spring wrench, and that is what must stay bounded when stiffness
    changes. This is the fix for LOG-019 -- with it, PegInsertion's ``K_t = 2000``
    yields a small clip and the ceiling stays put instead of silently becoming
    100 N.

    Both a per-axis and a worst-case-norm ceiling are honoured, because franky
    clips **elementwise**: three saturated axes command ``K * clip * sqrt(3)``.
    Whichever ceiling binds first wins.

    Use :func:`clip_shortfall` to find out whether the returned clip actually
    achieves the requested ceiling -- at low stiffness the clip range clamps and
    the realised ceiling is lower, which stalls the arm.

    Args:
        k_trans: translational stiffness, N/m.
        k_rot: rotational stiffness, N.m/rad.
        force_n: per-axis force ceiling override, N.
        torque_nm: per-axis torque ceiling override, N.m.

    Returns:
        Clips in metres and radians, each clamped into its hard range.
    """
    root3 = float(np.sqrt(3.0))
    f_axis = force_ceiling_n() if force_n is None else float(force_n)
    t_axis = torque_ceiling_nm() if torque_nm is None else float(torque_nm)
    # Whichever of "per axis" and "all three axes at once" binds first.
    f_axis = min(f_axis, force_norm_ceiling_n() / root3)
    t_axis = min(t_axis, torque_norm_ceiling_nm() / root3)

    kt, kr = clamp_stiffness(k_trans, k_rot)
    trans_clip = f_axis / kt if kt > 0 else ERROR_CLIP_M_RANGE[1]
    rot_clip = t_axis / kr if kr > 0 else ERROR_CLIP_RAD_RANGE[1]

    trans_clip = min(max(trans_clip, ERROR_CLIP_M_RANGE[0]), ERROR_CLIP_M_RANGE[1])
    rot_clip = min(max(rot_clip, ERROR_CLIP_RAD_RANGE[0]), ERROR_CLIP_RAD_RANGE[1])
    return float(trans_clip), float(rot_clip)


def clip_shortfall(k_trans: float, k_rot: float) -> list[str]:
    """Report where the clip range clamped and the ceiling was NOT achieved.

    At any ``K_t`` below ``per_axis_ceiling / ERROR_CLIP_M_RANGE[1]`` the clip
    saturates and the realised force ceiling is ``K_t * 0.05``, which at the
    bottom of :data:`STIFFNESS_TRANS_RANGE` is 2.5 N -- not enough to overcome
    this arm's stiction, so it will simply not move. That is a failed session
    presented as a normal one, so say it out loud.

    Returns:
        Human-readable strings, empty when both ceilings were achieved.
    """
    root3 = float(np.sqrt(3.0))
    kt, kr = clamp_stiffness(k_trans, k_rot)
    trans_clip, rot_clip = error_clips_for_stiffness(kt, kr)
    f_req, f_norm = force_ceiling_n(), force_norm_ceiling_n()
    t_req, t_norm = torque_ceiling_nm(), torque_norm_ceiling_nm()
    want_f = min(f_req, f_norm / root3)
    want_t = min(t_req, t_norm / root3)
    got_f = kt * trans_clip
    got_t = kr * rot_clip
    out: list[str] = []
    # This must be checked BEFORE the stiffness-dependent comparisons below,
    # independent of K_t/K_r: comparing got_f against the already-min()-ed want_f
    # reports "achieved" even when --force-ceiling alone cannot reach what was
    # asked, because the norm ceiling silently capped it first (round-2 audit
    # finding 6: --force-ceiling 30 or 35 both yield only 23.1 N/axis with the
    # default 40 N norm ceiling, and nothing said so).
    if f_req > f_norm / root3 * 1.05:
        out.append(
            f"per-axis force ceiling {f_req:.1f}N is capped by the norm ceiling to "
            f"{f_norm / root3:.1f}N/axis; raise RLINF_CUBE_FORCE_NORM_CEILING_N "
            "(or --force-norm-ceiling) too"
        )
    if t_req > t_norm / root3 * 1.05:
        out.append(
            f"per-axis torque ceiling {t_req:.2f}Nm is capped by the norm ceiling "
            f"to {t_norm / root3:.2f}Nm/axis; raise RLINF_CUBE_TORQUE_NORM_CEILING_NM "
            "(or --torque-norm-ceiling) too"
        )
    if got_f < want_f * 0.95:
        out.append(
            f"translational ceiling not achieved: wanted {want_f:.1f}N/axis, "
            f"clip clamped at {trans_clip:.4f}m so K_t={kt:.0f} only delivers "
            f"{got_f:.1f}N/axis (the arm may not overcome stiction; raise K_t)"
        )
    if got_f > want_f * 1.05:
        out.append(
            f"translational ceiling exceeded: {got_f:.1f}N/axis > {want_f:.1f}N/axis"
        )
    if got_t < want_t * 0.95:
        out.append(
            f"rotational ceiling not achieved: wanted {want_t:.2f}Nm/axis, got "
            f"{got_t:.2f}Nm/axis"
        )
    return out


def cube_width_m() -> float:
    """Calibrated width of the grasped object, metres."""
    return env_float("FRANKA_CUBE_WIDTH_M", CUBE_WIDTH_M_DEFAULT, CUBE_WIDTH_RANGE_M)


def hold_tolerance_m() -> float:
    """Half-width of the "holding the cube" width window, metres."""
    return env_float("FRANKA_HOLD_TOL_M", HOLD_TOL_M_DEFAULT, HOLD_TOL_RANGE_M)


def holding_from_width(width) -> Optional[bool]:
    """"Is the calibrated cube in the gripper", decided from measured width alone.

    Deliberately does **not** consult libfranka's ``is_grasped``: with any usable
    ``epsilon_inner``/``epsilon_outer``, that flag cannot distinguish a held cube
    from an empty hand closed on air (an empty hand closing to ~0 m satisfies a
    window like ``[-0.04, 0.06]``), and it can also stay latched True from a PRIOR
    grasp -- e.g. across a pre-Ray probe that runs before this process ever
    commands the gripper. Round-2 audit finding 5: the pre-Ray gate in
    ``tcp_probe.py`` was still trusting it while the controller-side gripper
    (``franka_libfranka_gripper.py``) had already moved to this width-only rule.

    Args:
        width: measured gripper width, metres, or ``None`` if unavailable.

    Returns:
        ``True``/``False`` if a width was given, ``None`` if it was not (caller
        should treat that as "unknown", not as "not holding").
    """
    if width is None:
        return None
    return bool(abs(float(width) - cube_width_m()) <= hold_tolerance_m())


def quat_angle_rad(quat_a: Sequence[float], quat_b: Sequence[float]) -> float:
    """Shortest-arc angle between two ``xyzw`` quaternions, radians."""
    a = np.asarray(quat_a, dtype=np.float64).reshape(-1)
    b = np.asarray(quat_b, dtype=np.float64).reshape(-1)
    if a.size != 4 or b.size != 4:
        return 0.0
    na = np.linalg.norm(a)
    nb = np.linalg.norm(b)
    if na <= 0 or nb <= 0:
        return 0.0
    dot = abs(float(np.dot(a / na, b / nb)))
    return float(2.0 * np.arccos(min(1.0, dot)))


def orientation_fence_rad(
    roll_half_rad: float,
    pitch_half_rad: float,
    yaw_half_rad: float,
) -> float:
    """Fence radius for |measured orientation - target orientation|, radians.

    Rotation needs a fence too -- a rotational instability whips the wrist, and
    the commanded torque ceiling sits near the j5-j7 joint limit. But it must NOT
    be fenced by comparing euler angles to ``ee_pose_limit[3:]``: this task's
    target roll is about -3.116 rad, a hair from -pi, so a live roll reading of
    +3.13 (physically 0.03 rad away) would look wildly out of bounds and the guard
    would abort a perfectly good motion. Wrap-around bugs in a safety guard give
    you either nuisance aborts or false confidence, so the fence is on the
    **shortest-arc angle** (:func:`quat_angle_rad`), which cannot wrap.

    The three half-widths are taken separately because they are not equal:
    ``PegInsertionConfig.__post_init__`` builds ``ee_pose_limit[3:]`` as
    ``target +/- 0.01`` on roll and pitch but ``target +/- clip_rz_range`` on yaw.
    An earlier version of this function assumed all three were ``clip_rz_range``
    and produced a fence 1.5x looser than its own reasoning implied.

    Returns:
        Allowed angle between measured and target orientation, radians.
    """
    slack = _env_float(
        "RLINF_CUBE_GUARD_ORIENT_SLACK",
        GUARD_ORIENT_SLACK_RAD_DEFAULT,
        GUARD_ORIENT_SLACK_RAD_RANGE,
    )
    worst = float(
        np.linalg.norm(
            [abs(float(roll_half_rad)), abs(float(pitch_half_rad)), abs(float(yaw_half_rad))]
        )
    )
    return float(worst + slack)


def interp_duration_s(
    from_pose: Sequence[float],
    to_pose: Sequence[float],
    *,
    requested: Optional[float] = None,
    speed_m_s: Optional[float] = None,
    speed_rad_s: Optional[float] = None,
) -> float:
    """Duration for an interpolated move so neither speed cap is exceeded.

    ``_interpolate_move(pose, timeout=t)`` emits ``t * step_frequency`` waypoints
    over ``t`` seconds, so ``t`` *is* the inverse of the commanded speed. Upstream
    hardcodes ``t`` (1.0 or 1.5 s) regardless of displacement, which is how a
    10 cm lift became a 10 cm/s command. Deriving ``t`` from the displacement
    makes the *speed* the constant instead.

    A ``requested`` duration is honoured only if it is slower than the caps; it is
    never allowed to make the move faster.
    """
    v_max = interp_speed_m_s() if speed_m_s is None else float(speed_m_s)
    w_max = interp_speed_rad_s() if speed_rad_s is None else float(speed_rad_s)

    a = np.asarray(from_pose, dtype=np.float64).reshape(-1)
    b = np.asarray(to_pose, dtype=np.float64).reshape(-1)
    dist = float(np.linalg.norm(b[:3] - a[:3]))
    need = dist / v_max if v_max > 0 else INTERP_DURATION_S_RANGE[1]

    if a.size >= 7 and b.size >= 7:
        ang = quat_angle_rad(a[3:7], b[3:7])
        if w_max > 0:
            need = max(need, ang / w_max)

    if requested is not None:
        need = max(need, float(requested))
    return float(min(max(need, INTERP_DURATION_S_RANGE[0]), INTERP_DURATION_S_RANGE[1]))


def reach_radius_m(xyz: Sequence[float]) -> float:
    """Distance from the shoulder to ``xyz``, i.e. what "reach" actually measures."""
    p = np.asarray(xyz, dtype=np.float64).reshape(-1)[:3]
    return float(
        np.linalg.norm([p[0], p[1], p[2] - PANDA_SHOULDER_Z_M])
    )


def reach_report(lo: Sequence[float], lower_is_box: bool = True) -> str:
    """One-line reach summary for a single point, for logs."""
    r = reach_radius_m(lo)
    return (
        f"r={r:.3f}m ({r / PANDA_MAX_REACH_M:.0%} of {PANDA_MAX_REACH_M}m reach)"
        + ("" if r <= PANDA_MAX_REACH_M * REACH_WARN_FRACTION else "  NEAR-SINGULAR")
    )


def worst_reach_corner(lo: Sequence[float], hi: Sequence[float]) -> tuple[float, list[float]]:
    """Largest shoulder-relative radius over the 8 corners of a box.

    Near the reach boundary the Jacobian is ill-conditioned radially, so a
    cartesian impedance controller commanding radial error there produces large
    null-space joint motion against only 5 N.m/rad of nullspace stiffness, which
    tends to end in a velocity-limit reflex mid-motion rather than a clean abort.
    Worth reporting, not worth refusing on -- the outer corners of a *fence* are
    places the arm is never supposed to reach.
    """
    lo_a = np.asarray(lo, dtype=np.float64).reshape(-1)[:3]
    hi_a = np.asarray(hi, dtype=np.float64).reshape(-1)[:3]
    best_r = -1.0
    best: list[float] = []
    for ix in (lo_a[0], hi_a[0]):
        for iy in (lo_a[1], hi_a[1]):
            for iz in (lo_a[2], hi_a[2]):
                r = reach_radius_m([ix, iy, iz])
                if r > best_r:
                    best_r = r
                    best = [float(ix), float(iy), float(iz)]
    return best_r, best


def describe_authority(k_trans: float, k_rot: float) -> str:
    """One-line summary for logs: the *products*, per-axis AND worst-case norm.

    Deliberately reports both numbers, and says the damping term is excluded --
    the first version of this function printed a single "= 25.0N" that was a
    per-axis spring bound presented as a total force ceiling.
    """
    root3 = float(np.sqrt(3.0))
    trans_clip, rot_clip = error_clips_for_stiffness(k_trans, k_rot)
    kt, kr = clamp_stiffness(k_trans, k_rot)
    f_axis = kt * trans_clip
    t_axis = kr * rot_clip
    return (
        f"spring force <= {f_axis:.1f}N/axis ({f_axis * root3:.1f}N worst-case 3-axis) "
        f"[K_t={kt:.0f} x clip={trans_clip:.4f}m]; "
        f"torque <= {t_axis:.2f}Nm/axis ({t_axis * root3:.2f}Nm worst-case) "
        f"[K_r={kr:.0f} x clip={rot_clip:.4f}rad]; "
        f"damping term NOT clipped (opposes motion only); "
        f"interp<={interp_speed_m_s() * 100:.1f}cm/s, <={MAX_INTERP_DISTANCE_M:.2f}m, "
        f"<={MAX_INTERP_ANGLE_RAD:.2f}rad; "
        f"guard margin={guard_margin_m():.3f}m floor={guard_floor_margin_m():.3f}m "
        f"lag={guard_max_lag_m():.3f}m dq<={guard_max_dq_rad_s():.2f}rad/s; "
        f"joint demand <= {joint_vel_demand_fraction():.2f} of joint vel limits"
    )
