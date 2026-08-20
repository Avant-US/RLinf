"""Extended FrankyController with FrankaEnv-compatible API (local extension).

Beyond adapting upstream ``FrankyController`` to the ``FrankaEnv`` call surface
(``move_arm`` / ``reconfigure_compliance_params`` / ``move_gripper`` / native
Franka Hand), this class owns the safety properties that upstream does not.

**Bounded commanded wrench.** ``translational_error_clip`` is derived from
``translational_stiffness`` so that their *product* -- the commanded spring force
-- is what stays bounded, per-axis and in norm. See
:mod:`franky_ext.motion_limits`. Before this, raising stiffness 500 -> 2000 via
PegInsertion's ``compliance_param`` silently raised the per-axis ceiling
25 N -> 100 N (LOG-019). Note what this does *not* bound: franky's damping term
is unclipped, and its magnitude grows as ``sqrt(K)``, so deriving the clip does
not make ``K_t = 2000`` equivalent to ``K_t = 500``.

**Soft joint-limit repulsion.** franky's impedance controller has a repulsive
torque near the joint limits, but it is inactive unless *both* limit vectors are
supplied. Upstream builds the tracker without them, so the only joint protection
was the hardware reflex. They are passed now.

**A motion guard that runs in this process.** The measured TCP is fenced against a
geometric box, and the fence is checked both when a waypoint is commanded and
continuously by a watchdog thread. It lives here rather than in the env because
this is the process that holds ``self._robot``: no Ray round-trip, so the check
costs nothing and can brake immediately.

  *Why "in this process" is load-bearing, not a nicety:* a ``RuntimeError`` raised
  in a Worker method does **not** propagate to the caller as an exception.
  ``WorkerGroupFuncResult._wait_for_results`` prints it, sets ``Cluster._run_failed``
  and signals the main thread, whose handler calls ``ray.kill(actor,
  no_restart=True)`` on every actor and exits. So nothing in this class -- not
  ``cleanup()``, not a ``finally`` -- runs after the guard fires. **The in-process
  brake inside ``_abort_motion`` is the entire software safety mechanism.** Any
  refactor that moves braking out of ``_abort_motion``, or that raises while the
  tracker is live without braking first, silently removes it.

  *And the corollary, which cost LOG-023:* the guard paths here therefore **do not
  raise at all**. They brake, latch a reason, and return; the env polls
  :meth:`guard_tripped` and raises on its own side, where ``finally`` still works.
  Raising instead put the process down before the poll could run, so the very
  mechanism added to preserve teardown was unreachable.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from rlinf.envs.realworld.franka import franky_controller as fc
from rlinf.envs.realworld.franka.franky_controller import FrankyController
from rlinf.scheduler import Cluster, NodePlacementStrategy

from franky_ext.motion_limits import (
    cartesian_collision_thresholds,
    clamp_stiffness,
    clip_shortfall,
    describe_authority,
    error_clips_for_stiffness,
    guard_floor_margin_m,
    guard_margin_m,
    guard_max_dq_rad_s,
    guard_max_lag_m,
    quat_angle_rad,
    reach_report,
    worst_reach_corner,
)
from franky_ext.tcp_probe import describe_robot_mode, require_motion_ready

#: Watchdog sampling period, seconds. The commanded-waypoint check only fires at
#: 10 Hz while waypoints are flowing and at 0 Hz otherwise -- and the unguarded
#: gaps in a real reset are long: ``time.sleep(1.0)`` after construction,
#: ``time.sleep(0.6)`` inside a gripper action, the whole settle after the final
#: waypoint, and every idle period between episodes. The tracker keeps commanding
#: torque at 1 kHz throughout all of them.
_WATCHDOG_PERIOD_S = 0.02

#: Budget for the braking dwell, seconds. Used as a *deadline* on a poll of joint
#: speed, not as a fixed sleep -- a fixed sleep both wastes time when the arm has
#: already stopped and, in the overshoot case, inserted a quarter second of travel
#: before the step that actually decelerated.
_BRAKE_DWELL_S = 0.25

#: Joint-speed norm below which the arm counts as stopped, rad/s.
_BRAKE_SETTLED_RAD_S = 0.02

#: Soft joint-limit repulsion, active only when both limit vectors are given.
_JOINT_LIMIT_ACTIVATION_RAD = 0.10
_JOINT_LIMIT_STIFFNESS = 4.0
_JOINT_LIMIT_DAMPING = 1.0
_JOINT_LIMIT_MAX_TORQUE = 5.0


class FrankyControllerExtended(FrankyController):
    """Adds move_arm, compliance reconfig, move_gripper, Franka Hand gripper."""

    def __init__(
        self,
        robot_ip: str,
        gripper_type: str = "franka",
        gripper_connection: Optional[str] = None,
    ):
        self._compliance_trans_k: float = fc._CART_TRANS_STIFFNESS
        self._compliance_rot_k: float = fc._CART_ROT_STIFFNESS
        # franky's own gains_time_constant. Deliberately NOT derived from
        # charger's ``compliance_param.translational_damping``: that key is the
        # ROS cartesian_impedance_controller's damping coefficient (N.s/m),
        # while this is a gain-ramp filter time constant (s). Mapping one onto
        # the other is the same class of error already retracted twice for
        # ``translational_clip_*`` (LOG-016).
        self._compliance_tc: float = fc._CART_GAINS_TC
        # Motion guard, installed by the env once the safety box is known.
        self._guard_min_xyz: Optional[np.ndarray] = None
        self._guard_max_xyz: Optional[np.ndarray] = None
        self._guard_z_ceiling: Optional[float] = None
        self._guard_max_lag: float = guard_max_lag_m()
        self._guard_max_dq: float = guard_max_dq_rad_s()
        self._guard_target_quat: Optional[np.ndarray] = None
        self._guard_max_orient_err: Optional[float] = None
        self._guard_enabled: bool = False
        self._guard_trip_reason: Optional[str] = None
        self._watchdog: Optional[threading.Thread] = None
        self._watchdog_stop: Optional[threading.Event] = None
        # Serialises trip handling between the main actor thread and the watchdog.
        # The reason can only be latched *after* braking (it records the distance
        # travelled while braking), so without this both threads can see
        # ``_guard_trip_reason is None``, both brake, and the two ``stop()`` /
        # ``set_target`` sequences interleave -- for a fence trip one of them can
        # re-point the target at a pose the other is stopping away from.
        self._guard_trip_lock = threading.Lock()
        super().__init__(
            robot_ip=robot_ip,
            gripper_type=gripper_type,
            gripper_connection=gripper_connection,
        )
        self._tighten_collision_behavior()

    def _tighten_collision_behavior(self) -> None:
        """Bring libfranka's own 1 kHz reflex down to the commanded ceilings.

        Upstream leaves ``_FORCE_THRESHOLD = [100, 100, 100, 25, 25, 25]``, five
        times above the commanded force ceiling, so the **hardware** bound could
        never fire before any software one. That left every actual bound in Python:
        the per-waypoint check, and a 50 Hz daemon thread inside a Ray actor
        supervising a 1 kHz torque loop. At the excursion rate LOG-019 reached, one
        watchdog period is ~5 mm and a 100 ms GIL stall is ~2.6 cm. This reflex is
        the only bound that does not depend on Python being scheduled.

        Joint torque thresholds are left at upstream's values on purpose: they guard
        joint-level collision, tightening them invites nuisance reflexes from the
        arm's own dynamics, and j5-j7 already sit at 11 N.m, below the commanded
        torque norm ceiling.
        """
        force = cartesian_collision_thresholds()
        try:
            self._robot.set_collision_behavior(fc._TORQUE_THRESHOLD, force)
            self._collision_force_thresholds = force
            self._logger.info(
                "collision behavior tightened: cartesian force/torque thresholds=%s "
                "(was %s), joint torque left at %s",
                [round(v, 2) for v in force],
                fc._FORCE_THRESHOLD,
                fc._TORQUE_THRESHOLD,
            )
        except Exception as exc:  # noqa: BLE001 - never block construction on this
            self._collision_force_thresholds = list(fc._FORCE_THRESHOLD)
            self._logger.error(
                "could not tighten collision behavior (%s: %s); libfranka's reflex "
                "stays at %s, i.e. far above the %.1fN commanded ceiling -- the only "
                "remaining bounds are the Python guard and watchdog",
                type(exc).__name__,
                exc,
                fc._FORCE_THRESHOLD,
                force[0],
            )

    @staticmethod
    def launch_controller(
        robot_ip: str,
        env_idx: int = 0,
        node_rank: int = 0,
        worker_rank: int = 0,
        gripper_type: str = "franka",
        gripper_connection: Optional[str] = None,
        node_group_label: Optional[str] = None,
    ):
        """Launch the controller actor, routed through ``node_group_label`` when known.

        LOG-034 (T17): ``NodePlacementStrategy(node_ranks=[node_rank])`` with no
        ``node_group_label`` resolves against ``cluster.get_node_group()`` --
        the cluster's *default* group (``env_configs: null``), NOT the YAML
        node_group (e.g. ``franky``) that actually carries
        ``env_configs.env_vars``. ``WorkerGroup._launch_worker`` reads env vars
        via ``self._cluster.get_node_group(placement.node_group_label)``
        (``worker_group.py``), so a controller placed in the default group is
        deaf to every env var the YAML aimed at it -- ``FRANKA_CUBE_WIDTH_M``,
        ``RLINF_SKIP_CAMERA``, etc. -- and silently runs on library defaults
        instead (measured: ``cube_width=0.046m`` instead of the YAML's
        ``0.0325m``, so ``holding_from_width`` disagreed with a cube that was, in
        fact, grasped).

        When ``node_group_label`` is given, ``node_rank`` must be resolvable as a
        member of that group's ``node_ranks`` -- the caller (see
        ``FrankySingleFrankaEnvMixin._setup_hardware``) only passes it when the
        controller is co-located with the env worker that already knows its own
        group. Any failure to resolve falls back to the old, env-var-blind
        placement rather than raising: a missing env var is a silent
        misconfiguration, but a broken controller launch stops the arm entirely.
        """
        placement_strategy = None
        if node_group_label is not None:
            try:
                node_group = Cluster().get_node_group(node_group_label)
                local_rank = list(node_group.node_ranks).index(node_rank)
                placement_strategy = NodePlacementStrategy(
                    node_ranks=[local_rank], node_group_label=node_group_label
                )
            except Exception as exc:  # noqa: BLE001 - fall back, do not abort the launch
                from rlinf.utils.logging import get_logger

                get_logger().warning(
                    "FrankyControllerExtended.launch_controller: could not resolve "
                    "node_rank=%s into node_group=%r (%s: %s); falling back to the "
                    "default node group, so env_configs.env_vars aimed at this "
                    "controller will NOT apply",
                    node_rank,
                    node_group_label,
                    type(exc).__name__,
                    exc,
                )
        if placement_strategy is None:
            placement_strategy = NodePlacementStrategy(node_ranks=[node_rank])
        return FrankyControllerExtended.create_group(
            robot_ip, gripper_type, gripper_connection
        ).launch(
            cluster=Cluster(),
            placement_strategy=placement_strategy,
            name=f"FrankyControllerExtended-{worker_rank}-{env_idx}",
        )

    def _build_gripper(self, gripper_type, gripper_connection, robot_ip):
        gt = (gripper_type or "franka").lower()
        if gt == "franka":
            from franky_ext.franka_libfranka_gripper import FrankaLibfrankaGripper

            return FrankaLibfrankaGripper(robot_ip=robot_ip)
        return super()._build_gripper(gripper_type, gripper_connection, robot_ip)

    # ------------------------------------------------------------------
    # Motion guard
    # ------------------------------------------------------------------

    def set_motion_guard(
        self,
        limit_min_xyz,
        limit_max_xyz,
        *,
        margin: Optional[float] = None,
        floor_margin: Optional[float] = None,
        max_lag: Optional[float] = None,
        extra_z_up: float = 0.0,
        z_ceiling: Optional[float] = None,
        target_quat=None,
        max_orient_err: Optional[float] = None,
    ) -> None:
        """Install a geometric fence on the *measured* TCP.

        Args:
            limit_min_xyz: ``ee_pose_limit_min[:3]`` (safety box lower corner).
            limit_max_xyz: ``ee_pose_limit_max[:3]`` (safety box upper corner).
            margin: how far outside the box the measured TCP may stray before
                aborting.
            floor_margin: same, on ``-z`` only. Kept much tighter than ``margin``
                because below the contact point there is a physical table, and
                "5 cm of slack" there means authorising a 5 cm press into it.
            max_lag: how far the measured TCP may lag the commanded target before
                the controller is declared "not tracking".
            extra_z_up: additional headroom on ``+z`` only. ``go_to_rest``
                legitimately lifts above the box top before descending to hover
                (upstream ``_interpolate_move`` does not clip to the box), so the
                fence must allow that one excursion and nothing more.
            z_ceiling: absolute ``+z`` limit, independent of the box. The last line
                of defence, and the only fence term that does not move when the
                caller tunes ``reset_z_lift_m``.
            target_quat: task target orientation as ``xyzw``. When given, the
                measured orientation is fenced by shortest-arc angle from it --
                deliberately not by comparing euler angles to ``ee_pose_limit[3:]``,
                because this task's roll sits a hair from -pi and wrap-around
                would abort good motions.
            max_orient_err: allowed angle from ``target_quat``, radians.
        """
        lo = np.asarray(limit_min_xyz, dtype=np.float64).reshape(-1)[:3].copy()
        hi = np.asarray(limit_max_xyz, dtype=np.float64).reshape(-1)[:3].copy()
        if lo.size != 3 or hi.size != 3:
            raise ValueError("motion guard needs 3-vectors for min/max xyz")
        if np.any(hi < lo):
            raise ValueError(f"motion guard box inverted: min={lo} max={hi}")

        m = guard_margin_m() if margin is None else float(margin)
        fm = guard_floor_margin_m() if floor_margin is None else float(floor_margin)
        self._guard_min_xyz = lo - m
        self._guard_min_xyz[2] = lo[2] - fm
        self._guard_max_xyz = hi + m
        self._guard_max_xyz[2] += max(0.0, float(extra_z_up))
        if z_ceiling is not None:
            self._guard_z_ceiling = float(z_ceiling)
            self._guard_max_xyz[2] = min(self._guard_max_xyz[2], self._guard_z_ceiling)
        self._guard_max_lag = guard_max_lag_m() if max_lag is None else float(max_lag)

        if target_quat is None or max_orient_err is None:
            self._guard_target_quat = None
            self._guard_max_orient_err = None
            self._logger.warning(
                "motion guard: no target orientation given; orientation NOT fenced"
            )
        else:
            q = np.asarray(target_quat, dtype=np.float64).reshape(-1)[:4]
            norm = float(np.linalg.norm(q)) if q.size == 4 else 0.0
            if norm > 0:
                self._guard_target_quat = q / norm
                self._guard_max_orient_err = float(max_orient_err)
            else:
                self._guard_target_quat = None
                self._guard_max_orient_err = None
                self._logger.warning(
                    "motion guard: bad target_quat %s; orientation NOT fenced",
                    target_quat,
                )

        self._guard_trip_reason = None
        self._guard_enabled = True
        _, corner_box = worst_reach_corner(lo, hi)
        _, corner_fence = worst_reach_corner(self._guard_min_xyz, self._guard_max_xyz)
        self._logger.info(
            "motion guard armed: xyz in [%s, %s] (box +/- %.3fm, floor -%.3fm, "
            "+z headroom %.3fm, ceiling %s), max_lag=%.3fm, max_dq=%.3frad/s, "
            "orient<=%s",
            np.round(self._guard_min_xyz, 4).tolist(),
            np.round(self._guard_max_xyz, 4).tolist(),
            m,
            fm,
            max(0.0, float(extra_z_up)),
            "none" if self._guard_z_ceiling is None else f"{self._guard_z_ceiling:.4f}",
            self._guard_max_lag,
            self._guard_max_dq,
            "off"
            if self._guard_max_orient_err is None
            else f"{self._guard_max_orient_err:.3f}rad",
        )
        self._logger.info(
            "reach: worst box corner %s %s; worst fence corner %s %s",
            np.round(corner_box, 4).tolist(),
            reach_report(corner_box),
            np.round(corner_fence, 4).tolist(),
            reach_report(corner_fence),
        )

    def clear_motion_guard(self) -> None:
        """Disarm the fence. No production caller; kept for deliberate manual use."""
        self._guard_enabled = False
        self._logger.warning("motion guard disarmed")

    def recover_from_guard_trip(self) -> dict:
        """Clear a latched trip so the caller may reset and continue. **Never raises.**

        LOG-040's T21. Before this existed the only ways out of a latch were to
        re-run :meth:`set_motion_guard` (which clears ``_guard_trip_reason`` as a
        side effect of installing a fence) or to restart the process -- and the
        env did neither, so one ``[watchdog:dq]`` trip in the third rollout epoch
        took the whole training job down with it.

        This is deliberately a *separate, named* method rather than a flag on
        ``set_motion_guard``, because "clear a safety latch" is not a thing that
        should ever happen as a side effect of installing a fence. Anything that
        clears a trip should be greppable.

        What it does **not** do is decide whether recovering is a good idea --
        that budget lives in the env
        (``FrankySingleFrankaEnvMixin._recover_from_trip``), which is the layer
        that knows how many times this has already happened this run. Here the
        job is only to establish that recovering is *physically* safe right now:

        1. The arm must actually be stopped. ``_abort_motion`` brakes before it
           latches, so a still-moving arm means the brake did not take -- exactly
           the case where clearing the latch would re-authorise motion on top of
           a runaway.
        2. No tracker may be live. The trip path tears it down
           (``_stop_cart_tracker_no_recover``); if one is somehow still there,
           stop it rather than letting the next command rebuild on top of it.
        3. ``recover_from_errors`` must succeed and leave the robot mode
           motion-ready. This is the programmatic equivalent of the "clear the
           fault in Desk" instruction the trip message prints -- deliberately
           skipped on the abort path so a human *had* to look, and re-enabled
           here only because the caller has now been asked to make that decision
           explicitly.

        The tracker and the watchdog are **not** restarted here. Both come back
        on the next commanded move via ``_ensure_cart_tracking_motion``, which is
        also the point at which the fence starts being enforced again -- so there
        is no window where motion is possible but unsupervised.

        Returns:
            A report dict. ``recovered`` is the only field a caller must check;
            ``refusal`` says why when it is ``False``, and ``previous_reason``
            preserves the trip text (the latch is gone by then).
        """
        report = {
            "recovered": False,
            "was_tripped": self._guard_trip_reason is not None,
            "previous_reason": self._guard_trip_reason,
            "refusal": None,
            "joint_speed": self._safe(self._joint_speed),
            "robot_mode": self._safe(lambda: str(self._robot.state.robot_mode)),
            "has_errors": self._safe(lambda: bool(self._robot.has_errors)),
            "guard_enabled": bool(self._guard_enabled),
        }
        if self._guard_trip_reason is None:
            report["recovered"] = True
            return report
        if not self._guard_enabled or self._guard_min_xyz is None:
            report["refusal"] = (
                "the motion guard is not armed, so there is no fence to resume "
                "under; re-arm it with set_motion_guard before recovering"
            )
            self._logger.error("guard recovery refused: %s", report["refusal"])
            return report

        # 1. Settled? Poll rather than sleep, same budget the brake itself uses.
        deadline = time.perf_counter() + _BRAKE_DWELL_S
        speed = self._joint_speed()
        while speed > _BRAKE_SETTLED_RAD_S and time.perf_counter() < deadline:
            time.sleep(0.005)
            speed = self._joint_speed()
        report["joint_speed"] = speed
        if not speed <= _BRAKE_SETTLED_RAD_S:
            report["refusal"] = (
                f"the arm is still moving after the brake (|dq|={speed:.4f} rad/s > "
                f"{_BRAKE_SETTLED_RAD_S:.4f}); refusing to clear a trip on a moving arm"
            )
            self._logger.error("guard recovery refused: %s", report["refusal"])
            return report

        # 2. No live tracker to rebuild on top of.
        if self._cart_tracker is not None:
            self._logger.warning(
                "guard recovery: a cartesian tracker was still present after the "
                "trip; stopping it before clearing the latch"
            )
            self._stop_cart_tracker_no_recover()

        # 3. Clear the fault and confirm the arm will accept motion again.
        try:
            self._robot.recover_from_errors()
        except Exception as exc:  # noqa: BLE001 - a refusal, not a crash
            report["refusal"] = (
                f"recover_from_errors failed ({type(exc).__name__}: {exc}); the "
                "fault needs clearing in Desk by hand"
            )
            self._logger.error("guard recovery refused: %s", report["refusal"])
            return report
        mode = str(self._safe(lambda: str(self._robot.state.robot_mode)))
        try:
            require_motion_ready(mode)
        except Exception as exc:  # noqa: BLE001 - a refusal, not a crash
            report["robot_mode"] = mode
            report["refusal"] = (
                f"robot mode is {describe_robot_mode(mode)} after recovery "
                f"({type(exc).__name__}: {exc}); it will not accept motion"
            )
            self._logger.error("guard recovery refused: %s", report["refusal"])
            return report
        report["robot_mode"] = mode
        report["has_errors"] = self._safe(lambda: bool(self._robot.has_errors))

        previous = self._guard_trip_reason
        with self._guard_trip_lock:
            self._guard_trip_reason = None
        report["recovered"] = True
        self._logger.warning(
            "motion guard trip CLEARED by explicit recovery: %s -- arm settled at "
            "|dq|=%.4f rad/s, mode=%s, has_errors=%s. The fence stays armed; the "
            "tracker and watchdog rebuild on the next commanded move.",
            previous,
            report["joint_speed"],
            mode,
            report["has_errors"],
        )
        return report

    @staticmethod
    def _safe(fn, default=None):
        """Evaluate ``fn`` for a diagnostic field, swallowing any failure.

        Used only by :meth:`motion_health`, whose whole job is to be readable from
        a teardown ``finally``. See there for why one raise is not acceptable.
        """
        try:
            return fn()
        except Exception:  # noqa: BLE001 - a diagnostic snapshot must not raise
            return default

    def motion_health(self) -> dict:
        """Read-only snapshot for callers on the other side of Ray.

        Reports what the fence actually *is*, not just that it is on -- otherwise
        the smoke script cannot confirm what got armed. Also re-checks the fence,
        so polling this covers gaps between commanded waypoints.

        **Nothing in here may raise.** The smoke script reads it from a ``finally``
        that runs *before* ``env.close()`` stops the impedance tracker, and an
        exception in a Worker method takes the driver down with every actor already
        killed -- so a raise here would skip the one call that stops the arm being
        driven, and would replace the real reason the run aborted with a diagnostic
        failure. Hence every field goes through :meth:`_safe`, and a failed fence
        evaluation is reported as ``guard_check_error`` rather than thrown.
        """
        guard_check_error = None
        try:
            self._check_motion_guard()
        except Exception as exc:  # noqa: BLE001 - see docstring
            guard_check_error = f"{type(exc).__name__}: {exc}"
            self._logger.error("motion_health: fence check failed: %s", guard_check_error)
        tracker = self._cart_tracker
        return {
            "robot_mode": self._safe(lambda: str(self._robot.state.robot_mode)),
            "has_errors": self._safe(lambda: bool(self._robot.has_errors)),
            "is_in_control": self._safe(lambda: bool(self._robot.is_in_control)),
            # NB: franky's tracker.is_running is literally robot.is_in_control,
            # i.e. "some motion is executing", not "this tracker is alive". It is
            # equivalent here only because the joint and cartesian trackers are
            # mutually exclusive.
            "robot_in_control": False
            if tracker is None
            else bool(self._safe(lambda: tracker.is_running, False)),
            "tracker_object_present": tracker is not None,
            "tcp_xyz": self._safe(
                lambda: np.asarray(
                    self._robot.state.O_T_EE.translation, dtype=np.float64
                ).tolist()
            ),
            "guard_enabled": bool(self._guard_enabled),
            "guard_min_xyz": None
            if self._guard_min_xyz is None
            else np.round(self._guard_min_xyz, 4).tolist(),
            "guard_max_xyz": None
            if self._guard_max_xyz is None
            else np.round(self._guard_max_xyz, 4).tolist(),
            "guard_max_orient_err": self._guard_max_orient_err,
            "guard_max_lag": self._guard_max_lag,
            "guard_max_dq": self._guard_max_dq,
            "joint_speed": self._safe(self._joint_speed),
            "guard_tripped": self._guard_trip_reason,
            "guard_check_error": guard_check_error,
            "watchdog_alive": bool(self._watchdog is not None and self._watchdog.is_alive()),
            "gripper_width": self._safe(lambda: float(self._gripper.position)),
            "authority": self._safe(
                lambda: describe_authority(
                    self._compliance_trans_k, self._compliance_rot_k
                )
            ),
        }

    def freeze_at_current(self) -> bool:
        """Brake by removing the position error. Returns whether it was verified.

        Setting the target to where the arm *is* makes the impedance decelerate it;
        stopping the motion outright would hand it to a controlled stop carrying
        whatever momentum it had. Order matters -- and so does the dwell in
        :meth:`_abort_motion` that gives this a chance to act.

        ``set_target`` writes into a wait-free triple buffer and returns
        unconditionally, so if the async control thread is already dead the write
        is a silent no-op. Report that rather than logging a brake that did not
        happen -- being fooled by exactly this is what LOG-017 was.
        """
        tracker = self._cart_tracker
        if tracker is None:
            self._logger.warning("freeze_at_current: no tracker; nothing to brake")
            return False
        live_running = bool(tracker.is_running)
        try:
            live = self._robot.state.O_T_EE
            xyz = np.asarray(live.translation, dtype=np.float64)
            quat = np.asarray(live.quaternion, dtype=np.float64)
            T = np.eye(4)
            T[:3, :3] = R.from_quat(quat / np.linalg.norm(quat)).as_matrix()
            T[:3, 3] = xyz
            tracker.set_target(self._franky.Affine(T))
            self._prev_cart_target_xyz = xyz
            self._prev_cart_target_quat = quat / np.linalg.norm(quat)
            if live_running:
                self._logger.warning(
                    "freeze_at_current: target <- measured %s (impedance decelerating)",
                    np.round(xyz, 4).tolist(),
                )
            else:
                self._logger.error(
                    "freeze_at_current: wrote target <- measured %s but the control "
                    "thread is NOT running, so the write has no effect and the arm "
                    "is not being braked by software",
                    np.round(xyz, 4).tolist(),
                )
            return live_running
        except Exception as exc:
            self._logger.error(
                "freeze_at_current failed: %s: %s", type(exc).__name__, exc
            )
            return False

    def _stop_cart_tracker_no_recover(self) -> str:
        """Stop the tracker without ``recover_from_errors``.

        Upstream ``_stop_cart_tracking_motion`` ends with
        ``self._robot.recover_from_errors()``, which on an abort path would *re-arm*
        the robot immediately after a runaway was detected. An abort should leave
        the robot faulted so a human has to clear it.
        """
        cause = "clean"
        tracker = self._cart_tracker
        if tracker is not None:
            try:
                tracker.stop()
            except Exception as exc:
                cause = f"{type(exc).__name__}: {exc}"
        self._cart_tracker = None
        self._prev_cart_target_xyz = None
        self._prev_cart_target_quat = None
        try:
            self._safe_join()
        except Exception as exc:  # noqa: BLE001 - teardown must not mask the cause
            cause = f"{cause}; join: {type(exc).__name__}: {exc}"
        return cause

    def _joint_speed(self) -> float:
        try:
            return float(
                np.linalg.norm(np.asarray(self._robot.state.dq, dtype=np.float64))
            )
        except Exception:  # noqa: BLE001 - diagnostic only
            return float("nan")

    def _brake(self, kind: str) -> tuple[bool, str]:
        """Decelerate the arm, choosing the right order for this failure mode.

        The two violations need **opposite** orders, and getting this wrong makes
        the brake weaker than doing nothing:

        ``fence`` / ``orient`` -- the arm has gone *past* where it was told to be.
            The impedance spring is therefore already pulling it back at up to the
            full clipped force. ``set_target(measured)`` would zero exactly that
            restoring term and leave only damping. So: ``stop()`` **first** --
            libfranka's controlled stop actively decelerates -- and do not throw the
            spring away on the way there.

        ``lag`` / ``dq`` -- the arm is *behind* a target that is running away from
            it. Here the target is the problem, so ``set_target(measured)`` first:
            it removes the runaway error before the motion is torn down. ``dq``
            shares this order because it is the same situation seen one step
            earlier -- the joints are fast *because* they are chasing a target
            they cannot reach (LOG-037), so removing that target is still the
            first thing that helps.

        The first version of this method used the ``lag`` order for both, plus a
        fixed 0.25 s dwell -- i.e. in the overshoot case it removed the restoring
        force and then waited a quarter second (6.5 cm at LOG-019's excursion rate)
        before the only step that actually decelerated.
        """
        v0 = self._joint_speed()
        if kind in ("lag", "dq"):
            braked = self.freeze_at_current()
            # Bounded poll rather than a fixed sleep: exit as soon as the arm is
            # actually slow, and never spend longer than the dwell budget.
            deadline = time.perf_counter() + _BRAKE_DWELL_S
            while time.perf_counter() < deadline:
                if self._joint_speed() < _BRAKE_SETTLED_RAD_S:
                    break
                time.sleep(0.005)
            cause = self._stop_cart_tracker_no_recover()
            order = "freeze-then-stop"
        else:
            cause = self._stop_cart_tracker_no_recover()
            braked = True  # robot.stop() is a controlled stop, not a release
            deadline = time.perf_counter() + _BRAKE_DWELL_S
            while time.perf_counter() < deadline:
                if self._joint_speed() < _BRAKE_SETTLED_RAD_S:
                    break
                time.sleep(0.005)
            order = "stop-first"
        self._logger.warning(
            "brake (%s, %s): |dq| %.4f -> %.4f rad/s, stop=%s",
            kind,
            order,
            v0,
            self._joint_speed(),
            cause,
        )
        return braked, cause

    def _abort_motion(self, kind: str, reason: str) -> None:
        """Brake, tear the tracker down, latch the reason. **Never raises.**

        Raising here is worse than useless. A ``RuntimeError`` from a Worker method
        is caught by ``WorkerGroupFuncResult``, which signals the driver, whose
        handler ``ray.kill``s every actor and exits -- so the env's teardown cannot
        run and ``motion_health`` cannot even be read afterwards. Worse, it made
        the ``guard_tripped()`` poll that exists for exactly this case
        *unreachable*: the poll runs after ``_move_action`` returns, and the raise
        happened inside it (LOG-023 finding 1).

        The brake is the part that matters, and it has already happened by the time
        this returns. Callers must then consult ``self._guard_trip_reason`` and
        refuse to command anything further; the env converts the latch into an
        ordinary Python exception in
        ``FrankySingleFrankaEnvMixin._raise_if_guard_tripped``.
        """
        with self._guard_trip_lock:
            if self._guard_trip_reason is not None:
                # The watchdog got there first and has already braked.
                return
            # Capture evidence BEFORE any teardown: has_errors read afterwards is
            # meaningless, because stopping the motion clears/recovers the fault.
            live = np.asarray(self._robot.state.O_T_EE.translation, dtype=np.float64)
            mode = str(self._robot.state.robot_mode)
            had_errors = bool(self._robot.has_errors)

            braked, cause = self._brake(kind)
            after = np.asarray(self._robot.state.O_T_EE.translation, dtype=np.float64)

            self._guard_trip_reason = (
                f"[{kind}] {reason}; tcp={np.round(live, 4).tolist()} -> "
                f"{np.round(after, 4).tolist()} after brake (braked={braked}, "
                f"stop={cause}, mode={mode})"
            )
        # Outside the lock: the watchdog may be blocked on it, and joining a thread
        # that is waiting for a lock this thread holds would deadlock until the
        # 1 s join timeout.
        self._stop_watchdog()
        self._logger.error(
            "motion guard abort [%s]: %s; tcp_at_trip=%s tcp_after_brake=%s "
            "(travelled %.4fm while braking) mode=%s has_errors_at_trip=%s "
            "braked=%s stop=%s -- arm braked and latched; this process will not "
            "command motion again, and the env-side poll will raise",
            kind,
            reason,
            np.round(live, 4).tolist(),
            np.round(after, 4).tolist(),
            float(np.linalg.norm(after - live)),
            mode,
            had_errors,
            braked,
            cause,
        )

    def _evaluate_guard(self) -> Optional[tuple[str, str]]:
        """Pure check. Returns ``(kind, description)`` or ``None`` if all clear.

        The ``kind`` decides how :meth:`_brake` decelerates, so it is part of the
        contract, not a label.
        """
        if not self._guard_enabled or self._guard_min_xyz is None:
            return None
        state = self._robot.state
        live = np.asarray(state.O_T_EE.translation, dtype=np.float64)
        if np.any(live < self._guard_min_xyz) or np.any(live > self._guard_max_xyz):
            axes = "xyz"
            worst = []
            for i in range(3):
                if live[i] < self._guard_min_xyz[i]:
                    worst.append(
                        f"{axes[i]}={live[i]:.4f} < {self._guard_min_xyz[i]:.4f}"
                    )
                elif live[i] > self._guard_max_xyz[i]:
                    worst.append(
                        f"{axes[i]}={live[i]:.4f} > {self._guard_max_xyz[i]:.4f}"
                    )
            return "fence", "measured TCP left the guard fence: " + ", ".join(worst)

        # Checked BEFORE lag, and that order is the whole point of this gate. A
        # joint runaway shows up as lag *second-hand*: the arm cannot follow, so
        # the position error grows, so the lag gate fires and reports "not
        # tracking" -- which is a symptom, not the cause. Both real trips
        # (LOG-034, LOG-036) were misfiled that way, and LOG-036 spent a rotation
        # ladder chasing a Cartesian explanation for a joint-space problem.
        # Checking joint speed first means the trip message names the mechanism.
        dq = np.asarray(state.dq, dtype=np.float64)
        dq_norm = float(np.linalg.norm(dq))
        if dq_norm > self._guard_max_dq:
            worst = int(np.argmax(np.abs(dq)))
            return (
                "dq",
                f"joint runaway: |dq|={dq_norm:.4f} rad/s > "
                f"{self._guard_max_dq:.4f} (worst j{worst + 1}={dq[worst]:+.4f}); "
                f"q={np.round(np.asarray(state.q, dtype=np.float64), 4).tolist()}",
            )

        cmd = self._prev_cart_target_xyz
        if cmd is not None:
            lag = float(np.linalg.norm(live - np.asarray(cmd, dtype=np.float64)))
            if lag > self._guard_max_lag:
                return (
                    "lag",
                    f"not tracking: |measured-commanded|={lag:.4f}m > "
                    f"{self._guard_max_lag:.4f}m (controller saturated, dead or "
                    "unstable)",
                )

        if self._guard_target_quat is not None and self._guard_max_orient_err is not None:
            live_quat = np.asarray(state.O_T_EE.quaternion, dtype=np.float64)
            ang = quat_angle_rad(live_quat, self._guard_target_quat)
            if ang > self._guard_max_orient_err:
                return (
                    "orient",
                    f"orientation {ang:.3f}rad from target > "
                    f"{self._guard_max_orient_err:.3f}rad (wrist is being whipped)",
                )
        return None

    def guard_tripped(self) -> Optional[str]:
        """Latched trip reason, or ``None``. **Does not raise.**

        The env side needs this because a ``RuntimeError`` raised in a Worker method
        never reaches the caller as an exception: ``WorkerGroupFuncResult`` catches
        it, prints it, signals the main thread and exits, and the signal handler
        ``ray.kill``s every actor. So an env-side ``try/finally`` cannot run for the
        very case it was written for. Polling this instead lets the *env* raise a
        normal Python exception, with its teardown intact -- while the in-process
        brake has already happened here, which is the part that matters.

        For that to hold, **no other method here may raise on a latched trip**
        either, or it fires first and the poll never happens. See
        :meth:`_check_motion_guard`.
        """
        return self._guard_trip_reason

    def _check_motion_guard(self) -> Optional[str]:
        """Evaluate the fence, braking on a NEW violation. Safe to call often.

        Returns the latched trip reason, or ``None`` while all is clear, so a caller
        that is about to command motion can refuse instead. It deliberately does
        **not** raise merely because a trip is already latched: the arm was braked
        when it tripped, and a raise from inside a Worker method kills every actor
        and exits the driver. That is how a latched trip used to take the whole
        process down on the very next ``get_state()`` -- including the one
        ``_close_env`` makes from a ``finally``, before ``env.close()`` had stopped
        the tracker (LOG-023 finding 1).
        """
        if self._guard_trip_reason is not None:
            return self._guard_trip_reason
        violation = self._evaluate_guard()
        if violation is not None:
            self._abort_motion(*violation)
        return self._guard_trip_reason

    def _check_requested_target(self, xyz: np.ndarray) -> None:
        """Refuse a *commanded* pose outside the fence, before the arm moves.

        The measured-TCP fence only objects after the arm has physically travelled
        outside the box. ``_CART_MAX_STEP_M`` clamps each call to 10 cm from the
        previous target, so a request 30 cm out is accepted and walked toward.
        Checking the request costs nothing and fails before any motion.
        """
        if not self._guard_enabled or self._guard_min_xyz is None:
            return
        p = np.asarray(xyz, dtype=np.float64).reshape(-1)[:3]
        if np.any(p < self._guard_min_xyz) or np.any(p > self._guard_max_xyz):
            raise RuntimeError(
                f"refusing commanded pose {np.round(p, 4).tolist()}: outside the "
                f"motion guard fence [{np.round(self._guard_min_xyz, 4).tolist()}, "
                f"{np.round(self._guard_max_xyz, 4).tolist()}]. The target geometry "
                "is wrong, or the arm is not where this task expects it."
            )

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def _watchdog_loop(self, stop_event: threading.Event) -> None:
        """Sample the fence continuously; brake once and latch, never raise.

        Raising from this thread would go nowhere, and tearing the tracker down
        from here would race the main thread's own ``stop()`` / ``join_motion()``.
        So the watchdog brakes and latches a reason. Every subsequent call into this
        actor then *refuses* rather than raising, and the env-side poll
        (:meth:`guard_tripped`) is what turns the latch into an exception.

        "Once" is enforced by ``_guard_trip_lock``, not by the ``_guard_trip_reason``
        read at the top of the loop: the reason is only latched *after* braking, so
        the main thread's ``_check_motion_guard`` and this loop can otherwise both
        see ``None`` and both brake, interleaving two ``stop()`` / ``set_target``
        sequences that pull in opposite directions.
        """
        while not stop_event.wait(_WATCHDOG_PERIOD_S):
            if self._cart_tracker is None or self._guard_trip_reason is not None:
                return
            try:
                violation = self._evaluate_guard()
            except Exception as exc:  # noqa: BLE001 - a dead robot handle must not spin
                self._logger.error("watchdog read failed: %s: %s", type(exc).__name__, exc)
                return
            if violation is None:
                continue
            kind, message = violation
            with self._guard_trip_lock:
                if self._guard_trip_reason is not None:
                    return
                live = np.asarray(
                    self._robot.state.O_T_EE.translation, dtype=np.float64
                )
                mode = str(self._robot.state.robot_mode)
                had_errors = bool(self._robot.has_errors)
                braked, cause = self._brake(kind)
                after = np.asarray(
                    self._robot.state.O_T_EE.translation, dtype=np.float64
                )
                self._guard_trip_reason = f"[watchdog:{kind}] {message}"
            self._logger.error(
                "motion guard WATCHDOG trip [%s]: %s; tcp=%s -> %s mode=%s "
                "has_errors=%s braked=%s stop=%s -- arm braked; the next command "
                "and the next guard_tripped() poll will report this",
                kind,
                message,
                np.round(live, 4).tolist(),
                np.round(after, 4).tolist(),
                mode,
                had_errors,
                braked,
                cause,
            )
            return

    def _start_watchdog(self) -> None:
        if self._watchdog is not None and self._watchdog.is_alive():
            return
        self._watchdog_stop = threading.Event()
        self._watchdog = threading.Thread(
            target=self._watchdog_loop,
            args=(self._watchdog_stop,),
            name="franky-motion-guard",
            daemon=True,
        )
        self._watchdog.start()

    def _stop_watchdog(self) -> None:
        if self._watchdog_stop is not None:
            self._watchdog_stop.set()
        thread = self._watchdog
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._watchdog = None
        self._watchdog_stop = None

    # ------------------------------------------------------------------
    # FrankaEnv-compatible arm API
    # ------------------------------------------------------------------

    def get_state(self):
        """``super`` + a fence check, so gaps between waypoints are covered.

        ``FrankaEnv`` calls this after every ``_interpolate_move`` and every
        ``step``, which turns those otherwise-unguarded moments into check points
        at no extra cost. The watchdog covers the rest.
        """
        state = super().get_state()
        self._check_motion_guard()
        return state

    def move_arm(self, position: np.ndarray) -> None:
        self.move_tcp_pose(np.asarray(position, dtype=np.float64))

    def move_tcp_pose(self, pose: np.ndarray) -> None:
        """``super`` + fail loudly on a dead tracker + enforce the motion guard.

        franky 0.19 ``CartesianImpedanceTracker.__init__`` starts
        ``robot.move(motion, asynchronous=True)``. If that thread dies it stores
        the exception and returns; ``set_target`` then writes to a dead reference
        handle and the arm silently stops following (``dz=0.0000`` across a whole
        interpolate -- LOG-017). ``stop()`` -> ``join_motion()`` re-raises the real
        cause, so surface it here instead of moving on.

        The guard checks cover the opposite failure (LOG-019): the arm moving
        *further* than commanded.

        A latched trip makes this a **no-op that logs**, rather than a raise. That
        is not leniency: ``super()`` would call ``_ensure_cart_tracking_motion``,
        which rebuilds the tracker that ``_abort_motion`` just tore down -- i.e.
        re-arming the impedance seconds after a runaway. Returning leaves the arm
        braked and lets the env's ``_raise_if_guard_tripped`` poll raise where a
        ``finally`` still runs.
        """
        if self._check_motion_guard() is not None:
            self._logger.error(
                "refusing to command motion: the motion guard is tripped (%s). "
                "The arm was braked and the tracker torn down; commanding again "
                "would rebuild it. Clear the fault in Desk and restart.",
                self._guard_trip_reason,
            )
            return
        self._check_requested_target(np.asarray(pose, dtype=np.float64)[:3])
        super().move_tcp_pose(pose)
        if self._cart_tracker is None or self._cart_tracker.is_running:
            self._check_motion_guard()
            return
        cause = "unknown (join_motion did not raise)"
        mode = str(self._robot.state.robot_mode)
        had_errors = bool(self._robot.has_errors)
        tcp = np.round(
            np.asarray(self._robot.state.O_T_EE.translation, dtype=np.float64), 4
        ).tolist()
        cause = self._stop_cart_tracker_no_recover()
        self._logger.error(
            "cartesian impedance control thread died (mode=%s has_errors=%s tcp=%s): %s",
            mode,
            had_errors,
            tcp,
            cause,
        )
        raise RuntimeError(
            f"cartesian impedance tracking stopped: {cause}; {describe_robot_mode(mode)}"
        )

    # ------------------------------------------------------------------
    # Gripper
    # ------------------------------------------------------------------

    def open_gripper(self) -> None:
        """Gentle open. Upstream asks for ``speed=1.0`` against a ~0.1 m/s hand."""
        self._gripper.open(speed=0.05)

    def close_gripper(self) -> None:
        """Gentle grasp (force/speed capped in FrankaLibfrankaGripper)."""
        self._gripper.close(speed=0.05)

    def close_gripper_force(self, force: float) -> None:
        self._gripper.close(speed=0.05, force=float(force))

    def stop_gripper(self) -> None:
        stop = getattr(self._gripper, "stop", None)
        if callable(stop):
            stop()
            return
        inner = getattr(self._gripper, "_gripper", None)
        inner_stop = getattr(inner, "stop", None) if inner is not None else None
        if callable(inner_stop):
            inner_stop()
            return
        raise RuntimeError("gripper has no stop()")

    def gripper_holding(self) -> bool:
        """True if the fingers currently hold an object of the calibrated size."""
        holding = getattr(self._gripper, "_hardware_holding", None)
        if callable(holding):
            return bool(holding())
        is_open = getattr(self._gripper, "is_open", None)
        return not bool(is_open) if is_open is not None else False

    def gripper_width(self) -> Optional[float]:
        try:
            return float(self._gripper.position)
        except Exception:  # noqa: BLE001 - diagnostic only
            return None

    def move_gripper(self, position: int, speed: float = 0.3) -> None:
        assert 0 <= position <= 255
        self._gripper.move(position=float(position), speed=speed)

    # ------------------------------------------------------------------
    # Impedance
    # ------------------------------------------------------------------

    def reconfigure_compliance_params(self, params: dict) -> None:
        """Apply charger-style ``compliance_param``, franky semantics only.

        Accepted: ``translational_stiffness``, ``rotational_stiffness`` (both
        clamped by :func:`~franky_ext.motion_limits.clamp_stiffness`).

        Deliberately ignored, with a warning:

        ``translational_damping`` / ``rotational_damping``
            ROS ``cartesian_impedance_controller`` damping coefficients. franky has
            no such knob (its damping is critical, derived from stiffness); the
            nearest parameter, ``gains_time_constant``, is a gain-ramp filter in
            *seconds*. The previous ``tc = 2*d/k`` mapping was numerology --
            ``2*89/2000 = 0.089 s`` happens to sit next to franky's own 0.1 s
            default, so it looked plausible while meaning nothing (LOG-019 R4).
        ``translational_clip_*`` / ``rotational_clip_*``
            Also ROS dynamic_reconfigure keys, and *not* franky's
            ``translational_error_clip``. Retracted once already in LOG-016; the
            clip is derived from stiffness instead.
        ``Ki``
            No integral term in the franky tracker.
        """
        req_trans = float(params.get("translational_stiffness", self._compliance_trans_k))
        req_rot = float(params.get("rotational_stiffness", self._compliance_rot_k))
        trans_k, rot_k = clamp_stiffness(req_trans, req_rot)
        if trans_k != req_trans or rot_k != req_rot:
            self._logger.warning(
                "compliance stiffness clamped: requested K_t=%.0f K_r=%.1f -> "
                "K_t=%.0f K_r=%.1f",
                req_trans,
                req_rot,
                trans_k,
                rot_k,
            )

        ignored = [
            key
            for key in (
                "translational_damping",
                "rotational_damping",
                "translational_clip_x",
                "translational_clip_y",
                "translational_clip_z",
                "rotational_clip_x",
                "rotational_clip_y",
                "rotational_clip_z",
                "rotational_clip_neg_x",
                "rotational_clip_neg_y",
                "rotational_clip_neg_z",
                "Ki",
            )
            if params.get(key) is not None
        ]
        if ignored:
            self._logger.warning(
                "reconfigure_compliance_params: ignoring ROS-only keys %s "
                "(franky has no equivalent; error clip is derived from stiffness)",
                ignored,
            )

        self._compliance_trans_k = trans_k
        self._compliance_rot_k = rot_k
        self._compliance_tc = fc._CART_GAINS_TC
        for msg in clip_shortfall(trans_k, rot_k):
            self._logger.warning("compliance: %s", msg)
        self._logger.info("compliance set -> %s", describe_authority(trans_k, rot_k))
        self._stop_cart_tracking_motion()
        self._stop_watchdog()

    def _ensure_cart_tracking_motion(self) -> None:
        if self._cart_tracker is not None:
            return
        if self._guard_trip_reason is not None:
            # Unreachable through move_tcp_pose, which refuses first. Kept as the
            # last line of defence because the failure it prevents -- re-arming a
            # 1 kHz torque motion right after a runaway was braked -- is the worst
            # one in this file, and raising costs only a process that must not be
            # commanding the arm anyway.
            raise RuntimeError(
                "refusing to rebuild the cartesian impedance tracker after a "
                f"motion guard trip: {self._guard_trip_reason}"
            )
        self._stop_tracking_motion()
        self._safe_join()
        self._robot.recover_from_errors()
        # A tracker started outside RobotMode.Idle dies within one control cycle;
        # set_target then writes to a dead handle and the arm silently holds still.
        # Say why up front instead.
        require_motion_ready(str(self._robot.state.robot_mode))
        nullspace_target = np.asarray(self._robot.state.q, dtype=np.float64).copy()
        # The clip is NOT an independent knob: stiffness * clip is the commanded
        # spring force, and that product is what must stay bounded. Taking the clip
        # from franky's module default while the stiffness came from PegInsertion's
        # compliance_param is exactly how the 100 N/axis ceiling of LOG-019 arose.
        trans_clip, rot_clip = error_clips_for_stiffness(
            self._compliance_trans_k, self._compliance_rot_k
        )
        for msg in clip_shortfall(self._compliance_trans_k, self._compliance_rot_k):
            self._logger.warning("tracker: %s", msg)

        kwargs = dict(
            translational_stiffness=self._compliance_trans_k,
            rotational_stiffness=self._compliance_rot_k,
            nullspace_target=nullspace_target,
            nullspace_stiffness=fc._CART_NULLSPACE_STIFFNESS,
            translational_error_clip=np.full(3, trans_clip, dtype=np.float64),
            rotational_error_clip=np.full(3, rot_clip, dtype=np.float64),
            max_delta_tau=fc._CART_MAX_DELTA_TAU,
            gains_time_constant=self._compliance_tc,
        )
        # Soft joint-limit repulsion is only active when BOTH limit vectors are
        # supplied; upstream supplies neither, so the impedance loop had no joint
        # protection at all beyond the hardware reflex. Passed defensively: an
        # older franky that rejects these kwargs must not take the tracker down.
        limit_kwargs = dict(
            lower_joint_limits=fc.JOINT_LIMITS_LOWER,
            upper_joint_limits=fc.JOINT_LIMITS_UPPER,
            joint_limit_activation_distance=_JOINT_LIMIT_ACTIVATION_RAD,
            joint_limit_stiffness=_JOINT_LIMIT_STIFFNESS,
            joint_limit_damping=_JOINT_LIMIT_DAMPING,
            joint_limit_max_torque=_JOINT_LIMIT_MAX_TORQUE,
        )
        joint_repulsion = True
        try:
            self._cart_tracker = self._franky.CartesianImpedanceTracker(
                self._robot, **kwargs, **limit_kwargs
            )
        except TypeError as exc:
            joint_repulsion = False
            self._logger.warning(
                "franky rejected the joint-limit kwargs (%s); starting the tracker "
                "WITHOUT soft joint-limit repulsion",
                exc,
            )
            self._cart_tracker = self._franky.CartesianImpedanceTracker(
                self._robot, **kwargs
            )

        self._logger.info(
            "Cartesian impedance tracker started (robot_in_control=%s, tc=%.3f, "
            "K_ns=%.1f, joint_repulsion=%s) %s",
            self._cart_tracker.is_running,
            self._compliance_tc,
            fc._CART_NULLSPACE_STIFFNESS,
            joint_repulsion,
            describe_authority(self._compliance_trans_k, self._compliance_rot_k),
        )
        self._start_watchdog()

    def cleanup(self) -> None:
        """Stop the watchdog before upstream tears the trackers down."""
        self._stop_watchdog()
        super().cleanup()

    def command_end_effector(self, action: np.ndarray) -> bool:
        raise NotImplementedError(
            "FrankyControllerExtended: dexterous hands not supported."
        )

    def reset_end_effector(self, target_state) -> None:
        raise NotImplementedError(
            "FrankyControllerExtended: dexterous hands not supported."
        )
