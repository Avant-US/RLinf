#!/usr/bin/env python3
"""Phase 1A: dummy gym.make for FrankyCubePlaceEnv-v1 (no robot / FCI)."""

from __future__ import annotations

import inspect
import os
import sys

import gymnasium as gym
import numpy as np

REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

from rlinf.envs.realworld.franka.franky_controller import (  # noqa: E402
    JOINT_VEL_LIMITS,
)

import franky_ext.tasks.register  # noqa: E402,F401
from franky_ext.motion_limits import (  # noqa: E402
    INTERP_DURATION_S_RANGE,
    MAX_INTERP_ANGLE_RAD,
    MAX_INTERP_DISTANCE_M,
    cartesian_collision_thresholds,
    clamp_stiffness,
    clip_shortfall,
    error_clips_for_stiffness,
    force_ceiling_n,
    force_norm_ceiling_n,
    guard_max_dq_rad_s,
    guard_recovery_budget,
    holding_from_width,
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
    torque_ceiling_nm,
    torque_norm_ceiling_nm,
)
from franky_ext.tasks.cube_place import CubePlaceConfig, FrankyCubePlaceEnv  # noqa: E402
from franky_ext.tcp_probe import (  # noqa: E402
    PEG_RPY_HALF_WIDTH_RAD,
    check_start_pose,
    effective_ee_pose_limits,
)


def _check_motion_limits() -> None:
    """Cover the LOG-019 safety arithmetic without a robot.

    All of it is pure numerics on purpose, so the guarantees that used to depend
    on nobody making a mistake are now regression-tested in 1-2 seconds.
    """
    root3 = float(np.sqrt(3.0))
    # 1. Force ceiling survives a stiffness change, per-axis AND in norm. This is
    #    R2: the old code paired PegInsertion's K_t=2000 with franky's default
    #    0.05 m clip and silently commanded 100 N/axis. The norm check exists
    #    because franky clips ELEMENTWISE, so three saturated axes command
    #    per_axis * sqrt(3) -- the first version of this test only checked
    #    per-axis and would have passed a 17.3 Nm worst-case torque against a
    #    12 Nm joint limit.
    f_axis = force_ceiling_n()
    t_axis = torque_ceiling_nm()
    f_norm = force_norm_ceiling_n()
    t_norm = torque_norm_ceiling_nm()
    for k_t, k_r in ((500.0, 40.0), (2000.0, 150.0), (3000.0, 300.0)):
        clip_m, clip_rad = error_clips_for_stiffness(k_t, k_r)
        force = k_t * clip_m
        torque = k_r * clip_rad
        assert force <= f_axis + 1e-6, f"K_t={k_t} -> {force:.1f}N/axis > {f_axis}N"
        assert torque <= t_axis + 1e-6, f"K_r={k_r} -> {torque:.2f}Nm/axis > {t_axis}Nm"
        assert force * root3 <= f_norm + 1e-6, (
            f"K_t={k_t} -> {force * root3:.1f}N worst-case norm > {f_norm}N"
        )
        assert torque * root3 <= t_norm + 1e-6, (
            f"K_r={k_r} -> {torque * root3:.2f}Nm worst-case norm > {t_norm}Nm"
        )
        assert clip_m > 0 and clip_rad > 0
    print(
        f"force ceiling holds across stiffness: <= {f_axis}N/axis "
        f"({f_norm}N norm) / {t_axis}Nm/axis ({t_norm}Nm norm)"
    )

    # 1b. The shipping stiffness must actually ACHIEVE its ceiling. At low K the
    #     clip range clamps and the realised ceiling silently drops -- at the
    #     bottom of the stiffness range that is 2.5 N, not enough to overcome this
    #     arm's stiction, i.e. a stalled arm presented as a normal run.
    assert not clip_shortfall(2000.0, 150.0), clip_shortfall(2000.0, 150.0)
    assert clip_shortfall(50.0, 5.0), "expected a shortfall warning at minimum stiffness"
    print("clip shortfall reported at low stiffness, silent at the shipping values")

    # 2. Stiffness itself is bounded, so a REPL typo or a bad YAML cannot build a
    #    monster tracker even at a small clip.
    assert clamp_stiffness(1e9, 1e9) == (3000.0, 300.0)
    assert clamp_stiffness(0.0, 0.0) == (50.0, 5.0)
    print("stiffness clamped to [50,3000] / [5,300]")

    # 3. Interpolated moves are speed-capped. Upstream would have run the 10 cm
    #    lift in the requested 1.0 s (= 10 cm/s); the cap stretches it.
    v_max = interp_speed_m_s()
    slow = interp_duration_s([0, 0, 0], [0, 0, 0.10], requested=1.0)
    assert slow >= 0.10 / v_max - 1e-6, slow
    assert 0.10 / slow <= v_max + 1e-6, f"{0.10 / slow:.4f} m/s > {v_max}"
    # A caller asking for something slower than the cap is honoured, never sped up.
    assert interp_duration_s([0, 0, 0], [0, 0, 0.01], requested=5.0) >= 5.0
    print(f"interpolate speed capped at {v_max * 100:.1f}cm/s (10cm needs {slow:.2f}s)")

    # 4. The start-pose gate catches exactly the LOG-019 setup: 6.6 mm below the
    #    calibrated contact point, xy on target.
    target = [0.7062065, 0.03620906, 0.23192134, -3.116, 0.026, 0.179]
    incident = [0.7074, 0.0445, 0.2254, -3.114, 0.067, 0.185]
    problems = check_start_pose(
        incident, target, clip_xy=0.05, z_low=0.005, z_high=0.08, min_clearance_m=0.005
    )
    assert problems, "start-pose gate missed the LOG-019 start pose"
    assert any("below" in p for p in problems), problems
    # And a legitimate start a few cm above the mark passes.
    good = [0.7062, 0.0362, 0.23192134 + 0.04, -3.116, 0.026, 0.179]
    assert not check_start_pose(
        good, target, clip_xy=0.05, z_low=0.005, z_high=0.08, min_clearance_m=0.005
    )
    # xy far away is caught too.
    far = [0.7062 + 0.20, 0.0362, 0.27, -3.116, 0.026, 0.179]
    assert check_start_pose(
        far, target, clip_xy=0.05, z_low=0.005, z_high=0.08, min_clearance_m=0.005
    )
    # Too high is caught as well, because every cm of start height is a cm out of
    # the motion guard's headroom (go_to_rest lifts from wherever it starts).
    high = [0.7062, 0.0362, 0.23192134 + 0.08 + 0.03, -3.116, 0.026, 0.179]
    assert check_start_pose(
        high, target, clip_xy=0.05, z_low=0.005, z_high=0.08, min_clearance_m=0.005
    )
    print("start-pose gate rejects the LOG-019 pose and accepts a good one")

    # 5. A displacement large enough that the duration clamp would speed the move
    #    back up must be REFUSED, not silently accelerated. Invariant:
    #    MAX_INTERP_DISTANCE_M <= max_duration * speed_cap.
    assert MAX_INTERP_DISTANCE_M <= INTERP_DURATION_S_RANGE[1] * v_max + 1e-9, (
        f"{MAX_INTERP_DISTANCE_M}m would need more than "
        f"{INTERP_DURATION_S_RANGE[1]}s at {v_max}m/s, so the duration clamp would "
        "re-introduce a fast move"
    )
    print(
        f"interp distance cap {MAX_INTERP_DISTANCE_M}m <= "
        f"{INTERP_DURATION_S_RANGE[1]}s x {v_max}m/s"
    )

    # 5b. Rotation gets the same refusal as translation. Reachable in practice: a
    #     calibration written from a quaternion's first three components puts the
    #     target orientation near identity instead of near gripper-down, both the
    #     box and the orientation fence re-centre on it, so no guard fires -- and
    #     reset commands a ~180 deg wrist flip with a cube in the gripper.
    assert 0.2 < MAX_INTERP_ANGLE_RAD < 1.0, MAX_INTERP_ANGLE_RAD
    assert MAX_INTERP_ANGLE_RAD < np.pi / 2, "a 90 deg wrist flip must be refused"
    print(f"interp angle cap {MAX_INTERP_ANGLE_RAD}rad")

    # 5c. The step() path has its own cap, because it does NOT go through
    #     _interpolate_move: FrankaEnv.step adds action*action_scale[0] to the
    #     measured pose and calls _move_action directly. PegInsertion's 0.02 m/step
    #     at 10 Hz is 20 cm/s -- twice LOG-019's commanded speed.
    step_cap = max_action_scale_xyz(10.0)
    assert abs(step_cap * 10.0 - step_speed_m_s()) < 1e-9
    assert step_cap < 0.02, f"step cap {step_cap} must be tighter than upstream 0.02"
    print(
        f"step action_scale cap {step_cap:.4f}m/step at 10Hz "
        f"= {step_speed_m_s() * 100:.0f}cm/s (upstream 0.02 = 20cm/s)"
    )

    # 5c-bis. The step slew clamp is deliberately SKIPPED during _interpolate_move,
    #     because upstream reads _franka_state once before its waypoint loop and never
    #     refreshes it, so the clamp's reference pose would be stale for the whole
    #     call (it pinned the effective target at start+budget and reset stalled 4.2 cm
    #     short of hover). Skipping is only safe while interpolation is the SLOWER of
    #     the two paths -- assert that, so raising RLINF_CUBE_INTERP_SPEED above the
    #     step speed cannot silently invalidate the reasoning.
    assert interp_speed_m_s() <= step_speed_m_s() + 1e-12, (
        f"interp speed {interp_speed_m_s()} exceeds step speed {step_speed_m_s()}: "
        "interpolation waypoints would exceed the step budget, and _move_action skips "
        "the slew clamp during interpolation"
    )
    print(
        f"interp {interp_speed_m_s() * 100:.1f}cm/s <= step "
        f"{step_speed_m_s() * 100:.1f}cm/s (so skipping the slew clamp mid-interp is safe)"
    )

    # 5c-ter. LOG-034: the step() path's ROTATION needed the same cap as 5c's
    #     translation, and did not have one -- action_scale[1]=0.1rad/step at 10Hz
    #     is 1.0rad/s per-axis (1.73rad/s worst-case 3-axis), unclamped anywhere on
    #     this path.
    rot_cap = max_action_scale_rot(10.0)
    assert abs(rot_cap * 10.0 - step_rot_speed_rad_s()) < 1e-9
    assert 0.0 < rot_cap < 0.1, f"rotation step cap {rot_cap} looks unbounded"
    print(
        f"step action_scale[1] cap {rot_cap:.4f}rad/step at 10Hz "
        f"= {step_rot_speed_rad_s():.2f}rad/s (upstream 0.1 = 1.0rad/s)"
    )

    # 5d. libfranka's own 1 kHz reflex must be brought down to the commanded
    #     ceiling. Upstream leaves it at 100 N, five times above -- so the hardware
    #     bound could never fire first and every real bound was Python, in a
    #     GIL-bound watchdog thread supervising a 1 kHz loop.
    thresholds = cartesian_collision_thresholds()
    assert len(thresholds) == 6, thresholds
    assert thresholds[0] <= force_norm_ceiling_n() + 1e-9, thresholds
    assert thresholds[0] < 100.0, "must be tighter than upstream's 100 N"
    assert thresholds[3] <= torque_norm_ceiling_nm() + 1e-9, thresholds
    print(
        f"cartesian collision thresholds {thresholds[0]:.0f}N / {thresholds[3]:.0f}Nm "
        "(upstream 100N / 25Nm)"
    )

    # 5e. holding_from_width must decide "is this the calibrated cube" from width
    #     alone, and it is what tcp_probe.py's pre-Ray gate now uses INSTEAD of
    #     libfranka's is_grasped (round-2 audit finding 5: the pre-Ray gate was the
    #     one place in the stack still trusting is_grasped after the controller-side
    #     gripper moved to a width-only rule).
    assert holding_from_width(None) is None
    assert holding_from_width(0.000) is False   # empty hand closed on air
    assert holding_from_width(0.046) is True    # nominal cube
    assert holding_from_width(0.090) is False   # fully open
    print("holding_from_width decides from measured width alone (empty/nominal/open)")

    # 5f. A per-axis ceiling request that the norm ceiling silently caps must warn.
    #     Previously --force-ceiling 30 or 35 both delivered only 23.1 N/axis
    #     (40 / sqrt(3)) with nothing saying so, because clip_shortfall compared
    #     against the already-min()-ed value (round-2 audit finding 6).
    assert not clip_shortfall(2000.0, 150.0), "shipping defaults must not warn"
    import os as _os

    _os.environ["RLINF_CUBE_FORCE_CEILING_N"] = "35"
    try:
        capped = clip_shortfall(2000.0, 150.0)
        assert any("capped by the norm ceiling" in m for m in capped), capped
    finally:
        _os.environ.pop("RLINF_CUBE_FORCE_CEILING_N", None)
    print("clip_shortfall warns when the norm ceiling silently caps a per-axis request")

    # 6. The orientation fence must be wrap-free. This task's roll is ~-3.116 rad,
    #    a hair from -pi, so comparing euler angles to ee_pose_limit[3:] would abort
    #    a perfectly good motion whenever the reading lands on the +pi side.
    from scipy.spatial.transform import Rotation as _R

    # The three half-widths are NOT equal: PegInsertion pins roll/pitch to 0.01 rad
    # and only yaw gets clip_rz. Assuming clip_rz on all three made the fence 1.5x
    # looser than its own reasoning implied.
    fence = orientation_fence_rad(PEG_RPY_HALF_WIDTH_RAD, PEG_RPY_HALF_WIDTH_RAD, 0.35)
    fence_wrong = orientation_fence_rad(0.35, 0.35, 0.35)
    assert fence < fence_wrong, (fence, fence_wrong)
    assert 0.4 < fence < 0.7, fence
    rpy = [-3.116, 0.026, 0.179]
    q_a = _R.from_euler("xyz", rpy).as_quat()
    # Physically identical rotation, expressed 2*pi away in roll: euler distance
    # 6.28, true angle 0.
    q_b = _R.from_euler("xyz", [rpy[0] + 2 * np.pi, rpy[1], rpy[2]]).as_quat()
    assert quat_angle_rad(q_a, q_b) < 1e-6, quat_angle_rad(q_a, q_b)
    # A genuine 90 deg wrist twist must be outside the fence.
    q_c = _R.from_euler("xyz", [rpy[0], rpy[1], rpy[2] + np.pi / 2]).as_quat()
    assert quat_angle_rad(q_a, q_c) > fence, quat_angle_rad(q_a, q_c)
    print(
        f"orientation fence {fence:.3f}rad is wrap-free "
        f"(2pi-equivalent -> {quat_angle_rad(q_a, q_b):.2e}rad, "
        f"90deg twist -> {quat_angle_rad(q_a, q_c):.3f}rad)"
    )

    # 7. The box printed to the operator must be the box the env enforces.
    #    PegInsertionConfig.__post_init__ recomputes ee_pose_limit_* from
    #    target_ee_pose + the clip ranges, pinning roll/pitch to +/-0.01 rad -- so a
    #    generic "probe-centred box with one rpy margin" reports an orientation
    #    window 35x wider than the enforced one, as pre-flight "verification".
    eff_lo, eff_hi = effective_ee_pose_limits(
        target, clip_xy=0.05, z_low=0.005, z_high=0.08, clip_rz=0.35
    )
    built = CubePlaceConfig(
        is_dummy=True,
        target_ee_pose=list(target),
        clip_x_range=0.05,
        clip_y_range=0.05,
        clip_z_range_low=0.005,
        clip_z_range_high=0.08,
        clip_rz_range=0.35,
    )
    assert np.allclose(eff_lo, np.asarray(built.ee_pose_limit_min), atol=1e-9), (
        eff_lo,
        built.ee_pose_limit_min,
    )
    assert np.allclose(eff_hi, np.asarray(built.ee_pose_limit_max), atol=1e-9), (
        eff_hi,
        built.ee_pose_limit_max,
    )
    # Tolerance, not equality: the half-width is recovered by subtracting two
    # numbers near -3.116, so it lands on 0.010000000000000231.
    assert abs(abs(eff_hi[3] - eff_lo[3]) / 2 - PEG_RPY_HALF_WIDTH_RAD) < 1e-9
    assert abs(abs(eff_hi[5] - eff_lo[5]) / 2 - 0.35) < 1e-9, "yaw should get clip_rz"
    print(
        "effective_ee_pose_limits matches the constructed config "
        f"(roll/pitch half-width {PEG_RPY_HALF_WIDTH_RAD}rad, not clip_rz)"
    )

    # 8. is_dummy and robot_ip must agree: is_dummy is the ONLY thing keeping this
    #    regression off the hardware, and it used to live in four copy-pasted
    #    places with the dangerous value as the dataclass default.
    try:
        CubePlaceConfig(is_dummy=False, robot_ip="0.0.0.0", target_ee_pose=list(target))
    except ValueError:
        pass
    else:
        raise AssertionError("is_dummy=False with a placeholder robot_ip was accepted")
    CubePlaceConfig(is_dummy=True, robot_ip=None, target_ee_pose=list(target))
    print("is_dummy / robot_ip invariant enforced")

    # 9. Config drift: every franky-specific safety field on
    #    FrankySingleFrankaEnvConfig must also exist on CubePlaceConfig, which
    #    re-declares them because it has to derive from PegInsertionConfig.
    import dataclasses

    from rlinf.envs.realworld.franka.franka_env import FrankaRobotConfig

    from franky_ext.franky_single_franka_env import FrankySingleFrankaEnvConfig

    base_names = {f.name for f in dataclasses.fields(FrankaRobotConfig)}
    franky_extra = {
        f.name for f in dataclasses.fields(FrankySingleFrankaEnvConfig)
    } - base_names
    cube_names = {f.name for f in dataclasses.fields(CubePlaceConfig)}
    missing = franky_extra - cube_names
    assert not missing, (
        f"CubePlaceConfig is missing franky env fields {sorted(missing)}; the mixin "
        "reads them with getattr defaults, so a new safety field would silently "
        "have no effect on the cube-place path"
    )
    print(f"config fields in sync: {sorted(franky_extra)}")


def _unwrap(env):
    cur = env
    seen = []
    while True:
        seen.append(type(cur).__name__)
        if isinstance(cur, FrankyCubePlaceEnv):
            return cur, seen
        nxt = getattr(cur, "env", None)
        if nxt is None or nxt is cur:
            break
        cur = nxt
    raise AssertionError(f"FrankyCubePlaceEnv not in wrapper stack: {seen}")


def main() -> int:
    spec = gym.spec("FrankyCubePlaceEnv-v1")
    print(f"gym_id={spec.id}")
    assert spec.id == "FrankyCubePlaceEnv-v1"

    env = gym.make(
        "FrankyCubePlaceEnv-v1",
        override_cfg={
            "is_dummy": True,
            "enable_camera_player": False,
            "camera_serials": ["000000000000"],
            "target_ee_pose": [0.5, 0.0, 0.1, 3.14, 0.0, 0.0],
        },
        worker_info=None,
        hardware_info=None,
        env_idx=0,
        env_cfg={},
    )
    inner, stack = _unwrap(env)
    print(f"wrapper_stack={stack}")
    print(f"action_space={env.action_space}")
    assert env.action_space.shape == (6,), env.action_space

    cfg = inner.config
    print(
        "clips "
        f"xy=({cfg.clip_x_range},{cfg.clip_y_range}) "
        f"z_low={cfg.clip_z_range_low} z_high={cfg.clip_z_range_high} "
        f"rand_xy={cfg.random_xy_range}"
    )
    assert cfg.clip_x_range == 0.05
    assert cfg.clip_y_range == 0.05
    assert cfg.clip_z_range_low == 0.005
    assert cfg.clip_z_range_high == 0.08
    assert cfg.random_xy_range == 0.03
    # 0.03, not PegInsertion's 0.10: see CubePlaceConfig.reset_z_lift_m (LOG-019).
    assert cfg.reset_z_lift_m == 0.03, cfg.reset_z_lift_m
    print(f"reset_z_lift_m={cfg.reset_z_lift_m}")

    # LOG-034 (T16): CubePlaceConfig must clamp BOTH action_scale entries, not
    # just translation -- a config built with upstream's uncapped rotation scale
    # must come out capped, exactly like the translation case above.
    oversized = CubePlaceConfig(
        is_dummy=True,
        target_ee_pose=[0.5, 0.0, 0.1, 3.14, 0.0, 0.0],
        action_scale=[0.02, 0.1],
    )
    assert oversized.action_scale[0] <= max_action_scale_xyz(oversized.step_frequency) + 1e-9
    assert oversized.action_scale[1] <= max_action_scale_rot(oversized.step_frequency) + 1e-9
    print(
        f"CubePlaceConfig clamps action_scale=[0.02, 0.1] -> "
        f"{np.round(oversized.action_scale, 4).tolist()}"
    )

    # _clamp_step_slew must clamp rotation as well as translation -- the source
    # inspection (rather than exercising it live, which needs a robot) at least
    # catches a regression that removes the rotation branch entirely.
    from franky_ext.franky_single_franka_env import FrankySingleFrankaEnvMixin

    slew_src = inspect.getsource(FrankySingleFrankaEnvMixin._clamp_step_slew)
    assert "max_action_scale_rot" in slew_src, (
        "_clamp_step_slew lost its rotation clamp (LOG-034/T16 regression)"
    )
    print("_clamp_step_slew still clamps rotation, not just translation")

    # The joint-space clamp is the only bound that sees WHERE the arm is, so a
    # regression that drops it re-opens LOG-036 exactly. Full coverage lives in
    # test_t19_joint_demand.py; this is the standing gate.
    assert "_clamp_joint_demand" in slew_src, (
        "_clamp_step_slew lost its joint-demand clamp (LOG-037/T19 regression)"
    )
    assert slew_src.find("_clamp_joint_demand") > slew_src.find(
        "max_action_scale_rot"
    ), "the joint clamp must run last, on the already-Cartesian-clamped pose"
    # A zeroed Jacobian is what a dummy env carries, and it must stay a no-op.
    assert joint_demand_scale(
        np.zeros((6, 7)), [0.005, 0, 0], [0, 0, 0], 0.1, JOINT_VEL_LIMITS
    )[0] == 1.0, "a zero (no-data) Jacobian must not clamp anything"
    # Unit-gain Jacobian, so the demand equals the commanded speed in rad/s: a
    # full-speed step is ~0.05 rad/s and must pass; a 100x-degenerate axis must not.
    unit_J = np.zeros((6, 7))
    unit_J[:6, :6] = np.eye(6)
    assert joint_demand_scale(
        unit_J, [step_speed_m_s() * 0.1, 0, 0], [0, 0, 0], 0.1, JOINT_VEL_LIMITS
    )[0] == 1.0, "a well-conditioned full-speed step must not be clamped"
    bad_J = unit_J.copy()
    bad_J[0, 0] = 0.01
    bad_scale, bad_ratio, bad_joint = joint_demand_scale(
        bad_J, [step_speed_m_s() * 0.1, 0, 0], [0, 0, 0], 0.1, JOINT_VEL_LIMITS
    )
    assert bad_scale < 1.0 and bad_joint == 1, "an ill-conditioned step must be clamped"
    print(
        f"joint-demand clamp wired and live: ill-conditioned step j{bad_joint} "
        f"{bad_ratio:.2f}x budget -> x{bad_scale:.4f}; "
        f"budget {joint_vel_demand_fraction():.2f} of joint limits, "
        f"guard |dq| <= {guard_max_dq_rad_s():.2f}rad/s"
    )

    # The guard must name a joint runaway as one. Both real trips were reported
    # as "lag" because joint speed was not a criterion (LOG-034, LOG-036).
    from franky_ext.controller_extended import FrankyControllerExtended

    guard_src = inspect.getsource(FrankyControllerExtended._evaluate_guard)
    dq_at, lag_at = guard_src.find('"dq"'), guard_src.find('"lag"')
    assert 0 <= dq_at < lag_at, (
        "_evaluate_guard must check |dq| BEFORE lag, or a joint runaway is "
        "misreported as a tracking failure (LOG-037/T19)"
    )
    assert 'kind in ("lag", "dq")' in inspect.getsource(
        FrankyControllerExtended._brake
    ), "a dq trip must brake with the lag order (freeze-then-stop)"
    print("guard checks |dq| before lag, and brakes it freeze-then-stop")

    # A trip must end the episode, not the run (LOG-040/T21), and it must be a
    # *trip* that is caught: catching bare RuntimeError here would also swallow
    # the interpolation refusals a few lines away, which must stay fatal. Full
    # coverage lives in test_t21_t22_recovery.py; this is the standing gate.
    from franky_ext.franky_single_franka_env import (
        FrankySingleFrankaEnvMixin,
        MotionGuardTripped,
    )

    assert issubclass(MotionGuardTripped, RuntimeError)
    assert "raise MotionGuardTripped(" in inspect.getsource(
        FrankySingleFrankaEnvMixin._raise_if_guard_tripped
    ), "the guard poll must raise the specific type, or recovery cannot tell it apart"
    for method in (FrankySingleFrankaEnvMixin.step, FrankySingleFrankaEnvMixin.reset):
        method_src = inspect.getsource(method)
        assert "except MotionGuardTripped" in method_src, (
            f"{method.__name__} lost its trip recovery (LOG-040/T21 regression)"
        )
        assert "except RuntimeError" not in method_src, (
            f"{method.__name__} must not catch bare RuntimeError: interpolation "
            "refusals are not trips and must stay fatal"
        )
    assert hasattr(FrankyControllerExtended, "recover_from_guard_trip"), (
        "controller lost the explicit trip-recovery API"
    )
    # The elbow self-check is the only thing in the stack that looks at joint
    # configuration at all; a dummy env carries a zero Jacobian and must stay
    # silent rather than reporting a perfect singularity (LOG-040/T22).
    assert jacobian_conditioning(np.zeros((6, 7))) is None
    unit_cond = jacobian_conditioning(unit_J)
    assert abs(unit_cond["sigma_min"] - 1.0) < 1e-12, unit_cond
    assert sigma_min_warn() < 0.1115, (
        "the sigma_min warning must stay below the known-good hover pose, or "
        "every reset warns (LOG-037/LOG-040 anchors)"
    )
    assert sigma_min_warn() > 0.0062, (
        "the sigma_min warning must stay above the LOG-036 trip pose, or it "
        "never fires where it mattered"
    )
    print(
        f"trip recovery wired (budget {guard_recovery_budget()} per env) and elbow "
        f"self-check warns below sigma_min={sigma_min_warn():.4f}"
    )

    src = inspect.getsource(FrankyCubePlaceEnv.go_to_rest)
    assert "np.array([-1.0])" in src or "np.array([-1.])" in src, src
    assert "np.array([1.0])" not in src and "np.array([1.])" not in src, src
    print("go_to_rest uses close (-1.0), not open (+1.0)")

    # The guard/limit plumbing must exist on the env, or a real run silently has
    # no fence: nothing else in the stack bounds an interpolated move.
    assert hasattr(inner, "arm_motion_guard"), "mixin lost arm_motion_guard"
    assert hasattr(FrankyCubePlaceEnv, "_check_start_pose")
    from franky_ext.controller_extended import FrankyControllerExtended

    for name in ("set_motion_guard", "motion_health", "freeze_at_current", "gripper_holding"):
        assert hasattr(FrankyControllerExtended, name), f"controller lost {name}"

    # arm_motion_guard() is never exercised without a robot: the dummy path skips
    # _setup_hardware, so there is no controller to call. A typo in the keyword
    # names would therefore only surface on the real arm, at the exact moment the
    # fence was supposed to be armed. Bind the real call signature here instead --
    # the Worker proxy forwards **kwargs verbatim (worker_group.py:436), so a
    # successful bind is the same check the RPC will do.
    guard_sig = inspect.signature(FrankyControllerExtended.set_motion_guard)
    guard_sig.bind(
        None,  # self
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        extra_z_up=0.03,
        target_quat=[0.0, 0.0, 0.0, 1.0],
        max_orient_err=0.8,
    )
    for name in ("motion_health", "freeze_at_current", "gripper_holding"):
        inspect.signature(getattr(FrankyControllerExtended, name)).bind(None)
    print("motion guard API present, and arm_motion_guard's call signature binds")

    # A latched guard trip must make the controller REFUSE, not raise. A raise from
    # a Worker method is caught by WorkerGroupFuncResult, which signals the driver,
    # whose handler ray.kills every actor and exits -- so it fired before the
    # env-side guard_tripped() poll could run, and before _close_env could stop the
    # impedance tracker (LOG-023 finding 1). Exercised on a bare instance because
    # __init__ needs a robot; only the guard bookkeeping is touched.
    import logging
    import threading

    latched = FrankyControllerExtended.__new__(FrankyControllerExtended)
    latched._guard_trip_reason = "[watchdog:fence] test"
    latched._guard_enabled = False
    latched._guard_min_xyz = None
    latched._guard_trip_lock = threading.Lock()
    latched._logger = logging.getLogger("phase1a-guard-latch")
    assert latched._check_motion_guard() == "[watchdog:fence] test"
    assert latched.guard_tripped() == "[watchdog:fence] test"
    # Must return without touching _robot / _cart_tracker -- both absent here, so an
    # attempt to command motion would raise AttributeError and fail this assertion.
    latched.move_tcp_pose(np.zeros(7, dtype=np.float64))
    # Same for a second trip: _abort_motion must see the latch and return before it
    # brakes again. Two brakes running concurrently is the failure _guard_trip_lock
    # exists for -- for a fence trip one thread's set_target(measured) can re-point
    # the target at a pose the other thread's stop() is decelerating away from.
    latched._abort_motion("fence", "second violation")
    assert latched._guard_trip_reason == "[watchdog:fence] test", (
        "_abort_motion overwrote an existing latch instead of returning"
    )
    print("a latched guard trip refuses further motion instead of raising")

    _check_motion_limits()

    obs, _info = env.reset()
    assert obs is not None
    action = np.zeros(env.action_space.shape, dtype=np.float32)
    obs, reward, terminated, truncated, info = env.step(action)
    assert obs is not None
    print(f"dummy_step reward={reward} terminated={terminated} truncated={truncated}")
    env.close()
    print("Phase1A PASS FrankyCubePlaceEnv-v1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
