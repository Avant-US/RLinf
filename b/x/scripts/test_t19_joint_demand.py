#!/usr/bin/env python3
"""Acceptance test for LOG-037's T19 fix: the joint-space demand clamp + |dq| gate.

Two halves, because the fix has two halves that fail differently:

* **Math** (default, no hardware): ``joint_demand_scale`` on synthetic Jacobians
  whose answers are known in closed form, plus the property that actually matters
  -- after scaling, the demand sits exactly at budget and not merely "lower".
  Also checks the pieces around it: that the guard threshold is between
  known-normal and known-bad, that ``_brake`` gives ``dq`` the ``lag`` order (the
  wrong order there is worse than not braking), and that ``_evaluate_guard``
  checks ``dq`` *before* ``lag`` so a joint runaway is not misfiled as a tracking
  failure the way LOG-034 and LOG-036 both were.
* **Real** (``--robot``, read-only): the same clamp against Jacobians computed at
  the two configurations LOG-037 measured -- the hover pose, where the clamp must
  be invisible, and the pose training tripped at, where it must bite. This is the
  half that would catch a wrong Jacobian frame or a transposed matrix, which no
  synthetic test can.

Run inside the franky container after ``source b/x/configs/setup_before_ray_5090.sh``.
``--robot`` connects but commands NO motion (it only reads ``F_T_EE`` and the
kinematic model, then evaluates it at given ``q``).
"""

from __future__ import annotations

import argparse
import inspect
import os
import sys

import numpy as np

REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

from rlinf.envs.realworld.franka.franky_controller import (  # noqa: E402
    JOINT_VEL_LIMITS,
)

from franky_ext.motion_limits import (  # noqa: E402
    GUARD_MAX_DQ_RAD_S_DEFAULT,
    guard_max_dq_rad_s,
    interp_speed_m_s,
    interp_speed_rad_s,
    joint_demand_scale,
    joint_vel_demand_fraction,
    step_rot_speed_rad_s,
    step_speed_m_s,
)

# LOG-037's measured configurations, and what this arm was measured to do there.
TRIP_Q = [0.0330, 1.1360, -0.0341, -0.4233, 0.0853, 1.4967, 0.5404]
HOVER_Q = [0.2304, 0.5462, -0.2855, -1.2968, 0.1800, 1.7952, 1.0502]

DT = 0.1  # 10 Hz step_frequency, this task's control period

_failures: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  {detail}" if detail else ""))
    if not ok:
        _failures.append(name)


def _well_conditioned_jacobian() -> np.ndarray:
    """A 6x7 Jacobian with unit gain: 1 m/s of twist costs 1 rad/s of joint."""
    J = np.zeros((6, 7))
    J[:6, :6] = np.eye(6)
    return J


def _ill_conditioned_jacobian(sigma_min: float) -> np.ndarray:
    """Unit gain on five axes, ``sigma_min`` on the sixth (the z-rotation row)."""
    J = _well_conditioned_jacobian()
    J[5, 5] = sigma_min
    return J


def test_math() -> None:
    print("\n=== math: joint_demand_scale ===")
    frac = joint_vel_demand_fraction()
    budget = JOINT_VEL_LIMITS * frac

    # No data must mean no opinion. The dataclass default for arm_jacobian is
    # zeros, which a dummy env and any pre-first-read state both carry; treating
    # that as "infinitely stiff" would clamp every step to nothing.
    scale, ratio, joint = joint_demand_scale(
        np.zeros((6, 7)), [0.005, 0, 0], [0, 0, 0], DT, JOINT_VEL_LIMITS
    )
    check("zero jacobian is a no-op", scale == 1.0 and joint == 0, f"scale={scale}")

    scale, _, _ = joint_demand_scale(
        np.zeros((3, 7)), [0.005, 0, 0], [0, 0, 0], DT, JOINT_VEL_LIMITS
    )
    check("wrong-shape jacobian is a no-op", scale == 1.0, f"scale={scale}")

    scale, _, _ = joint_demand_scale(
        _well_conditioned_jacobian(), [0, 0, 0], [0, 0, 0], DT, JOINT_VEL_LIMITS
    )
    check("zero step is a no-op", scale == 1.0, f"scale={scale}")

    scale, _, _ = joint_demand_scale(
        _well_conditioned_jacobian(), [0.005, 0, 0], [0, 0, 0], 0.0, JOINT_VEL_LIMITS
    )
    check("dt=0 is a no-op", scale == 1.0, f"scale={scale}")

    # Unit gain: a full-speed step demands exactly the commanded speed in rad/s,
    # which is ~0.05 -- two orders under budget, so it must pass untouched.
    step = step_speed_m_s() * DT
    scale, ratio, joint = joint_demand_scale(
        _well_conditioned_jacobian(), [step, 0, 0], [0, 0, 0], DT, JOINT_VEL_LIMITS
    )
    check(
        "well-conditioned full-speed step is unclamped",
        scale == 1.0,
        f"demand {ratio * budget[joint - 1]:.4f} rad/s vs budget {budget[joint - 1]:.4f}",
    )

    # The case the fix exists for: a step that is fully compliant with both
    # Cartesian caps, at a pose where the Jacobian turns it into an impossible
    # joint demand.
    rot_step = step_rot_speed_rad_s() * DT
    J = _ill_conditioned_jacobian(0.01)
    scale, ratio, joint = joint_demand_scale(
        J, [0, 0, 0], [0, 0, rot_step], DT, JOINT_VEL_LIMITS
    )
    expected_demand = (rot_step / DT) / 0.01  # 30 rad/s on joint 6
    check(
        "ill-conditioned step is clamped",
        scale < 1.0 and joint == 6,
        f"scale={scale:.4f} worst=j{joint} ratio={ratio:.2f}x",
    )
    check(
        "reported ratio matches the closed-form demand",
        abs(ratio * budget[joint - 1] - expected_demand) < 1e-6,
        f"{ratio * budget[joint - 1]:.4f} vs {expected_demand:.4f} rad/s",
    )

    # The property the clamp is only useful for: applying `scale` must land the
    # demand ON the budget. Anything else (a fixed backoff, an iterative shrink)
    # would either still violate it or throttle the arm more than needed.
    _, ratio_after, _ = joint_demand_scale(
        J, [0, 0, 0], [0, 0, rot_step * scale], DT, JOINT_VEL_LIMITS
    )
    check(
        "scaling the step lands the demand exactly at budget",
        abs(ratio_after - 1.0) < 1e-9,
        f"post-scale ratio={ratio_after:.9f}",
    )

    # Linearity is what lets one division replace an iteration. If this ever
    # stops holding (e.g. someone adds damping to the pseudo-inverse), the
    # single-shot scale becomes wrong and this is the test that says so.
    _, ratio_half, _ = joint_demand_scale(
        J, [0, 0, 0], [0, 0, rot_step * 0.5], DT, JOINT_VEL_LIMITS
    )
    check(
        "demand is linear in the step",
        abs(ratio_half - ratio * 0.5) < 1e-9,
        f"half-step ratio={ratio_half:.6f} vs {ratio * 0.5:.6f}",
    )

    # Translation and rotation must both be seen. A clamp that only looked at
    # one half would have passed LOG-036's trip, which was rotation-driven.
    scale_t, _, _ = joint_demand_scale(
        _ill_conditioned_jacobian(0.01), [0, 0, 0], [0, 0, rot_step], DT, JOINT_VEL_LIMITS
    )
    J_t = _well_conditioned_jacobian()
    J_t[0, 0] = 0.001
    scale_x, _, joint_x = joint_demand_scale(
        J_t, [step, 0, 0], [0, 0, 0], DT, JOINT_VEL_LIMITS
    )
    check(
        "translation-driven demand is clamped too",
        scale_x < 1.0 and joint_x == 1,
        f"scale={scale_x:.4f} worst=j{joint_x}",
    )
    check("rotation-driven demand is clamped", scale_t < 1.0, f"scale={scale_t:.4f}")


def test_threshold() -> None:
    print("\n=== guard: |dq| threshold placement ===")
    thr = guard_max_dq_rad_s()
    check(
        "default is the documented value",
        thr == GUARD_MAX_DQ_RAD_S_DEFAULT,
        f"{thr} rad/s",
    )
    # The threshold is only useful if it separates the two measured populations.
    # Values are from this arm: the rotation ladder's worst peak, the full
    # allowed twist at hover, and the two trips.
    check(
        "above known-normal (0.81 rad/s, full twist at hover)",
        thr > 0.81,
        f"{thr} > 0.81",
    )
    check("above the rotation ladder's worst peak (0.379)", thr > 0.379)
    check("below both real trips (2.03 / 2.69 rad/s)", thr < 2.03, f"{thr} < 2.03")
    check(
        "below Panda's own j1-j4 limit (2.075 rad/s)",
        thr < float(JOINT_VEL_LIMITS[0]),
    )


def test_guard_wiring() -> None:
    """Source-level checks for two orderings that are silent when wrong."""
    print("\n=== guard: wiring ===")
    from franky_ext.controller_extended import FrankyControllerExtended

    brake_src = inspect.getsource(FrankyControllerExtended._brake)
    check(
        "_brake gives 'dq' the lag order (freeze-then-stop)",
        'kind in ("lag", "dq")' in brake_src,
    )

    eval_src = inspect.getsource(FrankyControllerExtended._evaluate_guard)
    dq_at = eval_src.find('"dq"')
    lag_at = eval_src.find('"lag"')
    check(
        "_evaluate_guard checks dq BEFORE lag",
        0 <= dq_at < lag_at,
        "otherwise a joint runaway is reported as a tracking failure",
    )

    from franky_ext.franky_single_franka_env import FrankySingleFrankaEnvMixin

    slew_src = inspect.getsource(FrankySingleFrankaEnvMixin._clamp_step_slew)
    check(
        "_clamp_step_slew applies the joint clamp",
        "_clamp_joint_demand" in slew_src,
    )
    # It has to run last, on the already-Cartesian-clamped pose, or the two
    # clamps stop composing.
    check(
        "joint clamp runs after the Cartesian clamps",
        slew_src.find("_clamp_joint_demand") > slew_src.find("max_action_scale_rot"),
    )

    # The interpolation path skips the position clamp by design, so it needs the
    # duration-based bound instead -- and it needs it for real: 2 cm/s, the
    # *interpolation* speed, demands 1.73x budget at the LOG-036 trip pose.
    interp_src = inspect.getsource(FrankySingleFrankaEnvMixin._interpolate_move)
    check(
        "_interpolate_move bounds joint demand too",
        "_stretch_interp_for_joint_demand" in interp_src,
    )
    check(
        "the interp bound is applied to the capped duration, not the request",
        interp_src.find("_stretch_interp_for_joint_demand")
        > interp_src.find("interp_duration_s("),
    )


def test_real(robot_ip: str) -> None:
    print("\n=== real Jacobians (READ-ONLY, no motion commanded) ===")
    import franky

    robot = franky.Robot(robot_ip)
    state = robot.state
    model = robot.model
    frame = franky.Frame.EndEffector
    F_T_EE, EE_T_K = state.F_T_EE, state.EE_T_K
    print(f"  connected to {robot_ip}")

    # The worst legal step: every Cartesian cap saturated on all three axes.
    step = step_speed_m_s() * DT / np.sqrt(3.0)
    rot = step_rot_speed_rad_s() * DT / np.sqrt(3.0)
    d_xyz = np.array([step, step, step])
    d_rot = np.array([rot, rot, rot])

    def J_at(q):
        return np.asarray(
            model.zero_jacobian(frame, np.asarray(q), F_T_EE, EE_T_K), dtype=np.float64
        ).reshape(6, 7)

    scale_h, ratio_h, joint_h = joint_demand_scale(
        J_at(HOVER_Q), d_xyz, d_rot, DT, JOINT_VEL_LIMITS
    )
    check(
        "hover pose: full-speed step passes unclamped",
        scale_h == 1.0,
        f"worst j{joint_h} at {ratio_h:.3f}x budget",
    )

    scale_t, ratio_t, joint_t = joint_demand_scale(
        J_at(TRIP_Q), d_xyz, d_rot, DT, JOINT_VEL_LIMITS
    )
    check(
        "LOG-036 trip pose: the same step IS clamped",
        scale_t < 1.0,
        f"worst j{joint_t} at {ratio_t:.2f}x budget -> step x{scale_t:.4f}",
    )
    check(
        "trip pose worst joint is j4 (LOG-037 measured 9.64 rad/s there)",
        joint_t == 4,
        f"got j{joint_t}",
    )
    # The two poses differ by 14.6x in sigma_min; the clamp must reflect that
    # rather than treating them alike. If this ratio collapses toward 1, the
    # Jacobian being read is not the one the arm is using.
    print(
        f"  hover ratio {ratio_h:.3f}x vs trip ratio {ratio_t:.3f}x "
        f"= {ratio_t / max(ratio_h, 1e-9):.1f}x apart"
    )
    check("the two poses are treated very differently", ratio_t / max(ratio_h, 1e-9) > 5.0)

    # A clamped step must still be a real step, not a freeze. If this goes to
    # zero the arm cannot escape the bad pose and the episode stalls instead.
    print(
        f"  clamped step at trip pose: {np.linalg.norm(d_xyz) * scale_t * 1000:.2f} mm, "
        f"{np.linalg.norm(d_rot) * scale_t * 1000:.2f} mrad per cycle"
    )
    check("clamped step is non-degenerate (>0.1 mm/cycle)",
          np.linalg.norm(d_xyz) * scale_t > 1e-4)

    # The interpolation path, i.e. reset -- unprotected until the duration
    # stretch, and over budget at this pose even at the slower interp speed.
    i_xyz = np.full(3, interp_speed_m_s() * DT / np.sqrt(3.0))
    i_rot = np.full(3, interp_speed_rad_s() * DT / np.sqrt(3.0))
    scale_i, ratio_i, joint_i = joint_demand_scale(
        J_at(TRIP_Q), i_xyz, i_rot, DT, JOINT_VEL_LIMITS
    )
    check(
        "trip pose: even the INTERP speed is over budget (so the stretch is needed)",
        scale_i < 1.0,
        f"worst j{joint_i} at {ratio_i:.2f}x -> stretch x{1.0 / scale_i:.2f}",
    )
    # A typical reset hop, to confirm the stretch stays well inside the duration
    # cap rather than turning reset into a refusal.
    hop_m = 0.05
    hop_s = hop_m / interp_speed_m_s()
    stretched = hop_s / max(
        joint_demand_scale(
            J_at(TRIP_Q), [0, 0, hop_m], [0, 0, 0], hop_s, JOINT_VEL_LIMITS
        )[0],
        1e-9,
    )
    check(
        "a 5 cm reset hop stretches to a sane duration, not a refusal",
        stretched <= 20.0,
        f"{hop_s:.2f}s -> {stretched:.2f}s (cap 20s)",
    )
    scale_h_i, _, _ = joint_demand_scale(
        J_at(HOVER_Q), i_xyz, i_rot, DT, JOINT_VEL_LIMITS
    )
    check(
        "hover pose: interp speed needs no stretch",
        scale_h_i == 1.0,
    )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--robot",
        action="store_true",
        help="also evaluate real Jacobians (connects read-only; commands no motion)",
    )
    p.add_argument("--robot-ip", default=os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2"))
    args = p.parse_args()

    print(
        f"T19 acceptance: demand fraction={joint_vel_demand_fraction():.2f} of "
        f"{np.round(JOINT_VEL_LIMITS, 3).tolist()} rad/s, "
        f"guard |dq| <= {guard_max_dq_rad_s():.3f} rad/s, "
        f"caps {step_speed_m_s():.3f} m/s / {step_rot_speed_rad_s():.3f} rad/s"
    )
    test_math()
    test_threshold()
    test_guard_wiring()
    if args.robot:
        test_real(args.robot_ip)
    else:
        print("\n(skipping real-Jacobian checks; pass --robot to run them)")

    print()
    if _failures:
        print(f"FAILED {len(_failures)}: {', '.join(_failures)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
