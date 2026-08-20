#!/usr/bin/env python3
"""Read-only Jacobian conditioning report for a Franka Panda configuration.

Answers the one question LOG-036 left open (T19): when the arm is at the
configuration where training tripped, how much joint velocity does a *bounded*
Cartesian command actually demand? "Near-singular" was asserted there from the
joint angles alone; this script measures it.

Commands NO motion. It connects only to read ``F_T_EE`` / ``EE_T_K`` and the
kinematic model, then evaluates the model at whatever ``q`` it is given --
``franky.Model.zero_jacobian`` has a ``q``-taking overload, so configurations
the arm is NOT currently in (e.g. the hover pose) can be evaluated without
moving there. Run inside the franky container, after
``source b/x/configs/setup_before_ray_5090.sh``.

Reported per configuration:

* singular values of ``J`` (6x7, base frame), condition number, and
  ``sigma_min``. ``1/sigma_min`` is the worst-case amplification from a unit
  Cartesian twist to joint velocity, i.e. the exact quantity T19 is about.
* the same for the translation-only (``J[:3]``) and rotation-only (``J[3:]``)
  blocks, because this task's authority caps are specified separately for the
  two and a configuration can be well conditioned for one and not the other.
* the minimum-norm ``|dq| = |J^+ v|`` for the twist the policy is *allowed* to
  command (``--vel`` / ``--omega``, defaulting to this task's caps). Compare
  against the ``|dq|`` a real incident recorded: if the allowed twist alone
  explains it, the fix belongs in the command path; if it does not, something
  else (nullspace self-motion, joint-limit repulsion, contact) is the source
  and clamping the command path will not remove it.
* distance to each joint limit, flagged against franky's joint-limit repulsion
  activation distance -- repulsion torque is a second candidate source of joint
  velocity that no Cartesian-space cap can see.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from franky_ext.controller_extended import (  # noqa: E402
    _JOINT_LIMIT_ACTIVATION_RAD,
)
from franky_ext.motion_limits import (  # noqa: E402
    jacobian_conditioning,
    sigma_min_warn,
    step_rot_speed_rad_s,
    step_speed_m_s,
)

# Read-only import of upstream's joint limits, the same vectors the controller
# hands franky for soft joint-limit repulsion.
from rlinf.envs.realworld.franka.franky_controller import (  # noqa: E402
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
)

# Configurations worth keeping around, all recorded from this arm.
NAMED_CONFIGS: dict[str, tuple[list[float], str]] = {
    "trip_log036": (
        [0.0330, 1.1360, -0.0341, -0.4233, 0.0853, 1.4967, 0.5404],
        "where LOG-036 training tripped (|dq|=2.03rad/s, lag=0.0512m)",
    ),
    "hover_log036": (
        [0.2304, 0.5462, -0.2855, -1.2968, 0.1800, 1.7952, 1.0502],
        "hover pose: where 2.4b and the rotation ladder were measured (all clean)",
    ),
}


def _fmt(v: np.ndarray, prec: int = 4) -> str:
    return "[" + ", ".join(f"{x:.{prec}f}" for x in np.asarray(v).reshape(-1)) + "]"


def _block_report(name: str, J: np.ndarray) -> dict[str, float]:
    """Print one block's conditioning. The numbers come from ``motion_limits``.

    Deliberately not its own SVD: the online self-check on the reset path
    (LOG-040 T22) warns against ``sigma_min``, and an operator who then runs this
    script to see the number for themselves must get the same one. Two
    implementations of "compute sigma_min" is two things to keep in agreement.
    """
    cond = jacobian_conditioning(J)
    if cond is None:
        print(f"  {name:10s} no usable jacobian (shape {np.asarray(J).shape})")
        return {"sigma_min": float("nan"), "cond": float("nan")}
    print(f"  {name:10s} sigma={_fmt(cond['sigma'])}")
    if cond["sigma_min"] <= 1e-12:
        print(f"  {name:10s} SINGULAR (sigma_min={cond['sigma_min']:.2e})")
        return {"sigma_min": cond["sigma_min"], "cond": float("inf")}
    print(
        f"  {name:10s} sigma_min={cond['sigma_min']:.4f} "
        f"sigma_max={cond['sigma_max']:.4f} cond={cond['cond']:8.2f}  "
        f"worst-case |dq| per unit twist = {1.0 / cond['sigma_min']:7.2f}"
    )
    return {"sigma_min": cond["sigma_min"], "cond": cond["cond"]}


def report(model, frame, F_T_EE, EE_T_K, q: np.ndarray, label: str, args) -> None:
    print(f"\n=== {label} ===")
    print(f"q = {_fmt(q)}")
    J = np.asarray(model.zero_jacobian(frame, q, F_T_EE, EE_T_K), dtype=np.float64)
    J = J.reshape(6, 7)

    full = _block_report("full 6x7", J)
    _block_report("trans 3x7", J[:3])
    _block_report("rot   3x7", J[3:])

    manip = float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))
    print(f"  manipulability sqrt(det(J J^T)) = {manip:.6f}")

    # The same verdict the env logs on the reset path (LOG-040 T22), printed here
    # so "what does the training run think of this pose" needs no arithmetic.
    warn_at = sigma_min_warn()
    if np.isfinite(full["sigma_min"]) and full["sigma_min"] < warn_at:
        print(
            f"  VERDICT: ILL-CONDITIONED -- sigma_min={full['sigma_min']:.4f} < "
            f"{warn_at:.4f}; the env would warn here and ask for the elbow to be "
            "guided back (dmo_place_2 §S3.12)"
        )
    else:
        print(
            f"  VERDICT: ok -- sigma_min={full['sigma_min']:.4f} >= {warn_at:.4f} "
            "(the env's reset-path self-check would pass here)"
        )

    # What the policy is allowed to command, turned into a joint-velocity demand.
    v = np.zeros(6)
    v[:3] = args.vel / np.sqrt(3.0)  # per-axis cap, worst case is 3-axis combined
    v[3:] = args.omega / np.sqrt(3.0)
    dq = np.linalg.pinv(J) @ v
    print(
        f"  allowed twist |v|={np.linalg.norm(v[:3]):.4f}m/s "
        f"|w|={np.linalg.norm(v[3:]):.4f}rad/s  ->  min-norm |dq|={np.linalg.norm(dq):.4f}rad/s"
    )
    print(f"    per-joint dq = {_fmt(dq)}")

    # Worst-case direction: drive along the least-controllable singular vector.
    U, S, _Vt = np.linalg.svd(J)
    worst_dir = U[:, -1]
    scale = np.linalg.norm(v)
    dq_worst = np.linalg.pinv(J) @ (worst_dir * scale)
    print(
        f"  worst-direction twist of the same magnitude ({scale:.4f}) "
        f"->  |dq|={np.linalg.norm(dq_worst):.4f}rad/s"
    )

    lower = np.asarray(JOINT_LIMITS_LOWER, dtype=np.float64)
    upper = np.asarray(JOINT_LIMITS_UPPER, dtype=np.float64)
    d_low = q - lower
    d_high = upper - q
    near = []
    for i in range(7):
        d = min(d_low[i], d_high[i])
        tag = "  <-- inside repulsion band" if d < args.activation else ""
        if tag:
            near.append(i + 1)
        print(
            f"  joint {i + 1}: q={q[i]:+.4f}  to_lower={d_low[i]:.4f}  "
            f"to_upper={d_high[i]:.4f}{tag}"
        )
    if near:
        print(
            f"  !! joints {near} are within the {args.activation:.3f}rad joint-limit "
            "repulsion band: repulsion torque is active and produces joint velocity "
            "that NO Cartesian-space cap can bound"
        )
    else:
        print(
            f"  no joint within the {args.activation:.3f}rad repulsion band "
            "(repulsion is not a contributor at this configuration)"
        )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--config",
        action="append",
        default=[],
        help=f"named configuration to evaluate (repeatable). Known: {list(NAMED_CONFIGS)}",
    )
    p.add_argument("--q", help="explicit 7 comma-separated joint angles, rad")
    p.add_argument(
        "--live", action="store_true", help="also evaluate the arm's current q"
    )
    p.add_argument(
        "--vel",
        type=float,
        default=None,
        help="per-axis translation cap, m/s (default: this task's step speed cap)",
    )
    p.add_argument(
        "--omega",
        type=float,
        default=None,
        help="per-axis rotation cap, rad/s (default: this task's step rot speed cap)",
    )
    p.add_argument(
        "--activation",
        type=float,
        default=None,
        help="joint-limit repulsion activation distance, rad (default: controller's)",
    )
    p.add_argument("--robot-ip", default=os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2"))
    args = p.parse_args()

    import franky

    if args.vel is None:
        args.vel = step_speed_m_s()
    if args.omega is None:
        args.omega = step_rot_speed_rad_s()
    if args.activation is None:
        args.activation = _JOINT_LIMIT_ACTIVATION_RAD

    robot = franky.Robot(args.robot_ip)
    state = robot.state
    model = robot.model
    frame = franky.Frame.EndEffector
    F_T_EE = state.F_T_EE
    EE_T_K = state.EE_T_K

    print(f"connected to {args.robot_ip} (READ-ONLY; no motion is commanded)")
    print(
        f"caps used for the demand calculation: per-axis {args.vel:.4f}m/s / "
        f"{args.omega:.4f}rad/s; joint-limit repulsion activation {args.activation:.3f}rad"
    )

    todo = list(args.config) or (["trip_log036", "hover_log036"] if not args.q else [])
    for name in todo:
        if name not in NAMED_CONFIGS:
            print(f"unknown --config {name!r}; known: {list(NAMED_CONFIGS)}")
            return 2
        q, note = NAMED_CONFIGS[name]
        report(
            model, frame, F_T_EE, EE_T_K, np.asarray(q), f"{name} -- {note}", args
        )

    if args.q:
        q = np.asarray([float(x) for x in args.q.split(",")], dtype=np.float64)
        if q.size != 7:
            print(f"--q needs 7 values, got {q.size}")
            return 2
        report(model, frame, F_T_EE, EE_T_K, q, "explicit --q", args)

    if args.live:
        report(
            model,
            frame,
            F_T_EE,
            EE_T_K,
            np.asarray(state.q, dtype=np.float64),
            "live (arm's current configuration)",
            args,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
