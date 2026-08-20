"""TCP pose probe helpers (subprocess franky, releases FCI before env connect)."""

from __future__ import annotations

import json
import os
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

_PROBE_SCRIPT = r"""
import json, os, sys
import franky
import numpy as np
from scipy.spatial.transform import Rotation as R

ip = os.environ["FRANKA_ROBOT_IP"]
robot = franky.Robot(ip)
state = robot.state
affine = state.O_T_EE
xyz = np.asarray(affine.translation, dtype=np.float64)
quat = np.asarray(affine.quaternion, dtype=np.float64)
euler = R.from_quat(quat).as_euler("xyz")

def _opt(obj, name):
    try:
        v = getattr(obj, name)
    except Exception:
        return None
    return v

info = {
    "pose": np.concatenate([xyz, euler]).tolist(),
    "robot_mode": str(state.robot_mode),
    "has_errors": bool(robot.has_errors),
    "q": np.asarray(robot.current_joint_positions, dtype=np.float64).tolist(),
}
rate = _opt(state, "control_command_success_rate")
info["cmd_success_rate"] = None if rate is None else float(rate)
dq = _opt(robot, "current_joint_velocities")
info["joint_vel_norm"] = (
    None if dq is None else float(np.linalg.norm(np.asarray(dq, dtype=np.float64)))
)

# Gripper is a separate libfranka connection; reading width is read-only and lets
# a caller refuse a closed-gripper task with an empty hand BEFORE paying ~30 s of
# Ray startup. Failure to connect is not fatal here.
#
# Deliberately does NOT decide "holding" here, and does NOT trust libfranka's
# is_grasped: with any usable epsilon that flag cannot distinguish a held cube
# from an empty hand closed on air (round-2 audit finding 5 -- this was the one
# place in the stack still trusting it after franka_libfranka_gripper.py moved to
# a width-only rule). is_grasped_raw is kept only as an ADVISORY field so the two
# can be compared over a few real grips; "holding" is decided in the PARENT
# process (probe_robot_state) from gripper_width alone, via the same
# motion_limits.holding_from_width the controller-side gripper uses.
info["gripper_width"] = None
info["is_grasped_raw"] = None
try:
    g = franky.Gripper(ip)
    w = _opt(g, "width")
    info["gripper_width"] = None if w is None else float(w)
    grasped = _opt(g, "is_grasped")
    if callable(grasped):
        grasped = grasped()
    if grasped is not None:
        info["is_grasped_raw"] = bool(grasped)
except Exception as exc:
    info["gripper_error"] = f"{type(exc).__name__}: {exc}"

print(json.dumps(info))
"""

# Motion-ready mode. Guiding/UserStopped/Reflex all reject robot.move().
_READY_MODE = "RobotMode.Idle"

MODE_HINT = {
    "RobotMode.UserStopped": (
        "the user-stop button on the Franka enabling device is pressed. "
        "libfranka rejects every motion with 'Move command rejected: command "
        "not possible in the current mode (\"User stopped\")', while state "
        "reads and gripper commands keep working -- so reset looks like it ran "
        "but the arm never moves (dz=0.0000). Pull the user-stop button up, "
        "confirm Desk shows joints unlocked and FCI activated, then retry."
    ),
    "RobotMode.Guiding": (
        "the enabling device is held in guiding mode; release it so FCI can "
        "command motion."
    ),
    "RobotMode.Reflex": (
        "a reflex (collision / limit) is latched; recover in Desk or call "
        "recover_from_errors(), then retry."
    ),
}


def probe_robot_state(robot_ip: str, *, python: str | None = None) -> dict:
    """Read TCP pose, ``robot_mode`` and error flag in a child process.

    Runs in a subprocess so the FCI control connection is released before the
    env's own controller connects.
    
    为什么这样做:
    FCI 是独占的:Franka 的控制接口（FCI，走 1337 端口）同一时刻只允许一个 libfranka 客户端连接。franky.Robot("172.16.0.2") 的构造器一执行就建立这条连接，并且一直持有到这个对象被销毁为止。
    而 robot 这个对象的析构时机由 Python 垃圾回收决定，不确定。即使你在函数里 del robot，也没法保证 socket 在 gym.make 之前真的关了。结果就是间歇性的 Couldn't connect / FCI 被占——而且报错的现场离真正的占用者隔了好几层，很难查。
    但如果用 subprocess.run 起一个新的 python -c ... 子进程，子进程里 franky.Robot(ip) → 读状态 → 打印 JSON → 进程退出。  进程退出是操作系统级别的确定性事件：进程一死，它持有的 socket 立刻被内核关闭，FCI 连接必然释放。subprocess.run 又是阻塞等待子进程结束的，所以等 probe_robot_state 返回时，可以保证 FCI 已经空了，后面 env 的控制器来连接不会撞车。

    这句注释存在的意义，是防止以后有人图省事把子进程改成进程内调用——改动看起来无害（少起一次 python），实际引入的是一个时序不确定的 FCI 占用 bug。
    """
    env = {**os.environ, "FRANKA_ROBOT_IP": robot_ip}
    proc = subprocess.run(
        [python or sys.executable, "-c", _PROBE_SCRIPT],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            f"TCP probe failed (exit {proc.returncode}):\n{proc.stderr.strip()}"
        )
    line = proc.stdout.strip().splitlines()[-1]
    info = json.loads(line)
    pose = info["pose"]
    if len(pose) != 6:
        raise ValueError(f"expected 6-D pose, got {pose!r}")
    info["pose"] = [float(x) for x in pose]
    # Decide "holding" here, in the parent, from measured width alone -- not from
    # the child's advisory is_grasped_raw. See _PROBE_SCRIPT's comment and
    # motion_limits.holding_from_width (round-2 audit finding 5).
    from franky_ext.motion_limits import holding_from_width

    info["gripper_holding"] = holding_from_width(info.get("gripper_width"))
    return info


def probe_tcp_pose_euler(robot_ip: str, *, python: str | None = None) -> list[float]:
    """Read current TCP ``[x,y,z, roll,pitch,yaw]`` in a child process."""
    return probe_robot_state(robot_ip, python=python)["pose"]


def describe_robot_mode(mode: str) -> str:
    """Human-readable reason why ``mode`` cannot execute motions."""
    hint = MODE_HINT.get(mode, "FCI only executes motions in RobotMode.Idle.")
    return f"robot_mode={mode}: {hint}"


def require_motion_ready(mode: str) -> None:
    """Raise before commanding the arm if ``mode`` rejects motion."""
    if mode != _READY_MODE:
        raise RuntimeError(describe_robot_mode(mode))


#: Roll/pitch half-width hardcoded by ``PegInsertionConfig.__post_init__``, radians.
#: Mirrored here (not imported) so the read-only ``connect`` path -- which never
#: constructs an env -- can print the box that will actually be enforced.
PEG_RPY_HALF_WIDTH_RAD = 0.01


def effective_ee_pose_limits(
    target: list[float] | np.ndarray,
    *,
    clip_xy: float,
    z_low: float,
    z_high: float,
    clip_rz: float,
) -> tuple[list[float], list[float]]:
    """The ``ee_pose_limit_{min,max}`` the env will really use.

    ``PegInsertionConfig.__post_init__`` recomputes both from ``target_ee_pose``
    and the ``clip_*`` ranges, **overwriting anything passed in ``override_cfg``**.
    The xyz halves happen to match what a probe-derived box would give, but the
    orientation halves do not: roll and pitch get a hardcoded ``+/-0.01 rad``
    while only yaw gets ``clip_rz_range``.

    A generic "box centred on the pose with one rpy margin" helper therefore
    reports an orientation window 35x wider than the one that is enforced -- and
    printing that to an operator during pre-flight is worse than printing nothing,
    because it reads like verification. This function mirrors the real derivation
    so the printout and the enforcement agree.
    """
    t = np.asarray(target, dtype=np.float64).reshape(-1)
    lo = [
        float(t[0]) - float(clip_xy),
        float(t[1]) - float(clip_xy),
        float(t[2]) - float(z_low),
        float(t[3]) - PEG_RPY_HALF_WIDTH_RAD,
        float(t[4]) - PEG_RPY_HALF_WIDTH_RAD,
        float(t[5]) - float(clip_rz),
    ]
    hi = [
        float(t[0]) + float(clip_xy),
        float(t[1]) + float(clip_xy),
        float(t[2]) + float(z_high),
        float(t[3]) + PEG_RPY_HALF_WIDTH_RAD,
        float(t[4]) + PEG_RPY_HALF_WIDTH_RAD,
        float(t[5]) + float(clip_rz),
    ]
    return lo, hi


def check_start_pose(
    probed: list[float] | np.ndarray,
    target: list[float] | np.ndarray,
    *,
    clip_xy: float,
    z_low: float,
    z_high: float,
    min_clearance_m: float = 0.0,
    clip_rz: float | None = None,
) -> list[str]:
    """Violations of "the arm is where reset assumes it is". Empty list == OK.

    LOG-019 started ``reset`` with the TCP 6.6 mm *below* the calibrated contact
    point -- the cube pressed into the mark -- because the manual's "guide the arm
    a few cm above the mark" step was skipped and no code checked it. Pure
    arithmetic so the dummy regression covers it.

    Args:
        probed: current TCP, at least ``[x, y, z]``.
        target: H1 contact pose, at least ``[x, y, z]``.
        clip_xy: safety-box xy half-width, m.
        z_low: allowed drop below the contact point, m (box floor).
        z_high: hover height above the contact point, m (box top).
        min_clearance_m: require the start to be at least this far *above* the
            contact point. 0 means "not below contact".

    Returns:
        One string per violation, phrased as an instruction to the operator.
    """
    p = np.asarray(probed, dtype=np.float64).reshape(-1)
    t = np.asarray(target, dtype=np.float64).reshape(-1)
    problems: list[str] = []

    floor = float(t[2]) - float(z_low)
    need_z = float(t[2]) + float(min_clearance_m)
    top = float(t[2]) + float(z_high)

    if float(p[2]) < need_z:
        problems.append(
            f"start z={p[2]:.4f} is below the required {need_z:.4f} "
            f"(contact z={t[2]:.4f}"
            + (f" + {min_clearance_m:.3f} clearance" if min_clearance_m else "")
            + f"; box floor {floor:.4f}). The cube is on/into the mark: guide the "
            "arm a few cm above the mark before reset."
        )
    # Only 1 cm of slop above the box top, and that number is not arbitrary: the
    # controller's fence sits at ``box_top + guard_margin + reset_z_lift_m``, and
    # ``go_to_rest`` lifts ``reset_z_lift_m`` from wherever the arm *starts*. Every
    # centimetre allowed here is a centimetre eaten out of the fence's tracking
    # margin, so with the defaults (top 0.3119, margin 0.05, lift 0.03 -> fence
    # 0.3919) this keeps roughly 4 cm of headroom for legitimate lag.
    if float(p[2]) > top + 0.01:
        problems.append(
            f"start z={p[2]:.4f} is more than 1 cm above the box top {top:.4f}; "
            "reset would travel down a long way under impedance, and the lift would "
            "eat the motion guard's headroom. Guide the arm to 3-5 cm above the mark."
        )
    xy_err = float(np.linalg.norm(p[:2] - t[:2]))
    if xy_err > float(clip_xy):
        problems.append(
            f"start is {xy_err:.4f} m from the mark in xy, outside the "
            f"{clip_xy:.3f} m box. Guide the arm over the mark before reset."
        )

    # Orientation, if the caller knows the yaw window. This matters because the
    # very first thing ``go_to_rest`` commands is "stay exactly where you are", so
    # a wrist far from the target orientation trips the controller's orientation
    # fence with a motionless arm -- an abort that reads like a runaway. Better to
    # refuse here, before Ray starts, with an instruction the operator can act on.
    if clip_rz is not None and p.size >= 6 and t.size >= 6:
        from franky_ext.motion_limits import orientation_fence_rad, quat_angle_rad

        fence = orientation_fence_rad(
            PEG_RPY_HALF_WIDTH_RAD, PEG_RPY_HALF_WIDTH_RAD, float(clip_rz)
        )
        ang = quat_angle_rad(
            R.from_euler("xyz", p[3:6]).as_quat(),
            R.from_euler("xyz", t[3:6]).as_quat(),
        )
        if ang > fence * 0.8:
            problems.append(
                f"start orientation is {ang:.3f} rad ({ang * 180.0 / np.pi:.0f} deg) "
                f"from the target, against a {fence:.3f} rad guard fence. Re-orient "
                "the wrist with the enabling device so the cube face is flat on the "
                "mark, then re-read the pose."
            )
    return problems


def ee_pose_limits_from_probe(
    rest_pose: list[float] | np.ndarray,
    *,
    xyz_margin: float,
    rpy_margin: float,
) -> tuple[list[float], list[float]]:
    """Build 6-D ``ee_pose_limit_{min,max}`` centered on the probed TCP pose."""
    xyz = np.asarray(rest_pose[:3], dtype=np.float64)
    rpy = np.asarray(rest_pose[3:6], dtype=np.float64)
    xyz_m = np.full(3, xyz_margin, dtype=np.float64)
    rpy_m = np.full(3, rpy_margin, dtype=np.float64)
    lim_min = np.concatenate([xyz - xyz_m, rpy - rpy_m]).tolist()
    lim_max = np.concatenate([xyz + xyz_m, rpy + rpy_m]).tolist()
    return lim_min, lim_max


def peg_target_and_reset_from_probe(
    probed_pose: list[float] | np.ndarray,
    *,
    z_offset_m: float,
) -> tuple[list[float], list[float]]:
    """Peg task poses: ``target`` = probed TCP, ``reset`` = target + z hover."""
    target = [float(x) for x in probed_pose[:6]]
    reset = target.copy()
    reset[2] += float(z_offset_m)
    return target, reset
