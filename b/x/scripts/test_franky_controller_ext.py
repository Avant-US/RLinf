#!/usr/bin/env python3
"""Interactive Franky REPL for this 5090 (native Franka Hand).

Official ``python -m toolkits.realworld_check.test_franky_controller`` launches
upstream ``FrankyController``, which raises NotImplementedError for
``gripper_type=franka``. This script uses ``FrankyControllerExtended`` and
``FrankaLibfrankaGripper`` instead. Commands match the official REPL
(``open`` / ``close`` / ``getpos_euler`` / ``q``).

Does **not** auto-home or auto-open. Run inside the franky container after
``source b/x/configs/setup_before_ray_5090.sh``.
"""

from __future__ import annotations

import os
import sys
import time

import ray

if not ray.is_initialized():
    try:
        ray.init(address="auto", log_to_driver=False, logging_level="ERROR")
    except Exception:
        ray.init(log_to_driver=False, logging_level="ERROR")

import numpy as np
from scipy.spatial.transform import Rotation as R

# Four "..": the first only strips the filename (LOG-023 finding 4).
REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

from franky_ext.controller_extended import FrankyControllerExtended
from franky_ext.motion_limits import describe_authority

HOME_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

#: Per-call joint step cap for ``nudge``. ``stream`` already capped its *total*
#: displacement at 0.5 rad while ``nudge`` had no cap at all, so a mistyped
#: ``nudge 2 1.5`` would swing the shoulder 86 degrees in one blocking motion.
MAX_NUDGE_RAD = 0.2


def _print_help() -> None:
    print(
        "commands: q | getpos | getpos_euler | getjoint | mode | health | "
        "home yes | nudge <i> <d> | stream <i> <d> <n> | hold <secs> | "
        "open [yes] | close [N] | stop | grip <0-255> | impedance <k_trans k_rot>"
    )
    print("H1 标定: open → 放方块 → close → 引导贴住标记 → getpos_euler → q")
    print("close 为轻力抓取（默认约 20 N，上限 40 N），不是死夹。异常立刻 stop 或急停。")
    print("标定过程不要敲 home。home 会从当前任意位形横扫到工厂关节位，需写成 `home yes`。")
    print(f"nudge 单次上限 {MAX_NUDGE_RAD} rad；stream 总位移上限 0.5 rad。")
    print("夹着方块时 open 会让方块掉下来，需写成 `open yes`。")


def main() -> int:
    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    gripper_type = os.environ.get("FRANKA_GRIPPER_TYPE", "franka")
    gripper_connection = os.environ.get("FRANKA_GRIPPER_PORT")

    print(
        f"FrankyControllerExtended REPL  robot={robot_ip} gripper={gripper_type}"
    )
    if gripper_type.lower() != "franka":
        print(
            "WARNING: this machine uses the native Franka Hand; "
            "set FRANKA_GRIPPER_TYPE=franka"
        )

    controller = FrankyControllerExtended.launch_controller(
        robot_ip=robot_ip,
        gripper_type=gripper_type,
        gripper_connection=gripper_connection,
    )

    start_time = time.time()
    while not controller.is_robot_up().wait()[0]:
        time.sleep(0.5)
        if time.time() - start_time > 30:
            print(f"Waited {time.time() - start_time:.1f}s for Franka to be ready")
            break

    print(f"Connected to Franka at {robot_ip}")
    _print_help()

    while True:
        try:
            cmd_str = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd_str:
            continue

        parts = cmd_str.split()
        cmd = parts[0].lower()

        try:
            if cmd == "q":
                break
            elif cmd == "help":
                _print_help()
            elif cmd == "getpos":
                pose = controller.get_state().wait()[0].tcp_pose
                print(pose)
            elif cmd == "getpos_euler":
                pose = controller.get_state().wait()[0].tcp_pose
                euler = R.from_quat(pose[3:].copy()).as_euler("xyz")
                print(np.concatenate([pose[:3], euler]))
            elif cmd == "getjoint":
                state = controller.get_state().wait()[0]
                print(state.arm_joint_position)
            elif cmd == "mode" or cmd == "health":
                print(controller.motion_health().wait()[0])
            elif cmd == "home":
                # A blocking JointMotion from wherever the arm happens to be to
                # the factory pose. Near the table, holding a cube, that is a
                # long unplanned sweep -- and the calibration procedure explicitly
                # says not to use it, so require the intent to be spelled out.
                if len(parts) != 2 or parts[1].lower() != "yes":
                    print(
                        "home sweeps from the CURRENT pose to the factory joint "
                        "position in one blocking motion. It will drag a grasped "
                        "cube across the table and it invalidates the H1 start "
                        "pose. Type `home yes` if that is really what you want."
                    )
                    continue
                print(f"Resetting to home: {HOME_JOINTS}")
                controller.reset_joint(HOME_JOINTS).wait()
                print("Home reached")
            elif cmd == "nudge":
                if len(parts) != 3:
                    print("usage: nudge <joint_index 1..7> <delta_rad>")
                    continue
                idx = int(parts[1]) - 1
                delta = float(parts[2])
                assert 0 <= idx < 7, "joint index must be 1..7"
                if abs(delta) > MAX_NUDGE_RAD:
                    print(
                        f"refusing nudge: |{delta:.4f}| rad > {MAX_NUDGE_RAD} rad "
                        f"per-call cap (use several smaller nudges, or `stream`)"
                    )
                    continue
                current = controller.get_state().wait()[0].arm_joint_position
                target = current.copy()
                target[idx] += delta
                print(f"move_joints: {target}")
                controller.move_joints(target).wait()
            elif cmd == "stream":
                if len(parts) != 4:
                    print("usage: stream <joint_index 1..7> <delta_per_tick> <n_ticks>")
                    continue
                idx = int(parts[1]) - 1
                delta = float(parts[2])
                n = int(parts[3])
                assert 0 <= idx < 7, "joint index must be 1..7"
                total = abs(delta) * n
                if total > 0.5:
                    print(
                        f"refusing stream: total displacement "
                        f"{total:.3f} rad > 0.5 rad safety cap "
                        f"(reduce delta or n)"
                    )
                    continue
                current = controller.get_state().wait()[0].arm_joint_position
                print(
                    f"starting from joint {idx + 1} = {current[idx]:+.4f} rad; "
                    f"total planned displacement {delta * n:+.4f} rad"
                )
                target = current.copy()
                t0 = time.time()
                for _ in range(n):
                    target[idx] += delta
                    controller.move_joints(target).wait()
                    time.sleep(0.001)
                elapsed = time.time() - t0
                end = controller.get_state().wait()[0].arm_joint_position
                print(
                    f"streamed {n} ticks in {elapsed:.3f}s "
                    f"(~{n / elapsed:.0f} Hz); "
                    f"joint {idx + 1} now = {end[idx]:+.4f} rad"
                )
            elif cmd == "hold":
                secs = float(parts[1]) if len(parts) == 2 else 30.0
                print(f"holding for {secs:.1f}s — listen for buzz")
                time.sleep(secs)
                state = controller.get_state().wait()[0]
                print(
                    f"joint_vel rms = "
                    f"{np.sqrt(np.mean(state.arm_joint_velocity**2)):.5f} rad/s"
                )
            elif cmd == "open":
                # Opening while holding drops the cube. During H1 that is the
                # intended first step (hand empty), but after the cube is grasped
                # and the arm guided above the mark it is how you lose it.
                holding = False
                try:
                    holding = bool(controller.gripper_holding().wait()[0])
                except Exception as exc:
                    print(f"could not read gripper state ({exc}); proceeding")
                if holding and not (len(parts) == 2 and parts[1].lower() == "yes"):
                    print(
                        "gripper is currently HOLDING something; opening will drop "
                        "it. Type `open yes` to open anyway."
                    )
                    continue
                controller.open_gripper().wait()
                print("gripper opened")
            elif cmd == "close":
                if len(parts) == 2:
                    force = float(parts[1])
                    controller.close_gripper_force(force).wait()
                else:
                    controller.close_gripper().wait()
                print("gripper closed (gentle grasp; still holding at commanded N)")
            elif cmd == "stop":
                controller.stop_gripper().wait()
                print("gripper stop sent")
            elif cmd == "grip":
                if len(parts) != 2:
                    print("usage: grip <0-255>")
                    continue
                pos = int(parts[1])
                controller.move_gripper(pos).wait()
                print(f"gripper moved to {pos}")
            elif cmd == "impedance":
                if len(parts) == 3:
                    kt = float(parts[1])
                    kr = float(parts[2])
                    controller.reconfigure_compliance_params(
                        {
                            "translational_stiffness": kt,
                            "rotational_stiffness": kr,
                        }
                    ).wait()
                    # The controller clamps the stiffness and derives the error
                    # clip from it, so the commanded FORCE ceiling stays put. The
                    # tracker's next start log prints the resulting products;
                    # read that rather than assuming kt landed verbatim.
                    print(
                        f"impedance requested K_t={kt} K_r={kr}; "
                        f"resulting authority -> {describe_authority(kt, kr)}"
                    )
                elif len(parts) == 8:
                    print(
                        "7-int Kq form is upstream ROS-style; "
                        "this franky stack uses: impedance <k_trans> <k_rot>"
                    )
                else:
                    print("usage: impedance <k_trans> <k_rot>")
                    continue
            else:
                print(f"unknown cmd: {cmd_str}")
                _print_help()
        except Exception as exc:
            print(f"command failed: {exc}")

        time.sleep(0.05)

    print("shutting down...")
    try:
        controller.cleanup().wait()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
