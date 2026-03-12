"""
Station main loop — direct velocity mode.

PI0.5 DROID outputs normalized joint velocities in [-1, 1] + gripper position.
We send these directly to robot_server's JOINT_VELOCITY mode, which internally
converts to position deltas: q += clip(vel, -1, 1) * max_joint_delta.

This matches the DROID control architecture: one action per 15Hz step,
no intermediate trajectory interpolation or multi-level clamping.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

sys.path.insert(0, "/home/a25181/Desktop/franka/vendor/openpi_client_src")
from openpi_client import image_tools
from openpi_client.websocket_client_policy import WebsocketClientPolicy

from station.logger import JsonlLogger
from station.robot_client import RobotClient, RobotConnectionParams
from station.robot_protocol import CmdMode

DROID_CONTROL_HZ = 15.0
GRIPPER_MAX_WIDTH = 0.08


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class RealSenseD435:
    """Minimal RealSense D435 color capture (RGB uint8) with frame caching."""

    def __init__(
        self,
        *,
        width: int,
        height: int,
        fps: int,
        color_format: str,
        image_size: int,
        timeout_ms: int,
        serial: str = "",
    ) -> None:
        try:
            import pyrealsense2 as rs  # type: ignore
        except Exception as e:
            raise SystemExit("Missing dependency: pyrealsense2 (install after librealsense2).") from e

        self._rs = rs
        self._image_size = int(image_size)
        self._timeout_ms = int(timeout_ms)
        self._color_format = str(color_format)
        self._last_good_frame: Optional[np.ndarray] = None

        if self._color_format == "rgb8":
            fmt = rs.format.rgb8
        elif self._color_format == "bgr8":
            fmt = rs.format.bgr8
        else:
            raise ValueError(f"unsupported rs color_format: {self._color_format}")

        import time as _time
        last_err: RuntimeError | None = None
        for attempt in range(3):
            pipeline = rs.pipeline()
            config = rs.config()
            if serial:
                config.enable_device(str(serial))
            config.enable_stream(rs.stream.color, int(width), int(height), fmt, int(fps))
            profile = pipeline.start(config)
            self._pipeline = pipeline
            self._configure_sensor(profile)
            try:
                warmup_timeout = max(self._timeout_ms, 5000)
                for _ in range(30):
                    self._pipeline.wait_for_frames(warmup_timeout)
                for _ in range(2):
                    self.read_rgb_u8()
                last_err = None
                break
            except RuntimeError as e:
                last_err = e
                pipeline.stop()
                label = serial or "unknown"
                print(f"[camera] {label}: warmup attempt {attempt + 1}/3 failed ({e}), retrying …")
                _time.sleep(2)
        if last_err is not None:
            raise last_err

    def _configure_sensor(self, profile) -> None:
        rs = self._rs
        try:
            sensor = profile.get_device().first_color_sensor()
            if sensor.supports(rs.option.enable_auto_exposure):
                sensor.set_option(rs.option.enable_auto_exposure, 1)
            if sensor.supports(rs.option.enable_auto_white_balance):
                sensor.set_option(rs.option.enable_auto_white_balance, 1)
            if sensor.supports(rs.option.brightness):
                sensor.set_option(rs.option.brightness, 0)
            if sensor.supports(rs.option.contrast):
                sensor.set_option(rs.option.contrast, 50)
        except Exception as e:
            print(f"[camera] WARNING: sensor config failed: {e}")

    def close(self) -> None:
        try:
            self._pipeline.stop()
        except Exception:
            pass

    def read_rgb_u8(self) -> np.ndarray:
        frames = self._pipeline.wait_for_frames(self._timeout_ms)
        cf = frames.get_color_frame()
        if not cf:
            raise RuntimeError("no color frame")
        img = np.asanyarray(cf.get_data())
        if self._color_format == "bgr8":
            img = img[..., ::-1].copy()
        img = image_tools.resize_with_pad(img, self._image_size, self._image_size)
        self._last_good_frame = img
        return img

    @property
    def last_good_frame(self) -> Optional[np.ndarray]:
        return self._last_good_frame


def _reset_realsense_devices() -> None:
    """Hardware-reset all RealSense cameras to clear stale USB state."""
    try:
        import pyrealsense2 as rs  # type: ignore
    except ImportError:
        return
    import time
    ctx = rs.context()
    devs = ctx.query_devices()
    if not devs.size():
        return
    serials = []
    for d in devs:
        sn = d.get_info(rs.camera_info.serial_number)
        serials.append(sn)
        print(f"[camera] resetting {sn} ...")
        d.hardware_reset()
    expected = len(serials)
    for attempt in range(20):
        time.sleep(1)
        n = rs.context().query_devices().size()
        if n >= expected:
            print(f"[camera] all {n} device(s) back after {attempt + 1}s")
            return
    print(f"[camera] WARNING: only {n}/{expected} device(s) re-enumerated after 20s")


def list_realsense_cameras() -> list:
    try:
        import pyrealsense2 as rs  # type: ignore
    except ImportError:
        return []
    ctx = rs.context()
    devices = ctx.query_devices()
    return [(dev.get_info(rs.camera_info.serial_number), dev.get_info(rs.camera_info.name)) for dev in devices]


def gripper_to_normalized(width: float) -> float:
    return max(0.0, min(1.0, float(width) / GRIPPER_MAX_WIDTH))


def build_observation(
    *,
    prompt: str,
    q: Tuple[float, ...],
    gripper: float,
    wrist_image: Optional[np.ndarray] = None,
    exterior_image: Optional[np.ndarray] = None,
    image_size: int = 224,
) -> Dict[str, Any]:
    s = int(image_size)
    zero_img = np.zeros((s, s, 3), dtype=np.uint8)

    return {
        "observation/exterior_image_1_left": exterior_image if exterior_image is not None else zero_img,
        "observation/wrist_image_left": wrist_image if wrist_image is not None else zero_img,
        "observation/joint_position": np.array(q, dtype=np.float32),
        "observation/gripper_position": np.array([gripper_to_normalized(gripper)], dtype=np.float32),
        "prompt": prompt,
    }


def _save_debug_images(wrist_img: Optional[np.ndarray], ext_img: Optional[np.ndarray], log_dir: str) -> None:
    try:
        from PIL import Image
        import os
        os.makedirs(log_dir, exist_ok=True)
        if wrist_img is not None:
            Image.fromarray(wrist_img).save(os.path.join(log_dir, "obs_wrist.png"))
        if ext_img is not None:
            Image.fromarray(ext_img).save(os.path.join(log_dir, "obs_exterior.png"))
        print(f"[debug] saved observation images to {log_dir}/")
    except Exception as e:
        print(f"[debug] failed to save images: {e}")


def _reconnect_policy(host: str, port: int) -> WebsocketClientPolicy:
    while True:
        try:
            p = WebsocketClientPolicy(host=host, port=port)
            print(f"[policy] reconnected to {host}:{port}")
            return p
        except Exception as e:
            print(f"[policy] reconnect failed: {e}, retrying in 1s ...")
            time.sleep(1.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-host", default="127.0.0.1")
    ap.add_argument("--robot-port", type=int, default=5555)
    ap.add_argument("--robot-state-port", type=int, default=0, help="default: robot-port + 1")

    ap.add_argument("--policy-host", default="10.239.121.23")
    ap.add_argument("--policy-port", type=int, default=8003)
    ap.add_argument("--prompt", default="pick up the doll on the table")

    ap.add_argument("--control-hz", type=float, default=DROID_CONTROL_HZ)
    ap.add_argument("--open-loop-horizon", type=int, default=8,
                    help="execute N steps from each chunk before re-inferring (like DROID)")
    ap.add_argument("--steps", type=int, default=0, help="0=run forever; otherwise max policy steps")
    ap.add_argument("--validity-ms", type=int, default=500, help="robot command TTL")

    ap.add_argument("--gripper-threshold", type=float, default=0.5)
    ap.add_argument("--gripper-invert", action="store_true")

    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--enable-arm", action="store_true")
    ap.add_argument("--enable-gripper", action="store_true")
    ap.add_argument("--no-robot", action="store_true")

    ap.add_argument("--use-realsense", action="store_true")
    ap.add_argument("--wrist-serial", default="")
    ap.add_argument("--head-serial", default="")
    ap.add_argument("--swap-cameras", action="store_true")
    ap.add_argument("--list-cameras", action="store_true")
    ap.add_argument("--image-size", type=int, default=224)
    ap.add_argument("--rs-width", type=int, default=640)
    ap.add_argument("--rs-height", type=int, default=480)
    ap.add_argument("--rs-fps", type=int, default=30)
    ap.add_argument("--rs-color-format", choices=["rgb8", "bgr8"], default="rgb8")
    ap.add_argument("--rs-timeout-ms", type=int, default=1000)

    ap.add_argument("--log-dir", default="")
    ap.add_argument("--log-file", default="events.jsonl")
    ap.add_argument("--save-obs-images", default="")
    ap.add_argument("--print-vla", action="store_true")
    ap.add_argument("--print-vla-every", type=int, default=5)
    args = ap.parse_args()

    if args.list_cameras:
        cams = list_realsense_cameras()
        if not cams:
            print("No RealSense cameras found.")
        else:
            for serial, name in cams:
                print(f"  serial={serial}  name={name}")
        return

    logger = JsonlLogger(str(args.log_dir), filename=str(args.log_file)) if args.log_dir else None

    # ── Connect to policy server ──────────────────────────────────────
    print(f"[policy] connecting to {args.policy_host}:{args.policy_port} ...")
    policy = WebsocketClientPolicy(host=args.policy_host, port=args.policy_port)
    print("[policy] connected; metadata:", policy.get_server_metadata())

    # ── Cameras ───────────────────────────────────────────────────────
    wrist_cam: Optional[RealSenseD435] = None
    head_cam: Optional[RealSenseD435] = None
    if bool(args.use_realsense):
        _reset_realsense_devices()
        rs_kwargs = dict(
            width=int(args.rs_width), height=int(args.rs_height), fps=int(args.rs_fps),
            color_format=str(args.rs_color_format), image_size=int(args.image_size),
            timeout_ms=int(args.rs_timeout_ms),
        )
        wrist_serial = str(args.wrist_serial)
        head_serial = str(args.head_serial)
        if (not wrist_serial) or (not head_serial):
            available = list_realsense_cameras()
            print(f"[camera] found {len(available)} RealSense camera(s): {available}")
            if len(available) >= 2 and (not wrist_serial) and (not head_serial):
                if bool(args.swap_cameras):
                    wrist_serial, head_serial = available[1][0], available[0][0]
                else:
                    wrist_serial, head_serial = available[0][0], available[1][0]
                print(f"[camera] assigned wrist={wrist_serial} head={head_serial}")
            elif len(available) == 1 and (not wrist_serial):
                wrist_serial = available[0][0]
        if wrist_serial:
            wrist_cam = RealSenseD435(serial=wrist_serial, **rs_kwargs)
        if head_serial:
            head_cam = RealSenseD435(serial=head_serial, **rs_kwargs)

    # ── Robot connection ──────────────────────────────────────────────
    rc = RobotClient(
        RobotConnectionParams(
            host=args.robot_host,
            command_port=args.robot_port,
            state_port=(args.robot_state_port if args.robot_state_port > 0 else None),
        )
    )
    if not args.no_robot:
        try:
            rc.connect()
            print("[robot] connected")
        except Exception as e:
            print(f"[robot] WARNING: could not connect ({e})")
    else:
        print("[robot] disabled (--no-robot)")

    # ── Wait for robot state ──────────────────────────────────────────
    print("[station] waiting for robot_server ...")
    if rc.is_connected():
        for _ in range(50):
            st = rc.get_latest_state()
            if st is not None and int(st.robot_mode) in (1, 3) and int(st.error_code) == 0:
                print(f"[robot] ready (mode={st.robot_mode})")
                break
            time.sleep(0.1)
        else:
            if not args.dry_run:
                raise SystemExit("[safety] No valid robot state received.")

    # ── Main control loop (DROID-style: 15Hz, one action per step) ───
    control_period = 1.0 / max(1.0, float(args.control_hz))
    open_loop_horizon = max(1, int(args.open_loop_horizon))
    step = 0
    total_infers = 0
    action_chunk: Optional[np.ndarray] = None
    chunk_idx = 0
    consecutive_ws_failures = 0

    print(f"[station] running at {args.control_hz} Hz, open_loop_horizon={open_loop_horizon}, mode=JOINT_VELOCITY")

    try:
        while True:
            t_step_start = time.monotonic()

            # ── Get robot state ───────────────────────────────────────
            st = rc.get_latest_state()
            if st is not None:
                q = tuple(float(x) for x in st.q)
                g = float(st.gripper)
                robot_ok = int(st.robot_mode) in (1, 3) and int(st.error_code) == 0
            else:
                q = (0.0,) * 7
                g = 0.0
                robot_ok = args.dry_run or args.no_robot

            if not robot_ok:
                time.sleep(0.05)
                continue

            # ── Need new chunk? ───────────────────────────────────────
            if action_chunk is None or chunk_idx >= open_loop_horizon:
                wrist_img: Optional[np.ndarray] = None
                ext_img: Optional[np.ndarray] = None
                if wrist_cam is not None:
                    try:
                        wrist_img = wrist_cam.read_rgb_u8()
                    except Exception as e:
                        print(f"[camera] wrist WARNING: {e}")
                        wrist_img = wrist_cam.last_good_frame
                if head_cam is not None:
                    try:
                        ext_img = head_cam.read_rgb_u8()
                    except Exception as e:
                        print(f"[camera] head WARNING: {e}")
                        ext_img = head_cam.last_good_frame

                if total_infers == 0 and args.save_obs_images:
                    _save_debug_images(wrist_img, ext_img, args.save_obs_images)

                obs = build_observation(
                    prompt=args.prompt, q=q, gripper=g,
                    wrist_image=wrist_img, exterior_image=ext_img,
                    image_size=int(args.image_size),
                )

                t_infer = time.monotonic()
                try:
                    out = policy.infer(obs)
                    consecutive_ws_failures = 0
                except Exception as e:
                    consecutive_ws_failures += 1
                    print(f"[policy] infer failed ({consecutive_ws_failures}): {e}")
                    if consecutive_ws_failures >= 3:
                        policy = _reconnect_policy(args.policy_host, args.policy_port)
                        consecutive_ws_failures = 0
                    time.sleep(0.2)
                    continue
                infer_ms = (time.monotonic() - t_infer) * 1000.0

                actions = out.get("actions")
                if actions is None:
                    print("[policy] WARNING: no actions returned")
                    continue
                action_chunk = np.array(actions, dtype=np.float32)
                if action_chunk.ndim != 2 or action_chunk.shape[1] != 8:
                    print(f"[policy] WARNING: unexpected shape {action_chunk.shape}")
                    action_chunk = None
                    continue
                chunk_idx = 0

                if bool(args.print_vla) and (total_infers % max(1, args.print_vla_every) == 0):
                    print(
                        f"[vla #{total_infers}] infer={infer_ms:.0f}ms "
                        f"q=[{', '.join(f'{x:.3f}' for x in q)}] g={g:.3f} "
                        f"vel0=[{', '.join(f'{x:.3f}' for x in action_chunk[0, :7])}] grip0={action_chunk[0, 7]:.3f}"
                    )

                if logger:
                    logger.log("infer", {
                        "infer_idx": total_infers, "infer_ms": infer_ms,
                        "q": list(q), "g": g,
                        "actions": action_chunk.tolist(),
                    })
                total_infers += 1

            # ── Execute one action from chunk ─────────────────────────
            action = action_chunk[chunk_idx]
            chunk_idx += 1

            vel = np.clip(action[:7], -1.0, 1.0)
            grip_raw = float(action[7])
            binarized = 1.0 if grip_raw > args.gripper_threshold else 0.0
            if args.gripper_invert:
                binarized = 1.0 - binarized
            gripper_cmd = binarized * GRIPPER_MAX_WIDTH

            if args.dry_run:
                print(
                    f"[dry {step:05d}] vel=[{', '.join(f'{x:+.3f}' for x in vel)}] "
                    f"grip={gripper_cmd:.3f} chunk_idx={chunk_idx}/{open_loop_horizon}"
                )
            elif rc.is_connected():
                rc.send_velocity_command(
                    vel=tuple(float(x) for x in vel),
                    gripper=gripper_cmd,
                    validity_ms=int(args.validity_ms),
                    enable_arm=bool(args.enable_arm) and robot_ok,
                    enable_gripper=bool(args.enable_gripper) and robot_ok,
                )

            if logger and (step < 5 or step % 30 == 0):
                logger.log("step", {
                    "step": step, "chunk_idx": chunk_idx,
                    "vel": vel.tolist(), "gripper_cmd": gripper_cmd,
                    "q": list(q), "g": g, "robot_ok": robot_ok,
                })

            step += 1
            if int(args.steps) > 0 and step >= int(args.steps):
                print(f"[station] reached {args.steps} steps, stopping.")
                break

            elapsed = time.monotonic() - t_step_start
            time.sleep(max(0.0, control_period - elapsed))

    except KeyboardInterrupt:
        print("\n[station] Ctrl+C: stopping ...")
    finally:
        try:
            if rc.is_connected():
                rc.send_stop()
        except Exception:
            pass
        for cam in (wrist_cam, head_cam):
            try:
                if cam is not None:
                    cam.close()
            except Exception:
                pass
        try:
            rc.close()
        except Exception:
            pass
        if logger:
            logger.log("end", {})
            logger.close()
        print("[station] shutdown complete")


if __name__ == "__main__":
    main()
