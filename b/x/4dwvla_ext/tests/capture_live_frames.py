#!/usr/bin/env python3
"""Capture one live camera frame pair, exactly as the policy would receive it.

Read-only: it opens the RealSense pipelines and, optionally, an FCI connection
to read the current joint pose. It never commands the robot or the gripper.

Written because the ablation in diag_input_ablation.py had to fall back on PNGs
saved in an earlier session, whose scene did not match the failing run. This
captures a pair in the current session and reports how far the arm is from
HOME, so the ablation can be run on inputs the policy would really have seen.

Frames are saved twice: ``.npy`` holds the exact uint8 RGB array handed to the
server, and ``.png`` is for looking at. The ablation reads the ``.npy`` so no
PNG colour round-trip can creep in.

    /opt/venv/franky-0.19.0/bin/python \
        /workspace/RLinf/b/x/4dwvla_ext/tests/capture_live_frames.py \
        --global-camera-serial 250222073513 \
        --wrist-camera-serial 420122070525 \
        --robot-ip 172.16.0.2 \
        --out-dir /workspace/RLinf/b/x/4dwvla_ext/logs/live_capture
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

_BX_ROOT = Path(__file__).resolve().parents[2]
if str(_BX_ROOT) not in sys.path:
    sys.path.insert(0, str(_BX_ROOT))

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("capture-live")

#: Per-channel RGB means of the demonstrations, from meta/stats.json scaled to
#: 0-255. Used only to report the gap; nothing here corrects for it.
TRAIN_RGB_MEAN = {
    "global": np.array([114.4, 116.6, 114.2]),
    "wrist": np.array([122.3, 108.1, 100.8]),
}


def read_robot_pose(robot_ip: str):
    """Read q without commanding anything; returns None if unavailable."""
    try:
        import franky  # pyright: ignore[reportMissingImports]
    except ImportError:
        logger.warning("franky not importable; skipping the pose check")
        return None
    try:
        robot = franky.Robot(robot_ip)
        state = robot.state
        return np.asarray(state.q, dtype=np.float64)
    except Exception as exc:
        logger.warning("could not read robot state (%s); skipping the pose check", exc)
        return None


def capture(serials: dict[str, str], warmup: int):
    import pyrealsense2 as rs  # pyright: ignore[reportMissingImports]

    frames: dict[str, np.ndarray] = {}
    for name in ("global", "wrist"):
        pipe = rs.pipeline()
        cfg = rs.config()
        if serials.get(name):
            cfg.enable_device(serials[name])
        # Same stream format the eval env uses, so the array is already RGB.
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        pipe.start(cfg)
        try:
            for _ in range(warmup):
                pipe.wait_for_frames(timeout_ms=2000)
            fs = pipe.wait_for_frames(timeout_ms=2000)
            color = fs.get_color_frame()
            if not color:
                raise RuntimeError(f"no colour frame from {name}")
            frames[name] = np.asarray(color.get_data(), dtype=np.uint8).copy()
        finally:
            pipe.stop()
        logger.info("captured %s %s", name, frames[name].shape)
    return frames


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--global-camera-serial", default=None)
    p.add_argument("--wrist-camera-serial", default=None)
    p.add_argument("--robot-ip", default=None, help="read-only pose check")
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument(
        "--warmup",
        type=int,
        default=30,
        help="frames to discard so auto-exposure and white balance settle",
    )
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    home_q = None
    try:
        from franky_ext.dsplug.home_pose import load_home_joints

        home_q = load_home_joints()
    except Exception as exc:
        logger.warning("could not load HOME (%s)", exc)

    q = read_robot_pose(args.robot_ip) if args.robot_ip else None
    if q is not None and home_q is not None:
        dist = float(np.linalg.norm(q - home_q))
        logger.info("current q      = %s", np.array2string(q, precision=4))
        logger.info("HOME q         = %s", np.array2string(home_q, precision=4))
        logger.info("||q - HOME||   = %.4f rad", dist)
        if dist > 0.05:
            logger.warning(
                "arm is %.4f rad from HOME; the capture will not represent the "
                "start of an episode. Send it HOME first with "
                "dsplug/stop_and_home.py --execute.",
                dist,
            )

    frames = capture(
        {"global": args.global_camera_serial, "wrist": args.wrist_camera_serial},
        args.warmup,
    )

    print()
    print(f"{'camera':8s} {'live RGB mean':>26s} {'train RGB mean':>26s} {'delta':>26s}")
    meta: dict[str, object] = {
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "warmup_frames": args.warmup,
        "serials": {
            "global": args.global_camera_serial,
            "wrist": args.wrist_camera_serial,
        },
        "robot_q": None if q is None else q.tolist(),
        "home_q": None if home_q is None else home_q.tolist(),
        "dist_to_home_rad": None
        if (q is None or home_q is None)
        else float(np.linalg.norm(q - home_q)),
        "channel_means": {},
    }
    for name, img in frames.items():
        np.save(args.out_dir / f"{name}.npy", img)
        try:
            from PIL import Image

            Image.fromarray(img).save(args.out_dir / f"{name}.png")
        except Exception as exc:  # pragma: no cover - viewing aid only
            logger.warning("could not write %s.png (%s)", name, exc)
        m = img.reshape(-1, 3).mean(axis=0)
        t = TRAIN_RGB_MEAN[name]
        meta["channel_means"][name] = m.tolist()
        print(f"{name:8s} {np.array2string(m, precision=1):>26s} "
              f"{np.array2string(t, precision=1):>26s} "
              f"{np.array2string(m - t, precision=1):>26s}")

    (args.out_dir / "capture_meta.json").write_text(
        json.dumps(meta, indent=2), encoding="utf-8"
    )
    logger.info("wrote %s", args.out_dir)
    print("\nRESULT LIVE_CAPTURE PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
