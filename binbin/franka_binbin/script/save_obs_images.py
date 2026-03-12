#!/usr/bin/env python3
"""
Capture one frame from each RealSense camera, apply the exact same
processing pipeline as station/run.py (center crop → NN resize to 224x224),
and save as PNG. Use this to verify what the VLA model actually sees.

Usage:
    python3 script/save_obs_images.py --wrist-serial XXXXX --head-serial YYYYY
"""

import argparse
import os
import struct
import sys
import time
import zlib

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR = os.path.join(SCRIPT_DIR, "images")
PROJ_ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, PROJ_ROOT)

IMAGE_SIZE = 224


def save_png(filepath, data_bytes, w, h):
    """Minimal PNG writer for RGB uint8."""
    def _chunk(ctype, cdata):
        c = ctype + cdata
        crc = struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)
        return struct.pack(">I", len(cdata)) + c + crc
    header = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    raw = b""
    for y in range(h):
        raw += b"\x00" + data_bytes[y * w * 3 : (y + 1) * w * 3]
    compressed = zlib.compress(raw)
    png = header + _chunk(b"IHDR", ihdr) + _chunk(b"IDAT", compressed) + _chunk(b"IEND", b"")
    with open(filepath, "wb") as f:
        f.write(png)

def _resize_with_pad(np, img, size: int):
    """Match station/run.py: scale longest side to size, then pad to size."""
    h, w = img.shape[:2]
    scale = size / max(h, w)
    new_h = int(h * scale)
    new_w = int(w * scale)
    ys = (np.arange(new_h) * (h / new_h)).astype(np.int32)
    xs = (np.arange(new_w) * (w / new_w)).astype(np.int32)
    ys = np.clip(ys, 0, h - 1)
    xs = np.clip(xs, 0, w - 1)
    resized = img[ys[:, None], xs[None, :], :]
    out = np.zeros((size, size, 3), dtype=np.uint8)
    start_y = (size - new_h) // 2
    start_x = (size - new_w) // 2
    out[start_y : start_y + new_h, start_x : start_x + new_w, :] = resized
    return out


def main():
    import numpy as np
    import pyrealsense2 as rs
    from PIL import Image

    ctx = rs.context()
    devices = ctx.query_devices()
    cams = []
    for d in devices:
        sn = d.get_info(rs.camera_info.serial_number)
        name = d.get_info(rs.camera_info.name)
        cams.append((sn, name))
        print(f"Found camera: serial={sn} name={name}")
    
    if len(cams) < 2:
        print("Need 2 cameras (wrist + head)")
        sys.exit(1)

    ap = argparse.ArgumentParser()
    ap.add_argument("--wrist-serial", default="", help="RealSense serial for wrist camera (empty=auto)")
    ap.add_argument("--head-serial", default="", help="RealSense serial for head/exterior camera (empty=auto)")
    ap.add_argument("--swap-cameras", action="store_true", help="swap auto-assigned wrist/head")
    ap.add_argument("--color-format", choices=["rgb8", "bgr8"], default="rgb8", help="match station --rs-color-format")
    ap.add_argument("--image-size", type=int, default=IMAGE_SIZE, help="final size (default 224)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--warmup", type=int, default=15)
    ap.add_argument("--timeout-ms", type=int, default=1000)
    args = ap.parse_args()

    wrist_sn = str(args.wrist_serial)
    head_sn = str(args.head_serial)
    if (not wrist_sn) or (not head_sn):
        # Same default as station/run.py: first= wrist, second=head (unless swapped)
        if bool(args.swap_cameras):
            wrist_sn = cams[1][0]
            head_sn = cams[0][0]
        else:
            wrist_sn = cams[0][0]
            head_sn = cams[1][0]
    print("\nUsing:")
    print(f"  Wrist = {wrist_sn}")
    print(f"  Head  = {head_sn}")

    def get_frame(serial: str):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        if args.color_format == "rgb8":
            fmt = rs.format.rgb8
        else:
            fmt = rs.format.bgr8
        config.enable_stream(rs.stream.color, int(args.width), int(args.height), fmt, int(args.fps))
        pipeline.start(config)
        try:
            for _ in range(int(args.warmup)):
                pipeline.wait_for_frames(int(args.timeout_ms))
            frames = pipeline.wait_for_frames(int(args.timeout_ms))
            color_frame = frames.get_color_frame()
            if not color_frame:
                raise RuntimeError("no color frame")
            img = np.asanyarray(color_frame.get_data())
            img = _resize_with_pad(np, img, int(args.image_size))
            # station/run.py always returns RGB
            if args.color_format == "bgr8":
                img = img[..., ::-1]
            return img
        finally:
            pipeline.stop()

    print("\nCapturing WRIST image...")
    wrist_img = get_frame(wrist_sn)
    print("Capturing HEAD image...")
    head_img = get_frame(head_sn)
    
    os.makedirs(IMAGES_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    
    w_path = os.path.join(IMAGES_DIR, f"wrist_{wrist_sn}_{args.image_size}.png")
    h_path = os.path.join(IMAGES_DIR, f"head_{head_sn}_{args.image_size}.png")
    
    Image.fromarray(wrist_img).save(w_path)
    Image.fromarray(head_img).save(h_path)
    
    print(f"\nSaved images to verify orientation:")
    print(f"1. {w_path} (Should see the gripper/hand)")
    print(f"2. {h_path} (Should see the whole robot/table)")



if __name__ == "__main__":
    main()
