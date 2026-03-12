#!/usr/bin/env python3
"""
Test script: capture images from two RealSense D435 cameras and save as PNG.

Output saved to script/images/

Usage:
    python3 script/test_cameras.py                          # auto-detect, capture once
    python3 script/test_cameras.py --list                   # list cameras and exit
    python3 script/test_cameras.py --wrist SN1 --head SN2   # specify serials
    python3 script/test_cameras.py --continuous              # keep capturing every 2s
"""

import argparse
import os
import struct
import sys
import time
import zlib

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR = os.path.join(SCRIPT_DIR, "images")


def list_cameras():
    import pyrealsense2 as rs
    ctx = rs.context()
    devices = ctx.query_devices()
    result = []
    for dev in devices:
        sn = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        result.append((sn, name))
    return result


def start_camera(serial, width=640, height=480, fps=30):
    import pyrealsense2 as rs
    pipeline = rs.pipeline()
    config = rs.config()
    if serial:
        config.enable_device(serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
    pipeline.start(config)
    # warm up
    for _ in range(5):
        pipeline.wait_for_frames(5000)
    return pipeline


def save_png(filepath, img_rgb, width, height):
    """Save RGB uint8 numpy array as PNG (no opencv/PIL needed)."""
    def _make_png(data, w, h):
        # Minimal PNG writer for RGB
        def _chunk(chunk_type, chunk_data):
            c = chunk_type + chunk_data
            crc = struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)
            return struct.pack(">I", len(chunk_data)) + c + crc

        header = b"\x89PNG\r\n\x1a\n"
        ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)  # 8-bit RGB
        ihdr_chunk = _chunk(b"IHDR", ihdr)

        # Build raw scanlines: filter byte (0) + row data
        raw = b""
        for y in range(h):
            raw += b"\x00"  # filter: none
            raw += bytes(data[y * w * 3 : (y + 1) * w * 3])
        compressed = zlib.compress(raw)
        idat_chunk = _chunk(b"IDAT", compressed)
        iend_chunk = _chunk(b"IEND", b"")

        return header + ihdr_chunk + idat_chunk + iend_chunk

    png_bytes = _make_png(img_rgb.tobytes(), width, height)
    with open(filepath, "wb") as f:
        f.write(png_bytes)


def main():
    ap = argparse.ArgumentParser(description="Capture images from RealSense D435 cameras -> script/images/")
    ap.add_argument("--list", action="store_true", help="list cameras and exit")
    ap.add_argument("--wrist", default="", help="serial number of wrist camera")
    ap.add_argument("--head", default="", help="serial number of head camera")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--continuous", action="store_true", help="keep capturing every 2 seconds (Ctrl+C to stop)")
    args = ap.parse_args()

    try:
        import pyrealsense2 as rs
    except ImportError:
        print("ERROR: pyrealsense2 not installed. Run: pip install pyrealsense2")
        sys.exit(1)

    try:
        import numpy as np
    except ImportError:
        print("ERROR: numpy not installed. Run: pip install numpy")
        sys.exit(1)

    # List cameras
    cams = list_cameras()
    print(f"Found {len(cams)} RealSense camera(s):")
    for sn, name in cams:
        print(f"  serial={sn}  name={name}")

    if args.list:
        return

    if len(cams) == 0:
        print("No cameras found!")
        sys.exit(1)

    # Assign serials
    wrist_sn = args.wrist
    head_sn = args.head
    if len(cams) >= 2 and not wrist_sn and not head_sn:
        wrist_sn = cams[0][0]
        head_sn = cams[1][0]
        print(f"\nAuto-assigned: wrist={wrist_sn}  head={head_sn}")
        print("(use --wrist and --head to override)\n")
    elif len(cams) == 1 and not wrist_sn:
        wrist_sn = cams[0][0]
        print(f"\nOnly 1 camera, using as wrist: {wrist_sn}\n")

    # Start cameras
    wrist_pipe = None
    head_pipe = None
    if wrist_sn:
        print(f"Starting wrist camera ({wrist_sn})...")
        wrist_pipe = start_camera(wrist_sn, args.width, args.height, args.fps)
    if head_sn:
        print(f"Starting head camera ({head_sn})...")
        head_pipe = start_camera(head_sn, args.width, args.height, args.fps)

    # Create output directory
    os.makedirs(IMAGES_DIR, exist_ok=True)
    print(f"Saving images to: {IMAGES_DIR}/\n")

    snap_count = 0
    try:
        while True:
            snap_count += 1
            timestamp = time.strftime("%Y%m%d_%H%M%S")

            if wrist_pipe:
                frames = wrist_pipe.wait_for_frames(5000)
                cf = frames.get_color_frame()
                if cf:
                    img = np.asanyarray(cf.get_data())
                    fn = os.path.join(IMAGES_DIR, f"{timestamp}_wrist.png")
                    save_png(fn, img, args.width, args.height)
                    print(f"  [{snap_count}] Saved {fn} ({args.width}x{args.height})")

            if head_pipe:
                frames = head_pipe.wait_for_frames(5000)
                cf = frames.get_color_frame()
                if cf:
                    img = np.asanyarray(cf.get_data())
                    fn = os.path.join(IMAGES_DIR, f"{timestamp}_head.png")
                    save_png(fn, img, args.width, args.height)
                    print(f"  [{snap_count}] Saved {fn} ({args.width}x{args.height})")

            if not args.continuous:
                print("\nDone. Open the PNG files in script/images/ to view.")
                break

            print(f"  ... next capture in 2s (Ctrl+C to stop)")
            time.sleep(2)

    except KeyboardInterrupt:
        print(f"\nStopped. Captured {snap_count} snapshot(s).")
    finally:
        if wrist_pipe:
            wrist_pipe.stop()
        if head_pipe:
            head_pipe.stop()


if __name__ == "__main__":
    main()
