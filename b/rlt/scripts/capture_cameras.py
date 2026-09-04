"""Capture one frame from each RealSense camera and save as PNG."""

import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

CAMERAS = {
    "250222073513": "global",
    "420122070525": "wrist",
}

OUT_DIR = "/tmp"


def capture(serial: str, name: str) -> np.ndarray:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    try:
        # Skip a few frames to let auto-exposure settle
        for _ in range(30):
            pipeline.wait_for_frames()
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError(f"No color frame from {serial}")
        image = np.asanyarray(color_frame.get_data())  # BGR, 480x640x3
    finally:
        pipeline.stop()

    path = f"{OUT_DIR}/camera_{name}_{serial}.png"
    cv2.imwrite(path, image)
    print(f"[{name}] serial={serial}  shape={image.shape}  saved to {path}")

    # Also save the 224x224 center-crop version (same as deployment pipeline)
    h, w = image.shape[:2]
    crop_size = min(h, w)  # 480
    x0 = (w - crop_size) // 2
    y0 = (h - crop_size) // 2
    cropped = image[y0 : y0 + crop_size, x0 : x0 + crop_size]
    resized = cv2.resize(cropped, (224, 224))
    crop_path = f"{OUT_DIR}/camera_{name}_{serial}_224.png"
    cv2.imwrite(crop_path, resized)
    print(f"[{name}] 224x224 center-crop saved to {crop_path}")

    return image


def main():
    for serial, name in CAMERAS.items():
        try:
            capture(serial, name)
        except Exception as e:
            print(f"[{name}] ERROR: {e}", file=sys.stderr)
    print("Done.")


if __name__ == "__main__":
    main()
