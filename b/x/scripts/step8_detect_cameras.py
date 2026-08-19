#!/usr/bin/env python3
"""Step 8a: detect cameras on this host (no Franka FCI).

Enumerates USB / V4L devices and RLinf-supported backends (realsense, zed,
lumos). Writes JSON for Step 8b/8c. Does not open a RealSense pipeline
(avoids exclusive lock).

Usage (franky container, privileged, USB passed through):
  source b/x/configs/setup_before_ray_5090.sh
  python b/x/scripts/step8_detect_cameras.py
  python b/x/scripts/step8_detect_cameras.py --write-yaml
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

REPO = os.environ.get(
    "REPO_PATH",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")),
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))
sys.path.insert(0, os.path.dirname(__file__))

from step8_checks import check, is_placeholder_serial, result

DEFAULT_JSON = os.path.join(REPO, "b", "x", "configs", "camera_detected.json")
DEFAULT_YAML = os.path.join(REPO, "b", "x", "configs", "realworld_franky_camera.yaml")
CAMERA_TYPES = ("realsense", "zed", "lumos")


def _run(cmd: list[str]) -> str:
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    except FileNotFoundError:
        return f"(missing binary) {cmd[0]}"
    if proc.returncode != 0:
        err = (proc.stderr or proc.stdout or "").strip()
        return f"(exit {proc.returncode}) {err}"
    return (proc.stdout or "").rstrip()


def _list_paths(pattern: str) -> list[str]:
    import glob

    return sorted(glob.glob(pattern))


def _safe_rs_info(device: Any, camera_info: Any) -> str | None:
    try:
        value = device.get_info(camera_info)
    except Exception:
        return None
    return str(value) if value else None


def _realsense_details() -> tuple[list[dict[str, Any]], str | None]:
    try:
        import pyrealsense2 as rs
    except ImportError:
        return [], "pyrealsense2 not installed"

    devices_out: list[dict[str, Any]] = []
    ctx = rs.context()
    for device in ctx.devices:
        info: dict[str, Any] = {
            "serial": device.get_info(rs.camera_info.serial_number),
            "name": _safe_rs_info(device, rs.camera_info.name),
            "firmware": _safe_rs_info(device, rs.camera_info.firmware_version),
            "product_line": _safe_rs_info(device, rs.camera_info.product_line),
            "usb_type": _safe_rs_info(device, rs.camera_info.usb_type_descriptor),
            "physical_port": _safe_rs_info(device, rs.camera_info.physical_port),
            "supports_default_640x480_15": False,
            "streams": [],
        }
        seen: set[tuple[str, str, int, int, int]] = set()
        try:
            for sensor in device.query_sensors():
                sensor_name = _safe_rs_info(sensor, rs.camera_info.name) or "sensor"
                for profile in sensor.get_stream_profiles():
                    if not profile.is_video_stream_profile():
                        continue
                    video = profile.as_video_stream_profile()
                    key = (
                        str(profile.stream_type()),
                        str(profile.format()),
                        int(video.width()),
                        int(video.height()),
                        int(profile.fps()),
                    )
                    if key in seen:
                        continue
                    seen.add(key)
                    if (
                        "color" in key[0].lower()
                        and key[2] == 640
                        and key[3] == 480
                        and key[4] == 15
                    ):
                        info["supports_default_640x480_15"] = True
                    info["streams"].append(
                        {
                            "sensor": sensor_name,
                            "stream": key[0],
                            "format": key[1],
                            "width": key[2],
                            "height": key[3],
                            "fps": key[4],
                        }
                    )
        except Exception as exc:
            info["stream_error"] = str(exc)
        devices_out.append(info)
    return devices_out, None


def _enumerate() -> dict[str, list[str]]:
    from rlinf.scheduler.hardware.robots.franka import FrankaRobot

    found: dict[str, list[str]] = {}
    for camera_type in CAMERA_TYPES:
        try:
            serials = sorted(str(s) for s in FrankaRobot.enumerate_cameras(camera_type))
        except Exception as exc:
            print(f"  {camera_type}: enumerate failed: {exc}")
            serials = []
        found[camera_type] = serials
    return found


def _pick_primary(enumerated: dict[str, list[str]]) -> tuple[str, list[str]]:
    for camera_type in CAMERA_TYPES:
        serials = [
            s
            for s in enumerated.get(camera_type, [])
            if not is_placeholder_serial(s)
        ]
        if serials:
            return camera_type, serials
    return "realsense", []


def _write_yaml(path: str, serials: list[str], camera_type: str) -> None:
    import yaml

    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"unexpected YAML at {path}")

    node_groups = data.get("cluster", {}).get("node_groups") or []
    for group in node_groups:
        configs = ((group.get("hardware") or {}).get("configs")) or []
        for cfg in configs:
            cfg["camera_type"] = camera_type
            cfg["camera_serials"] = list(serials)

    override = data.get("env", {}).get("eval", {}).get("override_cfg")
    if isinstance(override, dict):
        override["camera_type"] = camera_type
        override["camera_serials"] = list(serials)
        names = override.get("camera_names") or {}
        for index, serial in enumerate(serials, start=1):
            names.setdefault(serial, f"wrist_{index}")
        override["camera_names"] = names

    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Step 8a: detect cameras")
    parser.add_argument(
        "--json-out",
        default=DEFAULT_JSON,
        help=f"output JSON path (default: {DEFAULT_JSON})",
    )
    parser.add_argument(
        "--write-yaml",
        action="store_true",
        help="fill camera_serials / camera_type in realworld_franky_camera.yaml",
    )
    parser.add_argument(
        "--yaml-out",
        default=DEFAULT_YAML,
        help=f"YAML to update with --write-yaml (default: {DEFAULT_YAML})",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("Step8a: detect cameras (no FCI)")
    print("hint: close realsense-viewer / other camera apps first")
    failed = False

    lsusb = _run(["lsusb"])
    print("--- lsusb ---")
    print(lsusb or "(empty)")

    video_nodes = _list_paths("/dev/video*")
    by_id = _list_paths("/dev/v4l/by-id/*")
    print("--- /dev/video* ---")
    print("\n".join(video_nodes) or "(none)")
    print("--- /dev/v4l/by-id ---")
    print("\n".join(by_id) or "(none)")
    usb_ok = (
        bool(lsusb)
        and not str(lsusb).startswith("(exit")
        and not str(lsusb).startswith("(missing")
    )
    nodes_ok = bool(video_nodes or by_id)
    if not check("usb_or_v4l_present", usb_ok or nodes_ok, "lsusb or /dev/video*"):
        # Not fatal by itself: some RealSense still enumerate via SDK only.
        print("note: no lsusb/v4l listing; SDK enumerate may still succeed")

    enumerated = _enumerate()
    print("--- RLinf enumerate_cameras ---")
    for camera_type, serials in enumerated.items():
        print(f"  {camera_type}: {serials or '(none)'}")

    rs_details, rs_error = _realsense_details()
    if rs_error:
        print(f"--- realsense details ---\n  {rs_error}")
    elif rs_details:
        print("--- realsense details ---")
        print(json.dumps(rs_details, indent=2, ensure_ascii=False))

    camera_type, serials = _pick_primary(enumerated)
    if camera_type == "lumos" and not enumerated.get("realsense"):
        print(
            "note: primary backend is lumos (V4L2). "
            "If this is a RealSense, install pyrealsense2 in the franky venv."
        )
    any_enum = any(enumerated.values())
    if not check("rlinf_enumerate_nonempty", any_enum, str(enumerated)):
        failed = True
    if not check("primary_serials_nonempty", bool(serials), str(serials)):
        failed = True
    ph = [s for s in serials if is_placeholder_serial(s)]
    if not check("serials_not_placeholder", not ph, str(ph or serials)):
        failed = True
    if camera_type == "realsense":
        sdk_ok = rs_error is None and bool(rs_details)
        if not check("realsense_sdk_devices", sdk_ok, rs_error or f"n={len(rs_details)}"):
            failed = True

    payload = {
        "camera_type": camera_type,
        "camera_serials": serials,
        "enumerated": enumerated,
        "realsense_details": rs_details,
        "realsense_error": rs_error,
        "lsusb": lsusb,
        "video_nodes": video_nodes,
        "v4l_by_id": [os.path.basename(p) for p in by_id],
        "default_camera_info": {
            "resolution": [640, 480],
            "fps": 15,
            "obs_frames_shape": [128, 128, 3],
        },
    }

    out_path = Path(args.json_out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    json_ok = out_path.is_file() and bool(serials)
    if not check("json_written", json_ok, str(out_path)):
        failed = True

    if args.write_yaml and serials and not failed:
        _write_yaml(args.yaml_out, serials, camera_type)
        print(f"updated {args.yaml_out} camera_type={camera_type} serials={serials}")
        if not check("yaml_written", os.path.isfile(args.yaml_out), args.yaml_out):
            failed = True

    print(f"primary camera_type={camera_type} serials={serials}")
    if failed:
        print("hint: USB in this container (--privileged), SDK, close realsense-viewer")
    return result(
        "8a",
        not failed,
        camera_type=camera_type,
        camera_serials=serials,
        json=str(out_path),
    )


if __name__ == "__main__":
    raise SystemExit(main())
