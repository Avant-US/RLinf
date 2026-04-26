# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Camera multiplexer for Galaxea R1 Pro (USB-direct + ROS 2 dual path).

The Mux owns N :class:`BaseCamera` instances, each independently
configurable via :class:`CameraSpec`:

* ``backend == "usb_direct"``: instantiates :class:`RealSenseCamera`
  reading from a local USB-connected D405.  Required when the wrist
  cable runs to the GPU server.
* ``backend == "ros2"``: instantiates :class:`ROS2Camera` subscribing
  to a remote topic (head GMSL, all-ROS 2 variant, or USB drop
  fallback).

The Mux exposes a single :meth:`get_frames` method returning a
``{name: ndarray}`` dict.  Each frame is centre-cropped + resized to
``image_size`` (matching :meth:`FrankaEnv._crop_frame`).

Soft-sync window:
    ``align_strategy="latest"`` returns the latest available frame
    from each camera (default).  ``align_strategy="sync_window"``
    additionally checks that all frames fall inside
    ``soft_sync_window_ms`` of each other; out-of-window frames are
    counted in ``camera/sync_window_reject_rate``.

USB-drop fallback:
    When ``fallback_to_ros2_on_usb_drop`` is ``True`` and a
    USB-direct camera errors / times out for
    ``usb_drop_consecutive_threshold`` consecutive reads, the Mux
    transparently re-instantiates that camera as a :class:`ROS2Camera`
    using the conventional ``/hdas/camera_<name>/...`` topic.

Dummy mode:
    ``CameraMuxConfig.is_dummy=True`` skips all camera construction
    and :meth:`get_frames` returns ``np.zeros((H, W, 3), uint8)``
    dictionaries — used by ``is_dummy=True`` runs and CI fixtures.
"""

from __future__ import annotations

import logging
import queue
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from rlinf.envs.realworld.common.camera import (
    BaseCamera,
    CameraInfo,
    create_camera,
)
from rlinf.scheduler.hardware.robots.galaxea_r1_pro import CameraSpec

_logger = logging.getLogger(__name__)


@dataclass
class CameraMuxConfig:
    """Runtime configuration for :class:`GalaxeaR1ProCameraMux`."""

    cameras: list = field(default_factory=list)
    image_size: tuple[int, int] = (128, 128)
    soft_sync_window_ms: int = 33
    align_strategy: str = "latest"  # "latest" or "sync_window"
    fallback_to_ros2_on_usb_drop: bool = True
    usb_drop_consecutive_threshold: int = 3
    is_dummy: bool = False
    enable_depth_in_frames: bool = True

    def __post_init__(self):
        if isinstance(self.image_size, list):
            self.image_size = tuple(self.image_size)
        if self.align_strategy not in ("latest", "sync_window"):
            raise ValueError(
                f"align_strategy must be latest/sync_window, "
                f"got {self.align_strategy!r}"
            )
        coerced: list[CameraSpec] = []
        for spec in self.cameras:
            if isinstance(spec, CameraSpec):
                coerced.append(spec)
            elif isinstance(spec, dict):
                coerced.append(CameraSpec(**spec))
            else:
                raise TypeError(
                    f"CameraMuxConfig.cameras entries must be dict "
                    f"or CameraSpec; got {type(spec).__name__}: {spec!r}"
                )
        self.cameras = coerced


class GalaxeaR1ProCameraMux:
    """Multi-backend camera multiplexer for R1 Pro.

    See module docstring for design rationale.
    """

    def __init__(self, cfg: CameraMuxConfig):
        self._cfg = cfg
        self._cameras: dict[str, BaseCamera] = {}
        self._fallback_topics: dict[str, str] = {}
        self._usb_drop_counters: dict[str, int] = {}
        self._sync_window_reject_counter: int = 0
        self._sync_window_total_counter: int = 0
        if cfg.is_dummy:
            return
        for spec in cfg.cameras:
            try:
                self._cameras[spec.name] = self._build_one(spec)
                self._cameras[spec.name].open()
            except Exception as e:  # pragma: no cover - hardware-dependent
                _logger.error(
                    "Failed to open camera %s (backend=%s): %s",
                    spec.name, spec.backend, e,
                )
                raise
            if spec.backend == "usb_direct":
                self._fallback_topics[spec.name] = self._derive_fallback_topic(
                    spec
                )
                self._usb_drop_counters[spec.name] = 0

    @property
    def names(self) -> list[str]:
        return [s.name for s in self._cfg.cameras]

    @property
    def cfg(self) -> CameraMuxConfig:
        return self._cfg

    # ── Camera construction ─────────────────────────────────────

    def _build_one(self, spec: CameraSpec) -> BaseCamera:
        info = CameraInfo(
            name=spec.name,
            serial_number=spec.serial_number or "",
            camera_type="realsense" if spec.backend == "usb_direct" else "ros2",
            resolution=spec.resolution,
            fps=spec.fps,
            enable_depth=spec.enable_depth and self._cfg.enable_depth_in_frames,
            backend=spec.backend,
            rgb_topic=spec.rgb_topic,
            depth_topic=spec.depth_topic,
            stale_threshold_ms=spec.stale_threshold_ms,
            align_depth_to_color=spec.align_depth_to_color,
        )
        return create_camera(info)

    def _derive_fallback_topic(self, spec: CameraSpec) -> str:
        # Conventional Galaxea ROS 2 topic for wrist cameras: when the
        # USB cable from D405 runs back to the Orin, the existing
        # `realsense2_camera` node publishes here.
        return (
            f"/hdas/camera_{spec.name}/color/image_raw/compressed"
        )

    # ── Public API ──────────────────────────────────────────────

    def get_frames(
        self,
        soft_sync_window_ms: Optional[int] = None,
    ) -> dict[str, np.ndarray]:
        """Return the latest frame from each camera as a dict.

        Returns ``{name: HxWx{3,4} uint8/uint16}`` (depth is uint16
        appended as 4th channel when available).  In dummy mode
        returns zero matrices of the configured shape.
        """
        if self._cfg.is_dummy:
            return self._dummy_frames()

        win = (
            soft_sync_window_ms
            if soft_sync_window_ms is not None
            else self._cfg.soft_sync_window_ms
        )
        out: dict[str, np.ndarray] = {}
        ages: dict[str, float] = {}

        for name in list(self._cameras.keys()):
            cam = self._cameras[name]
            try:
                frame = cam.get_frame(timeout=2)
                age_ms = self._get_age_ms(cam)
                if age_ms > cam._camera_info.stale_threshold_ms:
                    self._note_stale(name)
                    continue
                ages[name] = age_ms
                out[name] = self._postprocess(frame)
                # Reset USB drop counter on success.
                if name in self._usb_drop_counters:
                    self._usb_drop_counters[name] = 0
            except queue.Empty:
                self._note_stale(name)
                continue
            except Exception as e:  # pragma: no cover - runtime error
                _logger.warning(
                    "Camera %s read failed: %s", name, e,
                )
                self._note_stale(name)
                continue

        # Soft-sync window: count rejections but do not drop.  RLPD /
        # async PPO are robust to occasional missing frame; main
        # purpose is to surface drift via metrics.
        if (
            self._cfg.align_strategy == "sync_window"
            and len(ages) >= 2
        ):
            self._sync_window_total_counter += 1
            t_max = max(ages.values())
            for name, age in ages.items():
                if (t_max - age) > win:
                    self._sync_window_reject_counter += 1
                    _logger.debug(
                        "sync_window: %s out-of-window age=%.1fms (max=%.1fms)",
                        name, age, t_max,
                    )
                    break

        return out

    def get_metrics(self) -> dict:
        """Snapshot metrics for ``MetricLogger`` integration."""
        m: dict = {}
        for name in self.names:
            cam = self._cameras.get(name)
            if cam is None:
                continue
            m[f"camera/{name}_frame_age_ms"] = self._get_age_ms(cam)
        m["camera/sync_window_reject_rate"] = (
            self._sync_window_reject_counter
            / max(self._sync_window_total_counter, 1)
        )
        return m

    def close(self) -> None:
        for cam in list(self._cameras.values()):
            try:
                cam.close()
            except Exception:
                pass
        self._cameras.clear()

    # ── Internal helpers ────────────────────────────────────────

    def _dummy_frames(self) -> dict[str, np.ndarray]:
        h, w = self._cfg.image_size
        out: dict[str, np.ndarray] = {}
        for spec in self._cfg.cameras:
            if (
                spec.enable_depth
                and self._cfg.enable_depth_in_frames
            ):
                # 3 RGB + 1 depth (uint16 in mm) -> we keep uint8 RGB
                # and an extra channel for shape compatibility.
                rgb = np.zeros((h, w, 3), dtype=np.uint8)
                depth = np.zeros((h, w, 1), dtype=np.uint16)
                # Concatenate produces uint16 (numpy promotes); cast
                # to uint16 explicitly to avoid downstream surprises.
                stacked = np.concatenate(
                    [rgb.astype(np.uint16), depth], axis=-1,
                )
                out[spec.name] = stacked
            else:
                out[spec.name] = np.zeros((h, w, 3), dtype=np.uint8)
        return out

    def _postprocess(self, frame: np.ndarray) -> np.ndarray:
        # Defensive check: opencv import is heavy; only load when needed.
        import cv2  # type: ignore[import]

        h, w = frame.shape[:2]
        crop = min(h, w)
        sx = (w - crop) // 2
        sy = (h - crop) // 2
        cropped = frame[sy : sy + crop, sx : sx + crop]
        target_h, target_w = self._cfg.image_size
        if cropped.shape[2] == 3:
            return cv2.resize(cropped, (target_w, target_h))
        # 4-channel (BGR + depth): resize separately to avoid mixed dtype.
        rgb = cv2.resize(cropped[:, :, :3], (target_w, target_h))
        depth = cv2.resize(
            cropped[:, :, 3:],
            (target_w, target_h),
            interpolation=cv2.INTER_NEAREST,
        )
        return np.concatenate([rgb.astype(depth.dtype), depth], axis=-1)

    def _get_age_ms(self, cam: BaseCamera) -> float:
        # ROS2Camera exposes a fresh frame age; USB direct returns
        # ~5ms typical (no exposed clock), use a conservative default.
        if hasattr(cam, "get_frame_age_ms"):
            return cam.get_frame_age_ms()
        return 5.0

    def _note_stale(self, name: str) -> None:
        if name not in self._usb_drop_counters:
            return  # Not a USB-direct camera; ROS2 stale doesn't trigger fallback.
        self._usb_drop_counters[name] += 1
        if (
            self._cfg.fallback_to_ros2_on_usb_drop
            and self._usb_drop_counters[name]
            >= self._cfg.usb_drop_consecutive_threshold
        ):
            self._switch_to_ros2(name)

    def _switch_to_ros2(self, name: str) -> None:
        topic = self._fallback_topics.get(name)
        if not topic:
            return
        _logger.warning(
            "Camera %s USB direct dropped %d consecutive reads; "
            "falling back to ROS 2 topic %s",
            name, self._usb_drop_counters[name], topic,
        )
        try:
            old = self._cameras[name]
            old.close()
        except Exception:
            pass
        info = CameraInfo(
            name=name,
            serial_number="",
            camera_type="ros2",
            resolution=(640, 480),
            fps=15,
            enable_depth=False,
            backend="ros2",
            rgb_topic=topic,
        )
        try:
            self._cameras[name] = create_camera(info)
            self._cameras[name].open()
            self._usb_drop_counters[name] = 0
        except Exception as e:  # pragma: no cover - runtime
            _logger.error(
                "Camera %s ROS 2 fallback failed: %s", name, e,
            )
