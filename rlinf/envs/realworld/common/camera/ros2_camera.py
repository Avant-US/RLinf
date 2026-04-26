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

"""ROS 2 image-topic camera adapter.

This adapter subscribes to a ``sensor_msgs/CompressedImage`` (preferred,
JPEG decoded with TurboJPEG / OpenCV) or ``sensor_msgs/Image`` (raw
encoding) topic for RGB and an optional ``sensor_msgs/Image`` depth
topic (``16UC1`` aligned depth, e.g. RealSense D405 via Orin's
``realsense2_camera`` node, or ``32FC1`` depth from the head GMSL
``signal_camera_node``).

All ROS 2 imports (``rclpy`` / ``sensor_msgs`` / ``cv_bridge`` /
``turbojpeg``) are deferred until :meth:`_open_ros2`, allowing the
module to import on machines without ROS 2 installed (the dummy
``is_dummy=True`` path or any test runner).

The class exposes the same :class:`BaseCamera` contract as
:class:`RealSenseCamera` and :class:`ZEDCamera` so it is fully
interchangeable through the :func:`create_camera` factory.

A small extension over :class:`BaseCamera`: :meth:`get_frame_age_ms`
returns the age (in milliseconds) of the most recent frame, computed
from ``header.stamp``.  Used by
:class:`GalaxeaR1ProCameraMux` for soft-sync windowing.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import numpy as np

from .base_camera import BaseCamera, CameraInfo

_logger = logging.getLogger(__name__)


class ROS2Camera(BaseCamera):
    """Camera capture from a ROS 2 image topic.

    Args:
        camera_info: A :class:`CameraInfo` whose ``rgb_topic`` is set
            (and ``backend == "ros2"`` recommended).  Optional fields:
            ``depth_topic`` (enables depth stream), ``enable_depth``,
            ``stale_threshold_ms``.

    Raises:
        ValueError: When ``camera_info.rgb_topic`` is empty.

    Notes:
        * The instance creates its own :class:`rclpy.Node` and a
          :class:`rclpy.executors.SingleThreadedExecutor` running on a
          dedicated background thread, so it does not interfere with
          the controller's executor.
        * Multiple ROS2Camera instances coexist on the same process
          (no global state).  ``rclpy.init()`` is called once and
          guarded by ``rclpy.ok()``.
        * On every received frame, the latest decoded ndarray is
          stored.  :meth:`_read_frame` returns the latest available
          frame (BGR for color, ``[bgr, depth16]`` 4-channel stack
          when ``enable_depth`` is true and a depth frame has been
          received at least once).
    """

    _DEFAULT_QOS_DEPTH = 1
    _SPIN_PERIOD_S = 0.005

    def __init__(self, camera_info: CameraInfo):
        super().__init__(camera_info)
        if not camera_info.rgb_topic:
            raise ValueError(
                f"ROS2Camera({camera_info.name!r}) requires camera_info.rgb_topic"
            )
        self._rgb_topic: str = camera_info.rgb_topic
        self._depth_topic: Optional[str] = camera_info.depth_topic
        self._enable_depth: bool = bool(
            camera_info.enable_depth and camera_info.depth_topic
        )

        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_rgb_stamp_s: Optional[float] = None
        self._latest_depth_stamp_s: Optional[float] = None
        self._frame_lock = threading.Lock()

        # Lazy-initialised ROS 2 handles.
        self._rclpy = None
        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_running = False
        self._jpeg = None  # PyTurboJPEG handle, optional

        self._open_ros2()

    # ── Lazy ROS 2 init ──────────────────────────────────────────

    def _open_ros2(self) -> None:
        import rclpy  # type: ignore[import]
        from rclpy.executors import SingleThreadedExecutor  # type: ignore
        from rclpy.qos import qos_profile_sensor_data  # type: ignore
        from sensor_msgs.msg import CompressedImage, Image  # type: ignore

        if not rclpy.ok():
            rclpy.init(args=[])
        self._rclpy = rclpy
        node_name = (
            f"rlinf_ros2_camera_{self._camera_info.name}_{id(self)}"
        )
        # rclpy node names cannot contain slashes / colons.
        node_name = node_name.replace("-", "_").replace(":", "_")
        self._node = rclpy.create_node(node_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        if self._rgb_topic.endswith("/compressed"):
            self._node.create_subscription(
                CompressedImage,
                self._rgb_topic,
                self._on_rgb_compressed,
                qos_profile_sensor_data,
            )
        else:
            self._node.create_subscription(
                Image,
                self._rgb_topic,
                self._on_rgb_raw,
                qos_profile_sensor_data,
            )
        if self._enable_depth and self._depth_topic:
            self._node.create_subscription(
                Image,
                self._depth_topic,
                self._on_depth,
                qos_profile_sensor_data,
            )

        self._spin_running = True
        self._spin_thread = threading.Thread(
            target=self._spin_loop,
            name=f"ros2_camera_spin_{self._camera_info.name}",
            daemon=True,
        )
        self._spin_thread.start()

    def _spin_loop(self) -> None:
        try:
            while self._spin_running and self._rclpy is not None and self._rclpy.ok():
                # spin_once with a short timeout so we can exit cleanly.
                self._executor.spin_once(timeout_sec=self._SPIN_PERIOD_S)
        except Exception as e:  # pragma: no cover - defensive logging
            _logger.error(
                "ROS2Camera(%s) spin thread crashed: %s",
                self._camera_info.name, e,
            )

    # ── ROS 2 callbacks ──────────────────────────────────────────

    def _on_rgb_compressed(self, msg) -> None:
        try:
            arr = self._decode_jpeg_bytes(bytes(msg.data))
        except Exception as e:  # pragma: no cover - decode error path
            _logger.warning(
                "ROS2Camera(%s) JPEG decode failed: %s",
                self._camera_info.name, e,
            )
            return
        with self._frame_lock:
            self._latest_rgb = arr
            self._latest_rgb_stamp_s = self._stamp_to_seconds(msg.header.stamp)

    def _on_rgb_raw(self, msg) -> None:
        encoding = (msg.encoding or "").lower()
        if encoding in ("bgr8", ""):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            ).copy()
        elif encoding == "rgb8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
            arr = arr[:, :, ::-1].copy()  # RGB -> BGR
        elif encoding == "mono8":
            mono = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width
            )
            arr = np.repeat(mono[:, :, None], 3, axis=2).copy()
        else:
            _logger.warning(
                "ROS2Camera(%s) unsupported encoding %s",
                self._camera_info.name, encoding,
            )
            return
        with self._frame_lock:
            self._latest_rgb = arr
            self._latest_rgb_stamp_s = self._stamp_to_seconds(msg.header.stamp)

    def _on_depth(self, msg) -> None:
        encoding = (msg.encoding or "").lower()
        if encoding == "16uc1":
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width
            ).copy()
        elif encoding == "32fc1":
            arr_f = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width
            )
            # Convert metres -> millimetres uint16 to keep a uniform
            # depth representation across head (32FC1 metres) and
            # wrist (16UC1 millimetres).
            arr = (np.clip(arr_f * 1000.0, 0, 65535)).astype(np.uint16)
        else:
            _logger.warning(
                "ROS2Camera(%s) unsupported depth encoding %s",
                self._camera_info.name, encoding,
            )
            return
        with self._frame_lock:
            self._latest_depth = arr
            self._latest_depth_stamp_s = self._stamp_to_seconds(msg.header.stamp)

    # ── Decoders ─────────────────────────────────────────────────

    def _decode_jpeg_bytes(self, payload: bytes) -> np.ndarray:
        # Prefer TurboJPEG when available (~2-3x faster than cv2).
        if self._jpeg is None:
            try:
                from turbojpeg import TurboJPEG  # type: ignore[import]

                self._jpeg = TurboJPEG()
            except Exception:
                self._jpeg = False  # sentinel; never try again
        if self._jpeg:
            try:
                return self._jpeg.decode(payload)
            except Exception:
                pass
        import cv2  # type: ignore[import]

        buf = np.frombuffer(payload, dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        # builtin_interfaces/Time has sec (int32) and nanosec (uint32).
        return float(getattr(stamp, "sec", 0)) + float(
            getattr(stamp, "nanosec", 0)
        ) * 1e-9

    # ── BaseCamera contract ─────────────────────────────────────

    def _read_frame(self) -> tuple[bool, Optional[np.ndarray]]:
        with self._frame_lock:
            rgb = self._latest_rgb
            depth = self._latest_depth
        if rgb is None:
            return False, None
        if self._enable_depth and depth is not None:
            depth_3d = np.expand_dims(depth, axis=2)
            return True, np.concatenate([rgb, depth_3d], axis=-1)
        return True, rgb

    def _close_device(self) -> None:
        self._spin_running = False
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.5)
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        # Do NOT call rclpy.shutdown() here: other ROS2Camera /
        # GalaxeaR1ProController instances in the same process may
        # still be using rclpy.

    # ── Frame-age helper used by GalaxeaR1ProCameraMux ───────────

    def get_frame_age_ms(self) -> float:
        """Return age (ms) of the latest RGB frame relative to wall clock.

        Returns ``inf`` if no frame received yet.  Uses the ROS header
        stamp where possible.
        """
        if self._latest_rgb_stamp_s is None:
            return float("inf")
        return max(0.0, (time.time() - self._latest_rgb_stamp_s) * 1000.0)
