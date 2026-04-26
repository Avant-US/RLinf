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

"""Light-weight tests for :class:`ROS2Camera` that do NOT require
``rclpy`` to be installed.

We exercise:
* :func:`create_camera` factory dispatch for ``backend="ros2"``.
* JPEG / 16UC1 / 32FC1 decode routines via mocked attribute access on
  a manually-constructed :class:`ROS2Camera` (the heavy ``_open_ros2``
  path is patched out in :func:`_make_decoder`).
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from rlinf.envs.realworld.common.camera import CameraInfo, create_camera


def _make_decoder():
    """Construct a ROS2Camera bypassing the rclpy init path.

    Returns the instance; only the decode paths are exercised.
    """
    from rlinf.envs.realworld.common.camera.ros2_camera import ROS2Camera

    info = CameraInfo(
        name="head_left",
        serial_number="",
        camera_type="ros2",
        backend="ros2",
        rgb_topic="/hdas/camera_head/left_raw/image_raw_color/compressed",
        depth_topic=None,
        enable_depth=False,
    )
    with patch.object(ROS2Camera, "_open_ros2", lambda self: None):
        cam = ROS2Camera(info)
    return cam


def test_create_camera_dispatches_to_ros2_when_backend_ros2():
    info = CameraInfo(
        name="head_left",
        serial_number="",
        camera_type="ros2",
        backend="ros2",
        rgb_topic="/hdas/camera_head/left_raw/image_raw_color/compressed",
    )
    # We don't want to actually open ROS 2 in the test, so just patch
    # the open method.
    from rlinf.envs.realworld.common.camera.ros2_camera import ROS2Camera

    with patch.object(ROS2Camera, "_open_ros2", lambda self: None):
        cam = create_camera(info)
    assert isinstance(cam, ROS2Camera)


def test_decode_rgb_raw_bgr8():
    cam = _make_decoder()
    msg = SimpleNamespace(
        encoding="bgr8",
        height=2,
        width=3,
        data=np.array([[10, 20, 30], [40, 50, 60], [70, 80, 90],
                       [100, 110, 120], [130, 140, 150], [160, 170, 180]],
                      dtype=np.uint8).tobytes(),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
    )
    cam._on_rgb_raw(msg)
    assert cam._latest_rgb is not None
    assert cam._latest_rgb.shape == (2, 3, 3)
    assert cam._latest_rgb_stamp_s == 1.0


def test_decode_rgb_raw_rgb8_swaps_channels():
    cam = _make_decoder()
    msg = SimpleNamespace(
        encoding="rgb8",
        height=1,
        width=1,
        data=np.array([255, 0, 0], dtype=np.uint8).tobytes(),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=500000000)),
    )
    cam._on_rgb_raw(msg)
    # rgb8 (255,0,0) red -> bgr (0,0,255)
    np.testing.assert_array_equal(
        cam._latest_rgb.reshape(-1), [0, 0, 255]
    )
    assert cam._latest_rgb_stamp_s == pytest.approx(2.5)


def test_decode_depth_16uc1():
    cam = _make_decoder()
    cam._enable_depth = True
    arr = np.array([[100, 200, 300], [400, 500, 600]], dtype=np.uint16)
    msg = SimpleNamespace(
        encoding="16UC1",
        height=2,
        width=3,
        data=arr.tobytes(),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=0)),
    )
    cam._on_depth(msg)
    np.testing.assert_array_equal(cam._latest_depth, arr)


def test_decode_depth_32fc1_metres_to_uint16_mm():
    cam = _make_decoder()
    cam._enable_depth = True
    arr = np.array([[0.10, 0.50, 1.00]], dtype=np.float32)
    msg = SimpleNamespace(
        encoding="32FC1",
        height=1,
        width=3,
        data=arr.tobytes(),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=4, nanosec=0)),
    )
    cam._on_depth(msg)
    # 0.5 m -> 500 mm (uint16)
    np.testing.assert_array_equal(cam._latest_depth, [[100, 500, 1000]])


def test_get_frame_age_ms_default_inf():
    cam = _make_decoder()
    assert cam.get_frame_age_ms() == float("inf")
