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

"""Unit tests for :class:`GalaxeaR1ProCameraMux` (dummy mode).

Dummy mode is the only path runnable without rclpy / pyrealsense2.
Real-hardware paths are validated via the hardware-in-loop suite
(see imp1.md §7.4).
"""

from __future__ import annotations

import numpy as np
import pytest

from rlinf.envs.realworld.galaxear.r1_pro_camera_mux import (
    CameraMuxConfig,
    GalaxeaR1ProCameraMux,
)
from rlinf.scheduler.hardware.robots.galaxea_r1_pro import CameraSpec


def test_dummy_returns_zero_frames_for_each_camera():
    mux = GalaxeaR1ProCameraMux(
        CameraMuxConfig(
            cameras=[
                CameraSpec(name="wrist_right", backend="usb_direct",
                           serial_number="abc"),
                CameraSpec(name="head_left", backend="ros2",
                           rgb_topic="/x"),
            ],
            image_size=(64, 96),
            is_dummy=True,
        )
    )
    frames = mux.get_frames()
    assert set(frames.keys()) == {"wrist_right", "head_left"}
    for k, arr in frames.items():
        assert arr.shape[:2] == (64, 96)
        assert arr.shape[2] >= 3
        assert int(arr.sum()) == 0


def test_dummy_with_depth_returns_4_channel():
    mux = GalaxeaR1ProCameraMux(
        CameraMuxConfig(
            cameras=[
                CameraSpec(name="wrist_right", backend="usb_direct",
                           serial_number="abc", enable_depth=True),
            ],
            image_size=(32, 48),
            is_dummy=True,
        )
    )
    frames = mux.get_frames()
    arr = frames["wrist_right"]
    assert arr.shape == (32, 48, 4)


def test_invalid_align_strategy_raises():
    with pytest.raises(ValueError):
        CameraMuxConfig(align_strategy="bogus")


def test_dict_cameras_coerced_to_camera_spec():
    cfg = CameraMuxConfig(cameras=[
        {"name": "head_left", "backend": "ros2", "rgb_topic": "/x"},
    ], is_dummy=True)
    assert isinstance(cfg.cameras[0], CameraSpec)
    assert cfg.cameras[0].name == "head_left"
    assert cfg.cameras[0].backend == "ros2"


def test_close_is_idempotent_in_dummy_mode():
    mux = GalaxeaR1ProCameraMux(
        CameraMuxConfig(
            cameras=[CameraSpec(name="x", backend="usb_direct",
                                serial_number="a")],
            is_dummy=True,
        )
    )
    mux.close()
    mux.close()  # second call should not raise


def test_get_metrics_dummy():
    mux = GalaxeaR1ProCameraMux(
        CameraMuxConfig(cameras=[], is_dummy=True)
    )
    m = mux.get_metrics()
    assert "camera/sync_window_reject_rate" in m


def test_names_property():
    mux = GalaxeaR1ProCameraMux(
        CameraMuxConfig(
            cameras=[
                CameraSpec(name="a", backend="usb_direct", serial_number="x"),
                CameraSpec(name="b", backend="ros2", rgb_topic="/y"),
            ],
            is_dummy=True,
        )
    )
    assert mux.names == ["a", "b"]
