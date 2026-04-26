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

"""Unit tests for :mod:`rlinf.scheduler.hardware.robots.galaxea_r1_pro`.

These tests run without ROS 2, ``rclpy`` or ``pyrealsense2``.
"""

from __future__ import annotations

import pytest

from rlinf.scheduler.hardware.robots.galaxea_r1_pro import (
    CameraSpec,
    GalaxeaR1ProConfig,
    GalaxeaR1ProHWInfo,
    GalaxeaR1ProRobot,
)


def test_hw_type_constant():
    assert GalaxeaR1ProRobot.HW_TYPE == "GalaxeaR1Pro"


def test_camera_spec_defaults_and_validation():
    spec = CameraSpec(name="wrist_right", backend="usb_direct",
                      serial_number="123", enable_depth=True)
    assert spec.name == "wrist_right"
    assert spec.backend == "usb_direct"
    assert spec.resolution == (640, 480)
    assert spec.fps == 30
    assert spec.enable_depth is True
    # Resolution from list should be coerced to tuple.
    spec2 = CameraSpec(name="head", backend="ros2",
                       rgb_topic="/x", resolution=[1280, 720])
    assert spec2.resolution == (1280, 720)


def test_camera_spec_invalid_backend():
    with pytest.raises(ValueError):
        CameraSpec(name="x", backend="nope")


def test_galaxea_config_dummy_disable_validate():
    cfg = GalaxeaR1ProConfig(
        node_rank=0,
        ros_domain_id=72,
        use_right_arm=True,
        cameras=[
            {"name": "wrist_right", "backend": "usb_direct",
             "serial_number": "abc"},
            {"name": "head_left", "backend": "ros2",
             "rgb_topic": "/hdas/camera_head/left_raw/image_raw_color/compressed"},
        ],
        wrist_direct_camera_serials={"wrist_right": "abc"},
        controller_node_rank=1,
        camera_node_rank=0,
        disable_validate=True,
    )
    # cameras coerced to CameraSpec
    assert all(isinstance(c, CameraSpec) for c in cfg.cameras)
    assert cfg.cameras[0].backend == "usb_direct"
    assert cfg.cameras[1].backend == "ros2"
    # serials coerced to dict[str, str]
    assert cfg.wrist_direct_camera_serials == {"wrist_right": "abc"}


def test_galaxea_config_invalid_mobiman_mode():
    with pytest.raises(ValueError):
        GalaxeaR1ProConfig(node_rank=0, mobiman_launch_mode="bogus")


def test_enumerate_returns_none_for_other_node():
    cfg = GalaxeaR1ProConfig(
        node_rank=0, disable_validate=True,
    )
    # node_rank=99 has no matching config.
    res = GalaxeaR1ProRobot.enumerate(node_rank=99, configs=[cfg])
    assert res is None


def test_enumerate_returns_resource_with_matching_config():
    cfg = GalaxeaR1ProConfig(
        node_rank=0,
        disable_validate=True,
    )
    res = GalaxeaR1ProRobot.enumerate(node_rank=0, configs=[cfg])
    assert res is not None
    assert res.type == GalaxeaR1ProRobot.HW_TYPE
    assert res.count == 1
    info = res.infos[0]
    assert isinstance(info, GalaxeaR1ProHWInfo)
    assert info.config is cfg


def test_enumerate_handles_empty_configs():
    res = GalaxeaR1ProRobot.enumerate(node_rank=0, configs=None)
    assert res is None
    res2 = GalaxeaR1ProRobot.enumerate(node_rank=0, configs=[])
    assert res2 is None
