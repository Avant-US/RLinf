# Copyright 2025 The RLinf Authors.
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

from .base_camera import BaseCamera, CameraInfo
from .realsense_camera import RealSenseCamera

__all__ = [
    "BaseCamera",
    "CameraInfo",
    "RealSenseCamera",
    "create_camera",
]


def create_camera(camera_info: CameraInfo) -> BaseCamera:
    """Factory that instantiates the right camera backend from *camera_info*.

    Dispatch order:

    1. Explicit ``camera_info.backend == "ros2"`` (preferred for cross-host
       deployments such as Galaxea R1 Pro head GMSL or all-ROS2 variant)
       returns a :class:`ROS2Camera`.
    2. ``camera_info.camera_type == "ros2"`` is treated as a synonym for
       backend selection (legacy YAML compatibility).
    3. ``camera_info.camera_type == "zed"`` returns a :class:`ZEDCamera`
       (requires the Stereolabs ZED SDK / ``pyzed``).
    4. ``camera_info.camera_type in ("realsense", "rs")`` returns a
       :class:`RealSenseCamera` (requires ``pyrealsense2``).
    """
    backend = (camera_info.backend or "usb_direct").lower()
    camera_type = (camera_info.camera_type or "realsense").lower()
    if backend == "ros2" or camera_type == "ros2":
        from .ros2_camera import ROS2Camera

        return ROS2Camera(camera_info)
    if camera_type == "zed":
        from .zed_camera import ZEDCamera

        return ZEDCamera(camera_info)
    if camera_type in ("realsense", "rs"):
        return RealSenseCamera(camera_info)
    raise ValueError(
        f"Unsupported camera: backend={backend!r}, camera_type={camera_type!r}. "
        "Supported types: 'realsense', 'zed', 'ros2'."
    )
