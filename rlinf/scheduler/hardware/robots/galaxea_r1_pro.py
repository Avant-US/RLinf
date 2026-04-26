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

"""Galaxea R1 Pro hardware registration for RLinf scheduler.

The R1 Pro is a 26-DoF dual-arm humanoid: dual A2 7-DoF arms, dual G1
grippers, 4-DoF torso (T1-T4), 6-DoF chassis (3 steering wheels), head
GMSL stereo cameras, optional wrist depth cameras (Intel RealSense
D405), 360 deg LiDAR, IMU x 2, BMS and a wireless controller (SWA-D).

This module mirrors the structure of
:mod:`rlinf.scheduler.hardware.robots.franka` and
:mod:`rlinf.scheduler.hardware.robots.xsquare`.  The cross-host
``GalaxeaR1ProController`` worker is launched separately from
:class:`rlinf.envs.realworld.galaxear.r1_pro_controller.GalaxeaR1ProController`.

Naming conventions (per the design doc and team rule):

* file:  ``galaxea_r1_pro.py``
* class prefix: ``GalaxeaR1Pro*``
* hardware type: ``"GalaxeaR1Pro"``
"""

from __future__ import annotations

import importlib
import ipaddress
import logging
import os
import shutil
import subprocess
import warnings
from dataclasses import dataclass, field
from typing import Optional

from ..hardware import (
    Hardware,
    HardwareConfig,
    HardwareInfo,
    HardwareResource,
    NodeHardwareConfig,
)

logger = logging.getLogger(__name__)


@dataclass
class CameraSpec:
    """Per-camera entry inside :class:`GalaxeaR1ProConfig`.

    The R1 Pro has 9+ physical cameras (head x 2, wrist x 2 optional,
    chassis x 5).  Each camera can independently choose between two
    backends:

    * ``"usb_direct"``: use the local Intel RealSense / ZED SDK to read
      frames in-process.  Suitable for wrist D405 connected directly to
      the GPU server via USB / USB-AOC cable.
    * ``"ros2"``: subscribe to a ROS 2 image topic (compressed RGB and
      optional 16UC1/32FC1 depth).  Suitable for head GMSL cameras (which
      do not expose a USB interface) and as a fallback for any wrist
      camera.

    Args:
        name: Logical name used in observation dict, e.g. ``"wrist_right"``.
        backend: ``"usb_direct"`` or ``"ros2"``.
        rgb_topic: ROS 2 topic name for RGB image.  Required when
            ``backend == "ros2"``.  Examples:
            ``/hdas/camera_head/left_raw/image_raw_color/compressed``,
            ``/hdas/camera_wrist_right/color/image_raw/compressed``.
        depth_topic: Optional ROS 2 depth topic.  Compatible encodings
            are ``16UC1`` (D405 aligned depth) or ``32FC1`` (head depth).
        serial_number: RealSense / ZED serial number.  Required when
            ``backend == "usb_direct"``.
        resolution: ``(width, height)`` capture resolution.
        fps: Capture frame rate.
        enable_depth: Whether to capture the depth stream.
        stale_threshold_ms: Frames older than this (wall clock minus
            ROS header stamp) are rejected by ``GalaxeaR1ProCameraMux``.
        align_depth_to_color: For RealSense D405; align depth to color.
    """

    name: str
    backend: str = "ros2"
    rgb_topic: Optional[str] = None
    depth_topic: Optional[str] = None
    serial_number: Optional[str] = None
    resolution: tuple[int, int] = (640, 480)
    fps: int = 30
    enable_depth: bool = False
    stale_threshold_ms: float = 200.0
    align_depth_to_color: bool = True

    def __post_init__(self):
        # YAML may pass tuple as list
        if isinstance(self.resolution, list):
            self.resolution = tuple(self.resolution)
        backend = self.backend.lower()
        if backend not in ("usb_direct", "ros2"):
            raise ValueError(
                f"CameraSpec.backend must be 'usb_direct' or 'ros2', got {backend!r}"
            )
        self.backend = backend


@dataclass
class GalaxeaR1ProHWInfo(HardwareInfo):
    """Hardware information for a Galaxea R1 Pro robot."""

    config: "GalaxeaR1ProConfig" = None


@Hardware.register()
class GalaxeaR1ProRobot(Hardware):
    """Hardware policy for Galaxea R1 Pro.

    Mirrors :class:`FrankaRobot` in structure but performs ROS 2
    specific validation: rclpy import, Galaxea SDK install path,
    ``ROS_DOMAIN_ID``, optional ICMP ping, USB-direct D405 serials and
    a soft check on ``can0`` link state (Orin-only).
    """

    HW_TYPE = "GalaxeaR1Pro"
    ROBOT_PING_COUNT: int = 2
    ROBOT_PING_TIMEOUT: int = 1  # in seconds

    @classmethod
    def enumerate(
        cls,
        node_rank: int,
        configs: Optional[list["GalaxeaR1ProConfig"]] = None,
    ) -> Optional[HardwareResource]:
        """Enumerate R1 Pro resources on a node.

        Args:
            node_rank: Rank of the node being enumerated.
            configs: Hardware configs declared in YAML.

        Returns:
            HardwareResource holding one or more
            :class:`GalaxeaR1ProHWInfo`, or ``None`` when no config
            targets this node.
        """
        if configs is None:
            return None

        matched: list[GalaxeaR1ProConfig] = [
            c
            for c in configs
            if isinstance(c, GalaxeaR1ProConfig) and c.node_rank == node_rank
        ]
        if not matched:
            return None

        infos: list[GalaxeaR1ProHWInfo] = []
        for cfg in matched:
            infos.append(
                GalaxeaR1ProHWInfo(
                    type=cls.HW_TYPE,
                    model=cls.HW_TYPE,
                    config=cfg,
                )
            )

            if cfg.disable_validate:
                continue

            cls._validate_rclpy(node_rank)
            cls._validate_galaxea_install(node_rank, cfg.galaxea_install_path)
            cls._validate_ros_domain_id(node_rank, cfg.ros_domain_id)
            if cfg.robot_ip:
                cls._validate_connectivity(cfg.robot_ip, node_rank)
            cls._validate_d405_serials(node_rank, cfg.wrist_direct_camera_serials)
            cls._validate_can_link(node_rank)

        return HardwareResource(type=cls.HW_TYPE, infos=infos)

    # ── Validators ─────────────────────────────────────────────

    @classmethod
    def _validate_rclpy(cls, node_rank: int) -> None:
        try:
            importlib.import_module("rclpy")
        except ModuleNotFoundError as e:
            raise ModuleNotFoundError(
                f"Node {node_rank}: 'rclpy' not importable. Did you "
                f"`source /opt/ros/humble/setup.bash` and "
                f"`source $GALAXEA_INSTALL_PATH/setup.bash` BEFORE "
                f"`ray start`? "
                f"For dummy / CI runs set `disable_validate: true`."
            ) from e

    @classmethod
    def _validate_galaxea_install(cls, node_rank: int, path: str) -> None:
        path = os.path.expanduser(path)
        if not os.path.exists(os.path.join(path, "setup.bash")):
            warnings.warn(
                f"Node {node_rank}: GALAXEA_INSTALL_PATH={path!r} "
                f"does not contain setup.bash. The controller worker "
                f"will fail when launched on this node unless it "
                f"runs entirely in is_dummy mode."
            )

    @classmethod
    def _validate_ros_domain_id(cls, node_rank: int, domain_id: int) -> None:
        env_did = os.environ.get("ROS_DOMAIN_ID")
        if env_did is None:
            warnings.warn(
                f"Node {node_rank}: ROS_DOMAIN_ID not set in env; "
                f"YAML wants {domain_id}. The controller worker will "
                f"export it on launch."
            )
            return
        try:
            if int(env_did) != int(domain_id):
                warnings.warn(
                    f"Node {node_rank}: env ROS_DOMAIN_ID={env_did} "
                    f"differs from YAML config {domain_id}; controller "
                    f"will use the YAML value."
                )
        except ValueError:
            warnings.warn(
                f"Node {node_rank}: env ROS_DOMAIN_ID={env_did!r} is "
                f"not an integer; ignoring."
            )

    @classmethod
    def _validate_connectivity(cls, robot_ip: str, node_rank: int) -> None:
        try:
            ipaddress.ip_address(robot_ip)
        except ValueError as e:
            raise ValueError(f"Invalid robot_ip {robot_ip!r}: {e}")
        try:
            from icmplib import ping  # type: ignore[import]
        except ImportError:
            warnings.warn("icmplib not installed; skipping ping check.")
            return
        try:
            res = ping(
                robot_ip,
                count=cls.ROBOT_PING_COUNT,
                timeout=cls.ROBOT_PING_TIMEOUT,
            )
            if not res.is_alive:
                raise ConnectionError(
                    f"Cannot reach R1 Pro at IP {robot_ip} from "
                    f"node rank {node_rank}."
                )
            logger.info(
                "R1 Pro at %s reachable from node %d, RTT avg=%.1fms",
                robot_ip, node_rank, getattr(res, "avg_rtt", 0.0),
            )
        except PermissionError:
            warnings.warn(
                f"Permission denied pinging {robot_ip} from node "
                f"{node_rank}; skipping."
            )
        except Exception as e:
            warnings.warn(
                f"Unexpected error pinging {robot_ip} from node "
                f"{node_rank}: {e}; skipping."
            )

    @classmethod
    def _validate_d405_serials(
        cls, node_rank: int, serials: dict[str, str]
    ) -> None:
        if not serials:
            return
        try:
            import pyrealsense2 as rs  # type: ignore[import]
        except ImportError as e:
            raise ImportError(
                f"Node {node_rank}: pyrealsense2 missing but USB-direct "
                f"D405 serials configured: {serials}. Install with "
                f"`pip install pyrealsense2` or set "
                f"`disable_validate: true` in YAML to skip this check."
            ) from e
        connected = {
            d.get_info(rs.camera_info.serial_number) for d in rs.context().devices
        }
        for name, sn in serials.items():
            if sn and sn not in connected:
                raise ValueError(
                    f"Node {node_rank}: USB-direct D405 '{name}' "
                    f"serial {sn!r} is not connected. Available "
                    f"RealSense devices: {sorted(connected)}."
                )

    @classmethod
    def _validate_can_link(cls, node_rank: int) -> None:
        # Soft check; controller will hard-check at startup.  Only run
        # `ip link show can0` when the binary is available; CAN is only
        # present on Orin, so missing on GPU server is normal.
        if shutil.which("ip") is None:
            return
        try:
            res = subprocess.run(
                ["ip", "link", "show", "can0"],
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            if res.returncode != 0:
                # No can0 on this node (likely the GPU server). Silent.
                return
            if "UP" not in res.stdout:
                warnings.warn(
                    f"Node {node_rank}: can0 link is DOWN. Run "
                    f"`bash ~/can.sh` on Orin before starting the "
                    f"controller worker."
                )
        except Exception:
            # Any failure here is non-fatal at enumerate time.
            return


@NodeHardwareConfig.register_hardware_config(GalaxeaR1ProRobot.HW_TYPE)
@dataclass
class GalaxeaR1ProConfig(HardwareConfig):
    """Configuration for a Galaxea R1 Pro robot.

    The same config dataclass is parsed once per ``hardware.configs``
    entry in YAML.  Two entries (one with ``node_rank: 0`` for the GPU
    server, one with ``node_rank: 1`` for the Orin) let
    :meth:`GalaxeaR1ProRobot.enumerate` perform node-specific
    validation.

    Args:
        robot_ip: Optional IP for the Orin. When provided, validated
            with ICMP ping during enumerate.
        ros_domain_id: ROS 2 ``ROS_DOMAIN_ID`` (Galaxea default 72).
        ros_localhost_only: When False (default), allows cross-host
            DDS discovery — required if controller and EnvWorker live
            on different machines.
        galaxea_install_path: Filesystem path of the Galaxea SDK
            ``install/`` directory; checked for ``setup.bash``.
        use_left_arm / use_right_arm / use_torso / use_chassis: Stage
            flags consumed by the controller and SafetySupervisor.
        cameras: List of :class:`CameraSpec` entries.
        wrist_direct_camera_serials: Map ``name -> serial_number`` of
            USB-direct D405 cameras (default empty for the all-ROS2
            variant).
        controller_node_rank: Rank where ``GalaxeaR1ProController``
            should run.  Default ``None`` makes the env worker decide
            (typically Orin = 1).
        camera_node_rank: Rank that performs camera capture.  USB
            direct cameras must run on the node where the cable is
            physically connected.
        mobiman_launch_mode: ``"pose"`` (default; relaxed_ik) or
            ``"joint"`` (joint tracker, used for reset).
        bms_low_battery_threshold: Percentage below which the
            SafetySupervisor escalates to SAFE_STOP.
        stale_threshold_ms: Soft threshold for HDAS feedback freshness.
        disable_validate: Skip all enumerate-time checks (CI / dummy).
    """

    robot_ip: Optional[str] = None
    ros_domain_id: int = 72
    ros_localhost_only: bool = False
    galaxea_install_path: str = "~/galaxea/install"

    use_left_arm: bool = False
    use_right_arm: bool = True
    use_torso: bool = False
    use_chassis: bool = False

    cameras: list = field(default_factory=list)
    wrist_direct_camera_serials: dict = field(default_factory=dict)

    controller_node_rank: Optional[int] = None
    camera_node_rank: Optional[int] = None

    mobiman_launch_mode: str = "pose"
    bms_low_battery_threshold: float = 25.0
    stale_threshold_ms: float = 200.0
    disable_validate: bool = False

    def __post_init__(self):
        super().__post_init__()
        # Coerce camera dicts (from YAML) to CameraSpec.
        coerced: list[CameraSpec] = []
        for spec in self.cameras:
            if isinstance(spec, CameraSpec):
                coerced.append(spec)
            elif isinstance(spec, dict):
                coerced.append(CameraSpec(**spec))
            else:
                raise TypeError(
                    f"GalaxeaR1ProConfig.cameras entries must be dict "
                    f"or CameraSpec; got {type(spec).__name__}: {spec!r}"
                )
        self.cameras = coerced

        # Allow YAML to write `wrist_direct_camera_serials` as a list.
        if isinstance(self.wrist_direct_camera_serials, list):
            self.wrist_direct_camera_serials = {
                f"wrist_{i}": str(sn)
                for i, sn in enumerate(self.wrist_direct_camera_serials)
            }
        elif self.wrist_direct_camera_serials:
            self.wrist_direct_camera_serials = {
                str(k): str(v)
                for k, v in self.wrist_direct_camera_serials.items()
            }

        if self.mobiman_launch_mode not in ("pose", "joint", "hybrid"):
            raise ValueError(
                f"mobiman_launch_mode must be one of pose/joint/hybrid, "
                f"got {self.mobiman_launch_mode!r}"
            )
