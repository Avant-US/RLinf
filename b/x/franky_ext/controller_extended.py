"""Extended FrankyController with FrankaEnv-compatible API (local extension)."""

from __future__ import annotations

from typing import Optional

import numpy as np

from rlinf.envs.realworld.franka import franky_controller as fc
from rlinf.envs.realworld.franka.franky_controller import FrankyController
from rlinf.scheduler import Cluster, NodePlacementStrategy


class FrankyControllerExtended(FrankyController):
    """Adds move_arm, compliance reconfig, move_gripper, Franka Hand gripper."""

    def __init__(
        self,
        robot_ip: str,
        gripper_type: str = "franka",
        gripper_connection: Optional[str] = None,
    ):
        self._compliance_trans_k: float = fc._CART_TRANS_STIFFNESS
        self._compliance_rot_k: float = fc._CART_ROT_STIFFNESS
        self._compliance_tc: float = fc._CART_GAINS_TC
        super().__init__(
            robot_ip=robot_ip,
            gripper_type=gripper_type,
            gripper_connection=gripper_connection,
        )

    @staticmethod
    def launch_controller(
        robot_ip: str,
        env_idx: int = 0,
        node_rank: int = 0,
        worker_rank: int = 0,
        gripper_type: str = "franka",
        gripper_connection: Optional[str] = None,
    ):
        return FrankyControllerExtended.create_group(
            robot_ip, gripper_type, gripper_connection
        ).launch(
            cluster=Cluster(),
            placement_strategy=NodePlacementStrategy(node_ranks=[node_rank]),
            name=f"FrankyControllerExtended-{worker_rank}-{env_idx}",
        )

    def _build_gripper(self, gripper_type, gripper_connection, robot_ip):
        gt = (gripper_type or "franka").lower()
        if gt == "franka":
            from franky_ext.franka_libfranka_gripper import FrankaLibfrankaGripper

            return FrankaLibfrankaGripper(robot_ip=robot_ip)
        return super()._build_gripper(gripper_type, gripper_connection, robot_ip)

    def move_arm(self, position: np.ndarray) -> None:
        self.move_tcp_pose(np.asarray(position, dtype=np.float64))

    def move_gripper(self, position: int, speed: float = 0.3) -> None:
        assert 0 <= position <= 255
        self._gripper.move(position=float(position), speed=speed)

    def reconfigure_compliance_params(self, params: dict) -> None:
        trans_k = float(params.get("translational_stiffness", self._compliance_trans_k))
        rot_k = float(params.get("rotational_stiffness", self._compliance_rot_k))
        trans_d = params.get("translational_damping")
        ki = params.get("Ki", 0)
        if ki and float(ki) > 0:
            self._logger.warning(
                "reconfigure_compliance_params: Ki=%s ignored (no integral term)", ki
            )
        if trans_d is not None and trans_k > 0:
            tc = max(0.01, 2.0 * float(trans_d) / trans_k)
        else:
            tc = self._compliance_tc
        self._compliance_trans_k = trans_k
        self._compliance_rot_k = rot_k
        self._compliance_tc = tc
        self._stop_cart_tracking_motion()

    def _ensure_cart_tracking_motion(self) -> None:
        if self._cart_tracker is not None:
            return
        self._stop_tracking_motion()
        self._safe_join()
        self._robot.recover_from_errors()
        nullspace_target = np.asarray(self._robot.state.q, dtype=np.float64).copy()
        trans_clip = np.full(3, fc._CART_TRANS_ERROR_CLIP_M, dtype=np.float64)
        rot_clip = np.full(3, fc._CART_ROT_ERROR_CLIP_RAD, dtype=np.float64)
        self._cart_tracker = self._franky.CartesianImpedanceTracker(
            self._robot,
            translational_stiffness=self._compliance_trans_k,
            rotational_stiffness=self._compliance_rot_k,
            nullspace_target=nullspace_target,
            nullspace_stiffness=fc._CART_NULLSPACE_STIFFNESS,
            translational_error_clip=trans_clip,
            rotational_error_clip=rot_clip,
            max_delta_tau=fc._CART_MAX_DELTA_TAU,
            gains_time_constant=self._compliance_tc,
        )
        self._logger.info(
            "Cartesian impedance tracker started (K_t=%.0f K_r=%.1f tc=%.3f)",
            self._compliance_trans_k,
            self._compliance_rot_k,
            self._compliance_tc,
        )

    def command_end_effector(self, action: np.ndarray) -> bool:
        raise NotImplementedError(
            "FrankyControllerExtended: dexterous hands not supported."
        )

    def reset_end_effector(self, target_state) -> None:
        raise NotImplementedError(
            "FrankyControllerExtended: dexterous hands not supported."
        )
