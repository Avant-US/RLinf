#!/usr/bin/env python3
"""Forward-kinematics → normalised 4-D keypoint computation.

Converts 7 joint angles to 8 normalised keypoints (link1‥link7 + hand_tcp)
matching the ``pos_rot`` format used during training:

    [px, py, pz, qx, qy, qz, qw]   ×8 joints

*   Positions are base_link-relative and divided by ``bbox_radius``.
*   Quaternions use xyzw ordering, hemisphere-normalised (``qw ≥ 0``).

Keeps a sliding-window history buffer identical to the training pipeline's
``Extract3DKeypointTransformFn`` so that ``observation.his_kpts`` and
``observation.his_len`` can be fed straight into the model.
"""
from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation

import pytorch_kinematics as pk


class FKKeypointComputer:
    """Compute normalised keypoints and maintain a history buffer."""

    def __init__(
        self,
        urdf_path: str | Path,
        kpt_meta_path: str | Path,
        history_max_len: int = 200,
    ):
        with open(kpt_meta_path) as f:
            meta = json.load(f)

        self.bbox_radius: float = meta["bbox_radius"]
        self.keypoint_links: list[str] = meta["keypoint_links"]
        self.num_joints: int = len(self.keypoint_links)
        self.kpt_dim: int = meta["keypoint_dim"]
        self.history_max_len = history_max_len

        urdf_content = Path(urdf_path).read_text()
        self._chain = pk.build_chain_from_urdf(urdf_content)
        self._joint_names = self._chain.get_joint_parameter_names()
        self._arm_indices = [
            self._joint_names.index(f"fr3v2_1_joint{i + 1}") for i in range(7)
        ]

        self._history: deque[np.ndarray] = deque(maxlen=history_max_len)

    # ── public ──────────────────────────────────────────────────────────

    def compute(self, arm_q7: np.ndarray) -> np.ndarray:
        """Return ``[num_joints, kpt_dim]`` from 7 arm joint angles."""
        th = torch.zeros(1, len(self._joint_names), dtype=torch.float32)
        for i, idx in enumerate(self._arm_indices):
            th[0, idx] = float(arm_q7[i])

        fk = self._chain.forward_kinematics(th)
        kpts = np.empty((self.num_joints, self.kpt_dim), dtype=np.float32)

        for j, link_name in enumerate(self.keypoint_links):
            mat = fk[link_name].get_matrix()[0].numpy()  # [4, 4]
            pos = mat[:3, 3] / self.bbox_radius
            quat = Rotation.from_matrix(mat[:3, :3]).as_quat()  # xyzw
            if quat[3] < 0:
                quat = -quat
            kpts[j, :3] = pos
            kpts[j, 3:] = quat

        return kpts

    def append(self, arm_q7: np.ndarray) -> int:
        """Record a frame into history without treating it as the current frame."""
        self._history.append(self.compute(arm_q7))
        return len(self._history)

    def snapshot(self) -> tuple[np.ndarray, int]:
        """Pack history as-is: training's his_kpts excludes the current frame."""
        return self._pack()

    def step(self, arm_q7: np.ndarray) -> tuple[np.ndarray, int]:
        """Compute keypoints, append to history, return (his_kpts, his_len).

        Backward-compatible: combines append + snapshot.
        """
        self.append(arm_q7)
        return self.snapshot()

    def reset(self) -> None:
        """Clear the history buffer (call at episode start)."""
        self._history.clear()

    # ── private ─────────────────────────────────────────────────────────

    def _pack(self) -> tuple[np.ndarray, int]:
        his_len = len(self._history)
        buf = np.zeros(
            (self.history_max_len, self.num_joints, self.kpt_dim),
            dtype=np.float32,
        )
        for i, kpt in enumerate(self._history):
            buf[i] = kpt
        return buf, his_len
