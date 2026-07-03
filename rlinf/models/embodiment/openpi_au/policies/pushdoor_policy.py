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
"""Policy transforms for the RLinf-collected r1_pro "push door" LeRobot dataset.

The raw LeRobot dataset (see ``dataconfig/pushdoor_dataconfig.py``) stores state and
action as many separate per-limb columns (``observation.state.left_arm``,
``action.chassis.velocities``, ...) instead of one flat ``observation/state`` /
``actions`` array like LIBERO. This module concatenates those parts into the single
``state`` (20-dim) / ``actions`` (23-dim) vectors the pi0 model expects, and maps the
robot's 4 cameras onto the 3 image slots pi0 supports.

Layout (fixed order, must stay consistent between ``PushdoorInputs`` and any consumer
of ``PushdoorOutputs``):
    state   = [left_arm(7), right_arm(7), left_gripper(1), right_gripper(1), torso(4)]      -> 20 dims
    actions = [left_arm(7), right_arm(7), left_gripper(1), right_gripper(1), torso(4),
               chassis_velocities(3)]                                                       -> 23 dims

``chassis_velocities`` (base vx/vy/wz command) has no corresponding "current velocity"
slot in ``state`` on purpose: it is a rate command, not a joint position, so it is left
out of the delta-action mask in the data config (see ``PUSHDOOR_ACTION_DIM`` /
``PUSHDOOR_STATE_DIM`` below and the mask built in ``pushdoor_dataconfig.py``).
"""

import dataclasses

import einops
import numpy as np
from openpi import transforms
from openpi.models import model as _model

PUSHDOOR_STATE_DIM = 20
PUSHDOOR_ACTION_DIM = 23


def make_pushdoor_example() -> dict:
    """Creates a random input example for the pushdoor policy (for smoke tests)."""
    return {
        "image": {
            "base": np.random.randint(256, size=(1536, 1920, 3), dtype=np.uint8),
            "left_wrist": np.random.randint(256, size=(360, 640, 3), dtype=np.uint8),
            "right_wrist": np.random.randint(256, size=(360, 640, 3), dtype=np.uint8),
        },
        "state": {
            "left_arm": np.random.rand(7),
            "right_arm": np.random.rand(7),
            "left_gripper": np.random.rand(1),
            "right_gripper": np.random.rand(1),
            "torso": np.random.rand(4),
        },
        "actions": {
            "left_arm": np.random.rand(10, 7),
            "right_arm": np.random.rand(10, 7),
            "left_gripper": np.random.rand(10, 1),
            "right_gripper": np.random.rand(10, 1),
            "torso": np.random.rand(10, 4),
            "chassis_vel": np.random.rand(10, 3),
        },
    }


def _parse_image(image) -> np.ndarray:
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).astype(np.uint8)
    if image.shape[0] == 3:
        image = einops.rearrange(image, "c h w -> h w c")
    return image


def _concat_state(state: dict) -> np.ndarray:
    """Concatenate the per-limb state parts into the fixed 20-dim layout."""
    parts = [
        np.asarray(state["left_arm"]),
        np.asarray(state["right_arm"]),
        np.asarray(state["left_gripper"]),
        np.asarray(state["right_gripper"]),
        np.asarray(state["torso"]),
    ]
    return np.concatenate(parts, axis=-1)


def _concat_actions(actions: dict) -> np.ndarray:
    """Concatenate the per-limb action parts into the fixed 23-dim layout.

    Each part has shape ``[action_horizon, dim]`` (sequenced via
    ``action_sequence_keys`` in the data config).
    """
    parts = [
        np.asarray(actions["left_arm"]),
        np.asarray(actions["right_arm"]),
        np.asarray(actions["left_gripper"]),
        np.asarray(actions["right_gripper"]),
        np.asarray(actions["torso"]),
        np.asarray(actions["chassis_vel"]),
    ]
    return np.concatenate(parts, axis=-1)


@dataclasses.dataclass(frozen=True)
class PushdoorInputs(transforms.DataTransformFn):
    """Converts the repacked pushdoor dataset dict into the pi0 model input format.

    Expects (after the ``RepackTransform`` defined in ``pushdoor_dataconfig.py``):
        image: {"base": [h,w,3], "left_wrist": [h,w,3], "right_wrist": [h,w,3]}
        state: {"left_arm": [7], "right_arm": [7], "left_gripper": [1],
                "right_gripper": [1], "torso": [4]}
        actions (training only): {"left_arm": [T,7], "right_arm": [T,7],
                "left_gripper": [T,1], "right_gripper": [T,1], "torso": [T,4],
                "chassis_vel": [T,3]}
    """

    # Determines which model will be used. Do not change this for your own dataset.
    model_type: _model.ModelType

    def __call__(self, data: dict) -> dict:
        base_image = _parse_image(data["image"]["base"])
        left_wrist_image = _parse_image(data["image"]["left_wrist"])
        right_wrist_image = _parse_image(data["image"]["right_wrist"])

        inputs = {
            "state": _concat_state(data["state"]),
            "image": {
                "base_0_rgb": base_image,
                "left_wrist_0_rgb": left_wrist_image,
                "right_wrist_0_rgb": right_wrist_image,
            },
            # All 3 slots are real (non-padding) cameras, unlike single-camera setups
            # (e.g. Franka) that zero-pad and mask out the two unused wrist slots.
            "image_mask": {
                "base_0_rgb": np.True_,
                "left_wrist_0_rgb": np.True_,
                "right_wrist_0_rgb": np.True_,
            },
        }

        # Actions are only available during training.
        if "actions" in data:
            inputs["actions"] = _concat_actions(data["actions"])

        if "prompt" in data:
            inputs["prompt"] = data["prompt"]

        return inputs


@dataclasses.dataclass(frozen=True)
class PushdoorOutputs(transforms.DataTransformFn):
    """Converts model outputs back to the 23-dim pushdoor action layout (inference only).

    The returned vector keeps the fixed layout documented at the top of this file:
    ``[left_arm(7), right_arm(7), left_gripper(1), right_gripper(1), torso(4),
    chassis_vel(3)]``. Split it back into named parts using those slices if a future
    real-robot control loop for this task needs them individually.
    """

    action_dim: int = PUSHDOOR_ACTION_DIM

    def __call__(self, data: dict) -> dict:
        return {"actions": np.asarray(data["actions"][:, : self.action_dim])}
