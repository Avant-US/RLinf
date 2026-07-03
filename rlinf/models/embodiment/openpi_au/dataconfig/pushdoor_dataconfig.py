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
"""OpenPI data config for the RLinf-collected r1_pro "push door" LeRobot dataset.

The raw dataset stores state/action as many separate per-limb columns instead of one
flat ``observation.state`` / ``action`` array (see ``meta/info.json`` of the dataset),
and has 4 cameras where pi0 only supports 3 image slots. ``PushdoorInputs`` /
``PushdoorOutputs`` (in ``policies/pushdoor_policy.py``) do the concatenation /
camera-selection; this file only wires up the ``RepackTransform`` that renames the raw
LeRobot columns into the nested structure those classes expect, plus the delta-action
mask and the fixed language prompt.
"""

import dataclasses
import pathlib

import openpi.models.model as _model
import openpi.transforms as _transforms
from openpi.training.config import DataConfig, DataConfigFactory, ModelTransformFactory
from typing_extensions import override

from rlinf.models.embodiment.openpi_au.policies import pushdoor_policy

# Raw LeRobot column names actually read from `gs://.../pushdoor/0622_lerobot_data`
# (see `meta/info.json`). Camera columns not listed here (e.g. `head_rgb_right`) are
# simply not read.
_STATE_KEYS = {
    "left_arm": "observation.state.left_arm",
    "right_arm": "observation.state.right_arm",
    "left_gripper": "observation.state.left_gripper",
    "right_gripper": "observation.state.right_gripper",
    "torso": "observation.state.torso",
}
_ACTION_KEYS = {
    "left_arm": "action.left_arm",
    "right_arm": "action.right_arm",
    "left_gripper": "action.left_gripper",
    "right_gripper": "action.right_gripper",
    "torso": "action.torso",
    "chassis_vel": "action.chassis.velocities",
}
_IMAGE_KEYS = {
    "base": "observation.images.head_rgb",
    "left_wrist": "observation.images.wrist_left_rgb",
    "right_wrist": "observation.images.wrist_right_rgb",
}


@dataclasses.dataclass(frozen=True)
class LeRobotPushdoorDataConfig(DataConfigFactory):
    """OpenPI data config for the r1_pro "push door" task.

    State (20-dim) = left_arm(7) + right_arm(7) + left_gripper(1) + right_gripper(1)
    + torso(4). Actions (23-dim) = state layout (20-dim) + chassis_velocities(3). See
    ``policies/pushdoor_policy.py`` for the exact concatenation order.
    """

    # Fixed natural-language instruction injected via ModelTransformFactory, since this
    # dataset's own task tag ("open0622") is not a natural language prompt. Combined
    # with `base_config=DataConfig(prompt_from_task=False)` on the registered
    # TrainConfig (see `dataconfig/__init__.py`) so `PromptFromLeRobotTask` is never
    # inserted into `model_transforms.inputs`.
    default_prompt: str | None = "push open the door"

    @override
    def create(
        self, assets_dirs: pathlib.Path, model_config: _model.BaseModelConfig
    ) -> DataConfig:
        repack_transform = _transforms.Group(
            inputs=[
                _transforms.RepackTransform(
                    {
                        "image": dict(_IMAGE_KEYS),
                        "state": dict(_STATE_KEYS),
                        "actions": dict(_ACTION_KEYS),
                    }
                )
            ]
        )

        data_transforms = _transforms.Group(
            inputs=[pushdoor_policy.PushdoorInputs(model_type=model_config.model_type)],
            outputs=[pushdoor_policy.PushdoorOutputs()],
        )

        # Convert position-like components (arms + torso) to delta actions; grippers
        # stay absolute (open/close is a target pose, not a useful delta). The mask
        # only spans the 20 state dims: `chassis_vel` (the trailing 3 action dims,
        # beyond the mask length) is a base velocity *command*, not a joint position,
        # so `DeltaActions`/`AbsoluteActions` leave it untouched automatically.
        delta_action_mask = _transforms.make_bool_mask(7, 7, -1, -1, 4)
        data_transforms = data_transforms.push(
            inputs=[_transforms.DeltaActions(delta_action_mask)],
            outputs=[_transforms.AbsoluteActions(delta_action_mask)],
        )

        model_transforms = ModelTransformFactory(default_prompt=self.default_prompt)(
            model_config
        )

        return dataclasses.replace(
            self.create_base_config(assets_dirs, model_config),
            repack_transforms=repack_transform,
            data_transforms=data_transforms,
            model_transforms=model_transforms,
            action_sequence_keys=tuple(_ACTION_KEYS.values()),
        )
