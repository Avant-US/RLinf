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

"""M2 alternative task skeleton: dual-arm cap tightening.

Left arm holds the bottle; right arm grips and rotates the cap.
Reward is left as a TODO -- recommend pairing with a learned
RewardModel (ResNet) to detect cap-screwed-tight visually rather
than relying on geometric thresholds.
"""

from __future__ import annotations

from rlinf.envs.realworld.galaxear.r1_pro_env import GalaxeaR1ProEnv


class GalaxeaR1ProDualArmCapTightenEnv(GalaxeaR1ProEnv):
    """Skeleton task: dual-arm cap tightening.

    TODO(team): implement reward (recommend reward model + geometric
    co-rotation check).
    """

    @property
    def task_description(self) -> str:
        return (
            "Tighten the bottle cap: left arm holds the bottle, right "
            "arm rotates the cap clockwise. (skeleton)"
        )
