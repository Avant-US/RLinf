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

"""M1 bring-up task: right-arm reach to a fixed EE target.

Validates the entire stack (hardware reg + controller + camera mux +
safety + env) end-to-end with the simplest reward function: distance
to a target ``[xyz, rpy]`` in ``torso_link4`` frame.

The implementation is intentionally minimal so it can be used both as
an integration smoke-test and as the M1 MVP success criterion.
"""

from __future__ import annotations

from rlinf.envs.realworld.galaxear.r1_pro_env import GalaxeaR1ProEnv


class GalaxeaR1ProSingleArmReachEnv(GalaxeaR1ProEnv):
    """Right-arm reach to ``target_ee_pose_right``."""

    @property
    def task_description(self) -> str:
        return (
            "Move the right end-effector to the configured target pose "
            "and hold for `success_hold_steps` consecutive steps."
        )
