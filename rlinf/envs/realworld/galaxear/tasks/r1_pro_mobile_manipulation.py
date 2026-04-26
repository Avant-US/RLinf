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

"""M4 task skeleton: chassis + dual-arm mobile manipulation.

Adds the 6-DoF chassis to the action space.  Recommended only after
M3 has been validated, and with a conservative ``chassis_v_*_max``
in the SafetySupervisor.  A common pattern is to gate chassis
motion with ``brake_mode = True`` until the policy has converged on
arm-only navigation phases.
"""

from __future__ import annotations

from rlinf.envs.realworld.galaxear.r1_pro_env import GalaxeaR1ProEnv


class GalaxeaR1ProMobileManipulationEnv(GalaxeaR1ProEnv):
    """Skeleton task: chassis + arms (mobile manipulation).

    TODO(team): implement long-horizon reward (navigation goal +
    manipulation success).  Suggested: hierarchical reward (nav
    bonus -> grasp bonus -> place bonus) and use Sim-Real co-train
    in IsaacSim 4.5 with R1 Pro URDF for warm start.
    """

    @property
    def task_description(self) -> str:
        return (
            "Mobile manipulation: drive the chassis to a target then "
            "manipulate with the arms. (skeleton)"
        )
