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

"""Galaxea R1 Pro task registrations.

Importing this package registers eight Gym environments via
:func:`gymnasium.envs.registration.register`:

* ``GalaxeaR1ProSingleArmReach-v1`` (M1 bring-up, full -- legacy ee-RPY)
* ``GalaxeaR1ProSingleArmReach-joint-v1`` (M1 main, joint mode --
  per r1pro6op47.md §10.2; the recommended first real-robot task)
* ``GalaxeaR1ProSingleArmReach-ee-v1`` (M2, ee+quat mode --
  per r1pro6op47.md §10.3)
* ``GalaxeaR1ProPickPlace-v1`` (M3, full)
* ``GalaxeaR1ProDualArmHandover-v1`` (M4, skeleton)
* ``GalaxeaR1ProDualArmCapTighten-v1`` (M4 alt, skeleton)
* ``GalaxeaR1ProWholeBodyCleanup-v1`` (M5, skeleton)
* ``GalaxeaR1ProMobileManipulation-v1`` (M5, skeleton)
"""

from gymnasium.envs.registration import register

from .r1_pro_dual_arm_cap_tighten import GalaxeaR1ProDualArmCapTightenEnv
from .r1_pro_dual_arm_handover import GalaxeaR1ProDualArmHandoverEnv
from .r1_pro_mobile_manipulation import GalaxeaR1ProMobileManipulationEnv
from .r1_pro_pick_place import GalaxeaR1ProPickPlaceEnv
from .r1_pro_single_arm_reach import GalaxeaR1ProSingleArmReachEnv
from .r1_pro_single_arm_reach_ee import GalaxeaR1ProSingleArmReachEeEnv
from .r1_pro_single_arm_reach_joint import GalaxeaR1ProSingleArmReachJointEnv
from .r1_pro_whole_body_cleanup import GalaxeaR1ProWholeBodyCleanupEnv

_TASKS = (
    (
        "GalaxeaR1ProSingleArmReach-v1",
        "rlinf.envs.realworld.galaxear.tasks.r1_pro_single_arm_reach"
        ":GalaxeaR1ProSingleArmReachEnv",
    ),
    (
        "GalaxeaR1ProSingleArmReach-joint-v1",
        "rlinf.envs.realworld.galaxear.tasks.r1_pro_single_arm_reach_joint"
        ":GalaxeaR1ProSingleArmReachJointEnv",
    ),
    (
        "GalaxeaR1ProSingleArmReach-ee-v1",
        "rlinf.envs.realworld.galaxear.tasks.r1_pro_single_arm_reach_ee"
        ":GalaxeaR1ProSingleArmReachEeEnv",
    ),
    (
        "GalaxeaR1ProPickPlace-v1",
        "rlinf.envs.realworld.galaxear.tasks.r1_pro_pick_place"
        ":GalaxeaR1ProPickPlaceEnv",
    ),
    (
        "GalaxeaR1ProDualArmHandover-v1",
        "rlinf.envs.realworld.galaxear.tasks.r1_pro_dual_arm_handover"
        ":GalaxeaR1ProDualArmHandoverEnv",
    ),
    (
        "GalaxeaR1ProDualArmCapTighten-v1",
        "rlinf.envs.realworld.galaxear.tasks.r1_pro_dual_arm_cap_tighten"
        ":GalaxeaR1ProDualArmCapTightenEnv",
    ),
    (
        "GalaxeaR1ProWholeBodyCleanup-v1",
        "rlinf.envs.realworld.galaxear.tasks.r1_pro_whole_body_cleanup"
        ":GalaxeaR1ProWholeBodyCleanupEnv",
    ),
    (
        "GalaxeaR1ProMobileManipulation-v1",
        "rlinf.envs.realworld.galaxear.tasks.r1_pro_mobile_manipulation"
        ":GalaxeaR1ProMobileManipulationEnv",
    ),
)


def _register_all() -> None:
    from gymnasium.envs.registration import registry as _registry

    for env_id, entry in _TASKS:
        if env_id in _registry:
            continue
        register(id=env_id, entry_point=entry)


_register_all()


__all__ = [
    "GalaxeaR1ProSingleArmReachEnv",
    "GalaxeaR1ProSingleArmReachJointEnv",
    "GalaxeaR1ProSingleArmReachEeEnv",
    "GalaxeaR1ProPickPlaceEnv",
    "GalaxeaR1ProDualArmHandoverEnv",
    "GalaxeaR1ProDualArmCapTightenEnv",
    "GalaxeaR1ProWholeBodyCleanupEnv",
    "GalaxeaR1ProMobileManipulationEnv",
]
