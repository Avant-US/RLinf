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

"""Galaxea R1 Pro real-world env package.

Importing this module is sufficient to register all six R1 Pro tasks
into Gymnasium.  ``RealWorldEnv._create_env`` calls
``gym.make(init_params.id, ...)``, which requires the registration
side-effect to have happened beforehand.
"""

from .r1_pro_action_schema import ActionSchema, build_action_schema
from .r1_pro_camera_mux import CameraMuxConfig, GalaxeaR1ProCameraMux
from .r1_pro_env import GalaxeaR1ProEnv, GalaxeaR1ProRobotConfig
from .r1_pro_robot_state import GalaxeaR1ProRobotState
from .r1_pro_safety import (
    GalaxeaR1ProSafetySupervisor,
    SafetyConfig,
    SafetyInfo,
)
from .r1_pro_wrappers import (
    GalaxeaR1ProActionSmoother,
    GalaxeaR1ProDualArmCollisionWrapper,
    GalaxeaR1ProJoystickIntervention,
    GalaxeaR1ProVRIntervention,
)

# Importing tasks subpackage triggers gym.register for all 6 ids.
from . import tasks  # noqa: F401  (side-effect import)

__all__ = [
    "ActionSchema",
    "build_action_schema",
    "CameraMuxConfig",
    "GalaxeaR1ProActionSmoother",
    "GalaxeaR1ProCameraMux",
    "GalaxeaR1ProDualArmCollisionWrapper",
    "GalaxeaR1ProEnv",
    "GalaxeaR1ProJoystickIntervention",
    "GalaxeaR1ProRobotConfig",
    "GalaxeaR1ProRobotState",
    "GalaxeaR1ProSafetySupervisor",
    "GalaxeaR1ProVRIntervention",
    "SafetyConfig",
    "SafetyInfo",
    "tasks",
]
