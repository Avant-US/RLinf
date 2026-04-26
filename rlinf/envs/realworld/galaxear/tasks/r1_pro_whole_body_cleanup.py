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

"""M3 task skeleton: whole-body cleanup (torso + dual arms).

Brings the torso 4-DoF into the policy.  Use this together with
``use_torso: true`` and a higher ``soft_sync_window_ms`` because
torso motion increases stale-frame jitter.
"""

from __future__ import annotations

from rlinf.envs.realworld.galaxear.r1_pro_env import GalaxeaR1ProEnv


class GalaxeaR1ProWholeBodyCleanupEnv(GalaxeaR1ProEnv):
    """Skeleton task: torso + dual-arm tabletop cleanup.

    TODO(team): implement reward.  Suggested: per-object visual
    progress via reward model; phase the torso pose changes with the
    arm motion.
    """

    @property
    def task_description(self) -> str:
        return (
            "Whole-body tabletop cleanup: torso adjusts height while "
            "dual arms sweep / pick clutter to a bin. (skeleton)"
        )
