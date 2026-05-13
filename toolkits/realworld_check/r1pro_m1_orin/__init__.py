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
"""R1 Pro M1 single-arm joint-reach: local single-process RLinf SAC on Orin.

Two-phase data + training pipeline that REUSES RLinf components without
modifying any of them:

  * Phase 1 (collect): multiple ``(start_q, target_q)`` pairs generate
    diverse transitions via random policy + SafetyConfig-projected actions.
  * Phase 2 (sft, optional): behaviour cloning over Phase 1 data using
    ``MLPPolicy.sft_forward``.
  * Phase 3 (train_sac): online SAC on the single anchor pair, warm-started
    from Phase 1 buffer (and optionally Phase 2 weights).
  * Phase 4 (evaluate): deterministic policy on the anchor pair.

See ``bt/docs/rwRL/r1pro6op47_reach_joint3_2.md`` for the full design and
the recommended ``(start_q, target_q)`` + ``safety_step_scale`` /
``arm_qvel_max`` values.
"""
