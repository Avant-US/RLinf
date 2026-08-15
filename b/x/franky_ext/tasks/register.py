"""Register franky single-arm gym IDs (import before gym.make)."""

from __future__ import annotations

from typing import Any, Mapping

import gymnasium as gym
from gymnasium.envs.registration import register

from rlinf.envs.realworld.common.wrappers import apply_single_arm_wrappers
from franky_ext.franky_single_franka_env import FrankySingleFrankaEnv
from franky_ext.tasks.peg_insertion import FrankyPegInsertionEnv


def create_franky_franka_env(
    override_cfg: dict[str, Any],
    worker_info: Any,
    hardware_info: Any,
    env_idx: int,
    env_cfg: Mapping[str, Any],
) -> gym.Env:
    env = FrankySingleFrankaEnv(
        override_cfg=override_cfg,
        worker_info=worker_info,
        hardware_info=hardware_info,
        env_idx=env_idx,
    )
    return apply_single_arm_wrappers(env, env_cfg)


def create_franky_peg_insertion_env(
    override_cfg: dict[str, Any],
    worker_info: Any,
    hardware_info: Any,
    env_idx: int,
    env_cfg: Mapping[str, Any],
) -> gym.Env:
    env = FrankyPegInsertionEnv(
        override_cfg=override_cfg,
        worker_info=worker_info,
        hardware_info=hardware_info,
        env_idx=env_idx,
    )
    return apply_single_arm_wrappers(env, env_cfg)


register(
    id="FrankyFrankaEnv-v1",
    entry_point="franky_ext.tasks.register:create_franky_franka_env",
)

register(
    id="FrankyPegInsertionEnv-v1",
    entry_point="franky_ext.tasks.register:create_franky_peg_insertion_env",
)
