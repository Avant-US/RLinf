#!/usr/bin/env python3
"""Phase 1A: dummy gym.make for FrankyCubePlaceEnv-v1 (no robot / FCI)."""

from __future__ import annotations

import inspect
import os
import sys

import gymnasium as gym
import numpy as np

REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

import franky_ext.tasks.register  # noqa: E402,F401
from franky_ext.tasks.cube_place import FrankyCubePlaceEnv  # noqa: E402


def _unwrap(env):
    cur = env
    seen = []
    while True:
        seen.append(type(cur).__name__)
        if isinstance(cur, FrankyCubePlaceEnv):
            return cur, seen
        nxt = getattr(cur, "env", None)
        if nxt is None or nxt is cur:
            break
        cur = nxt
    raise AssertionError(f"FrankyCubePlaceEnv not in wrapper stack: {seen}")


def main() -> int:
    spec = gym.spec("FrankyCubePlaceEnv-v1")
    print(f"gym_id={spec.id}")
    assert spec.id == "FrankyCubePlaceEnv-v1"

    env = gym.make(
        "FrankyCubePlaceEnv-v1",
        override_cfg={
            "is_dummy": True,
            "enable_camera_player": False,
            "camera_serials": ["000000000000"],
            "target_ee_pose": [0.5, 0.0, 0.1, 3.14, 0.0, 0.0],
        },
        worker_info=None,
        hardware_info=None,
        env_idx=0,
        env_cfg={},
    )
    inner, stack = _unwrap(env)
    print(f"wrapper_stack={stack}")
    print(f"action_space={env.action_space}")
    assert env.action_space.shape == (6,), env.action_space

    cfg = inner.config
    print(
        "clips "
        f"xy=({cfg.clip_x_range},{cfg.clip_y_range}) "
        f"z_low={cfg.clip_z_range_low} z_high={cfg.clip_z_range_high} "
        f"rand_xy={cfg.random_xy_range}"
    )
    assert cfg.clip_x_range == 0.05
    assert cfg.clip_y_range == 0.05
    assert cfg.clip_z_range_low == 0.005
    assert cfg.clip_z_range_high == 0.08
    assert cfg.random_xy_range == 0.03

    src = inspect.getsource(FrankyCubePlaceEnv.go_to_rest)
    assert "np.array([-1.0])" in src or "np.array([-1.])" in src, src
    assert "np.array([1.0])" not in src and "np.array([1.])" not in src, src
    print("go_to_rest uses close (-1.0), not open (+1.0)")

    obs, _info = env.reset()
    assert obs is not None
    action = np.zeros(env.action_space.shape, dtype=np.float32)
    obs, reward, terminated, truncated, info = env.step(action)
    assert obs is not None
    print(f"dummy_step reward={reward} terminated={terminated} truncated={truncated}")
    env.close()
    print("Phase1A PASS FrankyCubePlaceEnv-v1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
