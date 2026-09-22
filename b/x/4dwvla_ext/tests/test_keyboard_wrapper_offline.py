#!/usr/bin/env python3
"""Offline test: KeyboardVLAEvalWrapper logic with mock listener.

Tests the key handling, debounce, abort latch, and reset blocking
without a real keyboard or robot.

    pip install gymnasium numpy
    python /path/to/4dwvla_ext/tests/test_keyboard_wrapper_offline.py
"""
import sys
import time
import types
from pathlib import Path
from unittest.mock import MagicMock

import gymnasium as gym
import numpy as np

_ext_dir = str(Path(__file__).resolve().parent.parent)
sys.path.insert(0, _ext_dir)
sys.path.insert(0, str(Path(_ext_dir).parent))  # b/x — contains franky_ext

from franky_joint_env import FrankyJointEnv

PASS = 0
FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1; print(f"  [PASS] {name}")
    else:
        FAIL += 1; print(f"  [FAIL] {name}: {detail}")


class MockKeyboardListener:
    """Stand-in for KeyboardListener that feeds scripted key presses."""
    REQUIRED_KEY_NAMES = ("KEY_A", "KEY_B", "KEY_C", "KEY_Q")

    def __init__(self):
        self._queue: list[str] = []

    def inject(self, key: str):
        self._queue.append(key)

    def pop_pressed_keys(self) -> list[str]:
        out = list(self._queue)
        self._queue.clear()
        return out

    def get_key(self):
        return None


# --- Mock rlinf import chain so keyboard_vla_eval can be imported on host ---
_mock_kl_module = types.ModuleType("keyboard_listener")
_mock_kl_module.KeyboardListener = MockKeyboardListener

for _mod_name in [
    "rlinf",
    "rlinf.envs",
    "rlinf.envs.realworld",
    "rlinf.envs.realworld.common",
    "rlinf.envs.realworld.common.keyboard",
]:
    sys.modules.setdefault(_mod_name, MagicMock())
sys.modules["rlinf.envs.realworld.common.keyboard.keyboard_listener"] = _mock_kl_module

import keyboard_vla_eval as kvmod  # noqa: E402


def make_wrapped_env():
    """Create KeyboardVLAEvalWrapper with mock keyboard."""
    env = FrankyJointEnv(is_dummy=True)
    wrapped = kvmod.KeyboardVLAEvalWrapper(env)
    return wrapped, wrapped.listener


def test_abort_key():
    """'r' key sets truncated=True and latches."""
    print("\n=== T10.1: Abort Key ('r') ===")
    env, mock = make_wrapped_env()
    env._running = True
    env._last_obs = env.env._get_observation()

    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    check("normal step not truncated", not truncated)

    mock.inject("r")
    obs, reward, terminated, truncated, info = env.step(action)
    check("r key truncated", truncated)
    check("r key abort_reset in info", info.get("abort_reset") is True)

    obs, reward, terminated, truncated, info = env.step(action)
    check("abort latches", truncated)
    env.close()


def test_success_failure_keys():
    """'c' and 'b' keys set terminated with correct rewards."""
    print("\n=== T10.2: Success/Failure Keys ('c'/'b') ===")

    env, mock = make_wrapped_env()
    env._running = True
    env._last_obs = env.env._get_observation()

    mock.inject("c")
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    check("c key terminated", terminated)
    check("c key reward=1", reward == 1.0)
    check("c key result=success", info.get("eval_result") == "success")
    env.close()

    env, mock = make_wrapped_env()
    env._running = True
    env._last_obs = env.env._get_observation()

    mock.inject("b")
    obs, reward, terminated, truncated, info = env.step(action)
    check("b key terminated", terminated)
    check("b key reward=0", reward == 0.0)
    check("b key result=failure", info.get("eval_result") == "failure")
    env.close()


def test_home_key():
    """'h' key calls go_to_rest but does not end episode."""
    print("\n=== T10.3: Home Key ('h') ===")
    env, mock = make_wrapped_env()
    env._running = True
    env._last_obs = env.env._get_observation()

    mock.inject("h")
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    check("h key not terminated", not terminated)
    check("h key not truncated", not truncated)
    check("h key episode continues", env._running)
    env.close()


def test_idle_before_start():
    """Before 'a' is pressed, step returns idle response."""
    print("\n=== T10.4: Idle Before Start ===")
    env, mock = make_wrapped_env()
    env._running = False
    env._last_obs = env.env._get_observation()

    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    check("idle not terminated", not terminated)
    check("idle not truncated", not truncated)
    check("idle eval_phase=pre", info.get("eval_phase") == "pre")
    env.close()


def test_debounce():
    """Same key within PEDAL_DEBOUNCE_S is ignored."""
    print("\n=== T10.5: Debounce ===")
    env, mock = make_wrapped_env()
    env._running = True
    env._last_obs = env.env._get_observation()

    mock.inject("c")
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    check("first c -> terminated", terminated)

    # Reset state for next episode
    env._running = True
    env._abort_requested = False

    # Immediately inject c again (within debounce window)
    mock.inject("c")
    obs, reward, terminated, truncated, info = env.step(action)
    check("debounced c -> not terminated", not terminated)
    env.close()


if __name__ == "__main__":
    test_abort_key()
    test_success_failure_keys()
    test_home_key()
    test_idle_before_start()
    test_debounce()
    print(f"\n=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
