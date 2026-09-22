"""Keyboard-controlled VLA evaluation wrapper.

Extends RLinf's KeyboardEvalControlWrapper pattern with abort/home keys.
Uses KeyboardListener (evdev-based, headless, survives USB disconnects).

Key bindings:
  'a': start rollout (blocks in reset() until pressed)
  'r': abort current episode + truncated=True (robot stops, waits for reset)
  'b': mark failure (terminated=True, reward=0)
  'c': mark success (terminated=True, reward=1)
  'h': go to HOME position (non-destructive, episode continues)

Source:
  KeyboardListener:         rlinf/envs/realworld/common/keyboard/keyboard_listener.py
  KeyboardEvalControlWrapper: rlinf/envs/realworld/common/wrappers/keyboard_eval_control_wrapper.py
"""
from __future__ import annotations

import math
import logging
import sys
import time
from typing import Any, SupportsFloat

import gymnasium as gym
from gymnasium.core import ActType, ObsType

sys.path.insert(0, "/workspace/RLinf")
from rlinf.envs.realworld.common.keyboard.keyboard_listener import KeyboardListener

logger = logging.getLogger(__name__)


class KeyboardVLAEvalWrapper(gym.Wrapper):
    """Foot-pedal / keyboard gated VLA evaluation with abort and home.

    Extends KeyboardEvalControlWrapper (a/b/c) with:
      'r' -- abort episode, stop robot, truncated=True
      'h' -- go to HOME position without ending episode
    """

    IDLE_POLL_S = 0.05
    PEDAL_DEBOUNCE_S = 0.2
    WAIT_HEARTBEAT_S = 10.0

    def __init__(self, env: gym.Env):
        super().__init__(env)
        self.listener = KeyboardListener()
        self._running = False
        self._abort_requested = False
        self._last_obs: Any = None
        self._last_press_ts: dict[str, float] = {}
        logger.info(
            "Keyboard controls: 'a'=start, 'r'=abort, "
            "'b'=failure, 'c'=success, 'h'=HOME"
        )

    def reset(self, *, seed=None, options=None):
        self._abort_requested = False
        self._last_press_ts.clear()
        self.listener.pop_pressed_keys()
        obs, info = self.env.reset(seed=seed, options=options)
        self._last_obs = obs

        logger.info(
            "Arms homed. Arrange scene, press 'a' to start "
            "(Ctrl-C to abort)."
        )
        last_heartbeat = time.monotonic()
        while True:
            time.sleep(self.IDLE_POLL_S)
            now = time.monotonic()
            if now - last_heartbeat >= self.WAIT_HEARTBEAT_S:
                last_heartbeat = now
                logger.info("Waiting for 'a' to start rollout...")
            for key in self.listener.pop_pressed_keys():
                if key == "a":
                    self._running = True
                    logger.info("'a' pressed -- starting rollout.")
                    return obs, info

    def step(
        self, action: ActType
    ) -> tuple[ObsType, SupportsFloat, bool, bool, dict[str, Any]]:
        if self._abort_requested:
            return self._last_obs, 0.0, False, True, {"abort_reset": True}

        if not self._running:
            time.sleep(self.IDLE_POLL_S)
            return self._idle_response(event=None)

        obs, reward, terminated, truncated, info = self.env.step(action)
        self._last_obs = obs

        terminated = False
        truncated = False

        result: str | None = None
        for key in self.listener.pop_pressed_keys():
            now = time.monotonic()
            if now - self._last_press_ts.get(key, -math.inf) < self.PEDAL_DEBOUNCE_S:
                continue
            self._last_press_ts[key] = now

            if key == "r":
                logger.warning(">>> ABORT: 'r' key <<<")
                self._abort_requested = True
                self._running = False
                if hasattr(self.env, "unwrapped"):
                    ctrl = getattr(self.env.unwrapped, "_controller", None)
                    if ctrl:
                        ctrl.stop()
                info["abort_reset"] = True
                return obs, 0.0, False, True, info

            elif key == "c":
                result = "success"
                terminated = True
                reward = 1.0
                self._running = False
                logger.info("'c' pressed -- success.")
                break

            elif key == "b":
                result = "failure"
                terminated = True
                reward = 0.0
                self._running = False
                logger.info("'b' pressed -- failure.")
                break

            elif key == "h":
                logger.info(">>> HOME: 'h' key <<<")
                if hasattr(self.env, "unwrapped"):
                    ctrl = getattr(self.env.unwrapped, "_controller", None)
                    if ctrl:
                        ctrl.stop()
                    go = getattr(self.env.unwrapped, "go_to_rest", None)
                    if go:
                        go()

        info["eval_phase"] = "rec" if self._running else "pre"
        info["eval_result"] = result
        return obs, reward, terminated, truncated, info

    def _idle_response(self, event: str | None):
        info: dict[str, Any] = {"eval_phase": "pre", "eval_event": event, "eval_result": None}
        return self._last_obs, 0.0, False, False, info
