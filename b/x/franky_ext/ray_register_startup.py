"""PYTHONSTARTUP hook: register Franky Gym envs in every Python interpreter.

Ray workers spawn fresh processes; set PYTHONSTARTUP before ``ray start`` so
``gym.make("FrankyFrankaEnv-v1")`` resolves inside env workers.
"""

from __future__ import annotations

import os
import sys


def _register_franky_gym_ids() -> None:
    repo = os.environ.get("REPO_PATH")
    if repo:
        bx = os.path.join(repo, "b", "x")
        if bx not in sys.path:
            sys.path.insert(0, bx)
    try:
        import franky_ext.runtime_bootstrap  # noqa: F401
    except Exception:
        pass


_register_franky_gym_ids()
