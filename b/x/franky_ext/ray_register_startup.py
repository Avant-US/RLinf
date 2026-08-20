"""Legacy ``PYTHONSTARTUP`` hook. **This file does not do anything in practice.**

``PYTHONSTARTUP`` is honoured only by *interactive* interpreters. Ray workers and
``python train_async.py`` are non-interactive, so this module never executes there
-- which means its original docstring ("set PYTHONSTARTUP before ``ray start`` so
``gym.make("FrankyFrankaEnv-v1")`` resolves inside env workers") was simply false.

The mechanism that actually registers the franky Gym ids in worker processes is
``RLINF_EXT_MODULE=franky_ext.runtime_bootstrap``, whose module body imports
``franky_ext.tasks.register``. Both setup scripts export that.

Kept (rather than deleted) only so an existing ``PYTHONSTARTUP`` export in a
running shell does not break, and so the next person who finds the export knows
why it is inert. Do not add logic here.

Unlike the original, a failed registration is **reported**: a partially-broken
``franky_ext`` import used to be swallowed, and the failure then surfaced much
later as an unrelated ``gym.make`` error on an unregistered id. That fails safe --
an unregistered id cannot command a robot -- but it costs debugging time on a stack
where lost evidence has already cost seven rounds.
"""

from __future__ import annotations

import os
import sys
import traceback


def _register_franky_gym_ids() -> None:
    repo = os.environ.get("REPO_PATH")
    if repo:
        bx = os.path.join(repo, "b", "x")
        if bx not in sys.path:
            sys.path.insert(0, bx)
    try:
        import franky_ext.runtime_bootstrap  # noqa: F401
    except Exception:
        print(
            "ray_register_startup: franky gym registration FAILED (this hook is "
            "inert in non-interactive interpreters anyway; the real path is "
            "RLINF_EXT_MODULE=franky_ext.runtime_bootstrap):\n"
            + traceback.format_exc(),
            file=sys.stderr,
        )


_register_franky_gym_ids()
