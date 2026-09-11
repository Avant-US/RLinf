"""4DWVLA extension module for RLinf (eval-only).

Register via: RLINF_EXT_MODULE=four_dwvla_ext.runtime_bootstrap

This module is called on every Ray worker process initialization (see
rlinf/scheduler/cluster/utils.py line 81 and rlinf/scheduler/worker/worker.py
line 383). The register() function must be idempotent -- it may be called
multiple times in the same process.

Design follows the franky_ext precedent (b/x/franky_ext/runtime_bootstrap.py).
"""

from __future__ import annotations

import logging
import sys

logger = logging.getLogger(__name__)

# ── Idempotency guard ──────────────────────────────────────────────────────
_four_dwvla_registered = False


def register() -> None:
    """RLINF_EXT_MODULE hook: register 4DWVLA model + rollout worker patch.

    Idempotent: safe to call multiple times in the same process.
    """
    global _four_dwvla_registered
    if _four_dwvla_registered:
        logger.debug("four_dwvla_ext.register() already called; skipping.")
        return

    try:
        # Step 1: Register model builder via public API
        _register_four_dwvla_model()

        # Step 2: Monkey-patch rollout worker predict() for embodied eval
        from four_dwvla_ext.patches.rollout_worker_patch import (
            patch_rollout_worker_predict,
        )
        patch_rollout_worker_predict()

        _four_dwvla_registered = True
        logger.info(
            "four_dwvla_ext: registered 4DWVLA model + rollout worker patch"
        )

    except Exception:
        logger.exception(
            "four_dwvla_ext: registration FAILED -- 4DWVLA will not be available"
        )
        print(
            "four_dwvla_ext: registration FAILED inside the worker.\n"
            + __import__("traceback").format_exc(),
            file=sys.stderr,
        )


def _register_four_dwvla_model() -> None:
    """Register '4dwvla' model type via rlinf.models.register_model()."""
    from rlinf.models import register_model

    def _build_four_dwvla(cfg, torch_dtype):
        from four_dwvla_ext.model_builder import build_four_dwvla_model
        return build_four_dwvla_model(cfg, torch_dtype)

    register_model(
        model_type="4dwvla",
        model_builder=_build_four_dwvla,
        category="embodied",
        force=True,
    )
    logger.debug("four_dwvla_ext: registered model type '4dwvla'")
