"""Monkey-patch MultiStepRolloutWorker.predict() to support 4DWVLA.

Target: rlinf/workers/rollout/hf/huggingface_worker.py
  - MultiStepRolloutWorker.predict() (line 469)
  - First if block (line 478-491): model type list for mode-kwargs
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


def patch_rollout_worker_predict() -> None:
    """Monkey-patch predict() to support 4DWVLA model type kwargs.

    Idempotent: checks _four_dwvla_rollout_patched flag.
    """
    from rlinf.workers.rollout.hf.huggingface_worker import (
        MultiStepRolloutWorker,
    )

    if getattr(MultiStepRolloutWorker, "_four_dwvla_rollout_patched", False):
        logger.debug("Rollout worker predict already patched; skipping.")
        return

    _orig_predict = MultiStepRolloutWorker.predict

    def _patched_predict(self, env_obs, mode="train"):
        """Patched predict with 4DWVLA mode-kwargs support."""
        from rlinf.config import SupportedModel

        if SupportedModel(self.model_cfg.model_type).value == "4dwvla":
            import torch

            if self.enable_dagger:
                kwargs = {"mode": "eval"}
            else:
                kwargs = {"mode": mode}

            with torch.no_grad():
                actions, result = self.hf_model.predict_action_batch(
                    env_obs=env_obs,
                    **kwargs,
                )
            return actions, result

        return _orig_predict(self, env_obs, mode)

    MultiStepRolloutWorker.predict = _patched_predict
    MultiStepRolloutWorker._four_dwvla_rollout_patched = True

    logger.debug(
        "Patched MultiStepRolloutWorker.predict() with 4DWVLA support"
    )
