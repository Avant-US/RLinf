import logging
import inspect

logger = logging.getLogger(__name__)


def check_rlinf_compatibility() -> list[str]:
    """Verify RLinf API compatibility before applying patches.

    Returns a list of warning messages for any detected incompatibilities.
    Empty list means all checks passed.
    """
    warnings = []

    # Check 1: register_model() API exists
    try:
        from rlinf.models import register_model
        sig = inspect.signature(register_model)
        expected_params = {"model_type", "model_builder", "category", "force"}
        actual_params = set(sig.parameters.keys())
        if not expected_params.issubset(actual_params):
            warnings.append(
                f"register_model() signature changed: "
                f"expected params {expected_params}, got {actual_params}"
            )
    except ImportError:
        warnings.append("rlinf.models.register_model not found!")

    # Check 2: MultiStepRolloutWorker.predict() exists
    try:
        from rlinf.workers.rollout.hf.huggingface_worker import (
            MultiStepRolloutWorker,
        )
        assert hasattr(MultiStepRolloutWorker, "predict"), \
            "MultiStepRolloutWorker.predict not found"
    except ImportError:
        warnings.append("MultiStepRolloutWorker not importable")

    # Check 3: SupportedModel.register() API
    try:
        from rlinf.config import SupportedModel
        assert hasattr(SupportedModel, "register"), \
            "SupportedModel.register not found"
    except ImportError:
        warnings.append("SupportedModel not importable")

    for w in warnings:
        logger.warning("RLinf compatibility check: %s", w)

    return warnings
