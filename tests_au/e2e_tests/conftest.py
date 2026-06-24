"""Conftest for e2e tests."""

import pytest
import torch


def pytest_configure(config):
    config.addinivalue_line("markers", "gpu: requires CUDA GPU")
    config.addinivalue_line("markers", "e2e: end-to-end test requiring model/data/GPU")
    config.addinivalue_line("markers", "slow: takes > 30 seconds")


def pytest_collection_modifyitems(config, items):
    skip_gpu = pytest.mark.skip(reason="CUDA not available")
    for item in items:
        if "gpu" in item.keywords and not torch.cuda.is_available():
            item.add_marker(skip_gpu)
