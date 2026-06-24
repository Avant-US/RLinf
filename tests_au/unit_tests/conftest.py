"""Shared fixtures and markers for tests_au."""

import os

import pytest
import torch


def pytest_configure(config):
    config.addinivalue_line("markers", "gpu: requires CUDA GPU")
    config.addinivalue_line("markers", "e2e: end-to-end test requiring model/data/GPU")
    config.addinivalue_line("markers", "slow: takes > 30 seconds")


def pytest_collection_modifyitems(config, items):
    skip_gpu = pytest.mark.skip(reason="CUDA not available")
    skip_e2e = pytest.mark.skip(reason="E2E resources not available (set OPENPI_AU_CKPT_DIR)")

    for item in items:
        if "gpu" in item.keywords and not torch.cuda.is_available():
            item.add_marker(skip_gpu)
        if "e2e" in item.keywords:
            if not torch.cuda.is_available():
                item.add_marker(skip_e2e)
            if not os.environ.get("OPENPI_AU_CKPT_DIR"):
                item.add_marker(skip_e2e)


@pytest.fixture
def device():
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


@pytest.fixture
def ckpt_dir():
    d = os.environ.get("OPENPI_AU_CKPT_DIR")
    if not d:
        pytest.skip("OPENPI_AU_CKPT_DIR not set")
    return d
