"""Shared logging and array-formatting helpers for 4DWVLA evaluation."""
from __future__ import annotations

import logging
import os
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_LOG_DIR = "/workspace/RLinf/b/x/4dwvla_ext/logs"


def configure_logging(role: str, log_dir: str | None = None) -> Path:
    """Configure timestamped console and file logging for one process."""
    start_time = datetime.now()
    output_dir = Path(log_dir or os.environ.get("VLA_LOG_DIR", DEFAULT_LOG_DIR))
    output_dir.mkdir(parents=True, exist_ok=True)
    log_path = output_dir / (
        f"{role}_{start_time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}.log"
    )

    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    logging.basicConfig(
        level=logging.INFO,
        handlers=[stream_handler, file_handler],
        force=True,
    )

    logger = logging.getLogger(f"vla-{role}")
    logger.info(
        "Logging started: role=%s start_time=%s log_file=%s",
        role,
        start_time.isoformat(timespec="seconds"),
        log_path,
    )
    return log_path


def format_array(value: Any, precision: int = 6) -> str:
    """Format an array without truncation so full actions remain inspectable."""
    return np.array2string(
        np.asarray(value),
        precision=precision,
        suppress_small=False,
        separator=", ",
        max_line_width=100_000,
    )


def summarize_array(value: Any) -> dict[str, Any]:
    """Return compact metadata for images, states, and other numeric arrays."""
    array = np.asarray(value)
    result: dict[str, Any] = {
        "shape": list(array.shape),
        "dtype": str(array.dtype),
    }
    if array.size == 0:
        return result

    if np.issubdtype(array.dtype, np.number):
        finite = np.isfinite(array)
        result["finite"] = bool(finite.all())
        if finite.any():
            finite_values = array[finite]
            result.update(
                min=float(finite_values.min()),
                max=float(finite_values.max()),
                mean=float(finite_values.mean()),
            )
    return result
