"""
Simple JSONL logger for station runs.
"""

from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional


def _ts_ms() -> int:
    return int(time.time() * 1000)


@dataclass
class JsonlLogger:
    out_dir: str
    filename: str = "events.jsonl"

    def __post_init__(self) -> None:
        os.makedirs(self.out_dir, exist_ok=True)
        self._path = os.path.join(self.out_dir, self.filename)
        self._fp = open(self._path, "a", encoding="utf-8")
        self._lock = threading.Lock()

    @property
    def path(self) -> str:
        return self._path

    def log(self, event: str, payload: Optional[Dict[str, Any]] = None) -> None:
        rec: Dict[str, Any] = {"t_ms": _ts_ms(), "event": event}
        if payload:
            rec.update(payload)
        line = json.dumps(rec, ensure_ascii=False) + "\n"
        with self._lock:
            self._fp.write(line)
            self._fp.flush()

    def close(self) -> None:
        try:
            with self._lock:
                self._fp.close()
        except Exception:
            pass

