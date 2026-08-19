"""Shared Step 8/9 CHECK / RESULT helpers (machine-readable acceptance)."""

from __future__ import annotations

import json
from typing import Any

PLACEHOLDER_SERIALS = frozenset(
    {
        "0123456789",
        "000000000000",
        "SERIAL1",
        "SERIAL2",
        "CAMERA_SERIAL1",
        "CAMERA_SERIAL2",
        "REPLACE_AFTER_STEP8A",
    }
)


def check(name: str, ok: bool, detail: str = "") -> bool:
    """Print ``CHECK <name> OK|FAIL`` and return *ok*."""
    status = "OK" if ok else "FAIL"
    extra = f"  {detail}" if detail else ""
    print(f"CHECK {name} {status}{extra}")
    return ok


def result(step: str, ok: bool, *, json_key: str | None = None, **payload: Any) -> int:
    """Print ``RESULT Step{step} PASS|FAIL`` plus a JSON payload line.

    Step-8 scripts keep the historical ``STEP8_RESULT=`` key. Other steps use
    ``STEP{step}_RESULT=`` unless *json_key* is set.
    """
    status = "PASS" if ok else "FAIL"
    print(f"RESULT Step{step} {status}")
    body = {"step": step, "pass": bool(ok), **payload}
    step_s = str(step)
    if json_key is None:
        json_key = (
            "STEP8_RESULT" if step_s.startswith("8") else f"STEP{step_s}_RESULT"
        )
    print(json_key + "=" + json.dumps(body, ensure_ascii=False, default=str))
    return 0 if ok else 1


def is_placeholder_serial(serial: str) -> bool:
    return str(serial).strip() in PLACEHOLDER_SERIALS


def collect_yaml_serials(data: dict) -> tuple[str | None, list[str]]:
    """Return (camera_type, serials) from realworld_franky_camera.yaml shape."""
    serials: list[str] = []
    camera_type = None
    groups = (data.get("cluster") or {}).get("node_groups") or []
    for group in groups:
        configs = ((group.get("hardware") or {}).get("configs")) or []
        for cfg in configs:
            if cfg.get("camera_type"):
                camera_type = str(cfg["camera_type"])
            for serial in cfg.get("camera_serials") or []:
                serials.append(str(serial))
    override = ((data.get("env") or {}).get("eval") or {}).get("override_cfg") or {}
    if override.get("camera_type"):
        camera_type = str(override["camera_type"])
    ov_serials = [str(s) for s in (override.get("camera_serials") or [])]
    if ov_serials:
        serials = ov_serials
    return camera_type, serials
