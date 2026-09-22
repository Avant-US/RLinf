#!/usr/bin/env python3
"""Offline tests for stable keyboard/mouse aliases in sync_input_devices.py."""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

_EXT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_EXT))

from sync_input_devices import (  # noqa: E402
    InputDeviceInfo,
    _atomic_symlink,
    _install_stable_links,
    _select_role_device,
)


PASS = 0
FAIL = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}: {detail}")


def _device(
    event_name: str,
    roles: tuple[str, ...],
    directory: Path,
    name: str | None = None,
) -> InputDeviceInfo:
    event_path = directory / event_name
    event_path.write_text("node", encoding="utf-8")
    if name is None:
        name = "Dell KB216 Wired Keyboard" if "keyboard" in roles else "USB Mouse"
    return InputDeviceInfo(
        event_name=event_name,
        event_path=str(event_path),
        sysfs_path=f"/sys/class/input/{event_name}",
        dev_number="13:66",
        name=name,
        phys="usb-0000:00:14.0-1/input0",
        uniq="",
        roles=roles,
        stable_id=f"{event_name}-deadbeefdeadbeef",
    )


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        input_dir = tmp_path / "input"
        run_dir = tmp_path / "run"
        input_dir.mkdir()
        run_dir.mkdir()

        keyboard = _device("event2", ("keyboard",), input_dir)
        mouse = _device("event3", ("mouse",), input_dir)

        created = _install_stable_links(run_dir, [keyboard, mouse], "", "")
        kb_alias = run_dir / "keyboard"
        ms_alias = run_dir / "mouse"

        check("keyboard alias is a symlink", kb_alias.is_symlink())
        check(
            "keyboard alias targets event2",
            os.path.abspath(os.readlink(kb_alias)) == os.path.abspath(keyboard.event_path)
            if kb_alias.is_symlink()
            else False,
            str(kb_alias),
        )
        check("mouse alias is a symlink", ms_alias.is_symlink())
        check("created set records absolute keyboard path", str(kb_alias) in created)
        check(
            "role-specific keyboard link exists",
            (run_dir / f"keyboard-{keyboard.stable_id}").is_symlink(),
        )

        replaced = _atomic_symlink(kb_alias, Path(mouse.event_path))
        check("existing symlink can be replaced", replaced and kb_alias.is_symlink())

        blocking = run_dir / "blocked"
        blocking.write_text("not a symlink", encoding="utf-8")
        check("refuse to replace a regular file", not _atomic_symlink(blocking, Path(keyboard.event_path)))

        two_keyboards = [
            keyboard,
            _device("event8", ("keyboard",), input_dir, name="Logitech K120"),
        ]
        check(
            "two keyboards do not produce a generic alias",
            len(_select_role_device(two_keyboards, "keyboard", "")) == 2,
        )
        created_multi = _install_stable_links(run_dir / "multi", two_keyboards, "", "")
        check(
            "ambiguous keyboards skip generic alias",
            str((run_dir / "multi" / "keyboard")) not in created_multi,
        )
        created_match = _install_stable_links(
            run_dir / "match", two_keyboards, "Dell KB216", ""
        )
        check(
            "keyboard-match selects Dell",
            str(run_dir / "match" / "keyboard") in created_match,
        )

    print(f"\n{PASS} passed, {FAIL} failed")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
