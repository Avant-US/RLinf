#!/usr/bin/env python3
"""Keep Linux input device nodes and stable links synchronized in a container.

Docker containers can keep running while USB input devices are unplugged and
reconnected.  The kernel then assigns a new ``eventN`` number, while a
container may still contain an old manually-created ``/dev/input/eventN`` node.

This utility uses ``/sys/class/input`` as the source of truth.  It:

* creates missing ``/dev/input/eventN`` nodes from their sysfs major/minor;
* optionally removes stale event nodes (only with ``--prune-stale``);
* creates identity links under ``/run/rlt-input/event-*``;
* updates ``keyboard`` and ``mouse`` aliases when devices are reconnected;
* mirrors the same links under ``/dev/input/by-rlt`` for compatibility.

Canonical aliases live on tmpfs (``/run/rlt-input``), not under ``/dev/input``.
udev may delete non-``event*`` names that appear under ``/dev/input`` even in a
subdirectory such as ``by-rlt``.  That is why a log line claiming
``/dev/input/by-rlt/keyboard`` can appear while ``ls`` shows only ``event-*``.

For the RLinf keyboard listener, start the watcher before the client and use:

    export RLINF_KEYBOARD_DEVICE=/run/rlt-input/keyboard

The alias name remains unchanged when the physical keyboard receives a new
event number, so ``KeyboardListener`` can reopen it after a USB disconnect.
"""

from __future__ import annotations

import argparse
import grp
import hashlib
import json
import logging
import os
import re
import signal
import stat
import time
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Iterable

LOGGER = logging.getLogger("sync-input-devices")

EVENT_NAME_RE = re.compile(r"^event[0-9]+$")
DEFAULT_SYSFS_DIR = Path("/sys/class/input")
DEFAULT_INPUT_DIR = Path("/dev/input")
DEFAULT_STABLE_DIR = Path("/run/rlt-input")
DEFAULT_COMPAT_DIR = DEFAULT_INPUT_DIR / "by-rlt"
DEFAULT_STATE_FILE = Path("/run/rlt-input-devices.json")


@dataclass(frozen=True)
class InputDeviceInfo:
    """A currently registered Linux evdev event device."""

    event_name: str
    event_path: str
    sysfs_path: str
    dev_number: str
    name: str
    phys: str
    uniq: str
    roles: tuple[str, ...]
    stable_id: str


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (FileNotFoundError, OSError):
        return ""


def _safe_name(value: str, fallback: str = "input") -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return value.strip("._-")[:48] or fallback


def _device_number(sysfs_event: Path) -> str:
    """Return the kernel major:minor value for an event device."""
    return _read_text(sysfs_event / "dev")


def _device_identity(sysfs_event: Path, name: str, phys: str, uniq: str) -> str:
    """Build an identity that does not depend on the event number."""
    device = sysfs_event / "device"
    fields = [
        _read_text(device / "id" / "bustype"),
        _read_text(device / "id" / "vendor"),
        _read_text(device / "id" / "product"),
        phys,
        uniq,
        name,
        os.path.realpath(device),
    ]
    raw = "\0".join(fields)
    digest = hashlib.sha256(raw.encode("utf-8")).hexdigest()[:16]
    return f"{_safe_name(name)}-{digest}"


def _classify_with_evdev(event_path: Path) -> tuple[str, ...]:
    """Classify keyboard/mouse capabilities when python-evdev is available."""
    try:
        from evdev import InputDevice, ecodes
    except ImportError:
        LOGGER.warning("python-evdev is unavailable; role aliases will be skipped.")
        return ()

    try:
        device = InputDevice(str(event_path))
        capabilities = device.capabilities(verbose=False)
    except (OSError, PermissionError) as exc:
        LOGGER.debug("Cannot inspect %s: %s", event_path, exc)
        return ()
    finally:
        try:
            device.close()
        except (NameError, AttributeError, OSError):
            pass

    key_codes = set(capabilities.get(ecodes.EV_KEY, []))
    rel_codes = set(capabilities.get(ecodes.EV_REL, []))

    keyboard_codes = {
        ecodes.KEY_A,
        ecodes.KEY_B,
        ecodes.KEY_C,
        ecodes.KEY_Q,
    }
    mouse_codes = {
        ecodes.REL_X,
        ecodes.REL_Y,
        ecodes.BTN_LEFT,
    }

    roles: list[str] = []
    if keyboard_codes.issubset(key_codes):
        roles.append("keyboard")
    if (
        {ecodes.REL_X, ecodes.REL_Y}.issubset(rel_codes)
        and ecodes.BTN_LEFT in key_codes
    ):
        roles.append("mouse")
    return tuple(roles)


def discover_devices(sysfs_dir: Path, input_dir: Path) -> list[InputDeviceInfo]:
    """Discover current event devices from sysfs, not from stale /dev nodes."""
    devices: list[InputDeviceInfo] = []
    for sysfs_event in sorted(sysfs_dir.glob("event*")):
        if not EVENT_NAME_RE.match(sysfs_event.name):
            continue

        dev_number = _device_number(sysfs_event)
        if not dev_number or ":" not in dev_number:
            LOGGER.debug("Skipping %s: no valid major:minor", sysfs_event)
            continue

        device = sysfs_event / "device"
        name = _read_text(device / "name") or _read_text(sysfs_event / "name")
        phys = _read_text(device / "phys")
        uniq = _read_text(device / "uniq")
        event_path = input_dir / sysfs_event.name
        roles = _classify_with_evdev(event_path) if event_path.exists() else ()
        devices.append(
            InputDeviceInfo(
                event_name=sysfs_event.name,
                event_path=str(event_path),
                sysfs_path=str(sysfs_event),
                dev_number=dev_number,
                name=name or sysfs_event.name,
                phys=phys,
                uniq=uniq,
                roles=roles,
                stable_id=_device_identity(sysfs_event, name, phys, uniq),
            )
        )
    return devices


def _is_mount_point(path: Path) -> bool:
    """Return whether path is a separate mount, such as a host bind mount."""
    return os.path.ismount(path)


def _ensure_event_node(info: InputDeviceInfo, repair: bool) -> bool:
    """Create or repair one event node from sysfs major:minor."""
    path = Path(info.event_path)
    major, minor = (int(part) for part in info.dev_number.split(":", 1))
    expected_rdev = os.makedev(major, minor)

    try:
        current_stat = os.stat(path)
    except FileNotFoundError:
        current_stat = None
    except OSError as exc:
        LOGGER.warning("Cannot stat %s: %s", path, exc)
        return False

    if current_stat is not None:
        if not stat.S_ISCHR(current_stat.st_mode):
            LOGGER.warning("Skipping %s: it is not a character device.", path)
            return False
        if current_stat.st_rdev == expected_rdev:
            return True
        if not repair:
            LOGGER.warning(
                "%s has major:minor %s, expected %s; use --repair-nodes.",
                path,
                os.major(current_stat.st_rdev),
                info.dev_number,
            )
            return False
        try:
            path.unlink()
        except OSError as exc:
            LOGGER.error("Cannot replace incorrect node %s: %s", path, exc)
            return False

    try:
        os.mknod(path, stat.S_IFCHR | 0o660, expected_rdev)
        try:
            os.chown(path, -1, grp.getgrnam("input").gr_gid)
        except (KeyError, PermissionError, OSError):
            LOGGER.debug("Could not assign input group to %s.", path)
        LOGGER.info("Created input node %s (%s).", path, info.dev_number)
        return True
    except FileExistsError:
        return True
    except PermissionError:
        LOGGER.error(
            "Cannot create %s: run the synchronizer as root or grant CAP_MKNOD.",
            path,
        )
        return False
    except OSError as exc:
        LOGGER.error("Cannot create %s: %s", path, exc)
        return False


def _atomic_symlink(link: Path, target: Path) -> bool:
    """Atomically replace a managed symlink without touching real devices."""
    if os.path.lexists(link) and not link.is_symlink():
        LOGGER.error("Refusing to replace non-symlink %s.", link)
        return False

    abs_target = Path(os.path.abspath(target))
    temporary = link.parent / f".{link.name}.tmp-{os.getpid()}"
    try:
        link.parent.mkdir(parents=True, exist_ok=True)
        temporary.unlink(missing_ok=True)
        temporary.symlink_to(abs_target)
        os.replace(temporary, link)
        if not link.is_symlink():
            LOGGER.error("Stable link vanished immediately after create: %s", link)
            return False
        LOGGER.info("Stable link %s -> %s", link, os.readlink(link))
        return True
    except OSError as exc:
        LOGGER.error("Cannot update stable link %s -> %s: %s", link, abs_target, exc)
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        return False


def _remove_managed_link(path: Path) -> None:
    try:
        if path.is_symlink():
            path.unlink()
    except OSError as exc:
        LOGGER.warning("Cannot remove stale managed link %s: %s", path, exc)


def _read_managed_links(state_file: Path) -> set[str]:
    try:
        state = json.loads(state_file.read_text(encoding="utf-8"))
        return set(state.get("managed_links", []))
    except (FileNotFoundError, OSError, ValueError):
        return set()


def _managed_link_path(stable_dir: Path, name_or_path: str) -> Path:
    path = Path(name_or_path)
    if path.is_absolute():
        return path
    return stable_dir / path


def _write_state(
    state_file: Path,
    devices: Iterable[InputDeviceInfo],
    managed_links: set[str],
) -> None:
    try:
        state_file.parent.mkdir(parents=True, exist_ok=True)
        state_file.write_text(
            json.dumps(
                {
                    "updated_at": time.time(),
                    "managed_links": sorted(managed_links),
                    "devices": [asdict(device) for device in devices],
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
    except OSError as exc:
        LOGGER.warning("Cannot write state file %s: %s", state_file, exc)


def _select_role_device(
    devices: list[InputDeviceInfo],
    role: str,
    selector: str,
) -> list[InputDeviceInfo]:
    candidates = [device for device in devices if role in device.roles]
    if selector:
        needle = selector.lower()
        candidates = [
            device
            for device in candidates
            if needle in f"{device.name} {device.phys} {device.uniq}".lower()
        ]
    return candidates


def _install_stable_links(
    stable_dir: Path,
    devices: list[InputDeviceInfo],
    keyboard_match: str,
    mouse_match: str,
) -> set[str]:
    """Create identity and role aliases under one directory. Return absolute paths."""
    stable_dir.mkdir(parents=True, exist_ok=True)
    created: set[str] = set()

    def _note(path: Path, ok: bool) -> None:
        if ok:
            created.add(str(path))

    for device in devices:
        identity = stable_dir / f"event-{device.stable_id}"
        _note(identity, _atomic_symlink(identity, Path(device.event_path)))
        for role in device.roles:
            role_link = stable_dir / f"{role}-{device.stable_id}"
            _note(role_link, _atomic_symlink(role_link, Path(device.event_path)))

    selectors = {"keyboard": keyboard_match, "mouse": mouse_match}
    for role, selector in selectors.items():
        alias = stable_dir / role
        candidates = _select_role_device(devices, role, selector)
        if len(candidates) == 1:
            ok = _atomic_symlink(alias, Path(candidates[0].event_path))
            _note(alias, ok)
            if not ok:
                LOGGER.error("Failed to create %s alias at %s", role, alias)
        else:
            _remove_managed_link(alias)
            if not candidates:
                LOGGER.info("No unique %s device; alias %s not created.", role, alias)
            else:
                LOGGER.warning(
                    "Multiple %s devices found; use --%s-match or a role-specific link: %s",
                    role,
                    role,
                    ", ".join(device.event_path for device in candidates),
                )
    return created


def sync_devices(
    *,
    sysfs_dir: Path,
    input_dir: Path,
    stable_dir: Path,
    state_file: Path,
    repair_nodes: bool,
    prune_stale: bool,
    force_prune: bool,
    keyboard_match: str,
    mouse_match: str,
    compat_dir: Path | None = None,
) -> list[InputDeviceInfo]:
    """Perform one synchronization pass and return current devices."""
    input_dir.mkdir(parents=True, exist_ok=True)
    stable_dir.mkdir(parents=True, exist_ok=True)
    devices = discover_devices(sysfs_dir, input_dir)
    active_events = {device.event_name for device in devices}

    for device in devices:
        _ensure_event_node(device, repair=repair_nodes)

    # A container may have a sysfs device but no corresponding /dev node until
    # mknod above has run.  Re-classify after node repair so newly materialized
    # keyboards and mice also receive role aliases during this pass.
    devices = [
        replace(
            device,
            roles=_classify_with_evdev(Path(device.event_path)),
        )
        for device in devices
    ]

    if prune_stale:
        if _is_mount_point(input_dir) and not force_prune:
            raise RuntimeError(
                f"{input_dir} is a mount point; refusing to remove stale nodes. "
                "Use --force-prune only after confirming it is container-local."
            )
        for path in sorted(input_dir.glob("event*")):
            if not EVENT_NAME_RE.match(path.name) or path.name in active_events:
                continue
            try:
                path_stat = os.lstat(path)
                if stat.S_ISCHR(path_stat.st_mode):
                    path.unlink()
                    LOGGER.info("Removed stale input node %s.", path)
            except OSError as exc:
                LOGGER.warning("Cannot remove stale input node %s: %s", path, exc)

    old_links = _read_managed_links(state_file)
    new_links = _install_stable_links(
        stable_dir, devices, keyboard_match, mouse_match
    )
    if compat_dir is not None and compat_dir.resolve() != stable_dir.resolve():
        new_links |= _install_stable_links(
            compat_dir, devices, keyboard_match, mouse_match
        )

    for old_link in old_links - new_links:
        _remove_managed_link(_managed_link_path(stable_dir, old_link))

    _write_state(state_file, devices, new_links)
    return devices


def _format_devices(devices: list[InputDeviceInfo]) -> str:
    lines = ["event  roles       name"]
    for device in devices:
        roles = ",".join(device.roles) or "-"
        lines.append(f"{device.event_name:<7} {roles:<10} {device.name}")
    return "\n".join(lines)


def _log_alias_status(path: Path, role: str) -> None:
    if path.is_symlink():
        LOGGER.info("%s alias: %s -> %s", role, path, os.readlink(path))
        return
    if os.path.lexists(path):
        LOGGER.warning("%s alias exists but is not a symlink: %s", role, path)
        return
    LOGGER.warning("%s alias was not created at %s", role, path)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Synchronize container input nodes and stable keyboard/mouse links."
    )
    parser.add_argument("--watch", action="store_true", help="repeat until interrupted")
    parser.add_argument("--once", action="store_true", help="run one synchronization pass (default)")
    parser.add_argument("--interval", type=float, default=0.5, help="watch interval in seconds")
    parser.add_argument("--repair-nodes", action="store_true", help="replace incorrect event nodes")
    parser.add_argument(
        "--prune-stale",
        action="store_true",
        help="remove event nodes absent from sysfs; container-local only by default",
    )
    parser.add_argument(
        "--force-prune",
        action="store_true",
        help="allow pruning when /dev/input is a separate mount",
    )
    parser.add_argument("--keyboard-match", default="", help="substring to select one keyboard")
    parser.add_argument("--mouse-match", default="", help="substring to select one mouse")
    parser.add_argument("--sysfs-dir", type=Path, default=DEFAULT_SYSFS_DIR)
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT_DIR)
    parser.add_argument("--stable-dir", type=Path, default=DEFAULT_STABLE_DIR)
    parser.add_argument(
        "--compat-dir",
        type=Path,
        default=DEFAULT_COMPAT_DIR,
        help="also mirror aliases here (empty string disables)",
    )
    parser.add_argument("--state-file", type=Path, default=DEFAULT_STATE_FILE)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    if args.interval <= 0:
        raise SystemExit("--interval must be positive")
    if args.force_prune and not args.prune_stale:
        raise SystemExit("--force-prune requires --prune-stale")
    if str(args.compat_dir) in {"", "-", "none"}:
        args.compat_dir = None

    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    previous_signature = None
    while True:
        devices = sync_devices(
            sysfs_dir=args.sysfs_dir,
            input_dir=args.input_dir,
            stable_dir=args.stable_dir,
            state_file=args.state_file,
            repair_nodes=args.repair_nodes,
            prune_stale=args.prune_stale,
            force_prune=args.force_prune,
            keyboard_match=args.keyboard_match,
            mouse_match=args.mouse_match,
            compat_dir=args.compat_dir,
        )
        signature = tuple(
            (device.event_name, device.stable_id, device.roles) for device in devices
        )
        if signature != previous_signature:
            LOGGER.info("Current input devices:\n%s", _format_devices(devices))
            _log_alias_status(args.stable_dir / "keyboard", "keyboard")
            _log_alias_status(args.stable_dir / "mouse", "mouse")
            previous_signature = signature

        if args.once or not args.watch or stop:
            return 0
        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
