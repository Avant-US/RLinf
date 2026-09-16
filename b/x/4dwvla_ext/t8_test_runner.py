#!/usr/bin/env python3
"""T8 automated test runner: exercises all 5 key bindings with real robot.

Creates UInput device in-process, spawns franka_vla_client as subprocess,
injects key events at the right moments.

Usage (in Franky container):
    source /opt/venv/franky-0.19.0/bin/activate
    python /workspace/RLinf/b/x/4dwvla_ext/t8_test_runner.py
"""
from __future__ import annotations

import ctypes
import fcntl
import glob
import json
import os
import re
import select
import struct
import subprocess
import sys
import time

EV_KEY = 0x01
EV_SYN = 0x00
SYN_REPORT = 0x00
UI_SET_EVBIT = 0x40045564
UI_SET_KEYBIT = 0x40045565
UI_DEV_CREATE = 0x5501

KEY_CODES = {"a": 30, "b": 48, "c": 46, "h": 35, "r": 19, "q": 16}

_uinput_fd = None
_uinput_dev_path = None


def create_uinput() -> str:
    global _uinput_fd, _uinput_dev_path

    fd = os.open("/dev/uinput", os.O_WRONLY | os.O_NONBLOCK)
    fcntl.ioctl(fd, UI_SET_EVBIT, EV_KEY)
    for code in KEY_CODES.values():
        fcntl.ioctl(fd, UI_SET_KEYBIT, code)

    name = b"T8-injector" + b"\x00" * 69
    setup = name + struct.pack("HHHHI", 0x03, 0x01, 0x01, 1, 0)
    setup += b"\x00" * (64 * 4 * 4)
    os.write(fd, setup)
    fcntl.ioctl(fd, UI_DEV_CREATE)
    _uinput_fd = fd

    time.sleep(0.3)

    for d in sorted(glob.glob("/sys/devices/virtual/input/input*")):
        try:
            n = open(os.path.join(d, "name")).read().strip()
        except Exception:
            continue
        if n == "T8-injector":
            for entry in sorted(os.listdir(d)):
                if entry.startswith("event") and entry[5:].isdigit():
                    dev_str = open(os.path.join(d, entry, "dev")).read().strip()
                    major, minor = map(int, dev_str.split(":"))
                    dev_path = f"/dev/input/{entry}"
                    if not os.path.exists(dev_path):
                        os.mknod(dev_path, 0o666 | 0o020000, os.makedev(major, minor))
                        os.chmod(dev_path, 0o666)
                    _uinput_dev_path = dev_path
                    print(f"[T8] UInput created: {dev_path} ({major}:{minor})")
                    return dev_path

    raise RuntimeError("Could not find UInput device in sysfs")


def inject_key(key_name: str):
    code = KEY_CODES[key_name]
    ts = time.time()
    sec = int(ts)
    usec = int((ts - sec) * 1e6)

    def write_event(typ, cod, val):
        ev = struct.pack("llHHi", sec, usec, typ, cod, val)
        os.write(_uinput_fd, ev)

    write_event(EV_KEY, code, 1)
    write_event(EV_SYN, SYN_REPORT, 0)
    time.sleep(0.05)
    write_event(EV_KEY, code, 0)
    write_event(EV_SYN, SYN_REPORT, 0)


def read_available(proc, timeout: float = 0.5) -> str:
    buf = ""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rem = max(0.01, deadline - time.monotonic())
        ready, _, _ = select.select([proc.stdout], [], [], min(rem, 0.1))
        if ready:
            chunk = os.read(proc.stdout.fileno(), 8192)
            if not chunk:
                break
            text = chunk.decode("utf-8", errors="replace")
            buf += text
            sys.stdout.write(text)
            sys.stdout.flush()
    return buf


def wait_for_pattern(proc, pattern: str, timeout: float = 60.0) -> str | None:
    deadline = time.monotonic() + timeout
    buf = ""
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        ready, _, _ = select.select([proc.stdout], [], [], min(remaining, 0.2))
        if ready:
            chunk = os.read(proc.stdout.fileno(), 8192)
            if not chunk:
                break
            text = chunk.decode("utf-8", errors="replace")
            buf += text
            sys.stdout.write(text)
            sys.stdout.flush()
            for line in buf.split("\n"):
                if re.search(pattern, line):
                    return line
            if "\n" in buf:
                buf = buf[buf.rfind("\n") + 1:]
    return None


def start_client(cmd, env):
    return subprocess.Popen(
        cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, env=env, bufsize=0,
    )


def kill_client(proc):
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    time.sleep(2)


def run_phase(phase_name, cmd, env, key_to_test, results):
    """Run one phase: start client, press 'a', wait for rollout, press test key."""
    print(f"\n{'=' * 60}")
    print(f"PHASE: {phase_name}")
    print("=" * 60)

    proc = start_client(cmd, env)

    line = wait_for_pattern(proc, r"(Waiting for 'a'|Arrange scene|press 'a')", timeout=60)
    if line is None:
        print(f"[T8] FAIL: Client did not reach ready state")
        kill_client(proc)
        results[f"{key_to_test}_key"] = "FAIL: timeout waiting for ready"
        return

    time.sleep(0.3)
    inject_key("a")

    line = wait_for_pattern(proc, r"'a' pressed.*starting rollout", timeout=10)
    if line is None:
        print("[T8] FAIL: 'a' key not recognized")
        kill_client(proc)
        results[f"{key_to_test}_key"] = "FAIL: 'a' not working"
        return

    if key_to_test == "a":
        results["a_key"] = "PASS"
        print("[T8] 'a' key: PASS - rollout started")
        # Wait for first inference to confirm
        line = wait_for_pattern(proc, r"Received \d+ actions", timeout=30)
        if line:
            print(f"[T8] Inference OK: {line.strip()}")
            results["a_inference"] = "PASS"
        # Let it finish
        wait_for_pattern(proc, r"Done:", timeout=60)
        kill_client(proc)
        return

    # Wait for first inference result, then inject test key immediately
    line = wait_for_pattern(proc, r"Received \d+ actions", timeout=30)
    if line is None:
        print("[T8] FAIL: No inference result")
        kill_client(proc)
        results[f"{key_to_test}_key"] = "FAIL: no inference"
        return

    # Inject test key after 1 step (0.2s at 5Hz)
    time.sleep(0.3)
    print(f"[T8] Injecting '{key_to_test}' key...")
    inject_key(key_to_test)

    if key_to_test == "c":
        line = wait_for_pattern(proc, r"'c' pressed.*success", timeout=10)
        if line:
            results["c_key"] = "PASS"
            print("[T8] 'c' key: PASS - success marked")
        else:
            results["c_key"] = "FAIL: 'c' not recognized"
            print("[T8] 'c' key: FAIL")

    elif key_to_test == "b":
        line = wait_for_pattern(proc, r"'b' pressed.*failure", timeout=10)
        if line:
            results["b_key"] = "PASS"
            print("[T8] 'b' key: PASS - failure marked")
        else:
            results["b_key"] = "FAIL: 'b' not recognized"
            print("[T8] 'b' key: FAIL")

    elif key_to_test == "h":
        line = wait_for_pattern(proc, r"HOME.*'h' key", timeout=10)
        if line:
            results["h_key"] = "PASS"
            print("[T8] 'h' key: PASS - HOME triggered")
        else:
            results["h_key"] = "FAIL: 'h' not recognized"
            print("[T8] 'h' key: FAIL")

    elif key_to_test == "r":
        line = wait_for_pattern(proc, r"ABORT.*'r' key", timeout=10)
        if line:
            results["r_key_abort"] = "PASS"
            print("[T8] 'r' key abort: PASS")
        else:
            results["r_key_abort"] = "FAIL: 'r' not recognized"
            print("[T8] 'r' key abort: FAIL")
            read_available(proc, 3.0)
            kill_client(proc)
            return

        line = wait_for_pattern(proc, r"(truncated|Reset scene)", timeout=10)
        if line:
            results["r_key_truncated"] = "PASS"
            print(f"[T8] truncated: PASS")
            time.sleep(0.3)
            try:
                proc.stdin.write(b"\n")
                proc.stdin.flush()
            except Exception as e:
                print(f"[T8] stdin write: {e}")

            line = wait_for_pattern(proc, r"(Waiting for 'a'|press 'a'|Arrange scene)", timeout=30)
            if line:
                results["r_key_reset"] = "PASS"
                print("[T8] reset cycle: PASS - back to waiting")
            else:
                results["r_key_reset"] = "FAIL: no return to waiting"
        else:
            results["r_key_truncated"] = "FAIL: no truncated"

    read_available(proc, 2.0)
    kill_client(proc)


def main():
    print("=== T8 Test Runner ===")

    keyboard_dev = create_uinput()
    print(f"Keyboard device: {keyboard_dev}")

    results = {}

    cmd = [
        sys.executable,
        "/workspace/RLinf/b/x/4dwvla_ext/franka_vla_client.py",
        "--robot-ip", "172.16.0.2",
        "--task", "plug into socket",
        "--use-realsense",
        "--global-camera-serial", "420122070525",
        "--wrist-camera-serial", "250222073513",
        "--max-steps", "50",
        "--control-hz", "5",
    ]

    env = os.environ.copy()
    env["RLINF_KEYBOARD_DEVICE"] = keyboard_dev

    # Phase 1: 'a' key (start rollout) + basic inference
    run_phase("Test 'a' (start rollout)", cmd, env, "a", results)

    # Phase 2: 'c' key (mark success)
    run_phase("Test 'c' (mark success)", cmd, env, "c", results)

    # Phase 3: 'h' key (HOME during rollout)
    run_phase("Test 'h' (HOME mid-rollout)", cmd, env, "h", results)

    # Phase 4: 'b' key (mark failure)
    run_phase("Test 'b' (mark failure)", cmd, env, "b", results)

    # Phase 5: 'r' key (abort reset)
    run_phase("Test 'r' (abort reset)", cmd, env, "r", results)

    # ── Summary ──────────────────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    print("T8 TEST RESULTS SUMMARY")
    print("=" * 60)
    for key, result in results.items():
        status = "PASS" if "PASS" in result else "FAIL"
        print(f"  [{status}] {key}: {result}")

    all_pass = all("PASS" in v for v in results.values())
    print(f"\nOverall: {'ALL PASS' if all_pass else 'SOME FAILURES'}")

    with open("/tmp/t8_results.json", "w") as f:
        json.dump(results, f, indent=2)

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
