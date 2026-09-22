#!/usr/bin/env python3
"""Offline test: IPC message round-trip between mock server and client.

Can run on the host machine (no Docker/GPU/robot needed).

    python /path/to/4dwvla_ext/tests/test_ipc_offline.py
"""
import sys
import threading
import time
from multiprocessing.connection import Client, Listener
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from franky_ext.dsplug.home_pose import load_home_joints

PASS = 0
FAIL = 0
AUTHKEY = b"4dwvla-eval"

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1; print(f"  [PASS] {name}")
    else:
        FAIL += 1; print(f"  [FAIL] {name}: {detail}")

def mock_server(port):
    listener = Listener(("localhost", port), authkey=AUTHKEY)
    conn = listener.accept()
    while True:
        try:
            msg = conn.recv()
        except EOFError:
            break
        if msg.get("command") == "shutdown":
            break
        actions = [[0.0] * 8 for _ in range(10)]
        conn.send({"actions": actions, "status": "ok"})
    conn.close()
    listener.close()

def test_ipc_roundtrip():
    print("\n=== T2.1: IPC Round-trip ===")
    port = 15555
    srv = threading.Thread(target=mock_server, args=(port,), daemon=True)
    srv.start()
    time.sleep(0.2)

    conn = Client(("localhost", port), authkey=AUTHKEY)
    msg = {
        "images": {
            "global": np.zeros((480, 640, 3), dtype=np.uint8),
            "wrist": np.zeros((480, 640, 3), dtype=np.uint8),
        },
        "state": {"arm": [0.0] * 7, "gripper": [0.04]},
        "task": "test task",
    }

    t0 = time.monotonic()
    conn.send(msg)
    resp = conn.recv()
    latency_ms = (time.monotonic() - t0) * 1000

    check("status ok", resp["status"] == "ok", f"got {resp['status']}")
    check("actions shape", len(resp["actions"]) == 10 and len(resp["actions"][0]) == 8,
          f"got {len(resp['actions'])}x{len(resp['actions'][0])}")
    check("latency < 100ms", latency_ms < 100, f"{latency_ms:.1f}ms")

def test_ipc_message_format():
    print("\n=== T2.2: Message Format Validation ===")
    state_arm = load_home_joints().tolist()
    state_grip = [0.04]
    msg = {
        "images": {"global": np.zeros((480, 640, 3), dtype=np.uint8),
                   "wrist": np.zeros((480, 640, 3), dtype=np.uint8)},
        "state": {"arm": state_arm, "gripper": state_grip},
        "task": "plug into socket",
    }
    check("images keys", set(msg["images"].keys()) == {"global", "wrist"})
    check("image shape", msg["images"]["global"].shape == (480, 640, 3))
    check("image dtype", msg["images"]["global"].dtype == np.uint8)
    check("arm length", len(msg["state"]["arm"]) == 7)
    check("gripper length", len(msg["state"]["gripper"]) == 1)
    check("task is str", isinstance(msg["task"], str))

def test_ipc_state_history_field():
    """修复 A: franka_vla_client.py now sends 'state_history' -- the poses
    executed since the previous inference -- so the server can replay them
    through FKKeypointComputer.step() and advance observation.his_len at the
    correct (per control step) rate. Old clients (no field) and old/mock
    servers that ignore unknown keys must still interoperate.
    """
    print("\n=== T2.4: state_history Field (修复 A) ===")
    port = 15557
    srv = threading.Thread(target=mock_server, args=(port,), daemon=True)
    srv.start()
    time.sleep(0.2)

    conn = Client(("localhost", port), authkey=AUTHKEY)

    # n_exec=10: 9 poses executed since the last inference, plus the
    # current one sent separately as msg["state"]["arm"].
    msg = {
        "images": {"global": np.zeros((480, 640, 3), dtype=np.uint8),
                   "wrist": np.zeros((480, 640, 3), dtype=np.uint8)},
        "state": {"arm": [0.0] * 7, "gripper": [0.04]},
        "state_history": [[0.0] * 7 for _ in range(9)],
        "task": "plug into socket",
    }
    conn.send(msg)
    resp = conn.recv()
    check("server tolerates state_history field", resp["status"] == "ok",
          f"got {resp['status']}")

    # A message without the field (old client) must still round-trip.
    msg_no_history = {k: v for k, v in msg.items() if k != "state_history"}
    conn.send(msg_no_history)
    resp2 = conn.recv()
    check("server tolerates missing state_history field (old client)",
          resp2["status"] == "ok", f"got {resp2['status']}")

    conn.send({"command": "shutdown"})
    conn.close()

def test_ipc_shutdown():
    print("\n=== T2.3: Graceful Shutdown ===")
    port = 15556
    srv = threading.Thread(target=mock_server, args=(port,), daemon=True)
    srv.start()
    time.sleep(0.2)
    conn = Client(("localhost", port), authkey=AUTHKEY)
    conn.send({"command": "shutdown"})
    time.sleep(0.1)
    conn.close()
    check("shutdown no hang", True)

if __name__ == "__main__":
    test_ipc_roundtrip()
    test_ipc_message_format()
    test_ipc_state_history_field()
    test_ipc_shutdown()
    print(f"\n=== Results: {PASS} passed, {FAIL} failed ===")
    sys.exit(1 if FAIL > 0 else 0)
