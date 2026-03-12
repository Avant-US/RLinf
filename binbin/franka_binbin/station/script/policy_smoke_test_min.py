#!/usr/bin/env python3
"""
Policy smoke test using openpi_client (no camera, no robot).

Runs a few `infer()` calls against the PI0.5_droid websocket server and prints:
- server metadata
- actions shape/dtype
- first action row (joint velocities + gripper position)

Usage:
  python3 /home/a25181/Desktop/franka/station/script/policy_smoke_test_min.py \
    --host 10.239.121.23 --port 8003 --steps 3 --prompt "connectivity test"
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

sys.path.insert(0, "/home/a25181/Desktop/franka/vendor/openpi_client_src")
from openpi_client.websocket_client_policy import WebsocketClientPolicy


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="10.239.121.23")
    ap.add_argument("--port", type=int, default=8003)
    ap.add_argument("--steps", type=int, default=3)
    ap.add_argument("--prompt", default="connectivity test")
    args = ap.parse_args()

    print(f"[smoke] connecting to {args.host}:{args.port} ...")
    policy = WebsocketClientPolicy(host=args.host, port=args.port)
    print("[smoke] connected")

    meta = policy.get_server_metadata()
    print(f"[meta] keys: {sorted(meta.keys())[:50]}")
    print(f"[meta] full: {meta}")

    zero_img = np.zeros((224, 224, 3), dtype=np.uint8)
    q_zero = np.zeros(7, dtype=np.float32)
    g_zero = np.zeros(1, dtype=np.float32)

    for i in range(args.steps):
        obs = {
            "observation/exterior_image_1_left": zero_img,
            "observation/wrist_image_left": zero_img,
            "observation/joint_position": q_zero,
            "observation/gripper_position": g_zero,
            "prompt": args.prompt,
        }
        out = policy.infer(obs)
        actions = out.get("actions")
        if actions is not None:
            actions = np.array(actions)
            print(f"[step {i}] actions shape={actions.shape} dtype={actions.dtype}")
            if actions.ndim == 2 and actions.shape[1] == 8:
                vel = actions[0, :7]
                grip = actions[0, 7]
                print(f"[step {i}] vel_0={vel.tolist()} grip_0={grip:.4f}")
                print(f"[step {i}] vel range: [{actions[:, :7].min():.4f}, {actions[:, :7].max():.4f}]")
        else:
            print(f"[step {i}] out_keys={sorted(out.keys())} (no 'actions' key)")


if __name__ == "__main__":
    main()
