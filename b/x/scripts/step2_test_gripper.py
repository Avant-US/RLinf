#!/usr/bin/env python3
"""Step 2: standalone Franka Hand gripper test via FrankaLibfrankaGripper."""

import os
import sys
import time

REPO = os.environ.get(
    "REPO_PATH", os.path.abspath(os.path.join(__file__, "../../../.."))
)
sys.path.insert(0, os.path.join(REPO, "b", "x"))

from franky_ext.franka_libfranka_gripper import FrankaLibfrankaGripper


def main() -> int:
    robot_ip = os.environ.get("FRANKA_ROBOT_IP", "172.16.0.2")
    print(f"Step2: FrankaLibfrankaGripper on {robot_ip}")
    grip = FrankaLibfrankaGripper(robot_ip=robot_ip)
    assert grip.is_ready(), "gripper not ready"
    print(f"initial: pos={grip.position:.4f} open={grip.is_open}")
    grip.open(speed=0.5)
    time.sleep(2.0)
    print(f"after open: pos={grip.position:.4f} open={grip.is_open}")
    assert grip.position > 0.06, f"open width too small: {grip.position}"
    grip.close(speed=0.5)
    time.sleep(2.0)
    print(f"after close: pos={grip.position:.4f} open={grip.is_open}")
    print("Step2 PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
