"""测试 SpaceMouse 是否能正常读取。

用法（在 Franky 容器中）：
    source b/rlt/configs/setup_franky.sh
    pip install pyspacemouse  # 如果未安装
    python b/rlt/scripts/test_spacemouse.py
"""

import sys
import time


def main():
    print("[1/3] Checking pyspacemouse installation ...")
    try:
        import pyspacemouse
    except ImportError:
        print("ERROR: pyspacemouse not installed.")
        print("Run:   pip install pyspacemouse")
        sys.exit(1)

    print("[2/3] Opening SpaceMouse device ...")
    try:
        device = pyspacemouse.open(device_index=0)
    except Exception as e:
        print(f"ERROR: Failed to open SpaceMouse: {e}")
        print("Check:")
        print("  1. SpaceMouse is connected via USB")
        print("  2. Docker has --privileged flag")
        print("  3. ls /dev/input/event* shows the device")
        sys.exit(1)

    if not device:
        print("ERROR: pyspacemouse.open() returned None.")
        sys.exit(1)

    print("[3/3] Reading SpaceMouse state (Ctrl+C to stop) ...")
    print("       Move SpaceMouse to see values change")
    print()
    try:
        while True:
            state = device.read()
            action = [
                -state.y,
                state.x,
                state.z,
                -state.roll,
                -state.pitch,
                -state.yaw,
            ]
            has_motion = any(abs(a) > 0.001 for a in action)
            marker = " <-- ACTIVE" if has_motion else ""
            print(
                f"\r  xyz=[{action[0]:+.3f}, {action[1]:+.3f}, {action[2]:+.3f}]  "
                f"rpy=[{action[3]:+.3f}, {action[4]:+.3f}, {action[5]:+.3f}]  "
                f"btn=[{state.buttons}]{marker}     ",
                end="",
                flush=True,
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nSpaceMouse test done.")


if __name__ == "__main__":
    main()
