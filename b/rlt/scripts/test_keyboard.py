"""Quick keyboard test — press a/b/c on the PHYSICAL keyboard."""
import os
from evdev import InputDevice, ecodes

path = os.environ.get("RLINF_KEYBOARD_DEVICE", "/dev/input/event2")
dev = InputDevice(path)
print(f"Listening on: {dev.name} ({path})")
print("Now press a / b / c on the PHYSICAL keyboard (Ctrl+C to exit)...")
print("(Do NOT type into this terminal — just press keys on the keyboard)")
print()
for event in dev.read_loop():
    if event.type == ecodes.EV_KEY and event.value == 1:
        name = ecodes.bytype[ecodes.EV_KEY].get(event.code, "?")
        print(f"  >>> Key pressed: {name}")
