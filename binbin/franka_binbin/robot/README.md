# `robot/` project (robot-side PC on/near FR3)

This project will run on the **robot-side PC** (mounted on / near the Franka Research 3 body).

## Responsibilities

- Maintain the **1 kHz realtime control loop** via FCI/`libfranka`.
- Expose a **network API** to the station:
  - receive sanitized targets (arm + gripper)
  - stream robot state back (q, dq, gripper width, status/errors)
- Enforce **safety**:
  - command watchdog (heartbeat)
  - joint limit clamps
  - smooth interpolation to 1 kHz
  - configured collision behavior / impedance where appropriate

## Pre-flight checklist (robot-side PC)

### A) Verify realtime kernel

```bash
uname -a
test -f /sys/kernel/realtime && cat /sys/kernel/realtime || echo "No /sys/kernel/realtime (not RT)"
```

### B) Realtime permissions

```bash
groups | tr ' ' '\n' | grep -x realtime || echo "User not in realtime group"
ulimit -r
ulimit -l
```

If needed, follow the commands in `docs/RUNBOOK.md` to add the `realtime` group and limits.

## Install dependencies

### libfranka build dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake git libpoco-dev libeigen3-dev
```

### Install libfranka (FR3)

`libfranka` must be compatible with your FR3 controller's FCI server version. If you see an error like:

> `Incompatible library version (server version: 10, library version: 6)`

then your program is linking against an **older** libfranka and you need to install a newer one.

Recommended (Ubuntu 20.04): install from the official release `.deb` files:

- `https://github.com/frankarobotics/libfranka/releases`

```bash
LIBFRANKA_VER="0.20.4"
cd ~
curl -L -o "libfranka_${LIBFRANKA_VER}_focal_amd64.deb" \
  "https://github.com/frankarobotics/libfranka/releases/download/${LIBFRANKA_VER}/libfranka_${LIBFRANKA_VER}_focal_amd64.deb"
sudo dpkg -i "libfranka_${LIBFRANKA_VER}_focal_amd64.deb" || sudo apt-get install -f
```

No-sudo / local install (useful on locked-down machines): extract the `.deb` and point CMake at it:

```bash
LIBFRANKA_VER="0.20.4"
dpkg-deb -x "libfranka_${LIBFRANKA_VER}_focal_amd64.deb" ./libfranka_local
cmake -S /path/to/franka/robot -B build -DCMAKE_BUILD_TYPE=Release \
  -DFranka_DIR="$(pwd)/libfranka_local/usr/lib/cmake/Franka"
```

## Verify the robot (before writing any custom code)

Set the arm IP:

```bash
export FRANKA_ARM_IP="<franka-arm-ip-for-FCI>"
ping -c 2 "${FRANKA_ARM_IP}"
```

Build and run this repo's read-only connectivity test (does not require a realtime kernel):

```bash
cd /path/to/franka/robot
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
./build/read_state --franka-ip "${FRANKA_ARM_IP}"
```

If you later test libfranka motion examples, do it in a large clear volume and at low speeds first
(and expect them to require realtime scheduling on some setups):

```bash
# If you installed libfranka system-wide (via .deb), this is usually in PATH:
generate_joint_velocity_motion "${FRANKA_ARM_IP}"

# If you extracted a local .deb instead:
./libfranka_local/usr/bin/generate_joint_velocity_motion "${FRANKA_ARM_IP}"
```

## Runtime behavior (implemented baseline)

### Realtime loop (1 kHz)

- Interface: **joint position motion generator** (recommended for first version).
- Internal controller: use Franka internal joint impedance controller to track commanded positions.
- Smoothness:
  - robot_server keeps the last received target and generates a smooth 1 kHz trajectory (interpolation).
  - optional `libfranka` smoothing features may be enabled once the signal is already smooth.

### Watchdog

- Station must send commands at a configured minimum rate (e.g., >= 5 Hz).
- If no command within `T_timeout` (e.g., 200 ms), robot_server:
  - freezes target (hold)
  - keeps holding until fresh commands resume (or the process is stopped)

### Gripper control

Gripper commands are **non-realtime** in `libfranka` (blocking TCP calls). We will:

- run gripper control in a separate thread
- rate-limit gripper commands (avoid spamming `move/grasp`)
- support:
  - `homing` on startup (optional)
  - `move(width, speed)` for position targets

## Build/run commands

Ports:

- Command port: `5555` (configurable via `--bind`)
- State stream port: `5556` (defaults to `bind_port + 1`, configurable via `--state-bind`)

```bash
cd /path/to/franka/robot
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"

# Mock mode (no libfranka required) - useful to test station ↔ robot networking:
./build/robot_server --bind 0.0.0.0:5555 --mock

# Real mode (robot-side PC, libfranka required):
sudo chrt -f 80 ./build/robot_server --franka-ip "${FRANKA_ARM_IP}" --bind 0.0.0.0:5555

# Optional: allow the station to request the server to exit:
#   (then `python -m station.stop --shutdown-server ...` will stop the process)
# sudo chrt -f 80 ./build/robot_server --franka-ip "${FRANKA_ARM_IP}" --bind 0.0.0.0:5555 --allow-shutdown
```

Stop the server:

- Press `Ctrl+C` in the terminal running `robot_server`, or
- From the station (requires `robot_server --allow-shutdown`):

```bash
python -m station.stop --robot-host "<robot-side-pc-ip>" --robot-port 5555 --shutdown-server
```

## Troubleshooting quick commands

- Check packet loss / command success (once we expose it via state stream):

```bash
# planned: station prints control_command_success_rate from streamed robot state
```

- If you hit an FCI error and it is safe to recover:

```bash
# Example (non-realtime): automatic error recovery (to be used carefully)
# robot.automaticErrorRecovery();
```

