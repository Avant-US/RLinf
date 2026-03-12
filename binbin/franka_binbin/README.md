# Franka Research 3 -- PI0.5 Pick-and-Place

End-to-end system for controlling a **Franka Research 3** arm + **Franka Hand** gripper using the **PI0.5_droid** vision-language-action model.

## Hardware

| Component | Details |
|-----------|---------|
| Robot | Franka Research 3 (arm v2.1) + Franka Hand gripper |
| Workstation | Ubuntu 20.04, RT kernel 5.14.2-rt21, IP `10.239.66.34` (wifi) |
| Wrist camera | RealSense D435 mounted on Franka Hand (USB) |
| Head camera | RealSense D435 (USB, exterior view) |
| Policy server | PI0.5_droid at `10.239.121.23:8003` (WebSocket) |

## Network

| Connection | Workstation IP | Remote IP | Interface |
|------------|---------------|-----------|-----------|
| Franka Desk (arm web UI) | 192.168.0.2 | 192.168.0.1 | USB ethernet adapter |
| Franka Controller (FCI) | 172.16.0.1 | 172.16.0.2 | USB ethernet adapter |
| Company wifi | 10.239.66.34 | -- | wlp113s0 |

Switch between Desk and FCI with:

```bash
./switch_franka.sh desk    # plug cable to arm, access Desk UI at 192.168.0.1
./switch_franka.sh fci     # plug cable to controller, enable FCI at 172.16.0.2
./switch_franka.sh status  # show current IP and connectivity
```

## Project Structure

```
franka/
├── robot/                      # C++ robot server (runs on this workstation near FR3)
│   ├── robot_server.cpp        # Main: 1kHz control loop, TCP command/state API, gripper
│   ├── protocol.h              # Binary wire protocol (CommandMsgV1 / StateMsgV1)
│   ├── read_state.cpp          # Read-only FCI connectivity check
│   ├── recover_reflex.cpp      # Recover robot from Reflex/error state
│   ├── motion_test.cpp         # Motion test helper
│   └── CMakeLists.txt
├── station/                    # Python station (camera + policy + commands)
│   ├── run.py                  # Main loop: camera -> policy -> robot commands
│   ├── action_adapter.py       # Policy actions -> safe joint position targets
│   ├── openpi_min_client.py    # Minimal WebSocket PI0.5 client (no numpy needed)
│   ├── robot_client.py         # TCP client for robot_server
│   ├── robot_protocol.py       # Python mirror of protocol.h
│   ├── jog.py                  # Manual jog/hold (no policy)
│   ├── stop.py                 # Stop robot / shutdown server
│   └── logger.py               # JSONL logging
├── script/
│   └── test_cameras.py         # Preview/test RealSense cameras
├── vendor/
│   └── libfranka_0.20.4/       # Vendored libfranka for building robot_server
├── switch_franka.sh            # Switch USB ethernet between Desk/FCI
├── fix_rt.sh                   # Apply RT fixes (CPU governor, ethernet coalescing)
├── guide.md                    # Hardware notes and working commands
└── README.md                   # This file
```

## Quick Start

### 1. Build robot_server

```bash
cd /home/a25181/Desktop/franka/robot
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

### 2. Switch to FCI network

```bash
./switch_franka.sh fci
```

### 3. Recover robot (if needed)

```bash
cd /home/a25181/Desktop/franka/robot
./build/recover_reflex 172.16.0.2
```

### 4. Start robot_server

```bash
cd /home/a25181/Desktop/franka/robot
taskset -c 3 chrt -f 80 ./build/robot_server \
  --franka-ip 172.16.0.2 \
  --bind 0.0.0.0:15555 \
  --watchdog-ms 1500 \
  --max-joint-step 0.15 \
  --realtime-ignore
# NOTE: robot_server must be rebuilt after changing robot_server.cpp
#   cd robot && cmake --build build -j"$(nproc)"
```

### 5. Run the policy station

```bash
cd /home/a25181/Desktop/franka
PYTHONUNBUFFERED=1 python3 -m station.run \
  --robot-host 127.0.0.1 --robot-port 15555 \
  --policy-host 10.239.121.23 --policy-port 8003 \
  --prompt "pick up the doll on the table" \
  --use-realsense \
  --action-mode joint_position \
  --speed-scale 1.0 --max-joint-step 0.15 --hz 5 \
  --validity-ms 1500 \
  --enable-arm --enable-gripper \
  --steps 500
```

### 6. Stop

- Press **Ctrl+C** in the station terminal, then **Ctrl+C** in the robot_server terminal.
- If the robot is in Reflex (red light): `./build/recover_reflex 172.16.0.2`

See **TROUBLESHOOTING.md** if the arm wanders or never grasps (control vs model/scene diagnosis).

## Protocol (Station <-> Robot Server)

TCP, fixed-size binary, little-endian:

| Message | Direction | Size | Port |
|---------|-----------|------|------|
| CommandMsgV1 | station -> robot | 104 bytes | 15555 |
| StateMsgV1 | robot -> station | 160 bytes | 15556 |

See `robot/protocol.h` and `station/robot_protocol.py` for field details.

## Policy (PI0.5_droid)

- **Input**: observation dict with two 224x224 RGB images (wrist + exterior), 7 joint positions, 1 gripper position, and a text prompt
- **Output**: `actions (15, 8)` -- 15 future timesteps, each with 7 absolute joint positions + 1 gripper value
- **Action mode**: `joint_position` (policy outputs absolute target joint angles)
- **Transport**: WebSocket + msgpack at `10.239.121.23:8003`

## Robot Server Key Parameters

| Parameter | Description | Recommended |
|-----------|-------------|-------------|
| `--max-joint-step` | Max position change per station command (rad) | 0.15 |
| `--watchdog-ms` | Command freshness timeout (ms) | 1500 |
| `--realtime-ignore` | Allow running without RT enforcement | Use on this workstation |

The robot_server uses a **second-order interpolation** (acceleration-limited trajectory) to smoothly move between commanded positions, avoiding velocity/acceleration discontinuities that trigger Franka's reflex safety system.

## Station Key Parameters

| Parameter | Description | Recommended |
|-----------|-------------|-------------|
| `--action-mode` | `joint_position` or `joint_delta` | `joint_position` |
| `--speed-scale` | Multiplier on policy output (>1 to amplify) | 1.0 |
| `--max-joint-step` | Clamp per-step delta (rad) | 0.15 |
| `--validity-ms` | Command TTL for robot watchdog | 1500 |
| `--hz` | Station loop rate | 5 |
| `--wrist-serial` | RealSense serial for wrist camera | auto-detect |
| `--head-serial` | RealSense serial for head camera | auto-detect |

## Cameras

List connected cameras:

```bash
python3 -m station.run --list-cameras
# or
python3 script/test_cameras.py --list
```

Test camera capture:

```bash
python3 script/test_cameras.py                    # capture once, save to script/images/
python3 script/test_cameras.py --continuous        # capture every 2s
```

Assign specific cameras:

```bash
python3 -m station.run --use-realsense --wrist-serial XXXXX --head-serial YYYYY ...
```

## Utility Scripts| Script | Purpose |
|--------|---------|
| `switch_franka.sh` | Switch USB ethernet between Desk UI and FCI |
| `fix_rt.sh` | Set CPU governor to performance, disable ethernet coalescing |
| `script/test_cameras.py` | Test/preview RealSense cameras, save PNG snapshots |## Dependencies**Python** (miniconda3, Python 3.13):
- `msgpack`, `websockets` -- policy client
- `numpy`, `pyrealsense2` -- camera capture**C++ build**:
- libfranka 0.20.4 (vendored in `vendor/`)
- fmt (from miniconda3)
- Eigen3, Poco (system packages)


rsync -avz --delete \
  --exclude='.venv' \
  --exclude='__pycache__' \
  --exclude='libfranka_src' \
  --exclude='vendor' \
  --exclude='robot/package' \
  /home/a25181/Desktop/franka/ \
  hanbingbing@10.239.121.23:/data0/hanbingbing/franka2/

rsync -avz \
  --exclude='.venv' \
  --exclude='logs' \
  --exclude='__pycache__' \
  --exclude='libfranka_src' \
  --exclude='vendor' \
  --exclude='robot/package' \
  hanbingbing@10.239.121.23:/data0/hanbingbing/franka2/ \
  /home/a25181/Desktop/franka/


cd /home/a25181/Desktop/franka/robot
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j


部署步骤
1. 重新编译 robot_server（必须，protocol.h 和 robot_server.cpp 改了）：

cd /data0/hanbingbing/franka2/robot
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
2. 同步到机器人端：

rsync -avz --delete \
  --exclude='logs/' --exclude='robot/build/' --exclude='__pycache__/' \
  /data0/hanbingbing/franka2/ a25181@<机器人IP>:~/franka2/
3. 在机器人端重新编译并重启 server：

cd ~/franka2/robot && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
bash ~/franka2/run_server.sh
4. 运行测试：

bash run_test.sh "pick up the green towel"