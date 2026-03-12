# Hardware & Network Configuration

## Machine

- Ubuntu 20.04, RT kernel 5.14.2-rt21
- IP: `10.239.66.34` (wifi)

## Franka Research 3

- Arm version 2.1
- Gripper: Franka Hand
- FCI docs: https://frankarobotics.github.io/docs/

## Network Layout

| Connection | Our IP | Remote IP | Cable |
|------------|--------|-----------|-------|
| Franka Desk (arm web UI) | 192.168.0.2/24 | 192.168.0.1 | USB ethernet to arm |
| Franka Controller (FCI) | 172.16.0.1/24 | 172.16.0.2 | USB ethernet to controller |
| Company wifi | 10.239.66.34 | -- | wlp113s0 |
| Policy server | -- | 10.239.121.23:8003 | via wifi |

FCI only works when connected to 172.16.0.2 using 172.16.0.1.

## Switch Network

```bash
./switch_franka.sh fci     # for robot control
./switch_franka.sh desk    # for Desk web UI
./switch_franka.sh status  # check current state
```

## Cameras

- Wrist: RealSense D435 on Franka Hand (USB)
- Head: RealSense D435 exterior view (USB)
- Both need USB 3.0 ports, ideally on different USB controllers

```bash
python3 script/test_cameras.py --list       # list cameras
python3 script/test_cameras.py              # capture test images to script/images/
```

---

## Build

```bash
cd /home/a25181/Desktop/franka/robot
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -Dfmt_DIR=/home/a25181/miniconda3/lib/cmake/fmt
cmake --build build -j"$(nproc)"
```

## Working Commands

### Recover from Reflex

```bash
cd /home/a25181/Desktop/franka/robot
./build/recover_reflex 172.16.0.2
```

### Read Robot State (no motion)

```bash
./build/read_state --franka-ip 172.16.0.2
```

### Start Robot Server

```bash
taskset -c 3 chrt -f 80 ./build/robot_server \
  --franka-ip 172.16.0.2 \
  --bind 0.0.0.0:15555 \
  --watchdog-ms 1500 \
  --max-joint-step 0.15 \
  --realtime-ignore
```

### Run Policy (pick up doll)

```bash
cd /home/a25181/Desktop/franka
PYTHONUNBUFFERED=1 python3 -m station.run \
  --robot-host 127.0.0.1 --robot-port 15555 \
  --policy-host 10.239.121.23 --policy-port 8003 \
  --prompt "pick up the doll on the table" \
  --use-realsense \
  --action-mode joint_delta \
  --speed-scale 1.0 --max-joint-step 0.15 --hz 5 \
  --validity-ms 1500 \
  --enable-arm --enable-gripper \
  --steps 500
```

### Tuning Speed

- **Too fast/jerky**: decrease `--max-joint-step` (try 0.08)
- **Too slow**: increase `--max-joint-step` (try 0.25)
- **Amplify policy**: increase `--speed-scale` (try 2.0)

### Manual Jog (no policy)

```bash
# Hold position for 2 seconds
python3 -m station.jog --robot-host 127.0.0.1 --robot-port 15555 \
  --hold --enable-arm --duration-s 2

# Small joint 7 movement
python3 -m station.jog --robot-host 127.0.0.1 --robot-port 15555 \
  --joint 7 --delta 0.05 --rate-hz 10 --duration-s 5 --enable-arm
```

### Stop / Shutdown

```bash
# Stop robot (hold position)
python3 -m station.stop --robot-host 127.0.0.1 --robot-port 15555

# Shutdown robot_server process
python3 -m station.stop --robot-host 127.0.0.1 --robot-port 15555 --shutdown-server
```

### Apply RT Fixes

```bash
sudo bash fix_rt.sh
```
