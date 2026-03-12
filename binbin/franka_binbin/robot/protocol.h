// Station ↔ Robot wire protocol (binary, fixed-size).
//
// This header mirrors `station/robot_protocol.py`.
// Endianness: little-endian (assumes x86_64 on both sides).
//
// TCP ports (recommended):
// - control / commands: 5555
// - state stream:      5556 (control_port + 1)

#pragma once

#include <array>
#include <cstdint>

namespace franka_station_protocol {

static constexpr std::array<char, 4> kCmdMagic = {'F', 'C', 'M', '1'};
static constexpr std::array<char, 4> kStateMagic = {'F', 'S', 'T', '1'};
static constexpr std::uint32_t kVersion = 1;

enum class CmdType : std::uint32_t {
  kHeartbeat = 0,
  kArmCommand = 1,
  kStop = 2,
  kShutdown = 3,  // request robot_server shutdown (if enabled)
};

enum class CmdMode : std::uint32_t {
  kHold = 0,
  kJointPosition = 1,
  kJointDelta = 2,
  kJointVelocity = 3,
};

enum Flags : std::uint32_t {
  kEnableArm = 1u << 0,
  kEnableGripper = 1u << 1,
};

// 104 bytes total. Layout must match Python struct "<4sIQQIIII8d".
struct CommandMsgV1 {
  std::array<char, 4> magic;      // "FCM1"
  std::uint32_t version;          // 1
  std::uint64_t seq;              // monotonically increasing
  std::uint64_t sent_time_ns;     // station time (monotonic or epoch)
  std::uint32_t cmd_type;         // CmdType
  std::uint32_t mode;             // CmdMode
  std::uint32_t validity_ms;      // command TTL for watchdog
  std::uint32_t flags;            // Flags bitmask
  double q[7];                    // target or delta (rad)
  double gripper;                 // width (m) or delta
};

static_assert(sizeof(CommandMsgV1) == 104, "CommandMsgV1 must be 104 bytes");

// 160 bytes total. Layout must match Python struct "<4sIQQII16d".
struct StateMsgV1 {
  std::array<char, 4> magic;      // "FST1"
  std::uint32_t version;          // 1
  std::uint64_t robot_time_ns;    // robot monotonic time
  std::uint64_t last_cmd_seq;     // last command seq applied/seen
  std::uint32_t robot_mode;       // 0=idle,1=running,2=error,3=mock
  std::uint32_t error_code;       // 0 OK
  double q[7];                    // measured q (rad)
  double dq[7];                   // measured dq (rad/s)
  double gripper;                 // gripper width (m), if available
  double cmd_age_ms;              // age of last command (ms)
};

static_assert(sizeof(StateMsgV1) == 160, "StateMsgV1 must be 160 bytes");

}  // namespace franka_station_protocol

