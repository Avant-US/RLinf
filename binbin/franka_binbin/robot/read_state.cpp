// Read-only FCI connectivity test (no motion).
//
// Usage:
//   ./build/read_state --franka-ip 192.168.0.1
//
// Notes:
// - This does NOT start a 1 kHz control loop; it only reads one robot state.
// - It can run on non-RT kernels, but you still must enable FCI in Desk.

#include <franka/exception.h>
#include <franka/robot.h>

#include <iostream>
#include <string>

namespace {

void usage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " --franka-ip <IP>\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string ip;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--franka-ip" && i + 1 < argc) {
      ip = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      usage(argv[0]);
      return 2;
    }
  }

  if (ip.empty()) {
    usage(argv[0]);
    return 2;
  }

  try {
    franka::Robot robot(ip, franka::RealtimeConfig::kIgnore);
    const franka::RobotState st = robot.readOnce();

    std::cout << "Connected to FRANKA at " << ip << "\n";
    std::cout << "q: ";
    for (int i = 0; i < 7; ++i) std::cout << st.q[i] << (i == 6 ? "" : ", ");
    std::cout << "\n";

    std::cout << "dq: ";
    for (int i = 0; i < 7; ++i) std::cout << st.dq[i] << (i == 6 ? "" : ", ");
    std::cout << "\n";

    std::cout << "robot_mode: " << static_cast<int>(st.robot_mode) << "\n";
  } catch (const franka::Exception& e) {
    std::cerr << "libfranka error: " << e.what() << "\n";
    std::cerr << "Make sure FCI is enabled in Desk and no other FCI client is connected.\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}

