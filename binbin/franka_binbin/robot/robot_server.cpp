// Robot-side server: receives station commands, runs a control loop, streams robot state.
//
// Build:
//   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
//   cmake --build build -j
//
// Run (mock mode on any machine):
//   ./build/robot_server --bind 0.0.0.0:5555 --mock
//
// Run (real robot-side PC, if libfranka installed and FCI enabled):
//   sudo chrt -f 80 ./build/robot_server --franka-ip <FRANKA_ARM_IP> --bind 0.0.0.0:5555
//
// Protocol:
//   - command TCP port: 5555 (CommandMsgV1, fixed 104 bytes)
//   - state  TCP port:  5556 (StateMsgV1, fixed 160 bytes)

#include "protocol.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <thread>

#if ROBOT_HAS_FRANKA
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/lowpass_filter.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#endif

using franka_station_protocol::CmdMode;
using franka_station_protocol::CmdType;
using franka_station_protocol::CommandMsgV1;
using franka_station_protocol::Flags;
using franka_station_protocol::StateMsgV1;

namespace {

std::atomic<bool> g_shutdown{false};
std::atomic<double> g_gripper_width{0.0};

void on_sigint(int) {
  g_shutdown.store(true);
}

// ... (rest of imports)

#if ROBOT_HAS_FRANKA
// gripper_thread_func moved below DoubleBuffer definition
#endif


std::uint64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<std::uint64_t>(ts.tv_nsec);
}

int num_cpus_online() {
  const long n = ::sysconf(_SC_NPROCESSORS_ONLN);
  return (n > 0) ? static_cast<int>(n) : 1;
}

void prefault_stack() {
  // Touch stack pages up front to avoid page faults later (especially inside the 1 kHz control loop).
  //
  // Keep this conservative: large allocations risk blowing the default pthread stack size.
  static constexpr std::size_t kBytes = 256u * 1024u;
  const long page_sz = ::sysconf(_SC_PAGESIZE);
  const std::size_t step = (page_sz > 0) ? static_cast<std::size_t>(page_sz) : 4096u;
  volatile std::uint8_t dummy[kBytes];
  for (std::size_t i = 0; i < kBytes; i += step) {
    dummy[i] = 0;
  }
  dummy[kBytes - 1] = 0;
  // Make the object "used" to silence -Wunused-but-set-variable under -Wall.
  const std::uint8_t sink = dummy[0];
  (void)sink;
}

cpu_set_t make_cpuset(std::initializer_list<int> cpus, int ncpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  bool any = false;
  for (const int c : cpus) {
    if (c >= 0 && c < ncpu) {
      CPU_SET(static_cast<unsigned>(c), &set);
      any = true;
    }
  }
  if (!any) {
    CPU_SET(0, &set);
  }
  return set;
}

void configure_current_thread(const char* tag, const cpu_set_t& affinity, int policy, int prio) {
  prefault_stack();

  const int aff_r = ::pthread_setaffinity_np(::pthread_self(), sizeof(cpu_set_t), &affinity);
  if (aff_r != 0) {
    std::cerr << "[" << tag << "] pthread_setaffinity_np failed: " << std::strerror(aff_r) << "\n";
  }

  // Clamp priority to the valid range for the requested policy.
  if (policy == SCHED_RR || policy == SCHED_FIFO) {
    const int lo = ::sched_get_priority_min(policy);
    const int hi = ::sched_get_priority_max(policy);
    prio = std::max(lo, std::min(hi, prio));
  } else {
    prio = 0;
  }

  sched_param sp{};
  sp.sched_priority = prio;
  const int sched_r = ::pthread_setschedparam(::pthread_self(), policy, &sp);
  if (sched_r != 0) {
    std::cerr << "[" << tag << "] pthread_setschedparam failed: " << std::strerror(sched_r) << "\n";
  }
}

void configure_network_thread(const char* tag) {
  const int ncpu = num_cpus_online();
  const cpu_set_t set = (ncpu >= 2) ? make_cpuset({0, 1}, ncpu) : make_cpuset({0}, ncpu);
  configure_current_thread(tag, set, SCHED_OTHER, /*prio=*/0);
}

void configure_control_thread(const char* tag) {
  const int ncpu = num_cpus_online();
  // Pin control thread to a dedicated CPU (avoid CPU 0 which handles network IRQs).
  const int ctrl_cpu = (ncpu >= 4) ? 3 : (ncpu >= 2) ? (ncpu - 1) : 0;
  const cpu_set_t set = make_cpuset({ctrl_cpu}, ncpu);
  configure_current_thread(tag, set, SCHED_FIFO, /*prio=*/80);
}

struct Args {
  std::string bind_host = "0.0.0.0";
  int bind_port = 5555;
  std::string state_host = "0.0.0.0";
  int state_port = 5556;

  std::string franka_ip;
  bool mock = false;
  bool realtime_ignore = false;  // allow running on non-RT kernels for testing
  bool allow_shutdown = false;   // allow station to request server exit

  int state_hz = 50;
  int watchdog_ms = 200;

  // Safety / smoothing (applies to both mock and real control).
  double filter_tau_s = 0.05;     // 50 ms time constant
  double max_joint_step = 0.05;   // rad per command (station-side should also clamp)
  double max_joint_delta = 0.20;  // DROID velocity→delta scale: delta = clip(vel,-1,1) * this
};

void print_usage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " [--bind HOST:PORT] [--state-bind HOST:PORT] "
      << "[--franka-ip IP] [--mock] [--realtime-ignore] [--allow-shutdown] [--state-hz N] [--watchdog-ms N]\n"
      << "  --bind HOST:PORT        command port (default 0.0.0.0:5555)\n"
      << "  --state-bind HOST:PORT  state stream port (default bind_port+1)\n"
      << "  --franka-ip IP          required for real mode\n"
      << "  --mock                  run mock control loop (no libfranka required)\n"
      << "  --realtime-ignore       do not enforce realtime scheduling (TEST ONLY)\n"
      << "  --allow-shutdown        allow station to request robot_server exit\n"
      << "  --state-hz N            state publish rate (default 50)\n"
      << "  --watchdog-ms N         command freshness timeout (default 200)\n"
      << "  --filter-tau-s S        first-order smoothing time constant (default 0.05)\n"
      << "  --max-joint-step R      clamp joint_delta per command (default 0.10)\n";
}

bool split_host_port(const std::string& s, std::string* host, int* port) {
  const auto pos = s.rfind(':');
  if (pos == std::string::npos) return false;
  *host = s.substr(0, pos);
  const std::string port_str = s.substr(pos + 1);
  try {
    *port = std::stoi(port_str);
  } catch (...) {
    return false;
  }
  if (*host == "") *host = "0.0.0.0";
  return true;
}

std::optional<Args> parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value after " + arg);
      }
      return argv[++i];
    };
    try {
      if (arg == "--help" || arg == "-h") {
        print_usage(argv[0]);
        return std::nullopt;
      } else if (arg == "--bind") {
        const auto v = next();
        if (!split_host_port(v, &a.bind_host, &a.bind_port)) {
          throw std::runtime_error("bad --bind, expected HOST:PORT");
        }
        // Default state port tracks bind port unless overridden later.
        a.state_host = a.bind_host;
        a.state_port = a.bind_port + 1;
      } else if (arg == "--state-bind") {
        const auto v = next();
        if (!split_host_port(v, &a.state_host, &a.state_port)) {
          throw std::runtime_error("bad --state-bind, expected HOST:PORT");
        }
      } else if (arg == "--franka-ip") {
        a.franka_ip = next();
      } else if (arg == "--mock") {
        a.mock = true;
      } else if (arg == "--realtime-ignore") {
        a.realtime_ignore = true;
      } else if (arg == "--allow-shutdown") {
        a.allow_shutdown = true;
      } else if (arg == "--state-hz") {
        a.state_hz = std::stoi(next());
      } else if (arg == "--watchdog-ms") {
        a.watchdog_ms = std::stoi(next());
      } else if (arg == "--filter-tau-s") {
        a.filter_tau_s = std::stod(next());
      } else if (arg == "--max-joint-step") {
        a.max_joint_step = std::stod(next());
      } else if (arg == "--max-joint-delta") {
        a.max_joint_delta = std::stod(next());
      } else {
        throw std::runtime_error("unknown arg: " + arg);
      }
    } catch (const std::exception& e) {
      std::cerr << "Argument error: " << e.what() << "\n";
      print_usage(argv[0]);
      return std::nullopt;
    }
  }

  // Convenience: if the server is bound to loopback only, allow shutdown requests by default.
  // This provides an operator-friendly stop path without exposing shutdown on the network.
  if (!a.allow_shutdown && (a.bind_host == "127.0.0.1" || a.bind_host == "localhost")) {
    a.allow_shutdown = true;
  }

  return a;
}

bool recv_all(int fd, void* buf, size_t len) {
  auto* p = static_cast<std::uint8_t*>(buf);
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::recv(fd, p + off, len - off, 0);
    if (n == 0) return false;  // disconnected
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Timeout (if SO_RCVTIMEO is configured). Allow graceful shutdown.
        if (g_shutdown.load()) return false;
        continue;
      }
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

bool send_all(int fd, const void* buf, size_t len) {
  const auto* p = static_cast<const std::uint8_t*>(buf);
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::send(fd, p + off, len - off, MSG_NOSIGNAL);
    if (n <= 0) {
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // Timeout (if SO_SNDTIMEO is configured). Allow graceful shutdown.
        if (g_shutdown.load()) return false;
        continue;
      }
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

int create_listen_socket_v4(const std::string& host, int port) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    throw std::runtime_error("socket() failed");
  }
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (host == "0.0.0.0" || host == "" || host == "*") {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else {
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
      ::close(fd);
      throw std::runtime_error("inet_pton failed for host=" + host);
    }
  }

  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    throw std::runtime_error("bind() failed for " + host + ":" + std::to_string(port));
  }
  if (::listen(fd, 1) != 0) {
    ::close(fd);
    throw std::runtime_error("listen() failed");
  }
  return fd;
}

bool poll_readable(int fd, int timeout_ms) {
  pollfd pfd{};
  pfd.fd = fd;
  pfd.events = POLLIN;
  while (true) {
    const int r = ::poll(&pfd, 1, timeout_ms);
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    return r > 0;
  }
}

void set_sock_timeouts_ms(int fd, int timeout_ms) {
  if (timeout_ms <= 0) return;
  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

struct LatestCommand {
  CommandMsgV1 cmd{};
  std::uint64_t recv_time_ns = 0;
  bool valid = false;
};

template <typename T>
class DoubleBuffer {
 public:
  void write(const T& v) {
    const std::uint8_t cur = idx_.load(std::memory_order_relaxed);
    const std::uint8_t next = static_cast<std::uint8_t>(cur ^ 1u);
    buf_[next] = v;
    idx_.store(next, std::memory_order_release);
  }

  T read() const {
    const std::uint8_t cur = idx_.load(std::memory_order_acquire);
    return buf_[cur];
  }

 private:
  std::atomic<std::uint8_t> idx_{0};
  std::array<T, 2> buf_{};
};

struct LatestState {
  StateMsgV1 msg{};
};

struct Stats {
  std::atomic<std::uint64_t> cmd_rx{0};
  std::atomic<std::uint64_t> cmd_bad_header{0};
  std::atomic<std::uint64_t> cmd_shutdown_requests{0};

  std::atomic<std::uint64_t> control_cycles{0};
  std::atomic<std::uint64_t> arm_enabled_cycles{0};
  std::atomic<std::uint64_t> hold_cycles{0};
  std::atomic<std::uint64_t> cmds_applied{0};

  // libfranka reports this value in [0, 1]. We store it scaled as ppm.
  std::atomic<std::uint32_t> last_control_success_ppm{0};
};

bool is_cmd_header_valid(const CommandMsgV1& c) {
  return (c.magic == franka_station_protocol::kCmdMagic) && (c.version == franka_station_protocol::kVersion);
}

void init_state_header(StateMsgV1* s) {
  s->magic = franka_station_protocol::kStateMagic;
  s->version = franka_station_protocol::kVersion;
}

double clamp(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

[[maybe_unused]] std::uint32_t clamp_to_ppm(double x) {
  if (!std::isfinite(x)) return 0u;
  x = clamp(x, 0.0, 1.0);
  const double ppm = x * 1e6;
  if (ppm <= 0.0) return 0u;
  if (ppm >= 1e6) return 1000000u;
  return static_cast<std::uint32_t>(ppm + 0.5);
}

void clamp_joint_limits(double q[7]) {
  // FR3 joint position limits (necessary conditions).
  //
  // Source: Franka docs "Robot and interface specifications" → "Limits for Franka Research 3":
  // https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3
  //
  // Note: Joint 6 has a *positive* minimum on FR3.
  static constexpr double kLo[7] = {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, +0.5445, -3.0159};
  static constexpr double kHi[7] = {+2.7437, +1.7837, +2.9007, -0.1518, +2.8065, +4.5169, +3.0159};
  for (int i = 0; i < 7; ++i) {
    q[i] = clamp(q[i], kLo[i], kHi[i]);
  }
}

void clamp_droid_limits(double q[7]) {
  static constexpr double kLo[7] = {-0.828, -0.840, -0.843, -2.773, -1.843, +1.172, -2.047};
  static constexpr double kHi[7] = {+0.900, +1.385, +0.692, -0.454, +1.732, +3.467, +2.198};
  for (int i = 0; i < 7; ++i) {
    q[i] = clamp(q[i], kLo[i], kHi[i]);
  }
}

void command_server_thread(const Args& args, int listen_fd, DoubleBuffer<LatestCommand>* shared_cmd, Stats* stats) {
  configure_network_thread("cmd");
  if (listen_fd < 0) {
    std::cerr << "[cmd] failed to start: invalid listen fd\n";
    g_shutdown.store(true);
    return;
  }

  while (!g_shutdown.load()) {
    // Avoid blocking forever in accept() so SIGINT/SIGTERM can shut us down cleanly.
    if (!poll_readable(listen_fd, 200)) {
      continue;
    }
    sockaddr_in cli{};
    socklen_t slen = sizeof(cli);
    const int client_fd = ::accept(listen_fd, reinterpret_cast<sockaddr*>(&cli), &slen);
    if (client_fd < 0) {
      if (errno == EINTR) continue;
      continue;
    }
    set_sock_timeouts_ms(client_fd, 200);
    std::cerr << "[cmd] client connected\n";

    while (!g_shutdown.load()) {
      CommandMsgV1 c{};
      if (!recv_all(client_fd, &c, sizeof(c))) {
        break;
      }
      if (!is_cmd_header_valid(c)) {
        if (stats) stats->cmd_bad_header.fetch_add(1, std::memory_order_relaxed);
        continue;
      }
      if (args.allow_shutdown && (c.cmd_type == static_cast<std::uint32_t>(CmdType::kShutdown))) {
        std::cerr << "[cmd] shutdown requested\n";
        if (stats) stats->cmd_shutdown_requests.fetch_add(1, std::memory_order_relaxed);
        g_shutdown.store(true);
        break;
      }
      LatestCommand lc{};
      lc.cmd = c;
      lc.recv_time_ns = monotonic_ns();
      lc.valid = true;
      shared_cmd->write(lc);
      if (stats) stats->cmd_rx.fetch_add(1, std::memory_order_relaxed);
    }

    ::close(client_fd);
    std::cerr << "[cmd] client disconnected\n";
  }

  ::close(listen_fd);
}

void state_server_thread(const Args& args, int listen_fd, const DoubleBuffer<LatestState>* shared_state) {
  configure_network_thread("state");
  if (listen_fd < 0) {
    std::cerr << "[state] failed to start: invalid listen fd\n";
    g_shutdown.store(true);
    return;
  }

  const auto period = std::chrono::duration<double>(1.0 / std::max(1, args.state_hz));

  while (!g_shutdown.load()) {
    // Avoid blocking forever in accept() so SIGINT/SIGTERM can shut us down cleanly.
    if (!poll_readable(listen_fd, 200)) {
      continue;
    }
    sockaddr_in cli{};
    socklen_t slen = sizeof(cli);
    const int client_fd = ::accept(listen_fd, reinterpret_cast<sockaddr*>(&cli), &slen);
    if (client_fd < 0) {
      if (errno == EINTR) continue;
      continue;
    }
    set_sock_timeouts_ms(client_fd, 200);
    std::cerr << "[state] client connected\n";

    auto next = std::chrono::steady_clock::now();
    while (!g_shutdown.load()) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      const LatestState st = shared_state->read();
      if (!send_all(client_fd, &st.msg, sizeof(st.msg))) {
        break;
      }
      std::this_thread::sleep_until(next);
    }

    ::close(client_fd);
    std::cerr << "[state] client disconnected\n";
  }

  ::close(listen_fd);
}

void stats_printer_thread(const DoubleBuffer<LatestState>* shared_state, Stats* stats) {
  configure_network_thread("stats");
  auto last_t = std::chrono::steady_clock::now();
  while (!g_shutdown.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto now_t = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now_t - last_t).count();
    last_t = now_t;

    const std::uint64_t cycles = stats ? stats->control_cycles.exchange(0, std::memory_order_relaxed) : 0;
    const std::uint64_t hold = stats ? stats->hold_cycles.exchange(0, std::memory_order_relaxed) : 0;
    const std::uint64_t enabled = stats ? stats->arm_enabled_cycles.exchange(0, std::memory_order_relaxed) : 0;
    const std::uint64_t applied = stats ? stats->cmds_applied.exchange(0, std::memory_order_relaxed) : 0;
    const std::uint64_t cmd_rx = stats ? stats->cmd_rx.exchange(0, std::memory_order_relaxed) : 0;
    const std::uint64_t bad = stats ? stats->cmd_bad_header.exchange(0, std::memory_order_relaxed) : 0;

    const LatestState st = shared_state->read();

    const double ctrl_hz = (dt > 0.0) ? static_cast<double>(cycles) / dt : 0.0;
    const double cmd_hz = (dt > 0.0) ? static_cast<double>(cmd_rx) / dt : 0.0;

    double hold_ratio = 0.0;
    double enabled_ratio = 0.0;
    if (cycles > 0) {
      hold_ratio = clamp(static_cast<double>(hold) / static_cast<double>(cycles), 0.0, 1.0);
      enabled_ratio = clamp(static_cast<double>(enabled) / static_cast<double>(cycles), 0.0, 1.0);
    }

    double succ = 0.0;
    if (stats) {
      succ = static_cast<double>(stats->last_control_success_ppm.load(std::memory_order_relaxed)) / 1e6;
      succ = clamp(succ, 0.0, 1.0);
    }

    std::cerr << "[stats] ctrl_hz=" << ctrl_hz << " cmd_rx_hz=" << cmd_hz << " enabled=" << (100.0 * enabled_ratio)
              << "% hold=" << (100.0 * hold_ratio) << "% applied=" << applied << " cmd_age_ms=" << st.msg.cmd_age_ms
              << " last_seq=" << st.msg.last_cmd_seq << " bad_hdr=" << bad << " ctrl_success=" << succ << "\n";
  }
}

void run_mock_control_loop(const Args& args,
                           const DoubleBuffer<LatestCommand>* shared_cmd,
                           DoubleBuffer<LatestState>* shared_state,
                           Stats* stats) {
  configure_control_thread("mock-ctrl");
  LatestState st{};
  init_state_header(&st.msg);

  st.msg.robot_mode = 3;  // mock
  st.msg.error_code = 0;
  st.msg.last_cmd_seq = 0;
  st.msg.gripper = 0.0;
  st.msg.cmd_age_ms = 1e9;
  for (int i = 0; i < 7; ++i) {
    st.msg.q[i] = 0.0;
    st.msg.dq[i] = 0.0;
  }

  std::array<double, 7> q_cmd{};
  std::array<double, 7> q_prev{};
  std::array<double, 7> q_setpoint{};
  double gripper_sim = 0.0;  // mock gripper width (m)
  std::uint64_t last_applied_seq = 0;
  bool hold_active = true;

  const auto dt = std::chrono::duration<double>(0.001);
  const double alpha = clamp(dt.count() / std::max(1e-6, args.filter_tau_s), 0.0, 1.0);

  auto next = std::chrono::steady_clock::now();
  while (!g_shutdown.load()) {
    next += std::chrono::milliseconds(1);

    if (stats) {
      stats->control_cycles.fetch_add(1, std::memory_order_relaxed);
      stats->last_control_success_ppm.store(1000000u, std::memory_order_relaxed);
    }

    const auto now_ns = monotonic_ns();
    const LatestCommand lc = shared_cmd->read();

    bool enable_arm = false;
    bool enable_gripper = false;
    CmdMode mode = CmdMode::kHold;
    bool fresh = false;
    CommandMsgV1 c{};
    std::uint64_t cmd_recv_ns = 0;
    if (lc.valid) {
      c = lc.cmd;
      cmd_recv_ns = lc.recv_time_ns;
      const double age_ms = static_cast<double>(now_ns - cmd_recv_ns) / 1e6;
      fresh = (age_ms <= static_cast<double>(args.watchdog_ms)) &&
              (age_ms <= static_cast<double>(c.validity_ms));
      enable_arm = fresh && (c.cmd_type == static_cast<std::uint32_t>(CmdType::kArmCommand)) &&
                   ((c.flags & Flags::kEnableArm) != 0u);
      enable_gripper = fresh && (c.cmd_type == static_cast<std::uint32_t>(CmdType::kArmCommand)) &&
                       ((c.flags & Flags::kEnableGripper) != 0u);
      mode = static_cast<CmdMode>(c.mode);
      st.msg.last_cmd_seq = c.seq;
      st.msg.cmd_age_ms = age_ms;
    } else {
      st.msg.cmd_age_ms = 1e9;
    }

    const bool request_hold = (mode == CmdMode::kHold);
    const bool allow_motion = enable_arm && !request_hold;
    if (stats) {
      if (allow_motion) {
        stats->arm_enabled_cycles.fetch_add(1, std::memory_order_relaxed);
      } else {
        stats->hold_cycles.fetch_add(1, std::memory_order_relaxed);
      }
    }

    // Determine target
    if (!allow_motion) {
      // Hold current command, but set the hold target only once to allow the rate limiter
      // to decelerate smoothly if we were moving.
      if (!hold_active) {
        hold_active = true;
        q_setpoint = q_cmd;
      }
    } else if (c.seq != last_applied_seq) {
      hold_active = false;
      last_applied_seq = c.seq;
      if (mode == CmdMode::kJointPosition) {
        for (int i = 0; i < 7; ++i) {
          q_setpoint[i] = clamp(c.q[i], q_setpoint[i] - args.max_joint_step, q_setpoint[i] + args.max_joint_step);
        }
      } else if (mode == CmdMode::kJointDelta) {
        for (int i = 0; i < 7; ++i) {
          const double step = clamp(c.q[i], -args.max_joint_step, args.max_joint_step);
          q_setpoint[i] = q_setpoint[i] + step;
        }
      } else if (mode == CmdMode::kJointVelocity) {
        for (int i = 0; i < 7; ++i) {
          const double vel = clamp(c.q[i], -1.0, 1.0);
          q_setpoint[i] = q_setpoint[i] + vel * args.max_joint_delta;
        }
      }
      double tmp[7];
      for (int i = 0; i < 7; ++i) tmp[i] = q_setpoint[i];
      clamp_joint_limits(tmp);
      clamp_droid_limits(tmp);
      for (int i = 0; i < 7; ++i) q_setpoint[i] = tmp[i];
      if (stats) stats->cmds_applied.fetch_add(1, std::memory_order_relaxed);
    }
    const std::array<double, 7> q_target = q_setpoint;

    // Mock gripper: simulate instantaneous move to commanded width
    if (enable_gripper && lc.valid) {
      double g_target = clamp(c.gripper, 0.0, 0.08);
      gripper_sim = g_target;
    }

    // First-order smoothing
    for (int i = 0; i < 7; ++i) {
      q_cmd[i] = q_cmd[i] + alpha * (q_target[i] - q_cmd[i]);
    }

    // dq estimate
    for (int i = 0; i < 7; ++i) {
      st.msg.dq[i] = (q_cmd[i] - q_prev[i]) / dt.count();
      q_prev[i] = q_cmd[i];
      st.msg.q[i] = q_cmd[i];
    }

    st.msg.gripper = gripper_sim;
    st.msg.robot_time_ns = now_ns;
    shared_state->write(st);

    std::this_thread::sleep_until(next);
  }
}

#if ROBOT_HAS_FRANKA
void run_franka_control_loop(const Args& args,
                             const DoubleBuffer<LatestCommand>* shared_cmd,
                             DoubleBuffer<LatestState>* shared_state,
                             Stats* stats) {
  LatestState st{};
  init_state_header(&st.msg);
  st.msg.robot_mode = 0;  // connecting
  st.msg.error_code = 0;
  st.msg.last_cmd_seq = 0;
  st.msg.gripper = 0.0;
  st.msg.cmd_age_ms = 1e9;
  for (int i = 0; i < 7; ++i) {
    st.msg.q[i] = 0.0;
    st.msg.dq[i] = 0.0;
  }
  st.msg.robot_time_ns = monotonic_ns();
  shared_state->write(st);

  // Outer loop: (re-)connect to the robot.
  while (!g_shutdown.load()) {
    std::unique_ptr<franka::Robot> robot;
    try {
      std::cerr << "[franka] connecting to " << args.franka_ip << " ...\n";
      const auto rt_cfg = args.realtime_ignore ? franka::RealtimeConfig::kIgnore : franka::RealtimeConfig::kEnforce;
      robot = std::make_unique<franka::Robot>(args.franka_ip, rt_cfg);
    } catch (const franka::Exception& e) {
      std::cerr << "[franka] connect failed: " << e.what() << "\n";
      std::cerr << "[franka] check: (1) open Franka Desk and start/activate FCI, (2) no other FCI client connected, (3) IP is correct.\n";
      st.msg.robot_mode = 2;
      st.msg.error_code = 1;
      st.msg.robot_time_ns = monotonic_ns();
      shared_state->write(st);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    // Apply RT thread policy once per connection (not per control restart).
    configure_control_thread("ctrl");

    // Collision detection: sensitive enough to stop on contact with table/objects,
    // but with enough margin to avoid false triggers from normal motion dynamics.
    // "lower" = detection threshold, "upper" = reflex (hard stop) threshold.
    try {
      robot->setCollisionBehavior(
          {{30.0, 30.0, 28.0, 28.0, 26.0, 24.0, 22.0}},  // lower_torque_thresholds_acceleration
          {{60.0, 60.0, 58.0, 58.0, 56.0, 54.0, 50.0}},  // upper_torque_thresholds_acceleration (raised for grasping)
          {{28.0, 28.0, 26.0, 26.0, 24.0, 22.0, 20.0}},  // lower_torque_thresholds_nominal
          {{60.0, 60.0, 58.0, 58.0, 56.0, 54.0, 50.0}},  // upper_torque_thresholds_nominal (raised for grasping)
          {{30.0, 30.0, 30.0, 35.0, 35.0, 35.0}},          // lower_force_thresholds_acceleration
          {{60.0, 60.0, 60.0, 65.0, 65.0, 65.0}},          // upper_force_thresholds_acceleration (raised for grasping)
          {{25.0, 25.0, 25.0, 30.0, 30.0, 30.0}},          // lower_force_thresholds_nominal
          {{55.0, 55.0, 55.0, 60.0, 60.0, 60.0}}           // upper_force_thresholds_nominal (raised for grasping)
      );
      robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
      robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
      std::cerr << "[franka] collision/impedance configured\n";
    } catch (const franka::Exception& e) {
      std::cerr << "[franka] WARNING: setCollisionBehavior failed: " << e.what() << "\n";
    }

    // Persistent state across control restarts -- survives reflex recovery.
    // q_setpoint and q_hold are re-initialized from the robot's actual state
    // inside the first callback tick, but we keep them here so they carry over
    // between successful control sessions.
    std::array<double, 7> q_setpoint{};
    std::array<double, 7> q_hold{};
    std::uint64_t last_applied_seq = 0;

    // Inner loop: recover + restart control() on the SAME connection.
    // This avoids the reconnection overhead which causes trajectory discontinuities.
    while (!g_shutdown.load()) {
      // Read current state for initialization.
      franka::RobotState initial_state{};
      try {
        initial_state = robot->readOnce();
      } catch (const franka::Exception& e) {
        std::cerr << "[franka] readOnce failed: " << e.what() << " (will reconnect)\n";
        break;  // -> outer loop reconnects
      }

      // Re-initialize from the robot's actual position after each restart.
      q_setpoint = initial_state.q;
      q_hold = initial_state.q;
      bool hold_active = true;
      bool finishing = false;
      bool initialized = false;
      int warmup_steps_remaining = 200;

      // Second-order interpolation state: position + velocity.
      // This generates smooth trajectories with bounded acceleration,
      // avoiding the velocity discontinuities that cause reflexes.
      std::array<double, 7> q_cmd_out = initial_state.q_d;
      std::array<double, 7> v_cmd = {};  // current velocity (rad/s), starts at zero
      static constexpr double kMaxVel = 0.8;   // rad/s max velocity (conservative to avoid reflex)
      static constexpr double kMaxAcc = 1.5;   // rad/s^2 max acceleration (conservative to avoid reflex)

      // Publish state so the station sees a healthy robot immediately.
      st.msg.robot_time_ns = monotonic_ns();
      st.msg.robot_mode = 1;
      st.msg.error_code = 0;
      for (int i = 0; i < 7; ++i) {
        st.msg.q[i] = initial_state.q[i];
        st.msg.dq[i] = initial_state.dq[i];
      }
      shared_state->write(st);

      try {
        robot->control([&](const franka::RobotState& rs, franka::Duration period) -> franka::JointPositions {
          const auto now_ns = monotonic_ns();

          if (stats) {
            stats->control_cycles.fetch_add(1, std::memory_order_relaxed);
            stats->last_control_success_ppm.store(clamp_to_ppm(rs.control_command_success_rate), std::memory_order_relaxed);
          }

          // Initialize from the robot's desired trajectory state (first tick only).
          if (!initialized) {
            q_setpoint = rs.q_d;
            q_hold = rs.q_d;
            q_cmd_out = rs.q_d;
            v_cmd = {};  // zero velocity on init
            hold_active = true;
            initialized = true;
            // Ignore any stale commands from before this control session.
            const LatestCommand init_lc = shared_cmd->read();
            last_applied_seq = init_lc.valid ? init_lc.cmd.seq : 0;
          }

          const LatestCommand lc = shared_cmd->read();
          bool enable_arm = false;
          CommandMsgV1 c{};
          CmdMode mode = CmdMode::kHold;
          if (lc.valid) {
            c = lc.cmd;
            const double age_ms = static_cast<double>(now_ns - lc.recv_time_ns) / 1e6;
            const bool fresh = (age_ms <= static_cast<double>(args.watchdog_ms)) &&
                                (age_ms <= static_cast<double>(c.validity_ms));
            enable_arm = fresh && (c.cmd_type == static_cast<std::uint32_t>(CmdType::kArmCommand)) &&
                          ((c.flags & Flags::kEnableArm) != 0u);
            mode = static_cast<CmdMode>(c.mode);
            st.msg.last_cmd_seq = c.seq;
            st.msg.cmd_age_ms = age_ms;
          } else {
            st.msg.cmd_age_ms = 1e9;
          }

          if (g_shutdown.load()) {
            finishing = true;
            enable_arm = false;
          }

          if (warmup_steps_remaining > 0) {
            enable_arm = false;
            --warmup_steps_remaining;
          }

          const bool request_hold = (mode == CmdMode::kHold);
          const bool allow_motion = enable_arm && !request_hold;
          if (stats) {
            if (allow_motion) {
              stats->arm_enabled_cycles.fetch_add(1, std::memory_order_relaxed);
            } else {
              stats->hold_cycles.fetch_add(1, std::memory_order_relaxed);
            }
          }

          if (!allow_motion) {
            if (!hold_active) {
              hold_active = true;
              q_hold = rs.q_d;
            }
          } else {
            if (hold_active) {
              hold_active = false;
              q_setpoint = q_hold;
            }

            if (c.seq != last_applied_seq) {
              last_applied_seq = c.seq;
              if (mode == CmdMode::kJointPosition) {
                for (int i = 0; i < 7; ++i) {
                  q_setpoint[i] = clamp(c.q[i], q_setpoint[i] - args.max_joint_step, q_setpoint[i] + args.max_joint_step);
                }
              } else if (mode == CmdMode::kJointDelta) {
                for (int i = 0; i < 7; ++i) {
                  const double step = clamp(c.q[i], -args.max_joint_step, args.max_joint_step);
                  q_setpoint[i] = q_setpoint[i] + step;
                }
              } else if (mode == CmdMode::kJointVelocity) {
                for (int i = 0; i < 7; ++i) {
                  const double vel = clamp(c.q[i], -1.0, 1.0);
                  q_setpoint[i] = q_setpoint[i] + vel * args.max_joint_delta;
                }
              }
              double tmp[7];
              for (int i = 0; i < 7; ++i) tmp[i] = q_setpoint[i];
              clamp_joint_limits(tmp);
              clamp_droid_limits(tmp);
              for (int i = 0; i < 7; ++i) q_setpoint[i] = tmp[i];
              if (stats) stats->cmds_applied.fetch_add(1, std::memory_order_relaxed);
            }
          }

          // Second-order interpolation: accelerate/decelerate toward goal.
          // This produces smooth velocity profiles that don't trigger reflexes.
          const std::array<double, 7> q_goal = hold_active ? q_hold : q_setpoint;
          const double dt_s = std::max(period.toSec(), 0.0005);
          for (int i = 0; i < 7; ++i) {
            const double err = q_goal[i] - q_cmd_out[i];
            // Compute stopping distance at current velocity
            const double stop_dist = (v_cmd[i] * v_cmd[i]) / (2.0 * kMaxAcc);
            // Desired acceleration: accelerate toward goal, but decelerate if close
            double a_des;
            if (std::abs(err) > 1e-6) {
              const double sign_err = (err > 0.0) ? 1.0 : -1.0;
              // If we need to decelerate (moving toward goal but would overshoot)
              if (v_cmd[i] * sign_err > 0.0 && stop_dist >= std::abs(err)) {
                a_des = -v_cmd[i] / std::max(dt_s, 1e-4);  // brake
                a_des = clamp(a_des, -kMaxAcc, kMaxAcc);
              } else {
                a_des = sign_err * kMaxAcc;
              }
            } else {
              a_des = -v_cmd[i] / std::max(dt_s, 1e-4);  // stop
              a_des = clamp(a_des, -kMaxAcc, kMaxAcc);
            }
            // Integrate velocity and position
            v_cmd[i] += a_des * dt_s;
            v_cmd[i] = clamp(v_cmd[i], -kMaxVel, kMaxVel);
            q_cmd_out[i] += v_cmd[i] * dt_s;
          }

          st.msg.robot_time_ns = now_ns;
          st.msg.robot_mode = 1;
          st.msg.error_code = 0;
          for (int i = 0; i < 7; ++i) {
            st.msg.q[i] = rs.q[i];
            st.msg.dq[i] = rs.dq[i];
          }
          st.msg.gripper = g_gripper_width.load(std::memory_order_relaxed);
          shared_state->write(st);

          if (finishing) {
            double max_abs_dq_des = 0.0;
            double max_abs_dq_meas = 0.0;
            for (int i = 0; i < 7; ++i) {
              max_abs_dq_des = std::max(max_abs_dq_des, std::abs(rs.dq_d[i]));
              max_abs_dq_meas = std::max(max_abs_dq_meas, std::abs(rs.dq[i]));
            }
            if (max_abs_dq_des < 0.02 && max_abs_dq_meas < 0.05) {
              franka::JointPositions cmd(q_cmd_out);
              cmd.motion_finished = true;
              return cmd;
            }
          }

          return franka::JointPositions(q_cmd_out);
        },
        franka::ControllerMode::kJointImpedance,
        /*limit_rate=*/false,
        /*cutoff_frequency=*/franka::kMaxCutoffFrequency);

        // control() returned normally (motion_finished). If shutting down, exit.
        if (g_shutdown.load()) break;

      } catch (const franka::Exception& e) {
        std::cerr << "[franka] control exception: " << e.what() << "\n";
        st.msg.robot_mode = 2;
        st.msg.error_code = 1;
        st.msg.robot_time_ns = monotonic_ns();
        shared_state->write(st);

        if (g_shutdown.load()) break;

        // Recover on the SAME connection -- no reconnect needed.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        try {
          std::cerr << "[franka] automaticErrorRecovery()...\n";
          robot->automaticErrorRecovery();
          std::cerr << "[franka] recovery OK, restarting control loop\n";
          // Inner loop continues -> readOnce + control() again on same connection.
        } catch (const franka::Exception& re) {
          std::cerr << "[franka] recovery failed: " << re.what() << " (will reconnect)\n";
          break;  // -> outer loop reconnects
        }
      }
    }  // inner loop
  }  // outer loop
}
#endif

#if ROBOT_HAS_FRANKA
void gripper_thread_func(const std::string& ip, const DoubleBuffer<LatestCommand>* shared_cmd, DoubleBuffer<LatestState>* /*shared_state*/) {
  configure_network_thread("gripper");
  try {
    franka::Gripper gripper(ip);
    
    // Best-effort homing (required after power-cycle, and improves repeatability).
    try {
      std::cerr << "[gripper] homing...\n";
      gripper.homing();
      std::cerr << "[gripper] homing done\n";
    } catch (const std::exception& e) {
      std::cerr << "[gripper] homing failed (continuing): " << e.what() << "\n";
    }

    // Initial state
    try {
      auto state = gripper.readOnce();
      g_gripper_width.store(state.width, std::memory_order_relaxed);
    } catch (...) {}

    double last_command_width = -1.0;
    std::uint64_t last_seq = 0;
    bool grasp_latched = false;

    while (!g_shutdown.load()) {
      // 1. Read Command
      LatestCommand lc = shared_cmd->read();
      
      // 2. Execute Move if commanded
      if (lc.valid && lc.cmd.seq != last_seq) {
        last_seq = lc.cmd.seq;
        bool enable = (lc.cmd.flags & Flags::kEnableGripper) != 0;
        
        if (enable) {
           double target = lc.cmd.gripper;
           try {
             // Clamp
             if (target > 0.08) target = 0.08;
             if (target < 0.0) target = 0.0;

             constexpr double kSpeed = 0.10;   // m/s
             constexpr double kForce = 60.0;   // N
             constexpr double kEpsInner = 0.005;   // m
             constexpr double kEpsOuter = 0.040;   // m (unknown object size tolerance)
             constexpr double kCloseThreshold = 0.055;      // m
             constexpr double kOpenReleaseThreshold = 0.072;  // m

             // Use a close/open hysteresis and sticky latch:
             // once we commit to grasp, keep holding until a clear open command arrives.
             double current_width = g_gripper_width.load(std::memory_order_relaxed);
             try {
               current_width = gripper.readOnce().width;
             } catch (...) {}
             const bool wants_open = target >= kOpenReleaseThreshold;
             const bool wants_close_cmd = target <= kCloseThreshold;
             const bool wants_close_motion = target < (current_width - 0.001);
             const bool wants_close = wants_close_cmd || wants_close_motion;

             const bool next_latch = (!wants_open) && (grasp_latched || wants_close);
             const bool latch_transition = (next_latch != grasp_latched);
             const bool target_changed = std::abs(target - last_command_width) > 0.002;

             if (target_changed || latch_transition) {
               if (wants_open) {
                 grasp_latched = false;
                 gripper.move(target, kSpeed);
               } else if (next_latch) {
                 // Close fully and hold with force; this is robust to command jitter.
                 grasp_latched = true;
                 (void)gripper.grasp(0.0, kSpeed, kForce, kEpsInner, kEpsOuter);
               } else {
                 grasp_latched = false;
                 gripper.move(target, kSpeed);
               }
               last_command_width = target;
             }

             auto state = gripper.readOnce();
             g_gripper_width.store(state.width, std::memory_order_relaxed);
           } catch (const std::exception& e) {
             std::cerr << "[gripper] move failed: " << e.what() << "\n";
           }
        } else {
          // New command stream can re-arm latch; disabled stream should not retain it.
          grasp_latched = false;
        }
      }

      // 3. Periodic Status Update
      static int tick = 0;
      if (tick++ % 10 == 0) {
        try {
           auto state = gripper.readOnce();
           g_gripper_width.store(state.width, std::memory_order_relaxed);
        } catch (...) {}
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } catch (const std::exception& e) {
    std::cerr << "[gripper] thread init failed: " << e.what() << "\n";
  }
}
#endif

}  // namespace

int main(int argc, char** argv) {
  // Avoid SIGPIPE killing the process if a client disconnects.
  ::signal(SIGPIPE, SIG_IGN);
  ::signal(SIGINT, on_sigint);
  ::signal(SIGTERM, on_sigint);

  const auto args_opt = parse_args(argc, argv);
  if (!args_opt.has_value()) {
    return 2;
  }
  const Args args = *args_opt;

  // Realtime process setup: avoid paging during the 1 kHz loop.
  if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::cerr << "[rt] mlockall(MCL_CURRENT|MCL_FUTURE) failed: " << std::strerror(errno) << "\n";
  }
  prefault_stack();

  if (!args.mock) {
#if ROBOT_HAS_FRANKA
    if (args.franka_ip.empty()) {
      std::cerr << "Error: --franka-ip is required unless --mock is set\n";
      return 2;
    }
#else
    std::cerr << "Error: built without libfranka; run with --mock or rebuild on robot PC with libfranka installed\n";
    return 2;
#endif
  }

  DoubleBuffer<LatestCommand> shared_cmd;
  DoubleBuffer<LatestState> shared_state;
  Stats stats;
  LatestState init{};
  init_state_header(&init.msg);
  init.msg.robot_time_ns = monotonic_ns();
  init.msg.last_cmd_seq = 0;
  init.msg.robot_mode = args.mock ? 3 : 0;
  init.msg.error_code = 0;
  init.msg.gripper = 0.0;
  init.msg.cmd_age_ms = 1e9;
  for (int i = 0; i < 7; ++i) {
    init.msg.q[i] = 0.0;
    init.msg.dq[i] = 0.0;
  }
  shared_state.write(init);

  int cmd_listen_fd = -1;
  int state_listen_fd = -1;
  try {
    cmd_listen_fd = create_listen_socket_v4(args.bind_host, args.bind_port);
    state_listen_fd = create_listen_socket_v4(args.state_host, args.state_port);
  } catch (const std::exception& e) {
    std::cerr << "[main] failed to bind command/state ports: " << e.what() << "\n";
    if (cmd_listen_fd >= 0) ::close(cmd_listen_fd);
    if (state_listen_fd >= 0) ::close(state_listen_fd);
    return 1;
  }

  std::cerr << "[cmd] listening on " << args.bind_host << ":" << args.bind_port << "\n";
  std::cerr << "[state] listening on " << args.state_host << ":" << args.state_port << "\n";

  std::thread cmd_thr(command_server_thread, std::cref(args), cmd_listen_fd, &shared_cmd, &stats);
  std::thread state_thr(state_server_thread, std::cref(args), state_listen_fd, &shared_state);
  std::thread stats_thr(stats_printer_thread, &shared_state, &stats);

#if ROBOT_HAS_FRANKA
  std::thread gripper_thr;
  if (!args.mock) {
     gripper_thr = std::thread(gripper_thread_func, args.franka_ip, &shared_cmd, &shared_state);
  }
#endif

  if (args.mock) {
    std::cerr << "[main] starting mock control loop\n";
    run_mock_control_loop(args, &shared_cmd, &shared_state, &stats);
  } else {
#if ROBOT_HAS_FRANKA
    std::cerr << "[main] starting libfranka control loop\n";
    run_franka_control_loop(args, &shared_cmd, &shared_state, &stats);
#endif
  }

  g_shutdown.store(true);
  if (cmd_thr.joinable()) cmd_thr.join();
  if (state_thr.joinable()) state_thr.join();
  if (stats_thr.joinable()) stats_thr.join();
#if ROBOT_HAS_FRANKA
  if (gripper_thr.joinable()) gripper_thr.join();
#endif
  std::cerr << "[main] shutdown\n";
  return 0;
}

