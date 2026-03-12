#include <franka/robot.h>
#include <franka/exception.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>

static constexpr std::array<double, 7> kHome = {
    0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, -3.0 * M_PI_4};

static constexpr double kMaxVel = 0.5;   // rad/s per joint
static constexpr double kMaxAcc = 0.5;   // rad/s^2 per joint
static constexpr double kTolerance = 1e-3;

int main(int argc, char** argv) {
  std::string ip = "172.16.0.2";
  double speed_factor = 1.0;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--ip" && i + 1 < argc) {
      ip = argv[++i];
    } else if (arg == "--speed" && i + 1 < argc) {
      speed_factor = std::max(0.1, std::min(2.0, std::stod(argv[++i])));
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [--ip IP] [--speed 0.1..2.0]\n"
                << "  Moves the Franka arm to the candle home pose using libfranka directly.\n"
                << "  Default IP: 172.16.0.2, default speed: 1.0\n";
      return 0;
    } else {
      ip = arg;
    }
  }

  const double max_vel = kMaxVel * speed_factor;
  const double max_acc = kMaxAcc * speed_factor;

  std::cout << "[home] connecting to " << ip << " ...\n";
  try {
    franka::Robot robot(ip);

    std::cout << "[home] recovering errors (if any) ...\n";
    robot.automaticErrorRecovery();

    robot.setCollisionBehavior(
        {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
        {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
        {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
        {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
        {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
        {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
        {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
        {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}});

    // Read initial state.
    auto state = robot.readOnce();
    std::array<double, 7> q0 = state.q;

    double max_err = 0.0;
    for (int j = 0; j < 7; ++j) {
      max_err = std::max(max_err, std::abs(kHome[j] - q0[j]));
    }
    std::cout << "[home] current q: [";
    for (int j = 0; j < 7; ++j) {
      if (j) std::cout << ", ";
      std::cout << q0[j];
    }
    std::cout << "]\n";
    std::cout << "[home] max_err = " << max_err << " rad (" << (max_err * 180.0 / M_PI) << " deg)\n";

    if (max_err < kTolerance) {
      std::cout << "[home] already at home.\n";
      return 0;
    }

    // Trapezoidal velocity profile per joint, scaled so ALL joints finish at the same time.
    // For each joint: compute the time needed to travel its distance with (max_vel, max_acc).
    // The slowest joint sets the overall duration.  Then scale each joint's profile to match.

    std::array<double, 7> dist{};
    std::array<double, 7> sign{};
    double t_total = 0.0;

    for (int j = 0; j < 7; ++j) {
      dist[j] = std::abs(kHome[j] - q0[j]);
      sign[j] = (kHome[j] > q0[j]) ? 1.0 : -1.0;

      // Time for trapezoidal: if dist < v^2/a, it's a triangle (no cruise phase).
      double t;
      if (dist[j] < (max_vel * max_vel) / max_acc) {
        t = 2.0 * std::sqrt(dist[j] / max_acc);  // triangle
      } else {
        t = dist[j] / max_vel + max_vel / max_acc;  // trapezoid
      }
      t_total = std::max(t_total, t);
    }

    // Use S-curve (cosine) profile for smoothness: q(t) = q0 + dist * s(t),
    // where s(t) goes from 0 to 1 using a cosine blend.
    const double duration = t_total;

    std::cout << "[home] moving to home (duration=" << duration << "s, speed_factor=" << speed_factor << ")\n";
    std::cout << "[home] keep hand near E-STOP\n";

    double time = 0.0;
    bool done = false;

    robot.control([&](const franka::RobotState& /*rs*/,
                      franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      double s;
      if (time >= duration) {
        s = 1.0;
        done = true;
      } else {
        s = 0.5 * (1.0 - std::cos(M_PI * time / duration));
      }

      std::array<double, 7> q_cmd{};
      for (int j = 0; j < 7; ++j) {
        q_cmd[j] = q0[j] + sign[j] * dist[j] * s;
      }

      franka::JointPositions output(q_cmd);
      if (done) {
        return franka::MotionFinished(output);
      }
      return output;
    });

    std::cout << "[home] done. Arm is at home position.\n";
  } catch (const franka::Exception& e) {
    std::cerr << "[home] ERROR: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
