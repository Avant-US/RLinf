#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <iostream>
#include <cmath>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-ip>\n";
    return -1;
  }

  try {
    franka::Robot robot(argv[1]);
    
    // Set collision behavior to be very forgiving
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::cout << "Starting joint velocity motion... (HOLDING ZERO)\n";
    
    // Defines a callback that returns 0 velocities (hold position)
    auto motion_callback = [](const franka::RobotState&, franka::Duration) -> franka::JointVelocities {
      return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    };

    // Run for 5 seconds
    double time = 0.0;
    robot.control([&time, motion_callback](const franka::RobotState& state, franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();
      if (time >= 5.0) {
        std::cout << "Finished 5s test.\n";
        return franka::MotionFinished(motion_callback(state, period));
      }
      return motion_callback(state, period);
    });

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
