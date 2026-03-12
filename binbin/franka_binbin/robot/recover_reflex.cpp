#include <franka/robot.h>
#include <franka/exception.h>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <franka-ip>\n";
    return 1;
  }
  std::string ip = argv[1];
  try {
    std::cout << "Connecting to " << ip << " ...\n";
    franka::Robot robot(ip);
    
    std::cout << "Attempting automaticErrorRecovery()...\n";
    robot.automaticErrorRecovery();
    
    std::cout << "Success! Robot should be white (Idle).\n";
  } catch (franka::Exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
