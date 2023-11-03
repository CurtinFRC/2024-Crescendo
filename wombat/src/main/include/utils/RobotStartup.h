#pragma once 

#include <frc/RobotBase.h>
#include <iostream>
#include <functional>
namespace wom {
namespace utils {

class RobotStartup {
 public: 
  static void Start(std::function<int()> func);
};

template <class RobotClass>
int StartRobot() {
  RobotStartup::Start(frc::StartRobot<RobotClass>);
  return 0;
}

#ifndef RUNNING_FRC_TESTS
#define WOMBAT_ROBOT_MAIN(RobotClz) int main() { wom::utils::StartRobot<RobotClz>(); }
#else 
#define WOMBAT_ROBOT_MAIN(RobotClz)
#endif

} // ns utils
} // ns wom
