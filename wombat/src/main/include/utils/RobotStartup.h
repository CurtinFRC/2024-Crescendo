// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/RobotBase.h>

#include <functional>
#include <iostream>

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
#define WOMBAT_ROBOT_MAIN(RobotClz)     \
  int main() {                          \
    wom::utils::StartRobot<RobotClz>(); \
  }
#else
#define WOMBAT_ROBOT_MAIN(RobotClz)
#endif

}  // namespace utils
}  // namespace wom
