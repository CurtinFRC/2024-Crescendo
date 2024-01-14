// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include "RobotMap.h"
#include "behaviours/MagBehaviour.h"
#include <frc/TimedRobot.h>
#include <frc/event/Eventloop.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;//skibidi
  void TeleopPeriodic() override;//bum
  void DisabledInit() override;//poo
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::EventLoop loop;

  RobotMap map;
  Mag *mag;
};
