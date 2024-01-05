// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include "tankBehaviour.h"
#include "armBehaviour.h"
#include "Wombat.h"
#include "RobotMap.h"
#include <frc/TimedRobot.h>
#include "RobotMap.h"
#include "Shooter.h"
#include "ShooterBehaviour.h"
#include <frc/event/EventLoop.h>

#include "Wombat.h"
#include "RobotMap.h"
#include <frc/event/EventLoop.h>
#include "intake.h"


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:

  frc::EventLoop loop;


  RobotMap map;
  Shooter *shooter;
  Intake *intake;
  TankDrive *tank;

};

