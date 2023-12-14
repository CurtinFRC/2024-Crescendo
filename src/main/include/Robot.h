// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/TimedRobot.h>
#include <frc/event/EventLoop.h>

#include "RobotMap.h"
#include "Wombat.h"

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
  void SimulationInit() override;
  void SimulationPeriodic() override;

 protected:
 private:
  frc::EventLoop loop;
  behaviour::BehaviourScheduler *sched;
  RobotMap                       robotmap;

  wom::Limelight *limelight;
  wom::Swerve    *swerve;
};
