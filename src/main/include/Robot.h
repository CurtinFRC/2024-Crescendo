// of the MIT License at the root of this project

#pragma once

#include "Shooter.h"
#include <frc/TimedRobot.h>
#include "Wombat.h"
#include "RobotMap.h"
#include <frc/event/EventLoop.h>
#include "ShooterBehaviour.h"




class Robot : public frc::TimedRobot {
  public:
  void TestPeriodic() override;
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;


 private:
 frc::EventLoop loop;
 RobotMap map;
 Shooter *shooter;
};