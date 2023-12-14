// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"
#include <ctre/Phoenix.h>

void Robot::RobotInit() {
  /* wom::BehaviourScheduler::GetInstance()->Register(&robotmap.swerve); */
  /* robotmap.swerve.SetDefaultBehaviour([this]() { */
  /*   return wom::make<wom::FieldRelativeSwerveDrive>(&robotmap.swerve, robotmap.controllers.driver); */
  /* }); */
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  /* sched = wom::BehaviourScheduler::GetInstance(); */
  /* sched->InterruptAll(); */
  /* sched->Schedule(wom::make<wom::FieldRelativeSwerveDrive>(&robotmap.swerve, robotmap.controllers.driver)); */
}
void Robot::TeleopPeriodic() {
  motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
