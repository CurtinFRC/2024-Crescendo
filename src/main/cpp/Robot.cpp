// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"

// include units
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/RamseteController.h>
#include <frc/Timer.h>

static units::second_t lastPeriodic;

void Robot::RobotInit() {

  shooter = new Shooter(robotmap.shooterSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(shooter);
  shooter->SetDefaultBehaviour(
     [this]() {return wom::make<ShooterManualControl>(shooter, &robotmap.controllers.codriver); });
  
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();
  shooter->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  loop.Clear();
}
void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler *sched = wom::BehaviourScheduler::GetInstance();
  sched->InterruptAll();
}
void Robot::TeleopPeriodic() {
  
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}