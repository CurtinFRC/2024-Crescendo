// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();
  limelight = new wom::Limelight("Limelight");
  swerve = new wom::Swerve(robotmap.swerveConfig, wom::SwerveState::kIdle, limelight);

  wom::BehaviourScheduler::GetInstance()->Register(swerve);
  swerve->SetDefaultBehaviour([this]() {
    return wom::make<wom::FieldRelativeSwerveDrive>(swerve, robotmap.controllers.driver);
  });
}
void Robot::RobotPeriodic() {
  units::second_t dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();
  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();

  swerve->OnUpdate(dt);
  limelight->OnUpdate(dt);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler *sched = wom::BehaviourScheduler::GetInstance();
  sched->InterruptAll(); // removes all previously scheduled behaviours
  
  swerve->OnStart();
}
void Robot::TeleopPeriodic() {
  sched = wom::BehaviourScheduler::GetInstance();
  sched->Schedule(wom::make<wom::FieldRelativeSwerveDrive>(swerve, robotmap.controllers.driver));
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
