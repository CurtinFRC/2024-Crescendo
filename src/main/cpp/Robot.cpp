// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();

  mag = new Mag(map.magSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(mag);
  mag->SetDefaultBehaviour([this]() {
    return wom::make<MagManualControl>(mag, map.codriver);
  });
}

void Robot::RobotPeriodic() {
  units::second_t dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler *scheduler = wom::BehaviourScheduler::GetInstance();
  scheduler->InterruptAll();
}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

