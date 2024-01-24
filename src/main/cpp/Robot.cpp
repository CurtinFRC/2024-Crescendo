// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/controller/RamseteController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();

  _swerveDrive->OnUpdate(dt);
  mag->OnUpdate(dt);
  climber->OnUpdate(dt);
}

void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler* scheduler = wom::BehaviourScheduler::GetInstance();
  scheduler->InterruptAll();

  m_chooser.SetDefaultOption("Default Auto", "Default Auto");

  frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

  m_path_chooser.SetDefaultOption("Path1", "paths/output/Path1.wpilib.json");

  m_path_chooser.AddOption("Path1", "paths/output/Path1.wpilib.json");
  m_path_chooser.AddOption("Path2", "paths/output/Path2.wpilib.json");

  frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);

  frc::SmartDashboard::PutData("Field", &m_field);

  simulation_timer = frc::Timer();

  robotmap.swerveBase.gyro->Reset();

  lastPeriodic = wom::now();

  _swerveDrive = new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d());
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  _swerveDrive->SetDefaultBehaviour(
      [this]() { return wom::make<wom::ManualDrivebase>(_swerveDrive, &robotmap.controllers.driver); });

  climber = new Climber(robotmap.climberSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(climber);
  climber->SetDefaultBehaviour(
      [this]() { return wom::make<ClimberManualControl>(climber, &robotmap.controllers.codriver); });

  mag = new Mag(robotmap.magSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(mag);
  mag->SetDefaultBehaviour(
      [this]() { return wom::make<MagManualControl>(mag, &robotmap.controllers.codriver); });

  // m_driveSim = new wom::TempSimSwerveDrive(&simulation_timer, &m_field);
  // m_driveSim = wom::TempSimSwerveDrive();
}

void Robot::AutonomousInit() {
  loop.Clear();
  wom::BehaviourScheduler* scheduler = wom::BehaviourScheduler::GetInstance();
  scheduler->InterruptAll();
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
