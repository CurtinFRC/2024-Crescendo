// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/RamseteController.h>
#include <frc/Timer.h>

#include <frc/TimedRobot.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

static units::second_t lastPeriodic;

void Robot::RobotInit() {

  lastPeriodic = wom::now();

  m_chooser.SetDefaultOption("Default Auto", "Default Auto");

  frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

  m_path_chooser.SetDefaultOption("Path1", "paths/output/Path1.wpilib.json");

  m_path_chooser.AddOption("Path1", "paths/output/Path1.wpilib.json");
  m_path_chooser.AddOption("Path2", "paths/output/Path2.wpilib.json");

  frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);

  frc::SmartDashboard::PutData("Field", &m_field);

  simulation_timer = frc::Timer();

  robotmap.swerveBase.gyro->Reset();

  _swerveDrive = new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d());
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  _swerveDrive->SetDefaultBehaviour(
      [this]() { return wom::make<wom::ManualDrivebase>(_swerveDrive, &robotmap.controllers.driver); });

  intake = new Intake(robotmap.intakeSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(intake);

  intake->SetDefaultBehaviour([this]() {
    return wom::make<IntakeManualControl>(intake, robotmap.controllers.driver);
  });

}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();

  _swerveDrive->OnUpdate(dt);
}

void Robot::AutonomousInit() {

  loop.Clear();
  sched->InterruptAll();
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler *scheduler = wom::BehaviourScheduler::GetInstance();
  scheduler->InterruptAll();
}

void Robot::TeleopPeriodic() {
  
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}
