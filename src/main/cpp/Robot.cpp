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
#include "behaviour/HasBehaviour.h"
#include "vision/Limelight.h"

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  sched = wom::BehaviourScheduler::GetInstance();
  m_chooser.SetDefaultOption("Default Auto", "Default Auto");

  // frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

  // m_path_chooser.SetDefaultOption("Path1", "paths/output/Path1.wpilib.json");

  // m_path_chooser.AddOption("Path1", "paths/output/Path1.wpilib.json");
  // m_path_chooser.AddOption("Path2", "paths/output/Path2.wpilib.json");

  // shooter = new Shooter(robotmap.shooterSystem.config);
  // wom::BehaviourScheduler::GetInstance()->Register(shooter);
  // shooter->SetDefaultBehaviour(
  //     [this]() { return wom::make<ShooterManualControl>(shooter, &robotmap.controllers.codriver); });

  // frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);
  // frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);

  // frc::SmartDashboard::PutData("Field", &m_field);

  // simulation_timer = frc::Timer();

  // robotmap.swerveBase.gyro->Reset();

  _swerveDrive =
      new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d(), new wom::Limelight("limelight"));
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  _swerveDrive->SetDefaultBehaviour(
      [this]() { return wom::make<wom::ManualDrivebase>(_swerveDrive, &robotmap.controllers.driver); });

  // alphaArm = new AlphaArm(robotmap.alphaArmSystem.config);
  // wom::BehaviourScheduler::GetInstance()->Register(alphaArm);
  // alphaArm->SetDefaultBehaviour(
  //     [this]() { return wom::make<AlphaArmManualControl>(alphaArm, &robotmap.controllers.codriver); });

  // // m_driveSim = new wom::TempSimSwerveDrive(&simulation_timer, &m_field);
  // // m_driveSim = wom::TempSimSwerveDrive();

  //   intake = new Intake(robotmap.intakeSystem.config);
  // wom::BehaviourScheduler::GetInstance()->Register(intake);
  // intake->SetDefaultBehaviour(
  //     [this]() { return wom::make<IntakeManualControl>(intake, robotmap.controllers.codriver); });

  robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(3.628_rad);
  robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(5.851_rad);
  robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(6.137_rad);
  robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(1.300_rad);

  // robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0_rad);

  // robotmap.swerveBase.moduleConfigs[0].driveMotor.motorController->SetInverted(true);
  // robotmap.swerveBase.moduleConfigs[1].driveMotor.motorController->SetInverted(true);

  // robotmap.swerveBase.moduleConfigs[0].driveMotor.motorController->SetInverted(true);
  // robotmap.swerveBase.moduleConfigs[2].driveMotor.motorController->SetInverted(true);

  // robotmap.alphaArmSystem.armEncoder->Reset();
  // robotmap.alphaArmSystem.armEncoder->Reset();

  // robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0_rad);

  _led = new LED();

  lastPeriodic = wom::now();
}

void Robot::RobotPeriodic() {
  // double encoderDistance = robotmap.alphaArmSystem.armEncoder.GetDistance();
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();
  sched->Tick();

  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 0 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 1 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 2 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 3 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

  // shooter->OnUpdate(dt);
  // intake->OnUpdate(dt);
  // alphaArm->OnUpdate(dt);

  _led->OnUpdate(dt);

  _swerveDrive->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  // m_driveSim->SetPath(m_path_chooser.GetSelected());

  loop.Clear();
  sched->InterruptAll();
}
void Robot::AutonomousPeriodic() {
  // m_driveSim->OnUpdate();
}

void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler* sched = wom::BehaviourScheduler::GetInstance();
  // shooter->OnStart();
  // alphaArm->OnStart();
  sched->InterruptAll();

  _swerveDrive->ResetPose(frc::Pose2d());
  // frontLeft->SetVoltage(4_V);
  // frontRight->SetVoltage(4_V);
  // backLeft->SetVoltage(4_V);
  // backRight->SetVoltage(4_V);
}

void Robot::TeleopPeriodic() {
  if (robotmap.controllers.driver.GetXButtonPressed()) {
    sched->Schedule(
        wom::make<wom::FollowTrajectory>(_swerveDrive, &robotmap.pathplanner, "output/Path1.wpilib.json"));
  }
}

void Robot::DisabledInit() {
  loop.Clear();
  sched->InterruptAll();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
