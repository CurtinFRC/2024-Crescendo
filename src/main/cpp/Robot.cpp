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
#include "behaviour/HasBehaviour.h"

static units::second_t lastPeriodic;

void Robot::RobotInit() {

  shooter = new Shooter(robotmap.shooterSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(shooter);
  shooter->SetDefaultBehaviour(
     [this]() {return wom::make<ShooterManualControl>(shooter, &robotmap.controllers.testController); });
  
  sched = wom::BehaviourScheduler::GetInstance();
  m_chooser.SetDefaultOption("Default Auto", "Default Auto");

  frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

  m_path_chooser.SetDefaultOption("Path1", "paths/output/Path1.wpilib.json");

  m_path_chooser.AddOption("Path1", "paths/output/Path1.wpilib.json");
  m_path_chooser.AddOption("Path2", "paths/output/Path2.wpilib.json");

  frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);

  frc::SmartDashboard::PutData("Field", &m_field);

  simulation_timer = frc::Timer();

  robotmap.swerveBase.gyro->Reset();

  _swerveDrive =
      new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d());
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  _swerveDrive->SetDefaultBehaviour([this]() {
    return wom::make<wom::ManualDrivebase>(_swerveDrive,
                                           &robotmap.controllers.driver);
  });

  
  // m_driveSim = new wom::TempSimSwerveDrive(&simulation_timer, &m_field);
  // m_driveSim = wom::TempSimSwerveDrive();

  robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0_rad);
  robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(0_rad);
  robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(0_rad);
  robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0_rad);




  // frontLeft = new ctre::phoenix6::hardware::TalonFX(7, "Drivebase");  // front left
  // frontRight = new ctre::phoenix6::hardware::TalonFX(2, "Drivebase");   // front right
  // backLeft = new ctre::phoenix6::hardware::TalonFX(6, "Drivebase");   // back left
  // backRight = new ctre::phoenix6::hardware::TalonFX(4, "Drivebase");  // back right
  // frontLeft = new ctre::phoenix6::hardware::TalonFX(9, "Drivebase");   // front left
  // frontRight = new ctre::phoenix6::hardware::TalonFX(1, "Drivebase");   // front right
  // backLeft = new ctre::phoenix6::hardware::TalonFX(5, "Drivebase");   // back left
  // backRight = new ctre::phoenix6::hardware::TalonFX(3, "Drivebase");
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();
  shooter->OnUpdate(dt);
  sched->Tick();

  robotmap.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder").SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("frontRightEncoder").SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("backLeftEncoder").SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("backRightEncoder").SetDouble(robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

  _swerveDrive->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  // loop.Clear();
  // sched->InterruptAll();
}
void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler *sched = wom::BehaviourScheduler::GetInstance();
  shooter->OnStart();
  sched->InterruptAll();
}
void Robot::TeleopPeriodic() {
  
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}