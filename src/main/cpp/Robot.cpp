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

// include units
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "behaviour/HasBehaviour.h"
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/RamseteController.h>
#include <frc/Timer.h>
#include "behaviour/HasBehaviour.h"
#include "frc/geometry/Pose2d.h"

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  sched = wom::BehaviourScheduler::GetInstance();
  m_chooser.SetDefaultOption("Default Auto", "Default Auto");

  sched = wom::BehaviourScheduler::GetInstance();
  m_chooser.SetDefaultOption("kTaxi", "kTaxi");

//   for (auto& option : autoOptions) {
//     m_chooser.AddOption(option, option);
//   }

//   frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // m_path_chooser.SetDefaultOption("Path1", "paths/output/Path1.wpilib.json");

  // m_path_chooser.AddOption("Path1", "paths/output/Path1.wpilib.json");
  // m_path_chooser.AddOption("Path2", "paths/output/Path2.wpilib.json");

  // frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);

  // frc::SmartDashboard::PutData("Field", &m_field);

  _swerveDrive =
      new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d());
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  _swerveDrive->SetDefaultBehaviour([this]() {
    return wom::make<wom::ManualDrivebase>(_swerveDrive,
                                           &robotmap.controllers.driver);
  });


  alphaArm = new AlphaArm(robotmap.alphaArmSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(alphaArm);
  alphaArm->SetDefaultBehaviour(
      [this]() { return wom::make<AlphaArmManualControl>(alphaArm, &robotmap.controllers.codriver); });


    intake = new Intake(robotmap.intakeSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(intake);
  intake->SetDefaultBehaviour(
      [this]() { return wom::make<IntakeManualControl>(intake, robotmap.controllers.codriver); });


  // m_driveSim = new wom::TempSimSwerveDrive(&simulation_timer, &m_field);
  // m_driveSim = wom::TempSimSwerveDrive();

  // robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0.5948_rad);
  // robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(2.6156_rad);
  // robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(2.8931_rad);
  // robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(-1.7417_rad);

  // robotmap.swerveBase.moduleConfigs[0].driveMotor.motorController->SetInverted(true);
  // robotmap.swerveBase.moduleConfigs[1].driveMotor.motorController->SetInverted(true);

  // robotmap.swerveBase.moduleConfigs[0].turnMotor.motorController->SetInverted(true);
  // robotmap.swerveBase.moduleConfigs[1].turnMotor.motorController->SetInverted(true);

  robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0.45229_rad);
  robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(2.6846_rad);
  robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(3.01121_rad);
  robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(4.4524_rad);


  // frontLeft = new ctre::phoenix6::hardware::TalonFX(7, "Drivebase");  // front left
  // frontRight = new ctre::phoenix6::hardware::TalonFX(2, "Drivebase");   // front right
  // backLeft = new ctre::phoenix6::hardware::TalonFX(6, "Drivebase");   // back left
  // backRight = new ctre::phoenix6::hardware::TalonFX(4, "Drivebase");  // back right
  // frontLeft = new ctre::phoenix6::hardware::TalonFX(9, "Drivebase");   // front left
  // frontRight = new ctre::phoenix6::hardware::TalonFX(1, "Drivebase");   // front right
  // backLeft = new ctre::phoenix6::hardware::TalonFX(5, "Drivebase");   // back left
  // backRight = new ctre::phoenix6::hardware::TalonFX(3, "Drivebase");


  lastPeriodic = wom::now();
}

void Robot::RobotPeriodic() {
  // double encoderDistance = robotmap.alphaArmSystem.armEncoder.GetDistance();
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();
//   shooter->OnUpdate(dt);
//   sched->Tick();

//   robotmap.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder")
//       .SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
//   robotmap.swerveTable.swerveDriveTable->GetEntry("frontRightEncoder")
//       .SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
//   robotmap.swerveTable.swerveDriveTable->GetEntry("backLeftEncoder")
//       .SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
//   robotmap.swerveTable.swerveDriveTable->GetEntry("backRightEncoder")
//       .SetDouble(robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());
//   sched->Tick();

//   _swerveDrive->OnUpdate(dt);
//   alphaArm->OnUpdate(dt);
//   shooter->OnStart();
  // _arm->OnUpdate(dt);

  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 0 offset: ").SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 1 offset: ").SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 2 offset: ").SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 3 offset: ").SetDouble(robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

  intake->OnUpdate(dt);
  alphaArm->OnUpdate(dt);
  _swerveDrive->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  loop.Clear();
  sched->InterruptAll();
}
void Robot::AutonomousPeriodic() {
  // m_driveSim->OnUpdate();
}


void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler* sched = wom::BehaviourScheduler::GetInstance();
  sched->InterruptAll();

  // frontLeft->SetVoltage(4_V);
  // frontRight->SetVoltage(4_V);
  // backLeft->SetVoltage(4_V);
  // backRight->SetVoltage(4_V);
}
// void Robot::TeleopPeriodic() {}
void Robot::TeleopPeriodic() {
  // if (robotmap.controllers.driver.GetXButtonPressed() &&
  //     vision->TargetIsVisible(VisionTargetObjects::kNote)) {
  //   units::degree_t turn = vision->LockOn(VisionTargetObjects::kNote);
  //
  //   frc::Pose2d current_pose = _swerveDrive->GetPose();
  //
  //   std::cout << "angle: " << turn.value() << std::endl;
  //   current_pose.RotateBy(turn);
  //
  //   wom::make<wom::DrivebasePoseBehaviour>(_swerveDrive, current_pose);
  // } else {
  //   wom::make<wom::ManualDrivebase>(_swerveDrive, &robotmap.controllers.driver);
  // }

//   if (robotmap.controllers.driver.GetXButtonPressed()) {
//     _vision->TurnToTarget(VisionTarget::kBlueSpeakerCenter, _swerveDrive);
//   }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
