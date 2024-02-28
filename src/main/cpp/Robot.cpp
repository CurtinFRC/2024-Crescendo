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

#include "Auto.h"
#include "behaviour/HasBehaviour.h"
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/RamseteController.h>
#include <frc/Timer.h>
// #include "ArmBehaviours.h"
#include "behaviour/HasBehaviour.h"
#include "drivetrain/SwerveDrive.h"
#include "frc/geometry/Pose2d.h"
#include "utils/Pathplanner.h"
#include "vision/Vision.h"
#include "vision/VisionBehaviours.h"

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  shooter = new Shooter(robotmap.shooterSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(shooter);
  // shooter->SetDefaultBehaviour(
      // [this]() { return wom::make<ShooterManualControl>(shooter, &robotmap.controllers.codriver); });

  sched = wom::BehaviourScheduler::GetInstance();
  m_chooser.SetDefaultOption("kTaxi", "kTaxi");

  for (auto& option : autoOptions) {
    m_chooser.AddOption(option, option);
  }

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // m_path_chooser.SetDefaultOption("Path1", "paths/output/Path1.wpilib.json");

  // m_path_chooser.AddOption("Path1", "paths/output/Path1.wpilib.json");
  // m_path_chooser.AddOption("Path2", "paths/output/Path2.wpilib.json");

  // frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);

  // frc::SmartDashboard::PutData("Field", &m_field);

  timer = frc::Timer();
  sched = wom::BehaviourScheduler::GetInstance();

  _vision = new Vision("limelight", FMAP("fmap.fmap"));

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

  // simulation_timer = frc::Timer();

  // robotmap.swerveBase.gyro->Reset();
  _swerveDrive = new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d(), _vision->GetLimelight());
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  
  // _swerveDrive->SetDefaultBehaviour([this]() { return robotmap.builder->GetAutoRoutine(); });

  // _arm = new wom::Arm(robotmap.arm.config);
  // wom::BehaviourScheduler::GetInstance()->Register(_arm);
  //
  // _arm->SetDefaultBehaviour(
  //     [this]() { return wom::make<ArmManualControl>(_arm, &robotmap.controllers.codriver); });
  // _swerveDrive->SetDefaultBehaviour(
     // [this]() { return wom::make<wom::ManualDrivebase>(_swerveDrive, &robotmap.controllers.driver); });

  // m_driveSim = new wom::TempSim_swerveDrive(&simulation_timer, &m_field);
  // m_driveSim = wom::TempSim_swerveDrive();

  // alphaArm = new AlphaArm(robotmap.alphaArmSystem.config);
  // wom::BehaviourScheduler::GetInstance()->Register(alphaArm);
  // alphaArm->SetDefaultBehaviour(
  //     [this]() { return wom::make<AlphaArmManualControl>(alphaArm, &robotmap.controllers.codriver); });

  // robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0_rad);

  // frontLeft = new ctre::phoenix6::hardware::TalonFX(7, "Drivebase");  // front left
  // frontRight = new ctre::phoenix6::hardware::TalonFX(2, "Drivebase");   // front right
  // backLeft = new ctre::phoenix6::hardware::TalonFX(6, "Drivebase");   // back left
  // backRight = new ctre::phoenix6::hardware::TalonFX(4, "Drivebase");  // back right
  // frontLeft = new ctre::phoenix6::hardware::TalonFX(9, "Drivebase");   // front left
  // frontRight = new ctre::phoenix6::hardware::TalonFX(1, "Drivebase");   // front right
  // backLeft = new ctre::phoenix6::hardware::TalonFX(5, "Drivebase");   // back left
  // backRight = new ctre::phoenix6::hardware::TalonFX(3, "Drivebase");
  lastPeriodic = wom::now();

  intake = new Intake(robotmap.intakeSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(intake);
  // intake->SetDefaultBehaviour(
      // [this]() { return wom::make<IntakeManualControl>(intake, robotmap.controllers.codriver); });

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

  robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0.588_rad);
  robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(2.647_rad);
  robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(2.979_rad);
  robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(4.388_rad);

  // robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0_rad);

  // robotmap.swerveBase.moduleConfigs[0].driveMotor.motorController->SetInverted(true);
  // robotmap.swerveBase.moduleConfigs[1].driveMotor.motorController->SetInverted(true);

  // robotmap.swerveBase.moduleConfigs[0].driveMotor.motorController->SetInverted(true);
  // robotmap.swerveBase.moduleConfigs[2].driveMotor.motorController->SetInverted(true);

  // robotmap.alphaArmSystem.armEncoder->Reset();

  // robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0_rad);

  
  shooter->OnStart();

  lastPeriodic = wom::now();

  robotmap.builder = autos::InitCommands(_swerveDrive, shooter, intake, alphaArm);
  robotmap.simSwerve = new wom::SimSwerve(_swerveDrive);
}

void Robot::RobotPeriodic() {
  // double encoderDistance = robotmap.alphaArmSystem.armEncoder.GetDistance();
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();
  // shooter->OnUpdate(dt);
  sched->Tick();

  robotmap.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder")
      .SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("frontRightEncoder")
      .SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("backLeftEncoder")
      .SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("backRightEncoder")
      .SetDouble(robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());
  // sched->Tick();

  // intake->OnUpdate(dt);
  // alphaArm->OnUpdate(dt);
  _swerveDrive->OnUpdate(dt);
  // alphaArm->OnUpdate(dt);
  // intake->OnUpdate(dt);
  // _arm->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  loop.Clear();
  sched->InterruptAll();

  _swerveDrive->GetConfig().gyro->Reset();

  m_autoSelected = m_chooser.GetSelected();

  if (m_autoSelected == "kTaxi") {
    sched->Schedule(autos::Taxi(robotmap.builder));
    // sched->Schedule(wom::make<wom::FollowPath>(_swerveDrive, "FirstNote"));
  }// else if (m_autoSelected == "Auto Test") {
  //   sched->Schedule(autos::AutoTest(_swerveDrive, shooter, intake, alphaArm));
  // } else if (m_autoSelected == "Quadruple Close") {
  //   sched->Schedule(autos::QuadrupleClose(_swerveDrive, shooter, intake, alphaArm));
  // } else if (m_autoSelected == "Quadruple Far") {
  //   sched->Schedule(autos::QuadrupleFar(_swerveDrive, shooter, intake, alphaArm));
  // } else if (m_autoSelected == "Quadruple Close Double Far") {
  //   sched->Schedule(autos::QuadrupleCloseDoubleFar(_swerveDrive, shooter, intake, alphaArm));
  // } else if (m_autoSelected == "Quadruple Close Single Far") {
  //   sched->Schedule(autos::QuadrupleCloseSingleFar(_swerveDrive, shooter, intake, alphaArm));
  // }
}
void Robot::AutonomousPeriodic() {
  fmt::print("Auto selected: {}\n", m_autoSelected);

  robotmap.simSwerve->OnTick();

  // robotmap.simSwerve->OnTick(_swerveDrive->GetSetpoint());
  _swerveDrive->MakeItAtSetpoint();
}

// void Robot::AutonomousPeriodic() {
// m_driveSim->OnUpdate();
// wom::make<AlignToAprilTag>(_vision, VisionTarget::kRedChain3, _swerveDrive);
// }

void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler* sched = wom::BehaviourScheduler::GetInstance();
  // shooter->OnStart();
  // alphaArm->OnStart();
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

  if (robotmap.controllers.driver.GetXButtonPressed()) {
    _vision->TurnToTarget(VisionTarget::kBlueSpeakerCenter, _swerveDrive);
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
