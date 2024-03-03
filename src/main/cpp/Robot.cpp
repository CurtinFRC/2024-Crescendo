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
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Auto.h"
#include "RobotMap.h"
#include "behaviour/HasBehaviour.h"
#include "networktables/NetworkTableInstance.h"

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  // shooter = new Shooter(robotmap.shooterSystem.config);
  // wom::BehaviourScheduler::GetInstance()->Register(shooter);
  // shooter->SetDefaultBehaviour(
  //     [this]() { return wom::make<ShooterManualControl>(shooter, &robotmap.controllers.codriver); });

  // sched = wom::BehaviourScheduler::GetInstance();
  // m_chooser.SetDefaultOption("Default Auto", "Default Auto");

  // m_chooser.SetDefaultOption("Default Auto", "Default Auto");

  frc::SmartDashboard::PutData("Auto Selector", &m_chooser);
  sched = wom::BehaviourScheduler::GetInstance();

  m_chooser.SetDefaultOption("kTaxi", "kTaxi");

  for (auto& option : autoOptions) {
    m_chooser.AddOption(option, option);
  }

  // m_path_chooser.SetDefaultOption("Path1", "paths/output/Path1.wpilib.json");

  // m_path_chooser.AddOption("Path1", "paths/output/Path1.wpilib.json");
  // m_path_chooser.AddOption("Path2", "paths/output/Path2.wpilib.json");

  // frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);

  // frc::SmartDashboard::PutData("Field", &m_field);

  // simulation_timer = frc::Timer();

  // robotmap.swerveBase.gyro->Reset();

  _swerveDrive = new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d());
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  _swerveDrive->SetDefaultBehaviour(
      [this]() { return wom::make<wom::ManualDrivebase>(_swerveDrive, &robotmap.controllers.driver); });

  alphaArm = new AlphaArm(&robotmap.alphaArmSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(alphaArm);
  alphaArm->SetDefaultBehaviour(
      [this]() { return wom::make<AlphaArmManualControl>(alphaArm, &robotmap.controllers.codriver); });

  // robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(0_rad);
  // robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0_rad);

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

  // intake = new Intake(robotmap.intakeSystem.config);
  // wom::BehaviourScheduler::GetInstance()->Register(intake);
  // intake->SetDefaultBehaviour(
  //     [this]() { return wom::make<IntakeManualControl>(intake, robotmap.controllers.codriver); });

  // _vision = new Vision("limelight", FMAP("fmap.fmap"));

  // robotmap->vision = new Vision("limelight", FMAP("fmap.fmap"));
  //
  robotmap._builder = autos::InitCommands(_swerveDrive, _shooter, _intake, alphaArm);
  robotmap._simSwerve = new wom::SimSwerve(_swerveDrive);
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();
  // shooter->OnUpdate(dt);
  // sched->Tick();

  // robotmap.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder")
  //     .SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  // robotmap.swerveTable.swerveDriveTable->GetEntry("frontRightEncoder")
  //     .SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  // robotmap.swerveTable.swerveDriveTable->GetEntry("backLeftEncoder")
  //     .SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  // robotmap.swerveTable.swerveDriveTable->GetEntry("backRightEncoder")
  //     .SetDouble(robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

  // _swerveDrive->OnUpdate(dt);
  // shooter->OnStart();
  // intake->OnUpdate(dt);

  // _swerveDrive->OnUpdate(dt);

  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 0 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 1 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 2 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 3 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

  alphaArm->OnUpdate(dt);
  _swerveDrive->OnUpdate(dt);

  robotmap._simSwerve->OnTick();
}

void Robot::AutonomousInit() {
  loop.Clear();
  sched->InterruptAll();

  _swerveDrive->GetConfig().gyro->Reset();

  m_autoSelected = m_chooser.GetSelected();

  if (m_autoSelected == "kTaxi") {
    sched->Schedule(autos::Taxi(robotmap._builder));
  }
}

void Robot::AutonomousPeriodic() {
  fmt::print("Auto selected: {}\n", m_autoSelected);

  robotmap._simSwerve->OnTick(_swerveDrive->GetSetpoint());
  _swerveDrive->MakeAtSetPoint();
}

void Robot::TeleopInit() {
  loop.Clear();

  wom::BehaviourScheduler* sched = wom::BehaviourScheduler::GetInstance();
  sched->InterruptAll();

  // frontLeft->SetVoltage(4_V);
  // frontRight->SetVoltage(4_V);
  // backLeft->SetVoltage(4_V);
  // backRight->SetVoltage(4_V);

  //  FMAP("fmap.fmap");

  // _swerveDrive->OnStart();
  // sched->InterruptAll();

  // reimplement when vision is reimplemented

  // _swerveDrive->SetPose(_vision->GetAngleToObject(VisionTargetObjects::kNote).first);
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {
  /*  std::string x = "[";
    std::string y = "[";
    // _vision->GetDistanceToTarget(16);
    for (int i = 1; i < 17; i++) {
      for (int j = 0; j < 17; j++) {
        frc::Pose2d pose = _vision->AlignToTarget(i, units::meter_t{j * 0.1}, _swerveDrive);
        x += std::to_string(pose.X().value()) + ",";
        y += std::to_string(pose.Y().value()) + ",";
      }
    }

    x += "]";
    y += "]";

    std::cout << x << std::endl;
    std::cout << y << std::endl; */
  // std::cout << _vision->TurnToTarget(1, _swerveDrive).Rotation().Degrees().value() << std::endl;
  // Reimplement when vision is reimplemented
  // frc::Pose2d pose = _vision->TurnToTarget(2, _swerveDrive);
  // nt::NetworkTableInstance::GetDefault().GetTable("vision")->PutNumber("rot",
  //                                                                     pose.Rotation().Degrees().value());
}

void Robot::SimulationPeriodic() {}
