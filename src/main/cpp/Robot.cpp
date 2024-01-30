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

<<<<<<< HEAD
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
=======
>>>>>>> 0029f49 (fix some formatting)
#include "behaviour/HasBehaviour.h"
#include "networktables/NetworkTableInstance.h"

static units::second_t lastPeriodic;

void Robot::RobotInit() {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======

>>>>>>> c83ac05 ([robot/vision] Started work on limelight vision)
=======
>>>>>>> bf9b1b4 (Merged with swerve)
  shooter = new Shooter(robotmap.shooterSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(shooter);
  shooter->SetDefaultBehaviour(
      [this]() { return wom::make<ShooterManualControl>(shooter, &robotmap.controllers.codriver); });

>>>>>>> 0029f49 (fix some formatting)
  sched = wom::BehaviourScheduler::GetInstance();
  m_chooser.SetDefaultOption("Default Auto", "Default Auto");

  // m_chooser.SetDefaultOption("Default Auto", "Default Auto");

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> c83ac05 ([robot/vision] Started work on limelight vision)
  // _swerveDrive = new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d());
  // wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  // _swerveDrive->SetDefaultBehaviour(
  //     [this]() { return wom::make<wom::ManualDrivebase>(_swerveDrive, &robotmap.controllers.driver); });
<<<<<<< HEAD
>>>>>>> 0029f49 (fix some formatting)
=======
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
=======

>>>>>>> c83ac05 ([robot/vision] Started work on limelight vision)
  _swerveDrive = new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d());
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  _swerveDrive->SetDefaultBehaviour(
      [this]() { return wom::make<wom::ManualDrivebase>(_swerveDrive, &robotmap.controllers.driver); });

<<<<<<< HEAD
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

=======
  // m_driveSim = new wom::TempSimSwerveDrive(&simulation_timer, &m_field);
  // m_driveSim = wom::TempSimSwerveDrive();

  alphaArm = new AlphaArm(robotmap.alphaArmSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(alphaArm);
  alphaArm->SetDefaultBehaviour(
      [this]() { return wom::make<AlphaArmManualControl>(alphaArm, &robotmap.controllers.codriver); });

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
>>>>>>> 0029f49 (fix some formatting)
  lastPeriodic = wom::now();
<<<<<<< HEAD
=======

  intake = new Intake(robotmap.intakeSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(intake);
  intake->SetDefaultBehaviour(
      [this]() { return wom::make<IntakeManualControl>(intake, robotmap.controllers.codriver); });

  // _vision = new Vision("limelight", FMAP("fmap.fmap"));

<<<<<<< HEAD
>>>>>>> c83ac05 ([robot/vision] Started work on limelight vision)
=======
  _vision = new Vision("limelight", FMAP("fmap.fmap"));
<<<<<<< HEAD

>>>>>>> c89d969 ([wpiformat] Run wpiformat)
=======
>>>>>>> bf9b1b4 (Merged with swerve)
}

void Robot::RobotPeriodic() {
  // double encoderDistance = robotmap.alphaArmSystem.armEncoder.GetDistance();
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();
  sched->Tick();

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 0 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 1 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 2 offset: ")
      .SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 3 offset: ")
=======
  robotmap.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder")
=======
  robotmap.swerveTable.swerveDriveTable
      ->GetEntry("frontLeftEncoder")

>>>>>>> 57d11c0 (Shooter pid (#117))
=======
  robotmap.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder")
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
      .SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("frontRightEncoder")
      .SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("backLeftEncoder")
      .SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  robotmap.swerveTable.swerveDriveTable->GetEntry("backRightEncoder")
>>>>>>> 0029f49 (fix some formatting)
      .SetDouble(robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

<<<<<<< HEAD
<<<<<<< HEAD
  // shooter->OnUpdate(dt);
  // intake->OnUpdate(dt);
  // alphaArm->OnUpdate(dt);
=======

>>>>>>> c83ac05 ([robot/vision] Started work on limelight vision)
=======
>>>>>>> bf9b1b4 (Merged with swerve)
  _swerveDrive->OnUpdate(dt);
<<<<<<< HEAD
=======
  alphaArm->OnUpdate(dt);
  shooter->OnStart();
  intake->OnUpdate(dt);
<<<<<<< HEAD
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
=======

  // _swerveDrive->OnUpdate(dt);
<<<<<<< HEAD

>>>>>>> c83ac05 ([robot/vision] Started work on limelight vision)
=======
>>>>>>> bf9b1b4 (Merged with swerve)
}

void Robot::AutonomousInit() {
  // m_driveSim->SetPath(m_path_chooser.GetSelected());

  loop.Clear();
  sched->InterruptAll();
}
<<<<<<< HEAD
void Robot::AutonomousPeriodic() {
  // m_driveSim->OnUpdate();
}
=======
void Robot::AutonomousPeriodic() {}
>>>>>>> 0029f49 (fix some formatting)

void Robot::TeleopInit() {
  loop.Clear();
  wom::BehaviourScheduler* sched = wom::BehaviourScheduler::GetInstance();
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // shooter->OnStart();
  // alphaArm->OnStart();
=======
>>>>>>> 0029f49 (fix some formatting)
=======
  shooter->OnStart();
>>>>>>> 57d11c0 (Shooter pid (#117))
=======
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
  sched->InterruptAll();

  // frontLeft->SetVoltage(4_V);
  // frontRight->SetVoltage(4_V);
  // backLeft->SetVoltage(4_V);
  // backRight->SetVoltage(4_V);
<<<<<<< HEAD
<<<<<<< HEAD
=======
  
=======
>>>>>>> bf9b1b4 (Merged with swerve)

  //  FMAP("fmap.fmap");

  // _swerveDrive->OnStart();
  // sched->InterruptAll();
<<<<<<< HEAD

>>>>>>> c83ac05 ([robot/vision] Started work on limelight vision)
=======
>>>>>>> bf9b1b4 (Merged with swerve)
}
<<<<<<< HEAD
void Robot::TeleopPeriodic() {}
=======

void Robot::TeleopPeriodic() {
  // std::cout << "Current AprilTag: " << _vision->CurrentAprilTag() << std::endl;
  // std::cout << "Current Target: " << _vision->TargetIsVisible(VisionTargetObjects::kNote) << std::endl;
  std::cout << "Dist to target: " << _vision->GetDistanceToTarget(VisionTarget::kBlueAmp).first.value()
            << std::endl;
  std::cout << "Angle to target: " << _vision->GetDistanceToTarget(VisionTarget::kBlueAmp).second.value()
            << std::endl;
}
>>>>>>> ae1a0fc ([wpiformat + add ctre_sim to gitignore])

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
  frc::Pose2d pose = _vision->TurnToTarget(2, _swerveDrive);
  nt::NetworkTableInstance::GetDefault().GetTable("vision")->PutNumber("rot",
                                                                       pose.Rotation().Degrees().value());
}

void Robot::SimulationPeriodic() {}
