// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"
#include "Auto.h"
#include "RobotMap.h"
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/controller/RamseteController.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/acceleration.h>
// #include <units/angle.h>

// include units
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "behaviour/HasBehaviour.h"
#include "networktables/NetworkTableInstance.h"

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

  _led = new LED();

  armServo = new frc::Servo(3);

  shooter = new Shooter(robotmap.shooterSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(shooter);
  shooter->SetDefaultBehaviour(
      [this]() { return wom::make<ShooterManualControl>(shooter, &robotmap.controllers.codriver, _led); });

  _swerveDrive = new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d());
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  _swerveDrive->SetDefaultBehaviour(
      [this]() { return wom::make<wom::ManualDrivebase>(_swerveDrive, &robotmap.controllers.driver); });

  alphaArm = new AlphaArm(&robotmap.alphaArmSystem.config);
  intake = new Intake(robotmap.intakeSystem.config);

  wom::BehaviourScheduler::GetInstance()->Register(alphaArm);
  alphaArm->SetDefaultBehaviour([this]() {
    return wom::make<AlphaArmManualControl>(alphaArm, intake, &robotmap.controllers.codriver);
  });

  climber = new Climber(robotmap.climberSystem.config);
  wom::BehaviourScheduler::GetInstance()->Register(climber);
  climber->SetDefaultBehaviour([this]() {
    return wom::make<ClimberManualControl>(climber, alphaArm, &robotmap.controllers.codriver);
  });

  wom::BehaviourScheduler::GetInstance()->Register(intake);
  intake->SetDefaultBehaviour(
      [this]() { return wom::make<IntakeManualControl>(intake, alphaArm, robotmap.controllers.codriver); });

  robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0.45229_rad);
  robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(2.6846_rad);
  robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(3.01121_rad);
  robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(4.4524_rad);

  lastPeriodic = wom::now();
}

void Robot::RobotPeriodic() {
  // double encoderDistance = robotmap.alphaArmSystem.armEncoder.GetDistance();
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();
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

  //   robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 0 offset:
  //   ").SetDouble(robotmap.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  //   robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 1 offset:
  //   ").SetDouble(robotmap.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  //   robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 2 offset:
  //   ").SetDouble(robotmap.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  //   robotmap.swerveTable.swerveDriveTable->GetEntry("Encoder 3 offset:
  //   ").SetDouble(robotmap.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

  _led->OnUpdate(dt);
  shooter->OnUpdate(dt);
  alphaArm->OnUpdate(dt);
  _swerveDrive->OnUpdate(dt);
  intake->OnUpdate(dt);
  climber->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  loop.Clear();
  sched->InterruptAll();

  _swerveDrive->GetConfig().gyro->Reset();

  m_autoSelected = m_chooser.GetSelected();

  robotmap._builder = autos::InitCommands(_swerveDrive, shooter, intake, alphaArm);

  if (m_autoSelected == "kTaxi") {
    sched->Schedule(autos::Taxi(robotmap._builder));
  }

  _swerveDrive->OnStart();
}

void Robot::AutonomousPeriodic() {
  // robotmap._simSwerve->OnTick(_swerveDrive->GetSetpoint());
  // _swerveDrive->MakeAtSetPoint();
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
  armServo->SetAngle(130);
}

void Robot::TeleopPeriodic() {
  if (robotmap.controllers.testController.GetPOV() == 0) {
    armServo->SetAngle(130);
  } else if (robotmap.controllers.codriver.GetPOV() == 270) {
    climberTimer.Start();
    if (climberTimer.Get() > 1_s) {
      armServo->SetAngle(0);
      climberTimer.Stop();
      climberTimer.Reset();
    }
  }

  robotmap.swerveTable.swerveDriveTable->GetEntry("SERVO POS").SetDouble(armServo->GetAngle());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
