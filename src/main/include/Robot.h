// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once
#include <frc/Encoder.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/event/EventLoop.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <string>
#include <vector>

#include "AlphaArm.h"
#include "AlphaArmBehaviour.h"
#include "Intake.h"
#include "IntakeBehaviour.h"
#include "RobotMap.h"
#include "Shooter.h"
#include "ShooterBehaviour.h"
#include "Wombat.h"

class Robot : public frc::TimedRobot {
 public:
  void TestInit() override;
  void TestPeriodic() override;
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;

 private:
  RobotMap robotmap;
  wom::BehaviourScheduler* sched;
  frc::EventLoop loop;

  frc::Field2d field;
  frc::Timer timer;
  frc::SendableChooser<std::string> m_path_chooser;

  wom::SwerveDrive* _swerveDrive;

  AlphaArm* alphaArm;
  Intake* intake;
  Shooter* shooter;

  frc::SendableChooser<std::string> m_chooser;
  const std::string kTaxi = "kTaxi";
  const std::string kAutoTest = "kAutoTest";
  const std::string kQuadrupleClose = "kQuadrupleClose";
  const std::string kQuadrupleFar = "kQuadrupleFar";
  const std::string kQuadrupleCloseDoubleFar = "kQuadrupleCloseDoubleFar";
  const std::string kQuadrupleCloseSingleFar = "kQuadrupleCloseSingleFar";
  std::string m_autoSelected;

  std::string defaultAuto = "kTaxi";
  std::vector<std::string> autoOptions = {
      kTaxi, kAutoTest, kQuadrupleClose, kQuadrupleFar, kQuadrupleCloseDoubleFar, kQuadrupleCloseSingleFar,
  };

  // ctre::phoenix6::hardware::TalonFX *frontLeft;
  // ctre::phoenix6::hardware::TalonFX *frontRight;
  // ctre::phoenix6::hardware::TalonFX *backLeft;
  // ctre::phoenix6::hardware::TalonFX *backRight;
};
