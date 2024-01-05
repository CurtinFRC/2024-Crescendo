// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/smartdashboard/SendableChooser.h"
#include <frc/smartdashboard/Field2d.h>

#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>

#include <frc/simulation/DifferentialDrivetrainSim.h>

#include "RobotMap.h"

#include <frc/Timer.h>

#include "Wombat.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 protected:

 private:
  behaviour::BehaviourScheduler *sched;
  RobotMap                       robotmap;

    frc::SendableChooser<std::string> m_chooser;

    frc::Field2d m_field;

    frc::Timer simulation_timer;

    frc::SendableChooser<std::string> m_path_chooser;

   wom::TempSimSwerveDrive *m_driveSim;
   wom::SwerveDrive *_swerveDrive;


};
