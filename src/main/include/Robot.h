#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/smartdashboard/SendableChooser.h"
#include <frc/smartdashboard/Field2d.h>

#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>

#include <frc/simulation/DifferentialDrivetrainSim.h>

#include "Wombat.h"


#include "RobotMap.h"

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
    frc::SendableChooser<std::string> m_chooser;

    frc::Field2d m_field;

    frc::sim::DifferentialDrivetrainSim m_driveSim{
            frc::DCMotor::NEO(2), // 2 NEO motors on each side of the drivetrain.
            7.29,               // 7.29:1 gearing reduction.
            7.5_kg_sq_m,        // MOI of 7.5 kg m^2 (from CAD model).
            60_kg,              // The mass of the robot is 60 kg.
            3_in,               // The robot uses 3" radius wheels.
            0.7112_m,           // The track width is 0.7112 meters.

            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}};

    Pathplanner m_pathplanner;


};
