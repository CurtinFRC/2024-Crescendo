#pragma once

#include <frc/XboxController.h>

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"

#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>

#include <frc/simulation/DifferentialDrivetrainSim.h>

#include "pathplanner.h"

namespace wom {
namespace drivetrain {
  namespace behaviours {

    class FieldRelativeSwerveDrive : public behaviour::Behaviour {
     public:
      explicit FieldRelativeSwerveDrive(wom::drivetrain::Swerve *swerve, frc::XboxController &driver, frc::Field2d *field = NULL);

      void OnTick(units::second_t dt) override;

     private:
      wom::drivetrain::Swerve *_swerve;
      frc::XboxController     &_driver;

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
        
        frc::Field2d *m_field;
    };

  }  // namespace behaviours
}  // namespace drivetrain
}  // namespace wom
