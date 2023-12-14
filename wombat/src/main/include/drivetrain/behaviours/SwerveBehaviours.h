// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"

#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>

#include <frc/simulation/DifferentialDrivetrainSim.h>

#include "utils/Pathplanner.h"

#include <frc/smartdashboard/Field2d.h>

namespace wom {
namespace drivetrain {
  namespace behaviours {

    class FieldRelativeSwerveDrive : public behaviour::Behaviour {
     public:
      explicit FieldRelativeSwerveDrive(wom::drivetrain::Swerve *swerve, frc::XboxController &driver);

      void OnTick(units::second_t dt) override;

     private:
      wom::drivetrain::Swerve *_swerve;
      frc::XboxController     &_driver;
        
    };

  }  // namespace behaviours
}  // namespace drivetrain
}  // namespace wom
