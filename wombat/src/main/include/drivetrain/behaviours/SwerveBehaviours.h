#pragma once

#include <frc/XboxController.h>

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"

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
