#pragma once

#include <frc/XboxController.h>

#include "behaviour/Behaviour.h"
#include "drivetrain/Drivetrain.h"

namespace wom {
namespace drivetrain {
  namespace behaviours {
    class TankDrive : public behaviour::Behaviour {
     public:
      TankDrive(wom::drivetrain::Drivetrain *drivebase, frc::XboxController &driver);

      void OnTick(units::second_t dt);

     protected:
     private:
      wom::drivetrain::Drivetrain *_drivebase;
      frc::XboxController &_driver;
    };
  }  // namespace behaviours
}  // namespace drivetrain
}  // namespace wom
