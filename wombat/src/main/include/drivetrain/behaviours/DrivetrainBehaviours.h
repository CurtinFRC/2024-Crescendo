#pragma once

#include "drivetrain/Drivetrain.h"
#include "behaviour/Behaviour.h"

namespace wom {
namespace drivetrain {
namespace behaviours {
  class TankDrive : public behaviour::Behaviour {
   public:
    TankDrive(wom::drivetrain::Drivetrain *drivebase);

    void OnTick(units::second_t dt);

   protected:

   private:
    wom::drivetrain::Drivetrain *_drivebase;

  };
}
}
}


