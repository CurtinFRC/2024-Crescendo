#pragma once

#include "Wombat.h"

#include <frc/XboxController.h>

struct RobotMap {
  struct Controllers {
    frc::XboxController driver = frc::XboxController(0);
    frc::XboxController coDriver = frc::XboxController(1);
    frc::XboxController testController = frc::XboxController(2);
  };

  struct Swerve {
    
  };
};
