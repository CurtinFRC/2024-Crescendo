#pragma once

#include "Wombat.h"
#include <frc/XboxController.h>
#include "arm.h"

class ArmManualControl : public behaviour::Behaviour {
 public:
  ArmManualControl(Arm *arm, frc::XboxController &codriver);

  void OnTick(units::second_t dt) override;

 private:
  Arm *_arm;
  frc::XboxController &_codriver;
};