#pragma once

#include "Wombat.h"
#include <frc/XboxController.h>
#include "tank.h"

class TankManualControl : public behaviour::Behaviour {
 public:
  TankManualControl(TankDrive *tank, frc::XboxController &driver);

  void OnTick(units::second_t dt) override;
 private:
  TankDrive *_tank;
  frc::XboxController &_driver;

  units::volt_t _leftStick;
  units::volt_t _rightStick;
};