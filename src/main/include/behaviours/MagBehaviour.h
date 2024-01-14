#pragma once
 
#include "wombat.h"
#include "mag.h"
#include <frc/XboxController.h>
 
class MagManualControl : public behaviour::Behaviour {
 public:
  MagManualControl(Mag *mag, frc::XboxController &codriver);
   
  void OnTick(units::second_t dt) override;
 private:
  Mag *_mag;
  frc::XboxController &_codriver;
  bool _rawControl; // Default of Bool is False.
};

class MagAutoControl : public behaviour::Behaviour {
 public:
  MagAutoControl(Mag *mag, units::volt_t voltage);
   
  void OnTick(units::second_t dt) override;
 private:
  Mag *_mag;
};