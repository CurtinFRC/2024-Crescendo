#pragma once
 
#include "wombat.h"
#include "mag.h"
#include <frc/XboxController.h>
 
class MagManualControl : public behaviour::Behaviour {
 public:
  MagManualControl(Mag *mag, frc::XboxController *codriver);
   
  void OnTick(units::second_t dt) override;
 private:
  Mag *_mag;
  frc::XboxController *_codriver;
  bool _rawControl; // Default of Bool is False.
};

class MagAutoPass : public behaviour::Behaviour {
 public:
  MagAutoPass(Mag *mag);
   
  void OnTick(units::second_t dt) override;
 private:
  Mag *_mag;
};

class MagAutoHold : public behaviour::Behaviour {
 public:
  MagAutoHold(Mag *mag);
   
  void OnTick(units::second_t dt) override;
 private:
  Mag *_mag;
};
