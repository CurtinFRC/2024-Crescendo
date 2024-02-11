#pragma once

#include <frc/PWM.h>
#include "Wombat.h"


enum class LEDState { kIntaking, kHold, kAiming, kShooterReady, kAmpReady, kIdle };

class LED : public behaviour::HasBehaviour {
 public:

  LED();

  void OnUpdate(units::second_t dt);

  void SetState(LEDState state);
  LEDState GetState();

 private:
  LEDState _state = LEDState::kIdle;

  frc::PWM _led{0};

};