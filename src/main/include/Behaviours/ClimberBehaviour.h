#pragma once

#include "wombat.h"
#include "climber.h"
#include <frc/XboxController.h>

class ClimberManualControl : public behaviour::Behaviour {
  public:
    explicit ClimberManualControl(Climber *climber, frc::XboxController *codriver);
    void OnTick(units::second_t dt) override;
  private:
    Climber *_climber;
    frc::XboxController *_codriver;
    bool _rawControl = false;
};

