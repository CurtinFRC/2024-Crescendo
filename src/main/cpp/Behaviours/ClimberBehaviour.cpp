// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Behaviours/ClimberBehaviour.h"

ClimberManualControl::ClimberManualControl(Climber* climber, frc::XboxController* codriver)
    : _climber(climber), _codriver(codriver) {
  Controls(climber);
}

void ClimberManualControl::OnTick(units::second_t dt) {
  if (_codriver->GetRightStickButtonPressed()) {
    if (_climber->GetState() == ClimberState::kIdle) {
      _climber->SetState(ClimberState::kRaw);
    } else {
     _climber->SetState(ClimberState::kIdle);
    }
  }
}
