// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "behaviours/ClimberBehaviour.h"

ClimberManualControl::ClimberManualControl(Climber* climber, frc::XboxController* codriver)
    : _climber(climber), _codriver(codriver) {
  Controls(climber);
}

void ClimberManualControl::OnTick(units::second_t dt) {
  if (_codriver->GetXButtonPressed()) {
    if (_rawControl == false) {
      _rawControl = true;
    } else {
      _rawControl = false;
    }
  }
  if (_rawControl) {
    _climber->SetRaw(wom::deadzone(_codriver->GetRightY()) * 8_V);
    _climber->SetState(ClimberState::kRaw);
  } else {
    if (_codriver->GetYButtonPressed()) {
      _climber->SetState(ClimberState::kClimb);
    }
  }
}
