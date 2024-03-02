// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Behaviours/ClimberBehaviour.h"

ClimberManualControl::ClimberManualControl(Climber* climber, frc::XboxController* codriver)
    : _climber(climber), _codriver(codriver) {
  Controls(climber);
}

void ClimberManualControl::OnTick(units::second_t dt) {
  if (_codriver->GetPOVButton() == 270) {
    if (_climber->GetState() == ClimberState::kIdle) {
      _climber->SetState(ClimberState::kReady);
    } else {
     _climber->SetState(ClimberState::kIdle);
    }
  } else if (_codriver->GetPOVButton() == 90) {
    if (_climber->GetState() == ClimberState::kReady) {
      _climber->SetState(ClimberState::kClimb);
    } else {
      _climber->SetState(ClimberState::kIdle);
    }
  }
}