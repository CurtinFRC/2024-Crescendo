// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "ClimberBehaviour.h"

ClimberManualControl::ClimberManualControl(Climber* climber, AlphaArm* arm, frc::XboxController* codriver)
    : _climber(climber), _arm(arm), _codriver(codriver) {
  Controls(climber);
}

void ClimberManualControl::OnTick(units::second_t dt) {
  if (_codriver->GetAButtonPressed()) {
    if (_rawControl) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if (_rawControl) {
    _climber->SetState(ClimberState::kRaw);
    if (wom::deadzone(_codriver->GetLeftY())) {
      _climber->SetRaw(_codriver->GetLeftY() * 6_V);
    } else {
      _climber->SetRaw(0_V);
    }
  } else {
    if (_codriver->GetPOV() == 90) {
      _climber->SetState(ClimberState::kMatch);
      _arm->SetState(AlphaArmState::kClimbAngle);
    } else if (_codriver->GetPOV() == 180) {
      _climber->SetState(ClimberState::kArmDown);
    } else  if (_codriver->GetPOV() == 270){
      _climber->SetState(ClimberState::kRatchet);
      _arm->SetState(AlphaArmState::kClimbed);
    }
  }
}
