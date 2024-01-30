// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "ShooterBehaviour.h"

ShooterManualControl::ShooterManualControl(Shooter* shooter, frc::XboxController* codriver)
    : _shooter(shooter), _codriver(codriver) {
  Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {
  _shooter->table->GetEntry("RawControl").SetBoolean(_rawControl);

  if (_codriver->GetAButtonReleased()) {
    if (_rawControl) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if (!_rawControl) {
    if (_codriver->GetYButton()) {
      _shooter->SetState(ShooterState::kSpinUp);
      _shooter->SetPidGoal(10_rad_per_s);
    }
  } else {
    if (_codriver->GetRightTriggerAxis() > 0.1) {
      _shooter->SetRaw(_codriver->GetRightTriggerAxis() * 12_V);
    } else if (_codriver->GetLeftTriggerAxis() > 0.1) {
      _shooter->SetRaw(_codriver->GetLeftTriggerAxis() * -12_V);
    } else {
      _shooter->SetRaw(0_V);
    }
    _shooter->SetState(ShooterState::kRaw);
  }
}

