// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "ShooterBehaviour.h"

ShooterManualControl::ShooterManualControl(Shooter* shooter, frc::XboxController* tester)
    : _shooter(shooter), _codriver(tester) {
  Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {
  table->GetEntry("RawControl").SetBoolean(_rawControl);


  if (_codriver->GetBackButtonPressed()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

    if (_rawControl) {
      _shooter->SetState(ShooterState::kRaw);
      
      if (_codriver->GetLeftTriggerAxis() > 0.1) {
        _shooter->SetRaw(12_V * _codriver->GetLeftTriggerAxis());
      } else if (_codriver->GetRightTriggerAxis() > 0.1) {
        _shooter->SetRaw(-12_V * _codriver->GetRightTriggerAxis());
      } else {

        _shooter->SetRaw(0_V);
      }
    } else {
      if (_codriver->GetXButton()) {
        _shooter->SetPidGoal(150_rad_per_s);
        _shooter->SetState(ShooterState::kSpinUp);
      } else if (_codriver->GetYButton()) {
        _shooter->SetPidGoal(300_rad_per_s);
        _shooter->SetState(ShooterState::kSpinUp);
      } else {
        _shooter->SetState(ShooterState::kIdle);
      }
    }
}
