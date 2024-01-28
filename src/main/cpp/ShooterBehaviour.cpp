// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "ShooterBehaviour.h"

ShooterManualControl::ShooterManualControl(Shooter* shooter, frc::XboxController* tester)
    : _shooter(shooter), _tester(tester) {
  Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {
  table->GetEntry("RawControl").SetBoolean(_rawControl);


  if (_tester->GetAButtonPressed()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

    if (_rawControl) {
      _shooter->SetState(ShooterState::kRaw);
      
      if (_tester->GetLeftTriggerAxis() > 0.1) {
        _shooter->SetRaw(12_V * _tester->GetLeftTriggerAxis());
      } else if (_tester->GetRightTriggerAxis() > 0.1) {
        _shooter->SetRaw(-12_V * _tester->GetRightTriggerAxis());
      } else {
        _shooter->SetRaw(0_V);
      }
    } else {
      _shooter->SetState(ShooterState::kSpinUp);
      _shooter->SetPidGoal(20_rad_per_s);
    }
  // }
}

