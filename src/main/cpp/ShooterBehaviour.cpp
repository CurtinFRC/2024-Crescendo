// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "ShooterBehaviour.h"

ShooterManualControl::ShooterManualControl(Shooter* shooter, frc::XboxController* tester, LED* led)
    : _shooter(shooter), _codriver(tester), _led(led) {
  Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {
  table->GetEntry("RawControl").SetBoolean(_rawControl);

  if (_codriver->GetStartButtonPressed()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if (_rawControl) {
    _shooter->SetState(ShooterState::kRaw);
    if (_codriver->GetRightBumper()) {
      _shooter->SetRaw(8_V);
    } else if (_codriver->GetLeftBumper()) {
      _shooter->SetRaw(-8_V);
    } else {
      _shooter->SetRaw(0_V);
    }
  } else {
    if (_codriver->GetLeftTriggerAxis() > 0.1) {
      _shooter->SetPidGoal(1500_rad_per_s);
      _shooter->SetState(ShooterState::kSpinUp);
    } else if (_codriver->GetLeftBumper()) {
      _shooter->SetPidGoal(300_rad_per_s);
      _shooter->SetState(ShooterState::kSpinUp);
      _led->SetState(LEDState::kAiming);
      } else if (_codriver->GetBButton()) {
      _shooter->SetState(ShooterState::kReverse);
      

    } else if (_codriver->GetAButton()) {
      _shooter->SetPidGoal(1500_rad_per_s);
      _shooter->SetState(ShooterState::kSpinUp);
      _led->SetState(LEDState::kAiming);
    } else {
      _shooter->SetState(ShooterState::kIdle);
      _led->SetState(LEDState::kIdle);
    }
  }
}
