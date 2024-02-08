// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "IntakeBehaviour.h"

#include <frc/XboxController.h>

IntakeManualControl::IntakeManualControl(Intake* intake, frc::XboxController& codriver)
    : _intake(intake), _codriver(codriver) {
  Controls(intake);
}

void IntakeManualControl::OnTick(units::second_t dt) {
  if (_codriver.GetBButtonReleased()) {
    if (_rawControl) {
      _rawControl = false;
      _intaking = false;
      _ejecting = false;
      _intake->SetState(IntakeState::kIdle);
    } else {
      _rawControl = true;
      _intaking = false;
      _ejecting = false;
      _intake->SetState(IntakeState::kRaw);
    }
  }

  if (_rawControl) {
    if (_codriver.GetRightTriggerAxis() > 0.1) {
      _intake->SetRaw(_codriver.GetRightTriggerAxis() * 10_V);
    } else if (_codriver.GetLeftTriggerAxis() > 0.1) {
      _intake->SetRaw(_codriver.GetLeftTriggerAxis() * -10_V);
    } else {
      _intake->SetRaw(0_V);
    }
    _intake->SetState(IntakeState::kRaw);

  } else {
    if (_codriver.GetRightTriggerAxis() > 0.1) {
      if (_intaking) {
        _intaking = false;
        _intake->SetState(IntakeState::kIdle);
      } else {
        _intaking = true;
        _ejecting = false;
      }
    }

    if (_codriver.GetLeftTriggerAxis() > 0.1) {
      if (_ejecting) {
        _ejecting = false;
        _intake->SetState(IntakeState::kIdle);
      } else {
        _ejecting = true;
        _intaking = false;
      }
    }

    if (_codriver.GetAButtonPressed()) {
      if (_passing) {
        _passing = false;
        _intake->SetState(IntakeState::kIdle);
      } else {
        _passing = true;
        _intaking = false;
      }
    }

    if (_intaking) {
      if (_intake->GetState() == IntakeState::kIdle) {
        _intake->SetState(IntakeState::kIntake);
      }
    }

    if (_passing) {
      if (_intake->GetState() == IntakeState::kHold) {
        _intake->SetState(IntakeState::kPass);
      }
    }

    if (_ejecting) {
      if (_intake->GetState() == IntakeState::kIdle || _intake->GetState() == IntakeState::kHold) {
        _intake->SetState(IntakeState::kEject);
      }
    }
  }
}

IntakeAutoControl::IntakeAutoControl(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void IntakeAutoControl::OnTick(units::second_t dt) {}
