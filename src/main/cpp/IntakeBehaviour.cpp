// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "IntakeBehaviour.h"

#include <frc/XboxController.h>

IntakeManualControl::IntakeManualControl(Intake* intake, frc::XboxController& codriver) : _intake(intake), _codriver(codriver) {
  Controls(intake);
}

void IntakeManualControl::OnTick(units::second_t dt) {

  if (_codriver.GetBackButtonReleased()) {
    if (_rawControl) {
      _rawControl = false;
      _intake->SetState(IntakeState::kIdle);
    } else {
      _rawControl = true;
      _intake->SetState(IntakeState::kRaw);
    }
  } 
  
  if (_rawControl) {
    _intake->SetState(IntakeState::kRaw);
    if (_codriver.GetLeftTriggerAxis() > 0.1) {
      _intake->SetRaw(_codriver.GetLeftTriggerAxis() * 10_V);
    }
    else if (_codriver.GetRightTriggerAxis() > 0.1) {
      _intake->SetRaw(_codriver.GetRightTriggerAxis() * -10_V);
    } else {
      _intake->SetRaw(0_V);
    }
    } else if (_codriver.GetXButton()) {
      if (_intake->GetState() == IntakeState::kIdle) {
        _intake->SetState(IntakeState::kIntake);
      }
    } else if (_codriver.GetRightBumper() || _codriver.GetRightTriggerAxis() > 0.1) {
      if (_intake->GetState() == IntakeState::kHold) {
        _intake->SetState(IntakeState::kPass);
        
      } else {
        _intake->SetState(IntakeState::kIdle);
      }
    } else if (_codriver.GetBButtonPressed()) {
      if (_intake->GetState() == IntakeState::kHold) {
        _intake->SetState(IntakeState::kEject);
      } else {
        _intake->SetState(IntakeState::kIdle);
      
    }
    } else {
        _intake->SetState(IntakeState::kIdle);
      }
}

AutoIntake::AutoIntake(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void AutoIntake::OnTick(units::second_t dt) {
  _intake->SetState(IntakeState::kIntake);
}

IntakeNote::IntakeNote(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void IntakeNote::OnTick(units::second_t dt) {
  _intake->SetState(IntakeState::kIntake);

  if (_intake->GetState() == IntakeState::kHold) {
    SetDone();
  }
}

PassNote::PassNote(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void PassNote::OnTick(units::second_t dt) {
  _intake->SetState(IntakeState::kPass);

  if (_intake->GetState() == IntakeState::kIdle) {
    SetDone();
  }
}

EjectNote::EjectNote(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void EjectNote::OnTick(units::second_t dt) {
  _intake->SetState(IntakeState::kEject);

  if (_intake->GetState() == IntakeState::kIdle) {
    SetDone();
  }
}
