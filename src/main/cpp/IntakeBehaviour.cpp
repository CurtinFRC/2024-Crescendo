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

  /*
  
  if a button is held down:
    set state to pass
  else if right trigger is pressed:
    set state to intaking
  else if left trigger is pressed:
    set state to eject
  else
    set state to idle
  */


  if (_codriver.GetBButtonReleased()) {
    if (_intake->GetState() == IntakeState::kRaw) {
      _intake->SetState(IntakeState::kIdle);
    } else {
      _intake->SetState(IntakeState::kIdle); // this is not a toggle button
    }
  } else if (_rawControl) {
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
        _intake->SetState(IntakeState::kIntake);
    } else if (_codriver.GetLeftTriggerAxis() > 0.1) {
        _intake->SetState(IntakeState::kIntake);
    } else if (_codriver.GetAButtonPressed()) {
      _intake->SetState(IntakeState::kPass);
    } else {
    _intake->SetState(IntakeState::kIdle);  
  }

      // if (_intaking) {
    //   if (_intake->GetState() == IntakeState::kIdle) {
    //     _intake->SetState(IntakeState::kIntake);
    //   }
    // }

    // if (_passing) {
    //   if (_intake->GetState() == IntakeState::kHold) {
    //     _intake->SetState(IntakeState::kPass);
    //   }
    // }

    // if (_ejecting) {
    //   if (_intake->GetState() == IntakeState::kIdle || _intake->GetState() == IntakeState::kHold) {
    //     _intake->SetState(IntakeState::kEject);
    //   }
    // }
  }
}

IntakeAutoControl::IntakeAutoControl(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void IntakeAutoControl::OnTick(units::second_t dt) {}
