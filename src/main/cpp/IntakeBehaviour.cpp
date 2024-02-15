// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "IntakeBehaviour.h"

#include <frc/XboxController.h>

IntakeManualControl::IntakeManualControl(Intake* intake, frc::XboxController& codriver) : _intake(intake), _codriver(codriver) {
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

    if (_rawControl) {
      _rawControl = false;
    } else {
      _rawControl = true;
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
    
    if (_intake->GetState() == IntakeState::kHold) {
      if (_codriver.GetAButtonReleased()) {
        _intake->SetState(IntakeState::kEject);
      } else if (_codriver.GetYButtonReleased()) {
        _intake->SetState(IntakeState::kPass);
      } else {
        _intake->SetState(IntakeState::kIdle);
      }
    } else if (_intake->GetState() == IntakeState::kIdle) {
      if (_codriver.GetRightTriggerAxis() > 0.1) {
        _intake->SetState(IntakeState::kIntake);
      } else {
        _intake->SetState(IntakeState::kIdle);
      }
    }
  }
}

AutoIntake::AutoIntake(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void AutoIntake::OnTick(units::second_t dt) {}
