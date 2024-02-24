// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "IntakeBehaviour.h"

#include <frc/XboxController.h>

IntakeManualControl::IntakeManualControl(Intake* intake, frc::XboxController& codriver) : _intake(intake), _codriver(codriver) {
  Controls(intake);
}

void IntakeManualControl::OnTick(units::second_t dt) {

  if (_codriver.GetBButtonReleased()) {
    if (_rawControl) {
      _intake->SetState(IntakeBehaviourState::kIdleing);
      _intake->SetState(IntakeState::kIdle);
    } else {
      _intake->SetState(IntakeBehaviourState::kRawControl);
      _intake->SetState(IntakeState::kRaw);
    }
  } 
  
  if (_intake->GetState() == IntakeBehaviourState::kRawControl) {
    if (_codriver.GetLeftTriggerAxis() > 0.1) {
      _intake->SetRaw(_codriver.GetRightTriggerAxis() * 10_V);
    } else if (_codriver.GetRightTriggerAxis() > 0.1) {
      _intake->SetRaw(_codriver.GetLeftTriggerAxis() * -10_V);
    } else {
      _intake->SetRaw(0_V);
    }
    _intake->SetState(IntakeState::kRaw);

  } else {
    if (_codriver.GetRightTriggerAxis() > 0.1) {
      if (_intake->GetState() == IntakeBehaviourState::kIntaking) {
        _intake->SetBehaviourState(IntakeBehaviourState::kIdleing);
        _intake->SetState(IntakeState::kIdle);
      } else {
        _intake->SetBehaviourState(IntakeBehaviourState::kIntaking)
      }
    }

    if (_codriver.GetLeftTriggerAxis() > 0.1) {
      if (_intake->GetState() == IntakeBehaviourState::kEjecting) {
        _intake->SetBehaviourState(IntakeBehaviourState::kIdleing);
        _intake->SetState(IntakeState::kIdle);
      } else {
        _intake->SetBehaviourState(IntakeBehaviourState::kEjecting)
      }
    }


    // if (_intake->GetState() == IntakeState::kHold) {
    //   if (_codriver.GetAButtonReleased()) {
    //     _intake->SetState(IntakeState::kEject);
    //   } else if (_codriver.GetYButtonReleased()) {
    //     _intake->SetState(IntakeState::kPass);
    //   } else {
    //     _intake->SetState(IntakeState::kIdle);
    //   }
    // } else if (_intake->GetState() == IntakeState::kIdle) {
    //   if (_codriver.GetRightTriggerAxis() > 0.1) {
    //     _intake->SetState(IntakeState::kIntake);
    //   } else {
    //     _intake->SetState(IntakeState::kIdle);
    //   }
    // }
  }
}

AutoIntake::AutoIntake(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void AutoIntake::OnTick(units::second_t dt) {}
