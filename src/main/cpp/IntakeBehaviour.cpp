// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "IntakeBehaviour.h"

#include <frc/XboxController.h>

IntakeManualControl::IntakeManualControl(Intake* intake, frc::XboxController& codriver) : _intake(intake), _codriver(codriver) {
  Controls(intake);
}

void IntakeManualControl::OnTick(units::second_t dt) {

  switch (_behaviourState) {
    case IntakeBehaviourState::kIdleing:
    {
      if (_intake->GetState() != IntakeState::kIdle) {
        _intake->SetState(IntakeState::kIdle);
      }
    }
    break;
    case IntakeBehaviourState::kIntaking:
    {
      if (_intake->GetState() == IntakeState::kIdle) {
        _intake->SetState(IntakeState::kIntake);
      }
    }
    break;
    case IntakeBehaviourState::kPassing:
    {
      if (_intake->GetState() == IntakeState::kIdle) {
        _intake->SetState(IntakeState::kPass);
      }
    }
    break;
    case IntakeBehaviourState::kEjecting:
    {
      if (_intake->GetState() == IntakeState::kIdle) {
        _intake->SetState(IntakeState::kIntake);
      }
    }
    break;
    case IntakeBehaviourState::kRawControl:
    {
      if (_intake->GetState() != IntakeState::kRaw)
      _intake->SetState(IntakeState::kRaw);
    }
    break;
  }

  if (_codriver.GetBButtonReleased()) {
    if (_rawControl) {
      IntakeManualControl::SetBehaviourState(IntakeBehaviourState::kIdleing);
    } else {
      IntakeManualControl::SetBehaviourState(IntakeBehaviourState::kRawControl);
    }
  } 
  
  if (IntakeManualControl::GetBehaviourState() == IntakeBehaviourState::kRawControl) {
    if (_codriver.GetLeftTriggerAxis() > 0.1) {
      _intake->SetRaw(_codriver.GetRightTriggerAxis() * 10_V);
    } else if (_codriver.GetRightTriggerAxis() > 0.1) {
      _intake->SetRaw(_codriver.GetLeftTriggerAxis() * -10_V);
    } else {
      _intake->SetRaw(0_V);
    }

  } else {
    if (_codriver.GetRightTriggerAxis() > 0.1) {
      if (IntakeManualControl::GetBehaviourState() == IntakeBehaviourState::kIntaking) {
        IntakeManualControl::SetBehaviourState(IntakeBehaviourState::kIdleing);
      } else {
        IntakeManualControl::SetBehaviourState(IntakeBehaviourState::kIntaking);
      }
    }

    if (_codriver.GetLeftTriggerAxis() > 0.1) {
      if (IntakeManualControl::GetBehaviourState() == IntakeBehaviourState::kEjecting) {
        IntakeManualControl::SetBehaviourState(IntakeBehaviourState::kIdleing);
      } else {
        IntakeManualControl::SetBehaviourState(IntakeBehaviourState::kEjecting);
      }
    }
  }
}

IntakeBehaviourState IntakeManualControl::IntakeManualControl::GetBehaviourState() {
  return _behaviourState;
}
void IntakeManualControl::IntakeManualControl::SetBehaviourState(IntakeBehaviourState behaviourState) {
  _behaviourState = behaviourState;
}

AutoIntake::AutoIntake(Intake* intake) : _intake(intake) {
  Controls(intake);
}
void AutoIntake::OnTick(units::second_t dt) {}
