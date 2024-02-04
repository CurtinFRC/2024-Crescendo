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
      _intake->setState(IntakeState::kIdle);
    } else {
      _rawControl = true;
      _intaking = false;
      _ejecting = false;
      _intake->setState(IntakeState::kRaw);
    }
  }

  if (_rawControl) {
    if (_codriver.GetRightTriggerAxis() > 0.1) {
      _intake->setRaw(_codriver.GetRightTriggerAxis() * 10_V);
    } else if (_codriver.GetLeftTriggerAxis() > 0.1) {
      _intake->setRaw(_codriver.GetLeftTriggerAxis() * -10_V);
    } else {
      _intake->setRaw(0_V);
    }
    _intake->setState(IntakeState::kRaw);

  } else {
    if (_codriver.GetRightTriggerAxis() > 0.1) {
      if (_intaking) {
        _intaking = false;
        _intake->setState(IntakeState::kIdle);
      } else {
        _intaking = true;
        _ejecting = false;
      }
    }

    if (_codriver.GetLeftTriggerAxis() > 0.1) {
      if (_ejecting) {
        _ejecting = false;
        _intake->setState(IntakeState::kIdle);
      } else {
        _ejecting = true;
        _intaking = false;
      }
    }

    if (_codriver.GetAButtonPressed()) {
      if (_passing) {
        _passing = false;
        _intake->setState(IntakeState::kIdle);
      } else {
        _passing = true;
        _intaking = false;
      }
    }

    if (_intaking) {
      if (_intake->getState() == IntakeState::kIdle) {
        _intake->setState(IntakeState::kIntake);
      }
    }

    if (_passing) {
      if (_intake->getState() == IntakeState::kHold) {
        _intake->setState(IntakeState::kPass);
      }
    }

    if (_ejecting) {
      if (_intake->getState() == IntakeState::kIdle || _intake->getState() == IntakeState::kHold) {
        _intake->setState(IntakeState::kEject);
      }
    }
  }
}

AutoIntake::AutoIntake(Intake* intake, units::volt_t intakeVolt) : _intake(intake) {
  Controls(intake);
}

void AutoIntake::OnTick(units::second_t dt) {
  // if (_intake->GetConfig().intakeSensor->Get() == 1) {
  //   _intake->setState(IntakeState::kPass);
  // } else if (_intake->GetConfig().magSensor->Get() == 0) {
  //   _intake->setState(IntakeState::kIdle);
  // }
} 