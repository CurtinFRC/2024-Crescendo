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
    if (_intake->getState() == IntakeState::kRaw) {
      _intake->setState(IntakeState::kIdle);
    } else {
      _intake->setState(IntakeState::kRaw);
    }
  }

  if (_intake->getState() == IntakeState::kRaw) {
    if (_codriver.GetRightBumper()) {
      _intake->setRaw(8_V);
    } else if (_codriver.GetLeftBumper()) {
      _intake->setRaw(-8_V);
    } else {
      _intake->setRaw(0_V);
    }

  } else {
    if (_codriver.GetRightBumperPressed()) {
      if (_intake->getState() == IntakeState::kIntake) {
        _intake->setState(IntakeState::kIdle);
      } else {
        _intake->setState(IntakeState::kIntake);
      }
    }

    if (_codriver.GetLeftBumper()) {
      if (_intake->getState() == IntakeState::kEject) {
        _intake->setState(IntakeState::kIdle);
      } else {
        _intake->setState(IntakeState::kEject);
      }
    }

    if (_codriver.GetAButtonPressed()) {
      if (_intake->getState() == IntakeState::kPass) {
        _intake->setState(IntakeState::kIdle);
      } else {
        _intake->setState(IntakeState::kPass);
      }
    }
  }
}

IntakeAutoControl::IntakeAutoControl(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void IntakeAutoControl::OnTick(units::second_t dt) {}
