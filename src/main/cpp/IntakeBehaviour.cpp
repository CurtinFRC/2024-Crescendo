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
<<<<<<< HEAD
    if (_intake->getState() == IntakeState::kRaw) {
      _intake->setState(IntakeState::kIdle);
    } else {
=======
    if (_rawControl) {
      _rawControl = false;
      _intaking = false;
      _ejecting = false;
      _intake->setState(IntakeState::kIdle);
    } else {
      _rawControl = true;
      _intaking = false;
      _ejecting = false;
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
      _intake->setState(IntakeState::kRaw);
    }
  }

<<<<<<< HEAD
  if (_intake->getState() == IntakeState::kRaw) {
    if (_codriver.GetRightBumper()) {
      _intake->setRaw(8_V);
    } else if (_codriver.GetLeftBumper()) {
      _intake->setRaw(-8_V);
=======
  if (_rawControl) {
    if (_codriver.GetRightTriggerAxis() > 0.1) {
      _intake->setRaw(_codriver.GetRightTriggerAxis() * 10_V);
    } else if (_codriver.GetLeftTriggerAxis() > 0.1) {
      _intake->setRaw(_codriver.GetLeftTriggerAxis() * -10_V);
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
    } else {
      _intake->setRaw(0_V);
    }
    _intake->setState(IntakeState::kRaw);

  } else {
<<<<<<< HEAD
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
=======
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
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
      }
    }

    if (_codriver.GetAButtonPressed()) {
<<<<<<< HEAD
      if (_intake->getState() == IntakeState::kPass) {
        _intake->setState(IntakeState::kIdle);
      } else {
        _intake->setState(IntakeState::kPass);
      }
    }
=======
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
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
  }
}

IntakeAutoControl::IntakeAutoControl(Intake* intake) : _intake(intake) {
  Controls(intake);
}

<<<<<<< HEAD
void IntakeAutoControl::OnTick(units::second_t dt) {}
=======
void IntakeAutoControl::OnTick(units::second_t dt) {}
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
