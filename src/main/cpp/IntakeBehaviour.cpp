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
      _rawControl = false;
      _intaking = false;
      _ejecting = false;
      _intake->setState(IntakeState::kIdle);
    } else {
      _rawControl = true;
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
      }
    }

    if (_codriver.GetLeftTriggerAxis() > 0.1) {
      if (_ejecting) {
        _ejecting = false;
      } else {
        _ejecting = true;
        _intaking = false;
      }
    }

    if (_intaking) {
      if (_intake->getState() == IntakeState::kHold || _intake->getState() == IntakeState::kPass) {
        if (_intake->GetConfig().intakeSensor->Get() == 0) {
          _intake->setState(IntakeState::kIdle);
          _intaking = false;
        } else {
          _intake->setState(IntakeState::kPass);
        }
      } else {
        if (_intake->GetConfig().intakeSensor->Get() == 1) {
          _intake->setState(IntakeState::kHold);
          _intaking = false;
        } else {
          _intake->setState(IntakeState::kIntake);
        }
      }
    }

    if (_ejecting) {
      if (_intake->GetConfig().intakeSensor->Get() == 0) {
        _intake->setState(IntakeState::kIdle);
        _ejecting = false;
      } else {
        _intake->setState(IntakeState::kEject);
      }
    }
  }
}

IntakeAutoControl::IntakeAutoControl(Intake* intake) : _intake(intake) {
  Controls(intake);
}

void IntakeAutoControl::OnTick(units::second_t dt) {}

  // } else {
  //   if (_setVoltage != 0_V) { // If the intake wasn't running last tick,
  //     if (_setVoltage < 0_V && _intake->GetConfig().intakeSensor->Get() == 0) { // If the intake was intaking and there is a note in the intake,
  //       _intake->setState(IntakeState::kHold); // Hold that note.
  //     } else if (_setVoltage > 0_V && _intake->GetConfig().intakeSensor->Get() == 1) { // Else if the intake was ejecting and there is noting in the intake,
  //       _intake->setState(IntakeState::kIdle); // Set the intake to idle mode.
  //     } 

  //   } else {
  //     if (_intake->GetConfig().intakeSensor->Get() == 0) { // If there is a note in the intake.
  //       if (_codriver.GetRightTriggerAxis() > 0.1) { // If the right trigger is pressed,
  //         _intake->setState(IntakeState::kPass); // Start passing the note into the shooter.
  //       } else { // Otherwise,
  //         _intake->setState(IntakeState::kHold); // Keep holding the note.
  //       }
  //     } else if (_intake->GetConfig().intakeSensor->Get() == 1) { // If there is nothing in the intake,
  //       if (_codriver.GetRightTriggerAxis() > 0.1) { // If the right trigger is pressed,
  //         _intake->setState(IntakeState::kIntake); // Start intaking.
  //       } else { // Otherwise,
  //         _intake->setState(IntakeState::kIdle); // Continue to go Idle.
  //       }
  //     } else if (_codriver.GetLeftTriggerAxis() > 0.1) { // If the left trigger is pressed,
  //       _intake->setState(IntakeState::kEject); // Eject the note.
  //     }
  //   }
