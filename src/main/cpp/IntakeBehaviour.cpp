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
  if (_codriver.GetBButtonPressed()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }
  if (_codriver.GetRightTriggerAxis() > 0.1) {
    _intake->setRaw(_codriver.GetRightTriggerAxis() * 10_V);
  } else if (_codriver.GetLeftTriggerAxis() > 0.1) {
    _intake->setRaw(_codriver.GetLeftTriggerAxis() * -10_V);
  } else if (_rawVoltage != 0_V) {
    if (_intake->GetConfig().intakeSensor->Get() == 1) {
      _intake->setRaw(0_V);
    } else {
      _intake->setRaw(0_V);
    } 
  } else {
    _intake->setRaw(0_V);
  }
    _intake->setState(IntakeState::kRaw);
}

  // } else {
  //   if (_codriver.GetYButtonPressed()) {
  //     _intake->setState(IntakeState::kIntake);
  //   }
  //   if (_codriver.GetAButtonPressed()) {
  //     _intake->setState(IntakeState::kPass);
  //   }
  // }

// IntakeAutoControl::IntakeAutoControl(Intake* intake) : _intake(intake) {}

void IntakeAutoControl::OnTick(units::second_t dt) {
  if (_intake->GetConfig().intakeSensor->Get() == 1) {
    _intake->setState(IntakeState::kPass);
  } /*else if (_intake->GetConfig().magSensor->Get() == 0) {
    _intake->setState(IntakeState::kIdle);
  }*/
}
