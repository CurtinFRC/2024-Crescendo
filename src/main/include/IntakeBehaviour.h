// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "Intake.h"
#include "LED.h"
#include "Wombat.h"

class IntakeManualControl : public behaviour::Behaviour {
 public:
  explicit IntakeManualControl(Intake* intake, frc::XboxController& codriver, LED* led);

  void OnTick(units::second_t dt) override;

 private:
  Intake* _intake;
  frc::XboxController& _codriver;
  LED* _led;

  units::volt_t _rawVoltage;
  units::volt_t _setVoltage;
};

class IntakeAutoControl : public behaviour::Behaviour {
 public:
  explicit IntakeAutoControl(Intake* intake);

  void OnTick(units::second_t dt) override;

 private:
  Intake* _intake;

  LED* _led;
};
