// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "Intake.h"
#include "Wombat.h"

class IntakeManualControl : public behaviour::Behaviour {
 public:
  explicit IntakeManualControl(Intake* intake, frc::XboxController& codriver);

  void OnTick(units::second_t dt) override;

 private:
  Intake* _intake;
  frc::XboxController& _codriver;

  units::volt_t _rawVoltage;
  units::volt_t _setVoltage;
<<<<<<< HEAD
=======
  bool _rawControl = true;
  bool _intaking = false;
  bool _ejecting = false;
  bool _passing = false;
>>>>>>> c30477a (Intake - Manual/Auto fixes (#114))
};

class IntakeAutoControl : public behaviour::Behaviour {
 public:
  explicit IntakeAutoControl(Intake* intake);

  void OnTick(units::second_t dt) override;

 private:
  Intake* _intake;
};