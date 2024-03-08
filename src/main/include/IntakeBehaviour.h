// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "AlphaArm.h"
#include "Intake.h"
#include "Wombat.h"

class IntakeManualControl : public behaviour::Behaviour {
 public:
  explicit IntakeManualControl(Intake* intake, AlphaArm* arm, frc::XboxController& codriver);

  void OnTick(units::second_t dt) override;

 private:
  Intake* _intake;
  AlphaArm* _arm;
  frc::XboxController& _codriver;
  bool _rawControl = false;
};

class AutoIntake : public behaviour::Behaviour {
 public:
  explicit AutoIntake(Intake* intake);
  void OnTick(units::second_t dt) override;

 private:
  Intake* _intake;
};

class IntakeNote : public behaviour::Behaviour {
 public:
  explicit IntakeNote(Intake* intake);
  void OnTick(units::second_t dt) override;
  void OnStop() override;
  void OnStart() override;

 private:
  Intake* _intake;
  frc::Timer _timer;
};

class PassNote : public behaviour::Behaviour {
 public:
  explicit PassNote(Intake* intake);
  void OnTick(units::second_t dt) override;

 private:
  Intake* _intake;
};

class EjectNote : public behaviour::Behaviour {
 public:
  explicit EjectNote(Intake* intake);
  void OnTick(units::second_t dt) override;

 private:
  Intake* _intake;
};
