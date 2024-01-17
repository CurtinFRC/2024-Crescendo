#pragma once

#include "Wombat.h"
#include <frc/XboxController.h>
#include "Intake.h"

class IntakeManualControl : public behaviour::Behaviour {
 public:
  IntakeManualControl(Intake *intake, frc::XboxController &codriver);

  void OnTick(units::second_t dt) override;
 private:
  Intake *_intake;
  frc::XboxController &_codriver;

  units::volt_t _rawVoltage;
  bool _rawControl;
}; 

class IntakeAutoControl : public behaviour::Behaviour {
 public:
  IntakeAutoControl(Intake *intake);

  void OnTick(units::second_t dt) override;
 private:
  Intake *_intake;
}; 