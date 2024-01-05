// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "subsystems/behaviours/ShooterBehaviours.h"

// Shooter Manual Set

wom::subsystems::behaviours::ShooterConstant::ShooterConstant(Shooter *s, units::volt_t setpoint)
    : _shooter(s), _setpoint(setpoint) {
  Controls(_shooter);
}

void wom::subsystems::behaviours::ShooterConstant::OnTick(units::second_t dt) {
  _shooter->SetManual(_setpoint);
}

// ShooterSpinup

wom::subsystems::behaviours::ShooterSpinup::ShooterSpinup(Shooter *s, units::radians_per_second_t speed,
                                                          bool hold)
    : _shooter(s), _speed(speed), _hold(hold) {
  Controls(_shooter);
}

void wom::subsystems::behaviours::ShooterSpinup::OnTick(units::second_t dt) {
  _shooter->SetPID(_speed);

  if (!_hold && _shooter->IsStable()) SetDone();
}
