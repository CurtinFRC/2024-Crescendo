// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "subsystems/Shooter.h"

#include <networktables/NetworkTableInstance.h>

wom::subsystems::Shooter::Shooter(std::string path, ShooterParams params)
    : _params(params),
      _state(ShooterState::kIdle),
      _pid{path + "/pid", params.pid},
      _table(nt::NetworkTableInstance::GetDefault().GetTable("shooter")) {}

void wom::subsystems::Shooter::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};
  units::revolutions_per_minute_t currentSpeed =
      _params.gearbox.encoder->GetEncoderAngularVelocity();

  switch (_state) {
    case ShooterState::kManual:
      voltage = _setpointManual;
      break;
    case ShooterState::kPID: {
      auto feedforward =
          _params.gearbox.motor.Voltage(0_Nm, _pid.GetSetpoint());
      voltage = _pid.Calculate(currentSpeed, dt, feedforward);
    } break;
    case ShooterState::kIdle:
      voltage = 0_V;
      break;
  }

  units::newton_meter_t max_torque_at_current_limit =
      _params.gearbox.motor.Torque(_params.currentLimit);
  units::volt_t max_voltage_for_current_limit =
      _params.gearbox.motor.Voltage(max_torque_at_current_limit, currentSpeed);

  voltage =
      1_V * std::min(voltage.value(), max_voltage_for_current_limit.value());

  // _params.gearbox.motorController->SetVoltage(voltage);

  _table->GetEntry("output_volts").SetDouble(voltage.value());
  _table->GetEntry("speed_rpm").SetDouble(currentSpeed.value());
  _table->GetEntry("setpoint_rpm")
      .SetDouble(units::revolutions_per_minute_t{_pid.GetSetpoint()}.value());
  _table->GetEntry("stable").SetBoolean(_pid.IsStable());
}

void wom::subsystems::Shooter::SetManual(units::volt_t voltage) {
  _setpointManual = voltage;
}

void wom::subsystems::Shooter::SetPID(units::radians_per_second_t goal) {
  _state = ShooterState::kPID;
  _pid.SetSetpoint(goal);
}

void wom::subsystems::Shooter::SetState(ShooterState state) {
  _state = state;
}

bool wom::subsystems::Shooter::IsStable() const {
  return _pid.IsStable();
}

// Shooter Manual Set

wom::subsystems::ShooterConstant::ShooterConstant(Shooter* s,
                                                  units::volt_t setpoint)
    : _shooter(s), _setpoint(setpoint) {
  Controls(_shooter);
}

void wom::subsystems::ShooterConstant::OnTick(units::second_t dt) {
  _shooter->SetManual(_setpoint);
}

// ShooterSpinup

wom::subsystems::ShooterSpinup::ShooterSpinup(Shooter* s,
                                              units::radians_per_second_t speed,
                                              bool hold)
    : _shooter(s), _speed(speed), _hold(hold) {
  Controls(_shooter);
}

void wom::subsystems::ShooterSpinup::OnTick(units::second_t dt) {
  _shooter->SetPID(_speed);

  if (!_hold && _shooter->IsStable())
    SetDone();
}
