// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Shooter.h"

Shooter::Shooter(ShooterConfig config)
    : _config(config)
// ,
// _pid{frc::PIDController (1, 0, 0, 0.005_s)}
{}  // config.path + "/pid", config.pidConfig
void Shooter::OnUpdate(units::second_t dt) {
  // _pid.SetTolerance(0.1, 1);
  // switch (_state) {
  //   case ShooterState::kIdle: {
  //     std::cout << "KIdle" << std::endl;
  //     _setVoltage = 0_V;
  //     // if (_shooterSensor.Get()) {
  //     //   _state = ShooterState::kReverse;
  //     // }
  //   } break;
  //   case ShooterState::kSpinUp: {
  //     // std::cout << "KSpinUp" << std::endl;
  //     // _pid.SetSetpoint(_goal.value());
  //     // units::volt_t pidCalculate =
  //     //
  //     units::volt_t{_pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value())};
  //     // _setVoltage = pidCalculate;

  //     // if (_pid.AtSetpoint()) {
  //     //   SetState(ShooterState::kShooting);
  //     // }
  //   } break;
  //   case ShooterState::kShooting: {
  //     // std::cout << "KShooting" << std::endl;
  //     // _pid.SetSetpoint(_goal.value());
  //     // units::volt_t pidCalculate =
  //     //
  //     units::volt_t{_pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value())};
  //     // _setVoltage = pidCalculate;

  //     // if (!_pid.AtSetpoint()) {
  //     //   SetState(ShooterState::kSpinUp);
  //     // }
  //     // if (_shooterSensor.Get()) {
  //     //   SetState(ShooterState::kIdle);
  //     // }
  //   } break;

  //   case ShooterState::kReverse: {
  //     _setVoltage = -5_V;
  //     std::cout << "KReverse" << std::endl;
  //     // if (!_shooterSensor.Get()) {
  //     //   SetState(ShooterState::kIdle);
  //     // }
  //   } break;
  //   case ShooterState::kRaw: {
  //     _setVoltage = _rawVoltage;
  //     std::cout << "KRaw" << std::endl;
  //     // if (_shooterSensor.Get()) {
  //     //   SetState(ShooterState::kRaw);
  //     // }
  //   } break;
  //   default: {
  //     std::cout << "Error shooter in invalid state" << std::endl;
  //   } break;
  // }
  // std::cout << "Voltage:" << _setVoltage.value() << std::endl;
  // _config.ShooterGearbox.motorController->SetVoltage(_setVoltage);
}

void Shooter::SetState(ShooterState state) {
  _state = state;
}
void Shooter::SetRaw(units::volt_t voltage) {
  _rawVoltage = voltage;
  _state = ShooterState::kRaw;
}
void Shooter::SetPidGoal(units::radians_per_second_t goal) {
  _goal = goal;
}
