// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Shooter.h"

<<<<<<< HEAD
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
=======
Shooter::Shooter(ShooterConfig config) : _config(config), _pid(config.path + "/pid", config.pidConfig) {}

void Shooter::OnStart() {
  _pid.Reset();
}

void Shooter::OnUpdate(units::second_t dt) {
  // _pid.SetTolerance(0.5, 4);
  table->GetEntry("Error").SetDouble(_pid.GetError().value());
  // table->GetEntry("Acceleration Error").SetDouble(_pid.GetVelocityError());
  table->GetEntry("SetPoint").SetDouble(_pid.GetSetpoint().value());
  // table->GetEntry("Current
  // Pos").SetDouble(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value());
  // table->GetEntry("EncoderValue").SetDouble(_config.ShooterGearbox.encoder->GetVelocityValue());
  table->GetEntry("Shooting").SetString(_statename);
  switch (_state) {
    case ShooterState::kIdle: {
      _statename = "Idle";
      _pid.Reset();
      holdVoltage = 0_V;
      std::cout << "KIdle" << std::endl;
      _setVoltage = 0_V;
      // if (_shooterSensor.Get()) {
      //   _state = ShooterState::kReverse;
      // }
    } break;
    case ShooterState::kSpinUp: {
      _statename = "SpinUp";
      std::cout << "KSpinUp" << std::endl;
      _pid.SetSetpoint(_goal);
      units::volt_t pidCalculate =
          units::volt_t{_pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity(), dt, 0_V)};
      std::cout << "KShooting" << std::endl;

      if (_pid.IsStable()) {
        holdVoltage = pidCalculate;
        std::cout << "STABLE" << std::endl;
        _state = ShooterState::kShooting;
      }

      if (holdVoltage.value() == 0) {
        _setVoltage = pidCalculate;
      } else {
        _setVoltage = holdVoltage;
      }
    } break;
    case ShooterState::kShooting: {
      _statename = "Shooting";

      _pid.SetSetpoint(_goal);
      units::volt_t pidCalculate =
          units::volt_t{_pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity(), dt, 0_V)};
      std::cout << "KShooting" << std::endl;

      if (_pid.IsStable()) {
        holdVoltage = pidCalculate;
        std::cout << "STABLE" << std::endl;
      }

      if (holdVoltage.value() == 0) {
        _setVoltage = pidCalculate;
      } else {
        _setVoltage = holdVoltage;
      }
    } break;

    case ShooterState::kReverse: {
      _statename = "Reverse";
      _pid.Reset();
      _setVoltage = -5_V;
      std::cout << "KReverse" << std::endl;
      // if (!_shooterSensor.Get()) {
      //   SetState(ShooterState::kIdle);
      // }
    } break;
    case ShooterState::kRaw: {
      _statename = "Raw";
      holdVoltage = 0_V;
      _pid.Reset();
      _setVoltage = _rawVoltage;
      std::cout << "KRaw" << std::endl;
      // if (_shooterSensor.Get()) {
      //   SetState(ShooterState::kRaw);
      // }
    } break;
    default: {
      std::cout << "Error shooter in invalid state" << std::endl;
    } break;
  }
  std::cout << "Voltage:" << _setVoltage.value() << std::endl;
  _config.ShooterGearbox.motorController->SetVoltage(_setVoltage);
>>>>>>> 4fff1bfdbcf5470c72fe140843a59616c642c5c8
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
