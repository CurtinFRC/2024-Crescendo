// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Shooter.h"

Shooter::Shooter(ShooterConfig config) : _config(config), _pid(frc::PIDController(0.02, 0.5, 0, 0.05_s)) {}

void Shooter::OnStart() {
  _pid.Reset();
}

void Shooter::OnUpdate(units::second_t dt) {
  #ifdef DEBUG
  table->GetEntry("Error").SetDouble(_pid.GetPositionError());
  table->GetEntry("SetPoint").SetDouble(_pid.GetSetpoint());
  table->GetEntry("Encoder Output")
      .SetDouble(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value());
  table->GetEntry("Shooting").SetString(_statename);
  // table->GetEntry("PID CONTROLLER").Set(_pid);
#endif

  switch (_state) {
    case ShooterState::kIdle: {
      _statename = "Idle";
      _pid.Reset();
      holdVoltage = 0_V;
      #ifdef DEBUG
      std::cout << "KIdle" << std::endl;
      #endif
      _setVoltage = 0_V;
      // if (_shooterSensor.Get()) {
      //   _state = ShooterState::kReverse;
      // }
    } break;
    case ShooterState::kSpinUp: {
      _statename = "SpinUp";
      #ifdef DEBUG
      std::cout << "KSpinUp" << std::endl;
      #endif
      _pid.SetSetpoint(_goal.value());
      units::volt_t pidCalculate =
          units::volt_t{_pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value())};
      #ifdef DEBUG
      std::cout << "KShooting" << std::endl;
      #endif

      _setVoltage = pidCalculate;

      // if (_pid.GetPositionError() < 1 && _pid.GetVelocityError() < 1) {
      //   holdVoltage = pidCalculate;
      //   std::cout << "STABLE" << std::endl;
      //   _state = ShooterState::kShooting;
      // }

      // if (holdVoltage.value() == 0) {
      //   _setVoltage = pidCalculate;
      // } else {
      //   _setVoltage = holdVoltage;
      // }
    } break;
    case ShooterState::kShooting: {
      _statename = "Shooting";

      _pid.SetSetpoint(_goal.value());
      units::volt_t pidCalculate =
          units::volt_t{_pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity().value())};
      #ifdef DEBUG
      std::cout << "KShooting" << std::endl;
      #endif

      if (_pid.GetPositionError() < 1 && _pid.GetVelocityError() < 1) {
        holdVoltage = pidCalculate;
        #ifdef DEBUG
        std::cout << "STABLE" << std::endl;
        #endif
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
      _setVoltage = -8_V;
      #ifdef DEBUG
      std::cout << "KReverse" << std::endl;
      #endif
      // if (!_shooterSensor.Get()) {
      //   SetState(ShooterState::kIdle);
      // }
    } break;
    case ShooterState::kRaw: {
      _statename = "Raw";
      holdVoltage = 0_V;
      _pid.Reset();
      _setVoltage = _rawVoltage;
      #ifdef DEBUG
      std::cout << "KRaw" << std::endl;
      #endif
      // if (_shooterSensor.Get()) {
      //   SetState(ShooterState::kRaw);
      // }
    } break;
    default: {
      std::cout << "Error shooter in invalid state" << std::endl;
    } break;
  }
  // table->GetEntry("Motor OutPut").SetDouble(_setVoltage.value());

  _config.ShooterGearbox.motorController->SetVoltage(_setVoltage);
}

//       // if (_pid.AtSetpoint()) {
//       //   SetState(ShooterState::kShooting);
//       // }
//       // table->GetEntry("PID Setpoint:").SetDouble(_pid.GetSetpoint());
//       std::cout << "KShooting" << std::endl;
//       _pid.SetSetpoint(20);
//       // _pid.SetSetpoint(_goal.value());

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
