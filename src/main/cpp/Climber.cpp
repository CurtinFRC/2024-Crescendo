// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Climber.h"

Climber::Climber(ClimberConfig config) : _config(config), _pid(frc::PIDController(0.35, 0, 0, 0.05_s)) {}

void Climber::OnUpdate(units::second_t dt) {
  switch (_state) {
    case ClimberState::kIdle: {
      _stringStateName = "Idle";
      _setVoltage = 0_V;
    } break;

    case ClimberState::kRatchet: {
      _stringStateName = "Ratchet";
      _setVoltage = 0_V;
      // armServo.SetAngle(0);
    } break;

    case ClimberState::kArmUp: {
      _stringStateName = "Arm Up";
      // _setVoltage = 8_V;

      _pid.SetSetpoint(0.035 * 100);
      _setVoltage = -units::volt_t{
          _pid.Calculate((-_config.climberGearbox.encoder->GetEncoderPosition().value()) * 100)};
  #ifdef DEBUG
      _table->GetEntry("PID OUTPUT")
          .SetDouble(_pid.Calculate((-_config.climberGearbox.encoder->GetEncoderPosition().value()) * 100));
  #endif
    } break;

    case ClimberState::kArmDown: {
      _stringStateName = "Arm Down";
      _pid.SetSetpoint(0.07 * 100);
      _setVoltage = -units::volt_t{
          _pid.Calculate((-_config.climberGearbox.encoder->GetEncoderPosition().value()) * 100)};
      // armServo.SetAngle(180);
      // _setVoltage = -8_V;
    } break;

    case ClimberState::kMatch: {
      _stringStateName = "Match";
      _pid.SetSetpoint(0.005 * 100);
      _setVoltage = -units::volt_t{
          _pid.Calculate((-_config.climberGearbox.encoder->GetEncoderPosition().value()) * 100)};

      // _pid.SetSetpoint()
      // _setVoltage = -8_V;
    } break;

    case ClimberState::kRaw: {
      _stringStateName = "Raw";
      _setVoltage = _rawVoltage;
    } break;

    default:
      std::cout << "Error climber in invalid state" << std::endl;
      break;
  }
  _config.climberGearbox.motorController->SetVoltage(_setVoltage);

  #ifdef DEBUG
  _table->GetEntry("State: ").SetString(_stringStateName);
  _table->GetEntry("Motor Voltage: ").SetDouble(_setVoltage.value());
  _table->GetEntry("Encoder Output: ")
      .SetDouble(_config.climberGearbox.encoder->GetEncoderPosition().value() * 100);
  #endif
}

void Climber::SetState(ClimberState state) {
  _state = state;
}

void Climber::SetRaw(units::volt_t voltage) {
  _rawVoltage = voltage;
}

ClimberState Climber::GetState() {
  return _state;
}
