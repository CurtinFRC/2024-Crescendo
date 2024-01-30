// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Intake.h"

Intake::Intake(IntakeConfig config) : _config(config) {}

IntakeConfig Intake::GetConfig() {
  return _config;
}

void Intake::OnUpdate(units::second_t dt) {
  switch (_state) {
    case IntakeState::kIdle: {
      if (_config.intakeSensor->Get() == false) {
        setState(IntakeState::kHold);
      }
      _stringStateName = "Idle";
      _setVoltage = 0_V;
    } break;

    case IntakeState::kRaw: {
      _stringStateName = "Raw";
      _setVoltage = _rawVoltage;
    } break;

    case IntakeState::kEject: {
      _stringStateName = "Eject";
      _setVoltage = 7_V;
      if (_config.intakeSensor->Get() == true) {
        setState(IntakeState::kIdle);
      }
    } break;

    case IntakeState::kHold: {
      _stringStateName = "Hold";
      _setVoltage = 0_V;
    } break;

    case IntakeState::kIntake: {
      _stringStateName = "Intake";
      _setVoltage = -7_V;
      if (_config.intakeSensor->Get() == false) {
        setState(IntakeState::kHold);
      }
    } break;

    case IntakeState::kPass: {
      _stringStateName = "Pass";
      _setVoltage = -7_V;
      if (_config.intakeSensor->Get() == true) {
        setState(IntakeState::kIdle);
      }
    } break;
    default:
      std::cout << "Error: Intake in INVALID STATE." << std::endl;
      break;
  }
  _table->GetEntry("State: ").SetString(_stringStateName);
  _table->GetEntry("Motor Voltage: ").SetDouble(_setVoltage.value());
  _table->GetEntry("Intake Sensor: ").SetBoolean(_config.intakeSensor->Get());
  // _table->GetEntry("Shooter Sensor: ").SetBoolean(_config.shooterSensor->Get());
  // _table->GetEntry("Intake Sensor: ").SetBoolean(_config.intakeSensor->Get());
  // _table->GetEntry("Shooter Sensor: ").SetBoolean(_config.shooterSensor->Get());
  // _table->GetEntry("Magazine Sensor: ").SetBoolean(_config.magSensor->Get());

  std::cout << _setVoltage.value() << std::endl;

  _config.IntakeMotor.motorController->SetVoltage(_setVoltage);
}

void Intake::setState(IntakeState state) {
  _state = state;
}
void Intake::setRaw(units::volt_t voltage) {
  _rawVoltage = voltage;
}
IntakeState Intake::getState() {
  return _state;
}