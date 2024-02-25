// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Intake.h"

Intake::Intake(IntakeConfig config) : _config(config), _pid(config.path + "/pid", config.pidConfig) {}

IntakeConfig Intake::GetConfig() {
  return _config;
}

void Intake::OnStart() {
  _pid.Reset();
}

void Intake::OnUpdate(units::second_t dt) {

  switch (_state) {
    case IntakeState::kIdle: 
    {
      if (_config.intakeSensor->Get() == false) { 
        SetState(IntakeState::kHold);
      }
      _stringStateName = "Idle";
      _pid.Reset();
      _setVoltage = 0_V;
    } 
    break;

    case IntakeState::kRaw: 
    {
      _stringStateName = "Raw";
      _pid.Reset();
      _setVoltage = _rawVoltage;
    } 
    break;

    case IntakeState::kEject: 
    {
      _stringStateName = "Eject";
      _setVoltage = 7_V;
      _pid.Reset();
      if (_config.intakeSensor->Get() == true) {
        SetState(IntakeState::kIdle); 
      }
    } 
    break;

    case IntakeState::kHold: 
    {
      _stringStateName = "Hold";

      if (_config.intakeSensor->Get() == true) {
        SetState(IntakeState::kIdle);
      }
      
      _pid.SetSetpoint(0_rad_per_s);
      units::volt_t pidCalculate =
          units::volt_t{_pid.Calculate(_config.IntakeGearbox.encoder->GetEncoderAngularVelocity(), dt, 0_V)};
      _setVoltage = pidCalculate;
    }
    break;

    case IntakeState::kIntake: 
    {
      _stringStateName = "Intake";
      _setVoltage = -7_V; 
      if (_config.intakeSensor->Get() == false) {

        SetState(IntakeState::kHold);
      }
    } 
    break;

    case IntakeState::kPass: 
    {
      _stringStateName = "Pass";
      _setVoltage = -7_V;
      if (_config.intakeSensor->Get() == true) {
        SetState(IntakeState::kIdle);
      }
    }
    break;
  }
  _table->GetEntry("State: ").SetString(_stringStateName);
  _table->GetEntry("Motor Voltage: ").SetDouble(_setVoltage.value());
  _table->GetEntry("Intake Sensor: ").SetBoolean(_config.intakeSensor->Get());
  _table->GetEntry("Error: ").SetDouble(_pid.GetError().value());
  _table->GetEntry("SetPoint: ").SetDouble(_pid.GetSetpoint().value());
  _table->GetEntry("Encoder: ").SetDouble(_config.IntakeGearbox.encoder->GetEncoderAngularVelocity().value());
 
  _config.IntakeGearbox.motorController->SetVoltage(_setVoltage);
}

void Intake::SetState(IntakeState state) {
  _state = state;
}
void Intake::SetRaw(units::volt_t voltage) {
  _rawVoltage = voltage;
}
IntakeState Intake::GetState() {
  return _state;
}
