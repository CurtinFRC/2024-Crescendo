// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Intake.h"

Intake::Intake(IntakeConfig config) : _config(config), _pid(frc::PIDController (0.0125, 0, 0, 0.05_s)), _pidPosition(frc::PIDController (0.0125, 0, 0, 0.05_s)) {}

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
      _recordNote = false;
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
      if (_config.intakeSensor->Get() == true && _config.passSensor->Get() == true) {
        SetState(IntakeState::kIdle); 
      }
      _stringStateName = "Eject";
      _setVoltage = 4_V;
      _pid.Reset();
    } 
    break;

    case IntakeState::kHold: 
    {
      if (_config.intakeSensor->Get() == true && _config.passSensor->Get() == true) {
        SetState(IntakeState::kIdle);
      }
      // _pid.SetSetpoint(0);
      // units::volt_t pidCalculate =
          // units::volt_t{_pid.Calculate(_config.IntakeGearbox.encoder->GetEncoderAngularVelocity().value())};
      // units::volt_t pidCalculate =
          // units::volt_t{_pidPosition.Calculate(_config.IntakeGearbox.encoder->GetEncoderPosition().value())};
      _setVoltage = 0_V;
      _stringStateName = "Hold";
    }
    break;

    case IntakeState::kIntake: 
    {
      if (_config.intakeSensor->Get() == false) {
        SetState(IntakeState::kHold);
      }
      _stringStateName = "Intake";
      _setVoltage = -4_V; 
    } 
    break;

    case IntakeState::kPass: 
    {
      if (_config.intakeSensor->Get() == true) {
        SetState(IntakeState::kIdle);
      } 

      if (!_recordNote) {
        _noteShot ++;
        _recordNote = true;
      }

      _stringStateName = "Pass";
      _setVoltage = -4_V;
    }
    break;
  }
  _table->GetEntry("State: ").SetString(_stringStateName);
  _table->GetEntry("Motor Voltage: ").SetDouble(_setVoltage.value());
  _table->GetEntry("Intake Sensor: ").SetBoolean(_config.intakeSensor->Get());
  _table->GetEntry("Pass Sensor: ").SetBoolean(_config.passSensor->Get());
  _table->GetEntry("Error: ").SetDouble(_pid.GetPositionError());
  _table->GetEntry("SetPoint: ").SetDouble(_pid.GetSetpoint());
  _table->GetEntry("Encoder: ").SetDouble(_config.IntakeGearbox.encoder->GetEncoderAngularVelocity().value());
  _table->GetEntry("Shot Count: ").SetDouble(_noteShot);
  // _table->GetEntry("Encoder: ").SetDouble(_config.IntakeGearbox.encoder->GetEncoderPosition().value());
 
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
