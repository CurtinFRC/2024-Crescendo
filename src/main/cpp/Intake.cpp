// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Intake.h"

#include <algorithm>

#include "frc/RobotController.h"
#include "units/base.h"

Intake::Intake(IntakeConfig config)
    : _config(config),
      _pid(frc::PIDController(0.02, 0, 0, 0.05_s)),
      _pidPosition(frc::PIDController(1, 0, 0, 0.05_s)) {}

IntakeConfig Intake::GetConfig() {
  return _config;
}

void Intake::OnStart() {
  _pid.Reset();

  _timer.Start();
}

void Intake::OnUpdate(units::second_t dt) {
  switch (_state) {
    case IntakeState::kIdle: {
      if (_config.intakeSensor->Get() == false) {
        SetState(IntakeState::kHold);
      }
      _stringStateName = "Idle";
      _pid.Reset();
      _setVoltage = 0_V;
      _recordNote = false;
      hasValue = false;
    } break;

    case IntakeState::kRaw: {
      _stringStateName = "Raw";
      _pid.Reset();
      _setVoltage = _rawVoltage;
    } break;

    case IntakeState::kEject: {
      if (_config.intakeSensor->Get() == true && _config.passSensor->Get() == true) {
        SetState(IntakeState::kIdle);
      }
      _stringStateName = "Eject";
      _setVoltage = 8_V;
      _pid.Reset();
    } break;

    case IntakeState::kHold: {
      std::cerr << "HEY FROM kHold" << std::endl;
      units::volt_t pidCalculate = 0_V;
      if (_config.intakeSensor->Get() == true && _config.passSensor->Get() == true) {
        SetState(IntakeState::kIdle);
      }
      // units::volt_t pidCalculate =
      //     units::volt_t{_pid.Calculate(_config.IntakeGearbox.encoder->GetEncoderAngularVelocity().value())};
      if (_config.IntakeGearbox.encoder->GetEncoderPosition().value() < 0) {
        pidCalculate = units::volt_t{
            -_pidPosition.Calculate(-_config.IntakeGearbox.encoder->GetEncoderPosition().value())};
      } else {
        pidCalculate = units::volt_t{
            _pidPosition.Calculate(_config.IntakeGearbox.encoder->GetEncoderPosition().value())};
      }

      _setVoltage = pidCalculate;
      _stringStateName = "Hold";
    } break;

    case IntakeState::kIntake: {
      if (_config.intakeSensor->Get() == false) {
        if (_config.IntakeGearbox.encoder->GetEncoderPosition().value() < 0) {
          _pidPosition.SetSetpoint((-_config.IntakeGearbox.encoder->GetEncoderPosition().value()) - 0.5);
        } else {
          _pidPosition.SetSetpoint(_config.IntakeGearbox.encoder->GetEncoderPosition().value() + 0.5);
        }

        SetState(IntakeState::kHold);
      }
      _stringStateName = "Intake";
      _setVoltage = -10_V;
    } break;

    case IntakeState::kPass: {
      if (_config.intakeSensor->Get() == true) {
        SetState(IntakeState::kIdle);
      }

      if (!_recordNote) {
        _noteShot++;
        _recordNote = true;
      }

      _stringStateName = "Pass";
      _setVoltage = -10_V;
    } break;
  }
  _table->GetEntry("State").SetString(_stringStateName);
  _table->GetEntry("Motor Voltage").SetDouble(_setVoltage.value());
  _table->GetEntry("Intake Sensor").SetBoolean(_config.intakeSensor->Get());
  _table->GetEntry("Pass Sensor").SetBoolean(_config.passSensor->Get());
  _table->GetEntry("Error").SetDouble(_pidPosition.GetPositionError());
  _table->GetEntry("SetPoint").SetDouble(_pidPosition.GetSetpoint());
  _table->GetEntry("Encoder").SetDouble(_config.IntakeGearbox.encoder->GetEncoderPosition().value());
  _table->GetEntry("Shot Count").SetDouble(_noteShot);
  // _table->GetEntry("Encoder: ").SetDouble(_config.IntakeGearbox.encoder->GetEncoderPosition().value());
  //
  if (_timer.Get() < 4_s) {
    _setVoltage = units::math::min(0_V, _setVoltage);
  } else {
    _timer.Stop();
    _timer.Reset();
    // _timer.Restart();
    // _timer.Start();
  }

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
