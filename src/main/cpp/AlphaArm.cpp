// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArm.h"

AlphaArm::AlphaArm(AlphaArmConfig* config /*, frc::Rotation2d initialAngle, wom::vision::Limelight* vision */)
    : _config(config),
      _pidArm{frc::PIDController(1.2, 0.4, 0)},
      _pidArmStates{frc::PIDController(37, 0.00070, 0.15)},
      _pidIntakeState{frc::PIDController(30, 0.00015, 0.005)},
      _pidClimberStates{frc::PIDController(25, 0.00015, 0.005)} {}

void AlphaArm::OnStart() {
  _pidArmStates.Reset();
  _pidIntakeState.Reset();
}

void AlphaArm::OnUpdate(units::second_t dt) {
  switch (_state) {
    case AlphaArmState::kIdle:

      _setAlphaArmVoltage = 0_V;
      break;
    case AlphaArmState::kRaw:
      #ifdef DEBUG
      std::cout << "RawControl" << std::endl;
      #endif
      _setAlphaArmVoltage = _rawArmVoltage;

      break;

    case AlphaArmState::kHoldAngle: {
      _pidIntakeState.SetSetpoint(-_goal);
      //_setAlphaArmVoltage =
      //_pidArm.Calculate(_alphaArm->GetConfig().config->alphaArmEncoder.GetEncoderPosition());
      _setAlphaArmVoltage = units::volt_t{
          _pidIntakeState.Calculate(-(_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
    } break;
    case AlphaArmState::kAmpAngle:
      #ifdef DEBUG
      std::cout << "Amp Angle" << std::endl;
      #endif

      _pidArmStates.SetSetpoint(-2.12);
      _setAlphaArmVoltage = units::volt_t{
          _pidArmStates.Calculate(-(_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};

      break;

    case AlphaArmState::kIntakeAngle:
      #ifdef DEBUG
      std::cout << "Intake Angle" << std::endl;
      #endif
      _pidIntakeState.SetSetpoint(-0.50);  //-0.48
      _setAlphaArmVoltage = units::volt_t{
          _pidIntakeState.Calculate(-(_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;
    case AlphaArmState::kIntakedAngle:
      #ifdef DEBUG
      std::cout << "Intake Angle" << std::endl;
      #endif
      _pidIntakeState.SetSetpoint(-0.55);  //-0.48
      _setAlphaArmVoltage = units::volt_t{
          _pidIntakeState.Calculate(-(_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;
    case AlphaArmState::kClimbAngle:
      #ifdef DEBUG
      std::cout << "Climb Angle" << std::endl;
      #endif
      _pidArmStates.SetSetpoint(-2.00);  //-0.48
      _setAlphaArmVoltage = units::volt_t{
          _pidArmStates.Calculate(-(_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;

    case AlphaArmState::kClimbed:
      #ifdef DEBUG
      std::cout << "Climb Angle" << std::endl;
      #endif
      _pidClimberStates.SetSetpoint(-0.5);  //-0.48
      _setAlphaArmVoltage = units::volt_t{_pidClimberStates.Calculate(
          -(_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
      break;

    case AlphaArmState::kSpeakerAngle:
      #ifdef DEBUG
      std::cout << "Speaker Angle" << std::endl;
      #endif
      _pidIntakeState.SetSetpoint(-0.82);
      _setAlphaArmVoltage = units::volt_t{
          _pidIntakeState.Calculate(-(_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};

      break;

      // case AlphaArmState::kVisionAngle:
      // std::cout << "Vision Angle" << std::endl;

      // break;

    case AlphaArmState::kStowed:
      #ifdef DEBUG
      std::cout << "Shuttling angle" << std::endl;
      #endif
      _pidIntakeState.SetSetpoint(-1.20);
      _setAlphaArmVoltage = units::volt_t{
          _pidIntakeState.Calculate(-(_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)))};
    default:
      std::cout << "Error: alphaArm in INVALID STATE" << std::endl;
      break;
  }
  _config->alphaArmGearbox.motorController->SetVoltage(_setAlphaArmVoltage);
  _config->alphaArmGearbox2.motorController->SetVoltage(_setAlphaArmVoltage);
  #ifdef DEBUG
  std::cout << "Encoder Value: " << (_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415))
            << std::endl;
  #endif
  _table->GetEntry("PID Error").SetDouble(_pidArm.GetPositionError());
  _table->GetEntry("SetPoint").SetDouble(_pidArm.GetSetpoint());
  _table->GetEntry("Input").SetDouble((_config->alphaArmEncoder.GetEncoderPosition().value() * (2 * 3.1415)));

  _table->GetEntry("PID Error State").SetDouble(_pidArmStates.GetPositionError());
  _table->GetEntry("SetPoint State").SetDouble(_pidArmStates.GetSetpoint());

  _table->GetEntry("Intake SetPoint State").SetDouble(_pidIntakeState.GetSetpoint());
  _table->GetEntry("Intake PID Error State").SetDouble(_pidIntakeState.GetPositionError());

  #ifdef DEBUG
  std::cout << "Voltage:" << _setAlphaArmVoltage.value() << std::endl;
  #endif
}

void AlphaArm::SetState(AlphaArmState state) {
  _state = state;
}

void AlphaArm::SetArmRaw(units::volt_t voltage) {
  _rawArmVoltage = voltage;
}

void AlphaArm::SetGoal(double goal) {
  _goal = goal;
}

void AlphaArm::SetControllerRaw(units::volt_t voltage) {
  _controlledRawVoltage = voltage;
}
