// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArm.h"

AlphaArm::AlphaArm(AlphaArmConfig config) : _config(config) {}

AlphaArmConfig AlphaArm::GetConfig() {
  return _config;
}

void AlphaArm::OnUpdate(units::second_t dt) {
  switch (_state) {
    case AlphaArmState::kIdle:

      // _config.alphaArmGearbox.motorController->SetVoltage(0_V);
      // _config.wristGearbox.motorController->SetVoltage(0_V);
      _setAlphaArmVoltage = 0_V;
      _setWristVoltage = 0_V;

      break;
    case AlphaArmState::kRaw:
      _setAlphaArmVoltage = _rawArmVoltage;
      _setWristVoltage = _rawWristVoltage;
      _config.alphaArmGearbox.motorController->SetVoltage(_rawArmVoltage);
      _config.wristGearbox.motorController->SetVoltage(_rawWristVoltage);

      break;
    case AlphaArmState::kForwardWrist:
      _config.wristGearbox.motorController->SetVoltage(3_V);
      _setWristVoltage = 3_V;

    case AlphaArmState::kReverseWrist:
      _config.wristGearbox.motorController->SetVoltage(-3_V);
      _setWristVoltage = -3_V;
    default:
      std::cout << "oops, wrong state" << std::endl;
      break;
  }
  // transmission translate
  // _config.armGearBox.motorController->SetVoltage(setAlphaArmVoltage);
  // _config.alphaArmGearbox.motorController->SetVoltage(setAlphaArmVoltage);
  // _config.wristGearbox.motorController->SetVoltage(setWristVoltage);
  _config.alphaArmGearbox.motorController->SetVoltage(_setAlphaArmVoltage);
  _config.wristGearbox.motorController->SetVoltage(_setWristVoltage);

  // _config.wristGearbox.motorController->SetVoltage(_setVoltage);
}

void AlphaArm::SetState(AlphaArmState state) {
  _state = state;
}

// void AlphaArm::SetRaw(units::volt_t voltage){
//     _rawArmVoltage = voltage;
//     _rawWristVoltage = voltage;
// }

void AlphaArm::SetArmRaw(units::volt_t voltage) {
  _rawArmVoltage = voltage;
}

void AlphaArm::setWristRaw(units::volt_t voltage) {
  _rawWristVoltage = voltage;
}