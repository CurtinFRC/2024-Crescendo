// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "drivetrain/Drivetrain.h"

#include <iostream>

#include "utils/Util.h"

using namespace frc;
using namespace units;

wom::drivetrain::Drivetrain::Drivetrain(DrivetrainConfig* config,
                                        XboxController& driver)
    : _config(config), _driver(driver) {}
wom::drivetrain::Drivetrain::~Drivetrain() {}

wom::drivetrain::DrivetrainConfig* wom::drivetrain::Drivetrain::GetConfig() {
  return _config;
}
wom::drivetrain::DrivetrainState wom::drivetrain::Drivetrain::GetState() {
  return _state;
}

void wom::drivetrain::Drivetrain::SetState(DrivetrainState state) {
  _state = state;
}

void wom::drivetrain::Drivetrain::OnStart() {
  std::cout << "Starting Tank" << std::endl;
}

void wom::drivetrain::Drivetrain::OnUpdate(second_t dt) {
  switch (_state) {
    case DrivetrainState::kIdle:
      break;
    case DrivetrainState::kTank: {
      double rightSpeed = wom::utils::deadzone(_driver.GetRightY());
      double leftSpeed = wom::utils::deadzone(_driver.GetLeftY());
      // _config->left1.motorController->SetVoltage(leftSpeed * maxVolts);
      // _config->left2.motorController->SetVoltage(leftSpeed * maxVolts);
      // _config->left3.motorController->SetVoltage(leftSpeed * maxVolts);
      // _config->right1.motorController->SetVoltage(rightSpeed * maxVolts);
      // _config->right2.motorController->SetVoltage(rightSpeed * maxVolts);
      // _config->right3.motorController->SetVoltage(rightSpeed * maxVolts);
      break;
    }
    case DrivetrainState::kAuto:
      break;
  }
}
