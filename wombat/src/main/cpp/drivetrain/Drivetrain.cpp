#include "drivetrain/Drivetrain.h"

using namespace frc;
using namespace units;

wom::Drivetrain::Drivetrain(wom::DrivetrainConfig *config, XboxController &driver): _config(config), _driver(driver) {}
wom::Drivetrain::~Drivetrain() {}

wom::DrivetrainConfig *wom::Drivetrain::GetConfig() { return _config; } 
wom::DrivetrainState wom::Drivetrain::GetState() { return _state; }

void wom::Drivetrain::SetState(DrivetrainState state) { _state = state; }

void wom::Drivetrain::OnStart() {
  std::cout << "Starting Tank" << std::endl;
}

void wom::Drivetrain::OnUpdate(second_t dt) {
  switch(_state) {
    case wom::DrivetrainState::kIdle:
      break;
    case wom::DrivetrainState::kTank:
      {
      double rightSpeed = wom::utils::deadzone(_driver.GetRightY());
      double leftSpeed = wom::utils::deadzone(_driver.GetLeftY());
      _config->left1.transmission->SetVoltage(leftSpeed * maxVolts);
      _config->left2.transmission->SetVoltage(leftSpeed * maxVolts);
      _config->left3.transmission->SetVoltage(leftSpeed * maxVolts);
      _config->right1.transmission->SetVoltage(rightSpeed * maxVolts);
      _config->right2.transmission->SetVoltage(rightSpeed * maxVolts);
      _config->right3.transmission->SetVoltage(rightSpeed * maxVolts);
      break;
      }
    case wom::DrivetrainState::kAuto:
      break;
  }
}
