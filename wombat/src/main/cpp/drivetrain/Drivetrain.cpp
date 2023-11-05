#include "drivetrain/Drivetrain.h"

using namespace frc;
using namespace units;

wom::drivetrain::Drivetrain::Drivetrain(wom::drivetrain::DrivetrainConfig *config, XboxController &driver): _config(config), _driver(driver) {}
wom::drivetrain::Drivetrain::~Drivetrain() {}

wom::drivetrain::DrivetrainConfig *wom::drivetrain::Drivetrain::GetConfig() { return _config; } 
wom::drivetrain::DrivetrainState wom::drivetrain::Drivetrain::GetState() { return _state; }

void wom::drivetrain::Drivetrain::SetState(wom::drivetrain::DrivetrainState state) { _state = state; }

void wom::drivetrain::Drivetrain::OnStart() {
  std::cout << "Starting Tank" << std::endl;
}

void wom::drivetrain::Drivetrain::TankControl() {
      double rightSpeed = wom::utils::deadzone(_driver.GetRightY());
      double leftSpeed = wom::utils::deadzone(_driver.GetLeftY());
      _config->left1.transmission->SetVoltage(leftSpeed * maxVolts);
      _config->left2.transmission->SetVoltage(leftSpeed * maxVolts);
      _config->left3.transmission->SetVoltage(leftSpeed * maxVolts);
      _config->right1.transmission->SetVoltage(rightSpeed * maxVolts);
      _config->right2.transmission->SetVoltage(rightSpeed * maxVolts);
      _config->right3.transmission->SetVoltage(rightSpeed * maxVolts);
}

void wom::drivetrain::Drivetrain::OnUpdate(second_t dt) {
  switch(_state) {
    case wom::drivetrain::DrivetrainState::kIdle:
      break;
    case wom::drivetrain::DrivetrainState::kTank:
      {
      TankControl();
      break;
      }
  }
}
