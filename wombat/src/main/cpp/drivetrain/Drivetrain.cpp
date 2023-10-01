#include "drivetrain/Drivetrain.h"

using namespace wom;
using namespace frc;
using namespace units;

Drivetrain::Drivetrain(DrivetrainConfig *config, XboxController &driver): _config(config), _driver(driver) {}
Drivetrain::~Drivetrain() {}

DrivetrainConfig *Drivetrain::GetConfig() { return _config; } 
DrivetrainState Drivetrain::GetState() { return _state; }

void Drivetrain::SetState(DrivetrainState state) { _state = state; }

void Drivetrain::OnStart() {
  std::cout << "Starting Tank" << std::endl;
}

void Drivetrain::OnUpdate(second_t dt) {
  switch(_state) {
    case DrivetrainState::kIdle:
      break;
    case DrivetrainState::kTank:
      {
      double rightSpeed = deadzone(_driver.GetRightY());
      double leftSpeed = deadzone(_driver.GetLeftY());
      _config->left1.transmission->SetVoltage(leftSpeed * maxVolts);
      _config->left2.transmission->SetVoltage(leftSpeed * maxVolts);
      _config->left3.transmission->SetVoltage(leftSpeed * maxVolts);
      _config->right1.transmission->SetVoltage(rightSpeed * maxVolts);
      _config->right2.transmission->SetVoltage(rightSpeed * maxVolts);
      _config->right3.transmission->SetVoltage(rightSpeed * maxVolts);
      break;
      }
    case DrivetrainState::kAuto:
      break;
  }
}
