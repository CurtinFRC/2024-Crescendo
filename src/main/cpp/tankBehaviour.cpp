#include "tankBehaviour.h"
#include <frc/XboxController.h>

TankManualControl::TankManualControl(TankDrive *tank, frc::XboxController &driver) : _tank(tank), _driver(driver) {
  Controls(tank);
};

void TankManualControl::OnTick(units::second_t dt) {
  _leftStick = (_driver.GetLeftY()>0.05?_driver.GetLeftY():0) * 8_V;
  _rightStick = (_driver.GetRightY()>0.05?_driver.GetRightY():0) * 8_V;

  _tank->setRaw(_leftStick, _rightStick);
}