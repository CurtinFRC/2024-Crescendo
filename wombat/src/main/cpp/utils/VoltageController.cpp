#include "utils/VoltageController.h"
#include <frc/RobotController.h>

#include <frc/RobotController.h>

units::volt_t wom::utils::VoltageController::GetEstimatedRealVoltage() const {
  units::volt_t vb = frc::RobotController::GetBatteryVoltage();
  return units::math::min(units::math::max(-vb, GetVoltage()), vb);
}

wom::utils::VoltageController::VoltageController(frc::MotorController *MotorController) : _MotorController(MotorController)
{}

void wom::utils::VoltageController::SetVoltage(units::volt_t voltage) {
  _MotorController->Set(voltage / GetBusVoltage());
}

units::volt_t wom::utils::VoltageController::GetVoltage() const {
  return _MotorController->Get() * GetBusVoltage();
}

units::volt_t wom::utils::VoltageController::GetBusVoltage() const {
  return frc::RobotController::GetInputVoltage() * 1_V;
}

void wom::utils::VoltageController::SetInverted(bool invert) {
  _MotorController->SetInverted(invert);
}

bool wom::utils::VoltageController::GetInverted() const {
  return frc::RobotController::GetInputVoltage();
}
