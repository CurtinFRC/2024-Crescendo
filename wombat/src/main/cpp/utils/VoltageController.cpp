#include "utils/VoltageController.h"
#include <frc/RobotController.h>

#include <frc/RobotController.h>

using namespace wom;

units::volt_t VoltageController::GetEstimatedRealVoltage() const {
  units::volt_t vb = frc::RobotController::GetBatteryVoltage();
  return units::math::min(units::math::max(-vb, GetVoltage()), vb);
}

VoltageController::VoltageController(frc::MotorController *MotorController) : _MotorController(MotorController)
{}

void VoltageController::SetVoltage(units::volt_t voltage) {
  _MotorController->Set(voltage / GetBusVoltage());
}

units::volt_t VoltageController::GetVoltage() const {
  return _MotorController->Get() * GetBusVoltage();
}

units::volt_t VoltageController::GetBusVoltage() const {
  return frc::RobotController::GetInputVoltage() * 1_V;
}

void VoltageController::SetInverted(bool invert) {
  _MotorController->SetInverted(invert);
}

bool VoltageController::GetInverted() const {
  return frc::RobotController::GetInputVoltage();
}
