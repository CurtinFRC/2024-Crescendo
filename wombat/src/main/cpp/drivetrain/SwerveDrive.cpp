#include "drivetrain/SwerveDrive.h"

wom::drivetrain::SwerveModule::SwerveModule(wom::drivetrain::SwerveModuleName name, wom::drivetrain::SwerveModuleConfig config, wom::drivetrain::SwerveModuleState state) :
  _name(name),
  _config(config),
  _state(state)
  {}

wom::drivetrain::SwerveModuleName wom::drivetrain::SwerveModule::GetName() {
  return _name;
}

wom::drivetrain::SwerveModuleConfig wom::drivetrain::SwerveModule::GetConfig() {
  return _config;
}

wom::drivetrain::SwerveModuleState wom::drivetrain::SwerveModule::GetState() {
  return _state;
}

void wom::drivetrain::SwerveModule::SetState(wom::drivetrain::SwerveModuleState state) {
  _state = state;
}

