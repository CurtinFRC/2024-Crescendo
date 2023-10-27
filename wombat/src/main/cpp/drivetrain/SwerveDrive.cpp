#include "drivetrain/SwerveDrive.h"

wom::SwerveModule::SwerveModule(wom::SwerveModuleName name, wom::SwerveModuleConfig config, wom::SwerveModuleState state) :
  _name(name),
  _config(config),
  _state(state)
  {}

wom::SwerveModuleName wom::SwerveModule::GetName() {
  return _name;
}

wom::SwerveModuleConfig wom::SwerveModule::GetConfig() {
  return _config;
}

wom::SwerveModuleState wom::SwerveModule::GetState() {
  return _state;
}

void wom::SwerveModule::SetState(wom::SwerveModuleState state) {
  _state = state;
}

