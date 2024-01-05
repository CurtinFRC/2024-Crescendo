#include "Shooter.h"

Shooter::Shooter(ShooterConfig config) : _config(config) {}

void Shooter::OnUpdate(units::second_t dt) {

  switch (_launcherState) {
    case LauncherState::kIdle: // no movement
     _launcherVoltage = 0_V;
     break;
    // case LauncherState::kLaunching: // keep rpm constant at 300
    //   _launcherPID.SetSetpoint(300_rpm);
    //   _launcherVoltage = _launcherPID.Calculate(_launcherRpm, dt, 0.2_V);
    //  break;
    case LauncherState::kSpinUp: // get to 300 rpm
      _launcherVoltage = 5_V;
     break;
    case LauncherState::kSpinDown: // go to 0 rpm (slowly)
      _launcherVoltage = 0_V;
     break;
    // case LauncherState::kInBattery: // 300 rpm
    //   ShooterConfig.launcherPID.SetSetpoint(300_rpm);
    //   _launcherVoltage = ShooterConfig.launcherPID.Calculate(_launcherRpm);
    //  break;
    case LauncherState::kRaw: //set voltage manually
     _launcherVoltage = _launcherRawVoltage;
     break;
    default:
     std::cout << "Error launcher in invalid state" << std::endl;
  }

  switch (_loaderState) {
    case LoaderState::kIdle: // no movement
      _loaderVoltage = 0_V;
     break;
    case LoaderState::kLoading: // inserting (blind)
     _loaderVoltage = 5_V;
     break;
    case LoaderState::kReversing: // go backwards
      _loaderVoltage = -5_V;
     break;
    case LoaderState::kRaw: // set voltage manually
      _loaderVoltage = _loaderRawVoltage;
     break;
    default:
     std::cout << "Error loader in invalid state" << std::endl;
  }

  // Update motors 
  _config.launcherGearBox.transmission->SetVoltage(_launcherVoltage);
  _config.loaderGearBox.transmission->SetVoltage(_loaderVoltage);
}


// Launcher
void Shooter::setLauncherState(LauncherState launcherState) {
  _launcherState = launcherState;
} 

void Shooter::setLauncherRaw(units::volt_t voltage) {
  _launcherRawVoltage = voltage;
}

void Shooter::setLauncherRPM(units::revolutions_per_minute_t rpm) {
  _launcherRpm = rpm;
}


// Loader
void Shooter::setLoaderState(LoaderState loaderState) {
  _loaderState = loaderState;
}

void Shooter::setLoaderRaw(units::volt_t voltage) {
  _loaderRawVoltage = voltage;
}

void Shooter::setLoaderRPM(units::revolutions_per_minute_t rpm) {
  _loaderRpm = rpm;
}


