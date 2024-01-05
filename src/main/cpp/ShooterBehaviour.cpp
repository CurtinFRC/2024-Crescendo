#include "ShooterBehaviour.h"

ShooterManualControl::ShooterManualControl(Shooter *shooter, frc::XboxController &codriver) : _shooter(shooter), _codriver(codriver) {
 Controls(shooter);
}

void ShooterManualControl::OnTick(units::second_t dt) {

  // if (_codriver.GetAButton()) {
  //  _shooter->setLauncherState(LauncherState::kSpinUp);
  // } else if (_codriver.GetBButton()) {
  //  _shooter->setLauncherState(LauncherState::kSpinDown);
  // } else if (_codriver.GetXButton()) {
  //  _shooter->setLauncherState(LauncherState::kInBattery);
  // } else if (_codriver.GetStartButton()) {
  //  _shooter->setLauncherState(LauncherState::kIdle);
  // } else if (_codriver.GetBackButton()) {
  //  _shooter->setLauncherState(LauncherState::kIdle);
  // }

  // use raw states for now
  if (_codriver.GetRightY() > 0.05) {
    _shooter->setLauncherState(LauncherState::kRaw);
    _shooter->setLauncherRaw(_codriver.GetRightY() * 8_V);
  } else if (_codriver.GetRightY() < -0.05) {
    _shooter->setLauncherState(LauncherState::kRaw);
    _shooter->setLauncherRaw(_codriver.GetRightY() * 8_V);
  } else {
    _shooter->setLauncherState(LauncherState::kIdle);
  }

  if (_codriver.GetLeftY() > 0.05) {
    _shooter->setLoaderState(LoaderState::kRaw);
    _shooter->setLoaderRaw(_codriver.GetLeftY() * 4_V);
  } else if (_codriver.GetLeftY() < -0.05) {
    _shooter->setLoaderState(LoaderState::kRaw);
    _shooter->setLoaderRaw(_codriver.GetLeftY() * 4_V);
  } else {
    _shooter->setLoaderState(LoaderState::kIdle);
  }


  if (_codriver.GetAButton()) {
    _shooter->setLauncherState(LauncherState::kSpinUp);
  } else if (_codriver.GetBButton()) {
    _shooter->setLauncherState(LauncherState::kSpinDown);
  // launch ball
  } else if (_codriver.GetXButton()) {
    _shooter->setLoaderState(LoaderState::kLoading);
  }
  else {
    _shooter->setLoaderState(LoaderState::kIdle);
  }
}