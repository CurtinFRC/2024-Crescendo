#include "behaviours/MagBehaviour.h"

MagManualControl::MagManualControl(Mag *mag, frc::XboxController &codriver) : _mag(mag), _codriver(codriver) {
  Controls(mag);
}
MagAutoControl::MagAutoControl(Mag *mag, units::volt_t voltage) {
  Controls(mag);
}

void MagManualControl::OnTick(units::second_t dt) {
  //manual control
  if (_codriver.GetAButton()) {
    //adjust voltage later
    _mag->setRaw(5_V);
  } else if (_codriver.GetBButton()) {
    _mag->setRaw(-5_V);
  } else {
    _mag->setRaw(0_V);
  }
}


void MagAutoControl::OnTick(units::second_t dt) {
  MagState _current = _mag->getState();
  //Auto control.
  //adjust voltage later.
  if (_current == MagState::kHold) {
    //-1_V is for keeping the note in the magazine during transit.
    _mag->setRaw(-1_V);
  } else if (_current == MagState::kEject) {
    _mag->setRaw(-5_V);
  } else if (_current == MagState::kPass) {
    _mag->setRaw(5_V);
  } else {
    _mag->setRaw(0_V);
  };
}

