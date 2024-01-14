#include "behaviours/MagBehaviour.h"

MagManualControl::MagManualControl(Mag *mag, frc::XboxController &codriver) : _mag(mag), _codriver(codriver) {
  Controls(mag);
}

void MagManualControl::OnTick(units::second_t dt) {

  if (_codriver.GetAButtonPressed()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }
  
  if (_rawControl) {
    // Manual control, right bumper for manual override.
    if (_codriver.GetLeftBumper()) {
      // Adjust voltage later.
      _mag->setRaw(5_V);
      _mag->setState(MagState::kRaw);
    } else if (_codriver.GetRightBumper()) {
      _mag->setRaw(-5_V);
      _mag->setState(MagState::kRaw);

    } else {
      _mag->setRaw(0_V);
      // _mag->setState(MagState::kIdle);
    }
    
  } else {
    _mag->setState(MagState::kIdle);
    if (_codriver.GetLeftBumper()) {
      _mag->setState(MagState::kPass);
      
    }
  }
}
    
    MagAutoControl::MagAutoControl(Mag *mag, units::volt_t voltage) {}
    
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
