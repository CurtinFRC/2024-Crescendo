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
      _mag->setRaw(5_V);
      _mag->setState(MagState::kRaw);
    } else if (_codriver.GetRightBumper()) {
      _mag->setRaw(-5_V);
      _mag->setState(MagState::kRaw);

    } else {
      _mag->setRaw(0_V);
    }
    
  } else {
    _mag->setState(MagState::kIdle);
    if (_codriver.GetLeftBumper()) {
      _mag->setState(MagState::kPass);
      
    }
  }
}
    
    MagAutoPass::MagAutoPass(Mag *mag) {}
    
    void MagAutoPass::OnTick(units::second_t dt) {  
      _mag->setState(MagState::kPass);
    }

    MagAutoHold::MagAutoHold(Mag *mag) {}
    
    void MagAutoHold::OnTick(units::second_t dt) {  
      _mag->setState(MagState::kHold);
    }
