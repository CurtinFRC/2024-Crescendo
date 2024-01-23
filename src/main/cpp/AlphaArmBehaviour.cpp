#include "AlphaArmBehaviour.h"
#include <frc/XboxController.h>

AlphaArmManualControl::AlphaArmManualControl(AlphaArm *alphaArm, frc::XboxController* codriver) : _alphaArm(alphaArm), _codriver(codriver){
    Controls(alphaArm);
}

void AlphaArmManualControl::OnTick(units::second_t dt){
    if (_codriver->GetXButtonPressed()){
        if(_rawControl == true) {
            _rawControl = false;
        } else {
            _rawControl = true;
        }
    }

if (_rawControl) {
    _alphaArm->SetState(AlphaArmState::kRaw);
    _alphaArm->SetArmRaw(_codriver->GetRightY() * 6_V);
    _alphaArm->setWristRaw(_codriver->GetLeftY() * -6_V);
  } else {
    if (_codriver->GetRightBumperPressed()){
        _alphaArm->SetState(AlphaArmState::kForwardWrist);
    }
    if (_codriver->GetLeftBumperPressed()){
        _alphaArm->SetState(AlphaArmState::kReverseWrist);
    }
  }
}
