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
    _alphaArm->SetRaw(_rightStick);
 }
}

// void AlphaArmManualControl::OnTick(units::second_t dt){
     

//     _alphaArm->SetState(AlphaArmState::kRaw); 
//     _alphaArm->SetRaw(_rightStick);
// }