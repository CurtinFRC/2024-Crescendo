// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArmBehaviour.h"

#include <frc/XboxController.h>

AlphaArmManualControl::AlphaArmManualControl(AlphaArm* alphaArm, frc::XboxController* codriver)
    : _alphaArm(alphaArm), _codriver(codriver) {
  Controls(alphaArm);
}

void AlphaArmManualControl::OnTick(units::second_t dt) {
  if (_codriver->GetXButtonPressed()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if (_rawControl) {
    _alphaArm->SetState(AlphaArmState::kRaw);
    _alphaArm->SetArmRaw(_codriver->GetRightY() * 12_V);  
  } else {
    // if(_codriver->GetLeftBumperPressed()) {
    //   _alphaArm->SetState(AlphaArmState::kAmpAngle);
    //   _alphaArm->SetGoal(1.57_rad);
      
    // } else if(_codriver->GetRightBumperPressed()){
    //   _alphaArm->SetState(AlphaArmState::kSpeakerAngle);
    //   _alphaArm->SetGoal(0.52_rad);
    // }

    _alphaArm->SetState(AlphaArmState::kAmpAngle);
    _alphaArm->setControllerRaw(wom::deadzone(_codriver->GetRightY()) * 12_V);
  }
}