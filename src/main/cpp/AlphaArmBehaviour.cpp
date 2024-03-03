// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArmBehaviour.h"

#include <frc/XboxController.h>

AlphaArmManualControl::AlphaArmManualControl(AlphaArm* alphaArm, Intake *intake, frc::XboxController* codriver)
    : _alphaArm(alphaArm), _intake(intake), _codriver(codriver) {
  Controls(alphaArm);
}

AlphaArmConfig AlphaArm::GetConfig() {
  return *_config;
}

void AlphaArmManualControl::OnTick(units::second_t dt) {

  _table->GetEntry("State").SetBoolean(_rawControl);
  _table->GetEntry("Goal Value").SetBoolean(_gotValue);


  if (_codriver->GetBackButton()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if (_rawControl) {
    _alphaArm->SetState(AlphaArmState::kRaw);
    if (wom::deadzone(_codriver->GetRightY())) {
      _alphaArm->SetArmRaw(_codriver->GetRightY() * 7_V);
    } else {
      _alphaArm->SetArmRaw(0_V);
    }
  } else {
    _table->GetEntry("CLIMBING:").SetBoolean(climbing);
    if (_codriver->GetPOV() == 90 || _codriver->GetPOV() == 180 || _codriver->GetPOV() == 270) {
      climbing = true;
    } if (_codriver->GetPOV() == 0) {
      climbing = false;
    } else {
      if (!climbing) {
        if(_codriver->GetLeftTriggerAxis() > 0.1){
          _alphaArm->SetState(AlphaArmState::kIntakeAngle);
        } else if (_codriver->GetLeftBumper()){
          _alphaArm->SetState(AlphaArmState::kAmpAngle);
        } else if(_codriver->GetAButton()){
          _alphaArm->SetState(AlphaArmState::kStowed);
        } else if(_codriver->GetPOV() == 90){
          _alphaArm->SetState(AlphaArmState::kClimbAngle);
        } else {
          if (_intake->GetState() == IntakeState::kHold) {
            _alphaArm->SetState(AlphaArmState::kIntakedAngle);
          } else {
            _alphaArm->SetState(AlphaArmState::kIntakeAngle);
          }
        }
      }
    }

  }

}


