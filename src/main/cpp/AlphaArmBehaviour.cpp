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
    _alphaArm->SetArmRaw(_codriver->GetRightY() * 6_V);
    _alphaArm->setWristRaw(_codriver->GetLeftY() * -6_V);
  } else {
    if (_codriver->GetRightBumperPressed()) {
      _alphaArm->SetState(AlphaArmState::kForwardWrist);
    }
    if (_codriver->GetLeftBumperPressed()) {
      _alphaArm->SetState(AlphaArmState::kReverseWrist);
    }
  }
}

ArmToSetPoint::ArmToSetPoint(AlphaArm* alphaArm, units::degree_t armAngle, float armSpeed) : _alphaArm(alphaArm) {
  Controls(alphaArm);
}
// // ArmSpeed is a float from 0-1, 1 being instantly and 0 being don't move at all.

void ArmToSetPoint::OnTick(units::second_t dt) {
  // _armCurrentDegree = _alphaArm->GetConfig().alphaArmGearbox.encoder.GetEncoderPosition();
  // _alphaArm->GetConfig().alphaArmGearbox.encoder.SetEncoderPosition(armAngle * armSpeed);
}