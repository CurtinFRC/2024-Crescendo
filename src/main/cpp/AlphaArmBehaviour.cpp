// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArmBehaviour.h"

#include <frc/XboxController.h>

#include "AlphaArm.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include "vision/Limelight.h"
#include "vision/Vision.h"

// AlphaArmManualControl::AlphaArmManualControl(AlphaArm* alphaArm, frc::XboxController* codriver)
// : _alphaArm(alphaArm), _codriver(codriver) {
// Controls(alphaArm);
// }

void AlphaArmManualControl::OnTick(units::second_t dt) {
  _table->GetEntry("State").SetBoolean(_rawControl);
  _table->GetEntry("Goal Value").SetBoolean(_gotValue);

  if (_codriver->GetBButton()) {
    if (_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if (_rawControl) {
    // _alphaArm->SetState(AlphaArmState::kRaw);
    // _alphaArm->SetArmRaw(_codriver->GetRightY() * 12_V);
  } else {
    if (_codriver->GetLeftTriggerAxis() > 0.1) {
      _alphaArm->SetState(AlphaArmState::kSpeakerAngle);
    } else if (_codriver->GetLeftBumper()) {
      _alphaArm->SetState(AlphaArmState::kAmpAngle);
    } else if (_codriver->GetYButton()) {
      _alphaArm->SetState(AlphaArmState::kStowed);
    } else if (_codriver->GetRightBumper()) {
      _alphaArm->SetState(AlphaArmState::kIntakeAngle);
    } else {
      _alphaArm->SetState(AlphaArmState::kIdle);
    }
  }
}
