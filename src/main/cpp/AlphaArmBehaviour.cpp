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
  if (_codriver->GetStartButtonPressed()) {
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
    // _alphaArm->SetState(AlphaArmState::kAmpAngle);
    // _alphaArm->setControllerRaw(wom::deadzone(_codriver->GetRightY()) * 12_V);
  }
}

AimToToAprilTag::AimToToAprilTag(AlphaArm* arm, VisionTarget target, Vision* vision)
    : _arm(arm), _target(static_cast<int>(target)), _vision(vision) {}
AimToToAprilTag::AimToToAprilTag(AlphaArm* arm, Vision* vision)
    : _arm(arm), _target(vision->CurrentAprilTag()), _vision(vision) {}

void AimToToAprilTag::OnTick(units::second_t dt) {
  units::meter_t dist = _vision->GetDistanceToTarget(_target).first;

  units::radian_t a = 1_rad;  // angle to shoot at one meter
  units::radian_t h = a * dist.value();

  _arm->SetGoal(h);
}
