// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "ArmBehaviours.h"

#include "Wombat.h"
#include "subsystems/Arm.h"

ArmManualControl::ArmManualControl(wom::Arm* arm, frc::XboxController* codriver)
    : _arm(arm), _codriver(codriver) {
  Controls(arm);
}

void ArmManualControl::OnTick(units::second_t dt) {
  if (_codriver->GetXButtonPressed()) {
    _arm->SetState(wom::ArmState::kRaw);
  }

  if (_arm->GetState() == wom::ArmState::kRaw) {
    _arm->SetRaw(_codriver->GetRightY() * 6_V);
  }
}
