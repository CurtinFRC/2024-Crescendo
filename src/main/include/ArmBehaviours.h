// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "AlphaArm.h"
#include "Wombat.h"

class ArmManualControl : public behaviour::Behaviour {
 public:
  // explicit ArmManualControl(wom::Arm* arm, frc::XboxController* codriver);
  void OnTick(units::second_t dt);

 private:
  // wom::Arm* _arm;
  frc::XboxController* _codriver;
  // units::volt_t _rightStick = ((_codriver->GetRightY()>0.05 || _codriver->GetRightY() < -0.05
  // )?_codriver->GetRightY():0) * 2_V;
  bool _rawControl;
};
