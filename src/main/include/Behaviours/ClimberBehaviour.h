// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "Climber.h"
#include "Wombat.h"

class ClimberManualControl : public behaviour::Behaviour {
 public:
  explicit ClimberManualControl(Climber* climber, frc::XboxController* codriver);
  void OnTick(units::second_t dt) override;

 private:
  Climber* _climber;
  frc::XboxController* _codriver;
  bool _rawControl = false;
};
