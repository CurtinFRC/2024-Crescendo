// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "Mag.h"
#include "Wombat.h"

class MagManualControl : public behaviour::Behaviour {
 public:
  explicit MagManualControl(Mag* mag, frc::XboxController* codriver);

  void OnTick(units::second_t dt) override;

 private:
  Mag* _mag;
  frc::XboxController* _codriver;
  bool _rawControl;  // Default of Bool is False.
};

class MagAutoPass : public behaviour::Behaviour {
 public:
  explicit MagAutoPass(Mag* mag);

  void OnTick(units::second_t dt) override;

 private:
  Mag* _mag;
};

class MagAutoHold : public behaviour::Behaviour {
 public:
  explicit MagAutoHold(Mag* mag);

  void OnTick(units::second_t dt) override;

 private:
  Mag* _mag;
};
