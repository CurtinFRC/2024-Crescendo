// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once
#include <frc/XboxController.h>

#include <memory>

#include "AlphaArm.h"
#include "Wombat.h"
#include "vision/Vision.h"

class AlphaArmManualControl : public behaviour::Behaviour {
 public:
  explicit AlphaArmManualControl(AlphaArm* alphaArm, frc::XboxController* codriver);
  void OnTick(units::second_t dt);

 private:
  AlphaArm* _alphaArm;
  frc::XboxController* _codriver;
  units::volt_t _setAlphaArmVoltage = 0_V;
  bool _rawControl = false;
  bool _gotValue = false;
  std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("AlphaArmSubsystem");
  bool climbing = false;
};

class AimToToAprilTag : public behaviour::Behaviour {
 public:
  explicit AimToToAprilTag(AlphaArm* arm, VisionTarget target, Vision* vision);
  explicit AimToToAprilTag(AlphaArm* arm, Vision* vision);

  void OnTick(units::second_t dt);

 private:
  AlphaArm* _arm;
  int _target;
  Vision* _vision;
};
