// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include "Robot.h"
#include "Shooter.h"
#include "Wombat.h"

class ShooterManualControl : public behaviour::Behaviour {
 public:
  ShooterManualControl(Shooter* shooter, frc::XboxController* codriver); 

  void OnTick(units::second_t dt) override;

 private:
  Shooter* _shooter;
  frc::XboxController* _codriver;

  bool _rawControl = false;
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("Shooter Behaviour");
};