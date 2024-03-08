// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>

#include <memory>

#include "Intake.h"
#include "LED.h"
#include "Shooter.h"
#include "Wombat.h"
#include "frc/Timer.h"
#include <units/angular_velocity.h>

class ShooterManualControl : public behaviour::Behaviour {
 public:
  ShooterManualControl(Shooter* shooter, frc::XboxController* codriver, LED* led);

  void OnTick(units::second_t dt) override;

 private:
  Shooter* _shooter;
  frc::XboxController* _codriver;

  LED* _led;

  bool _rawControl = false;
  std::shared_ptr<nt::NetworkTable> table =
      nt::NetworkTableInstance::GetDefault().GetTable("Shooter Behaviour");
};

class AutoShooter : public behaviour::Behaviour {
 public:
  AutoShooter(Shooter* shooter, Intake* intake, units::radians_per_second_t goal);

  void OnTick(units::second_t dt) override;

 private:
  Shooter* _shooter;
  Intake* _intake;
  units::radians_per_second_t _goal;

  bool hasShot = false;
  frc::Timer _timer;
};
