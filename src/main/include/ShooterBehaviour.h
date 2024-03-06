// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>
#include <networktables/NetworkTable.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <memory>

#include "LED.h"
#include "Shooter.h"
#include "Wombat.h"

class Vision;

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

class VisionShooterSpeed : public wom::Behaviour {
 public:
  VisionShooterSpeed(Shooter* shooter, Vision* vision, wom::SwerveDrive* swerve);

  units::radians_per_second_t GetDesiredSpeed(units::meter_t distance);

  void OnTick(units::second_t dt) override;

 private:
  Shooter* m_shooter;
  Vision* m_vision;
  wom::SwerveDrive* m_swerve;
  std::shared_ptr<nt::NetworkTable> m_table;
};
