// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/Servo.h>

#include <memory>
#include <string>

#include "Wombat.h"

struct ClimberConfig {
  wom::Gearbox climberGearbox;
};

enum class ClimberState { kIdle, kArmUp, kArmDown, kMatch, kRaw, kRatchet };

class Climber : public behaviour::HasBehaviour {
 public:
  explicit Climber(ClimberConfig config);

  void OnUpdate(units::second_t dt);
  void SetState(ClimberState state);
  ClimberState GetState();
  void SetRaw(units::volt_t voltage);

 private:
  ClimberConfig _config;
  ClimberState _state = ClimberState::kMatch;
  units::volt_t _rawVoltage = 0_V;
  std::string _stringStateName = "error";
  units::volt_t _setVoltage = 0_V;
  std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("Climber");

  frc::PIDController _pid;
};
