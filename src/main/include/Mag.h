// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once
#include <frc/DigitalInput.h>

#include <memory>
#include <string>

#include "wombat.h"

struct MagConfig {
  wom::Gearbox magGearbox;
  frc::DigitalInput* intakeSensor;
  frc::DigitalInput* magSensor;
  frc::DigitalInput* shooterSensor;
};

enum class MagState { kIdle, kHold, kEject, kRaw, kPass };

class Mag : public behaviour::HasBehaviour {
 public:
  explicit Mag(MagConfig config);

  void OnUpdate(units::second_t dt);
  void SetState(MagState state);
  void SetRaw(units::volt_t voltage);
  MagState GetState();

 private:
  MagConfig _config;
  MagState _state;
  units::volt_t _rawVoltage = 0_V;
  std::string _stringStateName = "No State";
  units::volt_t _setVoltage = 0_V;
  std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("Magazine");
};
