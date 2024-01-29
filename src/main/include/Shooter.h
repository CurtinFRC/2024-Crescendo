// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once
#include <frc/DigitalInput.h>
#include <frc/controller/PIDController.h>
#include <units/angular_velocity.h>

#include <memory>
#include <string>

#include "Wombat.h"

struct ShooterConfig {
  std::string path;
  wom::Gearbox ShooterGearbox;
  // wom::PIDConfig<units::radians_per_second, units::volt> pidConfig;
  // frc::DigitalInput* shooterSensor;
};

enum class ShooterState { kIdle, kShooting, kSpinUp, kReverse, kRaw };

class Shooter : public behaviour::HasBehaviour {
 public:
  explicit Shooter(ShooterConfig config);

  void OnUpdate(units::second_t dt);
  void SetState(ShooterState state);
  void SetRaw(units::volt_t voltage);
  void SetPidGoal(units::radians_per_second_t);
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("Shooter");
  ShooterConfig GetConfig() { return _config; }

 private:
  ShooterConfig _config;
  ShooterState _state = ShooterState::kRaw;
  units::volt_t _rawVoltage;
  units::radians_per_second_t _goal;
  units::volt_t _setVoltage = 0_V;
  // frc::PIDController _pid;
  // frc::DigitalInput _shooterSensor{0};
};