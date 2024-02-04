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

struct IntakeConfig {
  wom::Gearbox IntakeMotor;
  frc::DigitalInput* intakeSensor;
  frc::DigitalInput* magSensor;
  frc::DigitalInput* shooterSensor;
  wom::PIDConfig<units::radians_per_second, units::volt> pidConfig;
};

enum class IntakeState { kIdle, kRaw, kHold, kEject, kIntake, kPass };

class Intake : public behaviour::HasBehaviour {
 public:
  explicit Intake(IntakeConfig config);

  void OnUpdate(units::second_t dt);

  void setState(IntakeState state);
  void setRaw(units::volt_t voltage);
  void SetPidGoal(units::radians_per_second_t);
  IntakeState getState();
  IntakeConfig GetConfig();

 private:
  IntakeConfig _config;
  IntakeState _state = IntakeState::kIdle;

  units::volt_t _rawVoltage = 0_V;
  std::string _stringStateName = "error";
  units::volt_t _setVoltage = 0_V;
  bool _intaking;
  bool _ejecting;
  bool _passing;
  wom::PIDController<units::radians_per_second, units::volt> _pid;
  
  std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("Intake");
};
