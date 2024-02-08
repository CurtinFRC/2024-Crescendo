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
  std::string path;
  wom::Gearbox IntakeGearbox;
  frc::DigitalInput* intakeSensor;
  // frc::DigitalInput* magSensor;
  // frc::DigitalInput* shooterSensor;
  wom::PIDConfig<units::radians_per_second, units::volt> pidConfig;
};

enum class IntakeState { kIdle, kRaw, kHold, kEject, kIntake, kPass, kPID};

class Intake : public behaviour::HasBehaviour {
 public:
  explicit Intake(IntakeConfig config);

  void OnUpdate(units::second_t dt);

  void SetState(IntakeState state);
  void SetRaw(units::volt_t voltage);
  void SetPidGoal(units::radians_per_second_t goal);
  void OnStart();
  IntakeState GetState();
  IntakeConfig GetConfig();

 private:
  IntakeConfig _config;
  IntakeState _state = IntakeState::kIdle;

  units::volt_t _rawVoltage = 0_V;
  std::string _stringStateName = "error";
  units::volt_t _setVoltage = 0_V;
  units::volt_t holdVoltage = 0_V;
  bool _intaking; //NEEDS DEFAULTS
  bool _ejecting;
  bool _passing;
  units::radians_per_second_t _goal;
  wom::PIDController<units::radians_per_second, units::volt> _pid;
  
  std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("Intake");
};
