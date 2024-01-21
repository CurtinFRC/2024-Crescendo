#pragma once

#include "wombat.h"
#include <frc/DigitalInput.h>
#include <string>
#include <memory>

struct IntakeConfig {
  wom::Gearbox IntakeMotor;
  frc::DigitalInput* intakeSensor;
  frc::DigitalInput* magSensor;
  frc::DigitalInput* shooterSensor;
};

enum class IntakeState {
  kIdle,
  kRaw,
  kHold,
  kEject,
  kIntake,
  kPass
};

class Intake : public behaviour::HasBehaviour {
 public:
  explicit Intake(IntakeConfig config);
  
  void OnUpdate(units::second_t dt);
  
  void setState(IntakeState state);
  void setRaw(units::volt_t voltage);
  IntakeConfig GetConfig();

 private:
  IntakeConfig _config;
  IntakeState _state = IntakeState::kIdle;

  units::volt_t _rawVoltage = 0_V;
  std::string _stringStateName = "error";
  units::volt_t _setVoltage = 0_V;
  std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("Intake");
};