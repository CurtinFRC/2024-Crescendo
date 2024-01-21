#pragma once
#include "Wombat.h"
#include <frc/DigitalInput.h>
#include <string>
#include <memory>


struct MagConfig {
  wom::Gearbox magGearbox;
  frc::DigitalInput* intakeSensor;
  frc::DigitalInput* magSensor;
  frc::DigitalInput* shooterSensor;
};

enum class MagState {
  kIdle,
  kHold,
  kEject,
  kRaw,
  kPass
};

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