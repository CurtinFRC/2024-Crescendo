#pragma once
#include "wombat.h"
#include <frc/DigitalInput.h>

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
    Mag(MagConfig config);

    void OnUpdate(units::second_t dt);
    void SetState(MagState state);
    void SetRaw(units::volt_t voltage);
    MagState GetState();
  private:
    MagConfig _config;
    MagState _state;
    units::volt_t _voltage;
};