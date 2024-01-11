#pragma once
#include "wombat.h"
#include <frc/DigitalInput.h>

struct MagConfig {
  wom::Gearbox magGearbox;
  // frc::DigitalInput intakeSensor;
  // frc::DigitalInput magSensor;
};

enum class MagState {
  kIdle,
  kHold,
  kPass,
  kEject
};

class Mag : public behaviour::HasBehaviour {
  public:
    Mag(MagConfig config);

    void OnUpdate(units::second_t dt);
    void setState(MagState state);
    // void setReceive(units::volt_t voltage)
    // void setHold(units::volt_t voltage)
    // void setPass(units::volt_t voltage)
    // void setLeave(units::volt_t voltage)
    void setRaw(units::volt_t voltage);
    MagState getState();
  private:
    MagConfig _config;
    MagState _state;
    frc::DigitalInput _intakeSensor {0};
    frc::DigitalInput _magSensor {1};

    units::volt_t _voltage; 
};