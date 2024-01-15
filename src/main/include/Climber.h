#pragma once

#include "wombat.h"

struct ClimberConfig {
  wom::Gearbox climberGearBox;
};

enum class ClimberState {
  kIdle,
  kLift,
  kHang
};

class Climber : public behaviour:HasBehaviour {
  public:
    Climber(ClimberConfig config);

    void OnUpdate(units::second_t dt);
    void setState(ClimberState state);
    // void setReceive(units::volt_t voltage)
    // void setHold(units::volt_t voltage)
    // void setPass(units::volt_t voltage)
    // void setLeave(units::volt_t voltage)
    void setRaw(units::volt_t voltage);
    ClimberState getState();
  private:
    ClimberConfig _config;
    ClimberState _state;
    units::volt_t _voltage;

}