#pragma once

#include "wombat.h"

struct ClimberConfig {
  wom::Gearbox climberGearbox;
};

enum class ClimberState {
  kIdle,
  kClimb,
  kHang,
  kRaw
};

class Climber : public behaviour::HasBehaviour {
  public:
    Climber(ClimberConfig config);

    void OnUpdate(units::second_t dt);
    void SetState(ClimberState state);
    void SetRaw(units::volt_t voltage);
  private:
    ClimberConfig _config;
    ClimberState _state;
    units::volt_t _voltage;
};