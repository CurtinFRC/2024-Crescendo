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
    
}