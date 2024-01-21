#pragma once

#include "wombat.h"

struct ClimberConfig {
  wom::Gearbox climberGearbox;
};

enum class ClimberState {
  kIdle,
  kRaw,
  kClimb,
  kHang
};

class Climber : public behaviour::HasBehaviour {
  public:
    Climber(ClimberConfig config);

    void OnUpdate(units::second_t dt);
    void SetState(ClimberState state);
    void SetRaw(units::volt_t voltage);
  private:
    ClimberConfig _config;
    ClimberState _state = ClimberState::kIdle;
    units::volt_t _rawVoltage = 0_V;
    std::string _stringStateName = "error";
    units::volt_t _setVoltage;
    std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("Climber");
};