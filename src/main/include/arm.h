
#pragma once

#include "Wombat.h"

struct ArmConfig {
    wom::Gearbox armGearBox;
};


enum class ArmState {
    kIdle,
    kRaw,
    kAngle
};




class Arm : public behaviour::HasBehaviour{
 public:
  Arm(ArmConfig config);

  void OnUpdate(units::second_t dt);
  void setState(ArmState state);
  void setAngle(units::degree_t angle);
  void setRaw(units::volt_t voltage);
 private:
 ArmConfig _config;
 ArmState _state = ArmState::kIdle;
 units::degree_t _angle;
 units::volt_t _voltage;


};

