#pragma once

#include "wombat.h"

struct TankConfig {
  wom::Gearbox tankFrontLeftGearbox;
  wom::Gearbox tankFrontRightGearbox;
  wom::Gearbox tankBackLeftGearbox;
  wom::Gearbox tankBackRightGearbox;
};

enum class TankState {
  kIdle,
  kRaw, 
};

class TankDrive : public behaviour::HasBehaviour {
 public:
  TankDrive(TankConfig config);

  void OnUpdate(units::second_t dt);

  void setState(TankState state);
  void setRaw(units::volt_t voltL, units::volt_t voltR);
  
 private:
  TankConfig _config;
  TankState _state = TankState::kIdle;

  units::volt_t _voltL;
  units::volt_t _voltR;
};