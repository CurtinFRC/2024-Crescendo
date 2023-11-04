#pragma once

#include "subsystems/Shooter.h"
#include "behaviour/Behaviour.h"

namespace wom {
namespace subsystems {
namespace behaviours {
  class ShooterConstant : public behaviour::Behaviour {
   public:
    ShooterConstant(Shooter *s, units::volt_t setpoint);

    void OnTick(units::second_t dt) override;
   private:
    Shooter *_shooter;
    units::volt_t _setpoint;
  };

  class ShooterSpinup : public behaviour::Behaviour {
   public:
    ShooterSpinup(Shooter *s, units::radians_per_second_t speed, bool hold = false);

    void OnTick(units::second_t dt) override;
   private:
    Shooter *_shooter;
    units::radians_per_second_t _speed;
    bool _hold;
  };
}
}
}

