#pragma once

#include "utils/Gearbox.h"
#include "utils/PID.h"
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"

#include <networktables/NetworkTable.h>
#include <units/angular_velocity.h>
#include <units/charge.h>

#include <memory>

namespace wom {
  enum class ShooterState {
    kPID,
    kManual,
    kIdle
  };

  struct ShooterParams {
    wom::utils::Gearbox gearbox;
    wom::utils::PIDConfig<units::radians_per_second, units::volt> pid;
    units::ampere_t currentLimit;
  };

  class Shooter : public behaviour::HasBehaviour {
   public:
    Shooter(std::string path, ShooterParams params);

    void SetManual(units::volt_t voltage);
    void SetPID(units::radians_per_second_t goal);
    void SetIdle();

    void OnUpdate(units::second_t dt);

    bool IsStable() const;

   private:
    ShooterParams _params;
    ShooterState _state;

    units::volt_t _setpointManual{0};

    wom::utils::PIDController<units::radians_per_second, units::volt> _pid;

    std::shared_ptr<nt::NetworkTable> _table;
  };

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
