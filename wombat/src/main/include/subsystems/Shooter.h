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
namespace subsystems {
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
}
}
