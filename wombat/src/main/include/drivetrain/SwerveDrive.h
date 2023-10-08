#pragma once

#include "behaviour/behaviour.h"
#include "subsystems/Subsystem.h"
#include "utils/PID.h"

namespace wom {
    enum ModuleName {
        FrontLeft,
        FrontRight,
        BackLeft,
        BackRight,
    };

    struct SwerveModuleConfig {};

    enum SwerveModuleState {
        kIdle,
        kCalibration,
        kPID,
    };

    class SwerveModule : public wom::Subsystem<SwerveModuleConfig, SwerveModuleState>, public behaviour::HasBehaviour {
      public:
          SwerveModule(wom::Subsystem<SwerveModuleConfig, SwerveModuleState> _, wom::ModuleName name, wom::SwerveModuleConfig config, wom::SwerveModuleState state);
          ~SwerveModule();

          void OnStart() override;
          void OnUpdate(units::second_t dt) override;

      protected:

      private:
          wom::ModuleName _name;

    };

};
