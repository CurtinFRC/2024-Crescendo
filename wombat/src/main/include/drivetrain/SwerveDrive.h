#pragma once

#include "behaviour/HasBehaviour.h"

namespace wom {
    enum SwerveModuleName {
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

    class SwerveModule : public behaviour::HasBehaviour {
     public:
      SwerveModule(SwerveModuleName name, SwerveModuleConfig config, SwerveModuleState state);

      SwerveModuleName GetName();
      SwerveModuleConfig GetConfig();
      SwerveModuleState GetState();

      void SetState(SwerveModuleState state);

     protected:

     private:
      SwerveModuleName _name;
      SwerveModuleConfig _config;
      SwerveModuleState _state;

    };
};
