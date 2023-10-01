#pragma once

#include <string>
#include <iostream>

#include "behaviour/HasBehaviour.h"
#include "utils/Gearbox.h"

#include <units/time.h>
#include <units/voltage.h>

#include <frc/XboxController.h>

namespace wom {
  // TODO PID
  struct DrivetrainConfig {
    std::string path;

    wom::Gearbox left1;
    wom::Gearbox left2;
    wom::Gearbox left3;
    wom::Gearbox right1;
    wom::Gearbox right2;
    wom::Gearbox right3;
  };

  enum DrivetrainState {
    kIdle,
    kTank,
    kAuto,
  };

  class Drivetrain : public behaviour::HasBehaviour {
   public:
    Drivetrain(DrivetrainConfig *config, frc::XboxController &driver);
    ~Drivetrain();

    DrivetrainConfig *GetConfig(); DrivetrainState GetState(); 

    void SetState(DrivetrainState state);

    void OnStart();
    void OnUpdate(units::second_t dt);
   protected:

   private:
    DrivetrainConfig *_config;
    DrivetrainState _state;
    frc::XboxController &_driver;
    units::volt_t maxVolts = 9_V;
  };
}

