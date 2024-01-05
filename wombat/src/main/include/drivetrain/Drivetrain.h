// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <units/time.h>
#include <units/voltage.h>

#include <iostream>
#include <string>

#include "behaviour/Behaviour.h"
#include "behaviour/HasBehaviour.h"
#include "utils/Gearbox.h"

namespace wom {
namespace drivetrain {
  struct TankSpeed {
    double right;
    double left;
  };

  // TODO PID
  struct DrivetrainConfig {
    std::string path;

    wom::utils::Gearbox left1;
    wom::utils::Gearbox left2;
    wom::utils::Gearbox left3;
    wom::utils::Gearbox right1;
    wom::utils::Gearbox right2;
    wom::utils::Gearbox right3;
  };

  enum DrivetrainState {
    kIdle,
    kTank,
  };

  class Drivetrain : public behaviour::HasBehaviour {
   public:
    explicit Drivetrain(DrivetrainConfig *config);
    ~Drivetrain();

    DrivetrainConfig *GetConfig();
    DrivetrainState   GetState();

    void SetState(DrivetrainState state);
    void SetSpeed(TankSpeed speed);

    void TankControl(double rightSpeed, double leftSpeed);

    void TankControl();

    void OnStart();
    void OnUpdate(units::second_t dt);

   protected:
   private:
    DrivetrainConfig *_config;
    DrivetrainState   _state;
    units::volt_t     maxVolts = 9_V;
    TankSpeed         _speed;
  };
}  // namespace drivetrain
}  // namespace wom
