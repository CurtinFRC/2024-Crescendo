// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once

#include <frc/XboxController.h>
#include <units/time.h>
#include <units/voltage.h>

#include <string>

#include "behaviour/HasBehaviour.h"
#include "utils/Gearbox.h"

namespace wom {
namespace drivetrain {
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
  kAuto,
};

class Drivetrain : public behaviour::HasBehaviour {
 public:
  Drivetrain(DrivetrainConfig* config, frc::XboxController& driver);
  ~Drivetrain();

  DrivetrainConfig* GetConfig();
  DrivetrainState GetState();

  void SetState(DrivetrainState state);

  void OnStart();
  void OnUpdate(units::second_t dt);

 protected:
 private:
  DrivetrainConfig* _config;
  DrivetrainState _state;
  frc::XboxController& _driver;
  units::volt_t maxVolts = 9_V;
};
}  // namespace drivetrain
}  // namespace wom
