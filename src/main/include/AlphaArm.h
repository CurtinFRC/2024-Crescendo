// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once
#include <frc/DigitalInput.h>

#include "Wombat.h"

struct AlphaArmConfig {
  // wom::Gearbox armGearBox;
  wom::Gearbox alphaArmGearbox;
  wom::Gearbox wristGearbox;
};

enum class AlphaArmState { kIdle, kIntakeAngle, kAmpAngle, kSpeakerAngle, kRaw };

class AlphaArm : public ::behaviour::HasBehaviour {
 public:
  explicit AlphaArm(AlphaArmConfig config);

  void OnUpdate(units::second_t dt);
  void SetRaw(units::volt_t volt);
  void SetState(AlphaArmState state);

 private:
  AlphaArmConfig _config;
  AlphaArmState _state = AlphaArmState::kIdle;
  units::volt_t setAlphaArmVoltage = 0_V;
  units::volt_t setWristVoltage = 0_V;
  units::volt_t _armVoltage;
  units::volt_t _voltR;
};
