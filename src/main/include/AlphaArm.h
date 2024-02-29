// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once
#include <frc/DigitalInput.h>
#include "Wombat.h"

struct AlphaArmConfig {
  wom::Gearbox alphaArmGearbox;
  wom::Gearbox wristGearbox;
};

enum class AlphaArmState {
  kIdle,
  kIntakeAngle,
  kAmpAngle,
  kSpeakerAngle,
  kForwardWrist,
  kReverseWrist,
  kRaw
};

class AlphaArm : public ::behaviour::HasBehaviour {
 public:
  explicit AlphaArm(AlphaArmConfig config);

  void OnUpdate(units::second_t dt);
  void SetArmRaw(units::volt_t voltage);
  void setWristRaw(units::volt_t voltage);
  void SetState(AlphaArmState state);
  AlphaArmConfig GetConfig();
  // void SetRaw(units::volt_t voltage);

 private:
  AlphaArmConfig _config;
  AlphaArmState _state = AlphaArmState::kIdle;
  units::volt_t _setAlphaArmVoltage = 0_V;
  units::volt_t _setWristVoltage = 0_V;

  units::volt_t _rawArmVoltage = 0_V;
  units::volt_t _rawWristVoltage = 0_V;
};
