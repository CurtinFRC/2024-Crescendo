// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once
#include <frc/DigitalInput.h>

#include "Wombat.h"
//#include "vision/Vision.h"
#include "utils/PID.h"
#include <units/angle.h>
#include <units/voltage.h>
#include <frc/controller/PIDController.h>

struct AlphaArmConfig {
  wom::Gearbox alphaArmGearbox; //up+down
  wom::Gearbox alphaArmGearbox2;
  wom::DutyCycleEncoder alphaArmEncoder;
  //frc::PIDController pidArmConfig;
  //wom::PIDConfig<units::degree, units::volts> alphaArmPID;
  std::string path;
  // Vision *vision;
};

enum class AlphaArmState {
  kIdle,
  kIntakeAngle,
  kAmpAngle,
  kSpeakerAngle,
  kHoldAngle,
  kStowed,
  kRaw
};

class AlphaArm : public behaviour::HasBehaviour {
 public:
  AlphaArm(AlphaArmConfig *config);

  void OnUpdate(units::second_t dt);
  void SetArmRaw(units::volt_t voltage);
  void SetState(AlphaArmState state);
  void SetControllerRaw(units::volt_t voltage); 
  void SetGoal(double goal);
  AlphaArmConfig GetConfig(); //{ return _config; }
  frc::PIDController GetPID();

 private:
  // units::radian_t CalcTargetAngle();

  AlphaArmConfig *_config;
  AlphaArmState _state = AlphaArmState::kIdle;
  //wom::utils::PIDController<units::degree, units::volt> _alphaArmPID;
  //frc::DutyCycleEncoder armEncoder{4};
  frc::PIDController _pidArm;
  frc::PIDController _pidArmStates;
  std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("AlphaArm");
  units::volt_t _setAlphaArmVoltage = 0_V;

  units::volt_t _controlledRawVoltage = 0_V;
  units::volt_t _rawArmVoltage = 0_V;
  units::volt_t _testRawVoltage = 3_V;
  double _goal = 0;

};