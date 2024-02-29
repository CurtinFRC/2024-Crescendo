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
  wom::Gearbox alphaArmGearbox;
  wom::Gearbox alphaArmGearbox2;
  wom::DutyCycleEncoder alphaArmEncoder;

  std::string path;
  // Vision *vision;

};

enum class AlphaArmState {
  kIdle,
  kIntakeAngle,
  kAmpAngle,
  kSpeakerAngle,
  kHoldAngle,
  kVisionAngle,
  kStowed,
  kRaw
};

class AlphaArm : public behaviour::HasBehaviour {
 public:
  AlphaArm(AlphaArmConfig *config/*, frc::Rotation2d initialAngle, wom::vision::Limelight* vision*/);

  void OnUpdate(units::second_t dt);
  void SetArmRaw(units::volt_t voltage);
  void SetState(AlphaArmState state);
  void SetControllerRaw(units::volt_t voltage); 
  void SetGoal(double goal);
  void OnStart();
  AlphaArmConfig GetConfig(); //{ return _config; }
  frc::PIDController GetPID();

 private:
  // units::radian_t CalcTargetAngle();

  AlphaArmConfig *_config;
  wom::vision::Limelight* _vision;
  AlphaArmState _state = AlphaArmState::kIdle;
  //wom::utils::PIDController<units::degree, units::volt> _alphaArmPID;
  //frc::DutyCycleEncoder armEncoder{4};
  frc::PIDController _pidArm;
  frc::PIDController _pidArmStates;
  frc::PIDController _pidIntakeState;
  std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("AlphaArm");
  units::volt_t _setAlphaArmVoltage = 0_V;

  units::volt_t _controlledRawVoltage = 0_V;
  units::volt_t _rawArmVoltage = 0_V;
  units::volt_t _testRawVoltage = 3_V;
  double _goal = 0;

};
