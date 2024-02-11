// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#pragma once
#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <units/angular_velocity.h>
#include "Wombat.h"
#include "utils/PID.h"

struct AlphaArmConfig {
    wom::Gearbox alphaArmGearbox;
    //wom::Gearbox wristGearbox;
    //wom::DutyCycleEncoder* armEncoder;
    //wom::CANSparkMaxEncoder* armEncoder;
    wom::utils::PIDConfig<units::radian, units::volt> pidConfigA;
    //wom::utils::PIDConfig<units::radians_per_second, units::volt> velocityConfig;
    std::string path;
    //void WriteNT(std::shared_ptr<nt::NetworkTable> table);

};

enum class AlphaArmState {
    kIdle,
    kIntakeAngle,
    kAmpAngle,
    kSpeakerAngle,
    kStowed,
    kRaw
    //kForwardWrist,
    //kReverseWrist,
};

class AlphaArm : public::behaviour::HasBehaviour{
    public:
     explicit AlphaArm(AlphaArmConfig config);

    void OnStart();
    void OnUpdate(units::second_t dt);
    void SetArmRaw(units::volt_t voltage);
    void SetState(AlphaArmState state);
    void setControllerRaw(units::volt_t);
    //void setGoal(units::radians_per_second_t);
    void SetGoal(units::radian_t);
    double GetArmEncoder();
    AlphaArmConfig GetConfig();
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("AlphaArm");
    //units::radians_per_second_t goal;
    //double goal;
    
    //frc::DutyCycleEncoder armEncoder{12};
    //void SetRaw(units::volt_t voltage);

    private:
    //frc::PIDController _pidFRC;
    wom::utils::PIDController<units::radian, units::volt> _pidWom;
   // wom::utils::PIDController<units::radians_per_second, units::volt> _velocityPID;

    std::shared_ptr<nt::NetworkTable> _table = nt::NetworkTableInstance::GetDefault().GetTable("AlphaArm");
    AlphaArmConfig _config;
    AlphaArmState _state = AlphaArmState::kIdle;
    units::radian_t _goal; 
    units::volt_t _setAlphaArmVoltage = 0_V;
    units::volt_t _setWristVoltage = 0_V;

    units::volt_t _rawArmVoltage = 0_V;
    units::volt_t _rawWristVoltage = 0_V;
    //units::radiant_t maxAngle = 1_radian_t;

    units::radian_t _encoderSetpoint = 0_rad;
    units::volt_t _controlledRawVoltage = 0_V;
  units::radian_t _startingPos = 0_rad;

    std::string _stateName = "Default";
    bool started = false;
};
