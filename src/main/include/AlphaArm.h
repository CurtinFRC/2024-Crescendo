#pragma once
#include "Wombat.h"
#include <frc/DigitalInput.h>

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

class AlphaArm : public::behaviour::HasBehaviour{
    public:
     explicit AlphaArm(AlphaArmConfig config);

    void OnUpdate(units::second_t dt);
    void SetArmRaw(units::volt_t voltage);
    void setWristRaw(units::volt_t voltage);
    void SetState(AlphaArmState state);
    AlphaArmConfig GetConfig();
    //void SetRaw(units::volt_t voltage);

    private:
    AlphaArmConfig _config;
    AlphaArmState _state = AlphaArmState::kIdle;
    units::volt_t _setAlphaArmVoltage = 0_V;
    units::volt_t _setWristVoltage = 0_V;

    units::volt_t _rawArmVoltage = 0_V;
    units::volt_t _rawWristVoltage = 0_V;

    
   
};