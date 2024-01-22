#pragma once
#include "Wombat.h"
#include <frc/DigitalInput.h>

struct AlphaArmConfig {
    //wom::Gearbox armGearBox;
    wom::Gearbox alphaArmGearbox;
    wom::Gearbox wristGearbox;

};

enum class AlphaArmState {
    kIdle,
    kIntakeAngle,
    kAmpAngle,
    kSpeakerAngle,
    kRaw
};

class AlphaArm : public::behaviour::HasBehaviour{
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