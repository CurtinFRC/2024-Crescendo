#pragma once

#pragma once
#include "Wombat.h"
#include "AlphaArm.h"
#include <frc/XboxController.h>

class AlphaArmManualControl : public behaviour::Behaviour {
    public:
        explicit AlphaArmManualControl(AlphaArm* alphaArm, frc::XboxController* codriver);
        void OnTick(units::second_t dt);

    private:
    AlphaArm *_alphaArm;
    frc::XboxController* _codriver;
    //units::volt_t _rightStick = ((_codriver->GetRightY()>0.05 || _codriver->GetRightY() < -0.05 )?_codriver->GetRightY():0) * 2_V;
    bool _rawControl;
};