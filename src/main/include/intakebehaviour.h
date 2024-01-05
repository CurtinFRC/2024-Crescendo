#pragma once

#include "Wombat.h"
#include <frc/XboxController.h>
#include "intake.h"
class IntakeManualControl : public behaviour::Behaviour {
    public:
        IntakeManualControl(Intake *intake, frc::XboxController &codriver);
        void OnTick(units::second_t dt) override;
    private:
        Intake *_intake;
        frc::XboxController &_codriver;

};