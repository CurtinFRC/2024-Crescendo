#include "intakebehaviour.h"

IntakeManualControl::IntakeManualControl(Intake *intake, frc::XboxController &codriver) : _intake(intake), _codriver(codriver) {
    Controls(intake);
};

void IntakeManualControl::OnTick(units::second_t dt) {
    if (_codriver.GetRightBumper()) {
        _intake->setRaw(5_V);
    } else if (_codriver.GetLeftBumper()){
        _intake->setRaw(-5_V);
    } else {
        _intake->setRaw(0_V);
    }
}