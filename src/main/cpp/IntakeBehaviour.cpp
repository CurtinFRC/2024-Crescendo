#include "IntakeBehaviour.h"
#include <frc/XboxController.h>

IntakeManualControl::IntakeManualControl(Intake *intake, frc::XboxController &codriver) : _intake(intake), _codriver(codriver) {
  Controls(intake);
};

void IntakeManualControl::OnTick(units::second_t dt) {
  if (_codriver.GetBButtonPressed()) {
    _intake->setRaw(8_V);
  } else {
    _intake->setRaw(0_V);
  }

  //_intake->setRaw(_button);
}