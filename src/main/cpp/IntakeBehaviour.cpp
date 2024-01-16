#include "IntakeBehaviour.h"
#include <frc/XboxController.h>

IntakeManualControl::IntakeManualControl(Intake *intake, frc::XboxController &codriver) : _intake(intake), _codriver(codriver) {
  Controls(intake);
};

void IntakeManualControl::OnTick(units::second_t dt) {

  if (_codriver.GetBButtonPressed()) {
    if(_rawControl == true) {
      _rawControl = false;
    } else {
      _rawControl = true;
    }
  }

  if (_rawControl == true) {
      if (_codriver.GetBButton()) {
        _intake->setRaw(8_V);
        _intake->setState(IntakeState::kRaw);
      } else {
        _intake->setRaw(0_V);
        _intake->setState(IntakeState::kIdle);
      };
  } else {
    // intake auto control
  }
}
