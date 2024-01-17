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
  }; //else {
    
  // }
}

IntakeAutoControl::IntakeAutoControl(Intake *intake) : _intake(intake) {};

void IntakeAutoControl::OnTick(units::second_t dt) {
  if (_intake->getConfig().intakeSensor->Get() == 1) {
    _intake->setState(IntakeState::kPass);
  } else if (_intake->getConfig().magSensor->Get() == 0) {
    _intake->setState(IntakeState::kIdle);
  };
}