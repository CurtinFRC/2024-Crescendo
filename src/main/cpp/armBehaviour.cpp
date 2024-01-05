#include "armBehaviour.h"

ArmManualControl::ArmManualControl(Arm *arm, frc::XboxController &codriver) : _arm(arm), _codriver(codriver) {
  Controls(arm);
}

void ArmManualControl::OnTick(units::second_t dt) {

  if(_codriver.GetRightBumper()) {
    _arm->setRaw(5_V);
  } else if (_codriver.GetLeftBumper()) {
    _arm->setRaw(-5_V);
  } else {
    _arm->setRaw(0_V);
  }

}