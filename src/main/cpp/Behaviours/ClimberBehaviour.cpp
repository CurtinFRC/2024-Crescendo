#include "behaviours/ClimberBehaviour.h"

ClimberManualControl::ClimberManualControl(Climber *climber, frc::XboxController *codriver) : _climber(climber), _codriver(codriver) {
  Controls(climber); 
}

void ClimberManualControl::OnTick(units::second_t dt) {

  if (_codriver->GetXButtonPressed()) {
    //start climbing, use arrow later for doing GetButton.
  }
}