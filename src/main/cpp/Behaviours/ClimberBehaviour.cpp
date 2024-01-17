#include "behaviours/ClimberBehaviour.h"

ClimberManualControl::ClimberManualControl(Climber *climber, frc::XboxController *codriver) : _climber(climber), _codriver(codriver) {
  Controls(climber); 
}

void ClimberManualControl::OnTick(units::second_t dt) {

  if (_codriver->GetXButtonPressed()) {
    _climber->SetRaw(8_V);
    _climber->SetState(ClimberState::kRaw);
  } else if (_codriver->GetYButtonPressed()){
    _climber->SetRaw(-8_V);
    _climber->SetState(ClimberState::kRaw);
  } else {
    _climber->SetRaw(8_V);
    _climber->SetState(ClimberState::kIdle);
  }
}