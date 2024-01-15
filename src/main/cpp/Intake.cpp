#include "Intake.h"

Intake::Intake(IntakeConfig config) : _config(config) {
}

void Intake::OnUpdate(units::second_t dt) {

  switch (_state) {
      case IntakeState::kIdle:
      {
        _config.IntakeMotor.transmission->SetVoltage(0_V);
      }
      break;
      case IntakeState::kRaw:
      {
        _config.IntakeMotor.transmission->SetVoltage(_voltage);
      }
      break;
      default:
        std::cout <<"Error: Intake in INVALID STATE." << std::endl;
      break;
  }
  
}

void Intake::setState(IntakeState state) {
  _state = state;
}
void Intake::setRaw(units::volt_t voltage) {
  _voltage = voltage;
}