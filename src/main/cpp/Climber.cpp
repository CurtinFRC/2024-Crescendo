#include "Climber.h"

Climber::Climber(ClimberConfig config) : _config(config) {}

void Climber::OnUpdate(units::second_t dt) {
  switch (_state) {
    case ClimberState::kIdle: 
    break;
    case ClimberState::kClimb:
    break;
    case ClimberState::kHang:
    break;
    case ClimberState::kRaw:
    break;
    default:
       std::cout << "Error magazine in invalid state" << std::endl;
    break;
  }
}

void Climber::SetState(ClimberState state) {
  _state = state;
}

void Climber::SetRaw(units::volt_t voltage) {
  _voltage = voltage;
}
