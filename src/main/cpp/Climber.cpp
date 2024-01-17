#include "Climber.h"

Climber::Climber(ClimberConfig config) : _config(config) {}

void Climber::OnUpdate(units::second_t dt) {
  switch (_state) {
    case ClimberState::kIdle:
    {
      _config.climberGearbox.motorController->Set(0);
    }
    break;
    case ClimberState::kRaw:
    {
      _config.climberGearbox.motorController->Set(_rawVoltage.value());
    }
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
  _rawVoltage = voltage;
}
