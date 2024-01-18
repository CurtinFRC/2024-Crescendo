#include "Climber.h"

Climber::Climber(ClimberConfig config) : _config(config) {}

void Climber::OnUpdate(units::second_t dt) {
  switch (_state) {
    case ClimberState::kIdle:
    {
      _stringStateName = "Idle";
      _setVoltage = 0_V;
    }
    break;

    case ClimberState::kClimb:
    {
      _stringStateName = "Climb";
      _setVoltage = 8_V;
    }
    break;

    case ClimberState::kHang:
    {
      _stringStateName = "Hang";
      _setVoltage = -8_V;
    }
    break;

    case ClimberState::kRaw:
    {
      _stringStateName = "Raw";
      _setVoltage = _rawVoltage;
    }
    break;
    default:
       std::cout << "Error magazine in invalid state" << std::endl;
    break;
  }
  _config.climberGearbox.motorController->SetVoltage(_setVoltage);

  _table->GetEntry("State: ").SetString(_stringStateName);
  _table->GetEntry("Motor Voltage: ").SetDouble(_setVoltage.value());
  
}

void Climber::SetState(ClimberState state) {
  _state = state;
}

void Climber::SetRaw(units::volt_t voltage) {
  _rawVoltage = voltage;
}
