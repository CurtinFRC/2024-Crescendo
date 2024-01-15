#include "Mag.h"

Mag::Mag(MagConfig config) : _config(config) {}

void Mag::OnUpdate(units::second_t dt) {
  switch (_state) {
    case MagState::kIdle: 
    {
      if (_config.intakeSensor->Get()) {
        setState(MagState::kHold);
      }
      _config.magGearbox.transmission->SetVoltage(0_V);
    }
    break;

    case MagState::kHold:
    {
      if (_config.magSensor->Get() == 0) {
        setState(MagState::kIdle);
      }
      _config.magGearbox.transmission->SetVoltage(0_V);
    }
    break;

    case MagState::kEject:
    {
      if (_config.magSensor->Get() == 0 && _config.intakeSensor->Get() ==  0) {
        setState(MagState::kIdle);
      }
      _config.magGearbox.transmission->SetVoltage(-5_V);
    }
    break;

    case MagState::kRaw:
      _config.magGearbox.transmission->SetVoltage(_voltage);
    break;

    case MagState::kPass:
    {
      if (_config.shooterSensor->Get()) {
        setState(MagState::kIdle);
      } else {
        _config.magGearbox.transmission->SetVoltage(5_V);
      }
    }
    break;

    default:
      std::cout << "Error magazine in invalid state" << std::endl;
    break;
  }
}


void Mag::setState(MagState state) {
  _state = state;
}

void Mag::setRaw(units::volt_t voltage) {
  _voltage = voltage;
}
MagState Mag::getState() {
  return _state;
}




