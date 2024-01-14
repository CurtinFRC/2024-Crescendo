#include "Mag.h"

Mag::Mag(MagConfig config) : _config(config) {}

void Mag::OnUpdate(units::second_t dt) {
//Work on later. 
  switch (_state) {
    case MagState::kIdle: 
    {
      //set to 0 volts
      if (_config.intakeSensor->Get()) {
        setState(MagState::kHold);
      }
    }
    break;

    case MagState::kHold:
    {
      if (_config.magSensor->Get() == 0) {
        setState(MagState::kIdle);
      }
    }
    break;

    case MagState::kEject:
    {
      if (_config.magSensor->Get() == 0) {
        setState(MagState::kIdle);
      }
    }
    break;

    case MagState::kRaw:
    break;

    case MagState::kPass:
    {
      if (_config.shooterSensor->Get()) {
        setState(MagState::kIdle);
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

// void Mag::setPass(units::volt_t voltage) {
//   _voltage = voltage;
// }
// void Mag::setHold(units::volt_t voltage) {
//   _voltage = voltage;
// }
// void Mag::setEject(units::volt_t voltage) {
//   _voltage = voltage;
// }




