#include "mag.h"

Mag::Mag(MagConfig config) : _config(config) {}

void Mag::OnUpdate(units::second_t dt) {
//work on later 
  switch (_state) {
    case MagState::kIdle: 
    {
      // if (_config.intakeSensor.Get()) {
      //   setState(MagState::kHold);
      // }
    }
    break;
    case MagState::kHold:
    break;
    case MagState::kEject:
    break;
    case MagState::kPass:
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




