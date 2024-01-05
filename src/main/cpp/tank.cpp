#include "tank.h"

TankDrive::TankDrive(TankConfig config) : _config(config) {}

void TankDrive::OnUpdate(units::second_t dt) {

    switch (_state) {
        case TankState::kIdle:
        break;
        case TankState::kRaw:
        break;
        default:
          std::cout <<"Error: Tank Drive in INVALID STATE." << std::endl;
        break;
    }
}

void TankDrive::setState(TankState state) {
  _state = state;
}
void TankDrive::setRaw(units::volt_t voltL, units::volt_t voltR) {
//voltL and voltR stands for Voltage Left and Right.
  _voltL = voltL;
  _voltR = voltR;
}