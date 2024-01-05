#include "arm.h"


Arm::Arm(ArmConfig config) : _config(config) {}

void Arm::OnUpdate(units::second_t dt){
    switch(_state){
        case ArmState::kIdle:
        break;
        case ArmState::kRaw:
        break;
        case ArmState::kAngle:
        break;
        default:
        std::cout<<"Error arm in invalid state" << std::endl;
        break;
    }
}

void Arm::setState(ArmState state) {
    _state = state;
};
void Arm::setRaw(units::volt_t voltage) {
    _voltage = voltage;
};
void Arm::setAngle(units::degree_t angle) {
    _angle = angle;
};