#include "AlphaArm.h"

AlphaArm::AlphaArm(AlphaArmConfig config) : _config(config) {}

void AlphaArm::OnUpdate(units::second_t dt){
        switch(_state){
            case AlphaArmState::kIdle:
            //transmission translate
            _config.alphaArmGearbox.motorController->SetVoltage(0_V);
            _config.wristGearbox.motorController->SetVoltage(0_V);

            break;
            case AlphaArmState::kRaw:
            setAlphaArmVoltage = _armVoltage;
            break;
            default:
            std::cout << "oops, wrong state" << std::endl;
            break;
            
        }
        //transmission translate
       // _config.armGearBox.motorController->SetVoltage(setAlphaArmVoltage);
       _config.alphaArmGearbox.motorController->SetVoltage(setAlphaArmVoltage);
       _config.wristGearbox.motorController->SetVoltage(setWristVoltage);
        
}

void AlphaArm::SetState(AlphaArmState state){
    _state = state;
}

void AlphaArm::SetRaw(units::volt_t voltR){
    _voltR = voltR;
}

