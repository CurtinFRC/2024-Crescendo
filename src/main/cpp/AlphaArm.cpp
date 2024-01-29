// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArm.h"
//fiddle with these values
AlphaArm::AlphaArm(AlphaArmConfig config) : _config(config), _pidFRC{frc::PIDController (0, 0, 0, 0_s)} {}

 /*_pidWom(_config.path + "/pid", config.pidConfig), _velocityPID(config.path + "/velocityPID", config.velocityConfig), _table(nt::NetworkTableInstance::GetDefault().GetTable(config.path))*/ 

AlphaArmConfig AlphaArm::GetConfig(){
    return _config;
}

void AlphaArm::OnUpdate(units::second_t dt){
_table->GetEntry("Error").SetDouble(_pidFRC.GetPositionError());
_table->GetEntry("Acceleration Error").SetDouble(_pidFRC.GetVelocityError());
_table->GetEntry("Current Pos").SetDouble(_config.alphaArmGearbox.encoder->GetEncoderAngularVelocity().value());


        switch(_state){
            case AlphaArmState::kIdle:
            
            _setAlphaArmVoltage = 0_V;
            _setWristVoltage = 0_V;

            break;
            case AlphaArmState::kRaw:
            _setAlphaArmVoltage = _rawArmVoltage;
            _setWristVoltage = _rawWristVoltage;
            _config.alphaArmGearbox.motorController->SetVoltage(_rawArmVoltage);
            _config.wristGearbox.motorController->SetVoltage(_rawWristVoltage);

            break;
            case AlphaArmState::kAmpAngle:
            std::cout << "AmpAngle" << std::endl;
            //_pidFRC.SetSetpoint(_goal.value());
            //set value 
            //_pidFRC.SetSetpoint()
            _pidFRC.SetSetpoint(_goal.convert<units::rad_per_s>().value());
            units::volt_t{_pidFRC.Calculate(_config.alphaArmGearbox.encoder->GetEncoderAngularVelocity().value(), _goal.value())} = _setAlphaArmVoltage;

            case AlphaArmState::kSpeakerAngle:
            std::cout << "SpeakerAngle" << std::endl;
            _pidFRC.SetSetpoint(_goal.convert<units::rad_per_s>().value());
            units::volt_t{_pidFRC.Calculate(_config.alphaArmGearbox.encoder->GetEncoderAngularVelocity().value(), _goal.value())} = _setAlphaArmVoltage;

            case AlphaArmState::kStowed:
            //not hitting the floor magic
            std::cout << "Stowed" << std::endl;
            _pidFRC.SetSetpoint(_goal.convert<units::rad_per_s>().value());
            units::volt_t{_pidFRC.Calculate(_config.alphaArmGearbox.encoder->GetEncoderAngularVelocity().value(), _goal.value())} = _setAlphaArmVoltage;

            default:
            std::cout << "oops, wrong state" << std::endl;
            break;
            
        }

        _config.alphaArmGearbox.motorController->SetVoltage(_setAlphaArmVoltage);
        //_config.wristGearbox.motorController->SetVoltage(_setWristVoltage);

        
        //_config.wristGearbox.motorController->SetVoltage(_setVoltage);
}

void AlphaArm::SetState(AlphaArmState state) {
  _state = state;
}

void AlphaArm::setGoal(units::radians_per_second_t goal){
  _goal = goal;
}

void AlphaArm::SetArmRaw(units::volt_t voltage){
    _rawArmVoltage = voltage;
}

void AlphaArm::setWristRaw(units::volt_t voltage){
    _rawWristVoltage = voltage;
}
