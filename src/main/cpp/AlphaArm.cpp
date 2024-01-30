// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArm.h"
//fiddle with these values
AlphaArm::AlphaArm(AlphaArmConfig config) : _config(config), _pidWom(_config.path + "/pid", config.pidConfigA) /*_table(nt::NetworkTableInstance::GetDefault().GetTable(config.path)*/{}

void AlphaArm::OnUpdate(units::second_t dt){
  _table->GetEntry("Error").SetDouble(_pidWom.GetError().value());
  _table->GetEntry("Current Pos").SetDouble(-_config.alphaArmGearbox.encoder->GetEncoderPosition().value());
  _table->GetEntry("Setpoint").SetDouble(_pidWom.GetSetpoint().value());
        switch(_state){
          case AlphaArmState::kIdle:
            
            _setAlphaArmVoltage = 0_V;

          break;
          case AlphaArmState::kRaw:
          
            _setAlphaArmVoltage = _rawArmVoltage;

          break;
          case AlphaArmState::kAmpAngle:
            std::cout << "AmpAngle" << std::endl;
            // _pidWom.SetSetpoint(_goal);
            // _setAlphaArmVoltage = _pidWom.Calculate(_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0_V);


            if (_controlledRawVoltage.value() == 0) {
              if (_encoderSetpoint.value() != 0) {
                _pidWom.SetSetpoint(_encoderSetpoint);
                _setAlphaArmVoltage = -_pidWom.Calculate(-_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0_V);
                _table->GetEntry("Demand").SetDouble(_setAlphaArmVoltage.value());
              }
            } else {
              _pidWom.Reset();
              _encoderSetpoint = -_config.alphaArmGearbox.encoder->GetEncoderPosition();
              _setAlphaArmVoltage = _controlledRawVoltage;
            }
            break;
          case AlphaArmState::kSpeakerAngle:
            std::cout << "SpeakerAngle" << std::endl;
            //_pidWom.SetSetpoint(_goal);
           // _setAlphaArmVoltage = _pidWom.Calculate(_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0_V);
              break;
          case AlphaArmState::kStowed:
            std::cout << "Stowed" << std::endl;
            //_pidWom.SetSetpoint(_goal);
            //_setAlphaArmVoltage = _pidWom.Calculate(_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0_V);
          break;
          default:
            std::cout << "oops, wrong state" << std::endl;
          break;
        }
        std::cout << " ARM POSITION: " << _config.alphaArmGearbox.encoder->GetEncoderPosition().value() << std::endl;
        //std::cout << "OUTPUT VOLTAGE: " << _setAlphaArmVoltage.value() << std::endl;
        _config.alphaArmGearbox.motorController->SetVoltage(_setAlphaArmVoltage);
}

void AlphaArm::SetState(AlphaArmState state) {
  _state = state;
}

void AlphaArm::SetGoal(units::radian_t goal){
  _goal = goal;
}

void AlphaArm::SetArmRaw(units::volt_t voltage){
  std::cout << "VOLTAGE: " << voltage.value() << std::endl;
  _rawArmVoltage = voltage;
}

void AlphaArm::setControllerRaw(units::volt_t voltage) {
  _controlledRawVoltage = voltage;
}