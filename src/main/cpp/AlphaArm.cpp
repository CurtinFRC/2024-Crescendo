// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArm.h"
//fiddle with these values
AlphaArm::AlphaArm(AlphaArmConfig config) : _config(config), _pidWom(_config.path + "/pid", config.pidConfigA) /*_table(nt::NetworkTableInstance::GetDefault().GetTable(config.path)*/{}

void AlphaArm::OnStart() {
  started = false;
  // _pidWom.Reset();
  // _pidWom.SetSetpoint(_config.alphaArmGearbox.encoder->GetEncoderPosition());
}

void AlphaArm::OnUpdate(units::second_t dt){
  _table->GetEntry("Error").SetDouble(_pidWom.GetError().value());
  _table->GetEntry("Current Pos").SetDouble(_config.alphaArmGearbox.encoder->GetEncoderPosition().value());
  _table->GetEntry("Setpoint").SetDouble(_pidWom.GetSetpoint().value());
  _table->GetEntry("State ").SetString(_stateName);
        switch(_state){
          case AlphaArmState::kIdle:
            _stateName = "Idle";
            // _pidWom.SetSetpoint(_config.alphaArmGearbox.encoder->GetEncoderPosition());
          
            
            _setAlphaArmVoltage = 0_V;

          break;
          case AlphaArmState::kRaw:
            _stateName = "Raw";
          
            _setAlphaArmVoltage = _rawArmVoltage;

          break;
          case AlphaArmState::kAmpAngle:
          {
            _stateName = "Amp Angle";

            // _pidWom.SetSetpoint(_goal);
            // _setAlphaArmVoltage = _pidWom.Calculate(_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0_V);

            if (started) {
              if (_controlledRawVoltage.value() == 0) {
                if (-_config.alphaArmGearbox.encoder->GetEncoderPosition() > (_startingPos + (3.1415_rad/2))) {
                    // _pidWom.SetSetpoint(_encoderSetpoint);
                    // _setAlphaArmVoltage = -_pidWom.Calculate(-_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0.0_V);
                //   _table->GetEntry("Demand").SetDouble(_setAlphaArmVoltage.value());
                // } else if (_config.alphaArmGearbox.encoder->GetEncoderPosition() < 0_rad) {
                //   _pidWom.SetSetpoint(_encoderSetpoint);
                //   _setAlphaArmVoltage = _pidWom.Calculate(_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0.0_V);
                //   _table->GetEntry("Demand").SetDouble(_setAlphaArmVoltage.value());
                  _setAlphaArmVoltage = 0_V;
                } else {
                  _pidWom.SetSetpoint(_encoderSetpoint);
                  _setAlphaArmVoltage = -_pidWom.Calculate(-_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0.0_V);
                //   _pidWom.Reset();
                //   _encoderSetpoint = _config.alphaArmGearbox.encoder->GetEncoderPosition();
                //   _setAlphaArmVoltage = _controlledRawVoltage;
                }
              } else {
                _pidWom.Reset();
                _encoderSetpoint = -_config.alphaArmGearbox.encoder->GetEncoderPosition();
                _setAlphaArmVoltage = _controlledRawVoltage;
              }
            } else {
              _pidWom.Reset();
              _encoderSetpoint = -_config.alphaArmGearbox.encoder->GetEncoderPosition();
              _setAlphaArmVoltage = _controlledRawVoltage;

              if (std::abs(_controlledRawVoltage.value()) > 0) {
                _startingPos = -_config.alphaArmGearbox.encoder->GetEncoderPosition();
                started = true;
              }
            }
            
          }
            break;
          case AlphaArmState::kSpeakerAngle:
            _stateName = "Speaker Angle";
            //_pidWom.SetSetpoint(_goal);
           // _setAlphaArmVoltage = _pidWom.Calculate(_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0_V);
              break;
          case AlphaArmState::kStowed:
            _stateName = "Stowed";
            //_pidWom.SetSetpoint(_goal);
            //_setAlphaArmVoltage = _pidWom.Calculate(_config.alphaArmGearbox.encoder->GetEncoderPosition(), dt, 0_V);
          break;
          default:
            _stateName = "Error";
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
