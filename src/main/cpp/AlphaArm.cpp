// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "AlphaArm.h"


AlphaArm::AlphaArm(AlphaArmConfig *config/*, frc::Rotation2d initialAngle, wom::vision::Limelight* vision */) : _config(config), _pidArm{frc::PIDController(1.2, 0.4, 0)}, _pidArmStates{frc::PIDController(37, 0.00070, 0.15)}, _pidIntakeState{frc::PIDController(30, 0.00015, 0.005)} 
{
 
}

void AlphaArm::OnStart(){
  _pidArmStates.Reset();
  _pidIntakeState.Reset();

}

void AlphaArm::OnUpdate(units::second_t dt) {
  switch (_state) {
    case AlphaArmState::kIdle:
    
      _setAlphaArmVoltage = 0_V;
      break;
    case AlphaArmState::kRaw:
    std::cout << "RawControl" << std::endl;
       _setAlphaArmVoltage = _rawArmVoltage;
      
      break;

    case AlphaArmState::kHoldAngle:
    {
      _pidArm.SetSetpoint(-_goal);
       //_setAlphaArmVoltage = _pidArm.Calculate(_alphaArm->GetConfig().config->alphaArmEncoder.GetEncoderPosition());
      _setAlphaArmVoltage = units::volt_t{_pidArmStates.Calculate(-_config->alphaArmEncoder.GetEncoderPosition().value())};
    }
    break;
    case AlphaArmState::kAmpAngle:
      std::cout << "Amp Angle" << std::endl;
      
       _pidArmStates.SetSetpoint(-2.17);
       _setAlphaArmVoltage = units::volt_t{_pidArmStates.Calculate(-_config->alphaArmEncoder.GetEncoderPosition().value())};

    break;

    case AlphaArmState::kIntakeAngle:
    std::cout << "Intake Angle" << std::endl;
       _pidIntakeState.SetSetpoint(-0.48); //-0.48
       _setAlphaArmVoltage = units::volt_t{_pidIntakeState.Calculate(-_config->alphaArmEncoder.GetEncoderPosition().value())};
    break;

    case AlphaArmState::kSpeakerAngle:
    std::cout << "Speaker Angle" << std::endl;
       _pidArmStates.SetSetpoint(-0.82);
       _setAlphaArmVoltage = units::volt_t{_pidArmStates.Calculate(-_config->alphaArmEncoder.GetEncoderPosition().value())};

    break;

    // case AlphaArmState::kVisionAngle:
    // std::cout << "Vision Angle" << std::endl;

    // break;

    case AlphaArmState::kStowed:
    std::cout << "Stowed" << std::endl;
       _pidArmStates.SetSetpoint(-0.52);
       _setAlphaArmVoltage = units::volt_t{_pidArmStates.Calculate(-_config->alphaArmEncoder.GetEncoderPosition().value())};
      default:
      std::cout << "Error: alphaArm in INVALID STATE" << std::endl;
      break;

  }
   _config->alphaArmGearbox.motorController->SetVoltage(_setAlphaArmVoltage);
   _config->alphaArmGearbox2.motorController->SetVoltage(_setAlphaArmVoltage);
    std::cout << "Encoder Value: " << _config->alphaArmEncoder.GetEncoderPosition().value() << std::endl;
    _table->GetEntry("PID Error").SetDouble(_pidArm.GetPositionError());
    _table->GetEntry("SetPoint").SetDouble(_pidArm.GetSetpoint());
    _table->GetEntry("Input").SetDouble(_config->alphaArmEncoder.GetEncoderPosition().value());

    _table->GetEntry("PID Error State").SetDouble(_pidArmStates.GetPositionError());
    _table->GetEntry("SetPoint State").SetDouble(_pidArmStates.GetSetpoint());

    _table->GetEntry("Intake SetPoint State").SetDouble(_pidIntakeState.GetSetpoint());
    _table->GetEntry("Intake PID Error State").SetDouble(_pidIntakeState.GetPositionError());



     std::cout << "Voltage:" << _setAlphaArmVoltage.value() << std::endl;

}

void AlphaArm::SetState(AlphaArmState state) {
  _state = state;
}

void AlphaArm::SetArmRaw(units::volt_t voltage) {
  _rawArmVoltage = voltage;
}


void AlphaArm::SetGoal(double goal){
  _goal = goal;
}

void AlphaArm::SetControllerRaw(units::volt_t voltage){
  _controlledRawVoltage = voltage;
}


