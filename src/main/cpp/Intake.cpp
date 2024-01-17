#include "Intake.h"

Intake::Intake(IntakeConfig config) : _config(config) {

}

IntakeConfig Intake::getConfig() {
  return _config;
}

void Intake::OnUpdate(units::second_t dt) {

  switch (_state) {
      case IntakeState::kIdle:
      {
        _config.IntakeMotor.transmission->SetVoltage(0_V);
        if (_config.intakeSensor->Get()) {
          setState(IntakeState::kHold);
        }
      }
      break;
      case IntakeState::kRaw:
      {
        _config.IntakeMotor.transmission->SetVoltage(_rawVoltage);
      }
      break;
      case IntakeState::kEject:
      {
        _config.IntakeMotor.transmission->SetVoltage(-5_V);
        if (_config.intakeSensor->Get() == 0 && _config.magSensor->Get() == 0) {
          setState(IntakeState::kIdle);
        }
      }
      break;
      case IntakeState::kHold:
      {
        _config.IntakeMotor.transmission->SetVoltage(0_V);
      }
      break;
      case IntakeState::kIntake:
      {
        _config.IntakeMotor.transmission->SetVoltage(5_V);
      } 
      break;
      case IntakeState::kPass:
      {
        _config.IntakeMotor.transmission->SetVoltage(5_V);
        if (_config.shooterSensor->Get()) {
          setState(IntakeState::kIdle);
      }
      break;
      default:
        std::cout <<"Error: Intake in INVALID STATE." << std::endl;
      break;
    }
    
  }
}

void Intake::setState(IntakeState state) {
  _state = state;
}
void Intake::setRaw(units::volt_t voltage) {
  _rawVoltage = voltage;
}