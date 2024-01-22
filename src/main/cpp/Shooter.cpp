#include "Shooter.h"


Shooter::Shooter(ShooterConfig config) : _config(config), _pid(config.path + "/pid", config.pidConfig) {

}


void Shooter::OnUpdate(units::second_t dt){
    switch(_state){
        case ShooterState::kIdle:
				{
          _config.ShooterGearbox.motorController->SetVoltage(0_V);
          if (_shooterSensor.Get()) {       
            _state = ShooterState::kReverse;
          } 
				}
        break;
        case ShooterState::kSpinUp:
        {
          _pid.SetSetpoint(_goal);
          units::volt_t pidCalculate = _pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity(), dt);
          _config.ShooterGearbox.motorController->SetVoltage(pidCalculate);

          if (_pid.IsStable()) {
            SetState(ShooterState::kShooting);
          }

        }
        break;
        case ShooterState::kShooting:
				{
          _pid.SetSetpoint(_goal);
          units::volt_t pidCalculate = _pid.Calculate(_config.ShooterGearbox.encoder->GetEncoderAngularVelocity(), dt);
        	_config.ShooterGearbox.motorController->SetVoltage(pidCalculate);

           if (!_pid.IsStable()) {
             SetState(ShooterState::kSpinUp);
           }
          if (_shooterSensor.Get()) {
            SetState(ShooterState::kIdle);
          }
				}
        break;
        case ShooterState::kReverse:
				{
        	_config.ShooterGearbox.motorController->SetVoltage(-5_V);
          if (!_shooterSensor.Get()) {
            SetState(ShooterState::kIdle);
          }
				}
        break;
        case ShooterState::kRaw:
				{
        	_config.ShooterGearbox.motorController->SetVoltage(5_V);
          if (_shooterSensor.Get()) {
            SetState(ShooterState::kRaw);
          }
				}
        break;
        default:
				{
        	std::cout<<"Error shooter in invalid state" << std::endl;
				}
        break;
        }
}

void Shooter::SetState(ShooterState state) {
    _state = state;
}
void Shooter::SetRaw(units::volt_t voltage) {
    _rawVoltage = voltage;
}
void Shooter::SetPidGoal(units::radians_per_second_t goal) {
  _goal = goal;
} 
