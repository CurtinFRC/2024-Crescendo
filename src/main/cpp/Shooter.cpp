#include "Shooter.h"

    
Shooter::Shooter(ShooterConfig config) : _config(config), _pid(config.path + "/pid", config.pidConfig) {
  
}


void Shooter::OnUpdate(units::second_t dt){
    switch(_state){
        case ShooterState::kIdle:
				{
          _config.ShooterGearbox.transmission->SetVoltage(0_V);
          if (_config.shooterSensor) {       
            _state = ShooterState::kReverse;
            // kSpinup
            // kShooting
          }
				}
        break;
        case ShooterState::kSpinUp:
        {
          _pid.SetSetpoint(_goal);
          units::volt_t volts = _pid.Calculate(_goal, dt, 0.2_V);
        
          _voltage = volts;
          _config.ShooterGearbox.transmission->SetVoltage(_voltage);

          if (_pid.IsStable()) {
            setState(ShooterState::kShooting);
          }
  
        }
        break;
        case ShooterState::kShooting:
				{
          _pid.SetSetpoint(_goal);
          units::volt_t volts = _pid.Calculate(_goal, dt, 0.2_V);
          _voltage = volts;
        	_config.ShooterGearbox.transmission->SetVoltage(_voltage);
          // if (_pid.IsStable()) {
            // setState(ShooterState::kIdle);
          // }
          // do we need this first loop?
          if (_shooterSensor.Get()) {
            setState(ShooterState::kIdle);
          }
				}
        break;
        case ShooterState::kReverse:
				{
        	_config.ShooterGearbox.transmission->SetVoltage(-5_V);
          if (_shooterSensor.Get()) {
            setState(ShooterState::kIdle);
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

void Shooter::setState(ShooterState state) {
    _state = state;
}
void Shooter::setRaw(units::volt_t voltage) {
    _voltage = voltage;
}
void Shooter::setPidGoal(units::radians_per_second_t goal) {
  _goal = goal;
} 

