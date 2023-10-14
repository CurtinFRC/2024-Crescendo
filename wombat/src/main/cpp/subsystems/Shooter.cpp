#include "subsystems/Shooter.h"

#include <networktables/NetworkTableInstance.h>

using namespace wom;

Shooter::Shooter(std::string path, ShooterParams params) 
  : _params(params), _state(ShooterState::kIdle), 
    _pid{path + "/pid", params.pid}, 
    _table(nt::NetworkTableInstance::GetDefault().GetTable("shooter")) {}

void Shooter::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};
  units::revolutions_per_minute_t currentSpeed = _params.gearbox.encoder->GetEncoderAngularVelocity();

  switch(_state) {
    case ShooterState::kManual:
      voltage = _setpointManual;
      break;
    case ShooterState::kPID:
      {
        auto feedforward = _params.gearbox.motor.Voltage(0_Nm, _pid.GetSetpoint());
        voltage = _pid.Calculate(currentSpeed, dt, feedforward);
      }
      break;
    case ShooterState::kIdle:
      voltage = 0_V;
      break;
  }

  units::newton_meter_t max_torque_at_current_limit = _params.gearbox.motor.Torque(_params.currentLimit);
  units::volt_t max_voltage_for_current_limit = _params.gearbox.motor.Voltage(max_torque_at_current_limit, currentSpeed);

  voltage = 1_V * std::min(voltage.value(), max_voltage_for_current_limit.value());

  _params.gearbox.transmission->SetVoltage(voltage);

  _table->GetEntry("output_volts").SetDouble(voltage.value());
  _table->GetEntry("speed_rpm").SetDouble(currentSpeed.value());
  _table->GetEntry("setpoint_rpm").SetDouble(units::revolutions_per_minute_t{_pid.GetSetpoint()}.value());
  _table->GetEntry("stable").SetBoolean(_pid.IsStable());
}

void Shooter::SetManual(units::volt_t voltage) {
  _state = ShooterState::kManual;
  _setpointManual = voltage;

}

void Shooter::SetPID(units::radians_per_second_t goal) {
  _state = ShooterState::kPID;
  _pid.SetSetpoint(goal);
}

void Shooter::SetIdle() {
  _state = ShooterState::kIdle;
}

bool Shooter::IsStable() const {
  return _pid.IsStable();
}

//Shooter Manual Set 

ShooterConstant::ShooterConstant(Shooter *s, units::volt_t setpoint)
  : _shooter(s), _setpoint(setpoint) {
    Controls(_shooter);
  }
  
void ShooterConstant::OnTick(units::second_t dt) {
  _shooter->SetManual(_setpoint);
}

// ShooterSpinup

ShooterSpinup::ShooterSpinup(Shooter *s, units::radians_per_second_t speed, bool hold)
  : _shooter(s), _speed(speed), _hold(hold) {
  Controls(_shooter);
}

void ShooterSpinup::OnTick(units::second_t dt) {
  _shooter->SetPID(_speed);

  if (!_hold && _shooter->IsStable())
    SetDone();
}
