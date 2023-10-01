#include "subsystems/Elevator.h"
#include <networktables/NetworkTableInstance.h>
#include <iostream>

#include <units/acceleration.h>

using namespace wom;

void ElevatorConfig::WriteNT(std::shared_ptr<nt::NetworkTable> table) {
  table->GetEntry("radius").SetDouble(radius.value());
  table->GetEntry("mass").SetDouble(mass.value());
  table->GetEntry("maxHeight").SetDouble(maxHeight.value());
}

Elevator::Elevator(ElevatorConfig config)
  : _config(config), _state(ElevatorState::kIdle),
  _pid{config.path + "/pid", config.pid},
  _velocityPID{config.path + "/velocityPID", config.velocityPID},
  _table(nt::NetworkTableInstance::GetDefault().GetTable(config.path)) {
  // _config.leftGearbox.encoder->SetEncoderPosition(_config.initialHeight / _config.radius * 1_rad);
}

void Elevator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  units::meter_t height = GetElevatorEncoderPos() * 1_m;

  switch(_state) {
    case ElevatorState::kIdle:
      voltage = 0_V;
    break;
    case ElevatorState::kManual:
      voltage = _setpointManual;
    break;
    case ElevatorState::kVelocity:
      {
        units::volt_t feedforward = _config.rightGearbox.motor.Voltage((_config.mass * 9.81_mps_sq) * _config.radius, _velocityPID.GetSetpoint() / (14.0/60.0 * 2.0 * 3.1415 * 0.02225 * 1_m) * 1_rad);
        // units::volt_t feedforward = _config.rightGearbox.motor.Voltage(0_Nm, _velocityPID.GetSetpoint() / (14.0/60.0 * 2.0 * 3.1415 * 0.02225 * 1_m) * 1_rad);
        feedforward = 1.2_V;
        voltage = _velocityPID.Calculate(GetElevatorVelocity(), dt, feedforward);
        if (voltage > 6_V) {
          voltage = 6_V;
        }
        std::cout << "elevator feedforward: " << feedforward.value() << std::endl;
        // voltage = 0_V;
      }
      break;
    case ElevatorState::kPID:
      {
        units::volt_t feedforward = _config.rightGearbox.motor.Voltage((_config.mass * 9.81_mps_sq) * _config.radius, 0_rad_per_s);
        // std::cout << "feed forward" << feedforward.value() << std::endl;
        feedforward = 1.2_V;
        // voltage = _pid.Calculate(height, dt, feedforward);
        voltage = _pid.Calculate(height, dt, feedforward);
        if (voltage > 6_V) {
          voltage = 6_V;
        }
      }
    break;
  }

  // Top Sensor Detector
  // if(_config.topSensor != nullptr) {
  //   if(_config.topSensor->Get()) {
  //     _config.leftGearbox.encoder->SetEncoderPosition(_config.maxHeight / _config.radius * 1_rad);
  //     //voltage = 0_V;
  //   }
  // }

  // //Bottom Sensor Detection
  // if (_config.bottomSensor != nullptr) {
  //   if (_config.bottomSensor->Get()) {
  //     _config.leftGearbox.encoder->SetEncoderPosition(_config.minHeight / _config.radius * 1_rad);
  //     //voltage = 0_V;
  //   }
  // }

  // std::cout << "elevator: " << elevator.height

  // Set voltage to motors...
  voltage *= speedLimit;
  _config.leftGearbox.transmission->SetVoltage(voltage);
  _config.rightGearbox.transmission->SetVoltage(voltage);
}

void Elevator::SetManual(units::volt_t voltage) {
  _state = ElevatorState::kManual;
  _setpointManual = voltage;
}

void Elevator::SetPID(units::meter_t height) {
  _state = ElevatorState::kPID;
  _pid.SetSetpoint(height);
}

void Elevator::SetElevatorSpeedLimit(double limit) {
  speedLimit = limit;
}

void Elevator::SetVelocity(units::meters_per_second_t velocity) {
  _velocityPID.SetSetpoint(velocity);
  _state = ElevatorState::kVelocity;
}

void Elevator::SetIdle() {
  _state = ElevatorState::kIdle;
}

ElevatorConfig &Elevator::GetConfig() {
  return _config;
}

bool Elevator::IsStable() const {
  return _pid.IsStable();
}

ElevatorState Elevator::GetState() const {
  return _state;
}

double Elevator::GetElevatorEncoderPos() {
  return _config.elevatorEncoder.GetPosition() * 14/60 * 2 * 3.1415 * 0.02225;
}

units::meter_t Elevator::GetHeight() const {
  // std::cout << "elevator position"<< _config.rightGearbox.encoder->GetEncoderTicks() << std::endl;
  // return _config.rightGearbox.encoder->GetEncoderDistance() * 1_m;
  return _config.elevatorEncoder.GetPosition() * 14/60 * 2 * 3.1415 * 0.02225 * 1_m;
}

units::meters_per_second_t Elevator::GetElevatorVelocity() const {
  return _config.elevatorEncoder.GetVelocity() / 60_s * 14/60 * 2 * 3.1415 * 0.02225 * 1_m;
}

units::meters_per_second_t Elevator::MaxSpeed() const {
  return _config.leftGearbox.motor.Speed((_config.mass * 9.81_mps_sq) * _config.radius, 12_V) / 1_rad * _config.radius;
}
