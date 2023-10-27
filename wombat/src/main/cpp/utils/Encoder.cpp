#include "utils/Encoder.h"
#include <iostream>

using namespace wom;

Encoder::Encoder(double encoderTicksPerRotation, double reduction, int type) : _encoderTicksPerRotation(encoderTicksPerRotation), _reduction(reduction), _type(type) {};

double Encoder::GetEncoderTicks() const {
  return GetEncoderRawTicks();
}

double Encoder::GetEncoderTicksPerRotation() const {
  return _encoderTicksPerRotation * _reduction;
}

void Encoder::ZeroEncoder() {
  _offset = GetEncoderRawTicks() * 1_rad;
}

void Encoder::SetEncoderPosition(units::degree_t position) {
  // units::radian_t offset_turns = position - GetEncoderPosition();
  units::degree_t offset = position - (GetEncoderRawTicks() * 360 * 1_deg);
  _offset = offset;
  // _offset = -offset_turns;
}

void Encoder::SetEncoderOffset(units::radian_t offset) {
  _offset = offset;
  // units::turn_t offset_turns = offset;
  // _offset = offset_turns.value() * GetEncoderTicksPerRotation();
}

void Encoder::SetReduction(double reduction) {
  _reduction = reduction;
}

units::radian_t Encoder::GetEncoderPosition() {
  if (_type == 0) {
    units::turn_t n_turns{GetEncoderTicks() / GetEncoderTicksPerRotation()};
    return n_turns;
  } else if (_type == 2) {
    units::degree_t pos = GetEncoderTicks() * 1_deg;
    return pos - _offset;
  } else {
    units::degree_t pos = GetEncoderTicks() * 1_deg;
    return pos - _offset;
  }
}

double Encoder::GetEncoderDistance() {
  return (GetEncoderTicks() /*- _offset.value()*/) * 0.02032;
  // return (GetEncoderTicks() - _offset.value()) * 2 * 3.141592 * 0.0444754;
  // return (GetEncoderTicks() - _offset.value());
}

units::radians_per_second_t Encoder::GetEncoderAngularVelocity() {
  // return GetEncoderTickVelocity() / (double)GetEncoderTicksPerRotation() * 2 * 3.1415926;
  units::turns_per_second_t n_turns_per_s{GetEncoderTickVelocity() / GetEncoderTicksPerRotation()};
  return n_turns_per_s;
}

double DigitalEncoder::GetEncoderRawTicks() const {
  // Encoder.encoderType = 0;
  return _nativeEncoder.Get();
}

double DigitalEncoder::GetEncoderTickVelocity() const {
  return _nativeEncoder.GetRate();
}

CANSparkMaxEncoder::CANSparkMaxEncoder(rev::CANSparkMax *controller, double reduction)
  : Encoder(42, reduction, 2), _encoder(controller->GetEncoder()) {}

double CANSparkMaxEncoder::GetEncoderRawTicks() const {
  return _encoder.GetPosition() * _reduction;
}

double CANSparkMaxEncoder::GetEncoderTickVelocity() const {
  return _encoder.GetVelocity() * GetEncoderTicksPerRotation() / 60;
}

TalonFXEncoder::TalonFXEncoder(ctre::phoenix::motorcontrol::can::TalonFX *controller, double reduction)
  : Encoder(2048, reduction, 0), _controller(controller) {
    controller->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonFXFeedbackDevice::IntegratedSensor);
  }

double TalonFXEncoder::GetEncoderRawTicks() const {
  return _controller->GetSelectedSensorPosition();
}

double TalonFXEncoder::GetEncoderTickVelocity() const {
  return _controller->GetSelectedSensorVelocity() * 10;
}


TalonSRXEncoder::TalonSRXEncoder(ctre::phoenix::motorcontrol::can::TalonSRX *controller, double ticksPerRotation, double reduction) 
  : Encoder(ticksPerRotation, reduction, 0), _controller(controller) {
    controller->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice::QuadEncoder);
  }

double TalonSRXEncoder::GetEncoderRawTicks() const {
  return _controller->GetSelectedSensorPosition();
}

double TalonSRXEncoder::GetEncoderTickVelocity() const {
  return _controller->GetSelectedSensorVelocity() * 10;
}



DutyCycleEncoder::DutyCycleEncoder(int channel, double ticksPerRotation, double reduction) 
  : Encoder(ticksPerRotation, reduction, 0), _dutyCycleEncoder(channel) {}

double DutyCycleEncoder::GetEncoderRawTicks() const {
  return _dutyCycleEncoder.Get().value();
}

double DutyCycleEncoder::GetEncoderTickVelocity() const {
  return 0;
}

CanEncoder::CanEncoder(int deviceNumber, double ticksPerRotation, double reduction)
  : Encoder(ticksPerRotation, reduction, 1) {
    _canEncoder = new CANCoder(deviceNumber, "Drivebase");
  }

double CanEncoder::GetEncoderRawTicks() const {
  return _canEncoder->GetAbsolutePosition();
}

double CanEncoder::GetEncoderTickVelocity() const {
  return _canEncoder->GetVelocity();
}

