// Copyright (c) 2023 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "utils/Encoder.h"

using namespace wom;

utils::Encoder::Encoder(double encoderTicksPerRotation, double reduction, int type)
    : _reduction(reduction), _encoderTicksPerRotation(encoderTicksPerRotation), _type(type) {}

double utils::Encoder::GetEncoderTicks() const {
  return GetEncoderRawTicks();
}

double utils::Encoder::GetEncoderTicksPerRotation() const {
  return _encoderTicksPerRotation * _reduction;
}

void utils::Encoder::ZeroEncoder() {
  _offset = GetEncoderRawTicks() * 1_rad;
}

void utils::Encoder::SetEncoderPosition(units::degree_t position) {
  // units::radian_t offset_turns = position - Getutils::EncoderPosition();
  units::degree_t offset = position - (GetEncoderRawTicks() * 360 * 1_deg);
  _offset                = offset;
  // _offset = -offset_turns;
}

void utils::Encoder::SetEncoderOffset(units::radian_t offset) {
  _offset = offset;
  // units::turn_t offset_turns = offset;
  // _offset = offset_turns.value() * Getutils::EncoderTicksPerRotation();
}

void utils::Encoder::SetReduction(double reduction) {
  _reduction = reduction;
}

units::radian_t utils::Encoder::GetEncoderPosition() {
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

double utils::Encoder::GetEncoderDistance() {
  return (GetEncoderTicks() /*- _offset.value()*/) * 0.02032;
  // return (GetEncoderTicks() - _offset.value()) * 2 * 3.141592 * 0.0444754;
  // return (GetEncoderTicks() - _offset.value());
}

units::radians_per_second_t utils::Encoder::GetEncoderAngularVelocity() {
  // return Getutils::EncoderTickVelocity() /
  // (double)GetEncoderTicksPerRotation() * 2 * 3.1415926;
  units::turns_per_second_t n_turns_per_s{GetEncoderTickVelocity() / GetEncoderTicksPerRotation()};
  return n_turns_per_s;
}

double utils::DigitalEncoder::GetEncoderRawTicks() const {
  // utils::Encoder.encoderType = 0;
  return _nativeEncoder.Get();
}

double utils::DigitalEncoder::GetEncoderTickVelocity() const {
  return _nativeEncoder.GetRate();
}

utils::CANSparkMaxEncoder::CANSparkMaxEncoder(rev::CANSparkMax *controller, double reduction)
    : utils::Encoder(42, reduction, 2), _encoder(controller->GetEncoder()) {}

double utils::CANSparkMaxEncoder::GetEncoderRawTicks() const {
  return _encoder.GetPosition() * _reduction;
}

double utils::CANSparkMaxEncoder::GetEncoderTickVelocity() const {
  return _encoder.GetVelocity() * GetEncoderTicksPerRotation() / 60;
}

utils::TalonFXEncoder::TalonFXEncoder(ctre::phoenix::motorcontrol::can::TalonFX *controller, double reduction)
    : utils::Encoder(2048, reduction, 0), _controller(controller) {
  controller->ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::TalonFXFeedbackDevice::IntegratedSensor);
}

double utils::TalonFXEncoder::GetEncoderRawTicks() const {
  return _controller->GetSelectedSensorPosition();
}

double utils::TalonFXEncoder::GetEncoderTickVelocity() const {
  return _controller->GetSelectedSensorVelocity() * 10;
}

utils::TalonSRXEncoder::TalonSRXEncoder(ctre::phoenix::motorcontrol::can::TalonSRX *controller,
                                        double ticksPerRotation, double reduction)
    : utils::Encoder(ticksPerRotation, reduction, 0), _controller(controller) {
  controller->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice::QuadEncoder);
}

double utils::TalonSRXEncoder::GetEncoderRawTicks() const {
  return _controller->GetSelectedSensorPosition();
}

double utils::TalonSRXEncoder::GetEncoderTickVelocity() const {
  return _controller->GetSelectedSensorVelocity() * 10;
}

utils::DutyCycleEncoder::DutyCycleEncoder(int channel, double ticksPerRotation, double reduction)
    : utils::Encoder(ticksPerRotation, reduction, 0), _dutyCycleEncoder(channel) {}

double utils::DutyCycleEncoder::GetEncoderRawTicks() const {
  return _dutyCycleEncoder.Get().value();
}

double utils::DutyCycleEncoder::GetEncoderTickVelocity() const {
  return 0;
}

utils::CanEncoder::CanEncoder(int deviceNumber, double ticksPerRotation, double reduction, std::string name)
    : utils::Encoder(ticksPerRotation, reduction, 1) {
  _canEncoder = new CANCoder(deviceNumber, name);
}

double utils::CanEncoder::GetEncoderRawTicks() const {
  return _canEncoder->GetAbsolutePosition();
}

double utils::CanEncoder::GetEncoderTickVelocity() const {
  return _canEncoder->GetVelocity();
}
